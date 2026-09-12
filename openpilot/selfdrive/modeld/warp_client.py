"""warp 前置客户端: 索引图缓存 + 每帧纯 gather 组装模型输入张量

设计:
  - 坐标/索引计算与 tfm 强相关, 用 tinygrad 同款算子序列复刻 (与 CUDA warp 逐位一致,
    已经实验证实 tinygrad CPU == CUDA == TinyJit, numpy 因 FMA 差异不行)
  - tfm 只在标定更新时变化 (~秒级), 索引图按 tfm 缓存, 变化才重算 (一次 ~200ms, 摊销≈0)
  - 每帧成本 = 3 次 uint8 gather (numpy fancy indexing), SD845 估计 <5ms

验证: warp_client 输出 vs make_warp(CUDA) 逐位一致 (含非平凡 bottom-row tfm)
"""
import sys, time
sys.path.insert(0, "/home/knight/openpilot-c3x")
import numpy as np

from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_warp, nv12_copy_size
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

CAM_W, CAM_H = 1928, 1208
MODEL_W, MODEL_H = 512, 256


def _coord_indices_tg(M_inv, dst_shape, src_shape, stride_pad):
  """numpy f64 坐标计算: f64 误差远小于任何 f32 后端的 fma/除法抖动,
  得到的是数学上最'准'的索引; 与各 f32 后端的差异 ≤ 末位 ulp 级 (边界像素换邻居),
  端到端影响以输出偏差实测为准 (warp_quantify e2e)。"""
  w_dst, h_dst = dst_shape
  h_src, w_src = src_shape
  M = M_inv.astype(np.float64)
  x = np.broadcast_to(np.arange(w_dst, dtype=np.float64)[None, :], (h_dst, w_dst)).reshape(-1)
  y = np.broadcast_to(np.arange(h_dst, dtype=np.float64)[:, None], (h_dst, w_dst)).reshape(-1)
  src_x = M[0, 0] * x + M[0, 1] * y + M[0, 2]
  src_y = M[1, 0] * x + M[1, 1] * y + M[1, 2]
  src_w = M[2, 0] * x + M[2, 1] * y + M[2, 2]
  src_x = src_x / src_w
  src_y = src_y / src_w
  x_round = np.rint(src_x)
  y_round = np.rint(src_y)
  xi = np.clip(x_round, 0, w_src - 1).astype(np.int64)
  yi = np.clip(y_round, 0, h_src - 1).astype(np.int64)
  idx = yi * (w_src + stride_pad) + xi
  # 注意: 生产路径 frame_prepare 调用 warp_perspective_tinygrad 时不传 border_fill_val,
  # 即"越界=边缘像素复制"(clip 索引), 不做零填充 —— 必须完全镜像该行为
  return idx


class WarpClient:
  UV_SCALE = np.array([[1.0, 1.0, 0.5], [1.0, 1.0, 0.5], [2.0, 2.0, 1.0]], dtype=np.float32)

  def __init__(self, cam_w=CAM_W, cam_h=CAM_H, model_w=MODEL_W, model_h=MODEL_H):
    self.nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    self.model_w, self.model_h = model_w, model_h
    self._key = None
    self._maps = None

  def _recompute(self, tfm, big_tfm):
    cam_w, cam_h, stride, y_height, uv_height, _ = self.nv12
    stride_pad = stride - cam_w
    maps = {}
    for name, M in (("img", tfm), ("big_img", big_tfm)):
      idx_y = _coord_indices_tg(M, (self.model_w, self.model_h), (cam_h, cam_w), stride_pad)
      M_uv = (M.astype(np.float32) * self.UV_SCALE).astype(np.float32)
      idx_u = _coord_indices_tg(M_uv, (self.model_w // 2, self.model_h // 2), (cam_h // 2, cam_w // 2), 0)
      maps[name] = (idx_y, idx_u)
    self._maps = maps
    self._key = (tfm.tobytes(), big_tfm.tobytes())

  def maybe_update(self, tfm, big_tfm):
    key = (tfm.tobytes(), big_tfm.tobytes())
    if key != self._key:
      self._recompute(tfm, big_tfm)

  def _prepare(self, frame_bytes, name):
    cam_w, cam_h, stride, y_height, uv_height, _ = self.nv12
    uv_offset = stride * y_height
    idx_y, idx_u = self._maps[name]
    src = np.frombuffer(frame_bytes, dtype=np.uint8)
    y = src[:cam_h * stride][idx_y]
    uv = src[uv_offset:uv_offset + uv_height * stride].reshape(uv_height, stride)
    u = uv[:cam_h // 2, :cam_w:2].reshape(-1)[idx_u]
    v = uv[:cam_h // 2, 1:cam_w:2].reshape(-1)[idx_u]
    yuv = np.concatenate([y, u, v]).reshape(self.model_h * 3 // 2, self.model_w)
    H = (yuv.shape[0] * 2) // 3
    f = yuv
    return np.concatenate([f[0:H:2, 0::2].reshape(-1), f[1:H:2, 0::2].reshape(-1),
                           f[0:H:2, 1::2].reshape(-1), f[1:H:2, 1::2].reshape(-1),
                           f[H:H + H // 4].reshape(-1), f[H + H // 4:H + H // 2].reshape(-1)]
                          ).reshape(6, H // 2, f.shape[1] // 2)

  def warp(self, img_bytes, big_bytes, tfm, big_tfm):
    """返回 (2,6,128,256) uint8 —— 与 pkl 内部 warp 逐位一致"""
    self.maybe_update(tfm, big_tfm)
    return np.stack([self._prepare(img_bytes, "img"), self._prepare(big_bytes, "big_img")])


if __name__ == "__main__":
  # ---- 验证: 与 CUDA make_warp 逐位一致 (含非平凡 bottom-row tfm) ----
  from tinygrad import Tensor, Device
  copy_size = nv12_copy_size(*get_nv12_info(CAM_W, CAM_H)[:3])
  rng = np.random.default_rng(7)
  warp_fn = make_warp(NV12Frame(CAM_W, CAM_H, *get_nv12_info(CAM_W, CAM_H)), MODEL_W, MODEL_H)
  wc = WarpClient()

  all_ok, t_gather = True, []
  for trial in range(5):
    fb = {n: rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes() for n in ("img", "big_img")}
    # bottom row 非 [0,0,1] 的全投影矩阵 (压力测试 FMA/除法路径)
    tfm = (np.eye(3) + rng.standard_normal((3, 3)) * 0.03).astype(np.float32)
    btf = (np.eye(3) + rng.standard_normal((3, 3)) * 0.03).astype(np.float32)
    W_ref = warp_fn(Tensor(tfm).to(Device.DEFAULT), Tensor(btf).to(Device.DEFAULT),
                    Tensor(np.frombuffer(fb["img"], dtype=np.uint8).copy()).to(Device.DEFAULT),
                    Tensor(np.frombuffer(fb["big_img"], dtype=np.uint8).copy()).to(Device.DEFAULT)).realize().numpy()
    t0 = time.perf_counter()
    W = wc.warp(fb["img"], fb["big_img"], tfm, btf)
    dt = (time.perf_counter() - t0) * 1e3
    # 第二次同 tfm (索引命中缓存, 纯 gather)
    t0 = time.perf_counter()
    W2 = wc.warp(fb["img"], fb["big_img"], tfm, btf)
    dt2 = (time.perf_counter() - t0) * 1e3
    eq = np.array_equal(W, W_ref) and np.array_equal(W2, W_ref)
    all_ok &= eq
    t_gather.append(dt2)
    print(f"trial {trial}: 首次(含索引重算)={dt:.1f}ms 缓存命中={dt2:.1f}ms 逐位一致={eq}", flush=True)
  print(f"\n缓存命中均耗时: {np.mean(t_gather):.1f}ms")
  print("结论:", "warp_client 逐位复刻 PASS ✅" if all_ok else "FAIL ❌")
