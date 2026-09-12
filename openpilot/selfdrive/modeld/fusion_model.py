"""FusionModelState: adaptive-rate spectrum fusion between on-device small
model (20 Hz, C3X) and remote big model (async UDP, 0-20 Hz).

Implements the v2 architecture from c3x_remote_bigmodel_design.md §3:
  - small model is always on → pose/cameraOdometry never stale
  - big model async fire-and-forget → results arrive with measured staleness
  - fusion happens in parsed semantic space (Parser double-instantiation)
  - plan: time-splice (t<τ small, t>τ big, transition band)
  - lanes/edges: distance-splice (near small, far big)
  - lead / meta / desire: big-priority
  - cold-start ramp w: 0→target over 2 s
  - exposed interface isomorphic to ModelState → modeld.py anchor unchanged
"""
from __future__ import annotations

import os
import time

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient, RESP_OK

T_IDXS = ModelConstants.T_IDXS
X_IDXS = ModelConstants.X_IDXS


def _sigmoid(x: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-x))


class FusionModelState:
  """Drop-in replacement for ModelState / RemoteModelState in modeld.py."""

  # tunables (centralised, see §9.11-13 of design doc)
  RAMP_S: float = 2.0
  TAU0_MS: float = 150.0
  STALE_RESULT_MAX_S: float = 1.0   # 大模型结果最大允许年龄；超过则作废（纯小模型，设计文档断链回落语义）
  INFER_EVERY_N_FRAMES: int = 2     # 每 N 帧发一次大模型请求（设计定位异步 0-20Hz；20Hz 打满会让服务端 worker 线程 GIL 饿死主循环心跳）
  PLAN_TRANSITION_S: float = 0.4
  LANE_DISTANCE_THRESHOLD_M: float = 20.0   # 静态参考值(已被下方动态边界取代)
  LANE_MARGIN_S: float = 0.5      # 动态边界余量(秒): threshold = v_ego × (staleness + margin)
  LANE_DISTANCE_MIN_M: float = 10.0
  LANE_DISTANCE_MAX_M: float = 80.0

  # UI publish thresholds (RemoteModelState / RemoteModelFusionWeight params)
  UI_FRESH_S: float = 0.5    # last big result younger than this = link fresh
  UI_DOWN_S: float = 2.0     # no result for this long = remote down
  UI_WEAK_W: float = 0.5     # weight below this (while link alive) = weak fusion

  def __init__(self, cam_w: int, cam_h: int, meta: dict):
    self.chestnut = True
    self.cam_w, self.cam_h = cam_w, cam_h

    # small model created lazily in warmup() to avoid circular import
    self._small_model = None

    # UDP big-model link
    host = os.environ.get("REMOTE_MODEL_HOST", "")
    if not host:
      try:
        host = (self._get_params().get("RemoteModelHost", encoding="utf-8") if self._get_params() else "") or ""
      except Exception:
        pass
    port = int(os.environ.get("REMOTE_MODEL_PORT", "8571"))
    self.udp_client = UdpRemoteClient(host, port, cam_w, cam_h)

    # parser for big outputs
    self.parser = Parser()
    # meta["output_slices"] may arrive as lists [a,b] (wire JSON) or slice objects (tests)
    def _to_slice(v):
      return v if isinstance(v, slice) else slice(v[0], v[1])
    self.big_output_slices = {k: _to_slice(v) for k, v in meta["output_slices"].items()}
    self.input_shapes = {k: tuple(v) for k, v in meta["input_shapes"].items()}
    self.vision_input_names = [k for k in self.input_shapes if "img" in k]

    # fusion runtime state
    self._frame_n: int = 0
    self._big_send_ts: dict[int, float] = {}
    # warp 前置模式: 服务端 meta 宣告 payload=warped 时, 本地 warp 后只发 384KB 张量
    self._warp_mode = bool((meta or {}).get("payload") == "warped")
    self._warp_client = None
    if self._warp_mode:
      from openpilot.selfdrive.modeld.warp_client import WarpClient
      self._warp_client = WarpClient(cam_w, cam_h)   # frame_seq -> infer发出时刻(monotonic)，用于逐帧真实年龄
    self._session_start_ts: float | None = None
    self._last_big_parsed: dict | None = None
    self._last_big_staleness_ms: float = 0.0
    self._last_big_ok_ts: float | None = None
    self._last_pub_state: int = -1
    self._last_pub_pct: int = -1
    self._params = None          # 惰性加载 (见 _get_params)
    self._params_failed = False

  def warmup(self) -> None:
    from openpilot.selfdrive.modeld.modeld import ModelState
    self._small_model = ModelState(self.cam_w, self.cam_h, False)
    self._small_model.warmup()
    self.udp_client.start()
    if not self.udp_client._session_ready.wait(timeout=10):
      raise RuntimeError("FusionModelState: UDP handshake timeout")
    self._session_start_ts = time.monotonic()

  def _get_params(self):
    """Params 惰性加载: 原生 Windows 等平台无 params native 库, 降级为 None (UI 指示关闭)。"""
    if self._params is None and not self._params_failed:
      try:
        from openpilot.common.params import Params
        self._params = Params()
      except Exception:
        self._params_failed = True
    return self._params

  def run(self, bufs: dict, transforms: dict, inputs: dict, after_enqueue=None) -> dict:
    if self._small_model is None:
      raise RuntimeError("FusionModelState: warmup() not called")

    # 1) small model first — pose/odometry must never wait
    small_outputs = self._small_model.run(bufs, transforms, inputs, after_enqueue=after_enqueue)

    # 2) fire async big model request (non-blocking, throttled)
    self._frame_n += 1
    if self.udp_client.ready and self._frame_n % self.INFER_EVERY_N_FRAMES == 0:
      if self._warp_mode and self._warp_client is not None:
        # warp 前置: 本地把 NV12 warp 成 (2,6,128,256) uint8, 只发 384KB
        w = self._warp_client.warp(bufs["img"].data, bufs["big_img"].data,
                                   transforms["img"], transforms["big_img"])
        seq = self.udp_client.infer({"warped": _BytesBuf(w.tobytes())}, {}, inputs)
      else:
        seq = self.udp_client.infer(bufs, transforms, inputs)
      if seq is not None:
        self._big_send_ts[seq] = time.monotonic()
        # 发送记录只留最近 5s（结果乱序/丢失由这里兜底清理）
        cutoff = time.monotonic() - 5.0
        for s in [s for s, t in self._big_send_ts.items() if t < cutoff]:
          del self._big_send_ts[s]

    # 3) drain result queue, keep only the newest
    newest: dict | None = None
    while True:
      r = self.udp_client.poll_result()
      if r is None:
        break
      newest = r

    if newest is not None and newest["status"] == RESP_OK:
      self._last_big_ok_ts = time.monotonic()
      raw = newest["output"]
      big_parsed = self.parser.parse_outputs(
        {k: raw[np.newaxis, v] for k, v in self.big_output_slices.items()}
      )
      send_ts = self._big_send_ts.pop(newest.get("frame_seq"), None)
      if send_ts is not None:
        # 逐帧真实年龄: INFER发出 -> 结果到手（含排队/网络/推理）
        self._last_big_staleness_ms = max(0.0, (time.monotonic() - send_ts) * 1000.0)
      else:
        stats = self.udp_client.stats()
        self._last_big_staleness_ms = max(0.0, stats.get("rtt_ms", 0.0) / 2.0)
      self._last_big_parsed = big_parsed

    # 3.5) 超时作废: 超过 STALE_RESULT_MAX_S 没有新的大模型结果 -> 丢弃，纯小模型
    if self._last_big_ok_ts is not None and (time.monotonic() - self._last_big_ok_ts) > self.STALE_RESULT_MAX_S:
      self._last_big_parsed = None

    # 4) compute fusion weight w ∈ [0,1] and publish to UI (throttled)
    w = self._compute_weight()
    self._publish_ui_state(w)

    if w > 0.0 and self._last_big_parsed is not None:
      return self._fuse(small_outputs, self._last_big_parsed, w)
    return small_outputs

  def _compute_weight(self) -> float:
    if self._session_start_ts is None:
      return 0.0
    ramp = min(1.0, (time.monotonic() - self._session_start_ts) / self.RAMP_S)
    w_staleness = np.exp(-self._last_big_staleness_ms / self.TAU0_MS)
    return float(w_staleness * ramp)

  def _publish_ui_state(self, w: float) -> None:
    """Write RemoteModelState (INT) + RemoteModelFusionWeight to params,
    only when the value actually changes (params writes are disk-backed)."""
    if self._last_big_ok_ts is None:
      state = 0  # CONNECTING: session up, no big result yet
    else:
      dt = time.monotonic() - self._last_big_ok_ts
      if dt > self.UI_DOWN_S:
        state = 3    # DOWN: remote unreachable, fused output = pure small model
      elif dt <= self.UI_FRESH_S and w >= self.UI_WEAK_W:
        state = 1    # ACTIVE
      else:
        state = 2    # WEAK: link alive but stale / low weight
    if state != self._last_pub_state:
      p = self._get_params()
      if p:
        p.put("RemoteModelState", state)
      self._last_pub_state = state
    pct = int(round(w * 100))
    if pct != self._last_pub_pct:
      p2 = self._get_params()
      if p2:
        p2.put("RemoteModelFusionWeight", str(round(w, 4)))
      self._last_pub_pct = pct

  def _fuse(self, small: dict, big: dict, w: float) -> dict:
    """Semantic-space fusion. `small` and `big` are Parser output dicts."""
    fused = dict(small)

    if "plan" in big and "plan" in small:
      fused["plan"] = self._blend_plan(small["plan"], big["plan"], w)
      if "plan_stds" in big and "plan_stds" in small:
        fused["plan_stds"] = self._blend_plan(small["plan_stds"], big["plan_stds"], w)

    # 当前车速: 小模型 plan 轨迹0 在 t=0 的速度模长(小模型永远新鲜, 无需订阅 carState)
    try:
      v_ego = float(np.linalg.norm(np.asarray(small["plan"])[0, 0, Plan.VELOCITY]))
    except Exception:
      v_ego = 0.0

    if "lane_lines" in big and "lane_lines" in small:
      fused["lane_lines"] = self._blend_by_distance(small["lane_lines"], big["lane_lines"], w, v_ego)
      if "lane_lines_stds" in big and "lane_lines_stds" in small:
        fused["lane_lines_stds"] = self._blend_by_distance(small["lane_lines_stds"], big["lane_lines_stds"], w, v_ego)
    if "road_edges" in big and "road_edges" in small:
      fused["road_edges"] = self._blend_by_distance(small["road_edges"], big["road_edges"], w, v_ego)
      if "road_edges_stds" in big and "road_edges_stds" in small:
        fused["road_edges_stds"] = self._blend_by_distance(small["road_edges_stds"], big["road_edges_stds"], w, v_ego)

    if "lead" in big:
      fused["lead"] = big["lead"]
      for suffix in ("_weights", "_hypotheses", "_stds_hypotheses", "_stds"):
        k = f"lead{suffix}"
        if k in big:
          fused[k] = big[k]
      if "lead_prob" in big:
        fused["lead_prob"] = big["lead_prob"]

    if "meta" in big:
      fused["meta"] = big["meta"]
    for k in ("desire_pred", "desire_state"):
      if k in big:
        fused[k] = big[k]

    return fused

  def _blend_plan(self, small_plan: np.ndarray, big_plan: np.ndarray, w: float) -> np.ndarray:
    result = small_plan.copy()
    tau_s = self._last_big_staleness_ms / 1000.0
    half_trans = self.PLAN_TRANSITION_S / 2.0
    t_arr = np.array(T_IDXS, dtype=np.float32)
    alpha = _sigmoid((t_arr - tau_s) / max(half_trans, 0.05))
    alpha = alpha * w
    alpha = alpha.reshape(1, -1, 1)
    result = (1.0 - alpha) * small_plan + alpha * big_plan
    return result

  def _blend_by_distance(self, small_arr: np.ndarray, big_arr: np.ndarray, w: float, v_ego_mps: float = 0.0) -> np.ndarray:
    # 动态边界: v_ego × (staleness + 余量)，限幅 [MIN, MAX]
    # 直觉: 大模型答案到来之前车已开出 v×τ 米，这段距离必须小模型兜底; 再加余量覆盖抖动
    result = small_arr.copy()
    tau_s = self._last_big_staleness_ms / 1000.0
    threshold_m = float(np.clip(v_ego_mps * (tau_s + self.LANE_MARGIN_S),
                                self.LANE_DISTANCE_MIN_M, self.LANE_DISTANCE_MAX_M))
    idx = next((i for i, x in enumerate(X_IDXS) if x >= threshold_m), len(X_IDXS))
    if idx < len(X_IDXS):
      result[:, :, idx:, :] = (1.0 - w) * small_arr[:, :, idx:, :] + w * big_arr[:, :, idx:, :]
    return result


class _BytesBuf:
  """远程发送用的最小 buf 包装 (encode_infer_inner 只取 .data)。"""
  __slots__ = ("data",)

  def __init__(self, data: bytes):
    self.data = data
