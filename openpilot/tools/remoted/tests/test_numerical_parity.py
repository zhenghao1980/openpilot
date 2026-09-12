#!/usr/bin/env python3
"""Numerical parity (v2 UDP): raw-output equality over the UDP transport.

v2 的融合输出是"小模型 ∪ 大模型"的加权混合, 与纯大模型输出按 *设计* 就不相等
(近段 plan/近处车道线永远取小模型), 所以 v2 的 parity 定义在传输层:

  同一份输入经 UDP 序列化/分片/重组/反序列化后,
  服务端 run_raw 的原始输出向量必须与本机同 pkl 直接 run_raw 的结果逐元素一致。

覆盖 N_FRAMES 连续帧 —— 两侧 features_buffer / prev_desire 时序状态同步推进,
任何一帧的发散都会立刻暴露。融合层逻辑由 test_fusion_model.py 覆盖。

Run with the repo venv:
  python openpilot/tools/remoted/tests/test_numerical_parity.py
  python openpilot/tools/remoted/tests/test_numerical_parity.py --model /tmp/big_driving_tinygrad.pkl
"""
import os
import subprocess
import sys
import time
import unittest

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, REPO_ROOT)

CAM_W, CAM_H = 1928, 1208
N_FRAMES = 5

MODEL_PATH = os.environ.get("PARITY_MODEL", "")


def _parse_args():
  # keep unittest happy: only parse our flag when present
  if "--model" in sys.argv:
    idx = sys.argv.index("--model")
    global MODEL_PATH
    MODEL_PATH = sys.argv[idx + 1]
    del sys.argv[idx:idx + 2]


_parse_args()


class NumericalParityTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    from openpilot.selfdrive.modeld.helpers import modeld_pkl_path
    from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient
    from openpilot.tools.remoted.remote_modeld_server import _Buf, _load_model

    model_path = MODEL_PATH or str(modeld_pkl_path(False))
    cls.port = 20000 + (os.getpid() % 20000)
    # 服务端输出走文件而非 PIPE: 输出量大会塞满 64KB 管道缓冲,
    # 没人读时 print 阻塞 -> 服务端主循环停跳 -> 会话死亡(实测踩过)
    cls._server_log = open(f"/tmp/parity_server_{cls.port}.log", "wb")
    cls.server = subprocess.Popen(
      [sys.executable, os.path.join(REPO_ROOT, "openpilot/tools/remoted/remote_modeld_server_udp.py"),
       "--model", model_path, "--port", str(cls.port)],
      stdout=cls._server_log, stderr=subprocess.STDOUT)

    # readiness = UDP session probe (TCP connect probe 对 UDP 服务无意义)
    cls.client = UdpRemoteClient("127.0.0.1", cls.port, CAM_W, CAM_H)
    cls.client.start()
    t0 = time.monotonic()
    while not cls.client._session_ready.wait(timeout=2):
      if time.monotonic() - t0 > 180:
        cls.server.kill()
        raise RuntimeError("server failed to start: " + cls._read_log_tail(3000))

    # 本机参考实例: 同一个 pkl, 进程内直接跑
    cls.ref = _load_model(CAM_W, CAM_H, model_path)

    # deterministic synthetic sequence
    rng = np.random.default_rng(42)
    from openpilot.selfdrive.modeld.compile_modeld import nv12_copy_size
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    copy_size = nv12_copy_size(*get_nv12_info(CAM_W, CAM_H)[:3])
    cls.frames = [{name: rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes()
                   for name in cls.ref.vision_input_names} for _ in range(N_FRAMES)]
    cls.tfms = [{name: np.eye(3, dtype=np.float32) for name in cls.ref.vision_input_names}
                for _ in range(N_FRAMES)]
    cls.desires = [np.eye(8, dtype=np.float32)[i % 8] for i in range(N_FRAMES)]  # pulse on each frame
    cls.action_ts = [np.array([0.05 * i, 0.03 * i], dtype=np.float32) for i in range(N_FRAMES)]
    cls.traffic = np.array([1, 0], dtype=np.float32)
    cls._Buf = _Buf

  @classmethod
  def tearDownClass(cls):
    try:
      cls.client.close()
    except Exception:
      pass
    cls.server.terminate()
    try:
      cls.server.wait(10)
    except subprocess.TimeoutExpired:
      cls.server.kill()
    try:
      cls._server_log.close()
    except Exception:
      pass

  @classmethod
  def _read_log_tail(cls, n=1500):
    try:
      cls._server_log.flush()
      with open(cls._server_log.name, "rb") as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(max(0, size - n))
        return f.read().decode(errors="replace")
    except Exception:
      return "<unavailable>"

  def _server_tail(self):
    try:
      self.server.kill()
    except Exception:
      pass
    return self._read_log_tail()

  def _inputs(self, i):
    return {
      "desire_pulse": self.desires[i].copy(),
      "traffic_convention": self.traffic.copy(),
      "action_t": self.action_ts[i].copy(),
    }

  def _server_working(self):
    """服务端 BEAT 遥测: 还在排队/推理就算 working (慢 != 丢)。"""
    b = getattr(self.client, "_server_beat", None) or {}
    return bool(b.get("gpu_busy") or b.get("queue_depth"))

  def test_parity_over_sequence(self):
    from openpilot.tools.remoted.remote_modeld_server import run_raw
    from openpilot.selfdrive.modeld.remote_udp import RESP_OK

    for i in range(N_FRAMES):
      bufs = {name: self._Buf(self.frames[i][name]) for name in self.ref.vision_input_names}
      tfms = {k: v.copy() for k, v in self.tfms[i].items()}

      # 传输路径: UDP 序列化 -> 分片 -> 服务端 run_raw -> RESP
      # fire-and-forget: 7.5MB 帧 = ~5300 分片, WSL2 回环实测每帧约 1% 概率丢片 -> 整帧不完成。
      # 丢 INFER 时服务端状态未推进, 可原样重发(参考侧只在确认后推进)。
      # 但"慢"不等于"丢": 服务端 BEAT 带 gpu_busy/queue_depth 遥测, 还在算就继续等,
      # 盲重发只会重复排队、让服务端时序状态多推进一份(偶发 flake 根源)。
      result = None
      seq = self.client.infer(bufs, tfms, self._inputs(i))
      if seq is None:
        self.fail(f"frame {i}: infer() returned None; session_ready={self.client._session_ready.is_set()}\n"
                  f"server tail: {self._server_tail()}")
      retries = 0
      while result is None:
        timeout = 180.0 if (i == 0 and retries == 0) else 30.0  # frame0 含服务端冷启动
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
          r = self.client.poll_result()
          if r is not None and r.get("frame_seq") == seq:
            result = r
            break
          time.sleep(0.01)
        if result is not None or retries >= 3:
          break
        # 重发前等服务端遥测空闲 (慢 != 丢; 盲重发会重复推进服务端时序状态)
        while self._server_working():
          time.sleep(1.0)
        retries += 1
        seq = self.client.infer(bufs, tfms, self._inputs(i))
        if seq is None:
          break
      if result is None:
        self.fail(f"frame {i}: no RESP (retries={retries})\nserver tail: {self._server_tail()}")

      # 参考路径: 确认服务端已处理后才推进参考实例 (run_raw 会就地改 desire_pulse[0], 用独立副本)
      ref_out = run_raw(self.ref, {k: self._Buf(v.data) for k, v in bufs.items()},
                        {k: v.copy() for k, v in tfms.items()}, self._inputs(i))

      self.assertEqual(result["status"], RESP_OK, f"frame {i}: server error: {result.get('error')}")

      a = np.asarray(ref_out, dtype=np.float64)
      b = np.frombuffer(result["output"], dtype=np.float32).astype(np.float64)
      self.assertEqual(a.shape, b.shape, f"frame {i}: output length mismatch")
      np.testing.assert_allclose(a, b, rtol=1e-4, atol=1e-6,
                                 err_msg=f"frame {i} raw output diverged over UDP "
                                         f"(max abs diff {np.max(np.abs(a - b)) if a.size else 0})")


if __name__ == "__main__":
  unittest.main()
