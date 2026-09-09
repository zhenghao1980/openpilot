#!/usr/bin/env python3
"""Numerical parity test: remote big-model chain vs stock ModelState.run.

Runs the real compiled driving model on CPU. Feeds an identical deterministic
frame sequence to both paths and requires parsed outputs to match:

  stock:   ModelState.run(...)                      (in-process)
  remote:  RemoteModelState -> TCP -> run_raw(...)  (same host, real server)

Needs a built openpilot (scons) and the compiled model pickle. Run with the
repo venv:
  python openpilot/tools/remoted/tests/test_numerical_parity.py
"""
import os
import socket
import subprocess
import sys
import time
import unittest

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, REPO_ROOT)

CAM_W, CAM_H = 1928, 1208
N_FRAMES = 5


class _Buf:
  def __init__(self, data: bytes):
    self.data = data


def _wait_port(port: int, timeout: float = 120.) -> None:
  t0 = time.monotonic()
  while time.monotonic() - t0 < timeout:
    try:
      with socket.create_connection(("127.0.0.1", port), timeout=1):
        return
    except OSError:
      time.sleep(0.5)
  raise RuntimeError(f"server did not start on port {port}")


class NumericalParityTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    from openpilot.selfdrive.modeld.helpers import modeld_pkl_path

    cls.port = 20000 + (os.getpid() % 20000)
    cls.server = subprocess.Popen(
      [sys.executable, os.path.join(REPO_ROOT, "openpilot/tools/remoted/remote_modeld_server.py"),
       "--model", str(modeld_pkl_path(False)), "--port", str(cls.port)],
      stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
      _wait_port(cls.port)
    except Exception:
      cls.server.kill()
      out = cls.server.stdout.read().decode(errors="replace")
      raise RuntimeError(f"server failed to start:\n{out[-3000:]}")

    os.environ["REMOTE_MODEL_HOST"] = "127.0.0.1"
    os.environ["REMOTE_MODEL_PORT"] = str(cls.port)
    os.environ["REMOTE_MODEL_TIMEOUT_MS"] = "120000"  # first session loads the model

    from openpilot.selfdrive.modeld.modeld import ModelState
    from openpilot.selfdrive.modeld.remote_model import RemoteModelState, remote_model_metadata

    cls.stock = ModelState(CAM_W, CAM_H, False)
    cls.stock.warmup()

    meta = remote_model_metadata(CAM_W, CAM_H)
    assert meta is not None, "remote server not reachable"
    cls.remote = RemoteModelState(CAM_W, CAM_H, meta)
    cls.remote.warmup()

    # deterministic synthetic sequence
    rng = np.random.default_rng(42)
    from openpilot.selfdrive.modeld.compile_modeld import nv12_copy_size
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    copy_size = nv12_copy_size(*get_nv12_info(CAM_W, CAM_H)[:3])
    cls.frames = [{name: rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes()
                   for name in cls.stock.vision_input_names} for _ in range(N_FRAMES)]
    cls.tfms = [{name: np.eye(3, dtype=np.float32) for name in cls.stock.vision_input_names}
                for _ in range(N_FRAMES)]
    cls.desires = [np.eye(8, dtype=np.float32)[i % 8] for i in range(N_FRAMES)]  # pulse on each frame
    cls.action_ts = [np.array([0.05 * i, 0.03 * i], dtype=np.float32) for i in range(N_FRAMES)]
    cls.traffic = np.array([1, 0], dtype=np.float32)

  @classmethod
  def tearDownClass(cls):
    cls.server.terminate()
    try:
      cls.server.wait(10)
    except subprocess.TimeoutExpired:
      cls.server.kill()
    os.environ.pop("REMOTE_MODEL_HOST", None)
    os.environ.pop("REMOTE_MODEL_PORT", None)
    os.environ.pop("REMOTE_MODEL_TIMEOUT_MS", None)

  def _inputs(self, i):
    return {
      "desire_pulse": self.desires[i].copy(),
      "traffic_convention": self.traffic.copy(),
      "action_t": self.action_ts[i].copy(),
    }

  def test_parity_over_sequence(self):
    self.assertEqual(self.stock.vision_input_names, self.remote.vision_input_names)
    for i in range(N_FRAMES):
      bufs = {name: _Buf(self.frames[i][name]) for name in self.stock.vision_input_names}
      out_stock = self.stock.run(bufs, self.tfms[i], self._inputs(i))
      out_remote = self.remote.run(bufs, self.tfms[i], self._inputs(i))

      self.assertEqual(set(out_stock.keys()), set(out_remote.keys()))
      for k in out_stock:
        a, b = np.asarray(out_stock[k], dtype=np.float64), np.asarray(out_remote[k], dtype=np.float64)
        np.testing.assert_allclose(a, b, rtol=1e-4, atol=1e-6,
                                   err_msg=f"frame {i} output '{k}' diverged "
                                           f"(max abs diff {np.max(np.abs(a - b)) if a.size else 0})")


if __name__ == "__main__":
  unittest.main()
