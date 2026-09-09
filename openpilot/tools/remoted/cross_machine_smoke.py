#!/usr/bin/env python3
"""Cross-machine smoke test: C3X-side client talking to a remote_modeld_server
running on the NVIDIA host over the network. Mirrors exactly what the patched
modeld.py does on a C3X (probe -> RemoteModelState -> run), minus the camera.

Run from any machine with the openpilot repo (no GPU needed on this side):

  REMOTE_MODEL_HOST=192.168.43.203 python openpilot/tools/remoted/cross_machine_smoke.py

Expects a server on the NVIDIA host, e.g.:
  python openpilot/tools/remoted/remote_modeld_server.py --model /tmp/big_driving_tinygrad.pkl
"""
import os
import sys
import time

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO_ROOT)

CAM_W, CAM_H = 1928, 1208
N_FRAMES = int(os.environ.get("SMOKE_FRAMES", "3"))


class _Buf:
  def __init__(self, data: bytes):
    self.data = data


def main() -> int:
  host = os.environ.get("REMOTE_MODEL_HOST", "127.0.0.1")
  port = os.environ.get("REMOTE_MODEL_PORT", "8571")

  from openpilot.selfdrive.modeld.remote_model import RemoteModelState, remote_model_metadata

  t0 = time.monotonic()
  meta = remote_model_metadata(CAM_W, CAM_H)
  if meta is None:
    print(f"FAIL: no remote big model answered at {host}:{port} (probe returned None)")
    return 1
  print(f"probe OK in {time.monotonic() - t0:.2f}s from {host}:{port}")
  print(f"  input_shapes:   {meta['input_shapes']}")
  print(f"  vision inputs:  {meta['vision_input_names']}")
  print(f"  output keys:    {sorted(meta['output_slices'])}")

  state = RemoteModelState(CAM_W, CAM_H, meta)
  print(f"session established, vision_input_names={state.vision_input_names}")

  from openpilot.selfdrive.modeld.compile_modeld import nv12_copy_size
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  copy_size = nv12_copy_size(*get_nv12_info(CAM_W, CAM_H)[:3])
  rng = np.random.default_rng(42)

  ok = True
  for i in range(N_FRAMES):
    bufs = {name: _Buf(rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes())
            for name in state.vision_input_names}
    tfms = {name: np.eye(3, dtype=np.float32) for name in state.vision_input_names}
    inputs = {
      "desire_pulse": np.eye(8, dtype=np.float32)[i % 8],
      "traffic_convention": np.array([1, 0], dtype=np.float32),
      "action_t": np.array([0.05 * i, 0.03 * i], dtype=np.float32),
    }
    t = time.monotonic()
    out = state.run(bufs, tfms, inputs)
    dt_ms = (time.monotonic() - t) * 1000
    finite = all(np.all(np.isfinite(np.asarray(v, dtype=np.float64))) for v in out.values())
    ok &= finite
    print(f"frame {i}: {dt_ms:7.1f} ms round-trip, {len(out)} outputs, all_finite={finite}")
    if i == 0:
      plan = np.asarray(out["plan"], dtype=np.float64).flatten()
      print(f"  plan[0..4] = {plan[:5]}")

  print("SMOKE OK" if ok else "SMOKE FAILED (non-finite outputs)")
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
