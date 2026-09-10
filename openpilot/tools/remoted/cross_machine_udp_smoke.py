#!/usr/bin/env python3
"""Cross-machine UDP smoke: WSL client -> laptop UDP server (v2)."""
import os, sys, time
import numpy as np

REPO = os.path.expanduser("~/openpilot-c3x")
sys.path.insert(0, REPO)

CAM_W, CAM_H = 1928, 1208
HOST = os.environ.get("REMOTE_MODEL_HOST", "192.168.43.203")
PORT = int(os.environ.get("REMOTE_MODEL_PORT", "8571"))

from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient, RESP_OK
from openpilot.selfdrive.modeld.compile_modeld import nv12_copy_size
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

copy_size = nv12_copy_size(*get_nv12_info(CAM_W, CAM_H)[:3])
rng = np.random.default_rng(42)

class _Buf:
  def __init__(self, data):
    self.data = data

def make_bufs():
  return {name: _Buf(rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes())
          for name in ("img", "big_img")}
  return {name: {"data": rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes()}
          for name in ("img", "big_img")}

def make_tfms():
  return {"img": np.eye(3, dtype=np.float32), "big_img": np.eye(3, dtype=np.float32)}

def make_inputs(i):
  return {
    "desire_pulse": np.eye(8, dtype=np.float32)[i % 8],
    "traffic_convention": np.array([1, 0], dtype=np.float32),
    "action_t": np.array([0.05 * i, 0.03 * i], dtype=np.float32),
  }

print(f"connecting {HOST}:{PORT} ...")
c = UdpRemoteClient(HOST, PORT, CAM_W, CAM_H, beat_interval_s=0.5, beat_timeout_s=1.5)
c.start()
if not c._session_ready.wait(timeout=10):
  print("FAIL: handshake timeout")
  sys.exit(1)
print(f"handshake OK in {c.stats()['rtt_ms']:.0f} ms")
print(f"  model_sha256 prefix: {c.meta.get('model_sha256','')[:16]}...")

ok = True
N = int(os.environ.get("SMOKE_FRAMES", "5"))
for i in range(N):
  seq = c.infer(make_bufs(), make_tfms(), make_inputs(i))
  if seq is None:
    print(f"frame {i}: infer dropped (not ready)")
    ok = False
    continue
  # poll result with deadline
  deadline = time.monotonic() + 5.0
  r = None
  while time.monotonic() < deadline:
    r = c.poll_result()
    if r is not None and r["frame_seq"] == seq:
      break
    time.sleep(0.01)
  if r is None:
    print(f"frame {i}: result timeout")
    ok = False
    continue
  if r["status"] != RESP_OK:
    print(f"frame {i}: server error: {r.get('error','')}")
    ok = False
    continue
  finite = np.all(np.isfinite(r["output"]))
  ok &= finite
  print(f"frame {i}: seq={r['frame_seq']} output_len={len(r['output'])} finite={finite} "
        f"recv_ts={r['server_recv_ts']} done_ts={r['server_done_ts']}")

stats = c.stats()
print(f"final stats: rtt_ms={stats['rtt_ms']:.1f} ready={stats['ready']} dropped_rx={stats['dropped_rx']}")
c.close()
print("SMOKE OK" if ok else "SMOKE FAILED")
sys.exit(0 if ok else 1)
