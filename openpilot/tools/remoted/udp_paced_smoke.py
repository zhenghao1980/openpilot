import os, sys, time
import numpy as np
sys.path.insert(0, os.path.expanduser("~/openpilot-c3x"))
from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient, RESP_OK
from openpilot.selfdrive.modeld.compile_modeld import nv12_copy_size
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

copy_size = nv12_copy_size(*get_nv12_info(1928, 1208)[:3])
rng = np.random.default_rng(42)

def make_bufs():
  class _Buf:
    def __init__(self, d): self.data = d
  return {name: _Buf(rng.integers(0, 255, copy_size, dtype=np.uint8).tobytes())
          for name in ("img", "big_img")}

def make_tfms():
  return {"img": np.eye(3, dtype=np.float32), "big_img": np.eye(3, dtype=np.float32)}

def make_inputs(i):
  return {"desire_pulse": np.eye(8, dtype=np.float32)[i % 8],
          "traffic_convention": np.array([1, 0], dtype=np.float32),
          "action_t": np.array([0.05 * i, 0.03 * i], dtype=np.float32)}

c = UdpRemoteClient(os.environ.get("REMOTE_MODEL_HOST", "192.168.3.69"), 8571, 1928, 1208, beat_interval_s=0.5, beat_timeout_s=1.5)
c.start()
assert c._session_ready.wait(timeout=10)
print(f"handshake rtt={c.stats()['rtt_ms']:.0f}ms")

for i in range(3):
  seq = c.infer(make_bufs(), make_tfms(), make_inputs(i))
  print(f"sent frame {i} seq={seq}")
  deadline = time.monotonic() + 15
  while time.monotonic() < deadline:
    r = c.poll_result()
    if r and r["frame_seq"] == seq:
      ok = r["status"] == RESP_OK and np.all(np.isfinite(r["output"]))
      print(f"  -> ok={ok} len={len(r['output'])} recv={r['server_recv_ts']} done={r['server_done_ts']}")
      break
    time.sleep(0.02)
  else:
    print("  -> TIMEOUT")
  time.sleep(2)  # pacing between frames

print(f"final ready={c.stats()['ready']} dropped={c.stats()['dropped_rx']}")
c.close()
