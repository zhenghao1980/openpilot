#!/usr/bin/env python3
"""Watch modelV2 output of a running modeld: frameId, big flag, execution time.

  python openpilot/tools/remoted/watch_modelv2.py [max_seconds]
"""
import os
import sys
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO_ROOT)

from openpilot.cereal import messaging


def main() -> None:
  max_s = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")
  sm = messaging.SubMaster(["modelV2"])
  t0 = time.monotonic()
  last_id = -1
  while time.monotonic() - t0 < max_s:
    sm.update(1000)
    if not sm.updated["modelV2"]:
      continue
    m = sm["modelV2"]
    gap = "" if last_id < 0 else f" (gap {m.frameId - last_id - 1})" if m.frameId != last_id + 1 else ""
    print(f"t={time.monotonic() - t0:7.1f}s  frameId={m.frameId:5d}  big={m.big}  "
          f"exec={m.modelExecutionTime * 1000:6.0f} ms{gap}", flush=True)
    last_id = m.frameId


if __name__ == "__main__":
  main()
