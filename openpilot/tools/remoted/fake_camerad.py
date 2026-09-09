#!/usr/bin/env python3
"""Fake camerad for PC-side modeld simulation.

Publishes synthetic NV12 frames on VISION_STREAM_NARROW_ROAD (1928x1208) and
VISION_STREAM_WIDE_ROAD (1344x760) using the same VENUS buffer geometry as the
C3X camerad, plus the minimal cereal messages modeld needs to compute sane
warp transforms (deviceState / narrowRoadCameraState / extrinsicsCalibration).

This lets the unmodified (patched) modeld.py run on a PC exactly like it would
on the C3X, including the remote big model probe and the small-model fallback:

  terminal 1: python openpilot/tools/remoted/fake_camerad.py
  terminal 2: REMOTE_MODEL_HOST=<nvidia-host> python openpilot/selfdrive/modeld/modeld.py --demo
"""
import os
import sys
import time

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO_ROOT)

from openpilot.cereal import messaging
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcServer
from openpilot.common.realtime import Ratekeeper
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

# (stream, width, height). On the C3X class of devices (tici/ox03c10 config,
# which DEVICE_CAMERAS also maps "pc" to) BOTH road streams are 1928x1208 —
# modeld copies the same frame_copy_size bytes out of each buffer.
CAMS = [
  (VisionStreamType.VISION_STREAM_NARROW_ROAD, 1928, 1208),
  (VisionStreamType.VISION_STREAM_WIDE_ROAD, 1928, 1208),
]
VIPC_BUFFER_COUNT = 4
RATE = 20


def make_frame(stride: int, y_height: int, size: int, frame_id: int, height: int) -> bytes:
  """NV12 frame with a moving gradient in Y and neutral chroma — cheap, and
  consecutive frames differ so temporal state is exercised."""
  buf = np.zeros(size, dtype=np.uint8)
  x = np.arange(stride, dtype=np.uint16)[None, :]
  y_plane = ((x + frame_id * 3) % 256).astype(np.uint8)          # (1, stride)
  buf[:height * stride] = np.tile(y_plane, (height, 1)).reshape(-1)
  buf[stride * y_height:stride * y_height + (height // 2) * stride] = 128  # neutral UV
  return buf.tobytes()


def main() -> None:
  pm = messaging.PubMaster(["deviceState", "narrowRoadCameraState", "extrinsicsCalibration"])
  vipc = VisionIpcServer("camerad")

  geo = {}
  for stream, w, h in CAMS:
    stride, y_height, uv_height, size = get_nv12_info(w, h)
    vipc.create_buffers_with_sizes(stream, VIPC_BUFFER_COUNT, w, h, size, stride, stride * y_height)
    geo[stream] = (stride, y_height, size)
    print(f"created {stream.name} buffers: {w}x{h} stride={stride} size={size}", flush=True)
  vipc.start_listener()

  rk = Ratekeeper(RATE, print_delay_threshold=None)
  frame_id = 0
  while True:
    ts = time.monotonic_ns()
    for stream, w, h in CAMS:
      stride, y_height, size = geo[stream]
      vipc.send(stream, make_frame(stride, y_height, size, frame_id, h), frame_id, ts, ts)

    for name in ("deviceState", "narrowRoadCameraState", "extrinsicsCalibration"):
      msg = messaging.new_message(name)
      if name == "deviceState":
        msg.deviceState.deviceType = "pc"
      elif name == "narrowRoadCameraState":
        msg.narrowRoadCameraState.frameId = frame_id
        msg.narrowRoadCameraState.sensor = "unknown"  # maps to the AR/OX tici config
      else:
        msg.extrinsicsCalibration.rpyCalib = [0.0, 0.0, 0.0]
      pm.send(name, msg)

    frame_id += 1
    rk.keep_time()


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
