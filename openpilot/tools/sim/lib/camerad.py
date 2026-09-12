import time
import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcServer
from openpilot.cereal import messaging

from openpilot.tools.sim.lib.common import W, H


def _align(v: int, a: int) -> int:
  return (v + a - 1) // a * a


# 与真实 camerad / modeld 的 NV12 布局对齐（见 system/camerad/cameras/nv12_info.py），
# modeld 按 stride=align(W,128) 的视图读取帧，紧凑布局会导致 buffer size 不匹配崩溃
STRIDE = _align(W, 128)
Y_H = _align(H, 32)
UV_H = _align(H // 2, 16)
UV_OFFSET = STRIDE * Y_H
BUF_SIZE = STRIDE * (Y_H + UV_H)


_PAD_BUF = np.zeros(BUF_SIZE, dtype=np.uint8)  # 预分配复用，避免每帧 alloc+memset


def pad_nv12(packed: bytes) -> bytes:
  """Pad packed NV12 (stride=W) into the aligned camerad layout (stride=STRIDE)."""
  y = np.frombuffer(packed, dtype=np.uint8, count=W * H).reshape(H, W)
  uv = np.frombuffer(packed, dtype=np.uint8, count=W * H // 2, offset=W * H).reshape(H // 2, W)
  buf = _PAD_BUF
  y_plane = buf[:UV_OFFSET].reshape(Y_H, STRIDE)
  y_plane[:H, :W] = y
  uv_plane = buf[UV_OFFSET:].reshape(UV_H, STRIDE)
  uv_plane[:H // 2, :W] = uv
  return buf.tobytes()


def rgb_to_nv12(rgb):
  """Convert RGB image to NV12 (YUV420) format using BT.601 coefficients."""
  h, w = rgb.shape[:2]
  r = rgb[:, :, 0].astype(np.int16)
  g = rgb[:, :, 1].astype(np.int16)
  b = rgb[:, :, 2].astype(np.int16)

  # Y plane - BT.601 coefficients (matches original OpenCL kernel)
  # int16 安全: 最大 255*(13+65+33)=28305 < 32767
  y = (((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16
  y = np.clip(y, 0, 255).astype(np.uint8)

  # Subsample RGB for UV (2x2 box filter)
  # UV 系数会溢出 int16 (255*56+0x8080=47176 > 32767)，升 int32（仅 1/4 分辨率，开销小）
  r_sub = ((r[0::2, 0::2].astype(np.int32) + r[0::2, 1::2] + r[1::2, 0::2] + r[1::2, 1::2] + 2) >> 2)
  g_sub = ((g[0::2, 0::2].astype(np.int32) + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2] + 2) >> 2)
  b_sub = ((b[0::2, 0::2].astype(np.int32) + b[0::2, 1::2] + b[1::2, 0::2] + b[1::2, 1::2] + 2) >> 2)

  # U and V planes
  u = np.clip((b_sub * 56 - g_sub * 37 - r_sub * 19 + 0x8080) >> 8, 0, 255).astype(np.uint8)
  v = np.clip((r_sub * 56 - g_sub * 47 - b_sub * 9 + 0x8080) >> 8, 0, 255).astype(np.uint8)

  # Interleave UV for NV12 format
  uv = np.empty((h // 2, w), dtype=np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v

  return np.concatenate([y.ravel(), uv.ravel()]).tobytes()


class Camerad:
  """Simulates the camerad daemon"""
  def __init__(self, dual_camera):
    self.pm = messaging.PubMaster(['narrowRoadCameraState', 'wideRoadCameraState'])

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_NARROW_ROAD, 5, W, H, BUF_SIZE, STRIDE, UV_OFFSET)
    if dual_camera:
      self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H, BUF_SIZE, STRIDE, UV_OFFSET)

    self.vipc_server.start_listener()

  def cam_send_yuv_road(self, yuv):
    self._send_yuv(yuv, self.frame_road_id, 'narrowRoadCameraState', VisionStreamType.VISION_STREAM_NARROW_ROAD)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD)
    self.frame_wide_id += 1

  def rgb_to_yuv(self, rgb):
    """Convert RGB to NV12 YUV format."""
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8
    return rgb_to_nv12(rgb)

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    # 真机 camerad 的 timestampSof/Eof 是真实单调时钟；这里原来用 frame_id*50ms 的虚拟时钟，
    # 与 locationd 的 kf.t（真实时钟）差出 0.8s 回放上界 -> 观测全被拒。改回真实时钟。
    eof = time.monotonic_ns()
    self.vipc_server.send(yuv_type, pad_nv12(yuv), frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)
