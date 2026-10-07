"""maprenderd client: camera publisher + frame consumer (cereal mapRender*)."""
from __future__ import annotations  # pyray.Texture2D is a function at runtime, not a type
import ctypes
import os
import struct
import time

import pyray as rl

from openpilot.cereal import messaging

# Fast path: dlopen the C QOI decoder (openpilot/selfdrive/maprender/qoi_dec.c)
# when a prebuilt libqoi_dec.so is present. Pure-Python decoder below is the
# portable fallback (~1s per 1800x1020 frame — too slow on device).
_qoi_c = None
for _cand in (
    os.environ.get("QOI_DEC_LIB", ""),
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "maprender", "libqoi_dec.so"),
):
  if _cand and os.path.isfile(_cand):
    try:
      _lib = ctypes.CDLL(_cand)
      _lib.qoi_decode_rgba.restype = ctypes.c_int
      _lib.qoi_decode_rgba.argtypes = [ctypes.c_char_p, ctypes.c_int,
                                       ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                                       ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte))]
      _qoi_c = _lib
      break
    except OSError:
      pass


def qoi_decode(data: bytes):
  """Minimal QOI decoder -> (w, h, rgba bytes). Raises on bad data."""
  if data[:4] != b"qoif":
    raise ValueError("not qoi")
  w, h = struct.unpack(">II", data[4:12])
  channels, _colorspace = data[12], data[13]
  px = bytearray(w * h * 4)
  index = [(0, 0, 0, 0)] * 64
  r = g = b = 0
  a = 255  # QOI spec: initial pixel is (0,0,0,255)
  p = 14
  run = 0
  out = 0
  npix = w * h
  for _ in range(npix):
    if run > 0:
      run -= 1
    else:
      b1 = data[p]; p += 1
      if b1 == 0xFE:
        r, g, b = data[p], data[p+1], data[p+2]; p += 3
      elif b1 == 0xFF:
        r, g, b, a = data[p], data[p+1], data[p+2], data[p+3]; p += 4
      else:
        tag = b1 & 0xC0
        if tag == 0x00:
          r, g, b, a = index[b1 & 0x3F]
        elif tag == 0x40:
          r = (r + ((b1 >> 4) & 3) - 2) & 0xFF
          g = (g + ((b1 >> 2) & 3) - 2) & 0xFF
          b = (b + (b1 & 3) - 2) & 0xFF
        elif tag == 0x80:
          b2 = data[p]; p += 1
          dg = (b1 & 0x3F) - 32
          r = (r + dg + ((b2 >> 4) & 0xF) - 8) & 0xFF
          g = (g + dg) & 0xFF
          b = (b + dg + (b2 & 0xF) - 8) & 0xFF
        else:
          run = b1 & 0x3F
      index[(r * 3 + g * 5 + b * 7 + a * 11) % 64] = (r, g, b, a)
    px[out] = r; px[out+1] = g; px[out+2] = b; px[out+3] = a
    out += 4
  return w, h, bytes(px)


class MapRenderClient:
  """Sends camera to maprenderd at ~10Hz; converts latest frame to a texture."""

  def __init__(self):
    self._pm = messaging.PubMaster(["mapRenderCam"])
    self._tex: rl.Texture2D | None = None
    self._tex_size = (0, 0)
    self._last_cam_t = 0.0

  def send_cam(self, lat, lon, zoom, bearing, width, height, pitch=0.0):
    now = time.monotonic()
    # 50Hz：中心已是速度外推的连续轨迹，高频率下发让 maprenderd 每个渲染帧
    # 拿到的位置滞后 <20ms；10Hz 阶梯被渲染线程非均匀抽样会产生快慢性交替
    if now - self._last_cam_t < 0.02:
      return
    self._last_cam_t = now
    msg = messaging.new_message("mapRenderCam")
    c = msg.mapRenderCam
    c.lat, c.lon, c.zoom, c.bearing = lat, lon, zoom, bearing
    c.pitch = pitch
    c.width, c.height = width, height
    self._pm.send("mapRenderCam", msg)

  def frame_texture(self, sm) -> rl.Texture2D | None:
    if not sm.updated["mapRenderFrame"] or not sm.valid["mapRenderFrame"]:
      return self._tex
    f = sm["mapRenderFrame"]
    try:
      raw = bytes(f.img)
      if _qoi_c is not None:
        w_i, h_i = ctypes.c_int(0), ctypes.c_int(0)
        buf = ctypes.POINTER(ctypes.c_ubyte)()
        if _qoi_c.qoi_decode_rgba(raw, len(raw), ctypes.byref(w_i), ctypes.byref(h_i),
                                  ctypes.byref(buf)) != 0 or not buf:
          return self._tex
        try:
          rgba = ctypes.string_at(buf, w_i.value * h_i.value * 4)
        finally:
          libc = ctypes.CDLL("libc.so.6")
          libc.free.argtypes = [ctypes.c_void_p]
          libc.free(ctypes.cast(buf, ctypes.c_void_p))
        w, h = w_i.value, h_i.value
      else:
        w, h, rgba = qoi_decode(raw)
    except Exception:
      return self._tex
    # ADAPT: pyray Image struct construction varies; new pyray takes Image(data, w, h, mipmaps, format)
    try:
      img = rl.Image(bytes(rgba), w, h, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
      tex = rl.load_texture_from_image(img)
    except Exception:
      return self._tex
    if self._tex is not None and rl.is_texture_valid(self._tex):
      rl.unload_texture(self._tex)
    self._tex = tex
    self._tex_size = (w, h)
    return tex
