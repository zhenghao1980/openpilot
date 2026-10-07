"""maprenderd client: camera publisher + frame consumer (cereal mapRender*)."""
from __future__ import annotations  # pyray.Texture2D is a function at runtime, not a type
import ctypes
import os
import struct
import threading
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
      if hasattr(_lib, "qoi_decode_rgba_into"):
        _lib.qoi_decode_rgba_into.restype = ctypes.c_int
        _lib.qoi_decode_rgba_into.argtypes = [ctypes.c_char_p, ctypes.c_int,
                                              ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                                              ctypes.c_void_p]
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
  """Sends camera to maprenderd at ~50Hz; converts latest frame to a texture.

  Decode runs on a worker thread (RT UI core 每帧省 ~10ms）；主线程只做
  GPU 纹理上传。三连缓冲（FREE/READY/UPLOADING）避免读写竞争。"""

  _N_BUF = 3

  def __init__(self):
    self._pm = messaging.PubMaster(["mapRenderCam"])
    self._tex: rl.Texture2D | None = None
    self._tex_size = (0, 0)
    self._last_cam_t = 0.0
    # 帧订阅独立挂载：msgq socket 有重建竞态（后连者会 unlink 重建，先连者
    # 落在被删的旧 socket 上），ui_state.sm 可能因此永远收不到帧；用自己的
    # SubMaster 并在停滞时自动重挂到当前 socket
    self._fsm = None           # SubMaster(["mapRenderFrame"])
    self._fsm_born = 0.0       # _fsm 创建时间
    self._cam_active_t = 0.0   # 上次 send_cam 时间（面板活跃标志）
    self._last_frame_t = 0.0   # 上次收到帧的时间
    # 解码缓冲池：state 0=FREE 1=READY 2=UPLOADING
    self._buf = [None] * self._N_BUF        # bytearray(w*h*4)
    self._meta = [(0, 0)] * self._N_BUF
    self._cam_meta = [None] * self._N_BUF   # 每缓冲对应的实际渲染相机回显
    self._state = [0] * self._N_BUF
    self._cond = threading.Condition()
    self._raw = None                        # 待解码的 QOI 字节（只保留最新）
    self._raw_cam = None                    # 与 _raw 配对的相机回显
    self.cam_shown = None                   # 当前已上传纹理对应的相机回显
    self._worker = threading.Thread(target=self._decode_loop, daemon=True)
    self._worker.start()

  @staticmethod
  def _demote_thread():
    """解码线程降为普通调度并离开 RT 核心（core 5 归 UI）。"""
    try:
      os.sched_setaffinity(0, {4})
    except Exception:
      pass
    try:
      libc = ctypes.CDLL("libc.so.6")

      class _sp(ctypes.Structure):
        _fields_ = [("sched_priority", ctypes.c_int)]
      libc.sched_setscheduler(0, 0, ctypes.byref(_sp(0)))  # SCHED_OTHER
    except Exception:
      pass

  def _decode_loop(self):
    self._demote_thread()
    while True:
      with self._cond:
        while self._raw is None:
          self._cond.wait()
        raw = self._raw
        self._raw = None
        cam = self._raw_cam
        self._raw_cam = None
        idx = -1
        for i in range(self._N_BUF):
          if self._state[i] == 0:
            idx = i
            self._state[i] = 3  # WRITING
            break
      if idx < 0:
        continue  # 缓冲全占（不应发生），丢帧
      try:
        w, h, data = self._decode(raw, idx)
      except Exception:
        with self._cond:
          self._state[idx] = 0
        continue
      with self._cond:
        self._meta[idx] = (w, h)
        self._cam_meta[idx] = cam
        self._state[idx] = 1  # READY

  def _decode(self, raw: bytes, idx: int):
    """解码 QOI 进缓冲 idx，返回 (w, h, 可直接用于 rl.Image 的对象)。"""
    if _qoi_c is not None:
      w_i, h_i = ctypes.c_int(0), ctypes.c_int(0)
      # 先解析尺寸（用 into 变体需要外部缓冲；先走一遍 malloc 版拿尺寸代价大，
      # 直接从 QOI 头读宽高）
      if len(raw) >= 14 and raw[:4] == b"qoif":
        w, h = struct.unpack(">II", raw[4:12])
      else:
        raise ValueError("not qoi")
      need = w * h * 4
      buf = self._buf[idx]
      if buf is None or len(buf) < need:
        buf = bytearray(need)
        self._buf[idx] = buf
      if hasattr(_qoi_c, "qoi_decode_rgba_into"):
        if _qoi_c.qoi_decode_rgba_into(raw, len(raw), ctypes.byref(w_i), ctypes.byref(h_i),
                                       (ctypes.c_char * need).from_buffer(buf)) != 0:
          raise ValueError("decode failed")
      else:
        w_i, h_i = ctypes.c_int(0), ctypes.c_int(0)
        out = ctypes.POINTER(ctypes.c_ubyte)()
        if _qoi_c.qoi_decode_rgba(raw, len(raw), ctypes.byref(w_i), ctypes.byref(h_i),
                                  ctypes.byref(out)) != 0 or not out:
          raise ValueError("decode failed")
        try:
          buf[:] = ctypes.string_at(out, need)
        finally:
          libc = ctypes.CDLL("libc.so.6")
          libc.free.argtypes = [ctypes.c_void_p]
          libc.free(ctypes.cast(out, ctypes.c_void_p))
      return w_i.value, h_i.value, buf
    w, h, rgba = qoi_decode(raw)
    self._buf[idx] = bytearray(rgba)
    return w, h, self._buf[idx]

  def send_cam(self, lat, lon, zoom, bearing, width, height, pitch=0.0):
    now = time.monotonic()
    # 50Hz：中心已是速度外推的连续轨迹，高频率下发让 maprenderd 每个渲染帧
    # 拿到的位置滞后 <20ms；10Hz 阶梯被渲染线程非均匀抽样会产生快慢性交替
    if now - self._last_cam_t < 0.02:
      return
    self._last_cam_t = now
    self._cam_active_t = now
    msg = messaging.new_message("mapRenderCam")
    c = msg.mapRenderCam
    c.lat, c.lon, c.zoom, c.bearing = lat, lon, zoom, bearing
    c.pitch = pitch
    c.width, c.height = width, height
    self._pm.send("mapRenderCam", msg)

  def frame_texture(self, sm) -> rl.Texture2D | None:
    now = time.monotonic()
    if self._fsm is None:
      self._fsm = messaging.SubMaster(["mapRenderFrame"])
      self._fsm_born = now
    fsm = self._fsm
    fsm.update(0)
    if fsm.updated["mapRenderFrame"] and fsm.valid["mapRenderFrame"]:
      try:
        fr = fsm["mapRenderFrame"]
        raw = bytes(fr.img)
        cam = (fr.camLat, fr.camLon, fr.camZoom, fr.camBearing, fr.camPitch)
        self._last_frame_t = now
        with self._cond:
          self._raw = raw      # 只保留最新帧，解码跟不上就丢
          self._raw_cam = cam  # 相机回显与帧字节同生死，避免错配
          self._cond.notify()
      except Exception:
        pass
    # 停滞自愈：面板在发相机却 1.5s 收不到帧 → socket 落在旧实例上，
    # 重建 SubMaster 重新挂载（限频 3s 避免抖动）
    if (self._cam_active_t > 0.0 and now - self._cam_active_t < 1.0
        and (self._last_frame_t == 0.0 or now - self._last_frame_t > 1.5)
        and now - self._fsm_born > 3.0):
      self._fsm = None
    # 主线程：把最新解好的缓冲上传为纹理
    idx = -1
    with self._cond:
      for i in range(self._N_BUF):
        if self._state[i] == 1:
          self._state[i] = 2  # UPLOADING
          idx = i
    if idx < 0:
      return self._tex
    w, h = self._meta[idx]
    data = self._buf[idx] if self._buf[idx] is not None else b""
    try:
      if (self._tex is not None and rl.is_texture_valid(self._tex)
          and self._tex_size == (w, h)):
        # 同尺寸帧直接原位更新 GPU 显存，省每帧 ~3MB 纹理新建/销毁
        rl.update_texture(self._tex, bytes(data))
        tex = self._tex
      else:
        img = rl.Image(bytes(data), w, h, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
        tex = rl.load_texture_from_image(img)
    except Exception:
      with self._cond:
        self._state[idx] = 0
      return self._tex
    with self._cond:
      self.cam_shown = self._cam_meta[idx]
      self._state[idx] = 0  # FREE
    if tex is not self._tex:
      if self._tex is not None and rl.is_texture_valid(self._tex):
        rl.unload_texture(self._tex)
      self._tex = tex
    self._tex_size = (w, h)
    return tex
