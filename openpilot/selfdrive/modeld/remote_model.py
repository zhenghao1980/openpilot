"""Remote big-model client: replaces the chestnut co-processor with a
network-connected host running openpilot.tools.remoted.remote_modeld_server.

Wire protocol (length-prefixed, little-endian, TCP):

  HELLO  -> | u32 magic | u8 type=0 | u32 cam_w | u32 cam_h |
  META   <- | u32 magic | u8 status | u32 len | json |

  INFER  -> | u32 magic | u8 type=1 | u32 frame_id |
            | u32 n_bufs | per buf: u16 name_len, name, u32 len, yuv bytes |
            | u32 n_tfms | per tfm: u16 name_len, name, 9xf32 |
            | 8xf32 desire_pulse | 2xf32 traffic_convention | 2xf32 action_t |
  RESP   <- | u32 magic | u8 status | u32 len | f32 model output vector |

Configuration (environment):
  REMOTE_MODEL_HOST       IP/hostname of the NVIDIA host (unset disables remote)
  REMOTE_MODEL_PORT       TCP port, default 8571
  REMOTE_MODEL_TIMEOUT_MS per-request timeout, default 300

Any connection/timeout/protocol failure raises RemoteModelError, which the
try/except in modeld.py converts into the stock chestnut fallback: switch to
the small on-device model for the rest of the drive.
"""
import json
import os
import socket
import struct

import numpy as np

from openpilot.selfdrive.modeld.parse_model_outputs import Parser

MAGIC = 0x43335801

MSG_HELLO = 0
MSG_INFER = 1

RESP_OK = 0
RESP_ERR = 1

_HEADER = struct.Struct("<IB")       # magic, type
_HELLO = struct.Struct("<II")        # cam_w, cam_h
_INFER_HDR = struct.Struct("<II")    # frame_id, n_bufs
_NAME = struct.Struct("<H")          # name length
_LEN = struct.Struct("<I")           # payload length
_TFM_LEN = struct.Struct("<I")       # n transforms
_RESP_HDR = struct.Struct("<IBI")    # magic, status, payload_len

_N_F32 = struct.Struct("<8f")
_2_F32 = struct.Struct("<2f")

DEFAULT_PORT = 8571
DEFAULT_TIMEOUT_MS = 300


class RemoteModelError(RuntimeError):
  pass


def _remote_host() -> str | None:
  return os.environ.get("REMOTE_MODEL_HOST") or None


def remote_port() -> int:
  return int(os.environ.get("REMOTE_MODEL_PORT", DEFAULT_PORT))


def remote_timeout() -> float:
  return int(os.environ.get("REMOTE_MODEL_TIMEOUT_MS", DEFAULT_TIMEOUT_MS)) / 1000.


def _recv_exact(sock: socket.socket, n: int) -> bytes:
  buf = bytearray(n)
  mv = memoryview(buf)
  got = 0
  while got < n:
    k = sock.recv_into(mv[got:])
    if k == 0:
      raise RemoteModelError("connection closed by remote model server")
    got += k
  return bytes(buf)


class RemoteModelClient:
  """Persistent connection to a remote_modeld_server instance."""

  def __init__(self, cam_w: int, cam_h: int):
    self.cam_w, self.cam_h = cam_w, cam_h
    self._frame_id = 0
    self.sock: socket.socket | None = None
    self.meta: dict = {}
    self._connect()

  def _connect(self) -> None:
    host = _remote_host()
    if host is None:
      raise RemoteModelError("REMOTE_MODEL_HOST is not set")
    sock = socket.create_connection((host, remote_port()), timeout=remote_timeout())
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    self.sock = sock
    self._hello()

  def _hello(self) -> None:
    assert self.sock is not None
    self.sock.sendall(_HEADER.pack(MAGIC, MSG_HELLO) + _HELLO.pack(self.cam_w, self.cam_h))
    magic, status, plen = _RESP_HDR.unpack(_recv_exact(self.sock, _RESP_HDR.size))
    payload = _recv_exact(self.sock, plen)
    if magic != MAGIC:
      raise RemoteModelError("bad magic in hello response")
    if status != RESP_OK:
      raise RemoteModelError(payload.decode(errors="replace"))
    self.meta = json.loads(payload)

  def infer(self, bufs: dict, transforms: dict, inputs: dict, after_send=None) -> np.ndarray:
    """Send one model iteration, block for the raw float32 output vector."""
    assert self.sock is not None
    self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF

    parts = [_HEADER.pack(MAGIC, MSG_INFER), _INFER_HDR.pack(self._frame_id, len(bufs))]
    for name, buf in bufs.items():
      data = buf.data
      if not isinstance(data, (bytes, bytearray)):
        data = bytes(data)
      parts.append(_NAME.pack(len(name)) + name.encode())
      parts.append(_LEN.pack(len(data)) + data)
    parts.append(_TFM_LEN.pack(len(transforms)))
    for name, tfm in transforms.items():
      parts.append(_NAME.pack(len(name)) + name.encode())
      parts.append(np.ascontiguousarray(tfm, dtype=np.float32).tobytes())
    parts.append(_N_F32.pack(*np.asarray(inputs['desire_pulse'], dtype=np.float32)))
    parts.append(_2_F32.pack(*np.asarray(inputs['traffic_convention'], dtype=np.float32)))
    parts.append(_2_F32.pack(*np.asarray(inputs['action_t'], dtype=np.float32)))

    try:
      self.sock.sendall(b"".join(parts))
      if after_send is not None:
        after_send()  # do ancillary work (e.g. chestnutState publish) while the host computes
      magic, status, plen = _RESP_HDR.unpack(_recv_exact(self.sock, _RESP_HDR.size))
      payload = _recv_exact(self.sock, plen)
      if magic != MAGIC:
        raise RemoteModelError("bad magic in infer response")
      if status != RESP_OK:
        raise RemoteModelError(payload.decode(errors="replace"))
      return np.frombuffer(payload, dtype=np.float32).copy()
    except RemoteModelError:
      self.close()
      raise
    except (socket.timeout, TimeoutError, ConnectionError, OSError) as e:
      self.close()
      raise RemoteModelError(f"remote model request failed: {e}") from e

  def close(self) -> None:
    if self.sock is not None:
      try:
        self.sock.close()
      finally:
        self.sock = None


def remote_model_metadata(cam_w: int, cam_h: int) -> dict | None:
  """Probe for a reachable remote big model. Returns server metadata or None.

  Mirrors chestnut_present() + chestnut_compiled(): any failure means the big
  model is simply unavailable and modeld proceeds with the small model.
  """
  if _remote_host() is None:
    return None
  try:
    return RemoteModelClient(cam_w, cam_h).meta
  except Exception:
    return None


class RemoteModelState:
  """Interface-compatible with the subset of ModelState used by modeld:
  .chestnut, .vision_input_names, .run(). The actual inference happens on the
  remote host; parsing happens here so the modelV2 message contract is unchanged."""

  def __init__(self, cam_w: int, cam_h: int, meta: dict):
    self.chestnut = True  # upstream flags: the remote big model acts like chestnut
    self.client = RemoteModelClient(cam_w, cam_h)
    self.input_shapes = {k: tuple(v) for k, v in meta["input_shapes"].items()}
    self.vision_input_names = [k for k in self.input_shapes if "img" in k]
    self.output_slices = {k: slice(a, b) for k, (a, b) in meta["output_slices"].items()}
    self.parser = Parser()

  def warmup(self) -> None:
    # the HELLO handshake already validated the server and model; nothing to warm
    pass

  def run(self, bufs: dict, transforms: dict, inputs: dict, after_enqueue=None) -> dict:
    model_output = self.client.infer(bufs, transforms, inputs, after_send=after_enqueue)
    if not np.all(np.isfinite(model_output)):
      raise RuntimeError("remote model output not finite")
    outputs_dict = self.parser.parse_outputs({k: model_output[np.newaxis, v] for k, v in self.output_slices.items()})
    if os.environ.get('SEND_RAW_PRED'):
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict
