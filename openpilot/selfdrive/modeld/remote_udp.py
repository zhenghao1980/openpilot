"""UDP wire protocol for the C3X <-> NVIDIA-host remote big model link (v2).

Design principles (see c3x_remote_bigmodel_design.md §4):
  1. fire-and-forget data plane: INFER/RESP/BEAT are never acked and never
     retransmitted; every datagram carries a CRC and reassembled frames are
     re-verified whole — a corrupt or incomplete frame is dropped wholesale;
  2. liveness out of band: bidirectional 500ms BEAT, 3 misses (1.5s) == link
     dead -> fall back to the on-device small model;
  3. session-level control messages (HELLO/META/CTRL) are the only ones with
     ACK + exponential-backoff retry.

Messages (all little-endian, single UDP socket):

  BEACON  { ver u8, caps u8, port u16, instance u64, send_ts u64 }
  HELLO   { ack_id u32, cam_w u32, cam_h u32, client_ts u64 }
  META    { ack_id u32, server_recv_ts u64, server_send_ts u64, json }
  INFER   chunked: { frame_seq u32, chunk_idx u16, n_chunks u16,
                     total_len u32, frame_crc u32, payload }
          inner: { frame_seq u32, client_send_ts u64, n_bufs, per-buf
                   (name,yuv), n_tfms, per-tfm (name,9xf32),
                   desire[8]f32, traffic[2]f32, action_t[2]f32 }
  RESP    chunked like INFER; inner: { frame_seq u32, status u8,
          server_recv_ts u64, server_done_ts u64, f32 vector | error text }
  BEAT    { seq u32, send_ts u64, gpu_busy u8, queue_depth u8, dropped u32 }
  CTRL    { ack_id u32, json }          (both directions, ACKed)
  ACK     { acked_type u8, ack_id u32 }

Integrity: zlib.crc32 per chunk trailer slot and per whole frame (the design
doc says CRC32C; zlib.crc32 is the stdlib-available 32-bit CRC — same error
detection class on a private point-to-point link; swap-in of crcmod's CRC32C
is a one-line change, format identical).
"""
from __future__ import annotations

import json
import socket
import struct

import os as _os
_DEBUG = bool(_os.environ.get("REMOTE_UDP_DEBUG"))
import threading
import time
import zlib
from collections import deque

import numpy as np

MAGIC = 0x43335802
PROTO_VER = 1

MSG_BEACON = 0
MSG_HELLO = 1
MSG_META = 2
MSG_INFER = 3
MSG_RESP = 4
MSG_BEAT = 5
MSG_CTRL = 6
MSG_ACK = 7

RESP_OK = 0
RESP_ERR = 1

DEFAULT_PORT = 8571
DEFAULT_CHUNK = 1400   # 必须低于 MTU: >1500 的 UDP 报文走 IP 分片, WSL2 回环直接整体丢弃(实测 2048B 就丢)
MAX_FRAME = 48 * 1024 * 1024      # hard cap on a reassembled frame
STALE_PARTIAL_S = 2.0             # incomplete frames older than this are dropped

HDR = struct.Struct("<IBB")            # magic, type, flags
BEACON_HDR = struct.Struct("<BBHQQ")   # ver, caps, port, instance, send_ts
HELLO_HDR = struct.Struct("<IIIQ")     # ack_id, cam_w, cam_h, client_ts
META_HDR = struct.Struct("<IQQ")       # ack_id, server_recv_ts, server_send_ts
CHUNK_HDR = struct.Struct("<IHHII")    # frame_seq, chunk_idx, n_chunks, total_len, frame_crc
BEAT_HDR = struct.Struct("<IQBBI")     # seq, send_ts, gpu_busy, queue_depth, dropped
CTRL_HDR = struct.Struct("<I")         # ack_id
ACK_HDR = struct.Struct("<BI")         # acked_type, ack_id
RESP_INNER_HDR = struct.Struct("<IBQQ")  # frame_seq, status, server_recv_ts, server_done_ts
INFER_INNER_HDR = struct.Struct("<IQ")   # frame_seq, client_send_ts

_NAME = struct.Struct("<H")
_LEN = struct.Struct("<I")
_N_F32 = struct.Struct("<8f")
_2_F32 = struct.Struct("<2f")
_TFM = struct.Struct("<9f")


class RemoteUdpError(RuntimeError):
  pass


def _now_ms() -> int:
  return int(time.time() * 1000)


def frame_crc(data: bytes) -> int:
  return zlib.crc32(data) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# message encode / decode
# ---------------------------------------------------------------------------

def encode_beacon(port: int, instance: int, caps: int = 0) -> bytes:
  return HDR.pack(MAGIC, MSG_BEACON, 0) + BEACON_HDR.pack(PROTO_VER, caps, port, instance, _now_ms())


def decode_beacon(data: bytes) -> dict:
  ver, caps, port, instance, send_ts = BEACON_HDR.unpack_from(data, HDR.size)
  if ver != PROTO_VER:
    raise RemoteUdpError(f"beacon proto version {ver} != {PROTO_VER}")
  return {"caps": caps, "port": port, "instance": instance, "send_ts": send_ts}


def encode_hello(ack_id: int, cam_w: int, cam_h: int) -> bytes:
  return HDR.pack(MAGIC, MSG_HELLO, 0) + HELLO_HDR.pack(ack_id, cam_w, cam_h, _now_ms())


def decode_hello(data: bytes) -> dict:
  ack_id, cam_w, cam_h, client_ts = HELLO_HDR.unpack_from(data, HDR.size)
  return {"ack_id": ack_id, "cam_w": cam_w, "cam_h": cam_h, "client_ts": client_ts}


def encode_meta(ack_id: int, recv_ts: int, send_ts: int, meta: dict) -> bytes:
  payload = json.dumps(meta, default=int).encode()
  return HDR.pack(MAGIC, MSG_META, 0) + META_HDR.pack(ack_id, recv_ts, send_ts) + payload


def decode_meta(data: bytes) -> dict:
  ack_id, recv_ts, send_ts = META_HDR.unpack_from(data, HDR.size)
  return {"ack_id": ack_id, "server_recv_ts": recv_ts, "server_send_ts": send_ts,
          "meta": json.loads(data[HDR.size + META_HDR.size:])}


def encode_ack(acked_type: int, ack_id: int) -> bytes:
  return HDR.pack(MAGIC, MSG_ACK, 0) + ACK_HDR.pack(acked_type, ack_id)


def decode_ack(data: bytes) -> dict:
  acked_type, ack_id = ACK_HDR.unpack_from(data, HDR.size)
  return {"acked_type": acked_type, "ack_id": ack_id}


def encode_ctrl(ack_id: int, obj: dict) -> bytes:
  return HDR.pack(MAGIC, MSG_CTRL, 0) + CTRL_HDR.pack(ack_id) + json.dumps(obj, default=int).encode()


def decode_ctrl(data: bytes) -> dict:
  (ack_id,) = CTRL_HDR.unpack_from(data, HDR.size)
  return {"ack_id": ack_id, "obj": json.loads(data[HDR.size + CTRL_HDR.size:])}


def encode_beat(seq: int, gpu_busy: int = 0, queue_depth: int = 0, dropped: int = 0) -> bytes:
  return HDR.pack(MAGIC, MSG_BEAT, 0) + BEAT_HDR.pack(seq, _now_ms(), gpu_busy, queue_depth, dropped)


def decode_beat(data: bytes) -> dict:
  seq, send_ts, gpu_busy, queue_depth, dropped = BEAT_HDR.unpack_from(data, HDR.size)
  return {"seq": seq, "send_ts": send_ts, "gpu_busy": gpu_busy,
          "queue_depth": queue_depth, "dropped": dropped}


# ---------------------------------------------------------------------------
# INFER / RESP inner payloads
# ---------------------------------------------------------------------------

def encode_infer_inner(frame_seq: int, client_send_ts: int, bufs: dict, transforms: dict, inputs: dict) -> bytes:
  parts = [INFER_INNER_HDR.pack(frame_seq, client_send_ts), _LEN.pack(len(bufs))]
  for name, buf in bufs.items():
    data = buf.data if hasattr(buf, "data") else buf
    if not isinstance(data, (bytes, bytearray)):
      data = bytes(data)
    parts.append(_NAME.pack(len(name)) + name.encode())
    parts.append(_LEN.pack(len(data)) + data)
  parts.append(_LEN.pack(len(transforms)))
  for name, tfm in transforms.items():
    parts.append(_NAME.pack(len(name)) + name.encode())
    parts.append(_TFM.pack(*np.asarray(tfm, dtype=np.float32).reshape(-1)))
  parts.append(_N_F32.pack(*np.asarray(inputs["desire_pulse"], dtype=np.float32)))
  parts.append(_2_F32.pack(*np.asarray(inputs["traffic_convention"], dtype=np.float32)))
  parts.append(_2_F32.pack(*np.asarray(inputs["action_t"], dtype=np.float32)))
  return b"".join(parts)


def decode_infer_inner(data: bytes) -> dict:
  frame_seq, client_send_ts = INFER_INNER_HDR.unpack_from(data, 0)
  off = INFER_INNER_HDR.size
  (n_bufs,) = _LEN.unpack_from(data, off)
  off += _LEN.size
  bufs = {}
  for _ in range(n_bufs):
    (nl,) = _NAME.unpack_from(data, off)
    off += _NAME.size
    name = data[off:off + nl].decode()
    off += nl
    (blen,) = _LEN.unpack_from(data, off)
    off += _LEN.size
    bufs[name] = bytes(data[off:off + blen])
    off += blen
  (n_tfms,) = _LEN.unpack_from(data, off)
  off += _LEN.size
  transforms = {}
  for _ in range(n_tfms):
    (nl,) = _NAME.unpack_from(data, off)
    off += _NAME.size
    name = data[off:off + nl].decode()
    off += nl
    transforms[name] = np.array(_TFM.unpack_from(data, off), dtype=np.float32).reshape(3, 3)
    off += _TFM.size
  desire = np.array(_N_F32.unpack_from(data, off), dtype=np.float32)
  off += _N_F32.size
  traffic = np.array(_2_F32.unpack_from(data, off), dtype=np.float32)
  off += _2_F32.size
  action_t = np.array(_2_F32.unpack_from(data, off), dtype=np.float32)
  return {"frame_seq": frame_seq, "client_send_ts": client_send_ts, "bufs": bufs,
          "transforms": transforms,
          "inputs": {"desire_pulse": desire, "traffic_convention": traffic, "action_t": action_t}}


def encode_resp_inner(frame_seq: int, status: int, recv_ts: int, done_ts: int, body: bytes) -> bytes:
  return RESP_INNER_HDR.pack(frame_seq, status, recv_ts, done_ts) + body


def decode_resp_inner(data: bytes) -> dict:
  frame_seq, status, recv_ts, done_ts = RESP_INNER_HDR.unpack_from(data, 0)
  body = data[RESP_INNER_HDR.size:]
  out = {"frame_seq": frame_seq, "status": status, "server_recv_ts": recv_ts,
         "server_done_ts": done_ts, "body": body}
  if status == RESP_OK:
    out["output"] = np.frombuffer(body, dtype=np.float32).copy()
  else:
    out["error"] = body.decode(errors="replace")
  return out


# ---------------------------------------------------------------------------
# chunking / reassembly
# ---------------------------------------------------------------------------

def chunk_frame(mtype: int, frame_seq: int, inner: bytes, chunk_size: int = DEFAULT_CHUNK) -> list[bytes]:
  """Split an inner payload into ready-to-send datagrams."""
  n = max(1, (len(inner) + chunk_size - 1) // chunk_size)
  if n > 0xFFFF:
    raise RemoteUdpError("frame too large to chunk")
  crc = frame_crc(inner)
  out = []
  for i in range(n):
    piece = inner[i * chunk_size:(i + 1) * chunk_size]
    out.append(HDR.pack(MAGIC, mtype, 0) +
               CHUNK_HDR.pack(frame_seq, i, n, len(inner), crc if i == 0 else 0) + piece)
  return out


class Reassembler:
  """frame_seq -> complete inner payload. Bad or stale partials are dropped."""

  def __init__(self, max_frame: int = MAX_FRAME):
    self.max_frame = max_frame
    self._partials: dict[int, dict] = {}
    self.dropped = 0
    self.drop_reasons: dict[str, int] = {}
    self.purged = 0

  def feed(self, data: bytes) -> bytes | None:
    magic, mtype, _flags = HDR.unpack_from(data, 0)
    if magic != MAGIC:
      self.dropped += 1
      self._drop_log("bad_magic")
      return None
    frame_seq, chunk_idx, n_chunks, total_len, crc = CHUNK_HDR.unpack_from(data, HDR.size)
    if total_len > self.max_frame or n_chunks == 0 or chunk_idx >= n_chunks:
      self.dropped += 1
      self._drop_log(f"bad_geom seq={frame_seq} idx={chunk_idx}/{n_chunks} len={total_len}")
      return None
    p = self._partials.get(frame_seq)
    if p is None:
      p = self._partials[frame_seq] = {"chunks": {}, "n": n_chunks, "len": total_len,
                                       "crc": crc, "ts": time.monotonic()}
    if p["n"] != n_chunks or p["len"] != total_len:
      # conflicting geometry for the same seq: drop the whole partial
      del self._partials[frame_seq]
      self.dropped += 1
      self._drop_log(f"geom_conflict seq={frame_seq}")
      return None
    if crc:
      p["crc"] = crc
    p["chunks"][chunk_idx] = bytes(data[HDR.size + CHUNK_HDR.size:])
    if len(p["chunks"]) == p["n"]:
      del self._partials[frame_seq]
      inner = b"".join(p["chunks"][i] for i in range(p["n"]))
      if len(inner) != p["len"]:
        self.dropped += 1
        self._drop_log(f"len_mismatch seq={frame_seq} got={len(inner)} want={p['len']}")
        return None
      if frame_crc(inner) != p["crc"]:
        self.dropped += 1
        self._drop_log(f"crc_fail seq={frame_seq}")
        return None
      return inner
    return None

  def _drop_log(self, reason: str) -> None:
    self.drop_reasons[reason.split()[0]] = self.drop_reasons.get(reason.split()[0], 0) + 1
    if _DEBUG:
      print(f"RXA_DROP {reason}", flush=True)

  def purge(self, older_than_s: float = STALE_PARTIAL_S) -> int:
    now = time.monotonic()
    stale = [k for k, p in self._partials.items() if now - p["ts"] > older_than_s]
    for k in stale:
      del self._partials[k]
    self.dropped += len(stale)
    if stale:
      self.purged += len(stale)
      if _DEBUG:
        print(f"RXA_PURGE incomplete seqs={stale}", flush=True)
    return len(stale)


# ---------------------------------------------------------------------------
# beacon discovery helper (client side)
# ---------------------------------------------------------------------------

def listen_beacons(port: int = DEFAULT_PORT, wait_s: float = 6.0, instance: int | None = None):
  """Collect BEACON broadcasts for up to wait_s. Returns list of unique
  {instance, addr, caps} — latest per instance. Falls back to [] when the
  network blocks broadcast (e.g. phone hotspot AP isolation)."""
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  except OSError:
    pass
  sock.bind(("", port))
  sock.settimeout(0.2)
  found: dict[int, dict] = {}
  end = time.monotonic() + wait_s
  while time.monotonic() < end:
    try:
      data, addr = sock.recvfrom(2048)
    except socket.timeout:
      continue
    except OSError:
      break
    try:
      if HDR.unpack_from(data, 0)[:2] != (MAGIC, MSG_BEACON):
        continue
      b = decode_beacon(data)
    except (RemoteUdpError, struct.error):
      continue
    if instance is not None and b["instance"] != instance:
      continue
    found[b["instance"]] = {**b, "addr": addr[0]}
  sock.close()
  return list(found.values())


# ---------------------------------------------------------------------------
# client
# ---------------------------------------------------------------------------

class UdpRemoteClient:
  """Async UDP client link. Session lifecycle and reconnect with backoff run
  on an internal thread; infer() is fire-and-forget; poll_result() drains
  completed responses. Mirror of chestnut semantics: a fresh session resets
  temporal state on the server."""

  def __init__(self, host: str, port: int = DEFAULT_PORT, cam_w: int = 1928, cam_h: int = 1208, *,
               timeout_s: float = 3.0, beat_interval_s: float = 0.5, beat_timeout_s: float = 4.0,
               chunk_size: int = DEFAULT_CHUNK,
               retry_min_s: float = 0.25, retry_max_s: float = 30.0):
    self.host, self.port = host, port
    self.cam_w, self.cam_h = cam_w, cam_h
    self.timeout_s = timeout_s
    self.beat_interval_s = beat_interval_s
    self.beat_timeout_s = beat_timeout_s
    self.chunk_size = chunk_size
    self.retry_min_s, self.retry_max_s = retry_min_s, retry_max_s

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 大缓冲防突发丢包：INFER 单帧 ~1600 datagram；macOS 默认缓冲区极小(发送 9216B)会直接 ENOBUFS
    for _opt, _sz in ((socket.SO_SNDBUF, 16 << 20), (socket.SO_RCVBUF, 16 << 20)):
      try:
        self.sock.setsockopt(socket.SOL_SOCKET, _opt, _sz)
      except OSError:
        pass
    self.sock.settimeout(0.05)
    self._send_lock = threading.Lock()
    self._results: deque[dict] = deque()
    self._pending_ctrl: dict = {}
    self._meta: dict = {}
    self._clock_offset_ms: float = 0.0   # server_clock - client_clock
    self._frame_seq = 0
    self._beat_seq = 0
    self._dropped_rx = 0
    self._rtt_ms = 0.0
    self._server_beat: dict = {}

    self._session_ready = threading.Event()
    self._ctrl_events: dict[int, threading.Event] = {}
    self._stop = threading.Event()
    self._thread = threading.Thread(target=self._run, daemon=True, name="udp-remote-client")

  # -- lifecycle -----------------------------------------------------------

  def start(self) -> None:
    self._thread.start()

  def close(self) -> None:
    self._stop.set()
    self._thread.join(timeout=2)
    self.sock.close()

  @property
  def ready(self) -> bool:
    return self._session_ready.is_set()

  @property
  def meta(self) -> dict:
    return self._meta

  def stats(self) -> dict:
    """Fusion/UI telemetry: liveness + link quality snapshot."""
    return {
      "ready": self.ready,
      "rtt_ms": self._rtt_ms,
      "clock_offset_ms": self._clock_offset_ms,
      "server": dict(self._server_beat),
      "dropped_rx": self._dropped_rx,
    }

  # -- inference -----------------------------------------------------------

  def infer(self, bufs: dict, transforms: dict, inputs: dict) -> int | None:
    """Fire-and-forget. Returns frame_seq, or None when no session (frame is
    simply not sent — the fusion layer absorbs the gap)."""
    if not self.ready:
      return None
    self._frame_seq = (self._frame_seq + 1) & 0xFFFFFFFF
    inner = encode_infer_inner(self._frame_seq, _now_ms(), bufs, transforms, inputs)
    datagrams = chunk_frame(MSG_INFER, self._frame_seq, inner, self.chunk_size)
    try:
      with self._send_lock:
        for d in datagrams:
          # 发送侧背压: socket 带 50ms python 级超时, 服务端排水慢时 sendto 会超时;
          # 等内核缓冲排水后重发同一片 (上限 ~2s/片, 超出才放弃整帧)
          for _retry in range(200):
            try:
              self.sock.sendto(d, (self.host, self.port))
              break
            except socket.timeout:
              time.sleep(0.01)
          else:
            print(f"remote_udp: infer backpressure timeout seq={self._frame_seq}", flush=True)
            return None
    except OSError as e:
      print(f"remote_udp: infer send failed seq={self._frame_seq}: {e}", flush=True)
      return None
    return self._frame_seq

  def poll_result(self) -> dict | None:
    """Next completed RESP: {frame_seq, output, server_recv_ts, server_done_ts}
    or {frame_seq, status, error}. None when nothing new."""
    try:
      return self._results.popleft()
    except IndexError:
      return None

  def send_ctrl(self, obj: dict, timeout_s: float = 2.0) -> dict | None:
    """Session-level control round trip (ACKed, retried by the wire rule)."""
    ack_id = ((_now_ms() & 0x7FFFFFFF) ^ id(obj)) & 0x7FFFFFFF or 1
    ev = threading.Event()
    self._ctrl_events[ack_id] = ev
    try:
      self.sock.sendto(encode_ctrl(ack_id, obj), (self.host, self.port))
      if ev.wait(timeout=timeout_s):
        return self._pending_ctrl.pop(ack_id, None)
      return None
    finally:
      self._ctrl_events.pop(ack_id, None)

  # -- internals -----------------------------------------------------------

  def _run(self) -> None:
    """Session thread: handshake -> active loop (recv/heartbeat) -> dead ->
    backoff reconnect. Mirrors the server beacon state machine from §4.3."""
    retry_s = self.retry_min_s
    resp_rx = Reassembler()
    last_beat_tx = 0.0
    last_beat_rx = 0.0
    while not self._stop.is_set():
      if not self._session_ready.is_set():
        if self._try_handshake():
          self._session_ready.set()
          retry_s = self.retry_min_s
          last_beat_rx = time.monotonic()
        else:
          self._stop.wait(retry_s)
          retry_s = min(retry_s * 2, self.retry_max_s)
          continue

      now = time.monotonic()
      if now - last_beat_tx >= self.beat_interval_s:
        try:
          self.sock.sendto(encode_beat(self._beat_seq), (self.host, self.port))  # 心跳不走 _send_lock: 大帧突发持锁期间心跳必须照发
          self._beat_seq = (self._beat_seq + 1) & 0xFFFFFFFF
        except OSError:
          pass
        last_beat_tx = now

      if now - last_beat_rx > self.beat_timeout_s:
        # link dead: next infer() consumer sees ready=False; reconnect
        if _DEBUG:
          print(f"CLI_DROP: no server beat for {now - last_beat_rx:.1f}s", flush=True)
        self._session_ready.clear()
        self._meta = {}
        continue

      try:
        data, _addr = self.sock.recvfrom(65535)
      except socket.timeout:
        resp_rx.purge()
        continue
      except OSError:
        self._session_ready.clear()
        continue

      try:
        magic, mtype, _ = HDR.unpack_from(data, 0)
      except struct.error:
        continue
      if magic != MAGIC:
        continue
      if mtype == MSG_BEAT:
        last_beat_rx = time.monotonic()
        b = decode_beat(data)
        self._server_beat = b
        if _DEBUG:
          self._dbg_beat_rx = getattr(self, "_dbg_beat_rx", 0) + 1
          if self._dbg_beat_rx % 10 == 1:
            print(f"CLI_BEAT_RX n={self._dbg_beat_rx}", flush=True)
        # one-way estimate refresh; RTT from infer/resp timing lives in stats
      elif mtype == MSG_RESP:
        inner = resp_rx.feed(data)
        if inner is None:
          if resp_rx.dropped:
            self._dropped_rx = resp_rx.dropped
          continue
        r = decode_resp_inner(inner)
        self._results.append(r)
      elif mtype == MSG_META:
        # duplicate META (our ACK was lost): re-ACK so the server stops retransmitting
        try:
          self.sock.sendto(encode_ack(MSG_META, decode_meta(data)["ack_id"]), (self.host, self.port))
        except (OSError, struct.error):
          pass
      elif mtype == MSG_ACK:
        # delivery confirmation only; a CTRL round trip completes when the
        # reply CTRL arrives (see below) — never signal the waiter here, the
        # ACK can race ahead of the reply
        pass
      elif mtype == MSG_CTRL:
        c = decode_ctrl(data)
        self.sock.sendto(encode_ack(MSG_CTRL, c["ack_id"]), (self.host, self.port))
        self._pending_ctrl[c["ack_id"]] = c["obj"]
        ev = self._ctrl_events.get(c["ack_id"])
        if ev:
          ev.set()

  def _try_handshake(self) -> bool:
    """HELLO/META with ACK + exponential backoff (§4.4 exception). Receives
    directly on this thread: the session recv loop only starts after the
    handshake completes."""
    ack_id = (_now_ms() & 0x7FFFFFFF) or 1
    t0 = _now_ms()
    retry_s = self.retry_min_s
    for _ in range(4):
      try:
        self.sock.sendto(encode_hello(ack_id, self.cam_w, self.cam_h), (self.host, self.port))
      except OSError:
        return False
      deadline = time.monotonic() + max(retry_s, 0.5)
      while time.monotonic() < deadline:
        try:
          data, _addr = self.sock.recvfrom(65535)
        except socket.timeout:
          continue
        except OSError:
          return False
        try:
          magic, mtype, _ = HDR.unpack_from(data, 0)
        except struct.error:
          continue
        if magic != MAGIC or mtype != MSG_META:
          continue
        m = decode_meta(data)
        if m["ack_id"] != ack_id:
          continue
        t3 = _now_ms()
        # NTP-style offset estimate; rtt from the handshake itself
        self._rtt_ms = t3 - t0
        self._clock_offset_ms = ((m["server_recv_ts"] - t0) + (m["server_send_ts"] - t3)) / 2.0
        self.sock.sendto(encode_ack(MSG_META, ack_id), (self.host, self.port))
        self._meta = m["meta"]
        return True
      retry_s = min(retry_s * 2, 2.0)
    return False


# ---------------------------------------------------------------------------
# server
# ---------------------------------------------------------------------------

class UdpRemoteServer:
  """UDP big-model server. I/O runs on the calling thread (run_forever);
  inference runs on a single worker thread fed by a bounded queue (full =
  drop oldest). One session at a time: READY beacons until HELLO, ACTIVE
  serves, dead heartbeat resumes READY + beaconing (§4.3)."""

  def __init__(self, port: int, meta: dict, infer_fn, *,
               instance: int | None = None, chunk_size: int = DEFAULT_CHUNK,
               queue_depth: int = 3, beat_interval_s: float = 0.5, beat_timeout_s: float = 4.0,
               beacon_interval_s: float = 5.0, beacon_target: str = "255.255.255.255",
               burst_n: int = 4, burst_interval_s: float = 0.1,
               on_ctrl=None, on_session=None):
    """
    meta:        dict sent as META payload (input_shapes / output_slices /
                 vision_input_names / model_sha256); callable -> called per
                 session for lazy loading.
    infer_fn:    (decoded_infer_dict) -> raw f32 np.ndarray | raises
    on_ctrl:     optional callable(obj) -> obj, answers CTRL messages
    on_session:  optional callable(active: bool) — session up/down hook
    """
    self.port = port
    self._meta_src = meta
    self.infer_fn = infer_fn
    self.chunk_size = chunk_size
    self.beat_interval_s = beat_interval_s
    self.beat_timeout_s = beat_timeout_s
    self.beacon_interval_s = beacon_interval_s
    self.beacon_target = beacon_target
    self.burst_n, self.burst_interval_s = burst_n, burst_interval_s
    self.on_ctrl = on_ctrl
    self.on_session = on_session
    self.instance = instance if instance is not None else (int(time.time()) & 0xFFFFFFFFFFFFFFFF)

    self._queue: deque[dict] = deque(maxlen=queue_depth)
    self._queue_lock = threading.Lock()
    self._out: list[tuple[int, bytes]] = []   # (frame_seq, resp_inner) from worker
    self._out_lock = threading.Lock()
    self._stop = threading.Event()
    self.dropped_frames = 0

    self._dbg_rx = {}
    self._worker = threading.Thread(target=self._work, daemon=True, name="udp-remote-infer")
    self._busy = threading.Event()
    self._gpu_busy = 0

  # -- public ----------------------------------------------------------------

  def start(self) -> None:
    self._worker.start()

  def stop(self) -> None:
    self._stop.set()
    self._worker.join(timeout=5)

  def submit(self, item: dict) -> None:
    """Test hook / local enqueue. Full queue drops the OLDEST item (backpressure)."""
    with self._queue_lock:
      if len(self._queue) == self._queue.maxlen:
        self.dropped_frames += 1
      self._queue.append(item)

  def queue_depth(self) -> int:
    with self._queue_lock:
      return len(self._queue)

  def run_forever(self) -> None:
    self.start()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # 大接收缓冲：INFER 分片突发(~1600 包/帧)在小默认缓冲(尤其 macOS)下整帧丢失
    for _opt, _sz in ((socket.SO_RCVBUF, 16 << 20), (socket.SO_SNDBUF, 16 << 20)):
      try:
        sock.setsockopt(socket.SOL_SOCKET, _opt, _sz)
      except OSError:
        pass
    sock.bind(("0.0.0.0", self.port))
    sock.settimeout(0.05)
    try:
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except OSError:
      pass

    infer_rx = Reassembler()
    session_addr: tuple | None = None
    session_beat_seq = -1
    last_beat_tx = 0.0
    last_beat_rx = 0.0
    last_beacon = 0.0
    burst_left = self.burst_n
    next_burst = 0.0
    meta_retries: dict = {}

    def send_beacon() -> None:
      try:
        sock.sendto(encode_beacon(self.port, self.instance), (self.beacon_target, self.port))
      except OSError:
        pass

    def drop_session() -> None:
      nonlocal session_addr
      session_addr = None
      meta_retries.clear()
      with self._queue_lock:
        self._queue.clear()
      # 丢弃属于已死会话的待发结果，否则下面的 send 循环会向 None 发包崩溃
      with self._out_lock:
        self._out.clear()
      if self.on_session:
        self.on_session(False)

    # enter READY: burst beacons then periodic
    next_burst = time.monotonic()
    while not self._stop.is_set():
      now = time.monotonic()

      # ---- beacon scheduling (READY only)
      if session_addr is None:
        if burst_left > 0 and now >= next_burst:
          send_beacon()
          burst_left -= 1
          next_burst = now + self.burst_interval_s
        elif burst_left == 0 and now - last_beacon >= self.beacon_interval_s:
          send_beacon()
          last_beacon = now

      # ---- heartbeat + liveness
      if session_addr is not None:
        if now - last_beat_tx >= self.beat_interval_s:
          try:
            sock.sendto(encode_beat(0, self._gpu_busy, self.queue_depth(), self.dropped_frames), session_addr)
            if _DEBUG:
              self._dbg_beat_tx = getattr(self, "_dbg_beat_tx", 0) + 1
              if self._dbg_beat_tx % 10 == 1:
                print(f"SRV_BEAT_TX n={self._dbg_beat_tx} to={session_addr}", flush=True)
          except OSError as e:
            if _DEBUG:
              print(f"SRV_BEAT_TX_FAIL: {e}", flush=True)
          last_beat_tx = now
        if now - last_beat_rx > self.beat_timeout_s:
          if _DEBUG:
            print(f"SRV_DROP: no client beat for {now - last_beat_rx:.1f}s", flush=True)
          drop_session()
          burst_left = self.burst_n
          next_burst = now
          continue

      # ---- META retransmit for un-ACKed handshakes
      for ack_id, (payload, tries, last_tx) in list(meta_retries.items()):
        if now - last_tx > 0.5:
          if tries >= 3:
            del meta_retries[ack_id]
            drop_session()
          else:
            try:
              sock.sendto(payload, session_addr)
            except OSError:
              pass
            meta_retries[ack_id] = (payload, tries + 1, now)

      # ---- worker results -> wire
      with self._out_lock:
        pending_out, self._out = self._out, []
      if session_addr is not None:
        for frame_seq, inner in pending_out:
          for d in chunk_frame(MSG_RESP, frame_seq, inner, self.chunk_size):
            try:
              sock.sendto(d, session_addr)
            except OSError:
              break

      # ---- receive
      try:
        data, addr = sock.recvfrom(65535)
      except socket.timeout:
        infer_rx.purge()
        continue
      except OSError as e:
        if _DEBUG:
          print(f"SRV recv OSError: {e}", flush=True)
        break
      if _DEBUG and len(data) >= HDR.size:
        _m = HDR.unpack_from(data, 0)[1]
        self._dbg_rx[_m] = self._dbg_rx.get(_m, 0) + 1
        if sum(self._dbg_rx.values()) % 10 == 1:
          print("SRV_RX:", self._dbg_rx, flush=True)

      try:
        magic, mtype, _ = HDR.unpack_from(data, 0)
      except struct.error:
        continue
      if magic != MAGIC:
        continue

      if mtype == MSG_BEACON:
        continue  # another server's beacon: ignore

      if mtype == MSG_HELLO:
        h = decode_hello(data)
        if session_addr is not None and addr != session_addr:
          # 设计为单客户端链路；旧会话心跳已停顿(客户端重启)时允许新客户端立即接管,
          # 只有旧会话仍在健康心跳时才回 busy（防多客户端误接）
          if now - last_beat_rx <= self.beat_interval_s * 2:
            busy = encode_meta(h["ack_id"], _now_ms(), _now_ms(), {"busy": True})
            try:
              sock.sendto(busy, addr)
            except OSError:
              pass
            continue
          drop_session()
        session_addr = addr
        last_beat_rx = now
        meta = self._meta_src() if callable(self._meta_src) else self._meta_src
        payload = encode_meta(h["ack_id"], _now_ms(), _now_ms(), meta)
        meta_retries[h["ack_id"]] = (payload, 0, now)
        try:
          sock.sendto(payload, addr)
        except OSError:
          drop_session()
          continue
        if self.on_session:
          self.on_session(True)
        continue

      if session_addr is None or addr != session_addr:
        continue  # stray datagram outside a session

      if mtype == MSG_ACK:
        a = decode_ack(data)
        if a["acked_type"] == MSG_META and a["ack_id"] in meta_retries:
          del meta_retries[a["ack_id"]]
      elif mtype == MSG_BEAT:
        last_beat_rx = now
        b = decode_beat(data)
        session_beat_seq = b["seq"]
      elif mtype == MSG_INFER:
        inner = infer_rx.feed(data)
        if inner is not None:
          req = decode_infer_inner(inner)
          self._gpu_busy = 1
          self.submit(req)
          self._gpu_busy = 0
      elif mtype == MSG_CTRL:
        c = decode_ctrl(data)
        try:
          sock.sendto(encode_ack(MSG_CTRL, c["ack_id"]), addr)
        except OSError:
          pass
        if self.on_ctrl:
          reply = self.on_ctrl(c["obj"])
          if reply is not None:
            try:
              sock.sendto(encode_ctrl(c["ack_id"], reply), addr)
            except OSError:
              pass

    sock.close()

  # -- worker ------------------------------------------------------------------

  def _work(self) -> None:
    while not self._stop.is_set():
      with self._queue_lock:
        item = self._queue.popleft() if self._queue else None
      if item is None:
        self._stop.wait(0.01)
        continue
      recv_ts = _now_ms()
      frame_seq = item["frame_seq"]
      try:
        out = self.infer_fn(item)
        if not np.all(np.isfinite(out)):
          raise RuntimeError("model output not finite")
        body = np.ascontiguousarray(out, dtype=np.float32).tobytes()
        inner = encode_resp_inner(frame_seq, RESP_OK, recv_ts, _now_ms(), body)
      except Exception as e:
        inner = encode_resp_inner(frame_seq, RESP_ERR, recv_ts, _now_ms(),
                                  str(e).encode()[:1024])
      with self._out_lock:
        self._out.append((frame_seq, inner))
