"""Loopback tests for the v2 UDP remote-model protocol (remote_udp.py).

Run: python openpilot/tools/remoted/tests/test_remote_udp.py
No openpilot imports beyond numpy — plain stdlib event loop on 127.0.0.1.
"""
import socket
import struct
import threading
import time
import unittest

import numpy as np

from openpilot.selfdrive.modeld.remote_udp import (
  MAGIC, MSG_BEACON, MSG_HELLO, MSG_META, MSG_INFER, MSG_RESP, MSG_BEAT, MSG_CTRL,
  RESP_OK, RESP_ERR, HDR,
  RemoteUdpError, Reassembler, frame_crc, chunk_frame,
  encode_beacon, decode_beacon, encode_hello, decode_hello,
  encode_meta, decode_meta, encode_ack, decode_ack,
  encode_beat, decode_beat, encode_ctrl, decode_ctrl,
  encode_infer_inner, decode_infer_inner, encode_resp_inner, decode_resp_inner,
  listen_beacons, UdpRemoteClient, UdpRemoteServer,
)

PORT = 18771
CAM_W, CAM_H = 1928, 1208

META = {
  "input_shapes": {"img": [1, 12, 128, 256], "big_img": [1, 12, 128, 256]},
  "output_slices": {"plan": [0, 4950], "hidden_state": [6000, 6512]},
  "vision_input_names": ["img", "big_img"],
  "model_sha256": "ab" * 32,
}


def fake_bufs():
  return {
    "img": np.zeros(64, dtype=np.uint8),
    "big_img": np.ones(64, dtype=np.uint8),
  }


def fake_tfms():
  return {"img": np.eye(3, dtype=np.float32), "big_img": np.eye(3, dtype=np.float32)}


def fake_inputs():
  return {"desire_pulse": np.zeros(8, dtype=np.float32),
          "traffic_convention": np.array([1., 0.], dtype=np.float32),
          "action_t": np.array([0.1, 0.2], dtype=np.float32)}


class TestChunking(unittest.TestCase):
  def test_roundtrip_small_chunks(self):
    inner = np.arange(100000, dtype=np.float32).tobytes()
    for chunk_size in (1000, 64 * 1024, len(inner) + 10):
      rx = Reassembler()
      result = None
      for d in chunk_frame(MSG_INFER, 7, inner, chunk_size):
        result = rx.feed(d)
      self.assertIsNotNone(result)
      self.assertEqual(result, inner)
      self.assertEqual(rx.dropped, 0)

  def test_corrupt_chunk_drops_whole_frame(self):
    inner = b"x" * 100000
    dgrams = chunk_frame(MSG_INFER, 3, inner, 8000)
    dgrams[5] = dgrams[5][:-1] + b"\x00"          # flip a payload byte
    rx = Reassembler()
    self.assertIsNone(rx.feed(dgrams[5]))
    result = None
    for d in dgrams:
      if d is dgrams[5]:
        continue
      result = rx.feed(d)
    self.assertIsNone(result)
    self.assertGreaterEqual(rx.dropped, 1)

  def test_missing_chunk_then_purge(self):
    inner = b"y" * 50000
    dgrams = chunk_frame(MSG_INFER, 9, inner, 8000)
    rx = Reassembler()
    for d in dgrams[:-1]:
      self.assertIsNone(rx.feed(d))
    self.assertEqual(rx.purge(older_than_s=0.0), 1)

  def test_conflicting_geometry_dropped(self):
    inner = b"z" * 30000
    dgrams = chunk_frame(MSG_INFER, 1, inner, 8000)
    rx = Reassembler()
    rx.feed(dgrams[0])
    other = chunk_frame(MSG_INFER, 1, b"z" * 40000, 8000)[1]
    self.assertIsNone(rx.feed(other))
    self.assertGreaterEqual(rx.dropped, 1)

  def test_bad_magic_dropped(self):
    rx = Reassembler()
    self.assertIsNone(rx.feed(b"\x00" * 64))
    self.assertEqual(rx.dropped, 1)


class TestCodec(unittest.TestCase):
  def test_infer_inner_roundtrip(self):
    inner = encode_infer_inner(42, 123456, fake_bufs(), fake_tfms(), fake_inputs())
    d = decode_infer_inner(inner)
    self.assertEqual(d["frame_seq"], 42)
    self.assertEqual(d["client_send_ts"], 123456)
    self.assertEqual(set(d["bufs"]), {"img", "big_img"})
    self.assertTrue(np.array_equal(np.frombuffer(d["bufs"]["img"], dtype=np.uint8),
                                   np.zeros(64, dtype=np.uint8)))
    self.assertTrue(np.allclose(d["transforms"]["big_img"], np.eye(3)))
    self.assertEqual(d["inputs"]["desire_pulse"].shape, (8,))
    self.assertAlmostEqual(d["inputs"]["action_t"][1], 0.2)

  def test_resp_inner_roundtrip(self):
    out = np.linspace(-1, 1, 512, dtype=np.float32)
    inner = encode_resp_inner(5, RESP_OK, 111, 222, out.tobytes())
    d = decode_resp_inner(inner)
    self.assertEqual(d["status"], RESP_OK)
    self.assertEqual(d["server_recv_ts"], 111)
    self.assertTrue(np.allclose(d["output"], out))

  def test_resp_inner_error(self):
    inner = encode_resp_inner(5, RESP_ERR, 111, 222, b"boom")
    d = decode_resp_inner(inner)
    self.assertEqual(d["error"], "boom")

  def test_beacon_roundtrip(self):
    b = decode_beacon(encode_beacon(8571, instance=0xDEAD))
    self.assertEqual(b["port"], 8571)
    self.assertEqual(b["instance"], 0xDEAD)

  def test_hello_meta_ack_ctrl_beat_roundtrip(self):
    self.assertEqual(decode_hello(encode_hello(1, CAM_W, CAM_H))["cam_w"], CAM_W)
    m = decode_meta(encode_meta(2, 100, 200, META))
    self.assertEqual(m["meta"]["model_sha256"], META["model_sha256"])
    self.assertEqual(m["server_recv_ts"], 100)
    self.assertEqual(decode_ack(encode_ack(MSG_META, 2))["ack_id"], 2)
    self.assertEqual(decode_ctrl(encode_ctrl(3, {"a": 1}))["obj"]["a"], 1)
    self.assertEqual(decode_beat(encode_beat(9))["seq"], 9)


class ServerThread:
  """Run UdpRemoteServer.run_forever on a thread; infer_fn echoes seq into a vector."""

  def __init__(self, port, **kwargs):
    self.port = port

    def infer_fn(req):
      return np.full(8, req["frame_seq"], dtype=np.float32)

    kwargs.setdefault("meta", META)
    kwargs.setdefault("infer_fn", infer_fn)
    kwargs.setdefault("beat_interval_s", 0.1)
    kwargs.setdefault("beat_timeout_s", 0.4)
    kwargs.setdefault("beacon_interval_s", 0.5)
    kwargs.setdefault("burst_interval_s", 0.05)
    self.server = UdpRemoteServer(port, **kwargs)
    self.thread = threading.Thread(target=self.server.run_forever, daemon=True)

  def __enter__(self):
    self.thread.start()
    time.sleep(0.1)
    return self.server

  def __exit__(self, *exc):
    self.server.stop()
    self.thread.join(timeout=3)


class TestClientServer(unittest.TestCase):
  def _client(self, port, **kw):
    kw.setdefault("beat_interval_s", 0.1)
    kw.setdefault("beat_timeout_s", 0.4)
    c = UdpRemoteClient("127.0.0.1", port, CAM_W, CAM_H, **kw)
    c.start()
    return c

  def test_handshake_and_meta(self):
    with ServerThread(PORT):
      c = self._client(PORT)
      try:
        self.assertTrue(c._session_ready.wait(timeout=3), "handshake did not complete")
        self.assertEqual(c.meta["model_sha256"], META["model_sha256"])
        self.assertGreaterEqual(c.stats()["rtt_ms"], 0)
      finally:
        c.close()

  def test_infer_pipeline_and_order(self):
    with ServerThread(PORT + 1):
      c = self._client(PORT + 1)
      try:
        self.assertTrue(c._session_ready.wait(timeout=3))
        seqs = [c.infer(fake_bufs(), fake_tfms(), fake_inputs()) for _ in range(3)]
        self.assertEqual(seqs, [1, 2, 3])
        got = {}
        deadline = time.monotonic() + 3
        while len(got) < 3 and time.monotonic() < deadline:
          r = c.poll_result()
          if r is None:
            time.sleep(0.01)
            continue
          if r["status"] == RESP_OK:
            got[r["frame_seq"]] = r["output"]
        self.assertEqual(set(got), {1, 2, 3})
        for s, out in got.items():
          self.assertTrue(np.allclose(out, np.full(8, s, dtype=np.float32)))
      finally:
        c.close()

  def test_heartbeat_death_detection(self):
    with ServerThread(PORT + 2) as srv:
      c = self._client(PORT + 2)
      try:
        self.assertTrue(c._session_ready.wait(timeout=3))
        srv.stop()          # server I/O loop exits, heartbeats cease
        deadline = time.monotonic() + 3
        while c.ready and time.monotonic() < deadline:
          time.sleep(0.05)
        self.assertFalse(c.ready, "client did not detect dead link")
        self.assertIsNone(c.infer(fake_bufs(), fake_tfms(), fake_inputs()))
      finally:
        c.close()

  def test_ctrl_roundtrip(self):
    with ServerThread(PORT + 3, on_ctrl=lambda o: {"echo": o["q"]}):
      c = self._client(PORT + 3)
      try:
        self.assertTrue(c._session_ready.wait(timeout=3))
        reply = c.send_ctrl({"q": 7})
        self.assertIsNotNone(reply)
        self.assertEqual(reply["echo"], 7)
      finally:
        c.close()

  def test_busy_rejection(self):
    with ServerThread(PORT + 4):
      c1 = self._client(PORT + 4)
      self.assertTrue(c1._session_ready.wait(timeout=3))
      # raw second HELLO from a different source port while session is active
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      s.bind(("127.0.0.1", 0))
      s.settimeout(2)
      try:
        s.sendto(encode_hello(99, CAM_W, CAM_H), ("127.0.0.1", PORT + 4))
        deadline = time.monotonic() + 3
        busy = False
        while time.monotonic() < deadline and not busy:
          try:
            data, _ = s.recvfrom(65535)
          except socket.timeout:
            break
          if HDR.unpack_from(data, 0)[:2] == (MAGIC, MSG_META):
            m = decode_meta(data)
            if m["ack_id"] == 99:
              busy = bool(m["meta"].get("busy"))
              break
        self.assertTrue(busy, "second HELLO was not rejected with busy META")
      finally:
        s.close()
        c1.close()

  def test_beacon_burst_discovery(self):
    with ServerThread(PORT + 5, beacon_target="127.0.0.1") as srv:
      beacons = listen_beacons(PORT + 5, wait_s=1.0)
      instances = {b["instance"] for b in beacons}
      self.assertIn(srv.instance, instances)


if __name__ == "__main__":
  unittest.main()
