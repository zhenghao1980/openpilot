#!/usr/bin/env python3
"""Offline protocol tests for the remote big-model link (no openpilot deps).

Runs against a fake in-process server that speaks the same wire protocol as
openpilot/tools/remoted/remote_modeld_server.py, using the framing structs from
the module under test. openpilot's Parser is stubbed out, so this test only
needs numpy.

Covers: HELLO/META, INFER round-trip, error response, mid-session disconnect,
request timeout, non-finite output, availability probe, after_send callback.
"""
import json
import os
import socket
import struct
import sys
import threading
import time
import types
import unittest

import numpy as np

# ---- stub openpilot Parser so remote_model imports without the repo built ----
def _install_parser_stub():
  for name in ("openpilot", "openpilot.selfdrive", "openpilot.selfdrive.modeld",
               "openpilot.selfdrive.modeld.parse_model_outputs"):
    if name not in sys.modules:
      mod = types.ModuleType(name)
      mod.__path__ = []
      sys.modules[name] = mod

  class FakeParser:
    def parse_outputs(self, outs):
      return outs  # echo sliced outputs; protocol test does not validate parsing

  sys.modules["openpilot.selfdrive.modeld.parse_model_outputs"].Parser = FakeParser


HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.join(HERE, "..", "..", "..", "selfdrive", "modeld"),
              os.path.join(HERE, "..", "..", "selfdrive", "modeld"),
              HERE):
  if os.path.isfile(os.path.join(_cand, "remote_model.py")):
    sys.path.insert(0, _cand)
    break
_install_parser_stub()

import remote_model as rm  # noqa: E402

META = {
  "input_shapes": {"img": [1, 12, 128, 256], "big_img": [1, 12, 128, 256], "features_buffer": [1, 4, 512]},
  "output_slices": {"plan": [0, 495], "meta": [495, 500], "hidden_state": [500, 1012]},
  "vision_input_names": ["img", "big_img"],
}
OUT_LEN = max(v[1] for v in META["output_slices"].values())


class _Buf:
  def __init__(self, data: bytes):
    self.data = data


def _recv_exact(sock, n):
  return rm._recv_exact(sock, n)


class FakeServer(threading.Thread):
  """Speaks the remote-model wire protocol with an injectable fault mode.

  modes: ok | err | drop | hang | nonfinite
  """

  def __init__(self, mode="ok"):
    super().__init__(daemon=True)
    self.mode = mode
    self.received = {}
    self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.srv.bind(("127.0.0.1", 0))
    self.srv.listen(1)
    self.srv.settimeout(0.5)
    self.port = self.srv.getsockname()[1]
    self.started = threading.Event()
    self._stop = False

  def run(self):
    self.started.set()
    while not self._stop:
      try:
        conn, _ = self.srv.accept()
      except socket.timeout:
        continue
      except OSError:
        break
      try:
        self._handle(conn)
      except Exception:
        pass  # client went away mid-session
      finally:
        try:
          conn.close()
        except OSError:
          pass

  def _handle(self, conn):
    magic, mtype = rm._HEADER.unpack(_recv_exact(conn, rm._HEADER.size))
    assert magic == rm.MAGIC and mtype == rm.MSG_HELLO
    _recv_exact(conn, rm._HELLO.size)  # cam dims
    meta = json.dumps(META).encode()
    conn.sendall(rm._RESP_HDR.pack(rm.MAGIC, rm.RESP_OK, len(meta)) + meta)

    magic, mtype = rm._HEADER.unpack(_recv_exact(conn, rm._HEADER.size))
    assert magic == rm.MAGIC and mtype == rm.MSG_INFER
    frame_id, n_bufs = rm._INFER_HDR.unpack(_recv_exact(conn, rm._INFER_HDR.size))
    bufs = {}
    for _ in range(n_bufs):
      name_len = rm._NAME.unpack(_recv_exact(conn, rm._NAME.size))[0]
      name = _recv_exact(conn, name_len).decode()
      data_len = rm._LEN.unpack(_recv_exact(conn, rm._LEN.size))[0]
      bufs[name] = _recv_exact(conn, data_len)
    n_tfms = rm._TFM_LEN.unpack(_recv_exact(conn, rm._TFM_LEN.size))[0]
    tfms = {}
    for _ in range(n_tfms):
      name_len = rm._NAME.unpack(_recv_exact(conn, rm._NAME.size))[0]
      name = _recv_exact(conn, name_len).decode()
      tfms[name] = np.frombuffer(_recv_exact(conn, 36), dtype=np.float32).reshape(3, 3)
    desire = rm._N_F32.unpack(_recv_exact(conn, rm._N_F32.size))
    traffic = rm._2_F32.unpack(_recv_exact(conn, rm._2_F32.size))
    action_t = rm._2_F32.unpack(_recv_exact(conn, rm._2_F32.size))
    self.received = {"frame_id": frame_id, "bufs": bufs, "tfms": tfms,
                     "desire": desire, "traffic": traffic, "action_t": action_t}

    if self.mode == "hang":
      time.sleep(30)
      return
    if self.mode == "drop":
      return  # close without responding
    if self.mode == "err":
      msg = b"inference exploded"
      conn.sendall(rm._RESP_HDR.pack(rm.MAGIC, rm.RESP_ERR, len(msg)) + msg)
      return

    out = np.full(OUT_LEN, 0.5, dtype=np.float32)
    if self.mode == "nonfinite":
      out[0] = np.nan
    payload = out.tobytes()
    conn.sendall(rm._RESP_HDR.pack(rm.MAGIC, rm.RESP_OK, len(payload)) + payload)

  def stop(self):
    self._stop = True
    try:
      self.srv.close()
    except OSError:
      pass


class RemoteModelProtocolTest(unittest.TestCase):
  def setUp(self):
    self.server = FakeServer()
    self.server.start()
    self.server.started.wait(1)
    os.environ["REMOTE_MODEL_HOST"] = "127.0.0.1"
    os.environ["REMOTE_MODEL_PORT"] = str(self.server.port)
    os.environ["REMOTE_MODEL_TIMEOUT_MS"] = "500"

  def tearDown(self):
    os.environ.pop("REMOTE_MODEL_HOST", None)
    os.environ.pop("REMOTE_MODEL_PORT", None)
    os.environ.pop("REMOTE_MODEL_TIMEOUT_MS", None)
    self.server.stop()

  def _inputs(self):
    return {
      "desire_pulse": np.zeros(8, dtype=np.float32),
      "traffic_convention": np.array([1, 0], dtype=np.float32),
      "action_t": np.array([0.5, 0.7], dtype=np.float32),
    }

  def _bufs(self):
    return {"img": _Buf(bytes(range(256)) * 4), "big_img": _Buf(bytes(range(256)) * 4)}

  def _tfms(self):
    return {"img": np.eye(3, dtype=np.float32), "big_img": np.eye(3, dtype=np.float32)}

  def test_hello_meta_and_infer_roundtrip(self):
    state = rm.RemoteModelState(1928, 1208, rm.remote_model_metadata(1928, 1208))
    self.assertTrue(state.chestnut)
    self.assertEqual(state.vision_input_names, ["img", "big_img"])

    called = []
    out = state.run(self._bufs(), self._tfms(), self._inputs(), after_enqueue=lambda: called.append(1))
    self.assertEqual(called, [1])  # after_enqueue runs while the host computes
    self.assertEqual(set(out.keys()), {"plan", "meta", "hidden_state"})
    self.assertEqual(out["plan"].shape, (1, 495))
    self.assertEqual(out["hidden_state"].shape, (1, 512))

    self.server.join(2)
    rx = self.server.received
    self.assertEqual(set(rx["bufs"].keys()), {"img", "big_img"})
    self.assertTrue(np.array_equal(rx["tfms"]["img"], np.eye(3, dtype=np.float32)))
    self.assertEqual(rx["traffic"], (1.0, 0.0))
    np.testing.assert_allclose(rx["action_t"], (0.5, 0.7), rtol=1e-6)

  def test_error_response_raises(self):
    self.server.mode = "err"
    state = rm.RemoteModelState(1928, 1208, META)
    with self.assertRaises(rm.RemoteModelError) as ctx:
      state.run(self._bufs(), self._tfms(), self._inputs())
    self.assertIn("inference exploded", str(ctx.exception))

  def test_disconnect_raises(self):
    self.server.mode = "drop"
    state = rm.RemoteModelState(1928, 1208, META)
    with self.assertRaises((rm.RemoteModelError, ConnectionError)):
      state.run(self._bufs(), self._tfms(), self._inputs())

  def test_timeout_raises(self):
    self.server.mode = "hang"
    os.environ["REMOTE_MODEL_TIMEOUT_MS"] = "200"
    state = rm.RemoteModelState(1928, 1208, META)
    t0 = time.monotonic()
    with self.assertRaises(rm.RemoteModelError):
      state.run(self._bufs(), self._tfms(), self._inputs())
    self.assertLess(time.monotonic() - t0, 5)

  def test_nonfinite_output_raises(self):
    self.server.mode = "nonfinite"
    state = rm.RemoteModelState(1928, 1208, META)
    with self.assertRaises(RuntimeError):
      state.run(self._bufs(), self._tfms(), self._inputs())

  def test_metadata_probe_unreachable_returns_none(self):
    os.environ["REMOTE_MODEL_PORT"] = "1"  # nothing listening
    self.assertIsNone(rm.remote_model_metadata(1928, 1208))

  def test_metadata_probe_unset_host_returns_none(self):
    os.environ.pop("REMOTE_MODEL_HOST", None)
    self.assertIsNone(rm.remote_model_metadata(1928, 1208))

  def test_timeout_ms_parsing(self):
    os.environ["REMOTE_MODEL_TIMEOUT_MS"] = "250"
    self.assertAlmostEqual(rm.remote_timeout(), 0.25)


if __name__ == "__main__":
  unittest.main()
