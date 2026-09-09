#!/usr/bin/env python3
"""Serve the big driving model to a network-connected C3X.

Device side runs a patched modeld with openpilot.selfdrive.modeld.remote_model,
which replaces the chestnut co-processor with this host. One TCP connection is
one driving session: temporal state (hidden features) resets on reconnect,
mirroring chestnut semantics.

Usage (on the NVIDIA host, inside an openpilot checkout):
  # 1. compile the big model for CUDA
  python openpilot/selfdrive/modeld/compile_modeld.py \
    --model-size 256x128 --camera-resolutions 1928x1208 \
    --onnx openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx \
    --output /tmp/big_driving_tinygrad.pkl --frame-skip 4
  # 2. serve
  python openpilot/tools/remoted/remote_modeld_server.py --model /tmp/big_driving_tinygrad.pkl

Device side: set REMOTE_MODEL_HOST (and optionally REMOTE_MODEL_PORT,
REMOTE_MODEL_TIMEOUT_MS) before launching openpilot.
"""
import argparse
import json
import socket
import struct
import threading

import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.compile_modeld import MODELD_INPUTS
from openpilot.selfdrive.modeld.helpers import load_oob, modeld_pkl_path
from openpilot.selfdrive.modeld.modeld import ModelState
from openpilot.selfdrive.modeld.remote_model import (
  MAGIC, MSG_HELLO, MSG_INFER, RESP_OK, RESP_ERR,
  _HEADER, _HELLO, _INFER_HDR, _NAME, _LEN, _TFM_LEN, _RESP_HDR,
  _N_F32, _2_F32, _recv_exact, remote_port,
)

DESIRE_LEN = 8


class _Buf:
  """VisionBuf-compatible shim: ModelState only reads .data."""
  def __init__(self, data: bytes):
    self.data = data


def run_raw(model: ModelState, bufs: dict, transforms: dict, inputs: dict) -> np.ndarray:
  """Mirror of ModelState.run up to the raw output vector. Keep in sync with
  openpilot/selfdrive/modeld/modeld.py (ModelState.run)."""
  for key, buf in bufs.items():
    np.copyto(model.frame_views[key], np.frombuffer(buf.data, dtype=np.uint8, count=model.frame_copy_size))

  # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
  inputs['desire_pulse'][0] = 0
  model.npy['desire'][:] = np.where(inputs['desire_pulse'] - model.prev_desire > .99, inputs['desire_pulse'], 0)
  model.prev_desire[:] = inputs['desire_pulse']
  model.npy['traffic_convention'][:] = inputs['traffic_convention']
  model.npy['action_t'][:] = inputs['action_t']
  model.npy['tfm'][:, :] = transforms['img'][:, :]
  model.npy['big_tfm'][:, :] = transforms['big_img'][:, :]

  outs, = model.run_model(**{k: model.input_queues[k] for k in MODELD_INPUTS})
  model_output = outs.numpy()[0]
  if not np.all(np.isfinite(model_output)):
    raise RuntimeError("model output not finite")
  model.npy['prev_feat'][:] = model_output[model.output_slices['hidden_state']]
  return model_output


def meta_payload(model: ModelState) -> bytes:
  return json.dumps({
    "input_shapes": {k: list(v) for k, v in model.input_shapes.items()},
    "output_slices": {k: [v.start, v.stop] for k, v in model.output_slices.items()},
    "vision_input_names": model.vision_input_names,
  }, default=int).encode()


def _load_model(cam_w: int, cam_h: int, model_path: str) -> ModelState:
  """Load a compiled big-model pickle, mirroring ModelState.__init__ but with an
  explicit pickle path (ModelState hardcodes the built-in chestnut path)."""
  from openpilot.common.file_chunker import open_file_chunked
  from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, nv12_copy_size
  from openpilot.selfdrive.modeld.constants import ModelConstants
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

  model = ModelState.__new__(ModelState)
  jits = load_oob(open_file_chunked(model_path))
  input_devices = jits['input_devices']
  model.model_device = input_devices['model']
  metadata = jits['metadata']
  model.input_shapes = metadata['input_shapes']
  model.vision_input_names = [k for k in model.input_shapes if 'img' in k]
  model.output_slices = metadata['output_slices']
  model.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
  model.chestnut = True
  model.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
  model.frame_copy_size = nv12_copy_size(*get_nv12_info(cam_w, cam_h)[:3])
  model.input_queues, model.npy, model.frame_views = make_input_queues(
    model.input_shapes, model.frame_skip, device=model.model_device, frame_copy_size=model.frame_copy_size)
  model.parser = Parser()
  model.run_model = jits['run_model'][(cam_w, cam_h)]
  model.warmup()
  return model


class ModelCache:
  def __init__(self, model_path: str):
    self.model_path = model_path
    self.lock = threading.Lock()
    self.models: dict[tuple[int, int], ModelState] = {}

  def get(self, cam_w: int, cam_h: int) -> ModelState:
    with self.lock:
      key = (cam_w, cam_h)
      if key not in self.models:
        cloudlog.warning(f"loading big model for {cam_w}x{cam_h} from {self.model_path}")
        self.models[key] = _load_model(cam_w, cam_h, self.model_path)
        cloudlog.warning(f"big model ready for {cam_w}x{cam_h}")
      else:
        # fresh session: reset temporal state (fresh input queues, prev_desire=0)
        self.models[key].warmup()
      return self.models[key]


def handle_conn(conn: socket.socket, cache: ModelCache) -> None:
  with conn:
    try:
      magic, mtype = _HEADER.unpack(_recv_exact(conn, _HEADER.size))
      if magic != MAGIC or mtype != MSG_HELLO:
        raise RuntimeError("expected HELLO")
      cam_w, cam_h = _HELLO.unpack(_recv_exact(conn, _HELLO.size))

      model = cache.get(cam_w, cam_h)
      meta = meta_payload(model)
      conn.sendall(_RESP_HDR.pack(MAGIC, RESP_OK, len(meta)) + meta)
      cloudlog.warning(f"session started: cam {cam_w}x{cam_h}")

      while True:
        hdr = _recv_exact(conn, _HEADER.size)
        magic, mtype = _HEADER.unpack(hdr)
        if magic != MAGIC or mtype != MSG_INFER:
          break
        frame_id, n_bufs = _INFER_HDR.unpack(_recv_exact(conn, _INFER_HDR.size))
        bufs = {}
        for _ in range(n_bufs):
          name_len = _NAME.unpack(_recv_exact(conn, _NAME.size))[0]
          name = _recv_exact(conn, name_len).decode()
          data_len = _LEN.unpack(_recv_exact(conn, _LEN.size))[0]
          bufs[name] = _recv_exact(conn, data_len)

        n_tfms = _TFM_LEN.unpack(_recv_exact(conn, _TFM_LEN.size))[0]
        transforms = {}
        for _ in range(n_tfms):
          name_len = _NAME.unpack(_recv_exact(conn, _NAME.size))[0]
          name = _recv_exact(conn, name_len).decode()
          transforms[name] = np.frombuffer(_recv_exact(conn, 9 * 4), dtype=np.float32).reshape(3, 3).copy()

        inputs = {
          'desire_pulse': np.array(_N_F32.unpack(_recv_exact(conn, _N_F32.size)), dtype=np.float32),
          'traffic_convention': np.array(_2_F32.unpack(_recv_exact(conn, _2_F32.size)), dtype=np.float32),
          'action_t': np.array(_2_F32.unpack(_recv_exact(conn, _2_F32.size)), dtype=np.float32),
        }

        try:
          out = run_raw(model, {k: _Buf(v) for k, v in bufs.items()}, transforms, inputs)
          payload = np.ascontiguousarray(out, dtype=np.float32).tobytes()
          conn.sendall(_RESP_HDR.pack(MAGIC, RESP_OK, len(payload)) + payload)
        except Exception as e:
          cloudlog.exception("inference failed")
          msg = str(e).encode()[:1024]
          conn.sendall(_RESP_HDR.pack(MAGIC, RESP_ERR, len(msg)) + msg)
    except (ConnectionError, BrokenPipeError):
      pass
    except Exception:
      cloudlog.exception("session error")
    finally:
      cloudlog.warning("session ended")


def main() -> None:
  parser = argparse.ArgumentParser(description="Serve the big driving model over TCP to a remote C3X")
  parser.add_argument('--model', default=str(modeld_pkl_path(True)),
                      help='compiled big-model pickle (must be compiled for the NVIDIA device)')
  parser.add_argument('--port', type=int, default=None, help='default: REMOTE_MODEL_PORT or 8571')
  args = parser.parse_args()

  port = args.port if args.port is not None else remote_port()
  cache = ModelCache(args.model)

  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind(("0.0.0.0", port))
  srv.listen(1)
  cloudlog.warning(f"remote_modeld_server listening on :{port}, model={args.model}")

  while True:
    conn, addr = srv.accept()
    cloudlog.warning(f"connection from {addr}")
    handle_conn(conn, cache)


if __name__ == "__main__":
  main()
