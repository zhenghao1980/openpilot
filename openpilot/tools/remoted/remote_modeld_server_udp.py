#!/usr/bin/env python3
"""UDP remote big-model server (v2 protocol).

Evolves remote_modeld_server.py (v1, synchronous TCP) onto the v2 UDP wire
protocol in openpilot.selfdrive.modeld.remote_udp: fire-and-forget data
frames with CRC, 500ms bidirectional heartbeats, UDP broadcast beacons for
zero-config discovery, I/O thread + single inference worker with a bounded
drop-oldest queue. Model loading and run_raw are reused unchanged from the
v1 server (they are the verified chestnut-mirror inference path).

The big model is pre-loaded before the server enters READY (starts
beaconing): §4.3 self-check = "model loads, GPU works, port binds".

Usage (on the NVIDIA host, inside an openpilot checkout):
  python openpilot/tools/remoted/remote_modeld_server_udp.py \
    --model /tmp/big_driving_tinygrad.pkl
"""
import argparse
import hashlib
import json
import os

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import modeld_pkl_path
from openpilot.selfdrive.modeld.remote_udp import UdpRemoteServer, DEFAULT_PORT
from openpilot.tools.remoted.remote_modeld_server import ModelCache, _Buf, meta_payload, run_raw


def sha256_of(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
      h.update(chunk)
  return h.hexdigest()


def main() -> None:
  parser = argparse.ArgumentParser(description="Serve the big driving model over UDP to a remote C3X (v2 protocol)")
  parser.add_argument("--model", default=str(modeld_pkl_path(True)),
                      help="compiled big-model pickle (must be compiled for the NVIDIA device)")
  parser.add_argument("--port", type=int, default=int(os.environ.get("REMOTE_MODEL_PORT", DEFAULT_PORT)))
  parser.add_argument("--cam", default="1928x1208", help="pre-warm camera resolution (only resolution served)")
  parser.add_argument("--beacon-target", default="255.255.255.255",
                      help="broadcast address for READY beacons (set to the direct-link subnet broadcast if needed)")
  parser.add_argument("--queue-depth", type=int, default=3, help="bounded inference queue; full = drop oldest")
  args = parser.parse_args()

  cam_w, cam_h = map(int, args.cam.split("x"))
  cache = ModelCache(args.model)

  # self-check before READY: the model must load and warm up here, not in the
  # I/O loop, so HELLO/META stays fast and beacons never advertise a dud
  cloudlog.warning(f"pre-loading big model for {cam_w}x{cam_h} from {args.model}")
  cache.get(cam_w, cam_h)
  model_sha = sha256_of(args.model)
  cloudlog.warning("big model ready, entering READY (beaconing)")

  def meta_fn() -> dict:
    # fresh session: reset temporal state (mirrors chestnut reconnect semantics)
    model = cache.get(cam_w, cam_h)
    meta = json.loads(meta_payload(model))
    meta["model_sha256"] = model_sha
    return meta

  def infer_fn(req: dict):
    model = cache.get(cam_w, cam_h)
    return run_raw(model, {k: _Buf(v) for k, v in req["bufs"].items()},
                   req["transforms"], req["inputs"])

  def on_session(up: bool) -> None:
    cloudlog.warning(f"session {'up' if up else 'down'}")

  srv = UdpRemoteServer(args.port, meta_fn, infer_fn,
                        beacon_target=args.beacon_target, queue_depth=args.queue_depth,
                        on_session=on_session)
  cloudlog.warning(f"remote_modeld_server_udp listening on :{args.port}, beacon -> {args.beacon_target}")
  srv.run_forever()


if __name__ == "__main__":
  main()
