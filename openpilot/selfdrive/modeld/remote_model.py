"""remote_model.py — compatibility shim for modeld.py's v1 anchor.

v2 replaces the synchronous TCP RemoteModelState with FusionModelState
(adaptive-rate spectrum fusion).  The import line and construction site in
modeld.py are unchanged (§7.2 anchor discipline).

Exports:
  RemoteModelState      -> aliases FusionModelState
  remote_model_metadata -> UDP handshake probe (returns meta or None)
  RemoteModelError      -> kept for modeld.py except-clause compatibility

The v1 TCP code is preserved in git history at commit b02fecd.
"""
from __future__ import annotations

import os

from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient
from openpilot.selfdrive.modeld.fusion_model import FusionModelState


class RemoteModelError(RuntimeError):
  pass


def remote_model_metadata(cam_w: int, cam_h: int) -> dict | None:
  """UDP probe for a reachable remote big model.

  Mirrors chestnut_present() + chestnut_compiled(): any failure means the big
  model is simply unavailable and modeld proceeds with the small model.
  """
  host = os.environ.get("REMOTE_MODEL_HOST") or None
  if host is None:
    return None
  try:
    client = UdpRemoteClient(host, int(os.environ.get("REMOTE_MODEL_PORT", "8571")),
                             cam_w, cam_h, beat_interval_s=0.5, beat_timeout_s=1.5)
    client.start()
    if not client._session_ready.wait(timeout=5):
      client.close()
      return None
    meta = client.meta
    client.close()
    return meta
  except Exception:
    return None


# Backward-compatible name: modeld.py constructs `RemoteModelState(cam_w, cam_h, meta)`
RemoteModelState = FusionModelState
