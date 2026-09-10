"""FusionModelState: adaptive-rate spectrum fusion between on-device small
model (20 Hz, C3X) and remote big model (async UDP, 0-20 Hz).

Implements the v2 architecture from c3x_remote_bigmodel_design.md §3:
  - small model is always on → pose/cameraOdometry never stale
  - big model async fire-and-forget → results arrive with measured staleness
  - fusion happens in parsed semantic space (Parser double-instantiation)
  - plan: time-splice (t<τ small, t>τ big, transition band)
  - lanes/edges: distance-splice (near small, far big)
  - lead / meta / desire: big-priority
  - cold-start ramp w: 0→target over 2 s
  - exposed interface isomorphic to ModelState → modeld.py anchor unchanged
"""
from __future__ import annotations

import os
import time

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient, RESP_OK

T_IDXS = ModelConstants.T_IDXS
X_IDXS = ModelConstants.X_IDXS
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.remote_udp import UdpRemoteClient, RESP_OK


def _sigmoid(x: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-x))


class FusionModelState:
  """Drop-in replacement for ModelState / RemoteModelState in modeld.py.

  Construction (called from modeld.py anchor):
    model = FusionModelState(cam_w, cam_h, meta)   # meta from remote_model_metadata()
  """

  # tunables (centralised, see §9.11-13 of design doc)
  RAMP_S: float = 2.0               # cold-start w ramp duration
  TAU0_MS: float = 150.0            # staleness decay constant for w
  PLAN_TRANSITION_S: float = 0.4    # plan splice transition half-width
  LANE_DISTANCE_THRESHOLD_M: float = 20.0  # near/far split for lanes

  def __init__(self, cam_w: int, cam_h: int, meta: dict):
    self.chestnut = True            # upstream flags: we act like chestnut
    self.cam_w, self.cam_h = cam_w, cam_h

    # small model created lazily in warmup() to avoid circular import
    # (modeld.py -> remote_model -> fusion_model -> modeld.ModelState)
    self._small_model = None

    # UDP big-model link
    host = os.environ.get("REMOTE_MODEL_HOST", "")
    port = int(os.environ.get("REMOTE_MODEL_PORT", "8571"))
    self.udp_client = UdpRemoteClient(host, port, cam_w, cam_h)

    # parser for big outputs (small model has its own inside ModelState)
    self.parser = Parser()
    self.big_output_slices = {k: slice(a, b) for k, (a, b) in meta["output_slices"].items()}
    self.input_shapes = {k: tuple(v) for k, v in meta["input_shapes"].items()}
    self.vision_input_names = [k for k in self.input_shapes if "img" in k]

    # fusion runtime state
    self._session_start_ts: float | None = None
    self._last_big_parsed: dict | None = None
    self._last_big_staleness_ms: float = 0.0

  # -- ModelState-compatible interface ---------------------------------------

  def warmup(self) -> None:
    # lazy import: modeld.py is the importer, so ModelState is available at call time
    from openpilot.selfdrive.modeld.modeld import ModelState
    self._small_model = ModelState(self.cam_w, self.cam_h, False)
    self._small_model.warmup()
    self.udp_client.start()
    if not self.udp_client._session_ready.wait(timeout=10):
      raise RuntimeError("FusionModelState: UDP handshake timeout")
    self._session_start_ts = time.monotonic()

  def run(self, bufs: dict, transforms: dict, inputs: dict, after_enqueue=None) -> dict:
    """One modeld iteration: small model sync + async big request + fuse."""
    if self._small_model is None:
      raise RuntimeError("FusionModelState: warmup() not called")

    # 1) small model first — pose/odometry must never wait
    small_outputs = self._small_model.run(bufs, transforms, inputs, after_enqueue=after_enqueue)

    # 2) fire async big model request (non-blocking)
    if self.udp_client.ready:
      self.udp_client.infer(bufs, transforms, inputs)

    # 3) drain result queue, keep only the newest (discard stale intermediates)
    newest: dict | None = None
    while True:
      r = self.udp_client.poll_result()
      if r is None:
        break
      newest = r

    if newest is not None and newest["status"] == RESP_OK:
      raw = newest["output"]
      big_parsed = self.parser.parse_outputs(
        {k: raw[np.newaxis, v] for k, v in self.big_output_slices.items()}
      )
      stats = self.udp_client.stats()
      self._last_big_staleness_ms = max(0.0, stats.get("rtt_ms", 0.0) / 2.0)
      self._last_big_parsed = big_parsed

    # 4) compute fusion weight w ∈ [0,1]
    w = self._compute_weight()
    if w > 0.0 and self._last_big_parsed is not None:
      return self._fuse(small_outputs, self._last_big_parsed, w)
    return small_outputs

  # -- weight ----------------------------------------------------------------

  def _compute_weight(self) -> float:
    if self._session_start_ts is None:
      return 0.0
    ramp = min(1.0, (time.monotonic() - self._session_start_ts) / self.RAMP_S)
    w_staleness = np.exp(-self._last_big_staleness_ms / self.TAU0_MS)
    return float(w_staleness * ramp)

  # -- fusion ----------------------------------------------------------------

  def _fuse(self, small: dict, big: dict, w: float) -> dict:
    """Semantic-space fusion. `small` and `big` are Parser output dicts."""
    fused = dict(small)

    if "plan" in big and "plan" in small:
      fused["plan"] = self._blend_plan(small["plan"], big["plan"], w)
      if "plan_stds" in big and "plan_stds" in small:
        fused["plan_stds"] = self._blend_plan(small["plan_stds"], big["plan_stds"], w)

    if "lane_lines" in big and "lane_lines" in small:
      fused["lane_lines"] = self._blend_by_distance(small["lane_lines"], big["lane_lines"], w)
      if "lane_lines_stds" in big and "lane_lines_stds" in small:
        fused["lane_lines_stds"] = self._blend_by_distance(small["lane_lines_stds"], big["lane_lines_stds"], w)
    if "road_edges" in big and "road_edges" in small:
      fused["road_edges"] = self._blend_by_distance(small["road_edges"], big["road_edges"], w)
      if "road_edges_stds" in big and "road_edges_stds" in small:
        fused["road_edges_stds"] = self._blend_by_distance(small["road_edges_stds"], big["road_edges_stds"], w)

    if "lead" in big:
      fused["lead"] = big["lead"]
      for suffix in ("_weights", "_hypotheses", "_stds_hypotheses", "_stds"):
        k = f"lead{suffix}"
        if k in big:
          fused[k] = big[k]
      if "lead_prob" in big:
        fused["lead_prob"] = big["lead_prob"]

    if "meta" in big:
      fused["meta"] = big["meta"]
    for k in ("desire_pred", "desire_state"):
      if k in big:
        fused[k] = big[k]

    return fused

  def _blend_plan(self, small_plan: np.ndarray, big_plan: np.ndarray, w: float) -> np.ndarray:
    """Time-splice plan.  Shape (1, IDX_N, PLAN_WIDTH=15).
    small for t<τ-δ, big for t>τ+δ, smooth sigmoid in between."""
    result = small_plan.copy()
    tau_s = self._last_big_staleness_ms / 1000.0
    half_trans = self.PLAN_TRANSITION_S / 2.0
    t_arr = np.array(T_IDXS, dtype=np.float32)
    alpha = _sigmoid((t_arr - tau_s) / max(half_trans, 0.05))
    alpha = alpha * w
    alpha = alpha.reshape(1, -1, 1)
    result = (1.0 - alpha) * small_plan + alpha * big_plan
    return result

  def _blend_by_distance(self, small_arr: np.ndarray, big_arr: np.ndarray, w: float) -> np.ndarray:
    """Distance-splice for lanes/edges.  Shape (1, N, IDX_N, 2).
    Hard threshold at LANE_DISTANCE_THRESHOLD_M; blended by w on far side."""
    result = small_arr.copy()
    idx = next((i for i, x in enumerate(X_IDXS) if x >= self.LANE_DISTANCE_THRESHOLD_M), len(X_IDXS))
    if idx < len(X_IDXS):
      result[:, :, idx:, :] = (1.0 - w) * small_arr[:, :, idx:, :] + w * big_arr[:, :, idx:, :]
    return result
