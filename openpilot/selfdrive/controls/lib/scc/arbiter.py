"""Confidence-gated arbitration with frame hysteresis.

Replaces the bare per-frame min() used by sunnypilot: sources enter the race
only after passing their confidence gates, and both adoption of a lower speed
and release back to cruise must persist for a few frames before it takes
effect (kills single-frame model hallucinations and map data jumps).
"""
from openpilot.selfdrive.controls.lib.scc.constants import (
  CONF_VISION_B_GATE, CURVE_MIN_SPEED, HYSTERESIS_DOWN_FRAMES, HYSTERESIS_EPS, HYSTERESIS_UP_FRAMES,
)


class SccArbiter:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.v_raw = 0.          # winning target this frame, before hysteresis
    self.source = "none"     # "vision_a" | "vision_b" | "map" | "none"
    self._down_cnt = 0       # frames a lower speed has been pending
    self._up_cnt = 0         # frames a higher speed has been pending
    self._v_adopted = 0.     # currently adopted (hysteresis-applied) speed
    self._have_target = False

  def _vision_winner(self, v_a: float, c_a: float, v_b: float, c_b: float) -> float:
    valid_a = v_a > 0.
    valid_b = v_b > 0. and c_b >= CONF_VISION_B_GATE
    if valid_b:
      self.source = "vision_b"
      if valid_a and v_a < v_b * 0.8:
        # vision_a sees something much worse; on disagreement trust the lower
        self.source = "vision_a"
        return v_a
      return v_b
    if valid_a and c_a > 0.3:
      self.source = "vision_a"
      return v_a
    return 0.

  def update(self, v_map: float, map_conf: float, map_min_conf: float,
             v_a: float, c_a: float, v_b: float, c_b: float) -> tuple[float, bool]:
    """Returns (v_target, has_target); v_target is 0 when no source is valid."""
    candidates = {}
    if v_map > 0. and map_conf >= map_min_conf:
      candidates["map"] = v_map
    v_vis = self._vision_winner(v_a, c_a, v_b, c_b)
    if v_vis > 0.:
      candidates[self.source] = v_vis

    if not candidates:
      self.source = "none"
      self.v_raw = 0.
      self._down_cnt = self._up_cnt = 0
      self._have_target = False
      self._v_adopted = 0.
      return 0., False

    self.source = min(candidates, key=candidates.get)
    self.v_raw = candidates[self.source]

    v_raw = max(self.v_raw, CURVE_MIN_SPEED)
    if not self._have_target:
      self._v_adopted, self._have_target = v_raw, True
      return self._v_adopted, True

    if v_raw < self._v_adopted - HYSTERESIS_EPS:
      self._down_cnt += 1
      self._up_cnt = 0
      if self._down_cnt >= HYSTERESIS_DOWN_FRAMES:
        self._v_adopted = v_raw
        self._down_cnt = 0
    elif v_raw > self._v_adopted + HYSTERESIS_EPS:
      self._up_cnt += 1
      self._down_cnt = 0
      if self._up_cnt >= HYSTERESIS_UP_FRAMES:
        self._v_adopted = v_raw
        self._up_cnt = 0
    else:
      self._down_cnt = self._up_cnt = 0

    return self._v_adopted, True
