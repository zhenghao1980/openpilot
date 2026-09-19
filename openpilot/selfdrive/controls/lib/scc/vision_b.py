"""Vision-B curvature estimator (dragonpilot VisionTurnController method).

Fits a cubic polynomial to the lane-line center path and evaluates geometric
curvature over a 20-150 m window. Slower than vision_a but carries an explicit
quality confidence (lane probability modulated by lane width and line std),
and provides the curve-entry distance needed for overshoot protection.

Lane-line messages are not present in the driving model of recent releases;
in that case every estimate reports zero confidence and the arbiter falls
back to vision_a.
"""
import numpy as np

from openpilot.selfdrive.controls.lib.scc.constants import EVAL_RANGE, EVAL_STEP, EVAL_START, MIN_LANE_PROB

# lane width confidence discount: full confidence for plausible widths
# (3.5-4.5 m), ramping to zero for implausibly narrow (<3 m) or wide (>5 m)
_WIDTH_BP = (3.0, 3.5, 4.5, 5.0)
_WIDTH_FP = (0.0, 1.0, 1.0, 0.0)
# line stds above ~0.3 m mean the fit is garbage
_STD_OK = (0.15, 0.3)


def _curvature_of_cubic(poly, x_vals):
  """Geometric curvature of cubic y=poly(x) (see Wikipedia: Curvature, local expressions)."""
  def kappa(x):
    dy = 3 * poly[0] * x ** 2 + 2 * poly[1] * x + poly[2]
    ddy = abs(2 * poly[1] + 6 * poly[0] * x)
    return ddy / (1. + dy ** 2) ** 1.5
  return np.array([kappa(x) for x in x_vals])


class VisionBEstimator:
  def __init__(self):
    self.confidence = 0.   # lane quality gate result, [0, 1]
    self.max_pred_curvature = 0.   # 1/m over the eval window
    self.path_poly = None
    self.overshoot = False
    self.overshoot_distance = 0.   # m to first point where lat-acc would exceed the limit
    self.overshoot_speed = 0.      # m/s required at that point

  def update(self, model_v2, v_ego: float, a_lat_reg_max: float) -> None:
    self.path_poly = None
    self.confidence = 0.
    self.max_pred_curvature = 0.
    self.overshoot = False

    lane_lines = model_v2.laneLines
    if len(lane_lines) != 4 or len(lane_lines[1].t) == 0 or len(lane_lines[2].t) == 0 or \
       len(model_v2.laneLineProbs) < 4 or len(model_v2.laneLineStds) < 4:
      return

    # -- lane quality gating (DP): probability, width plausibility, line std ----
    ll_x = np.array(lane_lines[1].x)
    lll_y = np.array(lane_lines[1].y)
    rll_y = np.array(lane_lines[2].y)
    l_prob = float(model_v2.laneLineProbs[1])
    r_prob = float(model_v2.laneLineProbs[2])
    l_std = float(model_v2.laneLineStds[1])
    r_std = float(model_v2.laneLineStds[2])

    width_pts = rll_y - lll_y
    width_mods = []
    for t_check in (0.0, 1.5, 3.0):
      if len(ll_x):
        width_at_t = np.interp(t_check * (v_ego + 7.), ll_x, width_pts)
        width_mods.append(np.interp(width_at_t, _WIDTH_BP, _WIDTH_FP))
    width_mod = min(width_mods) if width_mods else 0.
    l_prob *= width_mod * np.interp(l_std, _STD_OK, [1.0, 0.0])
    r_prob *= width_mod * np.interp(r_std, _STD_OK, [1.0, 0.0])

    self.confidence = min(l_prob, r_prob)
    if self.confidence < MIN_LANE_PROB:
      self.confidence = 0.
      return

    # -- fit center path cubic --------------------------------------------------
    center_y = width_pts / 2. + lll_y
    # N-03/R-02: a degenerate lane-line x vector (corrupted frame: all-equal or
    # too few distinct points) makes np.polyfit raise LinAlgError (killing
    # plannerd) or return garbage coefficients. Refuse the fit instead.
    if len(ll_x) < 4 or np.unique(ll_x).size < 4:
      self.confidence = 0.
      return
    try:
      self.path_poly = np.polyfit(ll_x, center_y, 3)
    except np.linalg.LinAlgError:
      self.confidence = 0.
      return
    if not np.all(np.isfinite(self.path_poly)):
      self.confidence = 0.
      return

    pred_curvatures = _curvature_of_cubic(self.path_poly, EVAL_RANGE)
    self.max_pred_curvature = float(np.amax(pred_curvatures))

    # -- overshoot protection (DP): where does current speed exceed the limit? --
    max_curvature_for_vego = a_lat_reg_max / max(v_ego, 0.1) ** 2
    overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    if len(overshoot_idxs) > 0:
      self.overshoot = True
      # first eval point past the limit; EVAL_RANGE starts at EVAL_START
      self.overshoot_distance = float(overshoot_idxs[0]) * EVAL_STEP + EVAL_START
      if self.max_pred_curvature > 0:
        self.overshoot_speed = float(np.sqrt(a_lat_reg_max / self.max_pred_curvature))
