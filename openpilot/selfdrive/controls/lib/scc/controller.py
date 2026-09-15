"""SCC-X unified state machine and controller.

State machine skeleton follows sunnypilot's SmartCruiseControlVision
(disabled/enabled/overriding/entering/turning/leaving, hysteresis thresholds,
accel lookup tables), with dragonpilot's overshoot solution blended into the
ENTERING state. Consumes the arbiter output; drives two planner hooks:

  out.v_cruise_cap  -> cap on the cruise target speed [m/s], or None
  out.a_target      -> minimum acceleration candidate [m/s^2], or None

The longitudinal planner applies the cap to the cruise branch and appends
a_target to the existing min()-accel arbitration, so SCC-X can only ever make
control more conservative; with the feature disabled or inactive the planner
behaves exactly as stock.
"""
import numpy as np
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.scc.arbiter import SccArbiter
from openpilot.selfdrive.controls.lib.scc.constants import (
  A_LAT_REG_MAX_BY_PERSONALITY, A_TARGET_MIN, ABORT_ENTERING_PRED_LAT_ACC_TH, CURVE_MIN_SPEED,
  ENTERING_PRED_LAT_ACC_TH, ENTERING_SMOOTH_DECEL_BP, ENTERING_SMOOTH_DECEL_V,
  FINISH_LAT_ACC_TH, LEAVING_ACC, LEAVING_LAT_ACC_TH, MIN_V, NO_OVERSHOOT_TIME_HORIZON,
  SCC_X_ENABLED_PARAM, TURNING_ACC_BP, TURNING_ACC_V, TURNING_LAT_ACC_TH,
)
from openpilot.selfdrive.controls.lib.scc.map import MapCurveEstimator
from openpilot.selfdrive.controls.lib.scc.vision_a import VisionAEstimator
from openpilot.selfdrive.controls.lib.scc.vision_b import VisionBEstimator

PARAMS_UPDATE_PERIOD = 1.0  # s
DT_MDL = 0.05               # modeld timestep [s], mirrors openpilot.common.realtime


@dataclass
class SccXOutput:
  v_cruise_cap: float | None = None   # m/s; None = no cap
  a_target: float | None = None       # m/s^2; None = no extra candidate
  active: bool = False
  state: str = "disabled"
  source: str = "none"
  debug_v_target: float = 0.          # arbiter-adopted curve speed [m/s]


class SccXController:
  def __init__(self, CP):
    self.CP = CP
    from openpilot.common.params import Params  # lazy: keeps module importable in tests
    self.params = Params()

    self.vision_a = VisionAEstimator()
    self.vision_b = VisionBEstimator()
    self.map_est = MapCurveEstimator()
    self.arbiter = SccArbiter()

    self.enabled = self.params.get_bool(SCC_X_ENABLED_PARAM)
    self.frame = -1
    self.state = "disabled"

    # Per-car decel limit: some brands clamp below the global -3.5 (VW MLB at
    # -2.95). The overshoot math must only assume braking the car will actually
    # deliver; planner clips at the global value, the car controller clamps
    # again per-car, so without this SCC-X would plan to arrive ~20% too hot
    # on MLB. Falls back to the global default if the car stack is unavailable.
    self._a_target_min = A_TARGET_MIN
    try:
      from opendbc.car.car_helpers import interfaces  # lazy: keeps module importable in tests
      if CP.carFingerprint in interfaces:
        car_min = interfaces[CP.carFingerprint].get_pid_accel_limits(CP, 0., 0.)[0]
        self._a_target_min = min(car_min, A_TARGET_MIN)
    except (ImportError, AttributeError):
      pass

    self._v_ego = 0.
    self._a_ego = 0.
    self._v_cruise = 0.
    self._current_lat_acc = 0.
    self._max_pred_lat_acc = 0.
    self._a_lat_reg_max = A_LAT_REG_MAX_BY_PERSONALITY[1]
    self._v_arb = 0.
    self._has_target = False
    self._a_target = 0.

  # -- helpers ---------------------------------------------------------------

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool(SCC_X_ENABLED_PARAM)

  def _update_estimates(self, model_v2, personality) -> None:
    # clamp defensively: an out-of-range personality must never crash plannerd
    personality_idx = min(max(int(personality), 0), len(A_LAT_REG_MAX_BY_PERSONALITY) - 1)
    self._a_lat_reg_max = A_LAT_REG_MAX_BY_PERSONALITY[personality_idx]
    self._max_pred_lat_acc = self.vision_a.update(model_v2, self._v_ego)
    self.vision_b.update(model_v2, self._v_ego, self._a_lat_reg_max)
    self.map_est.update(self._v_ego, self._a_ego)
    self._v_arb, self._has_target = self.arbiter.update(
      self.map_est.v_target, self.map_est.confidence, MapCurveEstimator.MIN_CONFIDENCE,
      self._vision_a_speed(), self.vision_a.confidence,
      self._vision_b_speed(), self.vision_b.confidence,
    )

  def _vision_a_speed(self) -> float:
    # convert predicted lat-acc back to the speed it would permit at v_ego:
    # a_pred = v^2 * kappa  ->  v_allow = v_ego * sqrt(a_max / a_pred)
    # A speed at/above the user's cruise setting means "this source imposes no
    # constraint" and must report 0, otherwise the arbiter adopts a huge
    # baseline and every real slowdown gets eaten by the down-hysteresis.
    if self._max_pred_lat_acc <= 0.:
      return 0.
    v_allow = self._v_ego * float(np.sqrt(self._a_lat_reg_max / self._max_pred_lat_acc))
    return v_allow if v_allow < self._v_cruise else 0.

  def _vision_b_speed(self) -> float:
    if self.vision_b.max_pred_curvature <= 0.:
      return 0.
    v_allow = float(np.sqrt(self._a_lat_reg_max / self.vision_b.max_pred_curvature))
    return v_allow if v_allow < self._v_cruise else 0.

  def _current_curve_speed(self) -> float:
    curvature = self._current_lat_acc / max(self._v_ego, 0.1) ** 2
    if curvature <= 0.:
      return 0.
    return float(np.sqrt(self._a_lat_reg_max / curvature))

  # -- state machine ----------------------------------------------------------

  def _update_state(self, long_enabled: bool, long_override: bool) -> None:
    if self.state != "disabled":
      # longitudinal and feature disable always have priority
      if not long_enabled or not self.enabled:
        self.state = "disabled"
      elif long_override:
        self.state = "overriding"
      elif self.state == "overriding":
        self.state = "enabled"
      elif self.state == "enabled":
        if self._v_ego > MIN_V and self._max_pred_lat_acc >= ENTERING_PRED_LAT_ACC_TH:
          self.state = "entering"
      elif self.state == "entering":
        if self._current_lat_acc >= TURNING_LAT_ACC_TH:
          self.state = "turning"
        elif self._max_pred_lat_acc < ABORT_ENTERING_PRED_LAT_ACC_TH:
          self.state = "enabled"
      elif self.state == "turning":
        if self._current_lat_acc <= LEAVING_LAT_ACC_TH:
          self.state = "leaving"
      elif self.state == "leaving":
        if self._current_lat_acc >= TURNING_LAT_ACC_TH:
          self.state = "turning"
        elif self._current_lat_acc < FINISH_LAT_ACC_TH:
          self.state = "enabled"
    else:
      if long_enabled and self.enabled:
        self.state = "overriding" if long_override else "enabled"

  def _update_solution(self) -> None:
    if self.state == "entering":
      a_target = float(np.interp(self._max_pred_lat_acc, ENTERING_SMOOTH_DECEL_BP, ENTERING_SMOOTH_DECEL_V))
      # DP overshoot: if we would arrive too fast at the curve entry point,
      # decelerate harder so we hit the overshoot speed exactly there.
      if self.vision_b.overshoot and self.vision_b.overshoot_distance > 0. and self.vision_b.overshoot_speed > 0.:
        v_overshoot = min(self.vision_b.overshoot_speed, self._v_cruise)
        a_required = (v_overshoot ** 2 - self._v_ego ** 2) / (2. * self.vision_b.overshoot_distance)
        # unclamped a_required goes to -inf as distance -> 0; keep the request
        # within what the car can physically do (per-car limit resolved at
        # init, e.g. MLB -2.95) so downstream math (including the no-overshoot
        # cap below) sees a realistic decel
        a_required = max(a_required, getattr(self, '_a_target_min', A_TARGET_MIN))
        a_target = min(a_target, a_required)
      self._a_target = a_target
    elif self.state == "turning":
      self._a_target = float(np.interp(self._current_lat_acc, TURNING_ACC_BP, TURNING_ACC_V))
    elif self.state == "leaving":
      self._a_target = LEAVING_ACC
    else:
      self._a_target = self._a_ego

  # -- main entry -------------------------------------------------------------

  def update(self, sm, v_ego: float, a_ego: float, v_cruise: float, long_enabled: bool,
             long_override: bool, personality) -> SccXOutput:
    """Advance one model frame.

    sm: SubMaster (reads 'modelV2' and 'controlsState')
    v_ego / a_ego: current speed [m/s] / acceleration [m/s^2]
    v_cruise: user's cruise set speed [m/s]
    long_enabled: openpilot longitudinal is active (not LongCtrlState.off)
    long_override: driver is requesting longitudinal override (gas pressed)
    personality: log.LongitudinalPersonality enum value (int-convertible)
    """
    self._v_ego, self._a_ego, self._v_cruise = v_ego, a_ego, v_cruise
    current_curvature = sm['controlsState'].curvature
    self._current_lat_acc = self._v_ego ** 2 * abs(current_curvature)

    self._update_params()
    if not self.enabled:
      self.state = "disabled"
      self.arbiter.reset()
      self.frame += 1
      return SccXOutput(state=self.state)

    self._update_estimates(sm['modelV2'], personality)
    self._update_state(long_enabled, long_override)
    self._update_solution()
    self.frame += 1

    out = SccXOutput(state=self.state, source=self.arbiter.source, debug_v_target=self._v_arb)
    if self.state in ("entering", "turning"):
      out.active = True
      out.a_target = self._a_target
      if self.state == "entering":
        v_target = max(self._v_arb, CURVE_MIN_SPEED) if self._has_target else 0.
        if v_target > 0.:
          if self.vision_b.overshoot and self.vision_b.overshoot_speed > 0.:
            # DP overshoot semantics: a_target is already the exact kinematic
            # decel to reach overshoot_speed at the curve entry point, so the
            # cap is the overshoot speed itself. Applying SP's 4 s margin on
            # top would double-count the deceleration and over-slow the car.
            # Still respect a lower arbiter-adopted target (vision_a may see
            # something worse than the lane fit does).
            v_cap = max(CURVE_MIN_SPEED, min(self.vision_b.overshoot_speed, v_target))
            out.v_cruise_cap = min(self._v_cruise, v_cap)
          else:
            # SP semantics: follow the smooth decel toward the curve speed,
            # with the no-overshoot margin; never above the user's cruise
            # setting. a_target is negative here, so the margin can push the
            # expression below zero; a speed cap must never go negative.
            out.v_cruise_cap = min(self._v_cruise, max(0., v_target + self._a_target * NO_OVERSHOOT_TIME_HORIZON))
      else:
        # hold the speed the current curvature permits
        v_cur = self._current_curve_speed()
        if v_cur > 0.:
          out.v_cruise_cap = min(self._v_cruise, v_cur)
    elif self.state == "leaving":
      out.a_target = self._a_target

    return out
