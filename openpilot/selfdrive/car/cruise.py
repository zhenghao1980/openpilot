import math
import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.structs import car
from openpilot.common.constants import CV


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}

# --- limit-tier step-down (double-tap RES) ------------------------------------
# Feature switch param (registered in openpilot/common/params_keys.h)
LIMIT_STEP_ENABLED_PARAM = "LimitTierStepEnabled"

# 高速限速挡位表（限速牌标定值）:
#   60  kph - 匝道、连接线
#   80  kph - 隧道、桥梁
#   100 kph - 高速支线道路
#   120 kph - 高速主干道路
LIMIT_TIERS_KPH = (60.0, 80.0, 100.0, 120.0)
# 目标 = 挡位 - 1 kph：确保压限速以下通过摄像头
LIMIT_STEP_TARGET_OFFSET_KPH = 1.0
# 设定速度斜坡速率 [kph/s]：planner 的 P 律误差（v_cruise - v_ego）永不阶跃，
# 减速度解析上界 ≈ slew/3.6 m/s^2（3.0 -> 0.83 m/s^2），blended/e2e 通用
LIMIT_STEP_SLEW_KPH_PER_S = 3.0

# Double-tap gesture timing, in 100 Hz card frames: two RES release edges within
# 0.6 s, accepted only after longitudinal has been continuously active for 0.5 s.
# The settle guard keeps the "double-tap RES to engage longitudinal" habit
# (separate lat/long mode) from immediately stepping the speed down.
RES_DOUBLE_TAP_WINDOW_FRAMES = 60
LONG_SETTLE_FRAMES = 50


class ResDoubleTapGesture:
  """Frame-based double-tap RES detector (pure logic, unit-testable off-device).

  update() is called once per card frame with the frame counter, whether
  longitudinal is active this frame, and whether a RES release edge arrived in
  this frame's CarState. Returns True exactly on the second tap of a pair.
  A third tap within the window starts a fresh pair (it is the first tap of
  that pair), so a triple tap can never fire twice in a row.
  """

  def __init__(self):
    self._last_release_frame = -1
    self._long_active_frames = 0

  def update(self, frame: int, long_active: bool, res_release: bool) -> bool:
    if long_active:
      self._long_active_frames += 1
    else:
      self._long_active_frames = 0
    if not res_release:
      return False
    if not long_active or self._long_active_frames < LONG_SETTLE_FRAMES:
      return False
    if self._last_release_frame >= 0 and frame - self._last_release_frame <= RES_DOUBLE_TAP_WINDOW_FRAMES:
      self._last_release_frame = -1  # consume the pair
      return True
    self._last_release_frame = frame
    return False


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # limit-tier step-down (double-tap RES): saved set speed on entry + ramp
    # destination. None = feature idle. Trip-scoped, cleared on any manual
    # speed edit, full disengage, or cruise-unavailable.
    self.limit_step_saved: tuple[float, float] | None = None        # (v_cruise, v_cruise_cluster) on entry
    self.limit_step_target_kph: float | None = None                 # ramp destination (tier - offset)

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, long_active=True):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric, long_active)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      self.clear_limit_step()

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric, long_active=True):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    # limit-step mode: ramp the set speed toward the tier target at a fixed
    # rate so the planner's error term never steps (gentle, mode-independent).
    # Frozen while longitudinal is inactive (brake/cancel dropped long in
    # separate mode, gas override): the latch survives and the ramp resumes
    # where it left off when longitudinal rejoins.
    if self.limit_step_target_kph is not None and long_active:
      self._update_limit_step_ramp()

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # a manual +/- takes ownership of the set speed: the limit-step latch is
    # dropped (no silent auto-restore after the driver edited the speed)
    self.clear_limit_step()

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def clear_limit_step(self) -> None:
    self.limit_step_saved = None
    self.limit_step_target_kph = None

  def limit_step_toggle(self) -> str | None:
    """Double-tap RES: enter limit-step mode or restore the saved set speed.

    Enter: save the current set speed and ramp down to the highest limit tier
    strictly below the current set speed, minus LIMIT_STEP_TARGET_OFFSET_KPH.
    Exit: restore the saved set speed immediately (acceleration direction -
    the car eases up to it on its own).

    Returns "enter", "exit", or None when there is no lower tier (no-op).
    """
    if self.limit_step_saved is None:
      lower = [t for t in LIMIT_TIERS_KPH if t < self.v_cruise_kph]
      if not lower:
        return None  # already at/below the lowest tier: nothing to step down to
      self.limit_step_saved = (self.v_cruise_kph, self.v_cruise_cluster_kph)
      self.limit_step_target_kph = max(lower) - LIMIT_STEP_TARGET_OFFSET_KPH
      return "enter"

    saved, saved_cluster = self.limit_step_saved
    self.v_cruise_kph = float(min(saved, V_CRUISE_MAX))
    self.v_cruise_cluster_kph = float(min(saved_cluster, V_CRUISE_MAX))
    self.clear_limit_step()
    return "exit"

  def _update_limit_step_ramp(self) -> None:
    target = self.limit_step_target_kph
    self.v_cruise_kph = max(target, self.v_cruise_kph - LIMIT_STEP_SLEW_KPH_PER_S * DT_CTRL)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    # a fresh SET/init takes ownership of the set speed: drop the limit-step latch
    self.clear_limit_step()

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      # B8PA SET semantics (identical in experimental/chill mode): current speed
      # snapped UP to the next 5 km/h step; 40 km/h floor at or below it,
      # V_CRUISE_MAX cap above.
      v_kph = round(CS.vEgo * CV.MS_TO_KPH, 1)
      if v_kph <= V_CRUISE_INITIAL:
        self.v_cruise_kph = V_CRUISE_INITIAL
      else:
        self.v_cruise_kph = int(min(math.ceil(v_kph / 5.0) * 5, V_CRUISE_MAX))

    self.v_cruise_cluster_kph = self.v_cruise_kph
