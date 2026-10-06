import itertools
import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized_class
from openpilot.cereal import log
from openpilot.selfdrive.car.cruise import (
  VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_INITIAL, IMPERIAL_INCREMENT,
  LIMIT_TIERS_KPH, LIMIT_STEP_TARGET_OFFSET_KPH, LIMIT_STEP_SLEW_KPH_PER_S,
  ResDoubleTapGesture, LONG_SETTLE_FRAMES, RES_DOUBLE_TAP_WINDOW_FRAMES,
)
from opendbc.car import DT_CTRL
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed(OpenpilotTestCase):
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    self.assertAlmostEqual(simulation_steady_state, cruise_speed, delta=.01, msg=f'Did not reach {self.speed} m/s')


# TODO: test pcmCruise
@parameterized_class(('pcm_cruise',), [(False,)])
class TestVCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)

  def enable(self, v_ego, experimental_mode):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert (not pressed) == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts pressing set while enabled with gas pressed sets
    the speed to the maximum of vEgo and current cruise speed.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # first decrement speed, then perform gas pressed logic
      expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego * CV.MS_TO_KPH)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for v_ego in np.linspace(0, 100, 101):
        self.reset_cruise_speed_state()
        assert not self.v_cruise_helper.v_cruise_initialized

        self.enable(float(v_ego), experimental_mode)
        assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
        assert self.v_cruise_helper.v_cruise_initialized


class TestLimitStepMode(OpenpilotTestCase):
  """Double-tap RES limit-tier step-down: tier mapping, ramp, restore, clears."""

  def setup_method(self):
    self._setup()

  def _setup(self):
    self.CP = car.CarParams(pcmCruise=False)
    self.v_cruise_helper = VCruiseHelper(self.CP)

  def _set(self, v_cruise_kph):
    self.v_cruise_helper.v_cruise_kph = float(v_cruise_kph)
    self.v_cruise_helper.v_cruise_cluster_kph = float(v_cruise_kph)

  def _step(self, n=1, enabled=True, long_active=True, available=True):
    CS = car.CarState(cruiseState={"available": available})
    for _ in range(n):
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=True, long_active=long_active)
    return self.v_cruise_helper.v_cruise_kph

  def test_tier_mapping(self):
    # (current set, expected target): strictly-lower-tier minus 1 kph
    cases = [(132.0, 119.0), (121.0, 119.0), (120.0, 99.0), (110.0, 99.0),
             (100.0, 79.0), (99.0, 79.0), (88.0, 79.0), (80.0, 59.0),
             (61.0, 59.0)]
    for current, target in cases:
      self._setup()
      self._set(current)
      assert self.v_cruise_helper.limit_step_toggle() == "enter"
      assert self.v_cruise_helper.limit_step_target_kph == target
      assert self.v_cruise_helper.limit_step_saved == (current, current)

  def test_no_lower_tier_noop(self):
    for current in (60.0, 59.0, 40.0, V_CRUISE_MIN):
      self._setup()
      self._set(current)
      assert self.v_cruise_helper.limit_step_toggle() is None
      assert self.v_cruise_helper.limit_step_saved is None
      assert self.v_cruise_helper.v_cruise_kph == current

  def test_ramp_rate_and_floor(self):
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    # 3 kph/s: 1 s of 100 Hz frames steps 3.0 (float-tolerant: 0.03/frame
    # accumulates representation error)
    v = self._step(int(1.0 / DT_CTRL))
    self.assertAlmostEqual(v, 129.0, places=6)
    # ramp to the 119.0 floor and hold there
    v = self._step(int(5.0 / DT_CTRL))
    self.assertAlmostEqual(v, 119.0, places=6)
    # cluster mirrors on the non-pcm path
    self.assertAlmostEqual(self.v_cruise_helper.v_cruise_cluster_kph, 119.0, places=6)

  def test_restore(self):
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self._step(int(2.0 / DT_CTRL))  # partway down the ramp
    assert self.v_cruise_helper.v_cruise_kph < 132.0
    assert self.v_cruise_helper.limit_step_toggle() == "exit"
    assert self.v_cruise_helper.v_cruise_kph == 132.0
    assert self.v_cruise_helper.v_cruise_cluster_kph == 132.0
    assert self.v_cruise_helper.limit_step_saved is None
    # idle toggle after exit is an enter again (saved fresh)
    assert self.v_cruise_helper.limit_step_toggle() == "enter"

  def test_restore_clamped_to_max(self):
    self._set(200.0)
    self.v_cruise_helper.limit_step_toggle()
    assert self.v_cruise_helper.limit_step_toggle() == "exit"
    assert self.v_cruise_helper.v_cruise_kph == V_CRUISE_MAX

  def test_manual_button_clears(self):
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self._step(int(1.0 / DT_CTRL))  # mid-ramp
    # simulate a falling-edge decel press accepted by the button state machine
    helper = self.v_cruise_helper
    helper.button_change_states[ButtonType.decelCruise] = {"standstill": False, "enabled": True}
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
    helper.update_v_cruise(CS, enabled=True, is_metric=True, long_active=True)
    assert helper.limit_step_saved is None
    assert helper.limit_step_target_kph is None

  def test_initialize_clears(self):
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=30.0), False)
    assert self.v_cruise_helper.limit_step_saved is None
    assert self.v_cruise_helper.limit_step_target_kph is None

  def test_unavailable_clears(self):
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    CS = car.CarState(cruiseState={"available": False})
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True)
    assert self.v_cruise_helper.limit_step_saved is None
    assert self.v_cruise_helper.limit_step_target_kph is None
    assert not self.v_cruise_helper.v_cruise_initialized

  # --- mid-ramp interactions -------------------------------------------------

  def test_ramp_frozen_while_long_inactive(self):
    """Brake/cancel dropping longitudinal (separate mode: enabled stays True),
    gas override (DisengageOnAccelerator=off: overriding keeps enabled True),
    or any longActive gap freezes the ramp; the latch survives and the ramp
    resumes where it left off when longitudinal rejoins."""
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self.assertAlmostEqual(self._step(int(1.0 / DT_CTRL)), 129.0, places=6)
    # longActive drops for 2 s: set speed must not move
    self.assertAlmostEqual(self._step(int(2.0 / DT_CTRL), long_active=False), 129.0, places=6)
    # longitudinal rejoins: ramp resumes from where it froze
    self.assertAlmostEqual(self._step(int(1.0 / DT_CTRL), long_active=True), 126.0, places=6)
    assert self.v_cruise_helper.limit_step_saved == (132.0, 132.0)

  def test_restore_mid_ramp_stops_ramp(self):
    """Double-tap mid-ramp restores the saved speed and the ramp is gone."""
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self._step(int(2.0 / DT_CTRL))  # partway down
    assert self.v_cruise_helper.limit_step_toggle() == "exit"
    self.assertAlmostEqual(self.v_cruise_helper.v_cruise_kph, 132.0, places=6)
    # no residual ramp: it stays at the restored speed
    self.assertAlmostEqual(self._step(int(2.0 / DT_CTRL)), 132.0, places=6)

  def test_set_mid_ramp_reinitializes(self):
    """SET re-set mid-ramp clears the latch and re-initializes the speed
    (B8PA snap-up: 30 m/s -> 110)."""
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self._step(int(2.0 / DT_CTRL))
    assert self.v_cruise_helper.limit_step_saved is not None
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=30.0), False)
    assert self.v_cruise_helper.limit_step_saved is None
    assert self.v_cruise_helper.limit_step_target_kph is None
    assert self.v_cruise_helper.v_cruise_kph == 110.0

  def test_unavailable_mid_ramp_clears(self):
    """Cruise lever OFF mid-ramp (cruiseState.available drops) clears the latch."""
    self._set(132.0)
    self.v_cruise_helper.limit_step_toggle()
    self._step(int(1.0 / DT_CTRL))
    self._step(1, available=False)
    assert self.v_cruise_helper.limit_step_saved is None
    assert self.v_cruise_helper.limit_step_target_kph is None
    assert not self.v_cruise_helper.v_cruise_initialized


class TestResDoubleTapGesture(OpenpilotTestCase):
  """Frame-based double-tap RES detector semantics (pure logic)."""

  def _run(self, frames: list[tuple[bool, bool]]) -> list[int]:
    """frames: [(long_active, res_release), ...] -> frame indices that fired."""
    g = ResDoubleTapGesture()
    return [i for i, (la, rel) in enumerate(frames) if g.update(i, la, rel)]

  def test_settled_double_tap_fires_on_second_tap(self):
    frames = [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True), (True, True)]
    assert self._run(frames) == [LONG_SETTLE_FRAMES + 6]

  def test_single_tap_never_fires(self):
    frames = [(True, False)] * (LONG_SETTLE_FRAMES + 5) + [(True, True)]
    assert self._run(frames) == []

  def test_taps_outside_window_are_two_first_taps(self):
    frames = [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True)]
    frames += [(True, False)] * (RES_DOUBLE_TAP_WINDOW_FRAMES + 1)
    frames += [(True, True)]
    assert self._run(frames) == []

  def test_triple_tap_fires_once(self):
    frames = [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True)] * 3
    assert self._run(frames) == [LONG_SETTLE_FRAMES + 6]

  def test_unsettled_long_ignores_taps(self):
    frames = [(True, False)] * 10 + [(True, True), (True, True)]
    frames += [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True), (True, True)]
    fired = self._run(frames)
    assert fired == [len(frames) - 1]

  def test_long_inactive_resets_everything(self):
    frames = [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True)]                    # first tap armed
    frames += [(False, False)] * 20             # long drops: settle + tap state reset
    frames += [(True, True), (True, True)]      # only 2 settled? no: settle not reached
    assert self._run(frames) == []
    # and after a full re-settle the pair fires
    frames += [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True), (True, True)]
    assert self._run(frames) == [len(frames) - 1]

  def test_release_while_long_inactive_ignored(self):
    frames = [(False, True)] * 5
    frames += [(True, False)] * (LONG_SETTLE_FRAMES + 5)
    frames += [(True, True), (True, True)]
    fired = self._run(frames)
    assert fired == [len(frames) - 1]
