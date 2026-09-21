"""R-06 / R-02 / R-24: separate lat/long control-flow regression tests.

Covers selfdrived._update_separate_lat_long (button sequences + state machine)
and card.py's _pending_gra_cs GRA-intent stash window:

  R-02  blocked ALA press must inject buttonEnable ONCE per release edge and
        must NEVER flip self.enabled while NO_ENTRY is present
  R-06  (1) wrong-gear ALA press, (2) multiple NO_ENTRY events, (3) NO_ENTRY
        dither across frames
  R-24  SET/RES/brake/cancel sequences, resume-with-unset-speed, GRA stash
        window boundaries in card.py

SelfdriveD/CarD are built via __new__ with only the attributes the code under
test touches, so no sockets, params, or car interfaces are needed.
"""
import unittest
from types import SimpleNamespace
from unittest import mock

from opendbc.car.structs import car
from openpilot.selfdrive.selfdrived.events import Events, ET, EventName
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.car import card as card_mod

ButtonType = car.CarState.ButtonEvent.Type


def _be(btn_type, pressed):
  return SimpleNamespace(type=btn_type, pressed=pressed)


def _cs(button_events=(), v_ego=10.0, v_cruise=100.0, brake=False, standstill=False, cruise_available=True):
  return SimpleNamespace(buttonEvents=list(button_events), vEgo=v_ego, vCruise=v_cruise,
                         brakePressed=brake, standstill=standstill,
                         cruiseState=SimpleNamespace(available=cruise_available))


def _make_sd(separate=True, min_enable_speed=0.0):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = SimpleNamespace(openpilotLongitudinalControl=True, minEnableSpeed=min_enable_speed)
  sd.events = Events()
  sd.state_machine = StateMachine()
  sd.enabled = False
  sd.active = False
  sd.enabled_prev = False
  sd.separate_lat_long = separate
  sd.separate_lat_long_active = separate
  sd.lat_enabled = False
  sd.long_enabled = False
  sd.lat_wanted = False
  sd.long_wanted = False
  sd.CS_prev = SimpleNamespace(brakePressed=False)
  return sd


class TestSeparateLatLong(unittest.TestCase):

  @staticmethod
  def _frame(sd, cs):
    """Mirror step(): the main state-machine update runs BEFORE
    _update_separate_lat_long (selfdrived.py:577-582)."""
    sd.enabled, sd.active = sd.state_machine.update(sd.events)
    sd._update_separate_lat_long(cs)


  # --- R-02: blocked ALA press ---

  def test_blocked_ala_press_wrong_gear(self):
    """R-06(1)/R-02: ALA released in P gear -> NO_ENTRY alert surfaced via one
    buttonEnable injection, enabled stays False, lat_wanted never latches."""
    sd = _make_sd()
    sd.events.add(EventName.wrongGear)  # NO_ENTRY present
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertFalse(sd.enabled, "enabled must NOT flip True on a blocked press")
    self.assertFalse(sd.lat_wanted)
    self.assertFalse(sd.lat_enabled)
    self.assertTrue(sd.events.contains(ET.NO_ENTRY), "NO_ENTRY alert must surface")
    self.assertEqual(sd.events.names.count(EventName.buttonEnable), 1, "buttonEnable must be injected exactly once (R-02)")

  def test_blocked_ala_press_multiple_no_entry(self):
    """R-06(2): wrongGear + belowEngageSpeed stacked -> still blocked, once."""
    sd = _make_sd()
    sd.events.add(EventName.wrongGear)
    sd.events.add(EventName.belowEngageSpeed)
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertFalse(sd.enabled)
    self.assertFalse(sd.lat_wanted)

  def test_no_entry_dither_never_enables(self):
    """R-06(3)/R-02: NO_ENTRY flickering frame-to-frame while the driver holds
    the button must never produce an enabled=True frame on a blocked press."""
    sd = _make_sd()
    enabled_seen = []
    for frame in range(10):
      sd.events.clear()
      if frame % 2 == 0:
        sd.events.add(EventName.wrongGear)
      # re-press each frame is not physical; press once on the first frame only
      evs = [_be(ButtonType.lkas, False)] if frame == 0 else []
      self._frame(sd, _cs(evs))
      enabled_seen.append(sd.enabled)
    self.assertNotIn(True, enabled_seen[:1], "blocked frame must not enable")
    # the single press latches as soon as NO_ENTRY clears (can_engage on the
    # press frame only) -- since the press happened on a blocked frame, it must
    # NOT sneak in later:
    self.assertFalse(any(enabled_seen), "blocked attempt must not sneak in on later frames")

  def test_ala_press_engages_lateral_when_clear(self):
    """ALA release with no NO_ENTRY: lat_wanted latches, state machine enables,
    lat_enabled True while long_enabled stays False."""
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertTrue(sd.lat_wanted)
    self.assertTrue(sd.enabled)
    self.assertTrue(sd.lat_enabled)
    self.assertFalse(sd.long_enabled)

  def test_ala_press_toggles_lateral_off(self):
    """Lateral running: ALA release drops lateral; full disengage chime when it
    was the last active control."""
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertTrue(sd.lat_enabled)
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertFalse(sd.lat_wanted)
    self.assertFalse(sd.enabled)
    self.assertIn(EventName.buttonCancel, sd.events.names)

  # --- R-24: SET/RES sequences ---

  def test_set_press_engages_longitudinal(self):
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertTrue(sd.long_wanted)
    self.assertTrue(sd.enabled)
    self.assertTrue(sd.long_enabled)
    self.assertFalse(sd.lat_enabled)

  def test_blocked_set_does_not_sneak_in_later(self):
    """The anti-latch regression: SET blocked in P gear must not engage
    longitudinal on a later unrelated event."""
    sd = _make_sd()
    sd.events.add(EventName.wrongGear)
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertFalse(sd.long_wanted)
    self.assertFalse(sd.enabled)
    # NO_ENTRY clears (driver shifted to D), no button this frame:
    sd.events.clear()
    self._frame(sd, _cs())
    self.assertFalse(sd.long_wanted, "stale SET intent must not latch")
    self.assertFalse(sd.enabled)
    # a deliberate RES in D engages:
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.resumeCruise, False)]))
    self.assertTrue(sd.long_wanted)
    self.assertTrue(sd.long_enabled)

  def test_resume_with_unset_speed_is_noop(self):
    """Stock GRA: RES with no stored set speed (vCruise > 250) is a no-op even
    though lateral keeps can_engage True."""
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))  # lateral on
    self.assertTrue(sd.lat_enabled)
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.resumeCruise, False)], v_cruise=300.0))
    self.assertFalse(sd.long_wanted)

  def test_below_min_enable_speed_never_latches_long(self):
    """B8 TSK cruise floor: below minEnableSpeed with lateral running, SET must
    not latch long_wanted and must not play the engage chime."""
    sd = _make_sd(min_enable_speed=4.2)  # 15 kph
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertTrue(sd.lat_enabled)
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)], v_ego=1.0))
    self.assertFalse(sd.long_wanted)
    self.assertIn(EventName.longBelowEngageSpeed, sd.events.names)
    self.assertNotIn(EventName.buttonEnable, sd.events.names)

  def test_cancel_drops_longitudinal_only(self):
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertTrue(sd.lat_enabled and sd.long_enabled)
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.cancel, True)]))
    self.assertFalse(sd.long_wanted)
    self.assertTrue(sd.lat_wanted, "cancel must not touch lateral")
    self.assertTrue(sd.lat_enabled)
    self.assertIn(EventName.longDisabled, sd.events.names)

  def test_brake_drops_longitudinal_only(self):
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    sd.events.clear()
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertTrue(sd.long_enabled)
    sd.events.clear()
    sd.CS_prev.brakePressed = False
    self._frame(sd, _cs(brake=True))
    self.assertFalse(sd.long_wanted)
    self.assertTrue(sd.lat_wanted, "brake must not touch lateral")
    self.assertTrue(sd.lat_enabled)

  def test_cruise_main_off_drops_longitudinal(self):
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertTrue(sd.long_enabled)
    sd.events.clear()
    self._frame(sd, _cs(cruise_available=False))
    self.assertFalse(sd.long_wanted)

  def test_falling_edge_clears_wanted(self):
    """Safety/user disable (enabled True -> False) clears both wanted flags."""
    sd = _make_sd()
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self._frame(sd, _cs([_be(ButtonType.setCruise, False)]))
    self.assertTrue(sd.lat_wanted and sd.long_wanted)
    sd.events.clear()
    sd.events.add(EventName.wrongGear)  # USER_DISABLE + NO_ENTRY
    self._frame(sd, _cs())
    # the state machine disables on this frame (USER_DISABLE re-run); the
    # falling-edge clear (enabled True -> False) applies from the next frame
    self.assertFalse(sd.enabled)
    sd.events.clear()
    self._frame(sd, _cs())
    self.assertFalse(sd.lat_wanted)
    self.assertFalse(sd.long_wanted)

  # --- mode latch (R-15 context) ---

  def test_mode_latched_while_engaged(self):
    """Changing the SeparateLatLongControl setting mid-drive must not alter the
    active control behavior; the mode only re-syncs while disengaged."""
    sd = _make_sd(separate=True)
    self._frame(sd, _cs([_be(ButtonType.lkas, False)]))
    self.assertTrue(sd.enabled)
    sd.separate_lat_long = False  # param flipped mid-drive
    self._frame(sd, _cs())
    self.assertTrue(sd.separate_lat_long_active, "mode must stay latched while engaged")
    # after disengage it re-syncs
    sd.events.clear()
    sd.events.add(EventName.wrongGear)
    self._frame(sd, _cs())
    self.assertFalse(sd.enabled)
    sd.events.clear()
    self._frame(sd, _cs())
    self.assertFalse(sd.separate_lat_long_active)

  def test_passthrough_when_separate_off(self):
    """Feature off: lat/long simply mirror the overall enabled state."""
    sd = _make_sd(separate=False)
    self._frame(sd, _cs())
    self.assertFalse(sd.lat_wanted)
    sd.events.clear()
    sd.events.add(EventName.buttonEnable)  # external enable (stock SET path)
    self._frame(sd, _cs())
    self.assertTrue(sd.enabled)
    self.assertTrue(sd.lat_enabled and sd.long_enabled)
    self.assertTrue(sd.lat_wanted and sd.long_wanted)


class TestPendingGraStash(unittest.TestCase):
  """R-24: card.py _pending_gra_cs GRA-intent stash (scc_diff.patch:65-67)."""

  def _make_card(self):
    c = card_mod.Car.__new__(card_mod.Car)
    c.can_sock = None
    c.CI = mock.Mock()
    c.RI = mock.Mock()
    c.RI.update.return_value = None
    c.can_rcv_cum_timeout_counter = 0
    c.is_metric = True
    c.experimental_mode = False
    c._cs_frame = 0
    c._pending_gra_cs = None
    c._pending_gra_frame = -1
    c.v_cruise_helper = mock.Mock()
    c.v_cruise_helper.v_cruise_kph = 0.0
    c.v_cruise_helper.v_cruise_cluster_kph = 0.0
    c.CS_prev = _cs()
    c.CC_prev = SimpleNamespace(enabled=False, longActive=False)
    self._car_control = SimpleNamespace(enabled=False, longActive=False)
    c.sm = mock.Mock()
    c.sm.__getitem__ = mock.Mock(side_effect=lambda k: self._car_control)
    return c

  def _step(self, c, cs, enabled, long_active):
    c.CI.update.return_value = cs
    self._car_control.enabled = enabled
    self._car_control.longActive = long_active
    with mock.patch.object(card_mod.messaging, "drain_sock_raw", return_value=[b"x"]), \
         mock.patch.object(card_mod, "can_capnp_to_list", return_value=[]):
      out, _ = c.state_update()
    c.CS_prev = cs
    c.CC_prev = SimpleNamespace(enabled=enabled, longActive=long_active)
    return out

  def test_set_released_while_inactive_is_stashed_and_consumed_on_join(self):
    """SET while longActive=False -> stashed; when longitudinal joins 2 frames
    later with no new button, init uses the STASHED CarState."""
    c = self._make_card()
    cs_set = _cs([_be(ButtonType.setCruise, False)])
    self._step(c, cs_set, enabled=True, long_active=False)
    self.assertIs(c._pending_gra_cs, cs_set)
    # one frame gap, then longitudinal joins without any button
    self._step(c, _cs(), enabled=True, long_active=False)
    self._step(c, _cs(), enabled=True, long_active=True)
    # enabled was already True, no fresh button: only the stash could trigger init
    c.v_cruise_helper.initialize_v_cruise.assert_called_once()
    self.assertIs(c.v_cruise_helper.initialize_v_cruise.call_args[0][0], cs_set)
    self.assertIsNone(c._pending_gra_cs, "stash must be consumed once longActive")

  def test_stash_expires_after_window(self):
    """R-03 window: a stash older than PENDING_GRA_WINDOW_FRAMES must not init
    cruise speed when longitudinal joins for an unrelated reason; a stash at
    exactly the window edge still must."""
    c = self._make_card()
    cs_set = _cs([_be(ButtonType.setCruise, False)])
    self._step(c, cs_set, enabled=True, long_active=False)
    for _ in range(card_mod.PENDING_GRA_WINDOW_FRAMES + 1):
      self._step(c, _cs(), enabled=True, long_active=False)
    self._step(c, _cs(), enabled=True, long_active=True)
    c.v_cruise_helper.initialize_v_cruise.assert_not_called()

    c2 = self._make_card()
    cs_set2 = _cs([_be(ButtonType.setCruise, False)])
    self._step(c2, cs_set2, enabled=True, long_active=False)
    for _ in range(card_mod.PENDING_GRA_WINDOW_FRAMES - 1):
      self._step(c2, _cs(), enabled=True, long_active=False)
    self._step(c2, _cs(), enabled=True, long_active=True)
    c2.v_cruise_helper.initialize_v_cruise.assert_called_once()
    self.assertIs(c2.v_cruise_helper.initialize_v_cruise.call_args[0][0], cs_set2)

  def test_fresh_button_beats_stash(self):
    """If the joining frame carries its own SET/RES (previous CarState), the
    stash must NOT be the init source."""
    c = self._make_card()
    cs_set_old = _cs([_be(ButtonType.setCruise, False)])
    self._step(c, cs_set_old, enabled=True, long_active=False)
    # a fresh RES press happens one frame before longitudinal joins
    cs_res = _cs([_be(ButtonType.resumeCruise, False)])
    self._step(c, cs_res, enabled=True, long_active=False)
    self._step(c, _cs(), enabled=True, long_active=True)
    c.v_cruise_helper.initialize_v_cruise.assert_called_once()
    self.assertIs(c.v_cruise_helper.initialize_v_cruise.call_args[0][0], cs_res)

  def test_gas_override_resume_does_not_reinit(self):
    """longActive re-joins with enabled held True and no button and no stash:
    cruise speed must NOT be re-initialized (driver's set speed survives)."""
    c = self._make_card()
    self._step(c, _cs(), enabled=True, long_active=False)
    self._step(c, _cs(), enabled=True, long_active=True)
    c.v_cruise_helper.initialize_v_cruise.assert_not_called()


if __name__ == "__main__":
  unittest.main()
