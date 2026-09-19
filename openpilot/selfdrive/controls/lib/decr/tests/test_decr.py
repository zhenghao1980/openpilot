# SPDX-License-Identifier: MIT
"""Unit tests for DEC-R pure logic (no cereal/messaging required).

Run: python -m unittest openpilot.selfdrive.controls.lib.decr.tests.test_decr

Scenarios mirror the chapter-15 spec: lock ticket, R1/R2/R3a/R3b grid rules,
speed-band gating, T1/T2 events, the ANB/override stand-downs and the trust
monitor. Numeric thresholds here are the engineering initial values from
constants.py, not independently calibrated truths.
"""
import unittest

from openpilot.selfdrive.controls.lib.decr import constants
from openpilot.selfdrive.controls.lib.decr.controller import DecrController, RadarInput


class FakeParams:
  def __init__(self, enabled=True):
    self._enabled = enabled

  def get_bool(self, key):
    assert key == constants.DECR_ENABLED_PARAM
    return self._enabled


def radar(soll=-1.0, locked=True, healthy=True, abidx=400, neg=3.5, pos=1.0):
  return RadarInput(soll=soll, neg_grad=neg, pos_grad=pos, status=3,
                    relevant_obj=1 if locked else 0,
                    abstandsindex=abidx if locked else 1023,
                    healthy=healthy)


def run(ctrl, n, **kw):
  args = dict(long_active=True, gas_pressed=False, stock_aeb=False,
              v_ego=27.8, v_cruise=27.8, curvature=0.001,  # 100 km/h -> B1
              lead_prob=0.9, lead_x=45.0, lead_v=25.0,     # closing 2.8 m/s
              radar=radar(), a_op=0.5)
  args.update(kw)
  out = None
  for _ in range(n):
    out = ctrl.update(**args)
  return out


def armed_ctrl(**kw):
  """Controller past the B1 lock-arm time (0.5 s) in a steady R1 scene."""
  ctrl = DecrController(is_mlb=True, params=FakeParams())
  run(ctrl, 80, **kw)  # 0.8 s: arm (0.5 s) + R1 persistence (0.4 s) both done
  return ctrl


class TestIronRules(unittest.TestCase):
  def test_min_only_silent_when_radar_wants_less_decel(self):
    # radar asks -0.2, OP already brakes at -1.0: DEC-R must stay out
    ctrl = armed_ctrl(radar=radar(soll=-0.2), a_op=-1.0)
    out = run(ctrl, 20, radar=radar(soll=-0.2), a_op=-1.0)
    self.assertIsNone(out.a_target)

  def test_lock_ticket_required(self):
    # unlocked: every negative command is set-speed tracking -> never fuse
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    out = run(ctrl, 300, radar=radar(soll=-1.58, locked=False))
    self.assertIsNone(out.a_target)
    self.assertFalse(out.armed)

  def test_healthy_gate_silent_degradation(self):
    ctrl = armed_ctrl()
    out = run(ctrl, 5, radar=radar(healthy=False))
    self.assertIsNone(out.a_target)
    self.assertFalse(out.armed)  # lock re-arms from scratch after sickness

  def test_anb_yields_same_frame(self):
    ctrl = armed_ctrl()
    self.assertIsNotNone(run(ctrl, 30).a_target)  # past arm + persistence, DEC-R active
    out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=True,
                      v_ego=27.8, v_cruise=27.8, curvature=0.001,
                      lead_prob=0.9, lead_x=45.0, lead_v=25.0, radar=radar(), a_op=0.5)
    self.assertIsNone(out.a_target)

  def test_gas_override_stands_down(self):
    ctrl = armed_ctrl()
    out = run(ctrl, 5, gas_pressed=True)
    self.assertIsNone(out.a_target)

  def test_disabled_param_silent(self):
    ctrl = DecrController(is_mlb=True, params=FakeParams(enabled=False))
    out = run(ctrl, 200)
    self.assertIsNone(out.a_target)

  def test_params_default_off(self):
    # default-off must hold even under a worst-case T1 stimulus: deep sustained
    # braking + plummeting Abstandsindex would trigger T1/FCW if enabled
    ctrl = DecrController(is_mlb=True, params=FakeParams(enabled=False))
    run(ctrl, 45, radar=radar(soll=-3.2, abidx=450))
    saw_output = False
    for _ in range(90):
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.9, lead_x=45.0, lead_v=25.0,
                        radar=radar(soll=-3.2, abidx=200), a_op=0.5)
      saw_output = saw_output or out.a_target is not None or out.fcw or out.event != "none"
    self.assertFalse(saw_output)

  def test_non_mlb_silent(self):
    ctrl = DecrController(is_mlb=False, params=FakeParams())
    out = run(ctrl, 200)
    self.assertIsNone(out.a_target)


class TestR1(unittest.TestCase):
  def test_r1_deadzone_and_clip(self):
    ctrl = armed_ctrl()
    out = run(ctrl, 200)  # long enough for slew to converge
    self.assertEqual(out.grid, "R1")
    # a_radar_eff = soll + 0.2 deadzone = -0.8, above the B1 clip -1.5
    self.assertAlmostEqual(out.a_target, -0.8, places=3)

  def test_r1_persistence_gate(self):
    # excess appears for less than the 0.4 s persistence -> not yet active
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    run(ctrl, 80)                       # armed, persist accumulator at 0
    ctrl._excess_hold_s = 0.0
    out = run(ctrl, 20)                 # 0.2 s < 0.4 s persistence
    self.assertIsNone(out.a_target)

  def test_r1_not_approaching_no_borrow(self):
    # lead pulling away: a_req ~ 0, v_close < 0 -> no fusion even if radar brakes
    ctrl = armed_ctrl(lead_v=40.0)
    out = run(ctrl, 10, lead_v=40.0)
    self.assertIsNone(out.a_target)

  def test_overcruise_half_borrow(self):
    # vEgo > vCruise + 3 km/h: borrow only half of the offered decel
    # (lead_v=27 keeps v_close=3 m/s: above the R1 approach gate, below T2's)
    ctrl = armed_ctrl(v_ego=30.0, v_cruise=27.8, lead_v=27.0)
    out = run(ctrl, 200, v_ego=30.0, v_cruise=27.8, lead_v=27.0)
    # raw cand -0.8 vs a_op 0.5: half borrow -> 0.5 + 0.5*(-0.8-0.5) = -0.15
    self.assertAlmostEqual(out.a_target, -0.15, places=3)

  def test_base_clip_b1(self):
    ctrl = armed_ctrl(radar=radar(soll=-2.2))
    out = run(ctrl, 300, radar=radar(soll=-2.2))
    self.assertAlmostEqual(out.a_target, -1.5, places=3)  # B1 clip, not -2.0 raw


class TestGridCells(unittest.TestCase):
  def test_r2_flicker_halved_persistence(self):
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    run(ctrl, 80, lead_prob=0.4)  # armed in flicker
    ctrl._excess_hold_s = 0.0
    out = run(ctrl, 25, lead_prob=0.4)  # 0.25 s >= 0.4*0.5 halved gate
    self.assertEqual(out.grid, "R2")
    self.assertIsNotNone(out.a_target)

  def test_r3a_full_follow_no_deadzone(self):
    ctrl = armed_ctrl()
    out = run(ctrl, 40, lead_prob=0.1)  # vision lost, entered from R1; slew converges
    self.assertEqual(out.grid, "R3a")
    self.assertAlmostEqual(out.a_target, -1.0, delta=0.1)  # follows soll, no deadzone

  def test_r3a_clip(self):
    ctrl = armed_ctrl(radar=radar(soll=-3.0))
    out = run(ctrl, 300, radar=radar(soll=-3.0), lead_prob=0.1)
    self.assertAlmostEqual(out.a_target, -2.5, places=3)  # R3a clip, not -3.0

  def test_r3a_budget_expires(self):
    ctrl = armed_ctrl()
    # 6 s budget at B1; run 7 s of vision loss
    out = run(ctrl, 700, lead_prob=0.1)
    self.assertIn(out.grid, ("R3x",))
    self.assertIsNone(out.a_target)
    # vision recovers -> budget resets, R1 re-engages
    out = run(ctrl, 200, lead_prob=0.9)
    self.assertEqual(out.grid, "R1")
    self.assertIsNotNone(out.a_target)

  def test_r3a_low_speed_hands_back(self):
    ctrl = armed_ctrl()
    out = run(ctrl, 20, lead_prob=0.1, v_ego=2.0)
    self.assertIsNone(out.a_target)

  def test_r3b_permanently_closed(self):
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    out = run(ctrl, 300, lead_prob=0.1, radar=radar(soll=-1.58, locked=False))
    self.assertIn(out.grid, ("R3b", "none"))
    self.assertIsNone(out.a_target)


class TestBands(unittest.TestCase):
  def test_b3_r3a_fcw_only(self):
    # 36 km/h -> B3: no R3a full-follow, FCW prompt while radar brakes
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    run(ctrl, 250, v_ego=10.0)  # past B3 arm time (2.0 s)
    out = run(ctrl, 20, v_ego=10.0, lead_prob=0.1, radar=radar(soll=-1.2))
    self.assertIsNone(out.a_target)
    self.assertTrue(out.fcw)

  def test_b3_longer_arm(self):
    # B3 arm time is 2.0 s: at 1.0 s not armed yet
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    out = run(ctrl, 100, v_ego=10.0)
    self.assertFalse(out.armed)
    out = run(ctrl, 150, v_ego=10.0)
    self.assertTrue(out.armed)

  def test_band_hysteresis(self):
    # 70 km/h sits in B2; crossing to B1 needs 77, falling to B3 needs 49
    ctrl = DecrController(is_mlb=True, params=FakeParams())
    out = run(ctrl, 60, v_ego=70.0 / 3.6)
    self.assertEqual(out.band, "B2")
    out = run(ctrl, 5, v_ego=76.0 / 3.6)
    self.assertEqual(out.band, "B2")
    out = run(ctrl, 5, v_ego=78.0 / 3.6)
    self.assertEqual(out.band, "B1")
    out = run(ctrl, 5, v_ego=70.0 / 3.6)
    self.assertEqual(out.band, "B1")  # hysteresis holds until 67


class TestEvents(unittest.TestCase):
  def test_t1_skips_deadzone_and_widens_clip(self):
    ctrl = armed_ctrl()
    # deep braking held 0.15 s + abidx plummeting (450 -> 200 stays inside the
    # 0.5 s window for 0.8 s of T1); depth is slew-limited, so track the min
    run(ctrl, 45, radar=radar(soll=-2.5, abidx=450))
    saw_t1, deepest = False, 0.0
    for _ in range(90):
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.9, lead_x=45.0, lead_v=25.0,
                        radar=radar(soll=-2.5, abidx=200), a_op=0.5)
      saw_t1 = saw_t1 or out.event == "T1"
      if out.a_target is not None:
        deepest = min(deepest, out.a_target)
    self.assertTrue(saw_t1)
    self.assertLess(deepest, -1.5)  # clip widened past the -1.5 base

  def test_t1_blocked_on_cutin(self):
    ctrl = armed_ctrl()
    run(ctrl, 10, radar=radar(soll=-2.5, abidx=400))  # establish the abidx baseline
    # fresh cut-in: lead suddenly 20 m closer
    run(ctrl, 5, lead_x=60.0)
    run(ctrl, 2, lead_x=25.0)
    saw_t1 = False
    for _ in range(20):
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.9, lead_x=25.0, lead_v=25.0,
                        radar=radar(soll=-2.5, abidx=200), a_op=0.5)
      saw_t1 = saw_t1 or out.event == "T1"
    self.assertFalse(saw_t1)

  def test_t1_blocked_far_no_vision(self):
    ctrl = armed_ctrl()
    # abidx 130 (~107 m, far) + vision flicker: no T1 bypass
    run(ctrl, 10, lead_prob=0.4, radar=radar(soll=-2.5, abidx=400))
    saw_t1 = False
    for _ in range(20):
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.4, lead_x=45.0, lead_v=25.0,
                        radar=radar(soll=-2.5, abidx=130), a_op=0.5)
      saw_t1 = saw_t1 or out.event == "T1"
    self.assertFalse(saw_t1)

  def test_fcw_on_sustained_deep_t1(self):
    ctrl = armed_ctrl()
    run(ctrl, 45, radar=radar(soll=-3.2, abidx=450))
    saw_t1, saw_fcw = False, False
    for _ in range(90):  # T1 confirm 0.15 s + FCW hold 0.5 s inside the window
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.9, lead_x=45.0, lead_v=25.0,
                        radar=radar(soll=-3.2, abidx=200), a_op=0.5)
      saw_t1 = saw_t1 or out.event == "T1"
      saw_fcw = saw_fcw or out.fcw
    self.assertTrue(saw_t1)
    self.assertTrue(saw_fcw)


class TestEnvelopeAndTrust(unittest.TestCase):
  def test_slew_limited_by_borrowed_grad(self):
    ctrl = armed_ctrl()
    run(ctrl, 100)  # converge at the R1 candidate (-0.8)
    prev = ctrl._a_eff_prev
    self.assertAlmostEqual(prev, -0.8, places=3)
    out = run(ctrl, 1, radar=radar(soll=-1.4))  # deeper request: -1.2 raw
    # one frame may move at most neg_grad * DT_CTRL
    self.assertGreaterEqual(out.a_target, prev - 3.5 * constants.DT_CTRL - 1e-6)
    self.assertLess(out.a_target, prev)

  def test_zero_grad_fallback_does_not_freeze(self):
    ctrl = armed_ctrl(radar=radar(soll=-1.0, neg=0.0, pos=0.0))
    out = run(ctrl, 300, radar=radar(soll=-1.0, neg=0.0, pos=0.0))
    self.assertAlmostEqual(out.a_target, -0.8, places=3)  # fallback 3.5 still reaches

  def test_trust_monitor_downgrades(self):
    ctrl = armed_ctrl(lead_prob=0.9)
    # radar braking hard while vision sees empty road, > 2 s
    out = run(ctrl, 250, lead_prob=0.1, radar=radar(soll=-1.2))
    self.assertEqual(out.trust_notch, 1)
    self.assertTrue(out.trust_event or ctrl._trust_notch == 1)

  def test_trust_event_lifecycle(self):
    # trust_event is a rising-edge flag: True only on the frame the notch
    # jumps, False on every frame before and after (rlog review relies on it)
    ctrl = armed_ctrl(lead_prob=0.9)
    event_frames, notch_at_event = [], []
    for i in range(250):  # 2.5 s > TRUST_WINDOW_S: exactly one downgrade
      out = ctrl.update(long_active=True, gas_pressed=False, stock_aeb=False,
                        v_ego=27.8, v_cruise=27.8, curvature=0.001,
                        lead_prob=0.1, lead_x=45.0, lead_v=25.0,
                        radar=radar(soll=-1.2), a_op=0.5)
      if out.trust_event:
        event_frames.append(i)
        notch_at_event.append(out.trust_notch)
    self.assertEqual(len(event_frames), 1)          # single rising edge
    self.assertEqual(notch_at_event, [1])           # notch jumped on that frame
    self.assertGreaterEqual(event_frames[0], int(constants.TRUST_WINDOW_S / constants.DT_CTRL) - 1)
    self.assertEqual(ctrl._trust_notch, 1)          # notch sticks after the edge

  def test_abidx_to_meters_monotonic(self):
    self.assertLess(constants.abidx_to_meters(500), constants.abidx_to_meters(300))
    self.assertAlmostEqual(constants.abidx_to_meters(567), 22.0, places=1)
    self.assertAlmostEqual(constants.abidx_to_meters(130), 107.0, places=1)


if __name__ == "__main__":
  unittest.main()
