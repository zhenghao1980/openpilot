"""Unit tests for SCC-X pure logic (no cereal/messaging required).

Run: python -m unittest openpilot.selfdrive.controls.lib.scc.tests.test_scc
"""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.scc import arbiter, constants, vision_a, vision_b


def fake_model(orientation_rate_z, velocity_x, lane_lines=None, lane_probs=None, lane_stds=None):
  m = SimpleNamespace()
  m.orientationRate = SimpleNamespace(z=list(orientation_rate_z))
  m.velocity = SimpleNamespace(x=list(velocity_x))
  if lane_lines is None:
    m.laneLines = []
    m.laneLineProbs = []
    m.laneLineStds = []
  else:
    m.laneLines = lane_lines
    m.laneLineProbs = lane_probs
    m.laneLineStds = lane_stds
  return m


class TestVisionA(unittest.TestCase):
  def test_percentile_prediction(self):
    est = vision_a.VisionAEstimator()
    v_ego = 20.0
    # constant yaw rate -> constant lat acc = v * yaw
    yaw = 0.1  # rad/s
    model = fake_model([yaw] * 10, [v_ego] * 10)
    pred = est.update(model, v_ego)
    self.assertAlmostEqual(pred, v_ego * yaw, places=5)

  def test_confidence_drops_on_jump(self):
    est = vision_a.VisionAEstimator()
    v = 20.0
    est.update(fake_model([0.0] * 10, [v] * 10), v)
    c0 = est.confidence
    est.update(fake_model([1.0] * 10, [v] * 10), v)  # huge jump
    self.assertLess(est.confidence, c0)


def lane_lines_from_center(center_y, x=None):
  x = x if x is not None else np.linspace(0, 100, 33)
  half_w = 1.875
  ll = lambda ys: SimpleNamespace(t=[0.0] * len(ys), x=list(x), y=list(ys))
  return [ll([]), ll(np.array(center_y) - half_w), ll(np.array(center_y) + half_w), ll([])]


class TestVisionB(unittest.TestCase):
  def test_straight_road_zero_curvature(self):
    est = vision_b.VisionBEstimator()
    model = fake_model([], [], lane_lines_from_center(np.zeros(33)),
                       lane_probs=[0., 0.95, 0.95, 0.], lane_stds=[0., 0.1, 0.1, 0.])
    est.update(model, 20.0, 2.0)
    self.assertGreaterEqual(est.confidence, constants.MIN_LANE_PROB)
    self.assertAlmostEqual(est.max_pred_curvature, 0., places=6)

  def test_low_lane_prob_rejected(self):
    est = vision_b.VisionBEstimator()
    model = fake_model([], [], lane_lines_from_center(np.zeros(33)),
                       lane_probs=[0., 0.3, 0.3, 0.], lane_stds=[0., 0.1, 0.1, 0.])
    est.update(model, 20.0, 2.0)
    self.assertEqual(est.confidence, 0.)
    self.assertEqual(est.max_pred_curvature, 0.)

  def test_constant_radius_curvature(self):
    # y = x^2 / (2R) approximates a circle of radius R for small y
    R = 100.0
    x = np.linspace(0, 100, 33)
    center = x ** 2 / (2 * R)
    est = vision_b.VisionBEstimator()
    model = fake_model([], [], lane_lines_from_center(center, x),
                       lane_probs=[0., 0.95, 0.95, 0.], lane_stds=[0., 0.1, 0.1, 0.])
    est.update(model, 20.0, 2.0)
    # curvature of the fitted poly should approach 1/R near the start
    self.assertAlmostEqual(est.max_pred_curvature, 1. / R, delta=0.004)

  def test_overshoot_detection(self):
    # sharp turn only at the far end of the window
    R = 40.0
    x = np.linspace(0, 149, 30)
    center = np.where(x > 100, (x - 100) ** 2 / (2 * R), 0.0)
    est = vision_b.VisionBEstimator()
    model = fake_model([], [], lane_lines_from_center(center, x),
                       lane_probs=[0., 0.95, 0.95, 0.], lane_stds=[0., 0.1, 0.1, 0.])
    est.update(model, 20.0, 2.0)  # v^2/R = 10 m/s^2 lat acc -> overshoot
    self.assertTrue(est.overshoot)
    # global cubic fit smears the sharp bend toward the window start and
    # underestimates its peak; the guarantee is detection + self-consistent
    # required speed: v = sqrt(a_max / kappa_max_of_the_fitted_poly)
    self.assertGreaterEqual(est.overshoot_distance, 20.0)
    self.assertAlmostEqual(est.overshoot_speed, math.sqrt(2.0 / est.max_pred_curvature), delta=0.01)


class TestArbiter(unittest.TestCase):
  def test_vision_b_preferred_when_valid(self):
    a = arbiter.SccArbiter()
    v_b = math.sqrt(2.0 / 0.01)  # 14.1 m/s
    v_adopted, has = a.update(0., 0., 0.5, 30.0, 0.9, v_b, 0.9)
    self.assertTrue(has)
    self.assertEqual(a.source, "vision_b")
    self.assertAlmostEqual(v_adopted, v_b, delta=0.2)

  def test_map_gated_by_confidence(self):
    a = arbiter.SccArbiter()
    v, has = a.update(10.0, 0.2, 0.5, 0., 0., 0., 0.)  # map conf too low
    self.assertFalse(has)

  def test_map_wins_when_valid(self):
    a = arbiter.SccArbiter()
    v, has = a.update(10.0, 0.9, 0.5, 30.0, 0.9, 15.0, 0.9)
    self.assertTrue(has)
    self.assertEqual(a.source, "map")
    self.assertAlmostEqual(v, 10.0, delta=0.01)

  def test_hysteresis_blocks_single_frame_drop(self):
    a = arbiter.SccArbiter()
    v_b = math.sqrt(2.0 / 0.01)
    a.update(0., 0., 0.5, 30.0, 0.9, v_b, 0.9)  # adopt v_b
    # one-frame hallucination much slower
    v, _ = a.update(0., 0., 0.5, 30.0, 0.9, 5.0, 0.9)
    self.assertGreater(v, 5.0)  # not adopted yet
    for _ in range(constants.HYSTERESIS_DOWN_FRAMES):
      v, _ = a.update(0., 0., 0.5, 30.0, 0.9, 5.0, 0.9)
    self.assertAlmostEqual(v, max(5.0, constants.CURVE_MIN_SPEED), delta=0.01)

  def test_reset_clears_adopted(self):
    a = arbiter.SccArbiter()
    v_b = math.sqrt(2.0 / 0.01)
    a.update(0., 0., 0.5, 30.0, 0.9, v_b, 0.9)
    a.reset()
    v, has = a.update(0., 0., 0.5, 30.0, 0.9, 20.0, 0.9)
    self.assertTrue(has)
    self.assertAlmostEqual(v, 20.0, delta=0.01)  # adopted immediately, no up-hysteresis


class TestControllerStateMachine(unittest.TestCase):
  def _make(self, enabled=True):
    from openpilot.selfdrive.controls.lib.scc.controller import SccXController
    c = object.__new__(SccXController)
    c.enabled = enabled
    c.state = "disabled"
    c._v_ego = 20.0
    c._a_ego = 0.
    c._v_cruise = 33.0
    c._current_lat_acc = 0.
    c._max_pred_lat_acc = 0.
    c._a_lat_reg_max = 2.0
    c._v_arb, c._has_target = 0., False
    c._a_target = 0.
    c.vision_b = SimpleNamespace(overshoot=False, overshoot_distance=0., overshoot_speed=0.)
    return c

  def test_disabled_to_entering(self):
    c = self._make()
    c._max_pred_lat_acc = 1.5
    c._update_state(True, False)  # disabled -> enabled
    c._update_state(True, False)  # enabled -> entering
    self.assertEqual(c.state, "entering")

  def test_feature_off_stays_disabled(self):
    c = self._make(enabled=False)
    c._max_pred_lat_acc = 3.0
    c._update_state(True, False)
    self.assertEqual(c.state, "disabled")

  def test_override(self):
    c = self._make()
    c._max_pred_lat_acc = 1.5
    c._update_state(True, False)
    c._update_state(True, False)
    self.assertEqual(c.state, "entering")
    c._update_state(True, True)
    self.assertEqual(c.state, "overriding")
    c._update_state(True, False)  # overriding -> enabled
    c._update_state(True, False)  # enabled -> entering
    self.assertEqual(c.state, "entering")

  def test_entering_to_turning_and_leaving(self):
    c = self._make()
    c.state = "entering"
    c._current_lat_acc = 1.7
    c._update_state(True, False)
    self.assertEqual(c.state, "turning")
    c._current_lat_acc = 1.2
    c._update_state(True, False)
    self.assertEqual(c.state, "leaving")
    c._current_lat_acc = 0.5
    c._update_state(True, False)
    self.assertEqual(c.state, "enabled")

  def test_abort_on_prediction_drop(self):
    c = self._make()
    c.state = "entering"
    c._max_pred_lat_acc = 1.0
    c._update_state(True, False)
    self.assertEqual(c.state, "enabled")

  def test_overshoot_blends_into_entering_decel(self):
    c = self._make()
    c.state = "entering"
    c._max_pred_lat_acc = 2.0
    c.vision_b = SimpleNamespace(overshoot=True, overshoot_distance=50., overshoot_speed=14.0)
    c._v_ego = 30.0
    c._update_solution()
    # required decel = (14^2 - 30^2) / (2*50) = -7.04 -> must beat the -1.0 table floor
    self.assertLess(c._a_target, -1.5)

  def test_current_curve_speed(self):
    c = self._make()
    c._current_lat_acc = 2.0  # at v=20 -> kappa = 2/400 = 0.005
    v = c._current_curve_speed()
    self.assertAlmostEqual(v, math.sqrt(2.0 / 0.005), delta=0.01)

  def test_vision_speed_at_or_above_cruise_means_no_constraint(self):
    # regression: a "permitted speed" >= v_cruise must report 0, otherwise the
    # arbiter arms a huge baseline and the down-hysteresis delays real braking
    c = self._make()
    c._v_cruise = 33.0
    c._max_pred_lat_acc = 0.2  # nearly straight road -> allowed speed >> cruise
    self.assertEqual(c._vision_a_speed(), 0.)
    c.vision_b.max_pred_curvature = 1e-4
    self.assertEqual(c._vision_b_speed(), 0.)
    c.vision_b.max_pred_curvature = 0.01  # allowed = 14.1 m/s < 33 -> valid
    self.assertGreater(c._vision_b_speed(), 0.)


class TestArbiterConfidenceGate(unittest.TestCase):
  def test_vision_a_gated_below_confidence_threshold(self):
    a = arbiter.SccArbiter()
    v, has = a.update(0., 0., 0.5, 10.0, 0.4, 0., 0.)
    self.assertFalse(has)  # valid speed but confidence below CONF_VISION_A_GATE

  def test_disagreement_respects_vision_a_confidence(self):
    a = arbiter.SccArbiter()
    v_b = math.sqrt(2.0 / 0.01)
    # vision_a disagrees strongly (5.0 << v_b*0.8) but has low confidence:
    # it must NOT override the high-confidence vision_b estimate
    v, has = a.update(0., 0., 0.5, 5.0, 0.4, v_b, 0.9)
    self.assertTrue(has)
    self.assertEqual(a.source, "vision_b")
    self.assertAlmostEqual(v, v_b, delta=0.2)

  def test_disagreement_trusts_confident_vision_a(self):
    a = arbiter.SccArbiter()
    v_b = math.sqrt(2.0 / 0.01)
    v, has = a.update(0., 0., 0.5, 5.0, 0.9, v_b, 0.9)
    self.assertTrue(has)
    self.assertEqual(a.source, "vision_a")
    self.assertAlmostEqual(v, 5.0, delta=0.01)


class TestVisionBWidthGate(unittest.TestCase):
  def test_implausibly_narrow_lane_rejected(self):
    est = vision_b.VisionBEstimator()
    x = np.linspace(0, 100, 33)
    half_w = 0.9  # 1.8 m "lane" is physically implausible
    ll = lambda ys: SimpleNamespace(t=[0.0] * len(ys), x=list(x), y=list(ys))
    lines = [ll([]), ll(np.zeros(33) - half_w), ll(np.zeros(33) + half_w), ll([])]
    model = fake_model([], [], lines, lane_probs=[0., 0.95, 0.95, 0.], lane_stds=[0., 0.1, 0.1, 0.])
    est.update(model, 20.0, 2.0)
    self.assertEqual(est.confidence, 0.)
    self.assertEqual(est.max_pred_curvature, 0.)


class TestControllerRegression(TestControllerStateMachine):
  def _stub_estimators(self, c, pred_lat_acc=3.0, overshoot=False, overshoot_distance=20., overshoot_speed=5.):
    c.frame = 1  # skip the params read in _update_params (no Params in tests)
    c.vision_a = SimpleNamespace(update=lambda m, v: pred_lat_acc, confidence=0.9)
    c.vision_b = SimpleNamespace(update=lambda m, v, a: None, max_pred_curvature=0.02, confidence=0.9,
                                 overshoot=overshoot, overshoot_distance=overshoot_distance,
                                 overshoot_speed=overshoot_speed)
    c.map_est = SimpleNamespace(update=lambda v, a: None, v_target=0., confidence=0.)
    c.arbiter = arbiter.SccArbiter()

  def _sm(self):
    return {'modelV2': fake_model([0.0] * 10, [30.0] * 10),
            'controlsState': SimpleNamespace(curvature=0.0)}

  def test_v_cruise_cap_never_negative(self):
    # regression: smooth decel (a_target=-1.0) * NO_OVERSHOOT_TIME_HORIZON on a
    # very low curve speed pushed the cap below zero, commanding decel toward
    # a negative target speed
    c = self._make()
    c.state = "entering"
    self._stub_estimators(c, overshoot=False)
    c.vision_b.max_pred_curvature = 0.5  # v_allow = sqrt(2/0.5) = 2.0 -> arbiter floor 2.8
    out = c.update(self._sm(), 30.0, 0., 33.0, True, False, 1)
    self.assertIsNotNone(out.v_cruise_cap)
    self.assertGreaterEqual(out.v_cruise_cap, 0.)  # 2.8 + (-1.0)*4 = -1.2 without the clamp

  def test_smooth_decel_cap_uses_sp_margin(self):
    # no overshoot: SP semantics, cap = curve speed + a_target * 4 s
    c = self._make()
    c.state = "entering"
    self._stub_estimators(c, overshoot=False)  # v_arb = 10, a_target = -1.0
    out = c.update(self._sm(), 30.0, 0., 33.0, True, False, 1)
    self.assertAlmostEqual(out.v_cruise_cap, 10.0 + (-1.0) * constants.NO_OVERSHOOT_TIME_HORIZON, places=5)

  def test_overshoot_cap_uses_dp_semantics(self):
    # overshoot active: a_target already carries the exact kinematic decel, so
    # the cap must be the overshoot speed itself - no 4 s margin on top
    c = self._make()
    c.state = "entering"
    self._stub_estimators(c, overshoot=True, overshoot_distance=20., overshoot_speed=5.)
    out = c.update(self._sm(), 30.0, 0., 33.0, True, False, 1)
    self.assertAlmostEqual(out.v_cruise_cap, 5.0, places=5)

  def test_overshoot_cap_respects_lower_arbiter_target(self):
    # vision_a disagrees with the lane fit and wins the arbitration
    # (v_a = 8*sqrt(2/3) = 6.53 < v_b * 0.8 = 8.0): the cap must follow the
    # arbiter-adopted 6.53, not the higher overshoot_speed of 8.0
    c = self._make()
    c.state = "entering"
    self._stub_estimators(c, pred_lat_acc=3.0, overshoot=True, overshoot_distance=20., overshoot_speed=8.)
    out = c.update(self._sm(), 8.0, 0., 33.0, True, False, 1)
    self.assertAlmostEqual(out.debug_v_target, 8.0 * math.sqrt(2.0 / 3.0), places=5)
    self.assertAlmostEqual(out.v_cruise_cap, out.debug_v_target, places=5)

  def test_overshoot_decel_clamped_to_physical_limit(self):
    c = self._make()
    c.state = "entering"
    self._stub_estimators(c, overshoot=True, overshoot_distance=20., overshoot_speed=5.)
    out = c.update(self._sm(), 30.0, 0., 33.0, True, False, 1)
    # raw a_required would be (5^2-30^2)/(2*20) = -21.9; must clamp to A_TARGET_MIN
    self.assertAlmostEqual(out.a_target, constants.A_TARGET_MIN, places=5)

  def test_overshoot_decel_honors_per_car_limit(self):
    # cars with a weaker per-brand decel limit (VW MLB: -2.95) must clamp
    # there, not at the global -3.5 - otherwise the overshoot math assumes
    # braking the car cannot deliver
    c = self._make()
    c.state = "entering"
    c._a_target_min = -2.95
    self._stub_estimators(c, overshoot=True, overshoot_distance=20., overshoot_speed=5.)
    out = c.update(self._sm(), 30.0, 0., 33.0, True, False, 1)
    self.assertAlmostEqual(out.a_target, -2.95, places=5)

  def test_personality_out_of_range_clamped(self):
    c = self._make()
    self._stub_estimators(c, pred_lat_acc=0.0)
    c._update_estimates(self._sm()['modelV2'], personality=99)
    self.assertEqual(c._a_lat_reg_max, constants.A_LAT_REG_MAX_BY_PERSONALITY[-1])
    c._update_estimates(self._sm()['modelV2'], personality=-3)
    self.assertEqual(c._a_lat_reg_max, constants.A_LAT_REG_MAX_BY_PERSONALITY[0])

  def test_personality_accepts_capnp_dynamic_enum(self):
    # regression: on-device personality is a capnp _DynamicEnum which raises
    # TypeError on int() - must convert via .raw (crashed plannerd on device)
    class FakeDynamicEnum:
      def __init__(self, raw): self.raw = raw
    c = self._make()
    self._stub_estimators(c, pred_lat_acc=0.0)
    c._update_estimates(self._sm()['modelV2'], personality=FakeDynamicEnum(2))
    self.assertEqual(c._a_lat_reg_max, constants.A_LAT_REG_MAX_BY_PERSONALITY[2])
    c._update_estimates(self._sm()['modelV2'], personality=FakeDynamicEnum(99))
    self.assertEqual(c._a_lat_reg_max, constants.A_LAT_REG_MAX_BY_PERSONALITY[-1])


if __name__ == "__main__":
  unittest.main()
