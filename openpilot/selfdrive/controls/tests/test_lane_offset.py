import math
import unittest

from openpilot.selfdrive.controls.lib.lane_offset import (
  LaneOffsetController, KP, MAX_CORRECTION, MIN_SPEED, MAX_DESIRED_CURVATURE)


def step(ctrl, p, v_lat, v_ego, target_m, active=True, confident=True, dt=0.01):
  """One integration step of a simple lateral double-integrator plant.

  p: car position rel lane center (m, + = right). Lane center rel car = -p.
  """
  off = -p
  half = 1.8
  corr = ctrl.update(active, v_ego, 0.0, target_m,
                     off - half, off + half, 0.9 if confident else 0.1, 0.9 if confident else 0.1)
  if active:
    v_lat += (v_ego ** 2) * corr * dt
    p += v_lat * dt
  return p, v_lat, corr


class TestLaneOffset(unittest.TestCase):
  def test_converges_to_left_target(self):
    # car starts 11 cm right of center (measured B8PA baseline), target 5 cm left
    ctrl = LaneOffsetController()
    p, v_lat = 0.11, 0.0
    ps = []
    for _ in range(1500):  # 15 s
      p, v_lat, _ = step(ctrl, p, v_lat, 30.0, 0.05)
      ps.append(p)
    self.assertAlmostEqual(p, -0.05, delta=0.02)
    # no wild oscillation: after settling, position stays within +/-1 cm
    self.assertTrue(all(abs(x - ps[-1]) < 0.01 for x in ps[-300:]))

  def test_correction_clamped(self):
    ctrl = LaneOffsetController()
    corr = ctrl.update(True, 30.0, 0.0, 0.30, -1.8, 1.8, 0.9, 0.9)  # 30 cm target from centered
    self.assertLessEqual(abs(corr), MAX_CORRECTION)

  def test_gates_return_zero(self):
    ctrl = LaneOffsetController()
    # inactive
    self.assertEqual(ctrl.update(False, 30.0, 0.0, 0.05, -1.8, 1.8, 0.9, 0.9), 0.0)
    # param zero = disabled
    self.assertEqual(ctrl.update(True, 30.0, 0.0, 0.0, -1.8, 1.8, 0.9, 0.9), 0.0)
    # low speed
    self.assertEqual(ctrl.update(True, MIN_SPEED - 1, 0.0, 0.05, -1.8, 1.8, 0.9, 0.9), 0.0)
    # curvy road
    self.assertEqual(ctrl.update(True, 30.0, MAX_DESIRED_CURVATURE + 1e-4, 0.05, -1.8, 1.8, 0.9, 0.9), 0.0)

  def test_unconfident_holds_correction(self):
    ctrl = LaneOffsetController()
    # build up some correction while confident
    p, v_lat = 0.15, 0.0
    last = 0.0
    for _ in range(200):
      p, v_lat, last = step(ctrl, p, v_lat, 30.0, 0.05)
    self.assertNotEqual(last, 0.0)
    # lane lines lost: correction is held, not snapped to zero
    held = ctrl.update(True, 30.0, 0.0, 0.05, -1.0, 1.0, 0.0, 0.0)
    self.assertEqual(held, ctrl.correction)
    # width out of range also holds
    held = ctrl.update(True, 30.0, 0.0, 0.05, -0.5, 0.5, 0.9, 0.9)
    self.assertEqual(held, ctrl.correction)

  def test_reset_clears_state(self):
    ctrl = LaneOffsetController()
    for _ in range(100):
      ctrl.update(True, 30.0, 0.0, 0.05, -1.9, 1.7, 0.9, 0.9)
    ctrl.reset()
    self.assertIsNone(ctrl.measured)
    self.assertEqual(ctrl.correction, 0.0)

  def test_direction_left_is_negative_curvature(self):
    # car right of center (off negative), target left: correction must be negative
    # (curvature positive = right in controlsd convention)
    ctrl = LaneOffsetController()
    corr = ctrl.update(True, 30.0, 0.0, 0.05, -0.11 - 1.8, -0.11 + 1.8, 0.9, 0.9)
    self.assertLess(corr, 0.0)


if __name__ == "__main__":
  unittest.main()
