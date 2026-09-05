"""Lane position offset: bias lateral control to ride a fixed distance off lane center.

E2E lateral control has no lane-center offset knob (the old lane_planner/CameraOffset
path is gone). This measures the ego-lane center from modelV2 lane lines and adds a
small clamped PD curvature correction so the car tracks a target offset instead of
the model's natural lane position.

Coordinate frames (verified empirically, route 00000063--46e17e092d, B8PA):
  modelV2 y axis:   positive = RIGHT (laneLines[1] = left line, laneLines[2] = right line)
  desiredCurvature: positive = RIGHT (controlsd/latcontrol convention)

The correction is intentionally weak and clamped: it biases where the car settles,
it never fights the model for steering authority.
"""
import math

MIN_SPEED = 8.0               # m/s, no correction below this
MAX_DESIRED_CURVATURE = 1.5e-3  # 1/m, only correct on near-straight road
LANE_PROB_MIN = 0.5
LANE_WIDTH_MIN = 2.5          # m
LANE_WIDTH_MAX = 5.5          # m
MAX_CORRECTION = 5e-4         # 1/m, hard clamp (~0.45 m/s^2 at 30 m/s)
KP = 2e-3                     # 1/m curvature per meter of position error
ZETA = 0.9                    # damping ratio for the PD loop
FILTER_TC = 0.5               # s, first-order filter on measured lane center
DT = 0.01                     # controlsd rate


class LaneOffsetController:
  def __init__(self):
    self.measured = None      # filtered lane center offset (m, + = center right of car)
    self.correction = 0.0

  def reset(self):
    self.measured = None
    self.correction = 0.0

  def update(self, active, v_ego, desired_curvature, target_offset_m,
             left_line_y, right_line_y, left_prob, right_prob):
    """Curvature correction (1/m) to add to desired_curvature.

    target_offset_m: desired lane center position relative to the car,
    + = center to the right of the car = car left of center.
    """
    if not active or target_offset_m == 0.0 or v_ego < MIN_SPEED or \
       abs(desired_curvature) > MAX_DESIRED_CURVATURE:
      self.reset()
      return 0.0

    lane_width = right_line_y - left_line_y
    confident = left_prob > LANE_PROB_MIN and right_prob > LANE_PROB_MIN and \
                LANE_WIDTH_MIN < lane_width < LANE_WIDTH_MAX
    if not confident:
      return self.correction  # hold last correction, don't snap to zero

    off = (left_line_y + right_line_y) / 2  # lane center rel car, + = center right
    if self.measured is None:
      self.measured = off
    else:
      self.measured += (off - self.measured) * (DT / FILTER_TC)

    err = self.measured - target_offset_m
    err_dot = (off - self.measured) / FILTER_TC  # d(measured)/dt, smooth by construction
    kd = 2 * ZETA * math.sqrt(KP) / max(v_ego, MIN_SPEED)
    self.correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, KP * err + kd * err_dot))
    return self.correction
