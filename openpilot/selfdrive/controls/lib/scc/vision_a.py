# Derived from sunnypilot SCC-V (https://github.com/sunnyhaibin/sunnypilot) - MIT License
"""Vision-A curvature estimator (sunnypilot SCC-V method).

Predicts curve-induced lateral acceleration directly from the driving model's
orientation rate and velocity, using the 97th percentile for noise rejection.
Cheap, always available, but has no inherent quality estimate - confidence is
derived from frame-to-frame consistency instead.
"""
import numpy as np

from openpilot.selfdrive.controls.lib.scc.constants import PRED_LAT_ACC_PERCENTILE


class VisionAEstimator:
  def __init__(self):
    self.max_pred_lat_acc = 0.
    # plan speed at the percentile point [m/s]; the controller needs this to
    # convert predicted lat-acc back to an allowed speed with the same
    # reference velocity the prediction was built with (N-04)
    self.v_at_p97 = 0.
    self._prev_max_pred_lat_acc = 0.
    # consistency-based confidence in [0.3, 1.0]; starts optimistic
    self.confidence = 0.5

  def update(self, model_v2, v_ego: float) -> float:
    """Returns the predicted max lateral acceleration [m/s^2] ahead.

    model_v2: the capnp modelV2 message (uses .orientationRate.z, .velocity.x)
    """
    rate_plan = np.array(np.abs(model_v2.orientationRate.z))
    vel_plan = np.maximum(np.array(model_v2.velocity.x), 0.)

    # N-01/R-22: one corrupt model frame (NaN/Inf) must not poison the estimate.
    # Fail-LOUD, not fail-silent: zeroing NaN via nan_to_num would let the zeros
    # bias the percentile low and trigger phantom deceleration. Instead, reject
    # the frame outright (confidence = 0 -> the arbiter refuses this source) and
    # keep the last good estimate. Confidence recovers through the IIR below on
    # subsequent clean frames.
    predicted = rate_plan * vel_plan
    if not np.all(np.isfinite(predicted)):
      self.confidence = 0.0
      return self.max_pred_lat_acc
    if len(predicted):
      p = float(np.percentile(predicted, PRED_LAT_ACC_PERCENTILE))
      self.max_pred_lat_acc = p
      self.v_at_p97 = float(vel_plan[int(np.argmin(np.abs(predicted - p)))])
    else:
      self.max_pred_lat_acc = 0.
      self.v_at_p97 = 0.

    # confidence: punish large frame-to-frame swings (hallucination signature)
    jump = abs(self.max_pred_lat_acc - self._prev_max_pred_lat_acc)
    if not np.isfinite(jump):
      # belt-and-braces against the NaN absorbing state
      target = 0.3
    else:
      target = float(np.clip(1.0 - jump / 2.0, 0.3, 1.0))
    self.confidence = 0.8 * self.confidence + 0.2 * target
    self._prev_max_pred_lat_acc = self.max_pred_lat_acc

    return self.max_pred_lat_acc
