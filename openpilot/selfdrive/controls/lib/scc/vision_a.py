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
    self._prev_max_pred_lat_acc = 0.
    # consistency-based confidence in [0.3, 1.0]; starts optimistic
    self.confidence = 0.5

  def update(self, model_v2, v_ego: float) -> float:
    """Returns the predicted max lateral acceleration [m/s^2] ahead.

    model_v2: the capnp modelV2 message (uses .orientationRate.z, .velocity.x)
    """
    rate_plan = np.array(np.abs(model_v2.orientationRate.z))
    vel_plan = np.maximum(np.array(model_v2.velocity.x), 0.)

    predicted = rate_plan * vel_plan
    self.max_pred_lat_acc = float(np.percentile(predicted, PRED_LAT_ACC_PERCENTILE)) if len(predicted) else 0.

    # confidence: punish large frame-to-frame swings (hallucination signature)
    jump = abs(self.max_pred_lat_acc - self._prev_max_pred_lat_acc)
    target = float(np.clip(1.0 - jump / 2.0, 0.3, 1.0))
    self.confidence = 0.8 * self.confidence + 0.2 * target
    self._prev_max_pred_lat_acc = self.max_pred_lat_acc

    return self.max_pred_lat_acc
