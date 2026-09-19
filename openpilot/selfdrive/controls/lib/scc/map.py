# SCC-M framework placeholder (no upstream derivation)
"""SCC-M entry point / framework (NOT implemented).

This stub defines the contract a future map-based curve estimator must fulfil
so the arbiter and state machine can consume it without further changes:

  estimator.update(...)  -> None
  estimator.v_target     -> float, m/s, or 0. when no valid map curve target
  estimator.distance     -> float, m, distance to the curve target
  estimator.confidence   -> float in [0, 1]; arbiter drops the source below
                            MapCurveEstimator.MIN_CONFIDENCE

Planned data source (sunnypilot convention): a mapd process publishes a JSON
list of {latitude, longitude, velocity} into the Params key
"MapTargetVelocities" (shared memory params), computed offline from OSM road
geometry with v = sqrt(a_lat_max / curvature). The controller-side logic
(jerk-limited reachability, lowest-speed-wins) mirrors
sunnypilot's SmartCruiseControlMap.
"""
class MapCurveEstimator:
  MIN_CONFIDENCE = 0.5

  def __init__(self):
    # imported lazily so this module stays importable without the params
    # native extension (unit tests, offline tooling)
    from openpilot.common.params import Params
    self.params = Params()
    self.v_target = 0.
    self.distance = 0.
    self.confidence = 0.

  def update(self, v_ego: float, a_ego: float) -> None:
    """TODO(SCC-M): read MapTargetVelocities, locate nearest point ahead via
    GPS, run jerk-limited reachability (TARGET_JERK=-0.6, TARGET_ACCEL=-1.2),
    and publish the lowest reachable curve speed with its distance.

    Until implemented this always reports "no data" (v_target = 0), which the
    arbiter treats as source-absent."""
    self.v_target = 0.
    self.distance = 0.
    self.confidence = 0.
