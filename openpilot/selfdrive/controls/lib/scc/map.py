"""tsc-d (SCC-M): map-curve speed estimator on pfeiferj/mapd native output.

Data path A1 (mapd 功能说明书 v0.6): stock mapd -> mapdExtendedOut.path (1 Hz)
-> this estimator -> SccArbiter map slot. mapd side: zero changes, zero fork.

Contract (identical to the original stub, controller wiring unchanged):
  update(v_ego, a_ego)   advance one model frame (called at 20 Hz)
  v_target   [m/s]  0 = no valid map curve target (raw; the ">= v_cruise -> 0"
                    no-constraint convention is applied by the controller,
                    same as the vision sources)
  distance   [m]    distance to the first curve that binds the braking profile
  confidence [0..1] < MIN_CONFIDENCE -> arbiter drops the source
  reach_distance [m] jerk/accel-limited distance needed to slow v_ego down to
                    v_target; consumed by the controller's entering gate

Controller-side additions required by A1 (see controller.py.patch):
  - one update_msg(...) call per frame to inject the mapd messages
  - set_a_lat_max(...) whenever the personality-derived limit changes
"""
import math
import time

# map.py keeps its own copies (same values as the stub's design notes) so the
# estimator stays free of scc-package imports beyond what the controller passes.
TARGET_JERK = -0.6    # m/s^3
TARGET_ACCEL = -1.2   # m/s^2
_MSG_TIMEOUT = 2.0    # [s] path staleness limit
_EARTH_R = 6371000.0  # [m]
_PROFILE_DT = 0.05    # [s] integration step
_CURVE_KAPPA_MIN = 1e-6

MIN_CONFIDENCE = 0.5

# waySelectionType ordinals (pfeiferj/mapd WaySelectionType enum)
_MATCH_OK = ("current", "predicted", "possible")  # extended/fail excluded by design


def _haversine_m(lat1, lon1, lat2, lon2):
  dlat = math.radians(lat2 - lat1)
  dlon = math.radians(lon2 - lon1)
  a = (math.sin(dlat / 2) ** 2 +
       math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
  return 2 * _EARTH_R * math.asin(math.sqrt(a))


class MapCurveEstimator:
  MIN_CONFIDENCE = MIN_CONFIDENCE

  def __init__(self):
    self._a_lat_max = 2.0            # injected via set_a_lat_max (personality)
    self._path = []                  # [(distance_along_m, curvature)] from car
    self._path_mono = 0.0            # logMonoTime [ns] of the path we cached
    self._path_wall = 0.0            # time.monotonic() of the cached path
    self._match_ok = False
    self._match_possible = False     # possible-state profile is trusted less
    self._tile_loaded = False
    self.v_target = 0.
    self.distance = 0.
    self.confidence = 0.
    self.reach_distance = 0.

  # -- inputs ---------------------------------------------------------------

  def set_a_lat_max(self, a_lat_max: float) -> None:
    self._a_lat_max = max(float(a_lat_max), 0.5)

  def update_msg(self, mapd_ext, mapd_out, mono_time_ns: int) -> None:
    """Cache the mapd messages for this frame. Called once per model frame."""
    self._match_ok = False
    self._match_possible = False
    self._tile_loaded = False
    if mapd_out is not None:
      self._tile_loaded = bool(mapd_out.tileLoaded)
      ws = getattr(mapd_out.waySelectionType, "raw", mapd_out.waySelectionType)
      ws = str(getattr(ws, "name", ws))
      self._match_ok = ws in _MATCH_OK
      self._match_possible = (ws == "possible")
    self._path = []
    self._path_mono = mono_time_ns
    self._path_wall = time.monotonic()
    if mapd_ext is None or len(mapd_ext.path) < 2 or mapd_ext.position is None:
      return
    car = (mapd_ext.position.latitude, mapd_ext.position.longitude)
    d = 0.0
    prev = car
    for p in mapd_ext.path:
      d += _haversine_m(prev[0], prev[1], p.latitude, p.longitude)
      prev = (p.latitude, p.longitude)
      if p.curvature > _CURVE_KAPPA_MIN:
        self._path.append((d, float(p.curvature)))

  # -- evaluation ------------------------------------------------------------

  def _brake_profile(self, v_ego: float, a_ego: float, d_max: float):
    """(distance, v_max) samples of the hardest allowed slowdown from here."""
    a = min(max(a_ego, TARGET_ACCEL), 0.0)
    v = max(v_ego, 0.0)
    d = 0.0
    out = [(0.0, v)]
    while d < d_max and v > 0.05:
      a = max(a + TARGET_JERK * _PROFILE_DT, TARGET_ACCEL)
      v = max(0.0, v + a * _PROFILE_DT)
      d += v * _PROFILE_DT
      out.append((d, v))
    return out

  @staticmethod
  def _interp_profile(profile, d):
    if d <= 0:
      return profile[0][1]
    for i in range(1, len(profile)):
      if profile[i][0] >= d:
        (d0, v0), (d1, v1) = profile[i - 1], profile[i]
        t = (d - d0) / max(d1 - d0, 1e-6)
        return v0 + (v1 - v0) * t
    return profile[-1][1]

  def _reach_distance(self, v_ego: float, a_ego: float, v_tgt: float) -> float:
    if v_ego <= v_tgt:
      return 0.0
    a = min(max(a_ego, TARGET_ACCEL), 0.0)
    v, d = max(v_ego, 0.0), 0.0
    while v > v_tgt and d < 2000.0:
      a = max(a + TARGET_JERK * _PROFILE_DT, TARGET_ACCEL)
      v = max(0.0, v + a * _PROFILE_DT)
      d += v * _PROFILE_DT
    return d

  def update(self, v_ego: float, a_ego: float) -> None:
    self.v_target, self.distance, self.confidence, self.reach_distance = \
      self._evaluate(v_ego, a_ego)

  def _evaluate(self, v_ego, a_ego):
    from openpilot.selfdrive.controls.lib.scc.constants import CURVE_MIN_SPEED
    if (not self._match_ok) or (not self._tile_loaded):
      return 0., 0., 0., 0.
    if time.monotonic() - self._path_wall > _MSG_TIMEOUT:
      return 0., 0., 0., 0.
    conf = 0.7 if self._match_possible else 1.0
    if len(self._path) == 0:
      # 数据有效但前方无弯：无约束（confidence 保持正常，区别于"无数据"）
      return 0., 0., conf, 0.
    d_max = self._path[-1][0] + 50.0
    profile = self._brake_profile(v_ego, a_ego, d_max)
    v_tgt, d_first = 0., 0.
    for d, kappa in self._path:
      v_allow = math.sqrt(self._a_lat_max / kappa)
      v_allow = max(v_allow, CURVE_MIN_SPEED)
      if v_allow < self._interp_profile(profile, d) - 0.05:
        if d_first == 0.:
          d_first = d
        v_tgt = v_allow if (v_tgt == 0. or v_allow < v_tgt) else v_tgt
    if v_tgt == 0.:
      return 0., 0., conf, 0.
    return v_tgt, d_first, conf, self._reach_distance(v_ego, a_ego, v_tgt)
