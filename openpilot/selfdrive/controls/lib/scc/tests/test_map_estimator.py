"""Unit tests for MapCurveEstimator (run with pytest, no cereal needed)."""
import math
from openpilot.selfdrive.controls.lib.scc.map import MapCurveEstimator, _haversine_m


class FakePosition:
  def __init__(self, lat, lon):
    self.latitude, self.longitude = lat, lon


class FakePathPoint:
  def __init__(self, lat, lon, curvature):
    self.latitude, self.longitude, self.curvature = lat, lon, curvature
    self.targetVelocity = 0.0


class FakeExt:
  def __init__(self, car_lat, car_lon, pts):
    self.position = FakePosition(car_lat, car_lon)
    self.path = [FakePathPoint(*p) for p in pts]


class FakeOut:
  class WS:
    def __init__(self, name):
      self.raw = type("E", (), {"name": name})()
  def __init__(self, ws_name="current", tile=True):
    self.waySelectionType = self.WS(ws_name)
    self.tileLoaded = tile


def feed(est, pts, ws="current", tile=True, v=20.0, a=0.0, a_lat=2.0):
  ext = FakeExt(pts[0][0], pts[0][1], pts)
  est.set_a_lat_max(a_lat)
  est.update_msg(ext, FakeOut(ws, tile), 1_000_000_000)
  est.update(v, a)


def straight_then_curve(straight_m=60):
  # straight_m 米直道后接定曲率弯道（kappa=1/100）
  pts = [(39.0 + i * 1e-5, 116.0, 0.0) for i in range(0, straight_m, 10)]
  pts += [(39.0 + (straight_m + i) * 1e-5, 116.0, 0.01) for i in range(0, 300, 10)]
  return pts


def test_straight_no_target():
  est = MapCurveEstimator()
  pts = [(39.0 + i * 1e-5, 116.0, 0.0) for i in range(0, 500, 10)]
  feed(est, pts)
  assert est.v_target == 0. and est.confidence > 0.9


def test_curve_detected():
  est = MapCurveEstimator()
  feed(est, straight_then_curve(), v=20.0, a_lat=2.0)
  # v_allow = sqrt(2.0/0.01) = 14.1 m/s < 20 -> target ~14.1
  assert 13.0 < est.v_target < 15.5
  assert 55.0 < est.distance < 95.0       # first binding point at curve entry
  assert est.reach_distance > est.distance          # 弯在可达距离内才构成约束
  assert est.reach_distance > 0.0
  assert est.confidence == 1.0


def test_gating_fail_and_extended():
  for ws in ("fail", "extended"):
    est = MapCurveEstimator()
    feed(est, straight_then_curve(), ws=ws)
    assert est.v_target == 0. and est.confidence == 0.0


def test_gating_tile_unloaded():
  est = MapCurveEstimator()
  feed(est, straight_then_curve(), tile=False)
  assert est.v_target == 0. and est.confidence == 0.0


def test_curve_beyond_reach_is_not_a_constraint():
  # 弯道在 jerk 受限可达距离之外（~103m）：现在不刹车也能降到弯速 → 无约束
  est = MapCurveEstimator()
  feed(est, straight_then_curve(straight_m=100), v=20.0, a_lat=2.0)
  assert est.v_target == 0. and est.confidence > 0.9


def test_slow_enough_no_target():
  est = MapCurveEstimator()
  feed(est, straight_then_curve(), v=10.0)   # 10 m/s < v_allow 14.1
  assert est.v_target == 0.


def test_personality_scales_target():
  est_fast = MapCurveEstimator(); feed(est_fast, straight_then_curve(), a_lat=2.4)
  est_slow = MapCurveEstimator(); feed(est_slow, straight_then_curve(), a_lat=1.7)
  assert est_fast.v_target > est_slow.v_target


def test_haversine():
  d = _haversine_m(39.0, 116.0, 39.0, 116.01)
  assert 800.0 < d < 900.0
