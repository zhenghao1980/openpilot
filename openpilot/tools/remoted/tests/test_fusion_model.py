"""Unit tests for FusionModelState blending logic (no GPU / no network)."""
import os
import sys
import time
import unittest

import numpy as np

REPO = os.path.expanduser("~/openpilot-c3x")
sys.path.insert(0, REPO)

from openpilot.selfdrive.modeld.fusion_model import FusionModelState
from openpilot.selfdrive.modeld.constants import ModelConstants

T_IDXS = ModelConstants.T_IDXS
X_IDXS = ModelConstants.X_IDXS

META = {
  "input_shapes": {"img": [1, 12, 128, 256], "big_img": [1, 12, 128, 256]},
  "output_slices": {
    "plan": slice(0, 495),
    "lane_lines": slice(495, 759),
    "road_edges": slice(759, 957),
    "lead": slice(957, 1005),
    "lead_prob": slice(1005, 1006),
    "desire_state": slice(1006, 1014),
    "meta": slice(1014, 1044),
    "desire_pred": slice(1044, 1076),
    "pose": slice(1076, 1082),
    "wide_from_device_euler": slice(1082, 1085),
    "road_transform": slice(1085, 1091),
    "hidden_state": slice(1091, 1603),
  },
  "vision_input_names": ["img", "big_img"],
}


def _make_parsed(outputs: dict) -> dict:
  d = {}
  if "plan" in outputs:
    d["plan"] = outputs["plan"]
    d["plan_stds"] = np.ones_like(outputs["plan"]) * 0.1
  if "lane_lines" in outputs:
    d["lane_lines"] = outputs["lane_lines"]
    d["lane_lines_stds"] = np.ones_like(outputs["lane_lines"]) * 0.1
  if "road_edges" in outputs:
    d["road_edges"] = outputs["road_edges"]
    d["road_edges_stds"] = np.ones_like(outputs["road_edges"]) * 0.1
  if "lead" in outputs:
    d["lead"] = outputs["lead"]
  if "lead_prob" in outputs:
    d["lead_prob"] = outputs["lead_prob"]
  if "meta" in outputs:
    d["meta"] = outputs["meta"]
  if "desire_pred" in outputs:
    d["desire_pred"] = outputs["desire_pred"]
  if "desire_state" in outputs:
    d["desire_state"] = outputs["desire_state"]
  if "pose" in outputs:
    d["pose"] = outputs["pose"]
  return d


class TestFusionModelState(unittest.TestCase):
  def _make(self):
    f = FusionModelState(1928, 1208, META)
    f._session_start_ts = time.monotonic()
    return f

  def test_cold_start_ramp(self):
    f = self._make()
    f._last_big_staleness_ms = 0.0
    self.assertAlmostEqual(f._compute_weight(), 0.0, delta=0.01)
    f._session_start_ts = time.monotonic() - 1.0
    self.assertAlmostEqual(f._compute_weight(), 0.5, delta=0.05)
    f._session_start_ts = time.monotonic() - 3.0
    self.assertAlmostEqual(f._compute_weight(), 1.0, delta=0.05)

  def test_staleness_decay(self):
    f = self._make()
    f._session_start_ts = time.monotonic() - 3.0
    f._last_big_staleness_ms = 150.0
    w = f._compute_weight()
    self.assertAlmostEqual(w, np.exp(-1.0), delta=0.01)

  def test_blend_plan_time_splice(self):
    f = self._make()
    f._session_start_ts = time.monotonic() - 3.0
    f._last_big_staleness_ms = 200.0
    w = 1.0

    small_plan = np.zeros((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32)
    big_plan = np.ones((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32)
    blended = f._blend_plan(small_plan, big_plan, w)

    near_idx = next(i for i, t in enumerate(T_IDXS) if t >= 0.05)
    self.assertLess(blended[0, near_idx, 0].item(), 0.4)

    far_idx = next(i for i, t in enumerate(T_IDXS) if t >= 1.0)
    self.assertGreater(blended[0, far_idx, 0].item(), 0.7)

    trans_idx = next(i for i, t in enumerate(T_IDXS) if t >= 0.2)
    v = blended[0, trans_idx, 0].item()
    self.assertGreater(v, 0.2)
    self.assertLess(v, 0.8)

  def test_blend_by_distance(self):
    f = self._make()
    f._session_start_ts = time.monotonic() - 3.0
    w = 0.5

    small = np.zeros((1, ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, ModelConstants.LANE_LINES_WIDTH), dtype=np.float32)
    big = np.ones((1, ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, ModelConstants.LANE_LINES_WIDTH), dtype=np.float32)

    # 动态边界: v_ego=0 时 threshold=MIN(10m)，10m 内纯小、以外按 w 混合
    blended = f._blend_by_distance(small, big, w, v_ego_mps=0.0)
    near_idx = next(i for i, x in enumerate(X_IDXS) if x >= 5.0)
    self.assertAlmostEqual(blended[0, 0, near_idx, 0].item(), 0.0, delta=1e-5)
    far_idx = next(i for i, x in enumerate(X_IDXS) if x >= 60.0)
    self.assertAlmostEqual(blended[0, 0, far_idx, 0].item(), 0.5, delta=1e-5)

    # 动态边界: v_ego=30, staleness=0.1s+margin 0.5s -> threshold=18m，12m 处仍纯小，30m 处混合
    f._last_big_staleness_ms = 100.0
    blended = f._blend_by_distance(small, big, w, v_ego_mps=30.0)
    mid_idx = next(i for i, x in enumerate(X_IDXS) if x >= 12.0)
    self.assertAlmostEqual(blended[0, 0, mid_idx, 0].item(), 0.0, delta=1e-5)
    far_idx = next(i for i, x in enumerate(X_IDXS) if x >= 30.0)
    self.assertAlmostEqual(blended[0, 0, far_idx, 0].item(), 0.5, delta=1e-5)

    # 上限: v_ego=50 -> 30m 超 MAX 也会被 clip 到 80m 内
    blended = f._blend_by_distance(small, big, w, v_ego_mps=50.0)
    far_idx = next(i for i, x in enumerate(X_IDXS) if x >= 100.0)
    self.assertAlmostEqual(blended[0, 0, far_idx, 0].item(), 0.5, delta=1e-5)

  def test_fuse_big_priority_fields(self):
    f = self._make()
    f._session_start_ts = time.monotonic() - 3.0
    w = 1.0

    small = _make_parsed({
      "plan": np.zeros((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32),
      "lane_lines": np.zeros((1, ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, 2), dtype=np.float32),
      "lead": np.zeros((1, 6, 4), dtype=np.float32),
      "lead_prob": np.zeros((1, 1), dtype=np.float32),
      "meta": np.zeros((1, 30), dtype=np.float32),
      "desire_pred": np.zeros((1, 4, 8), dtype=np.float32),
      "desire_state": np.zeros((1, 8), dtype=np.float32),
      "pose": np.zeros((1, 6), dtype=np.float32),
    })
    big = _make_parsed({
      "plan": np.ones((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32),
      "lane_lines": np.ones((1, ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, 2), dtype=np.float32),
      "lead": np.ones((1, 6, 4), dtype=np.float32),
      "lead_prob": np.ones((1, 1), dtype=np.float32),
      "meta": np.ones((1, 30), dtype=np.float32),
      "desire_pred": np.ones((1, 4, 8), dtype=np.float32),
      "desire_state": np.ones((1, 8), dtype=np.float32),
      "pose": np.ones((1, 6), dtype=np.float32),
    })

    fused = f._fuse(small, big, w)
    self.assertAlmostEqual(fused["lead"][0, 0, 0].item(), 1.0, delta=1e-5)
    self.assertAlmostEqual(fused["meta"][0, 0].item(), 1.0, delta=1e-5)
    self.assertAlmostEqual(fused["desire_pred"][0, 0, 0].item(), 1.0, delta=1e-5)
    self.assertAlmostEqual(fused["desire_state"][0, 0].item(), 1.0, delta=1e-5)
    self.assertAlmostEqual(fused["pose"][0, 0].item(), 0.0, delta=1e-5)

  def test_fuse_no_big_returns_small(self):
    f = self._make()
    f._last_big_parsed = None
    small = _make_parsed({"plan": np.zeros((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH), dtype=np.float32)})
    fused = f._fuse(small, {}, 0.0)
    self.assertTrue(np.array_equal(fused["plan"], small["plan"]))


if __name__ == "__main__":
  unittest.main()

class _FakeParams:
  def __init__(self):
    self.writes = []

  def put(self, key, val, block=False):
    self.writes.append((key, val))


class TestPublishUiState(unittest.TestCase):
  def _make(self):
    f = FusionModelState(1928, 1208, META)
    f._session_start_ts = time.monotonic() - 3.0  # past ramp
    f._params = _FakeParams()
    return f

  def _state_writes(self, f):
    return [v for k, v in f._params.writes if k == "RemoteModelState"]

  def test_connecting_before_first_result(self):
    f = self._make()
    f._publish_ui_state(0.0)
    self.assertEqual(self._state_writes(f), [0])

  def test_active_when_fresh_and_high_weight(self):
    f = self._make()
    f._last_big_ok_ts = time.monotonic()
    f._publish_ui_state(0.85)
    self.assertEqual(self._state_writes(f), [1])

  def test_weak_when_stale_or_low_weight(self):
    f = self._make()
    f._last_big_ok_ts = time.monotonic() - 1.2  # stale but not down
    f._publish_ui_state(0.9)
    self.assertEqual(self._state_writes(f), [2])
    f2 = self._make()
    f2._last_big_ok_ts = time.monotonic()
    f2._publish_ui_state(0.3)  # fresh but low weight
    self.assertEqual(self._state_writes(f2), [2])

  def test_down_after_2s_silence(self):
    f = self._make()
    f._last_big_ok_ts = time.monotonic() - 3.0
    f._publish_ui_state(0.0)
    self.assertEqual(self._state_writes(f), [3])

  def test_writes_throttled_on_unchanged_values(self):
    f = self._make()
    f._last_big_ok_ts = time.monotonic()
    f._publish_ui_state(0.851)
    f._publish_ui_state(0.852)  # same state, same rounded pct
    f._publish_ui_state(0.852)
    self.assertEqual(len(f._params.writes), 2)  # one state + one weight
    f._publish_ui_state(0.86)   # pct changed -> one more weight write
    self.assertEqual(len(f._params.writes), 3)
    self.assertEqual(f._params.writes[-1], ("RemoteModelFusionWeight", "0.86"))
