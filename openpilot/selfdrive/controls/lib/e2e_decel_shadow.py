#!/usr/bin/env python3
"""Shadow recorder for e2e no-lead deceleration episodes.

Passive logger only: never touches control outputs. Records episodes where the
winning longitudinal source is e2e, the radar sees no lead, and the planner is
commanding deceleration. Purpose: gather field data on how far ahead of an
actual stop the model starts slowing down, to judge whether a future
"early-decel limiter" is worth building (and how to threshold it).

Output: JSON Lines, one record per closed episode, at LOG_PATH
(rotated to LOG_PATH.1 when exceeding MAX_BYTES).
All I/O is guarded: a logging failure must never affect planning.
"""
import json
import os
import time

LOG_PATH = "/data/e2e_decel_shadow.jsonl"
MAX_BYTES = 5 * 1024 * 1024

# Episode thresholds
A_ENTER = -0.15        # m/s^2, commanded decel to enter/stay in episode
V_ENTER = 2.5          # m/s, min speed to enter (below this is crawl/stop territory)
MIN_DUR_S = 2.0        # shorter episodes are not worth logging
HOLDOFF_S = 1.0        # conditions must stay false this long to close an episode


class E2EDecelShadow:
  def __init__(self, dt, log_path=LOG_PATH):
    self.dt = dt
    self.log_path = log_path
    self.ep = None          # active episode dict or None
    self.off_since = None   # monotonic time when conditions went false

  # ------------------------------------------------------------------
  def update(self, t, *, experimental, long_active, source_is_e2e, radar_lead,
             a_target, v_ego, gas_pressed, model_should_stop,
             model_v10_ms, model_lead_prob, model_lead_dist_m):
    now = t  # caller passes message mono time (s); keeps replay deterministic
    cond = (experimental and long_active and source_is_e2e and not radar_lead
            and a_target < A_ENTER and v_ego > (V_ENTER if self.ep is None else 0.0))

    if self.ep is None:
      if cond:
        self.ep = dict(
          t_wall=time.time(), dur_s=0.0, dist_m=0.0,
          v_start_kmh=round(v_ego * 3.6, 1), v_min_kmh=round(v_ego * 3.6, 1),
          a_min=a_target,
          vplan10_start_kmh=round(model_v10_ms * 3.6, 1),
          vplan10_min_kmh=round(model_v10_ms * 3.6, 1),
          lead_prob_max=model_lead_prob, lead_dist_min_m=model_lead_dist_m,
          should_stop_frames=0, gas_override_frames=0, ended_standstill=False,
        )
        self.off_since = None
      return

    # episode active
    if cond:
      self.off_since = None
      ep = self.ep
      ep["dur_s"] += self.dt
      ep["dist_m"] += v_ego * self.dt
      ep["v_min_kmh"] = min(ep["v_min_kmh"], round(v_ego * 3.6, 1))
      ep["a_min"] = min(ep["a_min"], a_target)
      ep["vplan10_min_kmh"] = min(ep["vplan10_min_kmh"], round(model_v10_ms * 3.6, 1))
      ep["lead_prob_max"] = max(ep["lead_prob_max"], model_lead_prob)
      if model_lead_dist_m > 0:
        ep["lead_dist_min_m"] = min(ep["lead_dist_min_m"], model_lead_dist_m)
      if model_should_stop:
        ep["should_stop_frames"] += 1
      if gas_pressed and a_target < 0.0:
        ep["gas_override_frames"] += 1
    else:
      if self.off_since is None:
        self.off_since = now
        self.ep["ended_standstill"] = v_ego < 0.3
      elif now - self.off_since >= HOLDOFF_S:
        self._close()

  def flush(self):
    """Close any open episode (call on shutdown / disengage)."""
    if self.ep is not None:
      self._close()

  # ------------------------------------------------------------------
  def _close(self):
    ep, self.ep = self.ep, None
    self.off_since = None
    if ep["dur_s"] < MIN_DUR_S:
      return
    ep["dur_s"] = round(ep["dur_s"], 2)
    ep["dist_m"] = round(ep["dist_m"], 1)
    ep["a_min"] = round(ep["a_min"], 2)
    ep["lead_prob_max"] = round(ep["lead_prob_max"], 2)
    ep["lead_dist_min_m"] = round(ep["lead_dist_min_m"], 1)
    ep["gas_override"] = ep.pop("gas_override_frames") > 0
    ep["t_wall"] = int(ep["t_wall"])
    try:
      self._rotate()
      with open(self.log_path, "a") as f:
        f.write(json.dumps(ep, ensure_ascii=False) + "\n")
    except Exception:
      pass  # logging must never raise into the planner

  def _rotate(self):
    if os.path.exists(self.log_path) and os.path.getsize(self.log_path) > MAX_BYTES:
      try:
        os.replace(self.log_path, self.log_path + ".1")
      except Exception:
        pass
