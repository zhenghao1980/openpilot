"""Shared constants for SCC-X.

Threshold lineage:
- Lateral-acceleration state thresholds (1.3/1.1/1.6) and the 97th-percentile
  prediction come from sunnypilot's SmartCruiseControlVision.
- The 20-150 m @ 5 m curvature evaluation window, lane-probability quality
  gating and the overshoot solution come from dragonpilot's
  VisionTurnController (move-fast lineage).
"""
import numpy as np

# Feature switch param (registered in openpilot/common/params_keys.h)
SCC_X_ENABLED_PARAM = "SccXEnabled"

# Do not operate below this speed (20 km/h), same as SP/DP.
MIN_V = 5.6  # m/s

# Predicted lateral acceleration thresholds [m/s^2]
ENTERING_PRED_LAT_ACC_TH = 1.3
ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1
TURNING_LAT_ACC_TH = 1.6
LEAVING_LAT_ACC_TH = 1.3
FINISH_LAT_ACC_TH = 1.1

# Maximum lateral acceleration allowed in turns, per longitudinal personality
# (log.LongitudinalPersonality: aggressive=0, standard=1, relaxed=2).
# Standard matches SP/DP hardcoded 2.0; relaxed/aggressive open the range.
A_LAT_REG_MAX_BY_PERSONALITY = (2.4, 2.0, 1.7)

# Vision-B (lane polynomial) curvature evaluation window
EVAL_STEP = 5.0   # m
EVAL_START = 20.0  # m
EVAL_STOP = 150.0  # m
EVAL_RANGE = np.arange(EVAL_START, EVAL_STOP, EVAL_STEP)

# Vision-B lane quality gate (from DP)
MIN_LANE_PROB = 0.6

# Prediction noise rejection: percentile of predicted lat-acc vector (SP)
PRED_LAT_ACC_PERCENTILE = 97

# Arbiter: minimum vision-B confidence to prefer it over vision-A
CONF_VISION_B_GATE = 0.6

# Arbiter hysteresis, in model frames (DT_MDL = 0.05 s -> 5 frames = 0.25 s,
# 10 frames = 0.5 s). Kills single-frame model hallucinations and map jumps.
HYSTERESIS_DOWN_FRAMES = 5   # confirm before adopting a lower target speed
HYSTERESIS_UP_FRAMES = 10    # confirm before releasing back to cruise
# Speed changes smaller than this [m/s] are treated as "unchanged" so the
# pending counters don't wind up on jitter.
HYSTERESIS_EPS = 0.1

# Minimum accel the controller may command while turning ( comfort floor )
LEAVING_ACC = 0.5  # m/s^2, regain speed after the turn

# ENTERING state smooth deceleration lookup (SP): min decel allowed depends on
# how much lat-acc is predicted ahead.
ENTERING_SMOOTH_DECEL_V = [-0.2, -1.0]
ENTERING_SMOOTH_DECEL_BP = [1.3, 3.0]

# TURNING state acceleration lookup (SP): comfortable accel for the lat-acc felt.
TURNING_ACC_V = [0.5, 0.0, -0.4]
TURNING_ACC_BP = [1.5, 2.3, 3.0]

# No-overshoot velocity margin when building the v_target we publish (SP):
# v_out = v_target + a_target * horizon
NO_OVERSHOOT_TIME_HORIZON = 4.0  # s

# Min speed any curve target may request
CURVE_MIN_SPEED = 2.8  # m/s (~10 km/h)
