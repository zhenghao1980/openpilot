# SPDX-License-Identifier: MIT
# Portions derived from sunnypilot (https://github.com/sunnyhaibin/sunnypilot) - MIT License
# Portions derived from dragonpilot (https://github.com/dragonpilot-community/dragonpilot) - MIT License
"""SCC-X: fused curve-speed control (Smart Cruise Control - eXperimental).

Combines the strengths of sunnypilot's SCC-V (direct model lateral-acceleration
prediction) and dragonpilot's VisionTurnController (lane-line polynomial
curvature with quality confidence, plus overshoot protection) behind a
confidence-gated, hysteresis-smoothed arbiter and a single state machine.

SCC-M (offline map based curve speed) is intentionally NOT implemented here;
map.py provides the entry point / framework only.

This package deliberately imports nothing but numpy/math so the estimators can
be unit-tested without the cereal/messaging stack.
"""

from openpilot.selfdrive.controls.lib.scc.controller import SccXController
from openpilot.selfdrive.controls.lib.scc.map import MapCurveEstimator

__all__ = ["SccXController", "MapCurveEstimator"]
