# SPDX-License-Identifier: MIT
"""DEC-R: stock radar deceleration fusion (Radar DECEleration - Radar).

Taps the stock J428 radar's own ACC deceleration command (ACC_01 on the
radar-side bus, blocked from the powertrain by panda) and fuses it into
openpilot longitudinal as a min()-only candidate behind a lock ticket, a
vision x lock grid, speed-band parameters and event triggers. Comfort and
continuity enhancement only; the safety floor stays the ANB yield protocol
plus stock AEB. Full spec: op-model-outputs.html chapter 15.

This package deliberately imports nothing but numpy/math so the controller can
be unit-tested without the cereal/messaging stack.
"""

from openpilot.selfdrive.controls.lib.decr.controller import DecrController, DecrOutput, RadarInput

__all__ = ["DecrController", "DecrOutput", "RadarInput"]
