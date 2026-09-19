# SPDX-License-Identifier: MIT
"""Shared constants for DEC-R (Radar Deceleration fusion).

Spec: op-model-outputs.html chapter 15 (DEC-R 原厂雷达融合方案, 2026-09-17).

DEC-R taps the stock J428 radar's own ACC_01 deceleration command (observed on
the radar-side bus; panda blocks it from the powertrain) and fuses it into
openpilot longitudinal as a min()-candidate: it can only brake EARLIER/SOFTER,
never faster. It is a comfort + continuity enhancement, NOT a safety feature
(the safety floor stays the ANB yield protocol + stock AEB), and it cannot see
stationary obstacles (radar doppler filters them).

Honesty layering (chapter 15 §9): the lock ticket, the R3b veto and the grid
structure are rlog-proven; every numeric threshold below marked "initial value"
is an engineering starting point to be calibrated by on-road A/B.
"""
import numpy as np

# Feature switch param (registered in openpilot/common/params_keys.h)
DECR_ENABLED_PARAM = "DecrEnabled"

DT_CTRL = 0.01  # controlsd timestep [s]

# --- vision grid thresholds (lead probability, modelV2.leadsV3[0].prob) -----
VISION_STEADY_PROB = 0.5   # R1: vision steady at/above this
VISION_LOST_PROB = 0.3     # below this: vision lost (R3a/R3b); between: R2 flicker

# --- lock ticket (chapter 15 §8.2, rlog-proven) ------------------------------
# Locked = ACC_02.Relevantes_Objekt != 0 AND Abstandsindex < 1000.
# 1022/1023 are the "no target" encodings (verified on 61 rlog segments).
ABIDX_NO_TARGET = 1000
LOCK_DISARM_S = 0.3        # lock lost this long -> disarm (hysteresis)
R3A_MIN_LOCK_AGE_S = 1.0   # R3a entry from cold requires lock held this long

# --- speed bands (chapter 15 §8.4): vEgo kph, ±5 km/h hysteresis ------------
# B2 (54-72) linearly blends the continuous parameters between B3 and B1.
BAND_B3_MAX_KPH = 54.0     # initial value, A/B calibrate
BAND_B1_MIN_KPH = 72.0     # initial value, A/B calibrate
BAND_HYST_KPH = 5.0

# Continuous per-band parameters, (B3_value, B1_value); B2 lerps between them.
ARM_TIME_S = (2.0, 0.5)          # lock arm time
R1_PERSIST_S = (0.8, 0.4)        # "radar brakes >0.3 more than OP" hold time
BASE_CLIP = (-1.0, -1.5)         # base decel clip [m/s^2]
R3A_BUDGET_S = (4.0, 6.0)        # continuous independent-braking budget in R3a
T2_DIST_M = (40.0, 80.0)         # T2/feedforward engagement distance

# --- R1 (vision steady + locked) ---------------------------------------------
R1_DEADZONE = 0.2          # a_radar_eff = soll + deadzone (borrow less)
R1_EXCESS_MIN = 0.3        # radar must offer this much more decel than OP [m/s^2]
APPROACH_A_REQ_TH = 0.3    # "approaching": a_req = v_close^2 / 2x [m/s^2]
APPROACH_V_CLOSE_TH = 2.0  # ... or closing speed above this [m/s]
# vEgo > vCruise + margin: the radar's stored set speed may be stale (lever OFF
# clears it, 21s window evidence), so borrow only half of the offered decel.
OVERCRUISE_MARGIN_KPH = 3.0
OVERCRUISE_BORROW = 0.5

# --- R2 (vision flicker + locked) --------------------------------------------
R2_PERSIST_SCALE = 0.5     # persistence gate halved vs R1
# Leaving R1/R2: keep the last effective radar decel, exponentially decaying,
# for at most this long so a one-frame flicker doesn't cause a throttle jump.
EXIT_HOLD_S = 1.0
EXIT_HOLD_TAU = 0.4        # decay time constant [s] (initial value)

# --- R3a (vision lost + locked, fog/night high-value cell) -------------------
R3A_CLIP = -2.5            # [m/s^2]
R3A_HAND_BACK_V = 3.0      # below this vEgo [m/s], hand back to stop logic
# B3 forbids R3a full-follow; while the radar brakes with vision lost there,
# prompt the driver instead (initial value for "braking meaningfully").
R3A_FCW_SOLL_TH = -1.0     # [m/s^2]
# Entry guard: only from R1/R2, or with lock age >= R3A_MIN_LOCK_AGE_S.

# --- T1: confirmed emergency braking (valid in ALL bands) ---------------------
# J428 <= -2.0 held 0.15s + locked + Abstandsindex RISING (big = NEAR:
# 22 m ~ 567, 107 m ~ 130; a real event is the gap collapsing => index jumps up)
T1_SOLL_TH = -2.0          # [m/s^2]
T1_CONFIRM_S = 0.15
T1_CLIP = -3.5             # clip widened (matches opendbc ACCEL_MIN)
T1_ABIDX_RISE = 50         # index units above the 0.5s window trough (~5-8 m of
                           # gap collapse at 60 kph; quantization/noise << 10)
T1_ABIDX_WINDOW_S = 0.5
# Sustained <= -3.0 -> FCW audible prompt
T1_FCW_SOLL_TH = -3.0
T1_FCW_HOLD_S = 0.5

# --- T2 / v2 derivative feedforward (sudden slow car, far detection) ---------
V2_MIN_KPH = 60.0          # feedforward only above this speed
FF_HORIZON_S = 0.3         # anticipation horizon, 200-400ms band (initial value)
T2_V_CLOSE_TH = 2.0        # closing speed that qualifies as T2 [m/s]
FAR_X_M = 80.0             # beyond this without vision lock: no T1 bypass,
T2_FAR_CAP = -0.8          #   and T2/feedforward is capped here [m/s^2]

# --- blind-spot scenario guards (chapter 15 §8.5) -----------------------------
# Cut-in: radar locks instantly and would over-brake for a merge the car could
# absorb smoothly. On a new/closer vision lead, suppress T1/T2 (R1 only).
CUTIN_X_DROP_M = 15.0      # lead distance drop within the window (initial value)
CUTIN_WINDOW_S = 0.5
CUTIN_SUPPRESS_S = 0.8
# Curves: the beam points straight; above this curvature even B1 needs vision
# corroboration for T1/T2.
CURVE_CORROB_KAPPA = 0.01  # [1/m], ~100 m radius (initial value)
CURVE_VISION_PROB = 0.5
# VRU (two-wheelers): lock flag is not reliable for them; without a VRU signal
# on the ext bus there is nothing to key on - T1/T2 require the lock ticket and
# (in B3 / curves) vision, which is the implementable part of the rule.

# --- trust monitor (Tesla lesson: on lasting contradiction, downgrade radar ---
# instead of half-believing it): radar braking hard while vision sees an empty
# road for >2s -> one band notch more conservative for the rest of the trip.
TRUST_SOLL_TH = -1.0       # "strong" radar decel [m/s^2]
TRUST_VISION_EMPTY_PROB = 0.2
TRUST_WINDOW_S = 2.0
TRUST_MAX_NOTCH = 2        # B1 -> B2 -> B3 floor

# --- borrowed OEM jerk envelope (chapter 15 §3.1, rlog-proven) ----------------
# a_radar_eff slew is limited by J428's own neg/pos_Sollbeschl_Grad. Non-
# regulating frames carry 0/0; fall back to the urban fit constants then.
GRAD_FALLBACK_NEG = 3.5    # m/s^3, brake direction is never self-limited
GRAD_FALLBACK_POS = 0.6    # m/s^3, release direction stays soft

# B8 cluster ACC_Abstandsindex -> meters, inverse of the legacy 1D display LUT
# (calibration speed band ~50 kph slice). mlbcan.py now carries the 2D
# (v_ego, distance) table _ABIDX2D; this mirror stays 1D because call sites
# have no v_ego plumbed and DEC-R far_no_vision semantics are tuned against it
# (107 > 80 m threshold). Revisit with a 2D inverse if DEC-R ever takes v_ego.
#   distance m:  22   27   32   37   42   47   52   57   62   67   72   77   82   87   92   97  102  107  115
#   abidx:      567  532  507  488  459  422  414  381  355  326  301  284  275  244  234  200  161  130  130
# 超过 107m 一律按 ≥107m 处理,far_no_vision 判据不受影响(107 > 80m 门限)。
_ABIDX_PT = np.array([567, 532, 507, 488, 459, 422, 414, 381, 355, 326, 301, 284, 275, 244, 234, 200, 161, 130], dtype=float)
_DIST_PT = np.array([22., 27., 32., 37., 42., 47., 52., 57., 62., 67., 72., 77., 82., 87., 92., 97., 102., 107.], dtype=float)


def abidx_to_meters(abidx: float) -> float:
  """Approximate target distance from the stock ACC_02 Abstandsindex.

  Only meaningful for abidx < ABIDX_NO_TARGET (a real target). Piecewise-linear
  inverse of the display LUT; beyond 107 m the index saturates at 130, so the
  result saturates too - treat anything past ~100 m as 'far'.
  """
  # xp must be increasing: index falls as distance grows, so interp on the
  # reversed arrays.
  return float(np.interp(abidx, _ABIDX_PT[::-1], _DIST_PT[::-1]))
