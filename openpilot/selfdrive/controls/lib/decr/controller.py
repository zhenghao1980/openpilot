# SPDX-License-Identifier: MIT
"""DEC-R (Radar Deceleration) fusion controller.

Spec: op-model-outputs.html chapter 15 (DEC-R 原厂雷达融合方案, 2026-09-17
定稿, B8PA data-backed). The stock J428 radar keeps running its full ACC stack
while openpilot longitudinal is engaged; panda blocks its ACC_01 commands from
the powertrain, but the radar-side bus copy is a free observation tap. DEC-R
fuses that deceleration request into OP longitudinal:

  a_out = min(a_op, a_radar_eff)        -- can only brake, never accelerate

Three iron rules (chapter 15 §2):
  1) min() only: the radar can never make the car faster than OP planned;
  2) the lock ticket (ACC_02 Relevantes_Objekt != 0 AND Abstandsindex < 1000)
     is mandatory - unlocked negative commands are all "regulate toward the
     stored set speed" (measured p10 -1.58 in the R3b cell), listening to them
     would import phantom braking;
  3) it cannot see stationary obstacles - not a stationary-hazard feature.

ANB yield protocol (chapter 14) always wins: stockAeb set -> DEC-R yields the
same frame.

This package deliberately imports nothing but numpy/math so the controller can
be unit-tested without the cereal/messaging stack (same pattern as scc).
"""
import math
from collections import deque
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.decr.constants import (
  ABIDX_NO_TARGET, APPROACH_A_REQ_TH, APPROACH_V_CLOSE_TH, ARM_TIME_S,
  BAND_B1_MIN_KPH, BAND_B3_MAX_KPH, BAND_HYST_KPH, BASE_CLIP,
  CURVE_CORROB_KAPPA, CURVE_VISION_PROB, CUTIN_SUPPRESS_S, CUTIN_WINDOW_S,
  CUTIN_X_DROP_M, DECR_ENABLED_PARAM, DT_CTRL, EXIT_HOLD_S, EXIT_HOLD_TAU,
  FAR_X_M, FF_HORIZON_S, GRAD_FALLBACK_NEG, GRAD_FALLBACK_POS,
  LOCK_DISARM_S, OVERCRUISE_BORROW, OVERCRUISE_MARGIN_KPH, R1_DEADZONE,
  R1_EXCESS_MIN, R1_PERSIST_S, R2_PERSIST_SCALE, R3A_BUDGET_S, R3A_CLIP,
  R3A_FCW_SOLL_TH, R3A_HAND_BACK_V, R3A_MIN_LOCK_AGE_S, T1_ABIDX_RISE,
  T1_ABIDX_WINDOW_S, T1_CLIP, T1_CONFIRM_S, T1_FCW_HOLD_S, T1_FCW_SOLL_TH,
  T1_SOLL_TH, T2_DIST_M, T2_FAR_CAP, T2_V_CLOSE_TH, TRUST_MAX_NOTCH,
  TRUST_SOLL_TH, TRUST_VISION_EMPTY_PROB, TRUST_WINDOW_S, V2_MIN_KPH,
  VISION_LOST_PROB, VISION_STEADY_PROB, abidx_to_meters,
)

PARAMS_UPDATE_PERIOD = 1.0  # s
KPH_TO_MS = 1.0 / 3.6
MS_TO_KPH = 3.6

# Trust downgrade shifts the band blend this much toward conservative per notch
# (1.0 = full B1 params, 0.0 = full B3 params; one notch halves the blend).
TRUST_NOTCH_STEP = 0.5


@dataclass
class RadarInput:
  """J428 radar feed, populated by CarState from the radar-side bus."""
  soll: float = 0.0            # ACC_Sollbeschleunigung [m/s^2], sample-hold staircase
  neg_grad: float = 0.0        # ACC_neg_Sollbeschl_Grad [m/s^3]
  pos_grad: float = 0.0        # ACC_pos_Sollbeschl_Grad [m/s^3]
  status: int = 0              # ACC_Status_ACC
  relevant_obj: int = 0        # ACC_02 ACC_Relevantes_Objekt (0/1/2)
  abstandsindex: int = 1023    # ACC_02 ACC_Abstandsindex (1022/1023 = no target)
  healthy: bool = False        # freshness + counter + rate-limit health (CarState)


@dataclass
class DecrOutput:
  a_target: float | None = None   # fused min()-candidate [m/s^2]; None = silent
  fcw: bool = False               # request an FCW prompt (visual + audible)
  grid: str = "none"              # none | R1 | R2 | R3a | R3b | R3x (budget exit)
  event: str = "none"             # none | T1 | T2
  band: str = "B2"
  locked: bool = False
  armed: bool = False
  healthy: bool = False
  trust_notch: int = 0
  abstandsindex: int = 1023
  soll: float = 0.0
  trust_event: bool = False       # rising edge: a downgrade was applied this frame


class DecrController:
  def __init__(self, is_mlb: bool = False, params=None):
    if params is None:
      from openpilot.common.params import Params  # lazy: keeps module importable in tests
      params = Params()
    self.params = params

    # DEC-R is only meaningful on MLB (B8PA) where the J428 ext-bus feed exists.
    self.is_mlb = is_mlb
    self.enabled = self.params.get_bool(DECR_ENABLED_PARAM)
    self.frame = -1

    # lock ticket
    self._armed = False
    self._lock_age_s = 0.0        # continuous locked time
    self._lock_lost_s = 0.0       # continuous unlocked time (disarm hysteresis)

    # speed band (discrete, hysteresis) + continuous blend factor
    self._band = "B2"
    self._band_t = 0.5            # 0.0 = B3 params, 1.0 = B1 params

    # grid / event state
    self._grid = "none"
    self._r3a_budget_used = 0.0
    self._r3a_blocked = False     # budget exhausted / low-speed hand-back; until vision recovers
    self._excess_hold_s = 0.0     # R1/R2 persistence accumulator
    self._exit_hold_t = 0.0       # remaining exit-hold decay time
    self._exit_hold_val = 0.0     # last effective radar decel at hold start

    # T1 / T2
    self._t1_confirm_s = 0.0
    self._t1_latched = False
    self._t1_fcw_s = 0.0
    self._soll_prev = 0.0
    self._dsoll_dt = 0.0          # lightly filtered derivative of soll

    # cut-in suppression
    self._cutin_s = 0.0
    self._lead_x_hist: deque[tuple[float, float]] = deque()  # (t, lead_x) while vision steady

    # abidx drop history (T1 third condition)
    self._t = 0.0
    self._abidx_hist: deque[tuple[float, int]] = deque()

    # trust monitor
    self._trust_s = 0.0
    self._trust_notch = 0

    # slew-limited output memory
    self._a_eff_prev = 0.0

  # -- helpers ---------------------------------------------------------------

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.enabled = self.params.get_bool(DECR_ENABLED_PARAM)

  def _update_band(self, v_kph: float) -> None:
    # ±5 km/h hysteresis on both boundaries (chapter 15 §8.4)
    if self._band == "B1":
      if v_kph < BAND_B1_MIN_KPH - BAND_HYST_KPH:
        self._band = "B2"
    elif self._band == "B3":
      if v_kph > BAND_B3_MAX_KPH + BAND_HYST_KPH:
        self._band = "B2"
    else:  # B2
      if v_kph >= BAND_B1_MIN_KPH + BAND_HYST_KPH:
        self._band = "B1"
      elif v_kph <= BAND_B3_MAX_KPH - BAND_HYST_KPH:
        self._band = "B3"
    span = BAND_B1_MIN_KPH - BAND_B3_MAX_KPH
    self._band_t = min(max((v_kph - BAND_B3_MAX_KPH) / span, 0.0), 1.0)

  def _lerp(self, pair: tuple[float, float], t: float) -> float:
    b3, b1 = pair
    return b3 + (b1 - b3) * t

  def _t_eff(self) -> float:
    """Band blend after trust-monitor downgrade (one notch = half the blend)."""
    return min(max(self._band_t - TRUST_NOTCH_STEP * self._trust_notch, 0.0), 1.0)

  def _update_lock(self, radar: RadarInput, dt: float, arm_time_s: float) -> None:
    locked_raw = radar.relevant_obj != 0 and radar.abstandsindex < ABIDX_NO_TARGET
    if locked_raw:
      self._lock_age_s += dt
      self._lock_lost_s = 0.0
      if self._lock_age_s >= arm_time_s:
        self._armed = True
    else:
      self._lock_lost_s += dt
      if self._lock_lost_s >= LOCK_DISARM_S:
        self._armed = False
        self._lock_age_s = 0.0

  def _update_cutin(self, lead_prob: float, lead_x: float) -> None:
    # Cut-in signature: the vision lead suddenly gets much closer (new lead
    # identity / lateral narrowing). Suppresses T1/T2 for CUTIN_SUPPRESS_S so
    # the radar's instant re-lock can't over-brake a merge OP could absorb.
    if lead_prob >= VISION_STEADY_PROB and lead_x > 1.0:
      self._lead_x_hist.append((self._t, lead_x))
    while self._lead_x_hist and self._lead_x_hist[0][0] < self._t - CUTIN_WINDOW_S:
      self._lead_x_hist.popleft()
    if self._lead_x_hist:
      x_then = self._lead_x_hist[0][1]
      if lead_prob >= VISION_STEADY_PROB and x_then - lead_x > CUTIN_X_DROP_M:
        self._cutin_s = CUTIN_SUPPRESS_S

  def _abidx_rising(self, abidx: int) -> bool:
    # T1 confirmation edge. The J428 display index is BIG = NEAR (22 m ~ 567,
    # 107 m ~ 130, verified on 82k paired ACC_02/vision frames): a real braking
    # event makes the index RISE as the gap collapses. The old falling-edge
    # gate armed on the radar RELEASING the target - the exact wrong direction.
    self._abidx_hist.append((self._t, abidx))
    while self._abidx_hist and self._abidx_hist[0][0] < self._t - T1_ABIDX_WINDOW_S:
      self._abidx_hist.popleft()
    if not self._abidx_hist:
      return False
    trough = min(a for _, a in self._abidx_hist)
    return abidx < ABIDX_NO_TARGET and trough < ABIDX_NO_TARGET and (abidx - trough) >= T1_ABIDX_RISE

  def _slew_limit(self, cand: float, radar: RadarInput) -> float:
    # Borrow J428's own jerk envelope (chapter 15 §3.1): toward more braking
    # consult neg_grad, toward release consult pos_grad. Non-regulating frames
    # carry 0/0 - fall back to the urban fit so the output can never freeze.
    neg = radar.neg_grad if radar.neg_grad > 0.05 else GRAD_FALLBACK_NEG
    pos = radar.pos_grad if radar.pos_grad > 0.05 else GRAD_FALLBACK_POS
    lo = self._a_eff_prev - neg * DT_CTRL
    hi = self._a_eff_prev + pos * DT_CTRL
    return min(max(cand, lo), hi)

  # -- main entry -------------------------------------------------------------

  def update(self, long_active: bool, gas_pressed: bool, stock_aeb: bool,
             v_ego: float, v_cruise: float, curvature: float,
             lead_prob: float, lead_x: float, lead_v: float,
             radar: RadarInput, a_op: float) -> DecrOutput:
    """Advance one controlsd frame (DT_CTRL = 10 ms).

    long_active: OP longitudinal is controlling this frame
    gas_pressed / stock_aeb: driver override / stock AEB - DEC-R stands down
    v_ego [m/s], v_cruise [m/s], curvature [1/m]
    lead_prob / lead_x [m] / lead_v [m/s]: modelV2.leadsV3[0] (kinematics only
      trusted at prob >= VISION_LOST_PROB)
    radar: J428 feed (RadarInput); a_op: planner's aTarget this frame
    """
    self.frame += 1
    self._t += DT_CTRL
    dt = DT_CTRL
    self._update_params()

    out = DecrOutput(locked=radar.relevant_obj != 0 and radar.abstandsindex < ABIDX_NO_TARGET,
                     healthy=radar.healthy, trust_notch=self._trust_notch,
                     abstandsindex=radar.abstandsindex, soll=radar.soll, band=self._band)

    # Input sanitation: clamp vision probability into [0, 1]; a NaN/inf soll
    # would poison _dsoll_dt and the slew state, so it stands DEC-R down the
    # same way a sick radar does (silent, nothing emitted).
    lead_prob = min(max(lead_prob, 0.0), 1.0)

    # Hard gates: feature/platform switch, radar health (silent degradation),
    # longitudinal active, driver override, and the ANB yield protocol - any
    # failure and DEC-R emits nothing this frame.
    if not (self.enabled and self.is_mlb and radar.healthy and long_active and math.isfinite(radar.soll)) or gas_pressed or stock_aeb:
      if not radar.healthy:
        # can't trust the lock flag from a sick radar: re-arm from scratch
        self._armed = False
        self._lock_age_s = self._lock_lost_s = 0.0
      self._grid = "none"
      self._excess_hold_s = 0.0
      self._exit_hold_t = 0.0
      self._t1_confirm_s = 0.0
      self._t1_latched = False
      self._t1_fcw_s = 0.0
      self._r3a_blocked = False
      self._r3a_budget_used = 0.0
      self._a_eff_prev = 0.0
      return out

    v_kph = v_ego * MS_TO_KPH
    self._update_band(v_kph)
    t_eff = self._t_eff()
    out.band = self._band
    arm_time = self._lerp(ARM_TIME_S, t_eff)
    persist_s = self._lerp(R1_PERSIST_S, t_eff)
    base_clip = self._lerp(BASE_CLIP, t_eff)
    r3a_budget = self._lerp(R3A_BUDGET_S, t_eff)
    t2_dist = self._lerp(T2_DIST_M, t_eff)

    # -- lock ticket -----------------------------------------------------------
    self._update_lock(radar, dt, arm_time)
    out.armed = self._armed

    # -- signal conditioning ----------------------------------------------------
    if self.frame > 0:
      raw_d = (radar.soll - self._soll_prev) / dt
      self._dsoll_dt += 0.2 * (raw_d - self._dsoll_dt)  # light low-pass
    self._soll_prev = radar.soll
    self._update_cutin(lead_prob, lead_x)
    self._cutin_s = max(0.0, self._cutin_s - dt)
    abidx_rise = self._abidx_rising(radar.abstandsindex)
    x_est = abidx_to_meters(radar.abstandsindex) if radar.abstandsindex < ABIDX_NO_TARGET else float('inf')

    # vision kinematics only when the model sees something at all
    vis_ok = lead_prob >= VISION_LOST_PROB and lead_x > 1.0
    v_close = (v_ego - lead_v) if vis_ok else 0.0
    a_req = (v_close ** 2) / (2.0 * lead_x) if (vis_ok and v_close > 0.0) else 0.0

    # corroboration requirements (curves and far targets, chapter 15 §8.5)
    in_curve = abs(curvature) > CURVE_CORROB_KAPPA
    vision_confirmed = lead_prob >= CURVE_VISION_PROB
    far_no_vision = x_est > FAR_X_M and not vision_confirmed
    event_corroborated = (not in_curve or vision_confirmed) and not far_no_vision

    # -- T1: confirmed emergency braking (all bands) ----------------------------
    if self._armed and radar.soll <= T1_SOLL_TH:
      self._t1_confirm_s += dt
    else:
      self._t1_confirm_s = 0.0
    if not self._t1_latched:
      # one-shot confirmation: deep request held + abidx rising (gap collapsing),
      # suppressed around cut-ins and unconfirmable far/curved targets
      self._t1_latched = (self._t1_confirm_s >= T1_CONFIRM_S and abidx_rise and
                          self._cutin_s <= 0.0 and event_corroborated)
    elif radar.soll > T1_SOLL_TH or not self._armed:
      # latch releases when the radar itself releases; the abidx rise is only
      # a confirmation gate, it ages out of its window while the braking event
      # is still ongoing
      self._t1_latched = False
    t1 = self._t1_latched

    # FCW prompt: sustained deep braking request
    if t1 and radar.soll <= T1_FCW_SOLL_TH:
      self._t1_fcw_s += dt
    else:
      self._t1_fcw_s = 0.0
    if self._t1_fcw_s >= T1_FCW_HOLD_S:
      out.fcw = True

    # -- grid classification ------------------------------------------------------
    prev_grid = self._grid
    if not self._armed:
      grid = "R3b" if lead_prob < VISION_LOST_PROB else "none"
    elif lead_prob >= VISION_STEADY_PROB:
      grid = "R1"
    elif lead_prob >= VISION_LOST_PROB:
      grid = "R2"
    else:
      grid = "R3a"

    # R3a entry guard: only from R1/R2, or with a mature lock
    if grid == "R3a" and prev_grid != "R3a" and not (prev_grid in ("R1", "R2") or self._lock_age_s >= R3A_MIN_LOCK_AGE_S):
      grid = "R3b"
    # R3a budget / low-speed hand-back: once spent, stay out until vision recovers
    if grid == "R3a" and (self._r3a_blocked or self._r3a_budget_used >= r3a_budget or v_ego < R3A_HAND_BACK_V):
      self._r3a_blocked = True
      grid = "R3x"
    if lead_prob >= VISION_LOST_PROB:
      self._r3a_blocked = False
      self._r3a_budget_used = 0.0

    # R3a in B3 (t_eff == 0): no full-follow, FCW prompt only
    r3a_follow_allowed = t_eff > 0.0
    if grid == "R3a" and not r3a_follow_allowed:
      if radar.soll <= R3A_FCW_SOLL_TH:
        out.fcw = True
      grid = "R3x"

    self._grid = grid
    out.grid = grid

    # -- candidate ----------------------------------------------------------------
    cand = None
    event = "none"
    if grid in ("R1", "R2"):
      base = radar.soll if t1 else radar.soll + R1_DEADZONE
      approaching = a_req > APPROACH_A_REQ_TH or v_close > APPROACH_V_CLOSE_TH
      # persistence: radar must offer > R1_EXCESS_MIN more decel than OP, held
      # (halved in R2 flicker so a recovering lead doesn't cause a throttle jump)
      gate_s = persist_s * (R2_PERSIST_SCALE if grid == "R2" else 1.0)
      if approaching and (a_op - base) > R1_EXCESS_MIN:
        self._excess_hold_s += dt
      else:
        self._excess_hold_s = 0.0
      if t1 or (approaching and self._excess_hold_s >= gate_s):
        cand = base
        if t1:
          event = "T1"
      # T2 / v2 derivative feedforward: radar reacts 200-400 ms before the
      # vision filter converges; anticipate the staircase with its derivative.
      v2_allowed = v_kph > V2_MIN_KPH and t_eff >= 0.5
      t2_needs_vision = t_eff <= 0.0  # B3: vision corroboration required
      # feedforward fires on the radar's onset ramp or a genuinely large speed
      # delta (2x the R1 approach threshold); a steady close-in stays with the
      # R1 deadzone so T2 doesn't silently replace it
      closing_fast = self._dsoll_dt < -0.5 or v_close > 2 * T2_V_CLOSE_TH
      if (v2_allowed and closing_fast and x_est <= t2_dist and self._cutin_s <= 0.0 and
          event_corroborated and (not t2_needs_vision or vision_confirmed)):
        a_ff = radar.soll + self._dsoll_dt * FF_HORIZON_S
        if far_no_vision:
          a_ff = max(a_ff, T2_FAR_CAP)
        cand = a_ff if cand is None else min(cand, a_ff)
        if cand == a_ff and event == "none":
          event = "T2"
    elif grid == "R3a":
      # fog/night high-value cell: follow the radar with no deadzone
      cand = radar.soll
      if t1:
        event = "T1"

    # clip: T1 widens to -3.5 everywhere; R3a uses its own -2.5
    if cand is not None:
      clip = T1_CLIP if t1 else (R3A_CLIP if grid == "R3a" else base_clip)
      cand = max(cand, clip)
      # stale-setpoint guard: above the stored cruise speed the radar's
      # deceleration may target an outdated setpoint - borrow only half
      if v_ego > v_cruise + OVERCRUISE_MARGIN_KPH * KPH_TO_MS and cand < a_op:
        cand = a_op + OVERCRUISE_BORROW * (cand - a_op)
      cand = self._slew_limit(cand, radar)

    # exit-hold: leaving R1/R2 keeps the last radar decel, decaying, <= 1 s
    if grid in ("R1", "R2") and cand is not None:
      self._exit_hold_val = cand
      self._exit_hold_t = EXIT_HOLD_S
    elif cand is None and self._exit_hold_t > 0.0 and grid in ("R2", "R3a"):
      held = self._exit_hold_val * math.exp(-(EXIT_HOLD_S - self._exit_hold_t) / EXIT_HOLD_TAU)
      cand = min(held, 0.0)
      self._exit_hold_t -= dt
    else:
      self._exit_hold_t = 0.0

    if cand is not None:
      self._a_eff_prev = cand
      out.a_target = cand
    else:
      self._a_eff_prev = 0.0
    out.event = event

    # -- trust monitor ------------------------------------------------------------
    # Radar braking hard while vision sees an empty road for > TRUST_WINDOW_S:
    # lasting contradiction -> one band notch more conservative for this trip
    # (logged via controlsState; the alternative "believe both halfway" is the
    # Tesla mistake this design explicitly avoids).
    if self._armed and radar.soll <= TRUST_SOLL_TH and lead_prob < TRUST_VISION_EMPTY_PROB:
      self._trust_s += dt
      if self._trust_s >= TRUST_WINDOW_S and self._trust_notch < TRUST_MAX_NOTCH:
        self._trust_notch += 1
        out.trust_notch = self._trust_notch
        out.trust_event = True
        self._trust_s = 0.0
    else:
      self._trust_s = 0.0

    return out
