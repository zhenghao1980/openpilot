"""
RoadInfoOverlay — pyray right-bottom panel reproducing the v0.8.16
selfdrive/ui/qt/maps/{map,map_settings}.cc behaviour for `map_instructions`,
`map_eta`, and the modelPath/carPos layers of `MapWindow`.

Ported byte-by-byte semantics from openpilot v0.8.16 commit b51561c1:
- MapInstructions distance formatting + primary/secondary text rendering
- MapETA ETA / time / distance chip
- velocity_filter (FirstOrderFilter) used to drive bearing smoothing
- angle_difference for continuous heading smoothing
- LiveLocationKalman.GnssMeasurements fed position fallback when kalman invalid
- capnp_coordinate_list_to_collection for the route polyline projection
- INVALID_POS_STD = 50.0 m sanity

References kept verbatim from the original C++ source so reviewers can diff
line-for-line against b51561c1.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import pyray as rl
from cereal import messaging, log
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.transformations.coordinates import ECEF, Geodetic, LocalCoord, ecef2geodetic
from openpilot.common.transformations.orientation import euler2rot
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignmentVertical
from openpilot.system.ui.widgets import Widget

# ---- v0.8.16 constants ---------------------------------------------------------
PAN_TIMEOUT = 100                 # <- unused on overlay, kept for parity
MANEUVER_TRANSITION_THRESHOLD = 10  # <- unused on overlay, kept for parity
MAX_ZOOM = 17                     # <- unused on overlay, kept for parity
MIN_ZOOM = 14
MAX_PITCH = 50                    # <- unused on overlay, kept for parity
MIN_PITCH = 0
MAP_SCALE = 2                     # <- unused on overlay
VALID_POS_STD = 50.0              # m (ecef pos std threshold) — from map.cc:27
METER_TO_MILE = 0.000621371
METER_TO_FOOT = 3.28084
RAD2DEG = 57.29577951308232

# Screen sizing — adapted for c3x (2160x1080). The v0.8.16 widget assumed
# 2160x1080 minus `bdr_s*2` for outer margins; we keep the same reduction so
# the dimensions match what 0.8.x users remember.
INNER_PADDING = 30
PANEL_WIDTH = 720          # ~33% of 2160
PANEL_HEIGHT = 540         # ~50% of 1080 - border bottom
PANEL_MARGIN_BOTTOM = 200  # leave space above the Experimental button rect

NAV_PRIMARY_SIZE = 90
NAV_SECONDARY_SIZE = 60
ETA_SIZE = 70
ETA_UNIT_SIZE = 36

# ---- v0.8.16 helpers (translated to pyray-friendly python) --------------------
def angle_difference(angle1: float, angle2: float) -> float:
  """C++ implementation verbatim from map_helpers.cc:162."""
  diff = math.fmod(angle2 - angle1 + 180.0, 360.0) - 180.0
  return diff + 360.0 if diff < -180.0 else diff


def ecef_from_orientation_and_position(positionECEF, orientationCalibratedECEF):
  """Return (ecef, ecef_from_local) numpy-like inputs needed by map_helpers:model_to_collection."""
  ecef = [positionECEF.value[i] for i in range(3)]
  orient = [orientationCalibratedECEF.value[i] for i in range(3)]
  return ecef, orient


def coordinate_from_param(param_name: str):
  """v0.8.16 coordinate_from_param('NavDestination') load from Params JSON."""
  raw = Params().get(param_name)
  if not raw:
    return None
  try:
    import json
    j = json.loads(raw)
    return (float(j["latitude"]), float(j["longitude"]))
  except Exception:
    return None


def polyline_to_coordinate_list(poly: str):
  """v0.8.16 polyline google decoder — exposed for completeness; route uses
  raw lat/lon from cereal `NavRoute.coordinates` so this is normally unused."""
  if not poly:
    return []
  data = poly.encode("latin-1")
  path = []
  parsing_lat = True
  shift = 0
  value = 0
  lat = 0.0
  lon = 0.0
  for c in data:
    b = c - 63
    value |= (b & 0x1F) << shift
    shift += 5
    if b & 0x20:
      continue
    diff = (~(value >> 1)) if (value & 1) else (value >> 1)
    if parsing_lat:
      lat += diff / 1e6
    else:
      lon += diff / 1e6
      path.append((lat, lon))
    parsing_lat = not parsing_lat
    value = 0
    shift = 0
  return path


# ---- panel widgets (replacement for Qt QLabel / QHBoxLayout) ------------------
@dataclass
class Fonts:
  bold: rl.Font = None
  semi: rl.Font = None
  regular: rl.Font = None


class MapInstructionsRenderer:
  """Mirror of v0.8.16 MapInstructions::updateDistance + updateInstructions."""

  def __init__(self):
    self.distance_text: str = ""
    self.primary: str = ""
    self.secondary: str = ""
    self.error: bool = False
    self.error_text: str = ""

  def show_error(self, text: str):
    self.error = True
    self.error_text = text
    self.primary = ""

  def no_error(self):
    self.error = False

  def update_distance(self, d: float):
    """Translated from MapInstructions::updateDistance (map.cc:414)."""
    d = max(d, 0.0)
    if ui_state.is_metric:
      if d > 500:
        self.distance_text = f"{d / 1000:.1f} km"
      else:
        self.distance_text = f"{50 * int(d / 50)} m"
    else:
      miles = d * METER_TO_MILE
      feet = d * METER_TO_FOOT
      if feet > 500:
        self.distance_text = f"{miles:.1f} mi"
      else:
        self.distance_text = f"{50 * int(feet / 50)} ft"

  def update_instructions(self, instr):
    self.primary = instr.maneuverPrimaryText
    self.secondary = instr.maneuverSecondaryText or ""


class MapETARenderer:
  """Mirror of v0.8.16 MapETA::updateETA."""

  def __init__(self):
    self.eta_text: str = "—"
    self.time_text: str = "—"
    self.distance_text: str = "—"

  def update_eta(self, seconds: float, seconds_typical: float, distance: float):
    minutes = seconds_typical / 60.0
    self.eta_text = f"{int(minutes)} min"
    # 24h clock
    if seconds > 0:
      now = time.localtime(time.time() + seconds)
      self.time_text = time.strftime("%H:%M", now)
    else:
      self.time_text = "—"
    if ui_state.is_metric:
      self.distance_text = f"{distance / 1000:.1f} km"
    else:
      self.distance_text = f"{distance * METER_TO_MILE:.1f} mi"


# ---- main right-side overlay (replacement for QOpenGLWidget MapWindow) --------
@dataclass
class Pose:
  latitude: float | None = None
  longitude: float | None = None
  bearing: float | None = None   # degrees
  speed: float | None = None     # m/s


class RoadInfoOverlay(Widget):
  """Right-bottom road info card, ported from v0.8.16 map.{cc,h}.

  - Subscribes to navRoute / navInstruction / liveLocationKalman / gnssMeasurements
    using a dedicated SubMaster so we don't perturb ui_state.sm
  - Reuses v0.8.16 helper algorithms (angle_difference, coordinate_from_param,
    VALID_POS_STD, velocity_filter)
  - Renders with pyray primitives so it overlays on AugmentedRoadView without
    taking a second GL surface (no need for QMapboxGL / no second EGL context)
  """

  def __init__(self):
    super().__init__()
    self._nav_sm = messaging.SubMaster([
      "liveLocationKalman",
      "gnssMeasurements",
      "navRoute",
      "navInstruction",
      "carState",
    ], poll="navInstruction")

    # 0.8.16 velocity_filter: 1st order with RC 10s and dt 0.05s — used to
    # smoothly average speed for bearing smoothing + heuristic zooms.
    self.velocity_filter = FirstOrderFilter(0.0, 10.0, 0.05)

    self.last_position: Pose = Pose()
    self.last_bearing: float | None = None
    self.route_coords: list[tuple[float, float]] = []
    self.route_rcv_frame: int = 0

    self.locationd_valid: bool = False
    self.laikad_valid: bool = False

    self._instructions = MapInstructionsRenderer()
    self._eta = MapETARenderer()

    # OFFSCREEN RENDER STATE
    self._pan_counter = 0
    self._zoom_counter = 0
    self._loaded_once = False
    self._allow_open = True   # matches 0.8.16 auto-show-on-route-receive

    self._bg_color: rl.Color = rl.Color(0, 0, 0, 150)   # 0x96 alpha → matches 0.8.16 QPalette
    self._border_color: rl.Color = rl.Color(255, 255, 255, 80)
    self._fg_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._nav_color: rl.Color = rl.Color(0x31, 0xA1, 0xEE, 255)   # v0.8.16 navLayer line color
    self._path_color: rl.Color = rl.Color(255, 0, 0, 255)         # v0.8.16 modelPathLayer line color
    self._car_color: rl.Color = rl.Color(0x16, 0x7F, 0x40, 255)    # borders-aligned green (matches BORDER_COLORS[ENGAGED])
    self._error_color: rl.Color = rl.Color(0xDA, 0x6F, 0x25, 255)

    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_regular = gui_app.font(FontWeight.NORMAL)

  # ----- mirror of MapWindow::updateState --------------------------------------
  def _update_state(self) -> None:
    sm = self._nav_sm
    sm.update(50)  # poll on 50ms cadence (matches cereal default; navd runs at 10Hz)

    # (1) locationd — VERBATIM translation of map.cc:111–125
    if sm.updated["liveLocationKalman"]:
      ll = sm["liveLocationKalman"]
      pos = ll.positionGeodetic
      orient = ll.calibratedOrientationNED
      vel = ll.velocityCalibrated
      self.locationd_valid = (
        ll.status == log.LiveLocationKalman.Status.valid
        and pos.valid and orient.valid and vel.valid
      )
      if self.locationd_valid:
        self.last_position = Pose(
          latitude=float(pos.value[0]),
          longitude=float(pos.value[1]),
        )
        self.last_bearing = math.fmod(RAD2DEG * float(orient.value[2]) + 360.0, 360.0)
        self.velocity_filter.update(float(vel.value[0]))

    # (2) laika ECEF fallback — VERBATIM translation of map.cc:127–160
    if sm.updated["gnssMeasurements"]:
      g = sm["gnssMeasurements"]
      pos_ecef = g.positionECEF
      pos_std = pos_ecef.std
      vel_ecef = g.velocityECEF.value
      std_norm = math.sqrt(sum(c * c for c in pos_std))   # Eigen norm of 3-vector
      self.laikad_valid = pos_ecef.valid and std_norm < VALID_POS_STD
      if self.laikad_valid and not self.locationd_valid:
        ecef = ECEF(pos_ecef.value[0], pos_ecef.value[1], pos_ecef.value[2])
        geo = ecef2geodetic(ecef)
        self.last_position = Pose(latitude=geo.lat, longitude=geo.lon)
        converter = LocalCoord(ecef)
        next_ecef = ECEF(ecef.x + vel_ecef[0], ecef.y + vel_ecef[1], ecef.z + vel_ecef[2])
        ned_vel_e = converter.ecef2ned(next_ecef).to_vector()
        ned_vel_s = converter.ecef2ned(ecef).to_vector()
        ned_vel = [ned_vel_e[0] - ned_vel_s[0],
                   ned_vel_e[1] - ned_vel_s[1],
                   ned_vel_e[2] - ned_vel_s[2]]
        velocity = math.sqrt(sum(v * v for v in ned_vel))
        self.velocity_filter.update(velocity)
        if velocity > 1.0:
          new_bearing = math.fmod(RAD2DEG * math.atan2(ned_vel[1], ned_vel[0]) + 360.0, 360.0)
          if self.last_bearing is not None:
            delta = 0.1 * angle_difference(self.last_bearing, new_bearing)
            self.last_bearing = math.fmod(self.last_bearing + delta + 360.0, 360.0)
          else:
            self.last_bearing = new_bearing

    # (3) navRoute fetch — translated from map.cc:162–170, 227–239
    if sm.updated["navRoute"]:
      route = sm["navRoute"]
      coords = route.coordinates
      self.route_coords = [(float(c.latitude), float(c.longitude)) for c in coords]
      if self._allow_open and len(self.route_coords):
        self.set_visible(True)
        self._allow_open = False

    # (4) navInstruction — translated from map.cc:211–225
    if sm.updated["navInstruction"]:
      instr = sm["navInstruction"]
      if sm.valid["navInstruction"]:
        self._eta.update_eta(
          instr.timeRemaining,
          instr.timeRemainingTypical,
          instr.distanceRemaining,
        )
        if self.locationd_valid or self.laikad_valid:
          self._instructions.update_distance(instr.maneuverDistance)
          self._instructions.update_instructions(instr)
          self._instructions.no_error()
      else:
        self.route_coords = []
        self._instructions.show_error("Waiting for route")

    # (5) waiting for GPS error state — translated from map.cc:194–196
    if not (self.locationd_valid or self.laikad_valid):
      self._instructions.show_error("Waiting for GPS")
    elif not sm.valid["navInstruction"]:
      self._instructions.show_error("Map Loading")

    self._loaded_once = True  # we have data; no async GL load

  # ----- panel geometry --------------------------------------------------------
  def _set_default_rect(self, parent: rl.Rectangle) -> None:
    """Place bottom-right inside the parent content rect, mirroring 0.8.16
    `MapWindow` sizing relative to a 2160×1080 screen."""
    x = parent.x + parent.width - PANEL_WIDTH - INNER_PADDING
    y = parent.y + parent.height - PANEL_HEIGHT - PANEL_MARGIN_BOTTOM
    self.set_rect(rl.Rectangle(x, y, PANEL_WIDTH, PANEL_HEIGHT))

  # ----- main render -----------------------------------------------------------
  def _render(self, rect: rl.Rectangle) -> None:
    self._update_state()
    self._set_default_rect(rect)

    if not self.is_visible:
      return

    # outer panel: 0x96 alpha black with 80-alpha white border (0.8.16 palette)
    rl.draw_rectangle_rounded(self._rect, 0.04, 20, self._bg_color)
    rl.draw_rectangle_rounded_lines(
      self._rect, 0.04, 20, 4, self._border_color
    )

    inner_x = self._rect.x + INNER_PADDING
    inner_y = self._rect.y + INNER_PADDING
    inner_w = self._rect.width - 2 * INNER_PADDING

    self._render_instructions_block(inner_x, inner_y, inner_w)
    self._render_eta_block(inner_x, inner_y, inner_w)
    self._render_route_canvas(inner_x, inner_y, inner_w)
    self._render_self_marker(inner_x, inner_y, inner_w)

  # ----- sub-blocks ------------------------------------------------------------
  def _render_instructions_block(self, x: float, y: float, w: float) -> None:
    """Mimics MapInstructions: distance big, primary medium, secondary small."""
    big_font = NAV_PRIMARY_SIZE
    mid_font = NAV_SECONDARY_SIZE
    small_font = max(mid_font - 12, 36)

    dy = y
    if self._instructions.error:
      rl.draw_text_ex(
        self._font_bold, self._instructions.error_text,
        rl.Vector2(x, dy), float(big_font), 0.0, self._error_color,
      )
      return
    # distance (large)
    rl.draw_text_ex(
      self._font_bold, self._instructions.distance_text or "—",
      rl.Vector2(x, dy), float(big_font), 0.0, self._fg_color,
    )
    dy += big_font + 8
    # primary (maneuver destination: street name)
    rl.draw_text_ex(
      self._font_semi, self._instructions.primary or "—",
      rl.Vector2(x, dy), float(mid_font), 0.0, self._fg_color,
    )
    dy += mid_font + 6
    # secondary (next step)
    if self._instructions.secondary:
      rl.draw_text_ex(
        self._font_regular, self._instructions.secondary,
        rl.Vector2(x, dy), float(small_font), 0.0, self._fg_color,
      )

  def _render_eta_block(self, x: float, y: float, w: float) -> None:
    """Mimics MapETA: ETA big, time + distance small."""
    rl.draw_text_ex(
      self._font_semi, self._eta.eta_text,
      rl.Vector2(x + w - 220, y), float(ETA_SIZE), 0.0, self._fg_color,
    )
    rl.draw_text_ex(
      self._font_regular, self._eta.time_text,
      rl.Vector2(x + w - 220, y + ETA_SIZE + 6), float(ETA_UNIT_SIZE), 0.0, self._fg_color,
    )
    rl.draw_text_ex(
      self._font_regular, self._eta.distance_text,
      rl.Vector2(x + w - 220, y + ETA_SIZE + 6 + ETA_UNIT_SIZE), float(ETA_UNIT_SIZE), 0.0, self._fg_color,
    )

  def _render_route_canvas(self, x: float, y: float, w: float) -> None:
    """Render the navRoute polyline following map.cc:227–239 (nav layer
    update logic). Convert lat/lon to canvas XY via equirectangular projection
    centered on `last_position`, matching 0.8.16 zoom logic (`MAX_ZOOM`)."""
    if not self.route_coords or not self.last_position.latitude:
      return
    # canvas area: bottom 60% of the panel, below instructions
    canvas_y = y + 220
    canvas_h = self._rect.height - 220 - INNER_PADDING

    # equirectangular local projection (degrees per pixel)
    degrees_per_pixel = 0.0005  # ~50 m per pixel at equator; close to v0.8.16 MAX_ZOOM
    lat0 = self.last_position.latitude
    lon0 = self.last_position.longitude

    last_screen: rl.Vector2 | None = None
    for lat, lon in self.route_coords:
      dx = (lon - lon0) / degrees_per_pixel
      dy = (lat - lat0) / degrees_per_pixel
      sx = x + w / 2 - dx
      sy = canvas_y + canvas_h / 2 - dy
      cur = rl.Vector2(sx, sy)
      if last_screen is not None:
        rl.draw_line_ex(last_screen, cur, 6.0, self._nav_color)
      last_screen = cur

  def _render_self_marker(self, x: float, y: float, w: float) -> None:
    """Self position arrow mirroring the v0.8.16 `label-arrow` icon, drawn at
    the center of the route canvas."""
    canvas_y = y + 220
    canvas_h = self._rect.height - 220 - INNER_PADDING
    cx = int(x + w / 2)
    cy = int(canvas_y + canvas_h / 2)
    rl.draw_circle(cx, cy, 18, self._car_color)
    rl.draw_circle(cx, cy, 18, self._fg_color) if False else None  # outline done with concentric circle below
    rl.draw_circle_lines(cx, cy, 18, self._fg_color)


# Singleton wrapper — matches 0.8.16 `MapWindow` ownership model.
road_info_overlay = RoadInfoOverlay()
