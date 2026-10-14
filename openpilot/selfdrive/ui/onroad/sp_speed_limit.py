"""
Road speed limit sign for the onroad HUD (drawn below the max cruise speed).

Data source: mapdOut (pfeiferj/mapd native service, offline OSM data) — no
navigation required, works anywhere the local OSM tiles cover.

  speedLimit            current road limit [m/s], 0 = unknown
  nextSpeedLimit        next limit change on the predicted path [m/s]
  nextSpeedLimitDistance approximate distance to that change [m]

Sign style: white disc / red ring (Vienna convention, used in CN/EU); the
number turns red when overspeeding. The sign is hidden when the data is
invalid (no tile, no road match, or no maxspeed tag on the way) instead of
showing a wrong value.
"""

import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached

# waySelectionType ordinals that mean "we are on a matched road" (same
# convention as sp_map_panel.py and scc/map.py)
_MATCH_OK = ("current", "predicted", "possible")

METER_TO_FOOT = 3.28084
METER_TO_MILE = 0.000621371


class SpeedLimitRenderer:
  SIGN_SPACING = 24  # px between the set speed box and the sign (vertical gap now)

  def __init__(self):
    import time as _time
    self._last_update = _time.monotonic() - 10.0
    self.speed_limit = 0.0        # m/s
    self.next_speed_limit = 0.0   # m/s
    self.next_speed_limit_dist = 0.0  # m
    self.valid = False

    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_demi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_norm = gui_app.font(FontWeight.NORMAL)

  def update(self) -> None:
    # mapdOut arrives at ~1 Hz; cache the values when a new one lands and keep
    # rendering from the cache in between (same pattern as the map panel).
    import time as _time
    sm = ui_state.sm
    if not (sm.valid["mapdOut"] and sm.updated["mapdOut"]):
      if _time.monotonic() - self._last_update > 3.0:
        self.valid = False
      return

    self._last_update = _time.monotonic()
    self.valid = False
    self.speed_limit = 0.0
    self.next_speed_limit = 0.0
    self.next_speed_limit_dist = 0.0

    out = sm["mapdOut"]
    # TEMP-DEBUG-FLICKER
    import sys
    self._dbg_n = getattr(self, '_dbg_n', 0) + 1
    if self._dbg_n % 60 == 0:
      print(f'[sl-fl] n={self._dbg_n} valid={self.valid} smvalid={sm.valid["mapdOut"]} upd={sm.updated["mapdOut"]} tile={out.tileLoaded} way={out.waySelectionType} sl={out.speedLimit} last={getattr(self, "_last_update", 0):.1f}',
            file=sys.stderr, flush=True)
    if not out.tileLoaded:
      return
    if str(out.waySelectionType) not in _MATCH_OK:
      return

    # mapdOut.speedLimit is m/s; <= 0 means the way has no maxspeed tag
    if 0.0 < out.speedLimit < 255.0 * CV.KPH_TO_MS:
      self.speed_limit = out.speedLimit
      self.valid = True

    if 0.0 < out.nextSpeedLimit < 255.0 * CV.KPH_TO_MS:
      self.next_speed_limit = out.nextSpeedLimit
      self.next_speed_limit_dist = out.nextSpeedLimitDistance

  def render(self, rect: rl.Rectangle) -> None:
    # TEMP-DEBUG-FLICKER
    import sys
    self._dbg_r = getattr(self, '_dbg_r', 0) + 1
    if self._dbg_r % 60 == 0:
      print(f'[sl-fl] render n={self._dbg_r} valid={self.valid} hide={ui_state.hide_v_ego_ui}', file=sys.stderr, flush=True)
    if not self.valid or ui_state.hide_v_ego_ui:
      return

    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    sign_size = UI_CONFIG.set_speed_height - 40
    # 与最大巡航速度指示牌中线对齐：方框 x 算法与 hud_renderer._draw_set_speed 完全一致
    box_x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    x = box_x + (set_speed_width - sign_size) / 2
    y = rect.y + 45 + UI_CONFIG.set_speed_height + self.SIGN_SPACING
    sign_rect = rl.Rectangle(x, y, sign_size, sign_size)

    is_overspeed = ui_state.sm["carState"].vEgo > self.speed_limit

    if ui_state.is_metric:
      self._render_vienna(sign_rect, str(round(self.speed_limit * CV.MS_TO_KPH)), is_overspeed)
    else:
      self._render_mutcd(sign_rect, str(round(self.speed_limit * CV.MS_TO_MPH)), is_overspeed)

    if self._ahead_visible():
      self._draw_ahead_info(sign_rect)

  def _ahead_visible(self) -> bool:
    return self.next_speed_limit > 0.0 and self.next_speed_limit != self.speed_limit

  def _render_vienna(self, rect: rl.Rectangle, val: str, is_overspeed: bool) -> None:
    center = rl.Vector2(rect.x + rect.width / 2, rect.y + rect.height / 2)
    radius = rect.width / 2
    white = rl.WHITE
    red = rl.Color(235, 32, 32, 255)

    rl.draw_circle_v(center, radius, white)
    rl.draw_ring(center, radius * 0.78, radius, 0, 360, 36, red)

    # 字固定黑色、放大加粗（is_overspeed 不再改色，保持用户要求的可读性）。
    # 字体最重只有 Inter-Bold，加粗用 5 次叠印（正中 + 上下左右 1.5px）实现。
    text_color = rl.BLACK
    font_size = int(radius * (0.80 if len(val) >= 3 else 1.00))
    for dx, dy in ((0, 0), (-1.5, 0), (1.5, 0), (0, -1.5), (0, 1.5)):
      self._draw_text_centered(self._font_bold, val, font_size,
                               rl.Vector2(center.x + dx, center.y + dy), text_color)

  def _render_mutcd(self, rect: rl.Rectangle, val: str, is_overspeed: bool) -> None:
    rl.draw_rectangle_rounded(rect, 0.18, 10, rl.WHITE)
    inner = rl.Rectangle(rect.x + 8, rect.y + 8, rect.width - 16, rect.height - 16)
    rl.draw_rectangle_rounded_lines_ex(inner, 0.18, 10, 4, rl.BLACK)

    mid_x = rect.x + rect.width / 2
    self._draw_text_centered(self._font_demi, "SPEED", int(rect.height * 0.14),
                             rl.Vector2(mid_x, rect.y + rect.height * 0.22), rl.BLACK)
    self._draw_text_centered(self._font_demi, "LIMIT", int(rect.height * 0.14),
                             rl.Vector2(mid_x, rect.y + rect.height * 0.38), rl.BLACK)

    text_color = rl.Color(235, 32, 32, 255) if is_overspeed else rl.BLACK
    self._draw_text_centered(self._font_bold, val, int(rect.height * 0.34),
                             rl.Vector2(mid_x, rect.y + rect.height * 0.70), text_color)

  def _draw_ahead_info(self, sign_rect: rl.Rectangle) -> None:
    w, h = 170, 160
    rect = rl.Rectangle(sign_rect.x + (sign_rect.width - w) / 2, sign_rect.y + sign_rect.height + 10, w, h)
    rl.draw_rectangle_rounded(rect, 0.35, 10, rl.Color(0, 0, 0, 180))

    conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    mid_x = rect.x + rect.width / 2
    self._draw_text_centered(self._font_demi, "AHEAD", 40, rl.Vector2(mid_x, rect.y + 28),
                             rl.Color(145, 155, 149, 255))
    self._draw_text_centered(self._font_bold, str(round(self.next_speed_limit * conv)), 70,
                             rl.Vector2(mid_x, rect.y + 82), rl.WHITE)
    self._draw_text_centered(self._font_norm, self._format_dist(self.next_speed_limit_dist), 36,
                             rl.Vector2(mid_x, rect.y + 134), rl.Color(145, 155, 149, 255))

  @staticmethod
  def _draw_text_centered(font, text: str, size: int, pos_center: rl.Vector2, color: rl.Color) -> None:
    sz = measure_text_cached(font, text, size)
    rl.draw_text_ex(font, text, rl.Vector2(pos_center.x - sz.x / 2, pos_center.y - sz.y / 2), size, 0, color)

  @staticmethod
  def _format_dist(d: float) -> str:
    if ui_state.is_metric:
      if d < 50:
        return tr("Near")
      if d >= 1000:
        return f"{d / 1000:.1f} km"
      return f"{int(round(d, -1) if d < 200 else round(d, -2))} m"

    d_ft = d * METER_TO_FOOT
    if d_ft < 100:
      return tr("Near")
    if d_ft >= 900:
      return f"{d * METER_TO_MILE:.1f} mi"
    if d_ft < 500:
      return f"{int(round(d_ft / 50) * 50)} ft"
    return f"{int(round(d_ft / 100) * 100)} ft"

