"""Map panel v5 — MapLibre GL Native rendered base map (maprenderd process).

Base map frames come from the standalone maprenderd process (its own EGL/GLES
context, zero conflict with raylib) over cereal "mapRenderFrame" (QOI RGBA).
Camera (center/zoom/bearing) is sent back on "mapRenderCam" at 10Hz. Overlay
(matched road, breadcrumb, arrow, cards) uses live mapd data projected with the
SAME camera math so vector graphics align with the rendered base map.

Fallback: without maprenderd the panel shows canvas + vector route only.
Orientation: MapOrientationMode param (0 heading-up / 1 north-up) — bearing is
applied server-side; the frame is drawn unrotated.
"""
import math
import os
import time
from collections import deque

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight, font_fallback
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.sp_map_tiles import EARTH_R
from openpilot.selfdrive.ui.onroad.sp_maprender_client import MapRenderClient

BG        = rl.Color(16, 20, 28, 255)
DIVIDER   = rl.Color(38, 44, 54, 255)
ROUTE_CAS = rl.Color(10, 13, 18, 255)
ROUTE_GLO = rl.Color(46, 140, 255, 60)
ROUTE     = rl.Color(46, 140, 255, 255)
TRAIL     = rl.Color(46, 140, 255, 50)
ARROW     = rl.WHITE
ARROW_RING= rl.Color(15, 19, 25, 210)
CARD      = rl.Color(24, 29, 37, 235)
TEXT      = rl.Color(228, 233, 240, 255)
TEXT_DIM  = rl.Color(130, 138, 150, 255)
TEXT_NONE = rl.Color(100, 106, 116, 255)
ACCENT_OK = rl.Color(76, 175, 80, 255)

PANEL_W_FRAC   = 0.45
VIEW_MIN_M     = 200.
VIEW_MAX_M     = 1200.
VIEW_SECS      = 8.
TRAIL_MAX      = 600
TRAIL_MIN_DIST = 5.0
TRAIL_TIMEOUT  = 60.0
SCALE_BAR_M    = 100.0
CENTER_FRAC    = 0.5   # 与 maprenderd 视口中心一致（对齐关键点）
ICON_SIZE      = 120
ICON_PAD       = 30
_NAV_ICON_CANDIDATES = [
  os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets", "icons", "navigation", "launcher_route_light.png"),
  "/home/zheng/openpilot/openpilot/selfdrive/assets/icons/navigation/launcher_route_light.png",
]
NAV_ICON_PATH  = next((p for p in _NAV_ICON_CANDIDATES if os.path.isfile(p)), _NAV_ICON_CANDIDATES[0])
DBLCLICK_DT    = 0.35   # 双击间隔上限（秒）


def _haversine_m(lat1, lon1, lat2, lon2):
  R = 6371000.0
  dlat, dlon = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
  a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
  return 2 * R * math.asin(math.sqrt(a))


def _mercator_xy(lat, lon):
  x = math.radians(lon)
  lat = max(min(lat, 85.05), -85.05)
  y = math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
  return x, y


class MapPanel:
  def __init__(self):
    self._trail = deque(maxlen=TRAIL_MAX)
    self._last_pos = None
    self._last_ts = 0.0
    self._font = gui_app.font(FontWeight.NORMAL)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._mr = MapRenderClient()
    # nav icon: 右下角 launcher（mode=0 单图、mode=1/2 始终在右下角）
    # nav icon: 右下角 launcher (openpilot 官方模式: gui_app.texture 自动加载+缩放+缓存)
    self._nav_tex = None
    try:
      rel = os.path.relpath(NAV_ICON_PATH, "/home/zheng/openpilot/openpilot/selfdrive/assets")
      self._nav_tex = gui_app.texture(rel, ICON_SIZE, ICON_SIZE)
    except Exception as _e:
      print(f"[sp_map_panel] nav icon load fail: {_e}", flush=True)
    # 双击检测状态
    self._last_click_t = 0.0
    self._last_click_x = 0
    self._last_click_y = 0

  def update(self, sm) -> None:
    if sm.updated['mapdExtendedOut']:
      pos = sm['mapdExtendedOut'].position
      cur = (pos.latitude, pos.longitude)
      now = time.monotonic()
      if self._last_pos is None or _haversine_m(*self._last_pos, *cur) > TRAIL_MIN_DIST:
        self._trail.append(cur)
        self._last_pos, self._last_ts = cur, now
      elif now - self._last_ts > TRAIL_TIMEOUT:
        self._trail.clear()
        self._last_pos = None

  @staticmethod
  def _stroke(pts, casing_color, casing_w, core_color, core_w):
    for color, w in ((casing_color, casing_w), (core_color, core_w)):
      for i in range(1, len(pts)):
        rl.draw_line_ex(rl.Vector2(pts[i - 1][0], pts[i - 1][1]), rl.Vector2(pts[i][0], pts[i][1]), w, color)
      r = w / 2
      for x, y in pts:
        rl.draw_circle(int(x), int(y), r, color)

  @staticmethod
  def _card(x, y, w, h):
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.28, 12, CARD)

  @staticmethod
  def _rotate(px, py, cx, cy, deg):
    r = math.radians(deg)
    dx, dy = px - cx, py - cy
    return cx + dx * math.cos(r) - dy * math.sin(r), cy + dx * math.sin(r) + dy * math.cos(r)

  def render(self, rect: rl.Rectangle) -> None:
    if not ui_state.started or not ui_state.off_line_map_panel:
      return

    sm = ui_state.sm
    mode = ui_state.map_panel_mode  # 0=hidden, 1=right half, 2=fullscreen
    # 缓存 content_rect，给 handle_tap 用
    self._last_content_rect = (rect.x, rect.y, rect.width, rect.height)
    if mode == 0:
      # 仅显示右下角 nav launcher（背景透明）
      self._draw_nav_icon(rect)
      return
    if mode == 2:
      px, py, pw, ph = rect.x, rect.y, rect.width, rect.height
    else:
      pw = rect.width * PANEL_W_FRAC
      px, py, ph = rect.x + rect.width - pw, rect.y, rect.height
    panel = rl.Rectangle(px, py, pw, ph)
    rl.draw_rectangle_rec(panel, BG)
    if mode != 2:
      rl.draw_line_ex(rl.Vector2(px, py), rl.Vector2(px, py + rect.height), 2, DIVIDER)

    ext, out, gps = sm['mapdExtendedOut'], sm['mapdOut'], sm['gpsLocationExternal']
    tile_ok = bool(out.tileLoaded) if sm.valid['mapdOut'] else False
    ws = str(out.waySelectionType)
    match_ok = ws in ('current', 'predicted', 'possible')
    match_dim = ws == 'possible'
    data_ok = sm.valid['mapdExtendedOut'] and len(ext.path) >= 2 and match_ok and tile_ok

    lat0, lon0 = ext.position.latitude, ext.position.longitude
    v_ego = sm['carState'].vEgo
    view_m = min(max(v_ego * VIEW_SECS, VIEW_MIN_M), VIEW_MAX_M)
    mpp_screen = view_m / panel.height
    zoom = math.log2(math.cos(math.radians(lat0)) * 2 * math.pi * EARTH_R / (256 * mpp_screen))
    zoom = min(zoom, 16.0)   # 性能封顶：z14 数据 + llvmpipe，z19 静止视图无意义且过重
    bearing = gps.bearingDeg if sm.valid['gpsLocationExternal'] and gps.horizontalAccuracy < 15.0 else 0.0
    if ui_state.map_orientation == 1:
      bearing = 0.0

    # ---- 底图：向 maprenderd 发相机，取回帧 ----
    self._mr.send_cam(lat0, lon0, zoom, bearing, int(panel.width), int(panel.height))
    tex = self._mr.frame_texture(sm)
    if tex is not None and rl.is_texture_valid(tex):
      w, h = self._mr._tex_size
      # 按 tex 原比例裁剪 source rect，避免拉伸变形
      # 目标: 完整显示 tex 不变形，居中裁剪 panel 区域
      scale = max(panel.width / w, panel.height / h)
      src_w = panel.width / scale
      src_h = panel.height / scale
      src_x = (w - src_w) / 2
      src_y = (h - src_h) / 2
      rl.draw_texture_pro(tex, rl.Rectangle(src_x, src_y, src_w, src_h), panel, rl.Vector2(0, 0), 0.0, rl.WHITE)

    # ---- 叠加层投影（与底图同一相机：中心/zoom/bearing 完全一致） ----
    cx_w, cy_w = _mercator_xy(lat0, lon0)
    b = math.radians(bearing)
    cx, cy = px + panel.width / 2, py + panel.height * CENTER_FRAC
    meters_per_rad_x = EARTH_R * math.cos(math.radians(lat0))

    def to_screen(lat, lon):
      mx, my = _mercator_xy(lat, lon)
      dx = (mx - cx_w) * meters_per_rad_x
      dy = (my - cy_w) * EARTH_R
      sx = dx * math.cos(b) - dy * math.sin(b)
      sy = dx * math.sin(b) + dy * math.cos(b)
      return cx + sx / mpp_screen, cy - sy / mpp_screen

    if len(self._trail) >= 2:
      pts = [to_screen(*p) for p in self._trail]
      for i in range(1, len(pts)):
        rl.draw_line_ex(rl.Vector2(pts[i - 1][0], pts[i - 1][1]), rl.Vector2(pts[i][0], pts[i][1]), 2.0, TRAIL)

    if data_ok:
      pts = [to_screen(p.latitude, p.longitude) for p in ext.path]
      alpha = 150 if match_dim else 255
      self._stroke(pts, ROUTE_CAS, 12.0, ROUTE_CAS, 9.0)
      self._stroke(pts, ROUTE_CAS, 8.0, rl.Color(ROUTE_GLO.r, ROUTE_GLO.g, ROUTE_GLO.b, 110 if not match_dim else 60), 7.0)
      self._stroke(pts, rl.Color(ROUTE.r, ROUTE.g, ROUTE.b, alpha), 4.5,
                   rl.Color(ROUTE.r, ROUTE.g, ROUTE.b, alpha), 4.5)

    # ---- 箭头与卡片（同 v4） ----
    gps_ok = sm.valid['gpsLocationExternal'] and gps.horizontalAccuracy < 10.0
    col = ARROW if gps_ok else rl.Color(150, 155, 162, 255)
    rot_deg = gps.bearingDeg if ui_state.map_orientation == 1 and sm.valid['gpsLocationExternal'] else 0.0
    rl.draw_circle(int(cx), int(cy) + 2, 13, ARROW_RING)
    head = [self._rotate(cx, cy - 18, cx, cy, rot_deg), self._rotate(cx - 10, cy + 4, cx, cy, rot_deg),
            self._rotate(cx + 10, cy + 4, cx, cy, rot_deg)]
    rl.draw_triangle(rl.Vector2(*head[0]), rl.Vector2(*head[1]), rl.Vector2(*head[2]), col)
    s0 = self._rotate(cx - 4.5, cy + 2, cx, cy, rot_deg)
    rl.draw_rectangle_rounded(rl.Rectangle(s0[0], s0[1], 9, 13), 0.4, 6, col)

    name = out.roadName or out.wayRef
    self._card(px + 16, py + 14, panel.width - 32, 64)
    if data_ok and name:
      # Use the system fallback font (NotoSansCJKsc with full CJK coverage
      # as configured by the user in application.py / po file). Don't load a
      # second atlas — that wastes raylib memory.
      font = font_fallback(self._font_bold if not match_dim else self._font)
      color = TEXT if not match_dim else TEXT_DIM
      rl.draw_text_ex(font, name, rl.Vector2(px + 34, py + 30), 34, 0, color)
    else:
      rl.draw_text_ex(self._font, "offline map: no data", rl.Vector2(px + 34, py + 32), 28, 0, TEXT_NONE)

    self._card(px + 16, py + panel.height - 78, panel.width - 32, 62)
    if sm.valid['mapdOut']:
      dot = ACCENT_OK if (match_ok and tile_ok) else TEXT_DIM
      rl.draw_circle(int(px + 40), int(py + panel.height - 47), 5, dot)
      status = ws if tile_ok else f"{ws} · no tiles"
    else:
      status = "mapd offline"
    rl.draw_text_ex(self._font, f"match: {status}", rl.Vector2(px + 54, py + panel.height - 58), 26, 0, TEXT_DIM)
    bar_px = SCALE_BAR_M / mpp_screen
    bx, by = px + panel.width - 36 - bar_px, py + panel.height - 44
    rl.draw_line_ex(rl.Vector2(bx, by), rl.Vector2(bx + bar_px, by), 3, TEXT)
    rl.draw_line_ex(rl.Vector2(bx, by - 6), rl.Vector2(bx, by + 6), 2, TEXT)
    rl.draw_line_ex(rl.Vector2(bx + bar_px, by - 6), rl.Vector2(bx + bar_px, by + 6), 2, TEXT)
    rl.draw_text_ex(self._font, f"{int(SCALE_BAR_M)} m", rl.Vector2(bx - 6, by - 36), 24, 0, TEXT_DIM)

    if ui_state.map_orientation == 1:
      rl.draw_circle(int(px + panel.width - 44), int(py + 104), 16, CARD)
      rl.draw_text_ex(self._font_bold, "N", rl.Vector2(px + panel.width - 50, py + 94), 24, 0, TEXT)

    # nav icon launcher (右下角，与 mode=0 同一位置；mode=2 全屏下也保留以便单击关闭)
    self._draw_nav_icon(rect)

  def _draw_nav_icon(self, rect: rl.Rectangle) -> None:
    # 模仿 exp_button.py: 圆形黑色背景 + 白色图标
    if self._nav_tex is None:
      return
    # 圆心 = 让圆右边距屏幕右边 = ICON_PAD (跟 openpilot exp button border_size 一致)
    r = ICON_SIZE // 2 + 10
    cx = int(rect.x + rect.width - ICON_PAD - r)
    cy = int(rect.y + rect.height - ICON_PAD - r)
    rl.draw_circle(cx, cy, r, rl.Color(0, 0, 0, 166))
    rl.draw_texture_ex(self._nav_tex, rl.Vector2(cx - self._nav_tex.width / 2, cy - self._nav_tex.height / 2),
                       0.0, 1.0, rl.Color(255, 255, 255, 255))

  def handle_tap(self, x: float, y: float) -> None:
    """触屏点击路由：命中 nav icon -> toggle on/off；命中地图 panel + 双击 -> toggle 1<->2."""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return
    rect_x, rect_y, rect_w, rect_h = self._last_content_rect
    # 圆心 = 让圆右边距屏幕右边 = ICON_PAD (跟 _draw_nav_icon 一致)
    r = ICON_SIZE // 2 + 10
    cx = rect_x + rect_w - ICON_PAD - r
    cy = rect_y + rect_h - ICON_PAD - r
    dx = x - cx
    dy = y - cy
    in_icon = (dx*dx + dy*dy) <= r*r

    now = time.monotonic()
    is_dblclick = (now - self._last_click_t <= DBLCLICK_DT
                   and abs(x - self._last_click_x) <= 40
                   and abs(y - self._last_click_y) <= 40)

    if in_icon:
      # 单击 nav icon: mode 0 -> 1；mode 1/2 -> 0 (关闭)
      mode = ui_state.map_panel_mode
      if mode == 0:
        ui_state.map_panel_mode = 1
      else:
        ui_state.map_panel_mode = 0
      return

    # 命中地图 panel 区域：单击切 mode 1↔2
    mode = ui_state.map_panel_mode
    if mode == 1:
      ui_state.map_panel_mode = 2
    elif mode == 2:
      ui_state.map_panel_mode = 1
