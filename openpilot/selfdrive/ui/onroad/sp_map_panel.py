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
SCALE_BAR_TARGET_PX = 120.0  # 比例尺目标像素长度：1-2-5 序列取不超过此值的最大档
_SCALE_STEPS = (20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000)
CENTER_FRAC    = 0.5   # 与 maprenderd 视口中心一致（对齐关键点）
ICON_SIZE      = 120
ICON_PAD       = 30
ICON_PAD_BOTTOM = 90   # 底缘让开底部状态卡/比例尺行（卡高 78px + 12px 间隙）
_NAV_ICON_CANDIDATES = [
  os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets", "icons", "navigation", "launcher_route_light.png"),
  "/home/zheng/openpilot/openpilot/selfdrive/assets/icons/navigation/launcher_route_light.png",
]
NAV_ICON_PATH  = next((p for p in _NAV_ICON_CANDIDATES if os.path.isfile(p)), _NAV_ICON_CANDIDATES[0])
DBLCLICK_DT    = 0.35   # 双击间隔上限（秒）
ZOOM_MIN       = 10.0   # mbtiles 数据下限，z10 以下瓦片过稀
ZOOM_MAX       = 16.0   # 与自动 zoom 封顶一致
ZOOM_MANUAL_MAX = 18.0  # 手动 +/- 允许超过自动封顶（z14 数据 overzoom，略糊但可用）
ZOOM_HIT_PAD   = 14     # 按钮热区四周外扩（触屏/坐标偏移容错）
ZOOM_STEP      = 1.0    # 每按一次 +/- 调一档
ZOOM_BTN_R     = 20     # 缩放按钮半径（px）
ZOOM_BIAS_LIM  = 8.0    # 手动 bias 限幅（最终 zoom 仍由 ZOOM_MIN/MAX clamp）


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
    # 手动缩放状态：bias 叠加在自动 zoom 上（in-memory，与 map_panel_mode 同生命周期）
    self._zoom_bias = 0.0
    self._zoom_plus_rect = None
    self._zoom_minus_rect = None
    self._last_zoom_btn_t = 0.0   # 防抖：WSLg/XTEST 可能产生幻影点击，限制缩放按钮触发频率
    # 中心点：GPS 航位推算 + 1Hz 匹配位置修正（消快慢性交替）
    # mapdExtendedOut.position 仅 1Hz 刷新，直接跟随会每秒跳变一次；
    # 纯指数平滑会 1Hz 锯齿（先快追再慢收尾）。改用 10Hz GPS 连续推进：
    # 每次匹配位置刷新时记录 gps→matched 的偏移锚点，平时中心=GPS+锚点。
    # GPS 不可用时退化为指数平滑（tau ≈ 0.35s）
    self._ctr = None       # [lat, lon] 平滑后的中心（降级路径用）
    self._ctr_t = None     # 上次平滑时间戳
    self._gps_anchor = None  # (dlat, dlon) 匹配位置相对 GPS 的偏移

  def _poll_taps(self) -> None:
    """触摸/鼠标自轮询 (替代 Widget 事件穿透——panel 非 Widget，事件到不了 handle_tap)。

    raylib 输入在 UI 进程全局有效，C3X 触摸即鼠标事件；is_mouse_button_pressed
    为帧级沿触发，每帧最多一次。命中与否由 handle_tap 内 hit_test 决定。
    rl.get_mouse_position() 返回 raylib logical 坐标，与绘制坐标系一致。
    注意: nav icon 点击需要在 mode=0 时也能命中 (toggle 0->1)，所以只在 panel 隐藏时跳过。"""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return
    if rl.is_mouse_button_pressed(rl.MouseButton.MOUSE_BUTTON_LEFT):
      m = rl.get_mouse_position()
      self.handle_tap(m.x, m.y)

  def update(self, sm) -> None:
    self._poll_taps()
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

    tlat, tlon = ext.position.latitude, ext.position.longitude
    gps_ok = sm.valid['gpsLocationExternal'] and gps.horizontalAccuracy < 15.0
    if not (sm.valid['mapdExtendedOut'] and data_ok):
      self._ctr = None
      self._ctr_t = None
      self._gps_anchor = None
    elif sm.updated['mapdExtendedOut'] and gps_ok:
      # 1Hz 锚点刷新：记录当前 GPS 与匹配位置的偏移（含地图匹配修正量）
      self._gps_anchor = (tlat - gps.latitude, tlon - gps.longitude)
    if self._gps_anchor is not None and gps_ok:
      # 10Hz GPS 连续推进 + 偏移修正 → 无 1Hz 跳变、无快慢性交替
      lat0 = gps.latitude + self._gps_anchor[0]
      lon0 = gps.longitude + self._gps_anchor[1]
    elif self._ctr is None or self._ctr_t is None:
      self._ctr = [tlat, tlon]
      self._ctr_t = time.monotonic()
      lat0, lon0 = self._ctr
    else:
      # GPS 不可用：退化为指数平滑
      now = time.monotonic()
      dt = max(now - self._ctr_t, 1e-3)
      self._ctr_t = now
      a = 1.0 - math.exp(-dt / 0.35)
      self._ctr[0] += (tlat - self._ctr[0]) * a
      self._ctr[1] += (tlon - self._ctr[1]) * a
      lat0, lon0 = self._ctr
    v_ego = sm['carState'].vEgo
    view_m = min(max(v_ego * VIEW_SECS, VIEW_MIN_M), VIEW_MAX_M)
    mpp_screen = view_m / panel.height
    zoom = math.log2(math.cos(math.radians(lat0)) * 2 * math.pi * EARTH_R / (256 * mpp_screen))
    zoom = min(zoom, 16.0)   # 性能封顶：z14 数据 + llvmpipe，z19 静止视图无意义且过重
    # 手动缩放（+/- 按钮）：bias 叠加后 clamp；overlay 投影与比例尺必须用最终 zoom
    # 反推 mpp_screen，否则底图（按 zoom 渲染）与矢量叠加（按 mpp 投影）错位
    zoom = min(max(zoom + self._zoom_bias, ZOOM_MIN), ZOOM_MANUAL_MAX)
    mpp_screen = math.cos(math.radians(lat0)) * 2 * math.pi * EARTH_R / (256 * (2 ** zoom))
    bearing = gps.bearingDeg if sm.valid['gpsLocationExternal'] and gps.horizontalAccuracy < 15.0 else 0.0
    if ui_state.map_orientation == 1:
      bearing = 0.0

    # 3D pitch: 60° when panel's 2D/3D toggle is active + zoom deep enough.
    # The toggle lives on the map panel itself; default = 2D (top-down).
    pitch = 60.0 if (ui_state.map_panel_3d_active and zoom >= 14.0) else 0.0

    # ---- 底图：向 maprenderd 发相机，取回帧 ----
    self._mr.send_cam(lat0, lon0, zoom, bearing, int(panel.width), int(panel.height), pitch)
    mpp_view = mpp_screen  # 无底图时 overlay/比例尺退化为相机 mpp
    tex = self._mr.frame_texture(sm)
    if tex is not None and rl.is_texture_valid(tex):
      w, h = self._mr._tex_size
      # 按 tex 原比例裁剪 source rect，避免拉伸变形
      # 目标: 完整显示 tex 不变形，居中裁剪 panel 区域
      scale = max(panel.width / w, panel.height / h)
      mpp_view = mpp_screen / scale if scale > 0 else mpp_screen  # 实际显示比例（含纹理放大）
      src_w = panel.width / scale
      src_h = panel.height / scale
      src_x = (w - src_w) / 2
      src_y = (h - src_h) / 2
      rl.draw_texture_pro(tex, rl.Rectangle(src_x, src_y, src_w, src_h), panel, rl.Vector2(0, 0), 0.0, rl.WHITE)

    # ---- 右上角 2D/3D 切换按钮 (绘制和 hit_test 用同一 rect) ----
    self._3d_pill_rect = self._3d_toggle_hit_rect(px, py, pw)
    self._draw_3d_toggle_button(self._3d_pill_rect)

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
      return cx + sx / mpp_view, cy - sy / mpp_view

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

    # ---- 自车图标：蓝色圆形带白色向上箭头 (ego_circle_A.png) ----
    if not hasattr(self, '_ego_loaded'):
      self._ego_loaded = True
      try:
        self._tex_ego = gui_app.texture("icons/ego_circle_A.png", 64, 64)
      except Exception as _e:
        print(f"[sp_map_panel] ego icon load fail: {_e}", flush=True)
        self._tex_ego = None
    rot_deg = gps.bearingDeg if ui_state.map_orientation == 1 and sm.valid['gpsLocationExternal'] else 0.0
    if getattr(self, '_tex_ego', None) is not None:
      half = 32
      rl.draw_texture_ex(self._tex_ego, rl.Vector2(cx - half, cy - half), float(rot_deg), 1.0, rl.WHITE)

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
    # 比例尺：1-2-5 序列选不超过目标像素的最大档，单位随量级切 m/km
    bar_m = _SCALE_STEPS[0]
    for _s in _SCALE_STEPS:
      if _s / mpp_view <= SCALE_BAR_TARGET_PX:
        bar_m = _s
      else:
        break
    bar_px = bar_m / mpp_view
    bar_label = f"{bar_m} m" if bar_m < 1000 else f"{bar_m / 1000:g} km"
    by = py + panel.height - 44
    # 缩放 +/- 按钮：+ 贴面板右缘，比例尺在其左，- 再在比例尺左侧
    zr, gap = ZOOM_BTN_R, 14
    plus_cx = px + panel.width - 16 - zr
    bx_right = plus_cx - zr - gap
    bx = bx_right - bar_px
    minus_cx = bx - gap - zr
    btn_cy = by - 4
    rl.draw_line_ex(rl.Vector2(bx, by), rl.Vector2(bx_right, by), 3, TEXT)
    rl.draw_line_ex(rl.Vector2(bx, by - 6), rl.Vector2(bx, by + 6), 2, TEXT)
    rl.draw_line_ex(rl.Vector2(bx_right, by - 6), rl.Vector2(bx_right, by + 6), 2, TEXT)
    _tw = rl.measure_text_ex(self._font, bar_label, 24, 0).x
    rl.draw_text_ex(self._font, bar_label, rl.Vector2(bx + (bar_px - _tw) / 2, by - 34), 24, 0, TEXT_DIM)
    # 缓存按钮 rect 供 handle_tap 命中（与绘制同一圆心/半径）
    self._zoom_plus_rect = rl.Rectangle(plus_cx - zr, btn_cy - zr, zr * 2, zr * 2)
    self._zoom_minus_rect = rl.Rectangle(minus_cx - zr, btn_cy - zr, zr * 2, zr * 2)
    self._draw_zoom_button(plus_cx, btn_cy, "+")
    self._draw_zoom_button(minus_cx, btn_cy, "-")

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
    cy = int(rect.y + rect.height - ICON_PAD_BOTTOM - r)
    rl.draw_circle(cx, cy, r, rl.Color(0, 0, 0, 166))
    rl.draw_texture_ex(self._nav_tex, rl.Vector2(cx - self._nav_tex.width / 2, cy - self._nav_tex.height / 2),
                       0.0, 1.0, rl.Color(255, 255, 255, 255))

  def _draw_zoom_button(self, cx: float, cy: float, label: str) -> None:
    """圆形缩放按钮。圆心/半径与 handle_tap 的 _zoom_plus/minus_rect 一致。"""
    r = ZOOM_BTN_R
    rl.draw_circle(int(cx), int(cy), r, CARD)
    rl.draw_circle_lines(int(cx), int(cy), r, DIVIDER)
    # 固定偏移近似居中（+ 字形比 - 宽）
    ox, oy = (10, 20) if label == "+" else (7, 22)
    rl.draw_text_ex(self._font_bold, label, rl.Vector2(cx - ox, cy - oy), 36, 0, TEXT)

  def _3d_toggle_hit_rect(self, px: float, py: float, pw: float) -> rl.Rectangle:
    """右上角 2D/3D 切换按钮的矩形。绘制和 hit_test 共用此函数返回的同一 rect。
    按钮大小跟当前显示的图标一致: 2D=110x110, 3D=90x90。"""
    is_3d = ui_state.map_panel_3d_active
    size = 90 if is_3d else 110
    pad_x = 24
    pad_y = 100  # 避开 road name card (高 64 + pad 18)
    return rl.Rectangle(px + pw - size - pad_x, py + pad_y, size, size)

  def _load_3d_icons(self):
    """懒加载 2D/3D 切换图标 texture (绝对路径引用桌面 icon)。"""
    if not hasattr(self, '_3d_icons_loaded'):
      self._3d_icons_loaded = True
      icon_size = int(self._3d_toggle_hit_rect(None, 0, 0).width) if False else 110
      try:
        self._tex_2d = gui_app.texture("icons/map_2d.png", 110, 110)
        self._tex_3d = gui_app.texture("icons/map_3d.png", 90, 90)
      except Exception as e:
        print(f"[mapd] icon load failed: {e}", flush=True)
        self._tex_2d = self._tex_3d = None

  def _draw_3d_toggle_button(self, rect: rl.Rectangle) -> None:
    """绘制 2D/3D 切换图标。rect 必须与 handle_tap 用的 _3d_toggle_hit_rect 完全相同。
    状态反着显示: 当前 2D 显示 3D 图标 (点切到 3D)，当前 3D 显示 2D 图标 (点切回 2D)。
    图标按 rect 尺寸绘制 (rect 由 is_3d 状态决定 size=90 或 110)。"""
    self._load_3d_icons()
    is_3d = ui_state.map_panel_3d_active
    tex = self._tex_3d if not is_3d else self._tex_2d
    if tex is not None:
      rl.draw_texture_ex(tex, rl.Vector2(rect.x, rect.y), 0.0, 1.0, rl.WHITE)

  def handle_tap(self, x: float, y: float) -> None:
    """触屏点击路由：命中 nav icon -> toggle on/off；命中地图 panel + 双击 -> toggle 1<->2."""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return
    rect_x, rect_y, rect_w, rect_h = self._last_content_rect
    # 圆心 = 让圆右边距屏幕右边 = ICON_PAD (跟 _draw_nav_icon 一致)
    r = ICON_SIZE // 2 + 10
    cx = rect_x + rect_w - ICON_PAD - r
    cy = rect_y + rect_h - ICON_PAD_BOTTOM - r
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

    # 缩放 +/- 按钮：命中即返回，不触发 panel 模式切换
    if ui_state.map_panel_mode != 0:
      pt = rl.Vector2(x, y)
      pr, mr = getattr(self, '_zoom_plus_rect', None), getattr(self, '_zoom_minus_rect', None)
      now = time.monotonic()
      zoom_btn_ok = (now - self._last_zoom_btn_t) > 0.5
      if pr is not None and (pr.x - ZOOM_HIT_PAD <= x <= pr.x + pr.width + ZOOM_HIT_PAD
                             and pr.y - ZOOM_HIT_PAD <= y <= pr.y + pr.height + ZOOM_HIT_PAD):
        if zoom_btn_ok:
          self._zoom_bias = min(self._zoom_bias + ZOOM_STEP, ZOOM_BIAS_LIM)
          self._last_zoom_btn_t = now
        return
      if mr is not None and (mr.x - ZOOM_HIT_PAD <= x <= mr.x + mr.width + ZOOM_HIT_PAD
                             and mr.y - ZOOM_HIT_PAD <= y <= mr.y + mr.height + ZOOM_HIT_PAD):
        if zoom_btn_ok:
          self._zoom_bias = max(self._zoom_bias - ZOOM_STEP, -ZOOM_BIAS_LIM)
          self._last_zoom_btn_t = now
        return

    # 在 panel 外点 nav icon 已被上面 return；这里只处理 panel 内的点击
    mode = ui_state.map_panel_mode

    # 命中 2D/3D 切换图标: 用绘制时缓存的 self._3d_pill_rect (与 _draw 用同一实例)
    if mode != 0 and hasattr(self, '_3d_pill_rect'):
      r = self._3d_pill_rect
      if rl.check_collision_point_rec(rl.Vector2(x, y), r):
        ui_state.map_panel_3d_active = not ui_state.map_panel_3d_active
        return

    # 命中地图 panel 区域：单击切 mode 1↔2
    mode = ui_state.map_panel_mode
    if mode == 1:
      ui_state.map_panel_mode = 2
    elif mode == 2:
      ui_state.map_panel_mode = 1
