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

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight, font_fallback
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.sp_map_tiles import EARTH_R
from openpilot.selfdrive.ui.onroad.sp_maprender_client import MapRenderClient

BG        = rl.Color(16, 20, 28, 255)
DIVIDER   = rl.Color(38, 44, 54, 255)
ARROW     = rl.WHITE
ARROW_RING= rl.Color(15, 19, 25, 210)
CARD      = rl.Color(24, 29, 37, 235)
TEXT      = rl.Color(228, 233, 240, 255)
TEXT_DIM  = rl.Color(130, 138, 150, 255)
TEXT_NONE = rl.Color(100, 106, 116, 255)
ACCENT_OK = rl.Color(76, 175, 80, 255)

PANEL_W_FRAC   = 0.5
VIEW_MIN_M     = 200.
VIEW_MAX_M     = 1200.
VIEW_SECS      = 8.
SCALE_BAR_TARGET_PX = 120.0  # 比例尺目标像素长度：1-2-5 序列取不超过此值的最大档
_SCALE_STEPS = (20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000)
CENTER_FRAC    = 0.5   # 与 maprenderd 视口中心一致（对齐关键点）
ICON_SIZE      = 160
# 圆心偏移：与左下角司机监控图标（driver_state.BTN_SIZE=192）严格对称
NAV_CENTER_OFF = UI_BORDER_SIZE + 192 // 2
_NAV_ICON_CANDIDATES = [
  os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "assets", "icons", "map_nav.png"),
  "/home/zheng/openpilot/openpilot/selfdrive/assets/icons/map_nav.png",
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


def _mercator_xy(lat, lon):
  x = math.radians(lon)
  lat = max(min(lat, 85.05), -85.05)
  y = math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
  return x, y


def _dest_point(lat, lon, bearing_deg, d_m):
  """从 (lat, lon) 沿方位角走 d_m 米的终点坐标。"""
  if d_m <= 0.0:
    return lat, lon
  R = 6371000.0
  d = d_m / R
  b = math.radians(bearing_deg)
  la1 = math.radians(lat)
  lo1 = math.radians(lon)
  la2 = math.asin(math.sin(la1) * math.cos(d) + math.cos(la1) * math.sin(d) * math.cos(b))
  lo2 = lo1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(la1),
                         math.cos(d) - math.sin(la1) * math.sin(la2))
  return math.degrees(la2), math.degrees(lo2)


def _cjk_road_name(name: str) -> str:
  """标题栏只保留中文路名：去掉 ASCII 字母（英文部分/拼音），收尾清理。"""
  name = "".join(ch for ch in name if not ("a" <= ch <= "z" or "A" <= ch <= "Z"))
  return name.strip(" \t/·•-—_")


BEAR_TURN_RATE = 110.0  # 视图方位角最大转速 °/s：转向时底图渐进旋转而非瞬切
NAV_AHEAD_M    = 120.0  # 3D 导航视角：相机中心沿航向提前量，车辆落在屏幕 ~62% 高度


def _project3d(lat, lon, cam_lat, cam_lon, zoom, bearing_rad, pitch_rad, W, H, alt_m=0.0):
  """maplibre 相机一致的 3D 透视投影：经纬度 -> 渲染帧像素坐标。

  fov = 2*atan(1/3)（maplibre 默认，焦距 f=3），相机距地面中心 1.5*H 像素。
  标定方法：与 mbgl-render 同参数输出逐点比对（bearing 0/90 均吻合）。
  alt_m: 海拔（米），沿世界竖直方向抬升（相机系分量 y2+=h·sin p, z+=h·cos p）。
  相机后方（z>=0）返回 None。
  """
  world = 512.0 * (2.0 ** zoom)

  def _w(la, lo):
    x = (lo + 180.0) / 360.0 * world
    s = math.sin(math.radians(la))
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * world
    return x, y

  wx, wy = _w(lat, lon)
  cx_, cy_ = _w(cam_lat, cam_lon)
  px_ = wx - cx_
  yup = cy_ - wy  # 北为正
  x1 = px_ * math.cos(bearing_rad) - yup * math.sin(bearing_rad)
  y1 = px_ * math.sin(bearing_rad) + yup * math.cos(bearing_rad)
  y2 = y1 * math.cos(pitch_rad)
  z = -y1 * math.sin(pitch_rad) - 1.5 * H
  if alt_m > 0.0:
    # 世界竖直方向经俯仰旋转后的相机系分量；hpx = 海拔换算成投影世界像素
    # （512*2^z 坐标系，1 世界px = 2πR·cosφ/world 米）
    hpx = alt_m * world / (2.0 * math.pi * EARTH_R * math.cos(math.radians(lat)))
    y2 += hpx * math.sin(pitch_rad)
    z += hpx * math.cos(pitch_rad)
  if z >= -1e-6:
    return None
  f = 3.0
  ndc_x = (f / (W / H)) * x1 / (-z)
  ndc_y = f * y2 / (-z)
  return ((ndc_x + 1) * 0.5 * W, (1 - ndc_y) * 0.5 * H)


class MapPanel:
  def __init__(self):
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
    # 中心点：GPS 速度外推 + 1Hz 匹配位置慢修正
    # 10Hz GPS 固定解是阶梯；maprenderd 以自身节奏抽样（~5-8Hz），随机相位
    # 采样阶梯会产生快慢交替的拍频。用速度×时间把最后固定解外推成连续轨迹，
    # 1Hz 匹配位置与 GPS 的偏移只做慢低通修正（tau=1.5s，消除匹配量化抖动）。
    self._ctr = None       # [lat, lon] 平滑后的中心（GPS 全失效时降级用）
    self._ctr_t = None     # 上次渲染时间戳（兼作低通 dt）
    self._gps_anchor = None  # (dlat, dlon) 匹配位置相对外推 GPS 的偏移（低通后）
    self._gps_last = None  # (lat, lon, t_mono, speed, bearing) 最新 GPS 固定解
    self._gps_pos = None   # [lat, lon] 连续 GPS 轨迹状态（积分 + 固定解软修正）
    self._gps_pos_t = None # 上次轨迹积分时间戳
    self._gps_vel = None   # (speed, bearing) 最新固定解速度
    self._gps_nfix = 0     # 已处理的固定解计数（初始收敛阶段用大增益）
    self._view_bearing = None  # 平滑后的视图方位角（转向时渐进旋转，避免底图瞬切）
    self._vb_t = None

  def _smooth_bearing(self, target: float, now: float) -> float:
    """视图方位角限速逼近目标值（沿最短弧），消除转向时底图的突变式切换。"""
    if self._view_bearing is None:
      self._view_bearing = target
      self._vb_t = now
      return target
    dt = min(max(now - (self._vb_t or now), 1e-3), 0.5)
    self._vb_t = now
    diff = (target - self._view_bearing + 180.0) % 360.0 - 180.0
    step = BEAR_TURN_RATE * dt
    if abs(diff) <= step:
      self._view_bearing = target % 360.0
    else:
      self._view_bearing = (self._view_bearing + math.copysign(step, diff)) % 360.0
    return self._view_bearing

  def _poll_taps(self) -> None:
    """触摸点击路由 (替代 Widget 事件穿透——panel 非 Widget，事件到不了 handle_tap)。

    事件源用 gui_app.mouse_events：它由 140Hz 后台线程采样触摸状态、逐帧合并成
    事件队列（press/release 状态跳变各记一条）。C3X 触摸极快（<50ms）也能被
    140Hz 采样捕获，因此不丢点击——边栏按钮每次必响应就是走的这条链。
    旧实现用渲染帧级的 is_mouse_button_pressed / GESTURE_TAP 轮询，点击落在
    两帧之间就被抵消漏掉（用户报告的"偶尔管用"）。GESTURE_TAP 保留作兜底，
    两路都触发时按时间+距离去重，避免一次点击切两档。"""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return
    now = time.monotonic()
    tx = ty = None
    for ev in gui_app.mouse_events:
      if ev.left_pressed:
        tx, ty = ev.pos.x, ev.pos.y
        break
    if tx is None and rl.is_gesture_detected(rl.GESTURE_TAP):
      p = rl.get_touch_position(0)
      tx, ty = p.x, p.y
    if tx is None:
      return
    lt = getattr(self, '_last_tap_handle', None)
    if lt is not None and now - lt[0] < 0.3 and abs(tx - lt[1]) < 40 and abs(ty - lt[2]) < 40:
      return
    self._last_tap_handle = (now, tx, ty)
    self.handle_tap(tx, ty)

  def update(self, sm) -> None:
    self._poll_taps()

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
    now = time.monotonic()
    if not (sm.valid['mapdExtendedOut'] and data_ok):
      self._ctr = None
      self._ctr_t = None
      self._gps_anchor = None
      self._gps_last = None
      self._gps_pos = None
      self._gps_pos_t = None
      self._gps_nfix = 0
    # GPS 轨迹跟踪器：位置状态连续积分（每帧按最新速度/方位角外推 dt_frame），
    # 新固定解只做小增益修正（tau=0.8s）。固定解的发布延迟有慢漂移（实测每
    # ~1.5s 滞后约一帧），若每帧硬重置外推基线，延迟抖动会直接变成画面的
    # 快慢性交替；连续状态 + 软修正可把该抖动压低两个数量级。
    dt_frame = min(max(now - (self._gps_pos_t or now), 1e-3), 0.5)
    if self._gps_pos is not None:
      g0 = self._gps_last
      sp, br = self._gps_vel if self._gps_vel is not None else (0.0, 0.0)
      if g0 is not None and now - g0[2] > 1.0:
        sp = 0.0   # 固定解久未更新，停止外推避免漂移
      self._gps_pos[0], self._gps_pos[1] = _dest_point(self._gps_pos[0], self._gps_pos[1], br, sp * dt_frame)
    if gps_ok and sm.updated['gpsLocationExternal']:
      if self._gps_pos is None:
        self._gps_pos = [gps.latitude, gps.longitude]
      else:
        # 初始收敛阶段用大增益：跟踪器从零初始化时需吸收固定解的发布延迟
        # （约 0.3~0.5s 行程），tau=0.8s 要拖 ~8s，头 12 个解用 0.25 缩短到 ~1s
        a = 0.25 if self._gps_nfix < 12 else 1.0 - math.exp(-dt_frame / 0.8)
        self._gps_nfix += 1
        self._gps_pos[0] += (gps.latitude - self._gps_pos[0]) * a
        self._gps_pos[1] += (gps.longitude - self._gps_pos[1]) * a
      self._gps_vel = (gps.speed, gps.bearingDeg)
      self._gps_last = (gps.latitude, gps.longitude, now, gps.speed, gps.bearingDeg)
    self._gps_pos_t = now
    elat = elon = None
    if self._gps_pos is not None:
      elat, elon = self._gps_pos[0], self._gps_pos[1]
    elif gps_ok:
      elat, elon = gps.latitude, gps.longitude
    if data_ok and elat is not None:
      if sm.updated['mapdExtendedOut']:
        # 1Hz 锚点测量值，慢低通收敛（tau=1.5s），吸收匹配位置量化抖动
        meas = (tlat - elat, tlon - elon)
        if self._gps_anchor is None:
          self._gps_anchor = meas
        else:
          dt = max(now - (self._ctr_t or now), 1e-3)
          a = 1.0 - math.exp(-dt / 1.5)
          self._gps_anchor = (self._gps_anchor[0] + (meas[0] - self._gps_anchor[0]) * a,
                              self._gps_anchor[1] + (meas[1] - self._gps_anchor[1]) * a)
      self._ctr_t = now
      if self._gps_anchor is not None:
        lat0 = elat + self._gps_anchor[0]
        lon0 = elon + self._gps_anchor[1]
      else:
        lat0, lon0 = elat, elon
    elif self._ctr is None or self._ctr_t is None:
      # GPS 全不可用：退化为对 1Hz 匹配位置的指数平滑
      self._ctr = [tlat, tlon]
      self._ctr_t = now
      lat0, lon0 = self._ctr
    else:
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
    tgt_bearing = gps.bearingDeg if sm.valid['gpsLocationExternal'] and gps.horizontalAccuracy < 15.0 else 0.0
    if ui_state.map_orientation == 1:
      tgt_bearing = 0.0
    bearing = self._smooth_bearing(tgt_bearing, now)

    # 3D pitch: 60° when panel's 2D/3D toggle is active + zoom deep enough.
    # The toggle lives on the map panel itself; default = 2D (top-down).
    pitch = 45.0 if (ui_state.map_panel_3d_active and zoom >= 14.0) else 0.0

    # 导航视角（高德/苹果式）：3D heading-up 时相机中心沿航向提前一段距离，
    # 车标落在屏幕下 1/3（画面 2/3 高度处，视野看向远方而非钉死屏幕中心）。
    # 提前量必须随缩放动态换算：固定距离放大后会占更多像素，车标滑向底缘
    # （z18 时几乎出屏）。由 _project3d 反解保持 φ=2/3 的提前量：
    #   r = D/H = 1.5(2φ-1) / ((2φ-1)·sin p + f·cos p)，f=3，p=45° → r≈0.212
    cam_lat0, cam_lon0 = lat0, lon0
    if pitch > 0.5 and ui_state.map_orientation != 1:
      pr = math.radians(pitch)
      t = 2.0 * (2.0 / 3.0) - 1.0
      r = 1.5 * t / (t * math.sin(pr) + 3.0 * math.cos(pr))
      # 注意单位：_project3d 的世界坐标系是 512*2^z（标准瓦片 256 的 2 倍），
      # 1 世界px 的地面米数 = mpp_screen/2；直接用 mpp_screen 会把提前量放大
      # 2 倍，全屏/缩小一档时车标被推出屏幕底缘（看不到图标）。
      # 注意高度：maprenderd 帧宽有 1280 上限（llvmpipe 性能保护），全屏时
      # 帧高(621) ≠ 面板高(1020)，H 必须用实际帧高，否则车标落到 ~82% 甚至出屏。
      frame_h = self._mr._tex_size[1] or panel.height
      cam_lat0, cam_lon0 = _dest_point(lat0, lon0, bearing, r * frame_h * mpp_screen * 0.5)

    # ---- 底图：向 maprenderd 发相机，取回帧 ----
    self._mr.send_cam(cam_lat0, cam_lon0, zoom, bearing, int(panel.width), int(panel.height), pitch)
    mpp_view = mpp_screen  # 无底图时 overlay/比例尺退化为相机 mpp
    tex = self._mr.frame_texture(sm)
    scale, src_x, src_y = 1.0, 0.0, 0.0
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

    # 3D 模式下用与 maplibre 一致的透视投影（参数同已标定的 _project3d）
    proj3d = None
    if pitch > 0.5 and tex is not None and rl.is_texture_valid(tex):
      fw, fh = self._mr._tex_size
      proj3d = (fw, fh, math.radians(bearing), math.radians(pitch))

    def _frame_to_panel(sx, sy):
      return px + (sx - src_x) * scale, py + (sy - src_y) * scale

    def to_screen(lat, lon):
      if proj3d is not None:
        fw, fh, brad, prad = proj3d
        r = _project3d(lat, lon, cam_lat0, cam_lon0, zoom, brad, prad, fw, fh)
        if r is None:
          return None  # 相机后方，跳过
        return _frame_to_panel(*r)
      mx, my = _mercator_xy(lat, lon)
      dx = (mx - cx_w) * meters_per_rad_x
      dy = (my - cy_w) * EARTH_R
      sx = dx * math.cos(b) - dy * math.sin(b)
      sy = dx * math.sin(b) + dy * math.cos(b)
      return cx + sx / mpp_view, cy - sy / mpp_view

    # 预测路径蓝线已移除（用户要求不显示）

    # ---- 自车图标：蓝色圆形带白色向上箭头 (ego_circle_A.png) ----
    if not hasattr(self, '_ego_loaded'):
      self._ego_loaded = True
      try:
        self._tex_ego = gui_app.texture("icons/ego_circle_A.png", 128, 128)
      except Exception as _e:
        print(f"[sp_map_panel] ego icon load fail: {_e}", flush=True)
        self._tex_ego = None
    if getattr(self, '_tex_ego', None) is not None:
      if pitch > 0.5:
        # 3D：图标压扁贴地（垂直向按俯仰角压缩），位置用透视投影（导航视角下在
        # 屏幕下 1/3），朝向 = 绝对航向 - 平滑视图航向——转向期间视图航向滞后，
        # 差值让箭头咬住真实路面延伸方向。
        ego_brg = gps.bearingDeg if sm.valid['gpsLocationExternal'] else 0.0
        icon_rot = ego_brg if ui_state.map_orientation == 1 else (ego_brg - bearing) % 360.0
        # 图标尺寸随缩放换算：z16（100m 档，约 1.8m/px）固定 128px 相当于 234m 长的
        # 巨毯盖在城市上空——这才是“漂浮在天空”的主因（挤出高度 10px 只是次要）。
        # 按 44m 世界长度换算。下限 96px = 20m 档（z18）的自然尺寸：50m 及更粗的
        # 比例保持该大小（用户要求），只有放大到 20m 以内才继续长到 128px。
        isz = max(96.0, min(128.0, 44.0 / max(mpp_view, 1e-3)))
        dh = isz * math.cos(math.radians(pitch))
        # 图标锚在 1.5m 世界高度（真实车辆高度）：任何比例下都用海拔投影，
        # 而非陷入地面；阴影仍留在地面点，车标微微悬于路面、影子在下方。
        pos_ground = to_screen(lat0, lon0)
        pos = pos_ground
        if proj3d is not None:
          fw, fh, brad, prad = proj3d
          r3 = _project3d(lat0, lon0, cam_lat0, cam_lon0, zoom, brad, prad, fw, fh, alt_m=1.5)
          if r3 is not None:
            pos = _frame_to_panel(*r3)
        ex, ey = pos if pos is not None else (cx, cy)
        gx, gy = pos_ground if pos_ground is not None else (ex, ey)
        # 厚度感：地面阴影 + 向上偏移的暗色副本（挤出侧壁）+ 主图标。
        # 屏幕上方向 ≈ 地面远离相机方向，偏移副本露出的边即车标厚度。
        # 阴影紧贴图标正下方（勿偏右下/过大，否则图标显得飞高）。
        # 注意：图标中心已修正为正对地面点，阴影若仍画在地面点会被图标(0.5isz×0.5dh)
        # 完全盖住；下移 0.32dh、压扁到 0.30dh，让下缘露出图标底约 0.1dh（~8px），
        # 两侧仍藏在图标内——只露出底部一弯月牙，保持"贴地"观感。
        # 挤出高度按真实世界高度(4m)随缩放换算：固定 10px 在 z16 相当于 18m 高柱。
        ext = max(1.0, min(24.0, 4.0 / max(mpp_view, 1e-3)))
        rl.draw_ellipse(int(gx), int(gy + dh * 0.32), isz * 0.40, max(dh * 0.30, 6.0), rl.Color(0, 0, 0, 100))
        # 注意 raylib DrawTexturePro 语义：dest.x/y 是旋转轴心落点，最终位置 = dest - origin。
        # dest 再预减半个图标尺寸会双份补偿，图标中心固定偏左 isz/2、偏上 dh/2（任何比例尺
        # 都一样），2D 的 draw_texture_ex 无 origin 所以正常。dest 直接给目标中心点即可。
        rl.draw_texture_pro(self._tex_ego, rl.Rectangle(0, 0, 128, 128),
                            rl.Rectangle(ex, ey - ext, isz, dh),
                            rl.Vector2(isz / 2, dh / 2), float(icon_rot), rl.Color(56, 84, 140, 255))
        rl.draw_texture_pro(self._tex_ego, rl.Rectangle(0, 0, 128, 128),
                            rl.Rectangle(ex, ey, isz, dh),
                            rl.Vector2(isz / 2, dh / 2), float(icon_rot), rl.WHITE)
      else:
        rot_deg = gps.bearingDeg if ui_state.map_orientation == 1 and sm.valid['gpsLocationExternal'] else 0.0
        half = 64
        rl.draw_texture_ex(self._tex_ego, rl.Vector2(cx - half, cy - half), float(rot_deg), 1.0, rl.WHITE)

    name = out.roadName or out.wayRef
    if name:
      name = _cjk_road_name(name)
    self._card(px + 16, py + 14, panel.width - 32, 128)
    if data_ok and name:
      # Use the system fallback font (NotoSansCJKsc with full CJK coverage
      # as configured by the user in application.py / po file). Don't load a
      # second atlas — that wastes raylib memory.
      font = font_fallback(self._font_bold if not match_dim else self._font)
      color = TEXT if not match_dim else TEXT_DIM
      rl.draw_text_ex(font, name, rl.Vector2(px + 34, py + 33), 90, 0, color)
    else:
      rl.draw_text_ex(self._font, "offline map: no data", rl.Vector2(px + 34, py + 46), 64, 0, TEXT_NONE)

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
    # 缩放 +/- 按钮与比例尺：整体底部居中，避开右下角地图 launcher 图标
    # （handle_tap 走缓存 rect，命中随绘制自动一致）
    zr, gap = ZOOM_BTN_R, 14
    group_w = zr * 4 + gap * 2 + bar_px
    group_cx = px + panel.width / 2
    plus_cx = group_cx + group_w / 2 - zr
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
      # 左上角路名卡加高后移到左侧下方，避开右上角导航图标列
      rl.draw_circle(int(px + 44), int(py + 170), 16, CARD)
      rl.draw_text_ex(self._font_bold, "N", rl.Vector2(px + 38, py + 160), 24, 0, TEXT)

    # nav icon launcher (右下角，与 mode=0 同一位置；mode=2 全屏下也保留以便单击关闭)
    self._draw_nav_icon(rect)

  def _draw_nav_icon(self, rect: rl.Rectangle) -> None:
    # 模仿 exp_button.py: 圆形黑色背景 + 白色图标
    if self._nav_tex is None:
      return
    # 圆心与左下角司机监控图标严格对称（同一边距、同一高度）
    r = ICON_SIZE // 2 + 10
    cx = int(rect.x + rect.width - NAV_CENTER_OFF)
    cy = int(rect.y + rect.height - NAV_CENTER_OFF)
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
    两种状态固定 110x110、同一位置——切换 2D/3D 时按钮原地不动。"""
    size = 110
    pad_x = 24
    pad_y = 166  # 避开 road name card (高 128 + pad 24)
    return rl.Rectangle(px + pw - size - pad_x, py + pad_y, size, size)

  def _load_3d_icons(self):
    """懒加载 2D/3D 切换图标 texture (绝对路径引用桌面 icon)。"""
    if not hasattr(self, '_3d_icons_loaded'):
      self._3d_icons_loaded = True
      try:
        self._tex_2d = gui_app.texture("icons/map_2d.png", 110, 110)
        self._tex_3d = gui_app.texture("icons/map_3d.png", 110, 110)
      except Exception as e:
        print(f"[mapd] icon load failed: {e}", flush=True)
        self._tex_2d = self._tex_3d = None

  def _draw_3d_toggle_button(self, rect: rl.Rectangle) -> None:
    """绘制 2D/3D 切换图标。rect 必须与 handle_tap 用的 _3d_toggle_hit_rect 完全相同。
    状态反着显示: 当前 2D 显示 3D 图标 (点切到 3D)，当前 3D 显示 2D 图标 (点切回 2D)。"""
    self._load_3d_icons()
    is_3d = ui_state.map_panel_3d_active
    tex = self._tex_3d if not is_3d else self._tex_2d
    if tex is not None:
      rl.draw_texture_ex(tex, rl.Vector2(rect.x, rect.y), 0.0, 1.0, rl.WHITE)

  def hit_test(self, x: float, y: float) -> bool:
    """点 (x,y) 是否落在面板交互区（mode!=0 时为整个 content rect；mode=0 时为 nav icon 圆）。
    供外部（augmented_road_view._handle_mouse_press）抑制边栏点击回调——
    否则点地图/行车画面收缩区的同时会切边栏，布局一跳导致面板按钮失灵。
    mode 1 时左半边是行车画面，点击语义为"收起地图"（见 handle_tap），同样要抑制边栏。"""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return False
    rx, ry, rw, rh = self._last_content_rect
    mode = ui_state.map_panel_mode
    if mode != 0:
      return rx <= x <= rx + rw and ry <= y <= ry + rh
    r = ICON_SIZE // 2 + 10
    cx = rx + rw - NAV_CENTER_OFF
    cy = ry + rh - NAV_CENTER_OFF
    return (x - cx) ** 2 + (y - cy) ** 2 <= r * r

  def handle_tap(self, x: float, y: float) -> None:
    """触屏点击路由：命中 nav icon -> toggle on/off；命中地图 panel + 双击 -> toggle 1<->2."""
    if not ui_state.started or not ui_state.off_line_map_panel:
      return
    rect_x, rect_y, rect_w, rect_h = self._last_content_rect
    # 圆心跟 _draw_nav_icon 一致：与左下司机监控图标对称
    r = ICON_SIZE // 2 + 10
    cx = rect_x + rect_w - NAV_CENTER_OFF
    cy = rect_y + rect_h - NAV_CENTER_OFF
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

    # 命中地图 panel 区域：
    # - mode 2 (全屏地图): 单击 -> 半屏
    # - mode 1 (半屏): 点右半边地图 -> 全屏地图；点左半边行车画面 -> 收起地图 (onroad 全屏)
    mode = ui_state.map_panel_mode
    if mode == 1:
      pw = rect_w * PANEL_W_FRAC
      if x >= rect_x + rect_w - pw:
        ui_state.map_panel_mode = 2
      else:
        ui_state.map_panel_mode = 0
    elif mode == 2:
      ui_state.map_panel_mode = 1
