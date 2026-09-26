"""
PC tizi UI wrapper (2026-09-08 实证, WSL + WSLg/DISPLAY=:0 跑通 tizi Big UI 在 Windows 桌面).

放 fork 根 (zh-fork-native/) 直接用:
  DISPLAY=:0 XDG_RUNTIME_DIR=/tmp LD_PRELOAD=/tmp/fake_libEGL.so.1 \
    python3 ./run_ui_tizi.py > mgr.log 2>&1 &

要看到窗口必须 WSLg 启用 (Windows 11 22H2+ 默认 WSLg), Xvfb 看不到.
DISPLAY=:99 (Xvfb) 也能跑, 但 raylib 渲染到 GL backbuffer 不进 X11 composite,
需要 raylib load_image_from_screen + export_image 直接出 PNG.

monkey patch 4 层 PC 假问题, 源码不动:
  1. AGNOS / devicetree/model / VERSION / proc/cmdline -> fake
  2. os.path.isfile / builtins.open -> 假文件 + 容忍 OSError
  3. realtime.config_realtime_process + os.sched_setscheduler -> no-op
  4. HardwareComma.get_voltage/current/serial/get_cmdline/set_ir_power -> stub
  5. openpilot.common.hardware.hw.Paths -> 改写 Paths 类, swaglog/persist 走 ~/.
     (不能 monkey-patch module-level Paths 字段, 只能整个类替换)
"""
import os, sys, builtins

os.environ["LOG_ROOT"] = "/home/zheng/zh-fork-native/data/.comma/media/0/realdata"
os.environ["COMMA_CACHE"] = "/tmp/comma_cache"
os.environ["DISPLAY"] = ":0"
os.environ["XDG_RUNTIME_DIR"] = "/tmp"
os.environ["PYTHONPATH"] = "/home/zheng/openpilot:/home/zheng/openpilot/openpilot:" + os.environ.get("PYTHONPATH", "")

# EGL shim: 现代 Mesa (24+) 把 eglCreateImageKHR 后缀合并, cffi 找不到
try:
    import _cffi_backend
    _orig_load_library = _cffi_backend.load_library
    def _patched_load_library(name, flags=0):
        if isinstance(name, str) and "EGL" in name:
            try:
                return _orig_load_library("/tmp/libEGL.so", flags)
            except OSError:
                return _orig_load_library(name, flags)
        return _orig_load_library(name, flags)
    _cffi_backend.load_library = _patched_load_library
except ImportError:
    pass

class _FakeFile:
    def __init__(self, content=""): self.content = content
    def __enter__(self): return self
    def __exit__(self, *a): return False
    def read(self, *a, **kw): return self.content
    def readline(self, *a, **kw): return self.content
    def write(self, *a, **kw): return len(a[0]) if a else 0
    def fileno(self): return -1
    def close(self): pass
    def flush(self): pass

_FAKE = {
    "/AGNOS": "",
    "/sys/firmware/devicetree/base/model": "tizi\n",
    "/VERSION": "99.99\n",
    "/proc/cmdline": "androidboot.serialno=PCFAKE0000 androidboot.mode=tizi\n",
}
_orig_open = builtins.open
def _patched_open(p, *a, **kw):
    if p in _FAKE: return _FakeFile(_FAKE[p])
    try:
        return _orig_open(p, *a, **kw)
    except (FileNotFoundError, PermissionError, OSError):
        return _FakeFile("")
builtins.open = _patched_open

_orig_isfile = os.path.isfile
# Per skill recipe §0b: NOT patching /AGNOS to True -- keep HARDWARE = HardwarePc,
# so COMMA_HARDWARE=False and cameraview skips EGL init (which fails on PC).
def _patched_isfile(p):
    return _orig_isfile(p)
os.path.isfile = _patched_isfile

from openpilot.common import realtime
realtime.config_realtime_process = lambda *a, **kw: None
def _no_op(*a, **kw): pass
os.sched_setscheduler = lambda pid, policy, param: None
os.sched_setaffinity = _no_op

from openpilot.common.hardware.comma import hardware as hw_mod
hw_mod.HardwareComma.get_voltage = lambda self: 12000
hw_mod.HardwareComma.get_current = lambda self: 0
hw_mod.HardwareComma.set_ir_power = lambda self, percent: None
hw_mod.HardwareComma.get_serial = lambda self: "PCFAKE0000"
hw_mod.HardwareBase.get_cmdline = lambda self: {"androidboot.serialno": "PCFAKE0000", "androidboot.mode": "tizi"}
hw_mod.HardwareBase.set_power_save = lambda self, on: None
hw_mod.HardwareBase.reboot = lambda self: None
hw_mod.HardwareBase.read_irq_lock = _no_op

import openpilot.common.hardware.hw as hw_paths
class _PathsOverride:
    @staticmethod
    def comma_home(): return os.path.join(os.path.expanduser("~"), ".comma")
    @staticmethod
    def log_root():
        return os.environ.get("LOG_ROOT", os.path.join(os.path.expanduser("~"), ".comma", "media", "0", "realdata"))
    @staticmethod
    def swaglog_root(): return os.path.join(os.path.expanduser("~"), ".comma", "log")
    @staticmethod
    def swaglog_ipc(): return "ipc:///tmp/logmessage"
    @staticmethod
    def download_cache_root(): return os.environ.get("COMMA_CACHE", "/tmp/comma_cache") + "/"
    @staticmethod
    def persist_root(): return os.path.join(os.path.expanduser("~"), ".comma", "persist")
    @staticmethod
    def config_root(): return os.path.join(os.path.expanduser("~"), ".comma")
    @staticmethod
    def shm_path(): return "/dev/shm"
hw_paths.Paths = _PathsOverride

import openpilot.common.hardware
# 注意: 这一步之前 HARDWARE 已被 __init__.py 设过 (= AGNOS=True -> HardwareComma),
# 我们替换 module 的 HARDWARE 给子进程用.
openpilot.common.hardware.HARDWARE = hw_mod.HardwareComma()

# Per skill recipe §0b step 3: use PC path + monkey-patch HardwarePc.get_device_type
# to return 'tizi'. This makes /AGNOS check fail (no fake patch on isfile) so HARDWARE
# becomes HardwarePc, COMMA_HARDWARE=False, and cameraview skips EGL init (which would
# otherwise fail on PC). big_ui() returns True via BIG=1 env.
import openpilot.common.hardware.pc.hardware as pc_hw
pc_hw.HardwarePc.get_device_type = lambda self: "tizi"

from openpilot.common.hardware import HARDWARE
print("HARDWARE:", type(HARDWARE).__name__, "device_type:", HARDWARE.get_device_type(), flush=True)
import openpilot.system.ui.lib.application as app
print("big_ui:", app.gui_app.big_ui(), flush=True)

# Monkey-patch ThermalZone.read: tolerate empty /sys/devices/virtual/thermal/thermal_zone*/temp
# (PC has files but contents are empty string; base.py only catches FileNotFoundError, not ValueError)
import openpilot.common.hardware.base as _hw_base
def _safe_tz_read(self):
    if self.zone_number < 0:
        try:
            for n in os.listdir("/sys/devices/virtual/thermal"):
                if not n.startswith("thermal_zone"):
                    continue
                with open(os.path.join("/sys/devices/virtual/thermal", n, "type")) as f:
                    if f.read().strip() == self.name:
                        self.zone_number = int(n.removeprefix("thermal_zone"))
                        break
        except (OSError, ValueError):
            return 0.0
    try:
        with open(f"/sys/devices/virtual/thermal/thermal_zone{self.zone_number}/temp") as f:
            raw = f.read().strip()
            return int(raw) / self.scale if raw else 0.0
    except (FileNotFoundError, ValueError, OSError):
        return 0.0
_hw_base.ThermalZone.read = _safe_tz_read
print("[hwmon] ThermalZone.read patched for empty PC sysfs", flush=True)

# Monkey-patch HARDWARE.get_device_type to return capnp enum int (6=tizi) instead of string,
# so msg.deviceState.deviceType write doesn't fail with 'enum has no such enumerant: tizi'
import types as _types_tizi
HARDWARE.get_device_type = _types_tizi.MethodType(lambda self: 6, HARDWARE)
print("[device] HARDWARE.get_device_type returns int 6 (tizi)", flush=True)

# Stub mici.layouts.main and home so ui.py's `from mici.layouts.main import MiciMainLayout`
# doesn't trigger import-time validation against ui_state. We're BIG_UI=True, never use mici.
import sys as _sys, types as _types_mici

class _StubWidget:
  def __init__(self, *a, **kw): pass
  def __getattr__(self, _n): return _StubWidget()
  def __call__(self, *a, **kw): return _StubWidget()

class _StubMiciMainLayout:
  def __init__(self, *a, **kw): pass

_mici_main_mod = _types_mici.ModuleType("openpilot.selfdrive.ui.mici.layouts.main")
_mici_main_mod.MiciMainLayout = _StubMiciMainLayout
_sys.modules["openpilot.selfdrive.ui.mici.layouts.main"] = _mici_main_mod
_mici_home_mod = _types_mici.ModuleType("openpilot.selfdrive.ui.mici.layouts.home")
_mici_home_mod.MiciHomeLayout = _StubMiciMainLayout
_sys.modules["openpilot.selfdrive.ui.mici.layouts.home"] = _mici_home_mod
# Pre-stub other mici modules likely imported at module-level with their common names.
# DO NOT stub openpilot.selfdrive.ui.ui_state - that's the real module we need.

# Force onroad mode for PC demo: override UIState._update_state so self.started=True
# regardless of panda ignition / deviceState.started. sp_map_panel.render() gates on
# ui_state.started, so without this the map panel never renders on PC (no real panda).
import openpilot.selfdrive.ui.ui_state as _ui_state_mod
_orig_update_state = _ui_state_mod.UIState._update_state
def _patched_update_state(self):
  _orig_update_state(self)
  self.started = True  # force onroad so onroad views (incl. sp_map_panel) render
_ui_state_mod.UIState._update_state = _patched_update_state
print("[onroad] UIState.started forced True via _update_state hook", flush=True)

# Note: deviceState publisher removed — manager.py is patched on disk to force
# `started = True` regardless of incoming deviceState messages (see __main__ below).

# In-process fake mapd service: must run inside UI process so PubMaster socket is
# shared with the UI's SubMaster. External processes don't share messaging sockets.
# Pre-build the 20-point path & 2 nearby roads once.
_FAKE_GPS_PATH = [
  (43.81800, 125.27800, 0.0), (43.81680, 125.27800, 0.0), (43.81560, 125.27800, 0.0),
  (43.81440, 125.27800, 0.0), (43.81320, 125.27800, 0.0), (43.81200, 125.27800, 0.0),
  (43.81080, 125.27800, 0.0), (43.80960, 125.27800, 0.0), (43.80840, 125.27800, 0.0),
  (43.80720, 125.27800, 0.0), (43.80600, 125.27750, 0.0001), (43.80550, 125.27620, 0.0008),
  (43.80540, 125.27470, 0.0008), (43.80580, 125.27340, 0.0004), (43.80640, 125.27250, 0.0001),
  (43.80760, 125.27220, 0.0), (43.80880, 125.27200, 0.0), (43.81000, 125.27180, 0.0),
  (43.81120, 125.27160, 0.0), (43.81240, 125.27140, 0.0),
]
_FAKE_NEARBY = [
  (3, "Nanhu East Rd", "E10", [(43.818, 125.282), (43.798, 125.282)]),
  (5, "Ziyou Rd", "G302", [(43.812, 125.265), (43.812, 125.290)]),
]

def _haversine_bearing(lat1, lon1, lat2, lon2):
  import math as _m
  lat1, lon1, lat2, lon2 = _m.radians(lat1), _m.radians(lon1), _m.radians(lat2), _m.radians(lon2)
  dlon = lon2 - lon1
  x = _m.sin(dlon) * _m.cos(lat2)
  y = _m.cos(lat1) * _m.sin(lat2) - _m.sin(lat1) * _m.cos(lat2) * _m.cos(dlon)
  return (_m.degrees(_m.atan2(x, y)) + 360.0) % 360.0

def _run_fake_mapd_inproc():
  """Run inside UI process: PubMaster shares socket namespace with SubMaster."""
  import threading as _th, time as _t
  from openpilot.cereal import messaging, custom
  def loop():
    pm = messaging.PubMaster(['mapdOut', 'mapdExtendedOut', 'gpsLocationExternal', 'carState'])
    print("[fake-mapd-inproc] starting", flush=True)
    _t.sleep(2.0)
    for tick in range(60 * 60 * 4):
      idx = tick % len(_FAKE_GPS_PATH)
      clat, clon, ccrv = _FAKE_GPS_PATH[idx]
      nlat, nlon, _ = _FAKE_GPS_PATH[(idx + 1) % len(_FAKE_GPS_PATH)]
      brg = _haversine_bearing(clat, clon, nlat, nlon)
      # gps
      g = messaging.new_message('gpsLocationExternal')
      g.gpsLocationExternal.latitude = float(clat); g.gpsLocationExternal.longitude = float(clon)
      g.gpsLocationExternal.bearingDeg = float(brg); g.gpsLocationExternal.horizontalAccuracy = 5.0
      g.gpsLocationExternal.speed = 13.5; g.gpsLocationExternal.unixTimestampMillis = int(_t.time() * 1e3)
      pm.send('gpsLocationExternal', g)
      # carState
      cs = messaging.new_message('carState')
      cs.carState.vEgo = 13.5; cs.carState.aEgo = 0.0
      pm.send('carState', cs)
      # mapdOut
      mo = messaging.new_message('mapdOut')
      mo.mapdOut.roadName = "Yatai Street"; mo.mapdOut.wayRef = "G302"
      mo.mapdOut.tileLoaded = True; mo.mapdOut.waySelectionType = 0  # current
      mo.mapdOut.mapCurveSpeed = 9.0 if ccrv > 0.0005 else 12.5
      mo.mapdOut.visionCurveSpeed = 11.0; mo.mapdOut.speedLimit = 12.5
      mo.mapdOut.lanes = 2; mo.mapdOut.highwayClass = 5
      pm.send('mapdOut', mo)
      # mapdExtendedOut
      me = custom.MapdExtendedOut.new_message()
      me.position.latitude = float(clat); me.position.longitude = float(clon)
      me.loopRateAverage = 1.0; me.loopRateMin = 0.95
      me.path = [custom.MapdPathPoint.new_message() for _ in _FAKE_GPS_PATH]
      for i, (plat, plon, pcrv) in enumerate(_FAKE_GPS_PATH):
        me.path[i].latitude = float(plat); me.path[i].longitude = float(plon)
        me.path[i].curvature = float(pcrv)
        me.path[i].targetVelocity = 9.0 if pcrv > 0.0005 else 12.5
      me.nearbyRoads = [custom.MapdRoadSegment.new_message() for _ in _FAKE_NEARBY]
      for j, (hcls, name, ref, pts) in enumerate(_FAKE_NEARBY):
        me.nearbyRoads[j].highwayClass = hcls; me.nearbyRoads[j].name = name; me.nearbyRoads[j].ref = ref
        me.nearbyRoads[j].points = [custom.MapdPosition.new_message() for _ in pts]
        for k, (rlat, rlon) in enumerate(pts):
          me.nearbyRoads[j].points[k].latitude = float(rlat); me.nearbyRoads[j].points[k].longitude = float(rlon)
      me_msg = messaging.new_message('mapdExtendedOut')
      me_msg.mapdExtendedOut = me
      pm.send('mapdExtendedOut', me_msg)
      _t.sleep(1.0)
  _th.Thread(target=loop, daemon=True, name="fake-mapd-inproc").start()

# NOTE: do NOT call _run_fake_mapd_inproc() here. The wrapper IS the manager process;
# running it in the wrapper would bind 4 PubSockets in this process. ui is forked
# as a child and inherits those sockets, so when ui's ui.py also calls
# fake_mapd_inproc.spawn(), the second PubMaster construction fails with
# 'Address already in use'. Spawn only in ui.py (try import guard).

from openpilot.system.manager import manager
if __name__ == "__main__":
    # Patch manager to always think we're onroad so it starts only_onroad
    # processes (mapd / maprenderd / plannerd / etc.). Done by replacing
    # the manager module's source on disk before importing manager.main.
    import openpilot.system.manager.manager as _mm
    _mm_path = _mm.__file__
    _mm_src = open(_mm_path).read()
    if "started = sm['deviceState'].started" in _mm_src and "_PC_FORCE_ONROAD" not in _mm_src:
        _mm_src = _mm_src.replace(
            "started = sm['deviceState'].started",
            "started = sm.valid['deviceState'] and sm['deviceState'].started or True  # _PC_FORCE_ONROAD",
        )
        # Patch on disk so manager_thread() reads it on next call.
        open(_mm_path, "w").write(_mm_src)
        # Re-import to pick up the change.
        import importlib
        importlib.reload(_mm)
        import openpilot.system.manager as _mgr
        importlib.reload(_mgr)
    try: manager.main()
    except KeyboardInterrupt: print("KeyboardInterrupt", flush=True)
