#!/usr/bin/env python3
"""mapd + tsc-d 集成完整性自检：在 openpilot 仓库根目录运行 `python verify_setup.py`。
纯 stdlib，只读检查，不改任何文件。全部 [OK] 才说明 M1/M2 核心补丁集成到位。"""
import os, re, subprocess, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
def read(p):
    fp = os.path.join(ROOT, p)
    return open(fp, encoding="utf-8").read() if os.path.exists(fp) else None

checks = []
def ck(name, ok, detail=""):
    checks.append((name, ok, detail))
    print(("[OK] " if ok else "[MISS] ") + name + (f"  -- {detail}" if detail and not ok else ""))

# --- 1. scc 核心补丁 ---
mp = read("openpilot/selfdrive/controls/lib/scc/map.py")
ck("map.py: MapCurveEstimator 存在", mp and "class MapCurveEstimator" in mp)
ck("map.py: A1 数据通路", mp and "update_msg" in mp and "reach_distance" in mp)
ctl = read("openpilot/selfdrive/controls/lib/scc/controller.py")
ck("controller: tsc-d 注入点", ctl and "map_est.update_msg(" in ctl)
ck("controller: 伪预测注入", ctl and "MAP_ENTERING_REACH_MARGIN" in ctl)
ck("controller: map 钳位", ctl and "v_map = self.map_est.v_target if" in ctl)
ck("controller: 状态转换日志", ctl and "cloudlog.info" in ctl and "scc-x:" in ctl)
const = read("openpilot/selfdrive/controls/lib/scc/constants.py")
ck("constants: MAP_ENTERING_REACH_MARGIN", const and "MAP_ENTERING_REACH_MARGIN = 1.2" in const)
ck("constants: SCC_MAP_ENABLED_PARAM", const and 'SCC_MAP_ENABLED_PARAM = "SccXMapEnabled"' in const)

# --- 2. 参数注册 ---
pk = read("openpilot/common/params_keys.h")
ck("params: SccXMapEnabled", pk and '"SccXMapEnabled"' in pk)
ck("params: MapdSettings", pk and '"MapdSettings"' in pk)

# --- 3. cereal 定义与服务注册 ---
log_cap = read("openpilot/cereal/log.capnp")
ck("log.capnp: mapdExtendedOut 字段@143", log_cap and "mapdExtendedOut @143 :Custom.MapdExtendedOut;" in log_cap)
ck("log.capnp: mapdIn 字段@144", log_cap and "mapdIn @144 :Custom.MapdIn;" in log_cap)
ck("log.capnp: mapdOut 字段@145", log_cap and "mapdOut @145 :Custom.MapdOut;" in log_cap)
ck("log.capnp: sccXState 字段@154", log_cap and "sccXState @154 :Custom.SccXState;" in log_cap)
ck("log.capnp: 旧 CustomReserved17 引用已清除", log_cap and "Custom.CustomReserved17" not in log_cap)
ck("log.capnp: mapRenderCam 字段@155", log_cap and "mapRenderCam @155 :Custom.MapRenderCam;" in log_cap)
ck("log.capnp: mapRenderFrame 字段@156", log_cap and "mapRenderFrame @156 :Custom.MapRenderFrame;" in log_cap)
cap = read("openpilot/cereal/custom.capnp") or read("cereal/custom.capnp") or ""
ck("capnp: MapdOut", "struct MapdOut" in cap)
ck("capnp: MapdExtendedOut", "struct MapdExtendedOut" in cap and "path @2 :List(MapdPathPoint)" in cap)
ck("capnp: SccXState", "struct SccXState" in cap)
svc = read("openpilot/cereal/services.py") or read("cereal/services.py") or ""
for s in ("mapdOut", "mapdExtendedOut", "mapdIn", "sccXState"):
    ck(f"services: {s} 注册", s in svc)

# --- 4. planner 订阅与发布 ---
pl = read("openpilot/selfdrive/controls/lib/longitudinal_planner.py")
ck("planner: 订阅 mapdExtendedOut", pl and "mapdExtendedOut" in pl)
ck("planner: 发布 sccXState", pl and "sccXState" in pl)

# --- 5. mapd 进程 ---
pc = read("openpilot/system/manager/process_config.py")
ck("manager: mapd 进程注册", pc and 'NativeProcess("mapd"' in pc)
ck("manager: maprenderd 进程注册", pc and 'NativeProcess("maprenderd"' in pc)
ck("maprender 三件套", all(os.path.exists(os.path.join(ROOT, p)) for p in
    ("openpilot/selfdrive/maprender/maprenderd.cc", "openpilot/selfdrive/maprender/qoi.h",
     "openpilot/selfdrive/maprender/CMakeLists.txt")))
mapd_bin = os.path.join(ROOT, "openpilot/selfdrive/mapd/mapd")
ck("mapd 二进制就位", os.path.exists(mapd_bin))

# --- 5.5 UI（上车调试版） ---
ck("UI: sp_map_panel.py 存在", os.path.exists(os.path.join(ROOT, "openpilot/selfdrive/ui/onroad/sp_map_panel.py")))
ui_arv = read("openpilot/selfdrive/ui/onroad/augmented_road_view.py")
ck("UI: 面板已接入 onroad", ui_arv and "sp_map_panel import MapPanel" in ui_arv and "_map_panel.render(" in ui_arv)
ui_st = read("openpilot/selfdrive/ui/ui_state.py")
ck("UI: sm 订阅 mapd 消息", ui_st and '"mapdOut"' in ui_st and '"mapdExtendedOut"' in ui_st)
ck("UI: 面板参数读取", ui_st and "map_panel_enabled" in ui_st and "map_orientation" in ui_st)
ui_tg = read("openpilot/selfdrive/ui/layouts/settings/toggles.py")
ck("UI(策略): 切换面板无 SCC-X 区块", (ui_tg is None) or ("SccXMapEnabled" not in ui_tg))

# --- 6. 单测 ---
r = subprocess.run([sys.executable, "-m", "pytest", "-x", "-q",
                    os.path.join(ROOT, "openpilot/selfdrive/controls/lib/scc/tests/test_map_estimator.py")],
                   capture_output=True, text=True)
ck("单测通过", r.returncode == 0, r.stdout[-300:] + r.stderr[-300:])

n_ok = sum(1 for _, ok, _ in checks if ok)
print(f"\n== {n_ok}/{len(checks)} 项通过 ==")
sys.exit(0 if n_ok == len(checks) else 1)
