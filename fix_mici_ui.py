#!/usr/bin/env python3
"""mici 开发者设置页补 SCC-X 地图源/地图面板开关（mapd 功能说明书 v0.7）。

背景：SCC-X 开关原本只在 classic TogglesLayout（本次包新增）——若设备渲染 mici
布局（DeveloperLayoutMici，release 隐藏），tsc-d 与地图面板开关不可见。本脚本在
mici 开发者页 _scc_x_toggle 旁插入两个 BigParamControl，行为与 classic 版一致：
绑同一参数、随时可切、release 隐藏。

仓库根目录运行：python fix_mici_ui.py（幂等）。
"""
import pathlib, sys

p = pathlib.Path("openpilot/selfdrive/ui/mici/layouts/settings/developer.py")
if not p.exists():
    sys.exit("找不到 mici/layouts/settings/developer.py，请在仓库根目录运行")
src = p.read_text(encoding="utf-8")

if "SccXMapEnabled" in src:
    print("[skip] mici 开发者页已包含 mapd 开关，无需重复应用"); sys.exit(0)

EDITS = [
    # 1. 在 _decr_toggle 定义前插入两个新控件
    ('    self._decr_toggle = BigParamControl("dec-r radar fusion", "DecrEnabled",',
     '    self._scc_x_map_toggle = BigParamControl("scc-x map curve source", "SccXMapEnabled",\n'
     '                                             description="tsc-d: offline OSM map curvature as a curve-speed source "\n'
     '                                                         "(SCC-M, mapd design v0.7). Requires downloaded map data; "\n'
     '                                                         "the road-name banner greys out when data is absent.")\n'
     '    self._map_panel_toggle = BigParamControl("map panel (debug)", "MapPanelEnabled",\n'
     '                                             description="Right-half offline map panel: matched road, driven trail, "\n'
     '                                                         "nearby roads. Debug build.")\n'
     '    self._decr_toggle = BigParamControl("dec-r radar fusion", "DecrEnabled",',
     "+2 BigParamControl"),
    # 2. 滚动列表
    ("      self._scc_x_toggle,\n      self._decr_toggle,",
     "      self._scc_x_toggle,\n      self._scc_x_map_toggle,\n      self._map_panel_toggle,\n      self._decr_toggle,",
     "scroller add_widgets"),
    # 3. 刷新表
    ('      ("SccXEnabled", self._scc_x_toggle),\n      ("DecrEnabled", self._decr_toggle),',
     '      ("SccXEnabled", self._scc_x_toggle),\n      ("SccXMapEnabled", self._scc_x_map_toggle),\n'
     '      ("MapPanelEnabled", self._map_panel_toggle),\n      ("DecrEnabled", self._decr_toggle),',
     "_refresh_toggles"),
    # 4. release 隐藏组（与 scc-x 一致）
    ("self._scc_x_toggle, self._decr_toggle)",
     "self._scc_x_toggle, self._scc_x_map_toggle, self._map_panel_toggle, self._decr_toggle)",
     "release_blocked_toggles"),
]
for anchor, repl, desc in EDITS:
    n = src.count(anchor)
    if n != 1:
        sys.exit(f"[ERROR] 锚点「{desc}」匹配 {n} 处（预期 1）——文件与 master(2026-09-23) 不一致，请手工核对")
    src = src.replace(anchor, repl)

p.write_text(src, encoding="utf-8")
chk = p.read_text(encoding="utf-8")
assert chk.count("SccXMapEnabled") >= 2 and chk.count("self._scc_x_map_toggle") >= 4
print("[OK] mici 开发者页补丁完成：+tsc-d 开关、+地图面板开关（4 处锚点，幂等）")
