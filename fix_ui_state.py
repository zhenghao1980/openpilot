#!/usr/bin/env python3
"""ui_state.py 的 mapd/maprender 补丁（按标记逐段幂等，兼容三种初始状态）。

兼容：① pristine master ② 只打过第一版补丁（仅 mapdOut/mapdExtendedOut +
map_panel_enabled/orientation，无 mapRenderFrame/map_panel_mode）③ 已全量。
每段先查标记，缺失则在候选锚点中找唯一匹配替换；都找不到则报错。

仓库根目录运行：python fix_ui_state.py
"""
import pathlib, sys

p = pathlib.Path("openpilot/selfdrive/ui/ui_state.py")
if not p.exists():
    sys.exit("找不到 openpilot/selfdrive/ui/ui_state.py，请在仓库根目录运行")
src = p.read_text(encoding="utf-8")

EDITS = [
    ("mapRenderFrame",
     [("""        "testJoystick",
        "rawAudioData",
      ]
    )""",
      """        "testJoystick",
        "rawAudioData",
        "mapdOut",
        "mapdExtendedOut",
        "mapRenderFrame",
      ]
    )"""),
     ("""        "mapdOut",
        "mapdExtendedOut",
      ]""",
      """        "mapdOut",
        "mapdExtendedOut",
        "mapRenderFrame",
      ]""")],
     "SubMaster +mapd/maprender 订阅"),
    ("self.map_panel_mode: int",
     [("""    self.developer_ui: int = int(self.params.get("DevUIInfo") or 0)

    self._params_thread""",
      """    self.developer_ui: int = int(self.params.get("DevUIInfo") or 0)
    # mapd map panel state (mapd 功能说明书 4.3/5, v0.7 debug)
    self.map_panel_enabled: bool = self.params.get_bool("MapPanelEnabled")
    self.map_panel_mode: int = int(self.params.get("MapPanelMode") or 1)
    self.map_orientation: int = int(self.params.get("MapOrientationMode") or 0)

    self._params_thread"""),
     ("""    self.map_orientation: int = int(self.params.get("MapOrientationMode") or 0)

    self._params_thread""",
      """    self.map_orientation: int = int(self.params.get("MapOrientationMode") or 0)
    self.map_panel_mode: int = int(self.params.get("MapPanelMode") or 1)

    self._params_thread""")],
     "_initialize 面板参数"),
    ("self.map_panel_mode = int",
     [("""    self.developer_ui = int(self.params.get("DevUIInfo") or 0)
    if not self.chestnut_compiled:""",
      """    self.developer_ui = int(self.params.get("DevUIInfo") or 0)
    self.map_panel_enabled = self.params.get_bool("MapPanelEnabled")
    self.map_panel_mode = int(self.params.get("MapPanelMode") or 1)
    self.map_orientation = int(self.params.get("MapOrientationMode") or 0)
    if not self.chestnut_compiled:"""),
     ("""    self.map_orientation = int(self.params.get("MapOrientationMode") or 0)
    if not self.chestnut_compiled:""",
      """    self.map_orientation = int(self.params.get("MapOrientationMode") or 0)
    self.map_panel_mode = int(self.params.get("MapPanelMode") or 1)
    if not self.chestnut_compiled:""")],
     "update_params 面板参数"),
]
applied = skipped = 0
for marker, cands, desc in EDITS:
    if marker in src:
        skipped += 1
        continue
    for anchor, repl in cands:
        if src.count(anchor) == 1:
            src = src.replace(anchor, repl)
            applied += 1
            break
    else:
        sys.exit(f"[ERROR] 锚点「{desc}」无唯一匹配——文件与 master 不一致，请手工核对")

p.write_text(src, encoding="utf-8")
chk = p.read_text(encoding="utf-8")
for m in ("mapRenderFrame", "mapdExtendedOut", "map_panel_enabled", "map_panel_mode", "map_orientation"):
    assert m in chk, m
assert "self.selfdriveState" not in chk
print(f"[OK] ui_state.py 补丁完成：应用 {applied} 段，跳过 {skipped} 段")
