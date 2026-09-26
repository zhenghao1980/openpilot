#!/usr/bin/env python3
"""回退 classic TogglesLayout 中的 SCC-X 区块（开发期开关统一驻留开发者面板）。

v0.7 包曾把三个 SCC-X 开关放进 layouts/settings/toggles.py——按本分支工作流
（新开关先进 mici 开发者面板，稳定后再晋升切换面板），此改动应撤回。
本脚本精确移除 DESCRIPTIONS 与 _toggle_defs 中的两个 SCC-X 区块，其余逐字节不动。
已用 git checkout 恢复过该文件的无需运行（幂等，检测到即跳过）。

仓库根目录运行：python fix_toggles_revert.py
"""
import pathlib, sys

p = pathlib.Path("openpilot/selfdrive/ui/layouts/settings/toggles.py")
if not p.exists():
    sys.exit("找不到 layouts/settings/toggles.py，请在仓库根目录运行")
src = p.read_text(encoding="utf-8")

if "SccXMapEnabled" not in src:
    print("[skip] toggles.py 无 SCC-X 区块（已是 pristine 或未应用过），无需回退"); sys.exit(0)

BLOCKS = [
    ("""  # SCC-X (mapd 功能说明书 v0.7): hot-updated at 1 Hz in the controller, so
  # these toggles never need a restart and stay switchable while driving
  # (switch = intent; effectiveness is shown by the on-road glyphs).
  "SccXEnabled": tr_noop(
    "SCC-X fused curve-speed control (vision + offline map). Only ever lowers "
    "the target speed; disabling returns stock behavior immediately."
  ),
  "SccXMapEnabled": tr_noop(
    "Use offline OSM map curvature as a curve-speed source (tsc-d). Requires "
    "downloaded map data; the road-name banner greys out when map data is absent."
  ),
  "MapPanelEnabled": tr_noop(
    "Show the right-half offline map panel (matched road, driven trail, nearby "
    "roads). Debug build: fixed panel, orientation via MapOrientationMode param."
  ),
""", "DESCRIPTIONS 区块"),
    ("""      # SCC-X block (v0.7): no restart, never blocked while engaged
      "SccXEnabled": (
        lambda: tr("SCC-X Curve Control"),
        DESCRIPTIONS["SccXEnabled"],
        "speed_limit.png",
        False,
      ),
      "SccXMapEnabled": (
        lambda: tr("SCC-X Map Curve Source (tsc-d)"),
        DESCRIPTIONS["SccXMapEnabled"],
        "speed_limit.png",
        False,
      ),
      "MapPanelEnabled": (
        lambda: tr("Map Panel (debug)"),
        DESCRIPTIONS["MapPanelEnabled"],
        "metric.png",
        False,
      ),
""", "_toggle_defs 区块"),
]
for block, desc in BLOCKS:
    n = src.count(block)
    if n == 0:
        print(f"[warn] 区块「{desc}」未找到——可能已被部分清理，继续"); continue
    if n > 1:
        sys.exit(f"[ERROR] 区块「{desc}」匹配 {n} 处（预期 ≤1），请手工核对")
    src = src.replace(block, "")

p.write_text(src, encoding="utf-8")
chk = p.read_text(encoding="utf-8")
assert "SccXMapEnabled" not in chk and "SccXEnabled" not in chk.replace("SccXEnabledToggle", "")
print("[OK] toggles.py 已回退：SCC-X 开关统一驻留开发者面板（mici developer）")
