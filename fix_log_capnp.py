#!/usr/bin/env python3
"""log.capnp 字段引用修复（配合 custom.capnp 的 CR17/18/19 更名）。
在仓库根目录运行：python fix_log_capnp.py
- customReserved17/18/19 @143/144/145 字段更名为 mapdExtendedOut/mapdIn/mapdOut
  （ordinal 与类型 ID 不变，上游 mapd 集成推荐用法；services.py 的服务名即由此字段名决定）
- 新增 sccXState @154 :Custom.SccXState（Event union 当前最大 ordinal 为 @153）
幂等：已应用则直接提示退出；锚点找不到则报错并给出诊断。
"""
import pathlib, sys

p = pathlib.Path("openpilot/cereal/log.capnp")
if not p.exists():
    sys.exit("找不到 openpilot/cereal/log.capnp，请在仓库根目录运行")

src = p.read_text(encoding="utf-8")

OLD = """    customReserved17 @143 :Custom.CustomReserved17;
    customReserved18 @144 :Custom.CustomReserved18;
    customReserved19 @145 :Custom.CustomReserved19;"""
NEW = """    mapdExtendedOut @143 :Custom.MapdExtendedOut;
    mapdIn @144 :Custom.MapdIn;
    mapdOut @145 :Custom.MapdOut;
    sccXState @154 :Custom.SccXState;
    mapRenderCam @155 :Custom.MapRenderCam;
    mapRenderFrame @156 :Custom.MapRenderFrame;"""

if "mapRenderFrame @156 :Custom.MapRenderFrame;" in src:
    print("[skip] log.capnp 已修复过（含 maprender 字段），无需重复应用"); sys.exit(0)
MAPRENDER_ONLY = ("    sccXState @154 :Custom.SccXState;",)
if "mapdOut @145 :Custom.MapdOut;" in src and "mapRenderFrame" not in src:
    a = "    sccXState @154 :Custom.SccXState;"
    assert src.count(a) == 1
    src = src.replace(a, a + "\n    mapRenderCam @155 :Custom.MapRenderCam;\n    mapRenderFrame @156 :Custom.MapRenderFrame;")
    p.write_text(src, encoding="utf-8")
    print("[OK] log.capnp 追加 maprender 字段（mapd 部分此前已修复）"); sys.exit(0)

if src.count(OLD) != 1:
    n = src.count(OLD)
    sys.exit(f"锚点匹配 {n} 处（预期 1）——log.capnp 可能与本包基线(master 2026-09-23)不一致，"
             "请手动核对 customReserved17/18/19 三行后按 README-GUIDE §2 手工修改")

src = src.replace(OLD, NEW)
p.write_text(src, encoding="utf-8")

# 应用后校验
chk = p.read_text(encoding="utf-8")
for s in ("mapdExtendedOut @143 :Custom.MapdExtendedOut;",
          "mapdIn @144 :Custom.MapdIn;",
          "mapdOut @145 :Custom.MapdOut;",
          "sccXState @154 :Custom.SccXState;"):
    assert s in chk, f"校验失败: {s}"
assert "Custom.CustomReserved17" not in chk
print("[OK] log.capnp 修复完成：3 个字段更名 + sccXState @154 新增，校验通过")
