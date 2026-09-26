# mapd + tsc-d (SCC-M) 全量文件包 v0.7 · 整文件替换版

## 使用方式（全部 9 个文件直接覆盖仓库对应路径）
    git checkout -b feature/mapd-sccx
    # 解压后按目录结构覆盖（包内路径 = 仓库相对路径）：
    cp -r mapd-sccx-full-v0.7/openpilot/* <repo>/openpilot/
    cp mapd-sccx-full-v0.7/verify_setup.py <repo>/
    git diff --stat   # 复核：改动面应只有预期增改（见下表）
    python fix_log_capnp.py
python fix_ui_state.py   # ui_state.py 补丁（幂等，必做）              # 修复 log.capnp 字段引用（幂等，必做）
cd <repo> && python verify_setup.py   # 23 项全 [OK] 再继续

## 改动面清单（git diff 应该看到的全部内容）
| 文件 | 改动 |
|---|---|
| cereal/custom.capnp | CustomReserved17/18/19 原位更名为 MapdExtendedOut/MapdIn/MapdOut（ID 不变，上游推荐用法）；追加 mapd 枚举/小 struct + MapdRoadSegment + nearbyRoads @6 + SccXState |
| cereal/services.py | 注册 mapdOut(20Hz) / mapdExtendedOut(1Hz) / mapdIn / sccXState(20Hz)，全量进 qlog |
| common/params_keys.h | +SccXMapEnabled(默认1) / MapdSettings / MapPanelEnabled / MapOrientationMode |
| controls/lib/scc/map.py | 整文件替换：MapCurveEstimator 完整实现（A1，8/8 单测） |
| controls/lib/scc/constants.py | 尾追加 MAP_ENTERING_REACH_MARGIN=1.2 / SCC_MAP_ENABLED_PARAM |
| controls/lib/scc/controller.py | 5 处：cloudlog、map_enabled、map 钳位、伪预测注入、update_msg 注入 |
| controls/lib/longitudinal_planner.py | _scc_x_out 暂存 + publish() 尾部 sccXState 发送 |
| controls/plannerd.py | SubMaster +mapdOut/mapdExtendedOut；PubMaster +sccXState |
| system/manager/process_config.py | +NativeProcess("mapd", ...) |

## 包外仍需你本地完成（3 件）
1. mapd 官方二进制 → openpilot/selfdrive/mapd/mapd（aarch64 release 或 GOARCH=arm64 交叉编译）
2. openpilot/selfdrive/mapd/SConscript（参照 loggerd 目录的 NativeProcess SConscript 安装二进制）
3. 编译 + 单测：pytest openpilot/selfdrive/controls/lib/scc/tests/test_map_estimator.py

## 验证记录（沙箱 2026-09-25）
- 单测 8/8 实跑通过；全部 Python py_compile 通过
- 除 plannerd.py 外所有文件均为 text/plain verbatim 转录；plannerd.py 源自 HTML 转义渲染、
  缩进按仓库惯例恢复——git diff 时请重点确认它只有两处列表差异
- 未验证：capnp 编译、整机构建、真机（需本地）
