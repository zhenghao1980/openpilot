# mapd + tsc-d (SCC-M) 全量包 · 安装 / 编译 / 测试 / 路试 完整指南

适用基线：zhenghao1980/openpilot **master（2026-09-23）** · 设备 comma 3X · VW MLB (B8)
包版本：v0.7 · 2026-09-25 · 对应设计文档《mapd 功能说明书 v0.7》

---

## 0. 这个包是什么 / 不是什么

**是什么**：SCC-X 第三路数据源 tsc-d（离线地图弯道减速）的 M1/M2 全量代码——
9 个仓库文件的完整替换版 + 集成自检脚本 + 8 个单测。
核心语义：mapd（Go，官方二进制，零修改）→ `mapdExtendedOut.path`（原生曲率，1Hz）
→ `MapCurveEstimator` → SCC-X 置信度仲裁 → 状态机伪预测注入进入减速。

**不是什么**（本包不含，需另行处理）：
- raylib UI（限速牌/弯道牌/路名横幅/地图面板/设置页）——全部未到，开关只能用 params 命令
- mapd 官方二进制与 SConscript（见 §4.2/§4.3）
- 周边路网 fork 扩展与地图态势页（M4 内容）
- 任何"已编译/已上车"的保证：沙箱已做 8/8 单测 + 语法/锚点校验，capnp 编译与真机需本地完成

---

## 1. 前置要求

| 项 | 要求 |
|---|---|
| 仓库 | zhenghao1980/openpilot，master，工作区干净（`git status` 无未提交改动） |
| 环境 | Linux / WSL2 / macOS，已配好 openpilot 构建环境（scons、python 依赖） |
| 设备 | comma 3X（aarch64），能 SSH |
| Go | 仅当你要自行交叉编译 mapd 时需要（ otherwise 用官方 release） |

---

## 2. 安装（5 分钟）

```bash
cd <你的仓库根目录>            # 例如 ~/openpilot 或 C:\hermes_works\openpilot 的 WSL 路径
git status                     # 确认干净
git checkout master && git pull
git checkout -b feature/mapd-sccx
git commit --allow-empty -m "baseline before mapd-sccx"

unzip mapd-sccx-full-v0.7.zip -d /tmp/mapd-sccx
cp -r /tmp/mapd-sccx/openpilot/* .
cp /tmp/mapd-sccx/verify_setup.py .
python fix_log_capnp.py
python fix_ui_state.py   # ui_state.py 补丁（幂等，必做）    # 修复 log.capnp 字段引用（幂等，必做）
```

**复核改动面**（这步不可跳过）：
```bash
git status --short    # 应为：9 个 modified + verify_setup.py 新增 + README.md（若在包内）
git diff --stat       # 与 §3 的清单逐项对照
```

### 改动面清单（git diff 应看到的全部内容）

| 文件 | 预期改动 |
|---|---|
| `openpilot/cereal/custom.capnp` | CustomReserved17/18/19 原位更名 MapdExtendedOut/MapdIn/MapdOut（类型 ID 不变，上游推荐用法）；追加 5 枚举 + 4 小 struct + MapdRoadSegment + `nearbyRoads @6` + SccXState |
| `openpilot/cereal/log.capnp` | **本步用 `fix_log_capnp.py` 自动修复**：customReserved17/18/19 字段更名 mapdExtendedOut/mapdIn/mapdOut（@143/144/145 不变），新增 `sccXState @154 :Custom.SccXState` |
| `openpilot/cereal/services.py` | `_services` 尾部 +4 条目：mapdOut(20Hz,qlog10) / mapdExtendedOut(1Hz,qlog1) / mapdIn / sccXState(20Hz,qlog10)，全量进 qlog |
| `openpilot/common/params_keys.h` | 在 SccXEnabled 行后 +4 键：SccXMapEnabled(默认"1") / MapdSettings / MapPanelEnabled / MapOrientationMode |
| `openpilot/selfdrive/controls/lib/scc/map.py` | 整文件替换（旧桩 → MapCurveEstimator 完整实现） |
| `openpilot/selfdrive/controls/lib/scc/constants.py` | 文件尾 +2：MAP_ENTERING_REACH_MARGIN=1.2、SCC_MAP_ENABLED_PARAM |
| `openpilot/selfdrive/controls/lib/scc/controller.py` | 5 处：cloudlog 导入、map_enabled 成员、map 钳位（≥v_cruise 报 0）、entering 伪预测注入、update_msg 注入 |
| `openpilot/selfdrive/controls/lib/longitudinal_planner.py` | __init__ +1 行、update() +1 行、publish() 尾部 +sccXState 发送块 |
| `openpilot/selfdrive/controls/plannerd.py` | SubMaster +mapdOut/mapdExtendedOut；PubMaster +sccXState。**此文件由 HTML 转义渲染还原，重点确认只有这两处列表差异** |
| `openpilot/system/manager/process_config.py` | radard 行后 +1：NativeProcess("mapd", "openpilot/selfdrive/mapd", ["./mapd"], only_onroad) |

任何**清单之外**的改动 hunk → `git checkout -- <file>` 还原该文件后重贴。

---

## 3. 静态验证（1 分钟）

```bash
python verify_setup.py
```
28 项检查（scc 补丁 8 + 参数 2 + cereal 4 + planner 2 + 进程与二进制 2），全 [OK] 才继续。
某一项 [MISS] 的提示会直接告诉你缺什么。

单测（可选，PC 即可跑）：
```bash
pytest openpilot/selfdrive/controls/lib/scc/tests/test_map_estimator.py -v   # 预期 8 passed
```

---

## 4. 编译与 mapd 二进制

### 4.1 编译 openpilot

```bash
cd <仓库根目录>
scons -j$(nproc)        # 部分分支是 ./build.sh 或 uv run scons；以你分支惯例为准
```

capnp schema 由 scons 自动重新生成。**首编常见错误**：
| 报错 | 原因与处理 |
|---|---|
| `Duplicate type ID` | custom.capnp 被重复合并过（贴了两遍 mapd 定义）。`git diff openpilot/cereal/custom.capnp` 检查，保留一份 |
| `MapdOut is not defined` / `sccXState is not defined` | custom.capnp 合并不完整，对照 §2 清单补齐 |
| services.py 相关 NameError | 4 个服务名拼写或缩进错误（须保持 dict 内 4 空格 + 尾逗号） |

### 4.2 放置 mapd 官方二进制（二选一）

**方式 A：官方 release（推荐）**
到 github.com/pfeiferj/mapd 的 Releases 下载 aarch64 的 `mapd`，放至：
```
openpilot/selfdrive/mapd/mapd
chmod +x openpilot/selfdrive/mapd/mapd
```

**方式 B：交叉编译**
```bash
git clone https://github.com/pfeiferj/mapd && cd mapd
GOOS=linux GOARCH=arm64 go build -o mapd .
scp mapd comma:/data/openpilot/selfdrive/mapd/mapd   # 或拷入源码树随构建走
```

### 4.3 selfdrive/mapd/SConscript（让构建带上二进制）

新建 `openpilot/selfdrive/mapd/SConscript`，最小可用模板（参照 system/loggerd 目录的写法）：
```python
Import('env')

# 安装预编译 mapd 二进制（方式 A/B 放置的 mapd 源文件）
env.Command('mapd', 'mapd', Copy('$TARGET', '$SOURCE'))
```
若你的分支对 NativeProcess 目录有额外构建约定，以 `system/camerad/SConscript` 为参照调整。

---

## 4.5 UI（v0.7 上车调试版，3 个文件 + 1 个新文件）

| 文件 | 改动 |
|---|---|
| `ui/layouts/settings/toggles.py` | **撤回**（开发者面板策略）：开关统一驻留 mici 开发者面板，稳定后再晋升此处 |
| `ui/mici/layouts/settings/developer.py` | **fix_mici_ui.py**：在已有 `scc-x curve speed` 开关旁 +tsc-d 地图源、+地图面板两个 BigParamControl（release 隐藏，随时可切） |
| `ui/ui_state.py` | **改用 `fix_ui_state.py` 脚本补丁**（3 锚点 6 行；包内旧整文件版有转录错误，已移除，勿用） |
| `ui/onroad/augmented_road_view.py` | 扩展点接入 MapPanel（import/init/render 三行） |
| `ui/onroad/sp_map_panel.py`（新） | 右半屏地图面板 debug v2（classic OP nav-panel style：深石板底/细灰路网/蓝色路线带光晕/白色导航箭头/顶底圆角卡）：当前匹配道路、breadcrumb 轨迹、周边路网（等 mapd fork 的 nearbyRoads）、路名横幅三态语义、朝向双模式；调色板为文件顶部常量，上车直接改色迭代 |

**调试版已知限制**：滑入滑出动画/手势未做（param 开关面板）；中文字体未显式加载（纯中文路名可能空白，英文/拼音正常）；面板始终渲染在行车画面上层（与 SP 面板重叠区域后续避让）。

## 4.6 瓦片底图流水线（tools/map_tiles，v4 面板必需）

观感与官方早期面板同宗：底图由 MapLibre 引擎按 navigation-night 样式离线渲染。
制备（PC，需联网一次）：
  1. 下载 OpenMapTiles 中国区 mbtiles（data.openmaptiles.org，免费需注册）→ tools/map_tiles/china.mbtiles
  2. cd tools/map_tiles && docker compose up -d   # tileserver-gl :8080
  3. 小范围试跑：python export_tiles.py --bbox 39.75 116.15 40.05 116.55 --zooms 14 15 16 --out ./tiles
  4. 满意后导出常跑城市 → scp -r tiles comma:/data/mapd_tiles
设备运行时全离线；面板按车速自动选 zoom（14/15/16 均可，就近取有数据的级别）。
无瓦片时优雅退化：画布 + mapd 矢量路线照常显示。

### 面板双规格（MapPanelMode）
- `1` = 右半屏（默认）；`2` = 全屏（覆盖整个行车画面，分隔线省略，布局自适应宽度）
- 总开关仍是开发者面板的 MapPanelEnabled；改规格：
  `python -c "from openpilot.common.params import Params; Params().put('MapPanelMode','2')"`（0.2s 热生效）
- 全屏底图按 960×960 帧拉伸，需要更高清晰度调 maprenderd.cc 的 W/H 常量重编

## 4.7 MapLibre GL Native 渲染进程（B 方案，全国矢量）

目录 openpilot/selfdrive/maprender/：maprenderd.cc + mbtiles_file_source.h + CMakeLists.txt。
架构：独立进程，自带 EGL pbuffer 上下文（与 raylib 零冲突），按 UI 发来的
mapRenderCam 相机参数离屏渲染，QOI 编码后经 cereal mapRenderFrame 发回，UI 贴图。
部署：
  1. PC：git submodule vendor maplibre-native → cmake 交叉编译 aarch64（comma 工具链），
     产物 maprenderd 放 openpilot/selfdrive/maprender/
  2. 设备数据（/data/mapd_render/）：china.mbtiles（1-2GB）+ style.json（沿用
     tools/map_tiles 的 Maputnik 工作流）+ glyphs/（fonts.openmaptiles.org 下载）
  3. UI 侧已就绪：sp_maprender_client.py + 面板 v5（相机同构对齐）
联调顺序：先裸跑 maprenderd 看日志出帧 → UI 开 MapPanelEnabled 看底图 → 对叠加层
对齐（CENTER_FRAC/zoom 公式两边一致，偏差在设备上调）。
API 基线（2026-09 已对照 main 头文件核实）：mln::HeadlessFrontend::render(Map&)
为**同步调用**直接返回 PremultipliedImage（无需 renderStill + 观察者）；Map 构造为
Map(RendererFrontend&, MapObserver&, MapOptions, ResourceOptions)；相机 jumpTo(
CameraOptions.withCenter/withZoom/withBearing)。include 需加 platform/default/include。

风险登记：① HeadlessFrontend 在 AGNOS 的 EGL/GLES 适配（最大未知，代码有 ADAPT 标记）
② mbgl API 签名随版本漂移 ③ MbtilesFileSource 需接入 mbgl FileSourceFactory（TODO，
当前为头文件助手） ④ pyray Image 内存构造（有回退） ⑤ glyphs 的 CJK 字体栈可用性。

## 5. 参数开关（UI 已覆盖 3 个；朝向模式仍用命令行）

```bash
# 设备或 PC（仓库环境）上：
python -c "from openpilot.common.params import Params; p=Params(); \
  print('SccXEnabled =', p.get_bool('SccXEnabled'))"          # SCC-X 总开关（先确认已是 true）
python -c "from openpilot.common.params import Params; Params().put_bool('SccXMapEnabled', True)"   # tsc-d 地图源（默认已开）
python -c "from openpilot.common.params import Params; Params().put_bool('SccXMapEnabled', False)"  # 只跑纯视觉基线对比时关
```

---

## 6. 下载离线地图数据（nation.CN）

UI 下载菜单未实现，首装用 SSH + mapd 终端 UI：
```bash
ssh comma
cd /data/openpilot/selfdrive/mapd
./mapd i          # mapd 交互终端：选中国 nation.CN 下载，数百 MB，进度可见可取消
```
前提：设备联网。下载后行车全程离线。

---

## 7. 台架与真机验证

### 7.1 进程与日志观察
```bash
ssh comma
tail -f /data/log/*.log | grep -E 'scc-x|mapd'   # 关注 scc-x: enabled -> entering ... 转换事件
./selfdrive/mapd/mapd i                           # mapd 自带终端：匹配状态/瓦片/下载
```
预期：onroad 后 mapd 进程常驻；进入弯道前 cloudlog 出现 `scc-x: enabled -> entering (src=..., v_arb=..., map=...@...m)`。

### 7.2 qlog 数据分析（路试后）
cabana / PlotJuggler 打开 qlog，看三个消息：
- `sccXState`：state/source/confMap/confVisionA/confVisionB/vMap/mapReachDistance —— tsc-d 标定全靠它
- `mapdExtendedOut`：path（曲率点列）/ 匹配五态 / tileLoaded
- `longitudinalPlan`：vCruise 被帽压低的时段

### 7.3 灰度路试顺序（强烈建议）
1. **基线**：`SccXMapEnabled=False` 跑熟路一圈 —— 纯视觉 SCC-X 行为
2. **开图**：`=True` 同路线一圈 —— 对比 sccXState 里 map 源参与时段、减速起点早晚
3. **标定**：匝道/互通处观察 entering 是否过早/过晚，调 `MAP_ENTERING_REACH_MARGIN`（constants.py，默认 1.2）
4. 全程开 rlog；误减速事件按 §8 排查

---

## 8. 故障排查

| 现象 | 排查路径 |
|---|---|
| `verify_setup.py` 某项 MISS | 对应文件没贴到位，按提示补 |
| mapd 进程反复重启/不存在 | 二进制没放/架构错（file mapd 应显示 ELF aarch64）；cereal 未先编译；SConscript 缺失 |
| cloudlog 无任何 `scc-x:` 事件 | SccXEnabled 是否 true；纵向是否激活；车速是否 >20km/h；看 sccXState.enabled |
| 弯道完全不减速（sccXState 里 confMap=0） | mapd 瓦片下载了吗（mapd i 看 tileLoaded）；GPS 精度；waySelectionType 是否 fail/extended |
| 不该减速时减速 | 看 sccXState.source：=map 则查地图数据陈旧/possible 态（confMap=0.7 降级信任）；=vision_* 则与地图无关 |
| 减速时机太早/太晚 | 调 MAP_ENTERING_REACH_MARGIN（大→更早） |
| 编译报 capnp 错 | 见 §4.1 表 |

---

## 9. 回滚

```bash
git log --oneline -2          # 找到 baseline commit
git checkout <baseline> -- openpilot/ cereal/   # 或整分支丢弃：
git checkout master && git branch -D feature/mapd-sccx
```
设备上残留的 MapdSettings 等参数无害；如需清理：`python -c "from openpilot.common.params import Params; [Params().remove(k) for k in ('SccXMapEnabled','MapdSettings','MapPanelEnabled','MapOrientationMode')]"`

---

## 10. 沙箱验证记录与边界

已验证（2026-09-25 沙箱）：单测 8/8 实跑通过；全部 Python 文件 py_compile 通过；
30+ 锚点与 master(2026-09-23) verbatim 一致；capnp 62 个类型 ID 无冲突。
未验证（需本地）：capnp/scons 编译、进程集成、真机行为——首编报错或路试异常请把
`git diff --stat`、编译报错或 cloudlog 片段发来。
