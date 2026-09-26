# WSL 联调指南（无设备跑通地图面板全链路）

## 0. 前提自检
- WSL2 + WSLg：`glxinfo | grep renderer` 有输出（Mesa d3d12 或 llvmpipe 均可）
- 仓库已在 WSL 构建通过：`uv sync` + `scons -j$(nproc)`（PC 模式）
- Go：`sudo apt install golang-go`（跑 mapd）
- Docker：瓦片制备用（见 README-GUIDE §4.6，可选用小区域 mbtiles 加速）

## 1. 准备地图数据（二选一）
- **从设备拷**：设备上已用 mapd 终端下载过数据 → scp 到 WSL 对应路径
- **本机制备**：tools/map_tiles 流水线，bbox 选模拟路线附近小区域（北京城区 z14-15 即可，
  几分钟出数），glyphs 用 fonts.openmaptiles.org 下载解压到 /data/mapd_render/glyphs

## 2. 构建三个原生组件（WSL x64 原生，无需交叉编译）
```bash
# mapd（Go）
cd ~/mapd-upstream && go build -o mapd . && cp mapd <repo>/selfdrive/mapd/mapd
# maprenderd（cmake + vendored maplibre-native，x64 即 CI 平台）
cd <repo>/selfdrive/maprender
git submodule add https://github.com/maplibre/maplibre-native vendor/maplibre-native
cmake -B build -DMAPLIBRE_DIR=$PWD/vendor/maplibre-native
cmake --build build -j$(nproc)   # 首次 30-60 分钟
```
数据目录：`sudo mkdir -p /data/mapd_render && sudo chown $USER /data/mapd_render`
放入 style.json / china.mbtiles(或小区域) / glyphs/

## 3. 参数预设
```bash
python -c "from openpilot.common.params import Params; p=Params(); p.put_bool('MapPanelEnabled',True); p.put('MapPanelMode','1'); p.put_bool('SccXEnabled',True); p.put_bool('SccXMapEnabled',True)"
```

## 4. 启动
```bash
bash tools/sim/run_ui_sim.sh     # tmux 四窗格：mapd / maprenderd / sim_gps / ui
```
预期：UI 窗口出现（无摄像头画面、黑底），打开开发者面板开关后地图面板显示：
瓦片底图 + 沿 S 弯移动的蓝色路线 + breadcrumb + 箭头。

## 5. 排错
| 现象 | 排查 |
|---|---|
| UI 启动即退 | 看 /tmp/ui.log；多为缺 sm 服务数据 → sim_gps 覆盖面不够时，换 watch3.py 回放真实 route（最省事的真实数据法） |
| 面板黑、无瓦片 | /tmp/maprenderd.log 是否有帧输出；mapRenderCam/Frame 是否注册（fix_log_capnp.py 跑过没） |
| 瓦片拼错位 | v5 对齐参数（CENTER_FRAC/zoom 公式）发我截图调 |
| EGL/GLX 报错 | WSLg 下 mbgl linux 头枕后端用 GLX（XWayland）；失败则试 EGL 变体（ADAPT 标记处） |
| mapd 无匹配 | 瓦片数据区域与 sim 起点（39.983,116.307）不一致 → 改 sim_gps.py 起点或重导瓦片 |

## 6. 与真机的差异（WSL 验证不能替代的）
- 真机 GLES 驱动（AGNOS）与 WSL Mesa 行为差异——maprenderd 的 EGL 适配最终要上 C3X 验
- 性能数字（WSL x64 不代表 C3X aarch64 的 CPU 余量）
- CAN/实际控制链路——WSL 只到 planner 日志/cloudlog 层
