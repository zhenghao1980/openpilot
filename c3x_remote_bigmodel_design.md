# C3X + NVIDIA 笔记本远程大模型辅助驾驶方案（设计文档 v2）

基于 openpilot master（zhenghao1980 fork，bb11d99）中 C4（comma four）与 chestnut 协处理板的联动机制改造。
分支：`c3x-remote-bigmodel`。本文档 v2 在已实现并验证的 v1（raw TCP 远程推理）基础上，定型
"自适应速率连续谱融合"目标架构。

## 1. 原厂 C4 + Chestnut 机制（调研结论）

chestnut 是 C4 通过 USB4/PCIe 隧道（ASMedia ASM2464 桥）连接的 AMD GPU 协处理板，跑大模型
（`big_driving_tinygrad.pkl`）；C4 本体跑小模型（`driving_tinygrad.pkl`）。关键代码：

| 机制 | 位置 | 说明 |
|---|---|---|
| 硬件探测 | `selfdrive/modeld/helpers.py:48` `chestnut_present()` | 扫 USB VID/PID + product 固件串匹配 |
| 模型就绪 | `helpers.py:59` `chestnut_compiled()` | big 模型编译产物 manifest 存在 |
| 双模型加载 | `selfdrive/modeld/modeld.py:282-306` | 后台线程加载 big（60s 超时）；期间小模型驱动；big 就绪后小模型仍驻留内存热备 |
| 状态参数 | Params `ChestnutLoading` / `ChestnutActive` | modeld ↔ selfdrived/UI 状态通信 |
| 运行时降级 | `modeld.py:430-440` | big `run()` 抛异常 → `ChestnutActive=False` → 切 small_model，本行程不再自动切回 |
| 输出校验 | `modeld.py:219-220` | big 输出非有限值即抛异常触发降级 |
| 健康遥测 | `modeld.py:75-162` `ChestnutState` | ~2Hz 发布 `chestnutState`（GPU 温度/功耗/占用、PCIe 链路、供电） |
| 告警 | `selfdrived/selfdrived.py:173-185,471-472` | bigModelLoading(NO_ENTRY) / bigModelFailed(SOFT_DISABLE+常驻提示) |
| 输入输出 | `modeld/constants.py` | 20Hz 推理（MODEL_RUN_FREQ=20），时序采样 5Hz（MODEL_CONTEXT_FREQ=5，frame_skip=4），hidden_state 512 维 |

**输出契约**：大模型返回的是"未来 10 秒场景剧本"而非执行器指令——33 个时间点的自车
位置/速度/加速度/姿态（plan）、车道线/路沿（4+2 条 × 33 点 + 概率）、前车轨迹与概率、
meta（脱手/急刹/刹车灯概率，FCW 用）、视觉里程计 pose、action 头（期望曲率/加速度），
外加 512 维 hidden_state 回喂自身。执行器指令（方向盘扭矩/油门/刹车）由 controls 层计算。
大小模型输出契约完全一致，下游（selfdrived/controls/UI）对"换脑子"无感。

**大小模型配合状态机**：启动探测 → 加载期小模型顶岗（NO_ENTRY）→ big 就绪切 big、小模型
转热备 → 单帧异常即永久切回小模型（本行程不自动恢复）→ 重启才重试 big。

## 2. v1：已实现并验证的基线（raw TCP 远程推理）

### 2.1 架构

```
┌─ C3X 车端 ────────────────────────────┐     ┌─ NVIDIA 笔记本 ─────────────────┐
│ camerad → VisionIPC → modeld(改造)     │ TCP │ remote_modeld_server.py          │
│   ├─ RemoteModelState ──8571───────────┼◄───►│   big 模型 (tinygrad CUDA)       │
│   └─ ModelState(小模型, 热备)          │ :8571│   单连接串行处理, hidden_state   │
│ card/controls ← modelV2 ← 解析在 C3X   │     │   断连重置时序状态(复刻chestnut)  │
│ panda → 车辆执行                        │     │                                  │
└────────────────────────────────────────┘     └──────────────────────────────────┘
```

控制环（modeld→modelV2→controls→panda）完全留在 C3X；笔记本只承担 big 模型前向推理；
解析（Parser/fill_model_msg）与消息契约全在 C3X。复用原厂 `ChestnutLoading`/`ChestnutActive`
参数与降级分支，selfdrived 告警、UI 徽标自动生效。

### 2.2 v1 协议（已实现，`remote_model.py` / `remote_modeld_server.py`）

长度前缀小端二进制分帧，单 TCP 连接一会话：

- `HELLO(cam_w, cam_h)` → `META{input_shapes, output_slices, vision_input_names}`（兼作可用性探测）
- `INFER{frame_id, [name, NV12 字节]×2, [name, 3×3 f32]×2, desire_pulse[8], traffic[2], action_t[2]}`
  → `RESP{raw f32 model_output 向量}`（回传仅几十 KB）
- 任何连接/超时/协议错误 → `RemoteModelError` → modeld 现成降级分支
- 环境变量：`REMOTE_MODEL_HOST` / `REMOTE_MODEL_PORT`(8571) / `REMOTE_MODEL_TIMEOUT_MS`(300)

### 2.3 v1 验证结果（全部实测）

| 验证 | 结果 |
|---|---|
| 协议单测（假服务器 8 用例） | 本机 + WSL 通过 |
| 数值一致性 parity（stock vs 远程链路，5 帧确定性序列，含 desire 脉冲） | 本机 CPU 小模型 + 笔记本 CUDA big 模型均通过（容差 1e-4/1e-6，big 版 10.3s OK） |
| 跨机冒烟（WSL 客户端 → 笔记本 RTX 4070 真推理） | SMOKE OK，22 输出全有限，big 输入 (1,12,128,256)+32 帧上下文确认 |
| PC 端全进程模拟（fake_camerad → 真 modeld → 笔记本 big） | `remote big model found` → modelV2 连续 big=True 出流 |
| 断链降级注入（杀服务器） | `Connection reset` → `big model failed, fall back to small` → big=False 无缝接续 |
| big 模型纯推理耗时（笔记本回环） | **~40ms/帧**（RTX 4070） |
| 手机热点吞吐/端到端 | 36Mbps / **~2.0s 每帧**（传输占 88%，推理仅 2%） |

### 2.4 v1 的硬伤（v2 要解决）

1. **丢帧门槛**：`selfdrived.py:471` `frameDropPerc > 1` 即 `modeldLagging`
   （NO_ENTRY + SOFT_DISABLE）。顺序式远程推理 ~110ms/帧对 20Hz 摄像头 ≈ 丢 55% 帧，
   **不允许挂挡**——big 模式能演示但不能上路。
2. **20Hz 全新鲜在裸传千兆不可行**：7.4MB/帧 × 20Hz ≈ 1.2Gbps > 千兆实速 ~110MB/s；
   上限 ~10-12Hz。
3. **协议同步阻塞**：一请求一等待，无法流水线，带宽与推理无法重叠。

## 3. v2 目标架构：自适应速率连续谱融合

### 3.1 核心思想

不做"非大即小"的台阶切换，而是把**请求帧率 r、融合权重 w、拼接点 τ** 都变成可观测、
可调度的连续运行时状态，形成从"20Hz 全大模型"到"纯小模型"之间的连续退化谱系：

- **快通道（小模型，C3X 本机，20Hz）**：低延迟、管近场反射；`pose`/cameraOdometry
  **永远只用小模型**（里程计最忌陈旧）。
- **慢通道（大模型，远端，r ∈ {0..20}Hz 自适应）**：异步流水线请求，管远场前瞻
  （前车 cut-in、并线意图、复杂场景）。
- **融合（modeld 内部，对外仍是一份 modelV2，下游无感）**：

| 输出 | 来源 | 规则 |
|---|---|---|
| pose / cameraOdometry | 小模型 | 恒定 |
| plan 轨迹 | 拼接 | 近场（t<τ）小模型；远场（t>τ）大模型；τ 前后 ~0.3-0.5s 过渡带按 `plan_stds` 加权混合，保证 C0/C1 连续 |
| lane lines / road edges | 按距离 | 近 ~20m 小模型（新鲜），远处大模型（强） |
| lead 前车 | 大模型为主 | 核心价值（cut-in 预判），按锚点时间平移 |
| meta / desire / 并线 | 大模型优先 | 意图判断是能力差距最大处；FCW 脱手保留小模型即时结果 |

- **大模型结果不怕迟到**：回传到时其超时近场（~RTT）被剪掉，t>RTT 的远场预测仍在未来，
  完全可用。密集请求（高 r）让远场每 50ms 刷新一次；降帧率时远场在两次更新间静态。

### 3.2 拼接点与权重

- τ = **实测 staleness**（协议时间戳动态测量，非固定值），网络抖动时 τ 与混合权重联动调整；
- 融合权重 w 是 staleness 与 plan_stds 的**连续函数**（示例：RTT=100ms→w≈0.95，
  300ms→w≈0.3），禁止硬阈值跳变；w 低通滤波 + 速率切换迟滞（governor 式），防控制抖动。

### 3.3 自适应速率控制律

```
r ≤ 可用带宽 × 安全系数 / 单帧字节数     (网络约束, 吞吐滑动平均实测)
r ≤ 1 / (远端推理时间 × (1+排队余量))      (算力约束, GPU 忙闲率反馈)
staleness ≤ τ_max (300ms)               (延迟约束, 超标先降 r 再降 w)
```

- 档位：20Hz（预 warp 后或更好链路）→ 10Hz（千兆裸传甜区）→ 5Hz（弱链路）→ 0（纯小模型）；
- **降帧率优先于降权重**（帧率降一档信息仍全量可信；权重降级是牺牲信息质量）；
- 锚点跳变：排队变深时丢弃积压旧锚点、以最新帧重新锚定（近场本就被剪，旧帧价值低）；
- 链路彻底断开 → 落 stock 纯小模型路径（现有 v1 降级分支，安全地板原样）；
- **原则：自适应逻辑只调性能旋钮，永不调安全地板；任何异常直接落 stock。**

### 3.4 时序状态语义

- 20Hz 全速请求时，hidden_state 逐帧链式传递——与官方 Chestnut 语义完全一致；
- 降帧率（10/5Hz）时，客户端在两次请求间照常推进本地输入队列，服务端按采样窗口
  （buf[::frame_skip]）取上下文——等价 5Hz 时序采样，与模型训练采样率
  （MODEL_CONTEXT_FREQ=5）对齐，两种模式均合法。

## 4. v2 协议规范（TCP + 二进制分帧，v1 向后兼容）

**传输层：TCP（点对点单连接）**。理由：hidden_state 链要求严格有序（TCP 天然保序）；
专线无竞争流量，队头阻塞几乎不触发；流控自带防打爆。否决项：UDP+自造可靠层
（安全路径不手写重传）、gRPC/HTTP2（50ms 周期下头开销与分配抖动不可接受）。

**应用层沿用 v1 分帧**（长度前缀小端），流水线只需客户端"连发 INFER、按序收 RESP"，
服务端维持单连接串行处理即天然保序。v2 增量：

1. `INFER` 增加 `client_send_ts`（锚点发送时间）→ 融合层精确计算 staleness；
2. `RESP` 增加 `server_recv_ts` / `server_done_ts` → RTT 分解（传输 vs 推理）；
3. 新增低频双向 `CTRL` 消息：客户端上报目标帧率/编码协商，服务端上报 GPU 忙闲、
   队列深度 → 喂自适应控制律；
4. `HELLO`/`META` 能力协商：payload 编码清单（raw_nv12 / fp16_warped / h264）。

**编码演进路线**：
- v2 现状：raw NV12（7.4MB/帧），千兆上限 ~10-12Hz；
- v2+ 优化：C3X 端预 warp+resize 后发送 fp16 张量（~1.5-3MB/帧），千兆可达 20Hz
  （传输 ~25ms + 推理 40ms 流水线重叠）；C3X 端 warp 图用 tinygrad/OpenCL 编译；
- v3 选项：C3X 硬件 HEVC 编码器（encoderd 已有）把帧压到 ~150KB——热点即可 20Hz，
  但引入有损 artifacts，必须过 parity 验证，不进 v1/v2 主线。
  （注：本设计早期版本曾设想 encoderd+HEVC+ZMQ 路径，v1 实现时评估编码延迟与
  复杂度后改用 raw NV12 over TCP，验证通过；HEVC 降级为 v3 备选。）

## 5. 网络硬件路径

- **上车（推荐）**：两头 USB3 转千兆网卡（RTL8153/AX88179，AGNOS 内核已内置
  `CONFIG_USB_RTL8152=y` / `CONFIG_USB_NET_AX88179_178A=y`，即插即用）+ 普通网线直连
  （Auto-MDI-X），静态 IP 同网段；Windows 防火墙入站放行 8571。
- 预期：有效吞吐 700-900Mbps → 每帧传输 70-90ms → 端到端 ~110-130ms（10Hz 档）。
- 调试期：手机热点可用（36Mbps，2s/帧）但仅供功能验证；gadget 单线方案
  （RNDIS/CDC-NCM）调研结论：AGNOS 未编 RNDIS function、aux 口设备树写死 host、
  USB2 带宽存疑，不推荐。
- 笔记本要求：WSL2 需镜像网络模式（mirrored）方可使 8571 端口从外部可达。

## 6. 验证走廊与测试方法

1. 验证矩阵：{5, 10, 20Hz} × {100, 200, 300ms 注入延迟} × {0/0.5/0.95 权重}；
2. 回放对比：同一 route 分别跑纯小模型 / 纯大模型 / 融合，对比轨迹偏差、
   曲率/加速度平滑度、FCW 触发一致性（parity 测试基建直接扩展）；
3. 注入测试：链路抖动、丢包、服务器 OOM、推理超时、TCP 半开——全部应平滑降档
   或落 stock，禁止异常穿越到控制层；
4. 门槛检查：所有工作点 `frameDropPerc ≈ 0`、`modeldLagging` 不触发。

## 7. 风险与限制

- C3X 负载：小模型从热备转全程在岗 + 可能的预 warp 图，需实测 GPU/CPU 余量；
- 融合引入官方没有的新失效模式（拼接/权重 bug 产生"看似合理的错误轨迹"），
  验证负担显著增加，未经验证的权重/速率组合宁可降档；
- 双模型系统性偏差：大小模型预测不一致时混合权重决定偏向，需实测标定而非拍脑袋；
- 合规与安全：openpilot 服务依赖、 longitudinal 限制及当地法规自行评估；
  笔记本车内固定/供电/散热；安全地板（stock 小模型闭环）始终在位。
