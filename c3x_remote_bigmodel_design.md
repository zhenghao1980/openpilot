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

### 3.5 UI 呈现与远端链路标识

**动机**：官方 UI 五态效果（绿栗子=大模型工作中/呼吸=加载/橙栗子=失败 + Big Model
Loading/Failed 弹窗）由 `deviceState.chestnutPresent`（USB 探测）驱动；网络方案该信号
恒 False → 不改 UI 则远端大模型激活时屏幕毫无显示。故 UI 需要一处受控扩展
（§7.2 纪律记录在案的所有者请求例外），且真 Chestnut 插拔时官方路径必须一行不动。

**设计**（真 Chestnut 优先，网络模式仅为无 USB Chestnut 时的平行来源）：

1. `ui_state.py`（~10 行守卫）：`chestnutPresent=True` → 走原逻辑；
   `False 且 RemoteModelPresent=True` → 同一套五态机 + `remote` 标志，效果与官方
   完全一致（含弹窗文案 "small model is still available"）；
2. 徽标（`hud_renderer.py`，~15 行）：栗子图标旁绘制链路标识——wifi 三级弧 /
   有线 RJ45 方块（raylib 过程化绘制，不引入图片资产）；**颜色编码链路质量**
   （绿 RTT<150ms / 橙 150-300ms / 红 >300ms 或降档中）；行车 HUD 为主展示位，
   家庭屏底部图标栏加同款小徽标；
3. 参数契约（远端客户端写、UI 轮询，与 ChestnutActive 同款机制）：
   `RemoteModelPresent`(bool，会话存活) + `RemoteModelStats`(1-2Hz JSON：
   `{iface, rtt_ms, rate_hz, w}`)——徽标与颜色的数据源；
4. v2 遗留决策：融合模式下 `modelV2.big` 语义（会话健康即报 True 还是按权重
   阈值），实车阶段再定，不涉及 UI 改动。

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

## 4. v2 协议规范：UDP 数据报 + 自校验分帧 + 心跳（数据面无重传）

三条设计原则（优先级高于一切实现细节）：

1. **最直接发送**：数据帧发完即忘；每个数据报自带完整性与正确性校验码（CRC32C，
   硬件指令加速，成本可忽略），接收端校验失败或分片不完整 → **整帧丢弃，此帧不再使用**；
2. **不重发**：接收端对坏帧不返回任何错误，发送端因此不存在重发逻辑——丢帧的代价
   由上层以"该锚点缺失、staleness 增大"自然吸收，这正是融合架构的设计语义；
3. **心跳**：双向 500ms 一个 BEAT（序号 + 发送时间戳 + 服务端 GPU 忙闲/排队深度/丢帧
   统计），连续丢 3 个判链路死亡 → 立即 `ChestnutActive=False` 落小模型（复刻官方
   `chestnutState`/`chestnutPresent` 语义），不等推理超时。心跳统计同时喂自适应控制律。

### 4.1 传输层：UDP（点对点数据报）

v1 曾选 TCP 求简单，v2 数据面改 UDP 的理由：数据报天然原子（TCP 分帧错位会级联污染
后续所有报文，UDP 坏一个报文只损失一个）；坏帧丢弃语义与"无重传"原则下 TCP 的
自动重传反而是队头停顿的负资产；hidden_state 链按**到达序**推进，模型时序上下文
本来就容忍跳帧（stock 掉帧同理），专线点对点下乱序极少。

### 4.2 报文与分片

- 单 UDP socket，六类消息：`BEACON`（服务发现信标，见 4.3）、`HELLO`/`META`
  （握手与能力协商）、`INFER`（上行推理请求）、
  `RESP`（下行结果，按 frame_seq 对号）、`BEAT`（心跳）、`CTRL`（低频控制）；
- 大报文（NV12 帧 ~7.4MB）应用层分片：每片带 `frame_seq + chunk_idx + n_chunks +
  payload + CRC32C`，推荐片大小 16-64KB（平衡系统调用数与 IP 分片风险）；任一缺失
  或 CRC 失败 → 丢弃整帧并计丢弃统计；
- 整帧重组后再做一次整帧 CRC 复核；
- INFER 携带 `client_send_ts` 锚点时间戳（算 staleness），RESP 携带
  `server_recv_ts/server_done_ts`（RTT 分解：传输 vs 推理）。

### 4.3 服务发现：UDP 广播信标（BEACON）

设计目标：C3X 零配置发现笔记本——免手工指定 `REMOTE_MODEL_HOST`，即插即用。

**服务端生命周期**：

```
自检通过(模型可加载/GPU可用/端口可绑定) → READY → 周期 5s 广播 BEACON
   ↓ 收到 C3X HELLO，握手+能力协商成功，会话建立
停止广播（宣告"本节点已被占用"）
   ↓ 会话结束（双向心跳丢失 N 个周期 / 发送失败 / 对端关闭）
重新自检 → 通过则回到 READY 恢复广播；不通过则不广播（不发布虚假可用性）
```

细节约定：

- **BEACON 载荷**：魔数 + 协议版本 + 服务端口 + 能力标志（支持的编码）+
  实例 ID + 发送时间戳；C3X 按魔数与版本过滤，网段内多台服务可区分；
- **突发信标**：刚进入 READY 状态时连发 3~5 个（间隔 100ms），把冷启动发现
  延迟从最长 5s 降到 <100ms，之后转入 5s 周期；
- **恢复广播的触发**依赖双向心跳：服务端心跳丢失判定与 C3X 端对称，避免
  会话僵尸导致信标永久沉默；
- **网段边界**：UDP 广播不跨网段，且手机热点普遍开 AP 隔离（广播/多播被拦）——
  有线直连（两端 USB 网卡同网段）下本机制完美工作；热点调试时 C3X 降级为
  `REMOTE_MODEL_HOST` 手动指定 + 优先重试上次成功连接的地址；
- Windows 防火墙需放行信标端口（可与 8571 同端口复用）。

### 4.4 唯一例外：控制消息带确认

`HELLO`/`META`/`CTRL` 是会话级消息（整个行程数次），不参与"不重发"原则：
接收方须回 ACK，发送方超时指数退避重试（上限数次，仍失败则判链路异常）。
数据帧（INFER/RESP/BEAT）依旧零确认、零重发。控制消息体量小（百字节级），
不占用数据面带宽。

### 4.5 编码演进（协商于 HELLO/META）

- v2 现状：raw NV12（7.4MB/帧），千兆上限 ~10-12Hz；
- v2+ 优化：C3X 端预 warp+resize 后发送 fp16 张量（~1.5-3MB/帧），千兆可达 20Hz；
- v3 选项：C3X 硬件 HEVC 编码（~150KB/帧，热点可用），有损、必须 parity 验证，
  不进主线。（设计早期曾设想 encoderd+HEVC+ZMQ，v1 实现时改用 raw NV12 over TCP
  并验证通过；HEVC 保留为 v3 备选。）

### 4.6 服务端构成（remote_modeld_server，笔记本常驻服务）

单一服务程序承担：握手/能力协商（+ACK）、分片重组与 CRC 校验、心跳应答
（500ms，捎带 GPU 忙闲率/推理队列深度/累计丢帧数——即自适应控制律的反馈源）、
CTRL 处理、推理循环（按到达序消费、hidden_state 链推进、结果分片回传）。

进程内结构为"I/O 线程 + 单推理工作线程 + 有界队列（深度 2~3）"：

- **推理天然单线程**：hidden_state 逐帧链式传递决定 GPU 推理必须严格串行，
  这不是性能妥协而是模型语义；
- **I/O 线程永不被 GPU 阻塞**：重组/校验/心跳/收发全在 I/O 层，模型卡死不影响
  链路状态如实上报；
- **背压 = 丢最旧**：队列满时丢弃最陈旧的待推理锚点（与 C3X 端锚点跳变同构），
  宁可少算旧帧不让排队延迟滚雪球；
- **异常隔离**：推理线程故障不得静默拖死 I/O 线程；进程级崩溃由 supervision
  重启（进程一死心跳即停，C3X 端 ~1.5s 内判链路死亡落小模型）。

服务化形态：开机自启（WSL2 systemd 或 Windows 计划任务）、崩溃自恢复、日志落盘，
实现"笔记本开机 → big 模型可用"无感。v1 的 `remote_modeld_server.py` 即此角色的
同步 TCP 版，已验证，v2 在其上演进。

## 5. 网络硬件路径

- **上车（推荐）**：两头 USB3 转千兆网卡（RTL8153/AX88179，AGNOS 内核已内置
  `CONFIG_USB_RTL8152=y` / `CONFIG_USB_NET_AX88179_178A=y`，即插即用）+ 普通网线直连
  （Auto-MDI-X），静态 IP 同网段；Windows 防火墙入站放行 8571（UDP 数据面 +
  TCP/UDP 控制面，按实现放行）。
- 预期：有效吞吐 700-900Mbps → 每帧传输 70-90ms → 端到端 ~110-130ms（10Hz 档）。
- 调试期：手机热点可用（36Mbps，2s/帧）但仅供功能验证；gadget 单线方案
  （RNDIS/CDC-NCM）调研结论：AGNOS 未编 RNDIS function、aux 口设备树写死 host、
  USB2 带宽存疑，不推荐。
- 笔记本要求：WSL2 需镜像网络模式（mirrored）方可使 8571 端口从外部可达。

## 6. 验证走廊与测试方法

1. 验证矩阵：{5, 10, 20Hz} × {100, 200, 300ms 注入延迟} × {0/0.5/0.95 权重}；
2. 回放对比：同一 route 分别跑纯小模型 / 纯大模型 / 融合，对比轨迹偏差、
   曲率/加速度平滑度、FCW 触发一致性（parity 测试基建直接扩展）；
3. 注入测试：链路抖动、丢包、报文损坏（CRC 失败）、服务器 OOM、推理超时、
   心跳中断、UDP 端口不可达——全部应平滑降档或落 stock，禁止异常穿越到控制层；
4. 门槛检查：所有工作点 `frameDropPerc ≈ 0`、`modeldLagging` 不触发。

## 7. 改造纪律与官方兼容性

**原则：不影响官方 C4+Chestnut 功能；新功能全部落在新增文件；官方文件改动仅限
不可绕过的最小锚点；官方成熟机制优先复用而非重写。**

### 7.1 当前侵入面审计（v1，实测）

整个分支对官方代码的改动**只有一个文件 17 行**：`selfdrive/modeld/modeld.py`
（1 行 import + 3 处锚点共 16 行）；其余 8 个文件全部为新增（remote_model.py、
remote_modeld_server.py、测试与工具）。锚点守卫分析：

| 锚点 | 守卫 | 官方路径行为 |
|---|---|---|
| 远端探测 `REMOTE_META = None if CHESTNUT else remote_model_metadata(...)` | `CHESTNUT` 为真即短路 | C4+Chestnut 用户：完全不走网络代码 |
| 远端加载 `elif REMOTE_META is not None:` | 无 env 配置时 `remote_model_metadata` 立即返回 None |  stock 小模型路径逐字节一致 |
| 小模型热备加载条件加 `or REMOTE_META is not None` | 同上 | 不影响 |

`REMOTE_MODEL_HOST` 未设置时，三个锚点均为严格 no-op——C4+Chestnut 用户和纯
stock 用户使用本分支与上游 master 行为一致。

### 7.2 v2 开发遵守的纪律

1. **新增文件承载新功能**：协议栈、融合层、自适应控制律全部放新模块
   （`selfdrive/modeld/remote_model.py` 及其子模块、`tools/remoted/`），
   官方文件只加守卫过的锚点；
2. **锚点最小化且可回归**：每个锚点必须是无配置下的严格 no-op，守卫条件写进
   commit message；`modeld.py` 的 diff 目标是始终 <20 行，便于 rebase 上游；
3. **复用清单**（复制官方设计而非重写）：`ChestnutLoading`/`ChestnutActive`
   参数语义、`chestnutState` 健康上报、stock 降级分支（modeld.py 现成
   try/except）、Parser/fill_model_msg 输出契约、VisionIpc 帧通路、
   `modeldLagging` 门槛自身不动（用 20Hz 复用发布满足它，而不是修改门槛）；
4. **禁止事项**：不改 `selfdrived` 安全逻辑、不改 modelV2 消息契约、不降
   frameDropPerc 门槛、不在官方类上打猴子补丁；UI 仅允许 §3.5 记录在案的
   一处受控扩展（远端链路标识），真 Chestnut 路径守卫不动；
5. **官方路径回归**：每次发布前跑 stock 配置（无环境变量） smoke——
   fake_camerad + modeld 应表现为纯小模型、无网络探测日志外的任何差异。

### 7.3 v2 侵入面预测（接口同构原则）

**预测结论：v2 对官方代码的净侵入为零新增，`modeld.py` 维持 v1 的 17 行守卫锚点不变。**
关键在于接口同构：`FusionModelState`（新文件）对外暴露与 `ModelState` 完全相同的接口
（`.run()` / `.vision_input_names` / `.chestnut` / `.warmup()`），融合、异步请求、
自适应帧率、UDP 收发全部发生在对象内部；主循环对模型的依赖只有
`model_output = model.run(...)` 一行，天然无感。拼接融合把平移后的大模型轨迹
**重采样回标准 T_IDXS 网格**再返回，`fill_model_msg` 无需知道融合发生过。
v2 仅有的官方接触点是复用而非改动：v1 构造锚点（换构造类名）、stock 降级分支
（网络死亡时 `run()` 主动抛异常走入）、`ChestnutActive` 参数（心跳超时由融合对象
的 I/O 线程直接写 Params，KV 存储并发安全，selfdrived 无感）。

## 8. 风险与限制

- C3X 负载：小模型从热备转全程在岗 + 可能的预 warp 图，需实测 GPU/CPU 余量；
- 融合引入官方没有的新失效模式（拼接/权重 bug 产生"看似合理的错误轨迹"），
  验证负担显著增加，未经验证的权重/速率组合宁可降档；
- 双模型系统性偏差：大小模型预测不一致时混合权重决定偏向，需实测标定而非拍脑袋；
- 合规与安全：openpilot 服务依赖、 longitudinal 限制及当地法规自行评估；
  笔记本车内固定/供电/散热；安全地板（stock 小模型闭环）始终在位。
