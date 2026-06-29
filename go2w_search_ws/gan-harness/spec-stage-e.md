# Product Specification: Go2W 阶段E — 房间级搜索编排（Room-level Search Orchestration）

> Generated from brief: "为 Go2W 阶段E：房间级搜索编排产出完整实现规格（GAN Planner 角色）"
> 主管角色: GAN Planner（架构/规格，不含代码实现）
> 状态: 待 Generator 实现待 Critic 审查
> 范围: 纯软件先落地（硬件在装），用 **mock Nav2 action + mock 检测** 验证编排状态机端到端
> 前置: 阶段A（web 上移 NX，已 gan 收敛）+ 阶段B（AI 上移 NX，已 gan 收敛）

---

## 0. 规格阅读约定

- 所有路径均为**相对仓库根** `go2w_search_ws/`（Generator 实现时拼绝对路径）。
- 每个文件给三段：**职责 / 关键签名 / 实现要点**。签名是契约，实现要点是约束。
- **阶段A/B 红线继续生效**：
  - 禁止改 `web/static/panel.html` / `web/static/map.js` 的 JS 业务逻辑与现有 WS 消息字段名
  - 禁止改 `nx_motion_node.py` / `nx_sensor_node.py` 的现有逻辑
  - 禁止改 `ai/detector.py` / `ai/vlm.py`（平台无关，NX 适配在 nx_ai_node.py）
  - 禁止把 VLM 常驻显存（阶段B 空闲 unload 机制继续生效）
- **阶段E 新红线**：
  - **Nav2 action client 只用标准接口** `nav2_msgs/action/NavigateToPose`，禁止自己发 `/cmd_vel` 做航点导航（与阶段A 的手柄式 `/cmd_vel` 控狗是两条独立链路，阶段E 编排走 Nav2 action）
  - **房间地图用静态 YAML**（不接语义建图自动识别，那是未来工作）
  - **阶段D Nav2 未就绪时，必须能用 mock Nav2 action 跑通编排状态机**（不依赖狗/SLAM）
  - 编排**不得阻塞** nx_web_server 的 HTTP/WS/broadcast 主循环（编排跑在 TaskManager 的 worker 线程内，Nav2 action 用 future/callback 异步）

---

## 1. Vision（阶段E 目标态）

用户在 PC 浏览器（或语音转文本经 `/api/command`）说 **"搜索客厅"**，载荷 NX 上的 web 服务编排一条完整的房间级搜索任务：

1. **房间选择**：从静态 YAML 房间地图（`config/rooms.yaml`）里匹配出"客厅"的导航入口 pose + 房间内搜索区域边界；
2. **房间导航**：通过 `nav2_msgs/action/NavigateToPose` action client 把狗导航到客厅入口 pose，等 Nav2 反馈到达（ARRIVED）；
3. **房间内覆盖搜索**：到达后用 `planner.plan_lawnmower` 把房间搜索区域切成航点序列，逐个发 Nav2 goal 依次到达，每个航点停留触发阶段B 的 YOLO 检测；
4. **检测+报告**：检测到的目标记录"发现位姿 + 类别 + 置信度 + 时间戳"，实时经 WS 推前端（`type=search` 增量推送），全部航点走完后生成 MissionReport（含目标清单 + 路径统计）。

一句话验收：**在 NX 上启动 nx_web_server + mock Nav2 action server + mock 检测注入，PC 浏览器输入"搜索客厅"，能看到任务队列出现 `search_room` 任务、WS 推 `type=search_room` 状态机进度（SELECT_ROOM → NAVIGATE → ARRIVED → SEARCH → DETECT → REPORT）、最终收到 `type=mission_report` 报告（含发现的目标）；整个过程不依赖真 Nav2 / 真 SLAM / 真狗硬件，编排状态机端到端可验证。**

---

## 2. 阶段A/B → 阶段E 链路对比（Generator 必读）

### 阶段A/B（已交付，矩形覆盖搜索，无房间概念）
```
浏览器 ──HTTP/WS──> nx_web_server.py(NX)
                      ├─ TaskManager._execute_search(task.type="search_area")
                      │    └─ plan_lawnmower(width, height, spacing)  ← 矩形覆盖
                      │    └─ _wp_to_moves(wp) → 一串 move 任务
                      │    └─ worker 线程逐个 robot.move(vx,vy,vyaw) + sleep(duration)
                      │         (调 /cmd_vel 直发, 不走 Nav2; 无"先导航到房间"概念)
                      └─ 阶段B: 搜索中 vx!=0 时 robot.get_frame() → detector.detect → ws type=search
```
痛点：搜索是"原地开割草机"，狗不知道自己在哪个房间；用户说"搜索客厅"只能 fallback 成"搜索 10×10m 矩形"；没有 Nav2 集成，狗不会先走到客厅再搜。

### 阶段E（本次实现，房间级编排 + Nav2 action）
```
浏览器 ──HTTP/WS──> nx_web_server.py(NX)
                      ├─ TaskManager._execute_search_room(task.type="search_room")  ← 新任务类型
                      │    1. RoomMap.load("config/rooms.yaml").find("客厅") → Room(nav_pose, search_area)
                      │    2. Nav2Client.send_goal(room.nav_pose) → 等 ARRIVED  ← nav2_msgs/NavigateToPose action
                      │    3. plan_lawnmower(room.search_area) → 航点序列
                      │    4. 逐航点 Nav2Client.send_goal(wp) → 等 ARRIVED → 阶段B 检测 → 记录
                      │    5. 全部走完 → 生成 MissionReport → ws type=mission_report
                      └─ Nav2Client: rclpy ActionClient(self, NavigateToPose, '/navigate_to_pose')
                                       阶段D 未就绪时由 mock_nav2_action.py 提供 fake server (立即 ARRIVED)
```
收益：用户从"搜矩形"升级到"搜房间"；狗会先 Nav2 导航到房间入口再开搜；编排状态机清晰（六态）；Nav2 用标准 action 接口，与阶段D 配置完全解耦（mock 可替真）。

---

## 3. 关键设计决策（已拍板，给推荐 + 理由）

### 决策 1：Nav2 集成方式 → **推荐 (a) rclpy action client（`nav2_msgs/action/NavigateToPose`），禁止自己发 `/cmd_vel` 做航点导航**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **(a) Nav2 NavigateToPose action client** | **标准 ROS2 接口**；与阶段D Nav2 配置完全解耦（编排只管发 goal，Nav2 怎么规划/避障是它的事）；feedback 给导航进度（剩余距离/估时）；action 自带 cancel/状态机；mock 容易（fake action server 立即 succeed）；与休眠包 `orchestrator_node.py:161-188` 已写的 `_send_nav_goal` 完全一致（参考实现） | 需要等 action server，异步回调/future 模型要在 worker 线程里正确处理（不能阻塞 ROS2 spin 线程） | ✅ **推荐** |
| (b) 自己发 `/cmd_vel` 做航点导航 | 不依赖 Nav2 | **重造轮子**：要自己做路径规划+避障+到位判断；与 nx_motion_node 的 `/cmd_vel` 消费逻辑冲突（阶段A 的手柄控狗 vs 阶段E 的自主导航会抢 `/cmd_vel`）；无标准 feedback；阶段D Nav2 一上线就得推倒重来 | ❌ 不选 |
| (c) 调 `nav2_msgs/NavigateThroughPoses` 一次发整条航点链 | 一次 action 调用走完所有航点 | 无法在每个航点停留触发 YOLO 检测（NavigateThroughPoses 是连续穿过，不在中间停）；丢失"每航点检测一次"的语义；feedback 粒度粗 | ⚠️ 不选，丢失检测语义 |

**理由总结**：(a) 是"用标准接口、与 Nav2 解耦"的唯一正解。休眠包 `orchestrator_node.py:161-216` 已经把 `_send_nav_goal` / `_nav_goal_response_cb` / `_nav_result_cb` 写好了，Generator 直接照搬这套 future/callback 模式，但**关键差异**：休眠包是在 `rclpy.spin` 主循环的 `_tick` 定时器里发 goal，阶段E 是在 **TaskManager worker 线程**里发 goal——worker 线程不能 spin，必须用 `rclpy.spin_until_complete(node, future)` 在 worker 线程内同步等 goal 接受 + 等结果（rclpy 官方文档明确：action client 的 future 可在任意线程 spin_until_complete，只要保证只有一个 spin 在跑——nx_web_server 的主 spin 在线程1，worker 线程用 `spin_until_complete` 临时驱动该 future 的回调，互不冲突，见决策 4）。

### 决策 2：房间数据来源 → **推荐 静态 YAML 房间地图（`config/rooms.yaml`）**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **(a) 静态 YAML** | 最简、可手编辑、不依赖建图；语义建图未做前唯一可行；可版本控制；部署时 scp 到 NX | 房间 pose 要人工标定（拿狗在建好的地图上站到客厅入口，记录 `/odom` 或 map 坐标） | ✅ **推荐**（阶段E） |
| (b) 前端绘制（用户在地图上画房间多边形 + 入口点） | 灵活、无需标定 | 前端要改 panel.html/map.js（违反阶段A 红线）；地图坐标系对齐复杂；用户操作负担重 | ❌ 不选，违反红线 |
| (c) 语义建图自动识别房间 | 终极方案，全自动 | 需要专门的语义分割+房间分割算法（如 roomseg/era），是未来阶段F+ 的工作；阶段E 不做 | ❌ 不选，超出范围 |

**理由总结**：(a) 是"阶段E 最小可行"。YAML schema 见 §6。每个房间只需 3 个字段：`name`（中文名 + 别名）、`nav_pose`（入口导航目标 x/y/yaw，map 坐标系）、`search_area`（房间内搜索矩形 width/height/origin_x/origin_y，相对 map 原点）。人工标定流程写进 `docs/room_calibration.md`（运维 SOP，不在本阶段代码里）：拿狗在建好的 Nav2 地图上站到客厅入口，`ros2 topic echo /odom` 记录 x/y/yaw，写进 YAML。

### 决策 3：编排放哪 → **推荐 (b) 集成进 `nx_web_server.py` 的 TaskManager（新增 `search_room` 任务类型 + `RoomSearchOrchestrator` 协作类）**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 复活 `src/go2w_orchestrator` 包做独立 ROS2 节点 | 符合 ROS2 包规范；与休眠包设计一致 | **跨进程通信爆炸**：编排要调阶段B 的 detector/VLM（同进程内存共享 vs ROS2 srv）、要经 WS 推前端（要再发明一套 nx_web ↔ orchestrator 的 IPC）、要读 nx_web 的订阅缓存（dog_state/imu）；orchestrator_node 顶层 import 了 `vlm_integration` / `voice_pipeline` 这俩**根本不存在的模块**（休眠包没真正跑过），复活=重写；与阶段A 决策 1"不复活 go2w_web"同款理由 | ❌ 不选，跨进程耦合太重 |
| **(b) 集成进 `nx_web_server.py` 的 TaskManager（加 `search_room` 任务类型 + `RoomSearchOrchestrator` 协作类）** | **零跨进程**：直接复用阶段A/B 的 robot bridge / detector / vlm / ws_broadcast；TaskManager worker 线程天然是编排执行位置；HTTP `/api/search_room` 直接入队；契约清晰（新增 task.type=search_room + 新增 ws type=search_room/mission_report，不改现有字段） | TaskManager 文件变长（但 RoomSearchOrchestrator 拆到新文件 `web/nx_room_orchestrator.py`，TaskManager 只加一个 `_execute_search_room` 分支调用它，文件不臃肿）；TaskManager 要持有 Nav2Client 句柄（注入） | ✅ **推荐** |
| (c) 新建独立 `nx_search_node.py`（独立 rclpy 节点进程） | 进程隔离，搜索崩了不影响 web | 同 (a) 的跨进程通信爆炸；还要解决"web 进程的 Nav2Client vs search 进程的 Nav2Client 谁发 goal"冲突；ChannelFactory 单例多进程问题 | ❌ 不选，过度工程 |

**理由总结**：(b) 是"组件注入而非进程拆分"——与阶段B 决策 1（NxAiEngine 注入 nx_web）完全同款哲学。新增一个 `web/nx_room_orchestrator.py` 文件装 `RoomSearchOrchestrator` 类（封装 Nav2 action client + 房间地图加载 + 状态机驱动 + MissionReport 生成），TaskManager 在 worker 线程遇到 `task.type == "search_room"` 时实例化（或复用注入的实例）调用其 `run(task)` 方法。HTTP `/api/search_room` 入口直接 `task_mgr.add_list([Task("search_room", {...})])`。

**协作方式（与阶段A/B 的接口）**：
- **阶段A**：`TaskManager`（worker 线程）+ `NxRobotBridge`（不直接用，房间搜索走 Nav2 不走 `/cmd_vel`）+ `ws_broadcast`（推 type=search_room/mission_report）+ `NxWebNode`（提供 rclpy 节点句柄给 Nav2Client 挂载）
- **阶段B**：`NxAiEngine._latest_dets` / `get_detections_world()`（搜索中读最新检测）+ `NxAiDetectorProxy`（可选，直接 detect 一帧）；阶段B 的检测是**持续后台跑**的（_video_yolo_loop 线程），房间搜索**不需要主动调 detect**，只需在每个航点 ARRIVED 后读 `_latest_dets` 快照即可（见 §10 实现要点）

### 决策 4：Nav2 action client 的线程模型 → **推荐 "TaskManager worker 线程内 spin_until_complete 同步等 goal，主 spin 线程不干涉"**

**问题**：rclpy action client 的 `send_goal_async` 返回 future，goal_handle 的 `get_result_async` 也返回 future。这两个 future 的回调要靠某个 `spin` 驱动。nx_web_server 的主 `rclpy.spin(node)` 在线程1 跑（阶段A 决策 2），它**会驱动**这些 future 的回调（因为都挂在同一个 node 的 wait_set 上）。所以有两种等结果方式：

| 方式 | 描述 | 优缺点 |
|---|---|---|
| (i) 主 spin 驱动 + worker 线程 `future.result()` 阻塞等 | worker 线程发完 goal 后 `rclpy.spin_until_complete(node, future)` **不能**用（会和主 spin 抢 wait_set），只能 `future.result()` 死等——但 future 的回调要靠主 spin 驱动，主 spin 在跑所以会推进 | 简单；但 `future.result()` 无超时会死锁（goal 永不到达则 worker 永远卡住）；要加 timeout 自己包一层 |
| **(ii) worker 线程 `spin_until_complete(node, future)` + 超时兜底** | worker 线程发 goal 后，在本线程内 `rclpy.spin_until_complete(node, goal_future, timeout_sec=N)` 等接受，再 `spin_until_complete(node, result_future, timeout_sec=M)` 等到达 | **rclpy 官方推荐**（action client 教程标准模式）；与主 spin **短暂并发**但 rclpy 内部有 wait_set 协调（spin_until_complete 退出后主 spin 接管）；timeout 自然支持 |

**推荐 (ii)**。**关键约束**：
1. **每次 spin_until_complete 的 timeout 必须设**（Nav2 goal 接受 5s，导航完成 120s——与休眠包 `orchestrator_node.py:56` 的 `nav_goal_timeout=120.0` 一致），避免 worker 线程永久卡死。
2. **worker 线程 spin_until_complete 期间，主 spin 仍在跑**——这会导致同一个 node 被 spin 两次（主线程 + worker 线程）。rclpy Humble 对此的处理是：两个 spin 用各自的 wait_set，回调可能在任一线程执行。**这要求所有回调（Nav2 的 `_nav_goal_response_cb` 等）必须线程安全**——RoomSearchOrchestrator 内部用 `threading.Lock` 保护状态。
3. **不要用 MultiThreadedExecutor**（阶段A 决策 2 继续生效）。spin_until_complete 在 worker 线程跑就够，不需要多线程 executor。
4. **goal_handle 拿到后存实例属性**（带 Lock），cancel 时能调 `handle.cancel_goal_async()`。

> ⚠️ **与休眠包 orchestrator_node.py 的差异**：休眠包是在 `_tick` 定时器（主 spin 内）发 goal，用 `future.add_done_callback` 异步等（回调在主 spin 线程跑），靠 `_nav_active` 标志位防止重入。阶段E 是在 worker 线程**同步等**（spin_until_complete），因为 TaskManager 的 worker 是"取一个任务执行到完成再取下一个"的串行模型，同步等更直观。Generator 不要照抄休眠包的 callback 模式——那套是为定时器主循环设计的，worker 线程用同步等更合适。

### 决策 5：Nav2 不可用时的降级 → **推荐 mock Nav2 action server（fake NavigateToPose，立即 ARRIVED）**

| 方案 | 描述 | 优缺点 |
|---|---|---|
| (i) Nav2 action client 探测 server 不在 → skip 任务 | 探测 `wait_for_server(timeout=2s)` 失败就 `task.status=failed` | 阶段D 没好时整个编排跑不起来，无法 mock 验证 |
| **(ii) 提供 mock_nav2_action.py（fake NavigateToPose action server，立即 succeed）** | 独立 rclpy 节点，实现 `/navigate_to_pose` action server，收到 goal 立即 feedback 几次 + result status=SUCCEEDED | **编排状态机可端到端 mock 验证**（选房间→导航→立即到→搜索→每航点立即到→检测→报告）；阶段D Nav2 一上线，kill mock 节点即可，编排零改动 |
| (iii) 编排内部 short-circuit（Nav2 不可用时直接跳过导航步） | 不发 goal，假装已到达 | 丢失了"Nav2 action client 标准接口"的验证价值；阶段D 上线时编排代码路径变了，可能出新 bug |

**推荐 (ii)**。**关键约束**：
1. **mock server 是独立 rclpy 节点**（`web/mock_nav2_action.py`），与 nx_web_server 进程隔离，避免 action server/client 同进程的某些 rclpy 边界 case。
2. **mock server 行为可配置**：默认立即 succeed（验证状态机用）；可设 `GO2W_MOCK_NAV_DELAY=2.0` 让导航耗时 2s（验证 feedback/进度推送）；可设 `GO2W_MOCK_NAV_FAIL_ROOM=客厅` 让指定房间导航失败（验证失败处理）。
3. **mock server 发 feedback**（`NavigateToPose.Feedback` 含 `distance_remaining`），让编排的进度推送（`type=search_room` 的 `progress` 字段）有数据源。
4. **启动顺序**：先起 mock_nav2_action，再起 nx_web_server（编排的 Nav2Client `wait_for_server` 才不超时）。verify 脚本里强制这个顺序。

### 决策 6：房间地图格式 → **推荐 schema 见 §6（rooms 数组，每房间 name/aliases/nav_pose/search_area）**

详见 §6 的 YAML schema + 示例 + 校验规则。关键点：`nav_pose` 的 yaw 用弧度（与 ROS REP-103 / Nav2 一致），`search_area` 的 origin 是 map 坐标系的绝对坐标（不是相对 nav_pose），方便直接喂给 `planner.plan_lawnmower(width, height, spacing, origin_x, origin_y)`（planner.py:11 的签名原样复用）。

---

## 4. 关键设计决策总表（Generator 速查）

| # | 决策点 | 推荐 | 一句话理由 |
|---|---|---|---|
| 1 | Nav2 集成方式 | rclpy action client (`NavigateToPose`) | 标准接口、与阶段D 解耦、mock 易替 |
| 2 | 房间数据来源 | 静态 YAML (`config/rooms.yaml`) | 最简、可手编辑、语义建图是未来工作 |
| 3 | 编排放哪 | 集成进 TaskManager + `nx_room_orchestrator.py` 协作类 | 零跨进程、复用阶段A/B 组件 |
| 4 | Nav2 线程模型 | worker 线程内 `spin_until_complete` + 超时 | rclpy 官方推荐、串行 worker 模型契合 |
| 5 | Nav2 降级 | mock Nav2 action server（fake 立即 ARRIVED） | 编排状态机端到端可 mock 验证 |
| 6 | 房间地图格式 | YAML schema（§6） | nav_pose 弧度、search_area 绝对坐标 |

---

## 5. 编排状态机图（IDLE → SELECT_ROOM → NAVIGATE → ARRIVED → SEARCH → DETECT → REPORT）

```
                          ┌──────────────────────────────────────────────┐
                          │                                              │
                          ▼                                              │
   ┌─────────┐   用户输入     ┌──────────────┐  房间未找到   ┌──────────┴───┐
   │  IDLE   │ ───"搜索客厅"─> │ SELECT_ROOM  │ ───────────> │ FAILED (no_room) │
   └─────────┘   /api/         └──────┬───────┘              └──────────────┘
        ▲          search_room         │ 房间找到
        │                               ▼
        │                          ┌──────────────┐  Nav2 server  ┌────────────────┐
        │                          │  NAVIGATE    │ ──不可用────> │ FAILED (no_nav) │
        │                          │ (发入口goal)  │               └────────────────┘
        │                          └──────┬───────┘
        │                                 │ goal 接受
        │                                 ▼
        │   cancel_all / e_stop     ┌──────────────┐  导航超时/失败  ┌──────────────────┐
        │   ┌─────────────────────│ NAVIGATING   │ ──────────────> │ FAILED (nav_err)  │
        │   │                       │ (feedback推送) │                  └──────────────────┘
        │   │                       └──────┬───────┘
        │   │                              │ goal SUCCEEDED (ARRIVED)
        │   │                              ▼
        │   │                       ┌──────────────┐
        │   │                       │   ARRIVED    │ (生成航点序列 plan_lawnmower)
        │   │                       └──────┬───────┘
        │   │                              │ 进入逐航点循环
        │   │                              ▼
        │   │   还有航点未走         ┌──────────────┐  发航点 Nav2 goal
        │   │  ┌───────────────────│   SEARCH     │ ────────────┐
        │   │  │                     │ (wp[i] 导航)  │              │
        │   │  │                     └──────┬───────┘              │
        │   │  │                            │ 航点 ARRIVED          │
        │   │  │                            ▼                       │
        │   │  │                     ┌──────────────┐               │
        │   │  │                     │   DETECT     │ 读最新检测快照 │
        │   │  │                     │ (记录发现)    │ + 推 type=search│
        │   │  │                     └──────┬───────┘               │
        │   │  │                            │                       │
        │   │  └────────────────────────────┘ 下一个航点             │
        │   │                                                     │
        │   │   所有航点走完                                       │
        │   │                            ▼                          │
        │   │                     ┌──────────────┐                  │
        │   │                     │   REPORT     │ 生成 MissionReport
        │   │                     │ (汇总+推送)   │ 推 type=mission_report
        │   │                     └──────┬───────┘                  │
        │   │                            │                          │
        │   └────────────────────────────┘ cancel 时 NAV2 cancel_goal│
        │                                                            │
        └────────────────────────────<──────────────────────────────┘
                                  任务完成, 回 IDLE


   状态机字段 (type=search_room 的 data.phase):
     SELECT_ROOM | NAVIGATE | NAVIGATING | ARRIVED | SEARCH | DETECT | REPORT | DONE | FAILED

   失败子态 (data.reason):
     no_room | no_nav | nav_timeout | nav_rejected | nav_aborted | cancelled
```

**状态机实现要点**：
1. **状态机不是独立线程**——它在 TaskManager worker 线程内同步推进（worker 取到 `search_room` 任务后，整个 `_execute_search_room` 是一个同步方法，内部按阶段顺序走完）。
2. **每个阶段切换时 ws_broadcast** `{"type":"search_room","data":{"phase":"NAVIGATE","room":"客厅","progress":0.0}}`，前端 console.log（不改 panel.html，新增 type 不破坏前端）。
3. **cancel 响应**：worker 线程在每个阶段切换点检查 `task.status == "cancelled"`（与 panel.py:619 的 search_area 取消检查同款），是则调 Nav2Client 的 `cancel_goal_async` 并退出。
4. **DETECT 阶段**：到达每个航点后，**主动调** `ai_engine.get_detections_world(robot_x, robot_y, robot_yaw)` 取最新检测快照（阶段B 的 _video_yolo_loop 持续跑，这里只读快照），把检测到的目标（去重）加进 mission 的 detections 列表，ws 推 `type=search`（复用阶段B 的 type=search 增量推送格式，`{"found":[...]}`）。
5. **REPORT 阶段**：生成 MissionReport dict（含 mission_id/room/start_time/end_time/duration/waypoints_total/waypoints_visited/targets_found/detections/result_path），ws 推 `type=mission_report`，task.result 存这份 dict。

---

## 6. 房间地图 YAML schema + 示例

### 6.1 Schema（`config/rooms.yaml`）

```yaml
# Go2W 房间级搜索地图 (阶段E, 静态 YAML, 人工标定)
# 坐标系: map (与 Nav2 的全局坐标系一致, 通常是 FAST_LIO/map_server 的 map frame)
# yaw: 弧度 (ROS REP-103, 正=左转), 与 Nav2 PoseStamped.orientation 四元数等价转换

frame_id: map              # 所有 pose 的坐标系 (发给 Nav2 的 goal.header.frame_id 用此值)
version: "1.0"             # schema 版本, RoomMap.load 时校验
default_search_spacing: 2.5  # 默认搜索行间距 (米), 房间级可被 room.search_area.spacing 覆盖
default_search_pattern: lawnmower  # 默认覆盖模式, 可被 room.search_area.pattern 覆盖

rooms:
  - name: 客厅              # 主名 (中文, 用户指令匹配的首选)
    aliases: ["living room", "living", "起居室"]  # 别名 (VLM/关键词匹配时也查这里)
    nav_pose:               # 房间入口导航目标 (狗先到这里再开搜)
      x: 2.5                # map 坐标系 X (米)
      y: 1.8                # map 坐标系 Y (米)
      yaw: 0.0              # 到达时朝向 (弧度)
    search_area:            # 房间内搜索矩形 (喂给 planner.plan_lawnmower)
      width: 5.0            # 矩形宽 (米, X 方向)
      height: 4.0           # 矩形高 (米, Y 方向)
      origin_x: 1.0         # 矩形左下角 X (map 绝对坐标, 不是相对 nav_pose)
      origin_y: 0.5         # 矩形左下角 Y
      spacing: 1.5          # 行间距 (覆盖 default_search_spacing)
      pattern: lawnmower    # 覆盖模式 (覆盖 default_search_pattern)
    target_classes: []      # 搜索目标类别过滤 (空=检测所有, ["person"]=只记人)

  - name: 卧室
    aliases: ["bedroom", "睡房"]
    nav_pose: {x: -1.2, y: 3.4, yaw: 1.57}
    search_area: {width: 4.0, height: 3.5, origin_x: -3.0, origin_y: 2.0, spacing: 1.2, pattern: lawnmower}
    target_classes: ["person"]

  - name: 厨房
    aliases: ["kitchen"]
    nav_pose: {x: 4.0, y: -2.0, yaw: -1.57}
    search_area: {width: 3.0, height: 3.0, origin_x: 3.0, origin_y: -3.5, spacing: 1.0}
    target_classes: []
```

### 6.2 校验规则（`RoomMap.load` 必查，Critic 会核对）

1. **顶层必填**：`frame_id`、`version`、`rooms`（数组，可为空）。
2. **每个 room 必填**：`name`（非空字符串）、`nav_pose.{x,y,yaw}`（数值）、`search_area.{width,height,origin_x,origin_y}`（数值）。
3. **可选字段**：`aliases`（数组，默认空）、`search_area.spacing`（默认顶层 `default_search_spacing`）、`search_area.pattern`（默认顶层 `default_search_pattern`，必须是 `lawnmower` 或 `spiral`）、`target_classes`（默认空）。
4. **数值约束**：`width > 0`、`height > 0`、`spacing > 0`（否则校验失败，load 抛 ValueError）。
5. **name 唯一**：rooms 数组内 `name` 不重复（重复则 load 抛 ValueError，避免匹配歧义）。
6. **YAML 缺失/格式错**：load 抛 FileNotFoundError / yaml.YAMLError，编排层 catch 后 ws 推 `type=search_room` 的 `phase:FAILED, reason:no_room_map`。

### 6.3 房间匹配逻辑（`RoomMap.find(query)`）

```python
def find(self, query: str) -> Optional[Room]:
    """用户指令文本 → 匹配房间。
    匹配优先级:
      1. name 完全相等 (区分大小写, 中文精确匹配)
      2. aliases 完全相等
      3. name 子串包含 (如 query="搜索客厅" 包含 name="客厅")
      4. aliases 子串包含
      5. 都不匹配 → None
    多个匹配时返回第一个 (name 唯一性保证主名不冲突; alias 冲突由人工 YAML 避免)
    """
```

**关键**：匹配在 `RoomMap` 内做纯文本匹配（不调 VLM，快）。VLM/关键词解析在更上游（TaskManager.process_command 或 `_execute_search_room` 入口前），把"搜索客厅"解析成 `{"type":"search_room","params":{"room":"客厅"}}` 任务后，再把 `"客厅"` 喂给 `RoomMap.find`。这样房间匹配逻辑与指令解析解耦（VLM 负责理解"搜索客厅"是 search_room 任务 + 提取房间名，RoomMap 负责把房间名映射到具体 pose）。

---

## 7. 新建/修改文件清单（文件级 + 函数级，Generator 直接实现）

### 7.1 新建文件

#### `web/nx_room_orchestrator.py`（核心，约 350 行）
**职责**：房间级搜索编排器。封装 Nav2 action client、房间地图加载、状态机驱动、MissionReport 生成。作为"组件"注入 TaskManager（与 NxAiEngine 同款注入模式），不直接处理 HTTP/WS（TaskManager 调用它）。

**关键签名**：
```python
import json
import math
import os
import threading
import time
import uuid
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

logger = logging.getLogger("go2w.room_orch")


# ============================================================================
# Room / RoomMap — 房间地图 (YAML 加载, spec §6)
# ============================================================================
class Room:
    """单个房间定义 (从 rooms.yaml 一条 room 反序列化)。"""
    def __init__(self, name: str, aliases: List[str], nav_pose: dict,
                 search_area: dict, target_classes: List[str]):
        self.name = name
        self.aliases = list(aliases)
        self.nav_pose = {
            "x": float(nav_pose["x"]),
            "y": float(nav_pose["y"]),
            "yaw": float(nav_pose["yaw"]),
        }
        self.search_area = {
            "width": float(search_area["width"]),
            "height": float(search_area["height"]),
            "origin_x": float(search_area["origin_x"]),
            "origin_y": float(search_area["origin_y"]),
            "spacing": float(search_area.get("spacing", 2.5)),
            "pattern": str(search_area.get("pattern", "lawnmower")),
        }
        self.target_classes = list(target_classes)

    def to_dict(self) -> dict: ...

    @classmethod
    def from_yaml_dict(cls, d: dict) -> "Room":
        """从 YAML 解析的 dict 构造, 校验必填字段, 失败抛 ValueError (spec §6.2)。"""


class RoomMap:
    """房间地图集合 (rooms.yaml 的内存表示)。"""
    def __init__(self, frame_id: str, rooms: List[Room],
                 default_spacing: float = 2.5, default_pattern: str = "lawnmower"):
        self.frame_id = frame_id
        self.rooms = rooms
        self.default_spacing = default_spacing
        self.default_pattern = default_pattern

    @classmethod
    def load(cls, path: str) -> "RoomMap":
        """从 YAML 文件加载 + 校验 (spec §6.2)。
        失败: FileNotFoundError (文件不存在) / ValueError (字段缺失/数值非法/name 重复)
              / yaml.YAMLError (格式错)
        """

    def find(self, query: str) -> Optional[Room]:
        """房间匹配 (spec §6.3): name>aliases 完全相等 > name/aliases 子串包含。"""

    def list_rooms(self) -> List[str]:
        """返回所有房间主名 (供 VLM/前端列出可搜房间)。"""


# ============================================================================
# Nav2ActionClient — Nav2 NavigateToPose action client 封装 (spec 决策 1/4)
# ============================================================================
class Nav2ActionClient:
    """Nav2 NavigateToPose action client 的同步等待封装。

    线程模型 (spec 决策 4):
      - 构造时挂在 nx_web_server 的 NxWebNode 上 (与主 rclpy.spin 共享 node)
      - send_goal_and_wait() 在 TaskManager worker 线程内调,
        用 rclpy.spin_until_complete(node, future, timeout) 同步等
      - 主 spin 线程 (线程1) 不干涉 (spin_until_complete 临时驱动 future 回调)
    """
    def __init__(self, node: Node, action_name: str = '/navigate_to_pose',
                 goal_accept_timeout: float = 5.0,
                 nav_complete_timeout: float = 120.0):
        self._node = node
        self._client = ActionClient(
            node, NavigateToPose, action_name,
            callback_group=ReentrantCallbackGroup()  # 避免 callback group 死锁
        )
        self._goal_accept_timeout = goal_accept_timeout
        self._nav_complete_timeout = nav_complete_timeout
        self._lock = threading.Lock()
        self._current_handle = None      # 当前 goal_handle (cancel 用)
        self._feedback_callback = None   # 外部注入的 feedback 回调 (推进度用)
        self._cancelled = False

    def wait_for_server(self, timeout: float = 2.0) -> bool:
        """探测 Nav2 action server 是否在线 (阶段D/mock 判据)。"""

    def set_feedback_callback(self, cb):
        """注入 feedback 回调: cb(distance_remaining, estimated_time)。"""
        self._feedback_callback = cb

    def send_goal_and_wait(self, x: float, y: float, yaw: float,
                           frame_id: str = 'map') -> Dict:
        """同步发 Nav2 goal + 等接受 + 等到达。
        返回:
          {"ok": True, "status": 4}                              成功 (STATUS_SUCCEEDED)
          {"ok": False, "reason": "no_server"}                   server 不在线
          {"ok": False, "reason": "rejected"}                    goal 被拒绝
          {"ok": False, "reason": "timeout"}                     导航超时
          {"ok": False, "reason": "aborted", "status": N}        Nav2 主动 abort
          {"ok": False, "reason": "cancelled"}                   被 cancel_goal 取消
        实现 (spec 决策 4):
          1. wait_for_server(goal_accept_timeout) → 不在则 no_server
          2. 构造 NavigateToPose.Goal (PoseStamped, yaw→四元数)
          3. send_goal_async(goal, feedback_callback) → goal_future
          4. spin_until_complete(node, goal_future, goal_accept_timeout)
             → 超时 no_server / rejected / 拿到 handle
          5. handle.get_result_async() → result_future
          6. spin_until_complete(node, result_future, nav_complete_timeout)
             → 超时 timeout / status==4 ok / 其他 aborted
        四元数转换: qz=sin(yaw/2), qw=cos(yaw/2) (休眠包 orchestrator_node.py:176-180 同款)
        """

    def cancel_current(self) -> bool:
        """取消当前进行中的 Nav2 goal (cancel_all / e_stop 时调)。"""
        # 标 _cancelled=True, 调 handle.cancel_goal_async()


# ============================================================================
# MissionReport — 任务最终报告 (go2w_interfaces/MissionReport.msg 的 dict 等价)
# ============================================================================
def build_mission_report(mission_id: str, room: Room, waypoints_total: int,
                         waypoints_visited: int, detections: List[dict],
                         start_time: float, end_time: float,
                         result_path: str = "") -> dict:
    """生成 MissionReport dict (spec §5 REPORT 阶段)。
    返回结构对齐 go2w_interfaces/MissionReport.msg:
      {mission_id, room, duration_sec, area:{width,height,...},
       waypoints_visited, waypoints_total, targets_found, detections, result_path}
    """


# ============================================================================
# RoomSearchOrchestrator — 房间级搜索编排器 (spec §5 状态机)
# ============================================================================
class RoomSearchOrchestrator:
    """房间级搜索编排器 (spec 决策 3, 集成进 TaskManager)。

    生命周期: nx_web_server.main() 创建一个实例, 注入 TaskManager
    (task_mgr.room_orchestrator = RoomSearchOrchestrator(node, ai_engine, ws_broadcast))
    TaskManager._execute_search_room 调用 self.room_orchestrator.run(task)。
    """

    def __init__(self, node: Node, ai_engine=None,
                 ws_broadcast_fn=None,
                 rooms_yaml_path: Optional[str] = None):
        self._node = node
        self._ai = ai_engine          # NxAiEngine (阶段B), 读 _latest_dets
        self._ws = ws_broadcast_fn    # ws_broadcast 注入 (与 nx_ai_node 同款)
        self._lock = threading.Lock()
        # 房间地图 (懒加载, 首次 run 时 load; 路径 env GO2W_ROOMS_YAML 默认 config/rooms.yaml)
        self._rooms_yaml = rooms_yaml_path or os.environ.get(
            'GO2W_ROOMS_YAML',
            os.path.join(os.path.dirname(_WEB_DIR) if False else os.getcwd(),
                         'config', 'rooms.yaml')
        )
        self._room_map: Optional[RoomMap] = None
        # Nav2 client (懒创建, 首次 run 时)
        self._nav: Optional[Nav2ActionClient] = None
        # 当前 mission 状态 (cancel 检查用)
        self._current_task_id: Optional[str] = None
        self._cancelled = False

    # ---- 房间地图加载 ----
    def _ensure_room_map(self) -> Optional[RoomMap]:
        """懒加载房间地图。失败 ws 推 phase:FAILED, reason:no_room_map, 返回 None。"""

    def reload_rooms(self) -> bool:
        """重新加载 YAML (供 HTTP /api/reload_rooms 调, 改 YAML 后热加载)。"""

    # ---- Nav2 client ----
    def _ensure_nav(self) -> Nav2ActionClient:
        """懒创建 Nav2ActionClient (首次 run 时)。"""

    # ---- 状态机驱动 (spec §5) ----
    def run(self, task) -> None:
        """房间级搜索主入口 (TaskManager worker 线程调)。

        task.params = {"room": "客厅", "target_classes": [...], ...}
        状态机 (spec §5): SELECT_ROOM → NAVIGATE → ARRIVED → SEARCH → DETECT → REPORT
        每阶段切换 ws_broadcast type=search_room, data.phase 更新。
        检测发现 ws_broadcast type=search (增量 found 列表, 复用阶段B 格式)。
        完成 ws_broadcast type=mission_report, data=MissionReport dict。
        任何阶段失败: task.status=failed, ws 推 phase:FAILED + reason。
        task.status 由调用方 (TaskManager worker) 在 run 返回后统一处理 (与 _execute_search 同款)。
        """
        # 1. _phase("SELECT_ROOM"); room = self._ensure_room_map().find(task.params["room"])
        #    → None 则 _fail("no_room") return
        # 2. _phase("NAVIGATE"); nav = self._ensure_nav()
        #    → nav.wait_for_server() 失败 _fail("no_nav") return
        # 3. _phase("NAVIGATING", progress=0.0); r = nav.send_goal_and_wait(room.nav_pose...)
        #    → r.ok False 则 _fail(r.reason) return; 中途被 cancel 检查 _cancelled
        # 4. _phase("ARRIVED"); 生成航点 wp = plan_lawnmower(room.search_area...)
        # 5. for i, w in enumerate(wp):
        #      _phase("SEARCH", progress=i/len(wp), current_wp=i)
        #      nav.send_goal_and_wait(w) → 失败 _fail("wp_nav_err") return
        #      _phase("DETECT", current_wp=i)
        #      self._snapshot_detections(room, robot_pose) → 增量 found 推 type=search
        #      if self._cancelled: _fail("cancelled") return
        # 6. _phase("REPORT"); report = build_mission_report(...)
        #    ws_broadcast type=mission_report; task.result = report; task.status=completed

    # ---- 辅助 ----
    def _phase(self, phase: str, **extra) -> None:
        """推送状态机进度: ws_broadcast({"type":"search_room","data":{"phase":phase, ...}})。"""

    def _fail(self, reason: str) -> None:
        """推送失败: ws_broadcast type=search_room, phase:FAILED, reason。"""

    def _snapshot_detections(self, room: Room, robot_x: float, robot_y: float,
                              robot_yaw: float, found_list: List[str],
                              detections_log: List[dict]) -> None:
        """读阶段B ai_engine 最新检测快照, 过滤 target_classes, 去重加进 found_list/detections_log,
        增量推 type=search (复用阶段B 格式 {"found":[...]}), 记录发现位姿 (robot_x/y/yaw + 时间戳)。
        ai_engine=None (阶段A 退化) 时跳过检测, found_list 保持空。
        """

    def cancel(self) -> None:
        """cancel_all / e_stop 时调: 标 _cancelled + nav.cancel_current()。"""
```

**实现要点**：
- **懒加载**：`__init__` 不加载 YAML、不创建 Nav2Client（启动快、Nav2 未就绪时不报错）。首次 `run` 才加载/创建。
- **`_WEB_DIR` 路径**：rooms.yaml 默认放仓库根 `config/rooms.yaml`（不在 web/ 下，因为它是配置不是代码）。env `GO2W_ROOMS_YAML` 覆盖路径。
- **yaw → 四元数**：`qz = sin(yaw/2), qw = cos(yaw/2), qx = qy = 0`（休眠包 orchestrator_node.py:176-180 同款，对齐 REP-103）。
- **`plan_lawnmower` 复用**：直接 `from planner import plan_lawnmower, plan_spiral`（planner.py 在 `src/go2w_orchestrator/go2w_orchestrator/`，要么 sys.path 加这目录，要么把 plan_lawnmower 复制到本文件——**推荐后者**，因为 nx_web_server.py 已经内联复制了 plan_lawnmower/plan_spiral（见 nx_web_server.py:110-148），nx_room_orchestrator 应该**从 nx_web_server import**这两个函数，避免三处复制）。
- **status==4 判定**：`NavigateToPose` action 的 `STATUS_SUCCEEDED = 4`（休眠包 orchestrator_node.py:207 同款，rclpy action 标准值）。
- **detection 快照**：调 `ai_engine.get_detections_world(robot_x, robot_y, robot_yaw)`（阶段B 已实现，nx_ai_node.py），返回 `[{x,y,class}]`。但这个返回的是世界坐标——阶段E 要记录的"发现位姿"是**狗当时位姿**（机器人坐标系），不是目标世界坐标。所以 `_snapshot_detections` 要记录的是 `{class, robot_x, robot_y, robot_yaw, t}`（狗发现目标时的位姿 + 时间戳），目标世界坐标作为辅助字段。对齐 `go2w_interfaces/TargetDetection.msg` 的 `robot_x/robot_y/robot_yaw` 字段。
- **target_classes 过滤**：`room.target_classes` 非空时，只记录 class 在该列表的检测（如 `["person"]` 只记人）；空则全记。

#### `web/mock_nav2_action.py`（验证用，约 150 行）
**职责**：mock Nav2 action server（fake `/navigate_to_pose`），让阶段E 编排状态机在**没有真 Nav2/SLAM/狗**时端到端可验证（spec 决策 5）。

**关键签名**：
```python
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose

logger = logging.getLogger("go2w.mock_nav2")


class MockNav2ActionServer(Node):
    """Fake NavigateToPose action server (spec 决策 5)。

    行为 (env 可配):
      - GO2W_MOCK_NAV_DELAY (默认 0.5s): 收到 goal 后模拟导航耗时, 期间发 feedback
      - GO2W_MOCK_NAV_FAIL (默认 ""): 空格分隔的 "x,y" 列表, 这些坐标的 goal 会 abort
      - GO2W_MOCK_NAV_REJECT (默认 ""): 这些坐标的 goal 会被 reject (测试 rejected 路径)
    默认: 收到 goal → 0.5s 后 status=SUCCEEDED (立即到达, 验证状态机用)
    """
    def __init__(self):
        super().__init__('mock_nav2_action_server')
        self._delay = float(os.environ.get('GO2W_MOCK_NAV_DELAY', '0.5'))
        self._fail_set = self._parse_xy_set(os.environ.get('GO2W_MOCK_NAV_FAIL', ''))
        self._reject_set = self._parse_xy_set(os.environ.get('GO2W_MOCK_NAV_REJECT', ''))
        self._action = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info(
            f"Mock Nav2 action server 就绪: /navigate_to_pose "
            f"(delay={self._delay}s, fail={self._fail_set}, reject={self._reject_set})"
        )

    def _parse_xy_set(self, s: str) -> set: ...   # "1.0,2.0 3.0,4.0" → {(1.0,2.0),(3.0,4.0)}

    def _goal_cb(self, goal_request):
        """reject 列表内的坐标拒绝, 其余接受。"""

    def _cancel_cb(self, goal_handle):
        return rclpy.action.CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """模拟导航: 发 feedback (distance_remaining 递减) → delay → succeed/abort。"""
        # 取 goal.pose.pose.position.x/y
        # if (x,y) in reject_set: goal_handle.abort(); return
        # if (x,y) in fail_set:
        #     发几次 feedback → goal_handle.abort(); return NavigateToPose.Result()
        # 正常: 按 delay 分 5 次发 feedback (distance_remaining 从 5m→0m 递减)
        # time.sleep(self._delay)
        # goal_handle.succeed()
        # return NavigateToPose.Result()


def main():
    rclpy.init()
    node = MockNav2ActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

**实现要点**：
- **feedback 字段**：`NavigateToPose.Feedback` 的 `distance_remaining`（float）+ `estimated_time_remaining`（duration）。mock 按延迟分 5 次发，distance 从某个初值递减到 0，让 RoomSearchOrchestrator 的 progress 推送有数据。
- **reject vs abort**：reject 是 `_goal_cb` 返回 `GoalResponse.REJECT`（goal 根本没接受）；abort 是 `_execute` 里调 `goal_handle.abort()`（接受了但执行中失败）。两条失败路径都覆盖。
- **cancel 响应**：`_cancel_cb` 总是 ACCEPT，`_execute` 内部检查 `goal_handle.is_cancel_requested` 则 abort。

#### `config/rooms.yaml`（房间地图，约 50 行）
**职责**：阶段E 默认房间地图。Generator 实现时填 §6.1 示例的内容（客厅/卧室/厨房三个房间，坐标是假的占位值，部署时人工标定替换）。

**实现要点**：
- 顶部注释明确"坐标为占位值，部署时用 `ros2 topic echo /odom` 在真 Nav2 地图上标定"。
- 至少 3 个房间，覆盖不同 `target_classes`（客厅全检、卧室只检 person、厨房全检）。
- 路径 `config/rooms.yaml`（仓库根，不在 web/ 下）。

#### `web/verify_stage_e.py`（验证脚本，约 120 行）
**职责**：阶段E 端到端验证。NX 上启动 nx_web + mock_nav2 + mock 检测注入，跑 curl + python websocket 客户端断言编排状态机全程。**不依赖狗硬件**。

**验证项**（每项独立 PASS/FAIL，详见 §9.1）：
1. `config/rooms.yaml` 存在且 `RoomMap.load` 校验通过
2. 启动 mock_nav2 + nx_web，`curl /api/search_room?room=客厅` → `{"ok":true}`
3. WS 10 秒内收到 `type=search_room` 且 `phase` 依次出现 `SELECT_ROOM/NAVIGATE/NAVIGATING/ARRIVED`
4. WS 30 秒内收到 `type=search_room` 的 `phase:REPORT` 或 `DONE`
5. WS 收到 `type=mission_report`，含 `room/waypoints_visited/targets_found/detections` 字段
6. mock 检测注入（环境变量或 mock_person.png），`type=search` 的 `found` 列表非空
7. `curl /api/search_room?room=不存在的房间` → WS 收到 `phase:FAILED, reason:no_room`
8. `GO2W_MOCK_NAV_FAIL=2.5,1.8`（客厅入口坐标）重启 mock_nav2，搜客厅 → WS `phase:FAILED, reason:nav_aborted` 或 `nav_err`
9. 搜中途 `curl /api/e_stop` → WS `phase:FAILED, reason:cancelled` + Nav2 cancel 被 mock 接收

#### `web/verify_stage_e.sh`（一键验证 wrapper，约 30 行）
**职责**：bash wrapper，按顺序起 mock_nav2 + nx_web + 跑 verify_stage_e.py，结束清理进程。参考阶段A `verify_nx_web.sh` 的模式。

#### `docs/room_calibration.md`（运维 SOP，约 40 行）
**职责**：房间标定流程文档（不在代码里）。步骤：建图完成 → 拿狗站到房间入口 → `ros2 topic echo /odom` 记录 x/y/yaw → 编辑 `config/rooms.yaml` → `/api/reload_rooms` 热加载 → 验证 `/api/search_room?room=Xxx`。

### 7.2 修改文件（仅 `web/nx_web_server.py`，改动最小化）

#### `web/nx_web_server.py`（阶段A/B 文件，阶段E 注入 RoomSearchOrchestrator + 加 search_room 任务类型 + HTTP 端点）
**改动点**（保持阶段A/B 线程模型 + 契约不变，只加房间编排注入）：

1. **import RoomSearchOrchestrator**（顶部，懒加载）：
   ```python
   # 阶段E: 房间级搜索编排 (懒加载, 与 NxAiEngine 同款注入)
   ROOM_ORCH_OK = False
   _ROOM_ORCH_ERR = ""
   RoomSearchOrchestrator = None
   try:
       from nx_room_orchestrator import RoomSearchOrchestrator
       ROOM_ORCH_OK = True
       logger.info("阶段E 房间级搜索编排可用 (nx_room_orchestrator)")
   except Exception as _e:
       _ROOM_ORCH_ERR = str(_e)
       logger.warning(f"阶段E 房间级搜索编排不可用: {_e}")
   ```

2. **TaskManager 加 `room_orchestrator` 属性 + `_execute_search_room` 分支**：
   ```python
   class TaskManager:
       def __init__(self, robot, vlm_engine=None, detector=None, room_orchestrator=None):
           ...
           self.room_orchestrator = room_orchestrator  # 阶段E 注入

       # _worker 的任务分发表加分支:
       #   elif task.type == "search_room":
       #       if self.room_orchestrator is None:
       #           task.status = "failed"; task.result = "房间编排未启用"
       #       else:
       #           self.room_orchestrator.run(task)  # 同步跑状态机, 内部设 task.status
       #       # (worker 会在 finally 清理 _active, 与 search_area 同款)

       def cancel_all(self):
           ...
           # 阶段E 新增: 取消房间编排
           if self.room_orchestrator and self._active and self._active.type == "search_room":
               self.room_orchestrator.cancel()
   ```
   **关键**：`_execute_search_room` 不单独写方法（RoomSearchOrchestrator.run 已经封装状态机），worker 直接调 `self.room_orchestrator.run(task)`。run 内部负责设 `task.status`（completed/failed/cancelled）和 `task.result`（MissionReport dict）。

3. **HTTP 加 `/api/search_room` 端点**（POST，新增端点，不改现有 12 端点）：
   ```python
   # do_POST 加分支:
   elif p.path == '/api/search_room':
       # query: room (必填), target_classes (可选, 逗号分隔)
       # body JSON 也支持: {"room":"客厅","target_classes":["person"]}
       room = q.get('room', [''])[0]
       if body:
           try:
               jb = json.loads(body)
               room = jb.get('room', room)
           except Exception: pass
       if not room:
           self._json({"ok": False, "msg": "缺少 room 参数"})
           return
       tc_str = q.get('target_classes', [''])[0]
       target_classes = [s.strip() for s in tc_str.split(',') if s.strip()] if tc_str else []
       task_mgr.add_list([Task("search_room",
                                {"room": room, "target_classes": target_classes}, 5)])
       self._json({"ok": True, "msg": f"搜索房间 '{room}' 已入队"})

   # do_GET 加分支 (列房间 + reload):
   elif p.path == '/api/rooms':
       # 列出 rooms.yaml 所有房间名
       rooms = (room_orchestrator.list_rooms() if room_orchestrator else [])
       self._json({"ok": True, "rooms": rooms})

   elif p.path == '/api/reload_rooms':
       ok = room_orchestrator.reload_rooms() if room_orchestrator else False
       self._json({"ok": ok})
   ```
   **关键**：新增 3 个端点（`/api/search_room` POST、`/api/rooms` GET、`/api/reload_rooms` GET），**不动现有 12 端点**（阶段A 契约不破坏）。

4. **`process_command` 扩展 search_room 解析**（`_vlm_parse_command` / `_fallback_parse` 加 search_room 任务类型）：
   ```python
   # _fallback_parse 加分支 (在 "搜索" 分支前):
   #   if "搜索" in text or "找" in text:
   #       # 阶段E: 尝试从指令里提取房间名
   #       room = self._extract_room_name(text)  # 扫 rooms.yaml 的 name/aliases
   #       if room:
   #           r["tasks"] = [{"type":"search_room","priority":7,"params":{"room":room}}]
   #           r["response"] = f"搜索{room}"
   #       else:
   #           # 无房间名, 退化阶段A 矩形搜索 (现有行为不破坏)
   #           r["tasks"] = [{"type":"search_area","priority":5,"params":{...}}]
   #           r["response"] = "开始搜索"

   # _vlm_parse_command 的 sys_prompt 加 search_room 任务类型说明 (让 VLM 学会输出 search_room):
   #   - search_room: {"room":"房间名"}  # 搜索指定房间 (客厅/卧室/厨房)
   #   示例: 输入"搜索客厅" → {"tasks":[{"type":"search_room","params":{"room":"客厅"}}]}
   ```

5. **`main()` 注入 RoomSearchOrchestrator**：
   ```python
   # 阶段E: 创建 RoomSearchOrchestrator + 注入 TaskManager
   room_orchestrator = None
   if ROOM_ORCH_OK and RoomSearchOrchestrator is not None:
       try:
           room_orchestrator = RoomSearchOrchestrator(
               node=node,
               ai_engine=ai_engine,           # 阶段B 的 NxAiEngine (读检测)
               ws_broadcast_fn=ws_broadcast,  # 推 type=search_room/mission_report
           )
           logger.info("阶段E: RoomSearchOrchestrator 已注入 TaskManager")
       except Exception as e:
           logger.error(f"阶段E RoomSearchOrchestrator 启动失败: {e}")
           room_orchestrator = None

   # TaskManager 构造加 room_orchestrator 参数:
   task_mgr = TaskManager(robot, vlm_engine=vlm_proxy, detector=detector_proxy,
                          room_orchestrator=room_orchestrator)
   ```

6. **广播退出时清理**（`main` finally 加 `room_orchestrator` 的 cancel）：
   ```python
   finally:
       ...
       try:
           if room_orchestrator is not None:
               room_orchestrator.cancel()  # 取消进行中的 Nav2 goal
       except Exception: pass
   ```

### 7.3 不动文件清单（Generator 勿碰，Critic 会核对）

- `web/static/panel.html`（前端无感，禁止改 JS；新增 ws type 走 console.log）
- `web/static/map.js`（同上）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py`（控狗逻辑红线）
- `src/go2w_bridge/go2w_bridge/nx_sensor_node.py`（NX 传感器，本阶段不动）
- `ai/detector.py` / `ai/vlm.py`（阶段B 红线继续生效）
- `web/nx_ai_node.py`（阶段B 文件，阶段E **只读不写**——RoomSearchOrchestrator 调它的 `get_detections_world` 读快照）
- `web/panel.py`（PC fallback，退役不删）
- `src/go2w_orchestrator/`（休眠包，决策 3 明确不复活；planner.py 的 `plan_lawnmower` 已在 nx_web_server.py 内联复制，阶段E 复用内联版）
- `src/go2w_interfaces/`（msg/srv 已定义齐全，阶段E **不新增 msg**，MissionReport 走 WS dict 不走 ROS2 msg——因为前端只接 WS）

### 7.4 文件改动量预估

| 文件 | 类型 | 预估行数 | 改动性质 |
|---|---|---|---|
| `web/nx_room_orchestrator.py` | 新建 | ~350 | 核心编排器 |
| `web/mock_nav2_action.py` | 新建 | ~150 | 验证用 mock |
| `config/rooms.yaml` | 新建 | ~50 | 房间地图 |
| `web/verify_stage_e.py` | 新建 | ~120 | 验证脚本 |
| `web/verify_stage_e.sh` | 新建 | ~30 | 验证 wrapper |
| `docs/room_calibration.md` | 新建 | ~40 | 标定 SOP |
| `web/nx_web_server.py` | 修改 | +60 | 注入 + search_room 任务 + HTTP 端点 |
| `gan-harness/eval-rubric-stage-e.md` | 新建 | ~120 | Critic 消费 |

---

## 8. Nav2 action 接线（订阅/发布/action client）

### 8.1 RoomSearchOrchestrator 的 Nav2 接线

| 方向 | 接口 | 类型 | 对端 | 频率/时机 | QoS/回调组 | 用途 |
|---|---|---|---|---|---|---|
| **Action Client** | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | → Nav2 bt_navigator（或阶段E 的 mock_nav2_action） | 按需（每个航点一次 + 房间入口一次） | ReentrantCallbackGroup（避免与主 spin 死锁） | 发导航目标 + 等到达 |

**没有订阅/发布**——阶段E 编排**不新增任何 ROS2 topic/service**，只新增一个 action client。理由：
- 编排的状态进度走 WS（`type=search_room`）给前端，不走 ROS2 topic（前端不接 ROS2）
- 检测结果走阶段B 的 NxAiEngine 内存读取（`get_detections_world`），不走 ROS2
- 狗状态走阶段A 的 NxWebNode 订阅缓存（`/dog_state` 等），RoomSearchOrchestrator 不重复订阅
- MissionReport 走 WS（`type=mission_report`），不走 ROS2 msg（go2w_interfaces/MissionReport.msg 是给未来 ROS2 节点间通信用，阶段E 前端用不到）

### 8.2 Nav2Client 与主 rclpy.spin 的共存（spec 决策 4 关键）

```
nx_web_server.py 进程
═══════════════════════════════════════════════════════════════════
NxWebNode (单个 rclpy node, 阶段A 创建)
  ├─ 阶段A 订阅: /dog_state /imu /scan /odom     (主 spin 驱动回调)
  ├─ 阶段A 发布: /cmd_vel /cmd_pose               (HTTP handler 调, 无需 spin)
  └─ 阶段E ActionClient: /navigate_to_pose        (挂在同一 node)
       ├─ send_goal_async → goal_future            (主 spin 或 worker spin 驱动)
       └─ handle.get_result_async → result_future   (同上)

线程模型:
  主线程: HTTPServer.serve_forever
  线程1 (daemon): rclpy.spin(NxWebNode)           ← 持续驱动订阅回调 + 闲置 future 回调
  线程2 (daemon): run_ws asyncio loop             ← WS server
  线程3 (daemon): broadcast_loop                  ← 推 status/slam/frame
  线程4-6 (daemon): NxAiEngine 3 线程              ← 阶段B
  线程X (TaskManager worker): 执行 search_room 时
       └─ RoomSearchOrchestrator.run(task)
            └─ Nav2ActionClient.send_goal_and_wait()
                 ├─ send_goal_async → goal_future
                 ├─ rclpy.spin_until_complete(node, goal_future, timeout=5s)  ← worker 线程临时驱动
                 ├─ handle.get_result_async → result_future
                 └─ rclpy.spin_until_complete(node, result_future, timeout=120s) ← worker 线程临时驱动
═══════════════════════════════════════════════════════════════════
```

**关键约束（Generator 必读，Critic 会核对）**：
1. **spin_until_complete 在 worker 线程跑时，主 spin 仍在跑**——rclpy Humble 允许同一 node 被多线程 spin，回调可能在任一线程执行。**所有回调线程安全**：Nav2ActionClient 内部用 `threading.Lock` 保护 `_current_handle`。
2. **ReentrantCallbackGroup**：ActionClient 构造时必须用 `callback_group=ReentrantCallbackGroup()`，否则与主 spin 的默认 MutuallyExclusiveCallbackGroup 死锁（goal_future 的 done_callback 永远不被调度）。这是 rclpy action client 的标准坑。
3. **spin_until_complete 的 timeout 必须设**（5s/120s），避免 worker 永久阻塞。
4. **不要用 MultiThreadedExecutor**（阶段A 决策 2 继续生效）——spin_until_complete + 主 spin 单线程 executor 足够。

### 8.3 Nav2 不可用时的探测

`Nav2ActionClient.wait_for_server(timeout=2.0)`：
- 返回 True：真 Nav2 或 mock_nav2_action 在线，正常发 goal。
- 返回 False：阶段D 没好 + mock 没起 → `RoomSearchOrchestrator._fail("no_nav")`，ws 推 `phase:FAILED, reason:no_nav`，task 失败。

**verify_stage_e 强制先起 mock_nav2_action 再起 nx_web**，保证 `wait_for_server` 不超时。

---

## 9. WS 协议增量（新增 type，不破坏现有）

### 9.1 新增 `type=search_room`（状态机进度推送）

```jsonc
// 状态机阶段切换 (RoomSearchOrchestrator._phase 推送)
{
  "type": "search_room",
  "data": {
    "mission_id": "a3f8c1",                  // 本次搜索任务 ID (uuid)
    "room": "客厅",                           // 房间名
    "phase": "NAVIGATING",                    // SELECT_ROOM|NAVIGATE|NAVIGATING|ARRIVED|SEARCH|DETECT|REPORT|DONE|FAILED
    "progress": 0.35,                         // [0,1] 进度 (NAVIGATING 时按 distance_remaining, SEARCH 时按 current_wp/total_wp)
    "current_wp": 2,                          // 当前航点序号 (SEARCH/DETECT 时有)
    "total_wp": 8,                            // 总航点数
    "targets_found": 1,                       // 已发现目标数
    "distance_remaining": 1.2,                // 当前 Nav2 goal 剩余距离 (NAVIGATING 时有, 来自 feedback)
    "reason": null,                           // 失败原因 (phase:FAILED 时有: no_room|no_nav|nav_timeout|nav_rejected|nav_aborted|cancelled)
    "timestamp": 1719600000.0                 // unix 时间戳
  }
}
```

**关键**：
- 前端 `panel.html` 的 WS 消息分发（panel.html:382-409）用 `if (data.type === 'status')...else if (data.type === 'slam')...` 链式判断，**未识别的 type 走 else 默认分支**（panel.html 的 else 通常是 console.log 或忽略）。新增 `type=search_room` 不破坏现有 type 的处理，前端 console 可见（开发者 F12 看进度）。
- **不改 panel.html**：用户感知是"任务队列出现 search_room 任务 + 地图上狗在动（Nav2 导航驱动 /odom 变化→broadcast_loop 推 slam）"。状态机进度走 console.log，足够调试用。**阶段F+ 再做前端 UI 面板**。

### 9.2 新增 `type=mission_report`（任务完成报告）

```jsonc
{
  "type": "mission_report",
  "data": {
    "mission_id": "a3f8c1",
    "room": "客厅",
    "status": "completed",                    // completed|failed
    "start_time": 1719600000.0,
    "end_time": 1719600123.4,
    "duration_sec": 123.4,
    "waypoints_total": 8,
    "waypoints_visited": 8,
    "targets_found": 2,
    "detections": [                           // 发现目标列表 (对齐 TargetDetection.msg 字段)
      {
        "class": "person",
        "confidence": 0.87,
        "robot_x": 2.3, "robot_y": 1.5, "robot_yaw": 0.1,   // 发现时狗位姿 (map 坐标)
        "world_x": 3.1, "world_y": 1.4,                      // 目标世界坐标 (阶段B 估算)
        "timestamp": 1719600050.0,
        "wp_index": 3                       // 在第几个航点发现的
      }
    ],
    "area": {"width": 5.0, "height": 4.0, "origin_x": 1.0, "origin_y": 0.5},
    "result_path": ""                         // 报告落盘路径 (阶段E 暂不落盘, 空)
  }
}
```

### 9.3 复用 `type=search`（检测发现增量推送，阶段B 已有）

阶段E 的 `_snapshot_detections` 发现新目标时，**复用阶段B 的 `type=search` 格式**（panel.py:639/646, nx_web_server.py:690/699）：
```jsonc
{"type": "search", "data": {"found": ["person(87%)", "chair(72%)"]}}
```
前端 panel.html 对 `type=search` 的处理（panel.html:405-408）已存在，**零改动**即可显示"搜索发现"。

### 9.4 复用 `type=tasks`（任务队列变化，阶段A 已有）

search_room 任务入队/active/completed 时，TaskManager 的 `ws_broadcast({"type":"tasks","data":...})` 自动推送（阶段A 机制，零改动）。前端任务队列 UI 显示 `search_room` 任务（type 字段值，前端原样显示）。

### 9.5 WS 协议契约核对表（Critic 必查）

| WS type | 阶段A/B 状态 | 阶段E 改动 | 字段是否变 | 前端是否需改 |
|---|---|---|---|---|
| `status` | 阶段A 发 | 不动 | 不变 | 否 |
| `slam` | 阶段A/B 发 | 不动 | 不变 | 否 |
| `frame` | 阶段B 发 | 不动 | 不变 | 否 |
| `tasks` | 阶段A 发 | 自动含 search_room 任务（type 字段新值，但字段结构不变） | 不变 | 否（type 值原样显示） |
| `vlm` | 阶段A/B 发 | sys_prompt 加 search_room 示例（不改 type/vlm 消息结构） | 不变 | 否 |
| `search` | 阶段A/B 发 | 复用（阶段E 检测发现也推这个 type） | 不变 | 否 |
| `follow` | 阶段A 发 | 不动 | 不变 | 否 |
| **`search_room`** | 阶段A/B 无 | **新增** | 新 type | 否（console.log） |
| **`mission_report`** | 阶段A/B 无 | **新增** | 新 type | 否（console.log） |

**结论**：阶段E **零改动前端**，新增 2 个 ws type（前端 console 可见，不影响现有 UI）。

---

## 10. mock 检测注入（让编排状态机能验检测分支）

阶段E 的 `_snapshot_detections` 读阶段B 的 `ai_engine.get_detections_world()`。但 verify_stage_e 要在**没有狗帧**的情况下让检测列表非空，需要 mock 检测注入。

### 10.1 方案：复用阶段B 的 MockFrameGenerator + mock_person.png

阶段B 的 `nx_ai_node.py` 已实现：
- `MockFrameGenerator` 生成 720p 灰底帧 + 贴 `web/static/mock_person.png`（COCO 人物裁图）
- `_video_yolo_loop` 持续 detect mock 帧 → `_latest_dets` 缓存 person 检测
- `get_detections_world(x, y, yaw)` 返回 `[{x,y,class:"person"}]`

**阶段E 复用**：verify_stage_e 启动 nx_web 时设 `GO2W_AI_MOCK_VIDEO=1`（阶段B 的 env），NxAiEngine 自动切 mock 视频，YOLO 检出 person，`_latest_dets` 持续有 person。RoomSearchOrchestrator 的 `_snapshot_detections` 读快照即得 person 检测，mission_report 的 detections 非空。

**关键约束**：
- verify_stage_e 需要 NX 上有 YOLO 模型（engine/onnx/pt 任一）+ `mock_person.png`。无 GPU 时 YOLO 降级 PT（慢但能验）。
- 无 YOLO 时（`_detector=None`），`_latest_dets` 恒空，mission_report 的 detections=[]，targets_found=0——这是 graceful 退化，不算 FAIL（verify 第 6 项标 SKIP）。

### 10.2 备用方案：纯 mock 检测注入（无 YOLO 也能验）

若 NX 上无 YOLO 模型，verify_stage_e 可设 env `GO2W_MOCK_DETECT=person,0.85`，NxAiEngine（阶段B 已有的代码或阶段E 补丁）在 `_video_yolo_loop` 里检测到该 env 时，不调真 YOLO，直接构造 `_latest_dets = [{"class": env_class, "confidence": env_conf, "bbox": [...]}]`。这样 mission_report 的 detections 字段在无 YOLO/GPU 时也能非空。

**推荐**：Generator 在 `nx_ai_node.py` 的 `_video_yolo_loop` 加 `GO2W_MOCK_DETECT` 分支（**这是阶段E 唯一对 nx_ai_node.py 的允许改动**，要 Critic 确认不破坏阶段B 契约）。若 Critic 反对改 nx_ai_node，退回 §10.1 方案（必须有 YOLO+mock_person.png）。

---

## 11. 编排状态机的边界情况与状态处理（Critic 必查）

| 场景 | 期望行为 | 实现位置 |
|---|---|---|
| `config/rooms.yaml` 不存在 | `RoomMap.load` 抛 FileNotFoundError，`_ensure_room_map` catch，ws `phase:FAILED, reason:no_room_map`，task 失败 | RoomSearchOrchestrator._ensure_room_map |
| YAML 格式错（缩进/类型错） | `yaml.YAMLError` catch，ws `phase:FAILED, reason:invalid_yaml` | 同上 |
| 房间名未找到（"搜索厕所"但 YAML 没厕所） | `RoomMap.find` 返回 None，ws `phase:FAILED, reason:no_room` | RoomSearchOrchestrator.run step1 |
| Nav2 action server 不在线（阶段D 没好 + mock 没起） | `wait_for_server(2s)` 超时，ws `phase:FAILED, reason:no_nav`，task 失败（不阻塞 worker） | Nav2ActionClient.wait_for_server |
| Nav2 goal 被拒绝（reject） | `goal_handle.accepted == False`，ws `phase:FAILED, reason:nav_rejected` | Nav2ActionClient.send_goal_and_wait |
| Nav2 导航超时（120s 未到达） | `spin_until_complete` 超时，ws `phase:FAILED, reason:nav_timeout`，调 `handle.cancel_goal_async` | 同上 |
| Nav2 导航 abort（路径规划失败/障碍堵死） | `result.status != 4`，ws `phase:FAILED, reason:nav_aborted, status:N` | 同上 |
| 房间入口到达，但房间内某航点导航失败 | 该航点 ws `phase:FAILED, reason:wp_nav_err`，**整个 search_room 任务失败**（不跳过该航点继续——避免漏搜）。可选 env `GO2W_SKIP_FAILED_WP=1` 让跳过失败航点继续 | RoomSearchOrchestrator.run step5 |
| 搜索中途用户 `e_stop` / `cancel_all` | TaskManager.cancel_all → room_orchestrator.cancel() → `_cancelled=True` + nav.cancel_current()；ws `phase:FAILED, reason:cancelled` | TaskManager.cancel_all + RoomSearchOrchestrator.cancel |
| Nav2 feedback 一直不来（mock 没配 delay） | 不影响——`spin_until_complete` 等 result_future，feedback 是可选的（只用来推 progress） | Nav2ActionClient |
| 阶段B ai_engine=None（阶段A 退化，无视频/YOLO） | `_snapshot_detections` 跳过检测，found_list 恒空，mission_report.detections=[], targets_found=0；状态机其他阶段正常走完 | RoomSearchOrchestrator._snapshot_detections |
| target_classes 过滤后无匹配检测（房间只要 person，但只检出 chair） | detections=[] 空（过滤掉 chair），targets_found=0，不算失败（搜完没目标是正常结果） | 同上 |
| 同时发两个 search_room（用户连点） | TaskManager worker 是串行的，第二个排队等第一个完成；不会并发发 Nav2 goal | TaskManager worker |
| Nav2 导航中用户又发 `/api/move`（手柄控狗抢权） | `/api/move` 发 `/cmd_vel`（阶段A 链路），与 Nav2 的 `/cmd_vel` 冲突——Nav2 会感知到外部速度干预，可能 abort 当前 goal。阶段E 不主动处理（用户手柄干预优先级高于自主导航，符合直觉）。ws 推 `phase:FAILED, reason:nav_aborted` | Nav2 行为，编排被动响应 |
| rooms.yaml 热加载（`/api/reload_rooms`）时正在跑搜索 | reload 只更新 `self._room_map`，不影响进行中的 mission（mission 已拿到 room 对象引用）。下次 search_room 用新地图 | RoomSearchOrchestrator.reload_rooms |
| Nav2 goal 发出后 rclpy shutdown（进程退出） | main finally 调 `room_orchestrator.cancel()` 取消 goal；worker 线程是 daemon，进程退出时自然终止 | main finally |

---

## 12. Anti-AI-slop / 反模式清单（Generator 自查）

- ❌ 不要复活 `src/go2w_orchestrator` 包做独立节点（决策 3，跨进程耦合太重）
- ❌ 不要自己发 `/cmd_vel` 做航点导航（决策 1，必须用 Nav2 action）
- ❌ 不要用 `NavigateThroughPoses` 一次发整条航点链（决策 1，丢失中间停留检测语义）
- ❌ 不要在 `RoomSearchOrchestrator.__init__` 里加载 YAML / 创建 Nav2Client（懒加载，启动快 + Nav2 未就绪不报错）
- ❌ 不要在主 rclpy.spin 线程里发 Nav2 goal（决策 4，worker 线程 spin_until_complete）
- ❌ 不要用 MultiThreadedExecutor（阶段A 决策 2 继续生效）
- ❌ 不要用 MutuallyExclusiveCallbackGroup 给 ActionClient（会死锁，必须 ReentrantCallbackGroup）
- ❌ 不要给 `spin_until_complete` 不设 timeout（会永久阻塞 worker）
- ❌ 不要改 panel.html / map.js（前端无感，新增 ws type 走 console.log）
- ❌ 不要改阶段B 的 nx_ai_node.py（决策红线；§10.2 的 GO2W_MOCK_DETECT 是唯一允许的补丁，要 Critic 确认）
- ❌ 不要新增 ROS2 msg/srv（go2w_interfaces 已齐全，MissionReport 走 WS dict）
- ❌ 不要在 RoomSearchOrchestrator 里直接 import ai.detector（走 ai_engine 注入，阶段A 退化时 ai_engine=None）
- ❌ 不要把房间搜索的进度做成"前端进度条动画"（不改 panel.html，进度走 ws type=search_room 的 console.log）
- ❌ 不要在 mock_nav2_action 里模拟真实物理（简单 delay + feedback 递减即可）
- ❌ 不要让 Nav2Client 跨进程共享 handle（ActionClient 挂在 NxWebNode 上，单进程）
- ❌ 不要在 search_room 任务里阻塞 HTTP handler（任务入队立即返回，worker 异步跑）
- ❌ 不要在 `_snapshot_detections` 里调 `ai_engine._detector.detect(frame)`（阶段B 已在 _video_yolo_loop 持续 detect，这里只读 `_latest_dets` 快照，避免重复推理拖慢搜索）
- ❌ 不要把 rooms.yaml 的 yaw 写成角度（必须弧度，与 ROS REP-103 一致）
- ❌ 不要把 search_area.origin 写成相对 nav_pose（必须 map 绝对坐标，直接喂 planner）
- ❌ 不要在 verify_stage_e 里依赖真 Nav2/SLAM/狗（必须 mock_nav2 + mock 检测全程跑通）

---

## 13. 分 Sprint 实现顺序（每步可独立验证，前 3 步不依赖狗硬件）

### Sprint 1：RoomMap + YAML schema + 房间匹配（不接 Nav2/狗）
**目标**：`RoomMap.load("config/rooms.yaml")` 能加载校验，`find("客厅")` 能匹配出 Room 对象。
**Features**：
- 新建 `config/rooms.yaml`（§6.1 示例，3 个房间占位坐标）
- 新建 `web/nx_room_orchestrator.py` 的 Room / RoomMap 类（load/find/list_rooms/to_dict）
- 单元验证：`python -c "from nx_room_orchestrator import RoomMap; m=RoomMap.load('config/rooms.yaml'); print(m.find('客厅').nav_pose)"` 打印出客厅 pose
- 校验测试：缺字段/数值非法/name 重复 都抛 ValueError
**Definition of Done**：
- `RoomMap.load` 校验规则 6 条全过（§6.2）
- `find("客厅")` / `find("living room")` / `find("搜索客厅")` 都匹配到客厅 Room
- `find("不存在的房间")` 返回 None
- **不依赖狗硬件**：纯 Python 单元测试
- **不依赖 Nav2**：纯数据结构

### Sprint 2：Nav2ActionClient + mock_nav2_action（不接狗，mock 验证 action 链路）
**目标**：`Nav2ActionClient.send_goal_and_wait(x, y, yaw)` 能发 goal 到 mock server 并收到 SUCCEEDED。
**Features**：
- 新建 `web/mock_nav2_action.py`（fake NavigateToPose action server，立即 succeed + feedback）
- `nx_room_orchestrator.py` 加 Nav2ActionClient 类（wait_for_server / send_goal_and_wait / cancel_current）
- 单元验证：起 mock_nav2_action，起一个临时 rclpy node + Nav2ActionClient，发 goal 到 (1,2,0)，等 0.5s 收到 ok=True, status=4
- 失败路径验证：`GO2W_MOCK_NAV_FAIL=1,2` 重启 mock，发 (1,2) 收到 ok=False, reason=aborted
**Definition of Done**：
- mock_nav2_action 启动日志 `/navigate_topose 就绪`
- Nav2ActionClient.send_goal_and_wait 默认 ok=True（mock 立即 succeed）
- `GO2W_MOCK_NAV_FAIL=1,2` 时 (1,2) goal 返回 aborted
- `GO2W_MOCK_NAV_REJECT=1,2` 时 (1,2) goal 返回 rejected
- cancel_current 能中断进行中的 goal（mock 的 `_cancel_cb` ACCEPT）
- **不依赖狗硬件**：mock action server
- **不依赖真 Nav2**：mock 替代

### Sprint 3：RoomSearchOrchestrator 状态机 + search_room 任务类型（不接狗，mock Nav2 + mock 检测验全程）
**目标**：浏览器/curl 发 `/api/search_room?room=客厅`，编排状态机 SELECT_ROOM→NAVIGATE→ARRIVED→SEARCH→REPORT 全程走完，ws 推 type=search_room + type=mission_report。
**Features**：
- `nx_room_orchestrator.py` 加 RoomSearchOrchestrator 类（run/_phase/_fail/_snapshot_detections/cancel）
- `nx_room_orchestrator.py` 加 build_mission_report 函数
- `nx_web_server.py` 改动：import RoomSearchOrchestrator + TaskManager 加 room_orchestrator 属性 + worker 加 search_room 分支 + HTTP `/api/search_room` `/api/rooms` `/api/reload_rooms` + main 注入
- `_fallback_parse` 加 search_room 分支（提取房间名）
- `_vlm_parse_command` 的 sys_prompt 加 search_room 示例
- 端到端验证：起 mock_nav2 + nx_web（GO2W_AI_MOCK_VIDEO=1），curl `/api/search_room?room=客厅`，ws 收到完整状态机推进 + mission_report
**Definition of Done**：
- curl `/api/search_room?room=客厅` → `{"ok":true}`，任务队列出现 search_room 任务
- ws 30 秒内依次收到 phase: SELECT_ROOM → NAVIGATE → NAVIGATING → ARRIVED → SEARCH → DETECT → REPORT → DONE
- ws 收到 type=mission_report，含 room/waypoints_visited/detections 字段
- mock 检测注入（GO2W_MOCK_VIDEO=1 + mock_person.png），detections 含 person
- curl `/api/search_room?room=不存在` → ws phase:FAILED, reason:no_room
- `GO2W_MOCK_NAV_FAIL=2.5,1.8`（客厅入口坐标）后搜客厅 → ws phase:FAILED, reason:nav_aborted
- 搜索中途 `/api/e_stop` → ws phase:FAILED, reason:cancelled
- **不依赖狗硬件**：全程 mock
- **不依赖真 Nav2**：mock_nav2_action

### Sprint 4：verify_stage_e 完整验证脚本 + 边界覆盖（不接狗）
**目标**：`bash web/verify_stage_e.sh` 一键跑通 9 项验证（§9.1），输出 PASS/FAIL。
**Features**：
- 新建 `web/verify_stage_e.py`（python websocket 客户端 + curl 断言）
- 新建 `web/verify_stage_e.sh`（bash wrapper：起 mock_nav2 + nx_web + 跑 verify_stage_e.py + 清理）
- 新建 `docs/room_calibration.md`（标定 SOP）
- 覆盖边界：房间不存在、Nav2 fail、cancel 中途、reload_rooms 热加载
**Definition of Done**：
- `bash web/verify_stage_e.sh` 9/9 PASS（无 YOLO 时第 6 项 SKIP）
- 输出明确的 PASS/FAIL 表格
- 进程清理干净（无僵尸 mock_nav2 / nx_web）
- **不依赖狗硬件**：全程 mock

### Sprint 5（可选，硬件就绪后）：真 Nav2 集成 + 实车标定
**目标**：阶段D Nav2 上线后，kill mock_nav2，编排直连真 Nav2，真房间搜索。
**Features**：无新代码（决策 5 的解耦价值）；用 `docs/room_calibration.md` 标定 rooms.yaml；kill mock_nav2 后编排自动用真 Nav2。
**Definition of Done**：
- NX 上 Nav2 bt_navigator 起，`ros2 action list` 含 `/navigate_to_pose`
- 标定 rooms.yaml 三个房间真实坐标
- 浏览器发"搜索客厅"，狗真走到客厅 + 房间内覆盖搜索 + YOLO 真检测
- mission_report 的 detections 含真场景目标
- **依赖狗硬件**：是
- **依赖阶段D Nav2**：是

---

## 14. 不依赖狗硬件的验证方法（Generator 必须实现并跑通）

### 14.1 一键验证 `web/verify_stage_e.sh`

**前置**：
- 阶段A `verify_nx_web.sh` 8/8 PASS（阶段A 契约不破坏）
- 阶段B `verify_nx_ai.sh` PASS（ai_engine 可用，能读检测快照；无 YOLO 时降级）

**流程**：
```bash
# 1. 起 mock Nav2 action server (背景)
python3 web/mock_nav2_action.py &
MOCK_NAV_PID=$!
sleep 2  # 等 action server 就绪

# 2. 起 nx_web (阶段B AI + 阶段E 编排, mock 视频)
GO2W_AI_MOCK_VIDEO=1 python3 web/nx_web_server.py &
WEB_PID=$!
sleep 5  # 等 web 就绪

# 3. 跑验证
python3 web/verify_stage_e.py
EXIT=$?

# 4. 清理
kill $WEB_PID $MOCK_NAV_PID 2>/dev/null
exit $EXIT
```

### 14.2 验证项（`verify_stage_e.py` 实现，每项独立 PASS/FAIL）

| # | 验证项 | 方法 | 通过标准 |
|---|---|---|---|
| 1 | rooms.yaml 加载校验 | `RoomMap.load('config/rooms.yaml')` | 不抛异常，room_map.rooms 长度 ≥ 3 |
| 2 | search_room 入队 | `curl -X POST '/api/search_room?room=客厅'` | `{"ok":true}`，ws 收到 type=tasks 含 search_room 任务 |
| 3 | 状态机推进（前半） | ws 监听 10s | 依次出现 phase: SELECT_ROOM → NAVIGATE → NAVIGATING → ARRIVED |
| 4 | 状态机推进（后半）+ 报告 | ws 监听 30s | 出现 phase: REPORT/DONE；收到 type=mission_report，data.room="客厅"，data.waypoints_visited ≥ 1 |
| 5 | 检测发现（mock 注入） | ws 监听 30s | 收到 type=search 的 found 非空（如 ["person(85%)"]）；mission_report.detections 非空。**无 YOLO 时 SKIP** |
| 6 | 房间不存在 | `curl '/api/search_room?room=厕所'` | ws phase:FAILED, reason:no_room |
| 7 | Nav2 导航失败 | `GO2W_MOCK_NAV_FAIL=2.5,1.8` 重启 mock，搜客厅 | ws phase:FAILED, reason 含 nav_aborted/nav_err |
| 8 | 中途取消 | 搜客厅中途 `curl -X POST /api/e_stop` | ws phase:FAILED, reason:cancelled |
| 9 | reload_rooms 热加载 | 改 rooms.yaml 加房间，`curl /api/reload_rooms` | `{"ok":true}`，`/api/rooms` 含新房间 |

**通过标准**：1-4、6-9 必须 PASS（8 项），5 条件 PASS（有 YOLO+mock_person.png 才验，否则 SKIP 不算 FAIL）。

### 14.3 websocket 客户端断言实现要点

```python
# verify_stage_e.py 核心逻辑 (python websockets 客户端)
import asyncio, json, websockets

async def collect_phases(room, timeout=30):
    """连 ws://localhost:8001, 收集 type=search_room 的 phase 序列。"""
    phases = []
    async with websockets.connect("ws://localhost:8001") as ws:
        # 并行: 一个 task 发 curl /api/search_room, 一个 task 收 ws 消息
        ...
        async for msg in ws:
            d = json.loads(msg)
            if d.get("type") == "search_room":
                phases.append(d["data"]["phase"])
                if d["data"]["phase"] in ("DONE", "FAILED"):
                    break
    return phases

# 断言: ["SELECT_ROOM","NAVIGATE","NAVIGATING","ARRIVED","SEARCH","DETECT","REPORT","DONE"]
# 是 phases 的子序列 (允许中间穿插, 顺序对即可)
```

### 14.4 跨机验证（PC 浏览器 → NX）

- PC 浏览器开 `http://NX_IP:8000`
- F12 Console，输入指令"搜索客厅"（`/api/command`）或直接 `fetch('/api/search_room?room=客厅',{method:'POST'})`
- Console 看到 `type=search_room` 的 phase 推送序列
- 地图上看到狗位姿变化（Nav2 导航驱动 /odom 变化 → broadcast_loop 推 slam，地图狗箭头移动）
- Console 看到 `type=mission_report` 报告

---

## 15. Evaluation Criteria（见 `gan-harness/eval-rubric-stage-e.md`，权重已定）

详见独立的 `gan-harness/eval-rubric-stage-e.md`，Critic 直接消费。核心五维：
- 阶段A/B 契约不破坏（0.25）：HTTP 12 端点 + WS 现有 type 字段逐字对齐；前端零改动；nx_ai_node 只读不写（除 §10.2 可选补丁）
- Nav2 action 集成正确性（0.25）：标准 NavigateToPose 接口、ReentrantCallbackGroup、spin_until_complete + timeout、worker 线程模型
- 编排状态机完整性（0.20）：六态 + 失败子态全覆盖、cancel 响应、阶段切换 ws 推送、MissionReport 字段
- 可验证性（0.20）：verify_stage_e.sh PASS、mock_nav2_action 行为可配、不依赖狗/真 Nav2
- 房间地图设计（0.10）：YAML schema 完整、校验规则、房间匹配逻辑、热加载
