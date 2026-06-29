# Evaluation Rubric: Go2W 阶段E — 房间级搜索编排

> 消费者: GAN Critic / Verifier
> 对应规格: `gan-harness/spec-stage-e.md`
> 评审范围: `web/nx_room_orchestrator.py` + `web/mock_nav2_action.py` + `config/rooms.yaml` + `web/verify_stage_e.py` + `web/nx_web_server.py` 改动 + 可选 `web/nx_ai_node.py` 补丁
> 评审方式: 静态契约核对 + 动态 verify_stage_e.sh 跑通 + 线程模型审查
> 通过门槛: 0 个 Critical / 0 个 High (GAN 收敛条件)

---

## 0. 总体评审原则

- **不实车**: 全程用 mock_nav2_action + mock 检测验证，不要求狗硬件/真 Nav2/SLAM 在场。
- **不破坏阶段A/B**: HTTP 12 端点 + WS 现有 type 字段 + 前端 panel.html/map.js 零改动是硬约束，破坏即 Critical。
- **标准接口优先**: Nav2 必须用 `nav2_msgs/action/NavigateToPose` action client，自己发 `/cmd_vel` 做航点导航即 Critical。
- **可 mock 验证**: 编排状态机必须能在 mock_nav2 + mock 检测下端到端跑通，否则可验证性维度直接 0 分。

---

## 1. 评分维度与权重

| 维度 | 权重 | 核心问题 |
|---|---|---|
| **D1. 阶段A/B 契约不破坏** | 0.25 | 是否零改动前端、不改 nx_motion/sensor、不破坏 HTTP/WS 现有字段 |
| **D2. Nav2 action 集成正确性** | 0.25 | 是否用标准接口、线程模型、callback group、timeout、cancel |
| **D3. 编排状态机完整性** | 0.20 | 六态 + 失败子态、cancel 响应、阶段推送、MissionReport 字段 |
| **D4. 可验证性** | 0.20 | verify_stage_e.sh 是否 PASS、mock 行为是否可配、是否依赖狗 |
| **D5. 房间地图设计** | 0.10 | YAML schema、校验规则、房间匹配、热加载 |

**综合分 = Σ(维度分 × 权重)，0-1 区间。≥ 0.85 视为通过（GAN 收敛）。**

---

## 2. D1. 阶段A/B 契约不破坏（权重 0.25）

### D1.1 [Critical] 前端零改动
- **检查**：`git diff web/static/panel.html web/static/map.js` 必须为空
- **失败判据**：任何对 panel.html / map.js 的改动 → Critical
- **验证命令**：`git diff --name-only web/static/`

### D1.2 [Critical] HTTP 12 端点不破坏
- **检查**：nx_web_server.py 的 `/` `/index.html` `/map.js` `/api/foxglove` `/api/status` `/api/connect` `/api/stand` `/api/sit` `/api/stop` `/api/e_stop` `/api/move` `/api/command` `/api/search` 这 13 个端点的路径/方法/响应字段**逐字不变**
- **新增端点允许**：`/api/search_room` (POST) / `/api/rooms` (GET) / `/api/reload_rooms` (GET)
- **验证**：阶段A `verify_nx_web.sh` 仍 8/8 PASS（阶段E 改动后回归）
- **失败判据**：任一现有端点字段被改/删 → Critical

### D1.3 [Critical] WS 现有 type 字段不破坏
- **检查**：`status` / `slam` / `frame` / `tasks` / `vlm` / `search` / `follow` 这 7 个 type 的字段名不变
- **新增 type 允许**：`search_room` / `mission_report`
- **验证**：python websocket 客户端连 ws://localhost:8001，5s 内收到 type=status（字段含 imu_yaw/stats/dog_state/tasks）+ type=slam（字段含 x/y/yaw/trail/map/scan/detections/waypoints/currentWP/slam_source）
- **失败判据**：现有 type 字段被改/删 → Critical

### D1.4 [High] 不改 nx_motion_node / nx_sensor_node
- **检查**：`git diff src/go2w_bridge/` 为空
- **失败判据**：改动 → High

### D1.5 [High] 不改 ai/detector.py / ai/vlm.py
- **检查**：`git diff ai/` 为空
- **失败判据**：改动 → High

### D1.6 [Medium] nx_ai_node.py 改动受控
- **检查**：阶段E 允许的唯一 nx_ai_node.py 改动是 §10.2 的 `GO2W_MOCK_DETECT` 分支（mock 检测注入）
- **允许**：`_video_yolo_loop` 加一个 env 判断分支，构造假 `_latest_dets`
- **禁止**：改 VLM/YOLO 加载逻辑、改 ChannelFactory 单例、改 `get_detections_world` 坐标转换公式
- **失败判据**：超出允许范围的改动 → Medium（需 Generator 解释）

### D1.7 [Medium] 不复活 go2w_orchestrator 包
- **检查**：`git diff src/go2w_orchestrator/` 为空（休眠包不动）
- **失败判据**：改动 → Medium

---

## 3. D2. Nav2 action 集成正确性（权重 0.25）

### D2.1 [Critical] 用标准 NavigateToPose action
- **检查**：`nx_room_orchestrator.py` 的 Nav2ActionClient 用 `from nav2_msgs.action import NavigateToPose` + `ActionClient(node, NavigateToPose, '/navigate_to_pose')`
- **禁止**：自己发 `/cmd_vel` 做航点导航（决策 1）
- **失败判据**：非标准接口 → Critical

### D2.2 [Critical] ReentrantCallbackGroup
- **检查**：ActionClient 构造传 `callback_group=ReentrantCallbackGroup()`
- **理由**：MutuallyExclusiveCallbackGroup 会与主 spin 死锁（goal_future 回调永不调度）
- **失败判据**：用默认/MutuallyExclusive 组 → Critical

### D2.3 [High] spin_until_complete 带 timeout
- **检查**：`Nav2ActionClient.send_goal_and_wait` 的 `rclpy.spin_until_complete(node, future, timeout_sec=N)` 必须传 timeout
- **goal 接受 timeout**：5s（与休眠包 orchestrator_node.py:163 wait_for_server(5.0) 一致）
- **导航完成 timeout**：120s（与休眠包 nav_goal_timeout=120.0 一致）
- **失败判据**：无 timeout 或 timeout 不合理 → High（worker 可能永久阻塞）

### D2.4 [High] worker 线程跑 spin_until_complete，不在主 spin 线程发 goal
- **检查**：RoomSearchOrchestrator.run 在 TaskManager worker 线程内调（task_mgr._worker → search_room 分支 → room_orchestrator.run）
- **禁止**：在 rclpy.spin 的回调（如 NxWebNode 的订阅回调或定时器）里发 Nav2 goal
- **失败判据**：发 goal 在错误线程 → High

### D2.5 [High] Nav2Client 线程安全
- **检查**：Nav2ActionClient 内部 `_current_handle` / `_cancelled` 用 `threading.Lock` 保护
- **理由**：cancel_current (HTTP e_stop 线程调) 与 send_goal_and_wait (worker 线程调) 并发
- **失败判据**：无锁保护 → High

### D2.6 [High] yaw → 四元数转换正确
- **检查**：`qz = sin(yaw/2), qw = cos(yaw/2), qx = qy = 0`（休眠包 orchestrator_node.py:176-180 同款）
- **失败判据**：公式错（如 qx/qy 非零，或 sin/cos 反了）→ High（狗朝向错）

### D2.7 [High] STATUS_SUCCEEDED == 4 判定
- **检查**：`result.status == 4` 判成功（休眠包 orchestrator_node.py:207 同款，rclpy action 标准值）
- **失败判据**：用错状态码 → High

### D2.8 [Medium] goal_handle 持有 + cancel_goal_async
- **检查**：goal 接受后存 `_current_handle`，cancel_current 调 `handle.cancel_goal_async()`
- **失败判据**：cancel 不生效 → Medium

### D2.9 [Medium] feedback callback 推 progress
- **检查**：Nav2ActionClient 收到 feedback 时调注入的 `_feedback_callback(distance_remaining, ...)`，RoomSearchOrchestrator 用它推 `type=search_room` 的 `progress` 字段
- **失败判据**：progress 恒 0 或不更新 → Medium

---

## 4. D3. 编排状态机完整性（权重 0.20）

### D3.1 [Critical] 六态全覆盖
- **检查**：RoomSearchOrchestrator.run 推送的 phase 序列含 `SELECT_ROOM` / `NAVIGATE` / `ARRIVED` / `SEARCH` / `DETECT` / `REPORT`
- **允许**：`NAVIGATING`（NAVIGATE 发 goal 后的等待态）作为 NAVIGATE 的细化
- **失败判据**：缺任一核心态 → Critical

### D3.2 [High] 失败子态全覆盖
- **检查**：phase:FAILED 时 reason 含以下之一：`no_room` / `no_nav` / `no_room_map` / `invalid_yaml` / `nav_rejected` / `nav_timeout` / `nav_aborted` / `wp_nav_err` / `cancelled`
- **测试方法**：verify_stage_e 第 6/7/8 项触发 no_room / nav_aborted / cancelled
- **失败判据**：失败时不推 reason 或 reason 字段缺失 → High

### D3.3 [High] cancel 响应正确
- **检查**：搜中途 `/api/e_stop` 或 `/api/stop`，RoomSearchOrchestrator.cancel 被调，`_cancelled=True` + nav.cancel_current()，ws 推 phase:FAILED, reason:cancelled
- **关键**：cancel 检查在每个阶段切换点 + 航点循环内（与 panel.py:619 search_area 取消检查同款）
- **失败判据**：cancel 不生效（搜索继续跑） → High

### D3.4 [High] MissionReport 字段完整
- **检查**：`type=mission_report` 的 data 含：`mission_id` / `room` / `status` / `start_time` / `end_time` / `duration_sec` / `waypoints_total` / `waypoints_visited` / `targets_found` / `detections` / `area`
- **对齐**：`go2w_interfaces/MissionReport.msg` 的字段（mission_id/duration/area/waypoints_visited/targets_found/detections）
- **detections 元素字段**：`class` / `confidence` / `robot_x` / `robot_y` / `robot_yaw` / `timestamp`（对齐 TargetDetection.msg）
- **失败判据**：关键字段缺失 → High

### D3.5 [Medium] 阶段切换 ws 推送
- **检查**：每次 phase 变化都 ws_broadcast `{"type":"search_room","data":{"phase":...}}`
- **失败判据**：漏推某个阶段 → Medium

### D3.6 [Medium] 检测快照读阶段B（不重复推理）
- **检查**：`_snapshot_detections` 调 `ai_engine.get_detections_world(x, y, yaw)` 读快照，**不**调 `ai_engine._detector.detect(frame)`
- **理由**：阶段B 的 `_video_yolo_loop` 持续 detect，重复推理拖慢搜索
- **失败判据**：重复推理 → Medium

### D3.7 [Medium] target_classes 过滤
- **检查**：room.target_classes 非空时，只记录 class 在该列表的检测
- **失败判据**：不过滤 → Medium

### D3.8 [Medium] 任务串行（不并发发 Nav2 goal）
- **检查**：TaskManager worker 是串行的（取一个任务执行到完成再取下一个），同时发两个 search_room 时第二个排队
- **失败判据**：并发发 goal → Medium

---

## 5. D4. 可验证性（权重 0.20）

### D4.1 [Critical] verify_stage_e.sh 跑通
- **检查**：`bash web/verify_stage_e.sh` 输出 9 项验证结果，1-4、6-9 必须 PASS，5 条件 PASS
- **失败判据**：任一必 PASS 项 FAIL → Critical

### D4.2 [Critical] 不依赖狗硬件
- **检查**：verify_stage_e.sh 全程不起 nx_motion_node / nx_sensor_node / 不连狗 SDK
- **失败判据**：验证依赖狗 → Critical

### D4.3 [Critical] 不依赖真 Nav2
- **检查**：verify_stage_e.sh 用 mock_nav2_action 替代真 Nav2
- **失败判据**：验证依赖真 Nav2 → Critical

### D4.4 [High] mock_nav2_action 行为可配
- **检查**：`GO2W_MOCK_NAV_DELAY` / `GO2W_MOCK_NAV_FAIL` / `GO2W_MOCK_NAV_REJECT` 三个 env 都生效
- **测试**：verify_stage_e 第 7 项用 `GO2W_MOCK_NAV_FAIL=2.5,1.8` 触发客厅导航失败
- **失败判据**：env 不生效 → High

### D4.5 [High] mock_nav2_action 发 feedback
- **检查**：mock 收到 goal 后发 `NavigateToPose.Feedback`（含 `distance_remaining`），让编排的 progress 推送有数据
- **失败判据**：不发 feedback → High

### D4.6 [High] 启动顺序强制
- **检查**：verify_stage_e.sh 先起 mock_nav2 再起 nx_web（wait_for_server 不超时）
- **失败判据**：顺序错导致 wait_for_server 超时 → High

### D4.7 [Medium] 进程清理
- **检查**：verify_stage_e.sh 结束时 kill mock_nav2 + nx_web，无僵尸进程
- **失败判据**：残留进程 → Medium

### D4.8 [Medium] 无 YOLO 时 graceful 降级
- **检查**：NX 无 YOLO 模型时，verify_stage_e 第 5 项 SKIP（不算 FAIL），mission_report.detections=[], targets_found=0
- **失败判据**：无 YOLO 时崩溃 → Medium

---

## 6. D5. 房间地图设计（权重 0.10）

### D5.1 [High] YAML schema 完整
- **检查**：`config/rooms.yaml` 含 `frame_id` / `version` / `default_search_spacing` / `default_search_pattern` / `rooms[]`
- **每个 room**：`name` / `aliases` / `nav_pose.{x,y,yaw}` / `search_area.{width,height,origin_x,origin_y,spacing,pattern}` / `target_classes`
- **失败判据**：字段缺失 → High

### D5.2 [High] 校验规则全实现
- **检查**：RoomMap.load 校验 §6.2 的 6 条规则
- **测试**：
  - 缺 nav_pose.x → ValueError
  - width=0 → ValueError
  - name 重复 → ValueError
  - 文件不存在 → FileNotFoundError
  - 格式错 → yaml.YAMLError
- **失败判据**：校验漏一条 → High

### D5.3 [High] 房间匹配逻辑正确
- **检查**：RoomMap.find 的 4 级匹配（name 完全相等 > aliases 完全相等 > name 子串 > aliases 子串）
- **测试**：
  - `find("客厅")` → 客厅 Room
  - `find("living room")` → 客厅 Room（alias）
  - `find("搜索客厅")` → 客厅 Room（子串）
  - `find("厕所")` → None
- **失败判据**：匹配错 → High

### D5.4 [Medium] search_area 坐标是 map 绝对坐标
- **检查**：search_area.origin_x/y 是 map 坐标系绝对值（不是相对 nav_pose），直接喂 `plan_lawnmower(width, height, spacing, origin_x, origin_y)`
- **失败判据**：写成相对坐标 → Medium

### D5.5 [Medium] yaw 是弧度
- **检查**：nav_pose.yaw 是弧度（不是角度）
- **失败判据**：写成角度 → Medium

### D5.6 [Medium] 热加载（reload_rooms）
- **检查**：`/api/reload_rooms` 重新加载 YAML，不影响进行中的 mission
- **测试**：改 YAML 加房间，reload，`/api/rooms` 含新房间
- **失败判据**：reload 不生效或影响进行中 mission → Medium

### D5.7 [Low] 至少 3 个示例房间
- **检查**：rooms.yaml 含至少 3 个房间（客厅/卧室/厨房），覆盖不同 target_classes
- **失败判据**：少于 3 个 → Low

---

## 7. 反模式核对（Generator 自查，Critic 复核）

每条违反按对应级别扣分：

| # | 反模式 | 级别 |
|---|---|---|
| 1 | 复活 go2w_orchestrator 包做独立节点 | Critical |
| 2 | 自己发 `/cmd_vel` 做航点导航（非 Nav2 action） | Critical |
| 3 | 用 NavigateThroughPoses 一次发整条航点链 | Critical |
| 4 | RoomSearchOrchestrator.__init__ 加载 YAML / 创建 Nav2Client（非懒加载） | Medium |
| 5 | 在主 rclpy.spin 线程发 Nav2 goal | High |
| 6 | 用 MultiThreadedExecutor | High |
| 7 | ActionClient 用 MutuallyExclusiveCallbackGroup | Critical |
| 8 | spin_until_complete 不传 timeout | High |
| 9 | 改 panel.html / map.js | Critical |
| 10 | 改 nx_ai_node.py 超出 §10.2 允许范围 | Medium |
| 11 | 新增 ROS2 msg/srv | Medium |
| 12 | RoomSearchOrchestrator 直接 import ai.detector | Medium |
| 13 | search_room 进度做成前端动画（改 panel.html） | Critical |
| 14 | mock_nav2_action 模拟真实物理 | Low |
| 15 | Nav2Client 跨进程共享 handle | High |
| 16 | search_room 任务阻塞 HTTP handler | High |
| 17 | _snapshot_detections 调 detector.detect（重复推理） | Medium |
| 18 | rooms.yaml yaw 写角度 | Medium |
| 19 | search_area.origin 写相对 nav_pose | Medium |
| 20 | verify_stage_e 依赖真 Nav2/SLAM/狗 | Critical |

---

## 8. 评审流程（Critic 执行）

### 8.1 静态契约核对（30 分钟）
1. `git diff --name-only` 列出所有改动文件，核对是否在允许范围内（§7.1/7.2/7.3）
2. `git diff web/static/` 必须为空（D1.1）
3. `git diff src/go2w_bridge/` 必须为空（D1.4）
4. `git diff ai/` 必须为空（D1.5）
5. grep `nx_room_orchestrator.py` 的 `NavigateToPose` / `ReentrantCallbackGroup` / `spin_until_complete` / `timeout_sec`（D2.1/2.2/2.3）
6. grep `nav_pose.yaw` 的弧度使用（D5.5）

### 8.2 动态验证（30 分钟）
1. 在 NX 上跑阶段A `verify_nx_web.sh`（D1.2 回归）
2. 跑阶段B `verify_nx_ai.sh`（D1.6 回归）
3. 跑 `bash web/verify_stage_e.sh`（D4.1 核心）
4. 手动触发边界：no_room / nav_fail / cancel（D3.2/3.3）

### 8.3 线程模型审查（30 分钟）
1. 读 RoomSearchOrchestrator.run，确认在 worker 线程跑（D2.4）
2. 读 Nav2ActionClient，确认 Lock 保护 _current_handle（D2.5）
3. 确认 spin_until_complete 不在主 spin 线程（D2.4）

### 8.4 输出
- 每条 D1.x / D2.x / ... 打分 PASS/FAIL/SKIP + 证据
- 列出所有 Critical / High 问题
- 综合分 + 收敛建议（继续 GAN 协商 or 通过）

---

## 9. 收敛门槛

- **0 Critical + 0 High** → 通过（GAN 收敛）
- **有 Critical** → 必须修复，继续 GAN 协商
- **0 Critical 但有 High** → Generator 解释或修复，Critic 判断是否阻塞
- **综合分 < 0.85** → 继续协商
