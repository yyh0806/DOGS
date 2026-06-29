# Evaluation Rubric: Go2W 阶段A — Web 通信层上移 NX

> Consumer: GAN Critic / Verifier
> Source of truth: `gan-harness/spec.md`（本文件是可执行检查清单）
> 项目根: `go2w_search_ws/`
> 验证前提: 不实车，纯软件；可用 mock_dog_state_publisher 模拟狗状态

## 总分计算

总分 = 契约一致性×0.30 + 链路正确性×0.30 + 可验证性×0.20 + 退役干净度×0.20
每维 0-100 分。**任一 Critical 项 = 0 分则整体不通过（GAN 协商继续）**。

---

## 维度 1: 契约一致性（权重 0.30）— 前端无感切换的硬保证

### Critical（任一失败 = 整体不通过）

- [ ] **C1.1 HTTP API 路径全集**：nx_web_server.py 的 HTTP handler 覆盖 panel.py:656-710 的**全部 12 个端点**：`GET /`、`GET /index.html`、`GET /map.js`、`GET /api/foxglove`、`GET /api/status`、`POST /api/connect`、`POST /api/stand`、`POST /api/sit`、`POST /api/stop`、`POST /api/e_stop`、`POST /api/move`、`POST /api/command`、`POST /api/search`。
  - 验证：grep nx_web_server.py 的 `parsed.path ==` / `p.path ==`，与 spec §5.1 表逐项核对
- [ ] **C1.2 WS 消息 type 字段名**：broadcast_loop 发出的消息 type ∈ {status, slam, tasks, vlm, search}，与 panel.html:382-409 的前端解析分支**一字不差**。
  - 验证：grep nx_web_server.py 的 `"type":`，对照 panel.py:822/830/836/847/857/876
- [ ] **C1.3 WS 端口 = 8001**：前端 panel.html:379 硬编码 `:8001`，NX web 的 WS 必须监听 8001。
  - 验证：grep `8001` / `ws_port`
- [ ] **C1.4 slam 消息字段**：`{"type":"slam","data":{x,y,yaw,trail,map,scan,detections,waypoints,currentWP,slam_source}}` 字段名全匹配 map.js:47-58 的 `update(data)`。
  - 验证：人工核对 broadcast_loop 的 slam dict 构造
- [ ] **C1.5 status 消息字段**：含 `imu_yaw, stats, dog_state, tasks`（panel.html:396-400 解析依赖）。
- [ ] **C1.6 CORS 头**：_json() 响应带 `Access-Control-Allow-Origin: *`（panel.py:714）。
- [ ] **C1.7 /api/move 参数**：query string `vx,vy,vyaw`（panel.html:242/248/317），不是 body JSON。

### High

- [ ] **H1.1 slam_source 值**：NX 模式必须发 `"ros2_nx"`（panel.py:828），让前端地图右上角显示 "SLAM: ros2_nx"。不能发 "dead_reckon"/"mock"。
- [ ] **H1.2 /api/status 响应结构**：`{connected, imu_yaw, stats, tasks}`，不是嵌套在 data 里。
- [ ] **H1.3 /api/search 响应**：`{ok:true, msg:"搜索 ..."}`（panel.py:709）。

### Medium

- [ ] M1.1 trail 采样逻辑：每 0.1m 一个点，上限 2000（panel.py:772-775）。
- [ ] M1.2 scan 转世界坐标公式正确（yaw 旋转，panel.py:815-821）。
- [ ] M1.3 broadcast_loop 频率 ~0.15s（panel.py:839/890）。

---

## 维度 2: 链路正确性（权重 0.30）— ROS2 接线无错

### Critical

- [ ] **C2.1 发布 /cmd_vel 类型 = geometry_msgs/Twist**：linear.x=vx, linear.y=vy, angular.z=vyaw。对照 cmd_publisher.py:70-74。
- [ ] **C2.2 发布 /cmd_pose 类型 = std_msgs/String**：data ∈ {"stand","sit","estop"}。对照 cmd_publisher.py:78-80 + nx_motion_node.py:126。
- [ ] **C2.3 订阅 /dog_state = std_msgs/String(JSON)**：解析 state/vx/vy/vyaw。对照 nx_motion_node.py:246-251 的发布格式。
- [ ] **C2.4 坐标不反转**：NX web 的 /cmd_vel angular.z 直接透传前端 vyaw（正=左转）。反转在 nx_motion_node.py:120 做。**若 NX web 也反转 = 双重反转 bug = Critical**。
  - 验证：grep angular.z 赋值，确认 = vyaw 不是 -vyaw
- [ ] **C2.5 不动 nx_motion_node.py**：`git diff src/go2w_bridge/go2w_bridge/nx_motion_node.py` 必须为空（红线）。
- [ ] **C2.6 不动 panel.html / map.js**：`git diff web/static/` 必须为空（前端无感）。
- [ ] **C2.7 rclpy spin 独立线程**：不能与 HTTPServer.serve_forever 同线程（会阻塞 publish）。
  - 验证：grep `Thread(target=rclpy.spin` 或 `MultiThreadedExecutor`（后者不允许）

### High

- [ ] **H2.1 订阅 /imu yaw 计算正确**：四元数 → yaw，公式 `atan2(2(wz+xy), 1-2(y²+z²))`（ros_to_json.py:52-55）。注意 ROS Imu orientation 顺序 x,y,z,w。
- [ ] **H2.2 订阅缓存线程安全**：rclpy 回调线程写 + broadcast_loop 读，必须 `threading.Lock` 保护（参考 nx_sensor_node.py:82）。
- [ ] **H2.3 publisher 单次创建**：在 node.__init__ 创建一次，handler 只 publish（不重复创建 publisher，否则性能差且可能内存泄漏）。
- [ ] **H2.4 QoS 选择**：/imu /scan 用 `qos_profile_sensor_data`（BEST_EFFORT），与 nx_sensor_node.py:104-107 发布端匹配（订阅端 QoS 不匹配会收不到数据）。
- [ ] **H2.5 不 import ai/audio**：`grep -E "from ai\.|import ai\.|from audio" web/nx_web_server.py` 必须为空（NX web 不跑 AI）。

### Medium

- [ ] M2.1 进程退出顺序：rclpy.shutdown → destroy_node（参考 nx_motion_node.py:263-272）。
- [ ] M2.2 web destroy_node 不调 Damp（web 不控狗，与 nx_motion_node 不同）。
- [ ] M2.3 主线程跑 HTTPServer.serve_forever（阻塞式，与 panel.py:940 一致）。

---

## 维度 3: 可验证性（权重 0.20）— 不依赖狗硬件能跑通

### Critical

- [ ] **C3.1 verify_nx_web.sh 存在且可执行**：`test -x web/verify_nx_web.sh`。
- [ ] **C3.2 8 项验证全 PASS**：Critic 实际在 NX（或 NX 模拟环境）跑 `bash web/verify_nx_web.sh`，输出 8/8 PASS。
  - 若 Critic 无 NX 环境，至少在装了 rclpy 的 Linux 跑 mock + web，验证 curl 部分（1-5）+ websocket 部分（6-8）。
- [ ] **C3.3 mock_dog_state_publisher 能独立跑**：`python3 web/mock_dog_state_publisher.py` 启动后，`ros2 topic list` 含 /dog_state /imu /scan /odom，`ros2 topic hz /dog_state` ≈ 2Hz。
- [ ] **C3.4 不需要狗硬件**：整个验证流程 0 处依赖 unitree_sdk2py / 狗主控 IP / USB 网卡。

### High

- [ ] H3.1 go2w-web.service 能 `systemctl start` 且 active。
- [ ] H3.2 deploy_nx_web.sh scp + 安装 service 流程无报错（可在 mock SSH 环境验）。
- [ ] H3.3 PC 浏览器访问 NX_IP:8000（非 localhost）能加载页面，F12 Console 无报错。

### Medium

- [ ] M3.1 start_nx_web.sh 用 setsid 脱离会话（参考 run_panel.sh:14）。
- [ ] M3.2 日志输出到 /tmp/nx_web.log（参考 run_panel.sh:15）。

---

## 维度 4: 退役干净度（权重 0.20）— 可回滚

### Critical

- [ ] **C4.1 不删源文件**：`git status` 显示 panel.py / cmd_publisher.py / ros_to_json.py 均为 unmodified 或不存在删除标记。
- [ ] **C4.2 PC 不再起 docker 容器**：`grep -r "go2w_humble\|docker exec" web/start_nx_web.sh web/start_pc_browser.sh` 必须为空。
- [ ] **C4.3 NX web 不读 dog_state.json**：`grep "dog_state.json" web/nx_web_server.py` 必须为空（文件桥退役）。

### High

- [ ] H4.1 start_ros2.sh 改名 .legacy 或保留但加 deprecation 注释。
- [ ] H4.2 README "快速开始" 段落更新为 NX web 路径。
- [ ] H4.3 go2w-web.service 的 After=go2w-motion.service（控狗先起）。

### Medium

- [ ] M4.1 nx_web_server.py 文件头 docstring 说明退役了哪些组件（可追溯）。
- [ ] M4.2 mock_dog_state_publisher.py 标注"仅验证用，勿部署到生产 NX"。

---

## 维度 5: Craft / 工程质量（权重 0，但影响 Critic 主观判断）

非硬性，但影响 Critic 是否判 "High" 问题：

- 日志格式与 panel.py 一致（`logging.basicConfig` + `logger = logging.getLogger("go2w.nx_web")`）。
- 错误处理：HTTP handler try/except 不让单次请求崩进程（panel.py 模式）。
- 命名：NxWebNode / NxRobotBridge / broadcast_loop 与 panel.py 风格一致。
- 注释：关键决策（如"为什么不反转坐标"）有 inline 注释指向 spec 决策号。

---

## 协商收敛标准（GAN 循环退出条件）

- 0 个 Critical 未通过
- 0 个 High 未通过（或 High 已有明确修复 plan 且 Critic 接受）
- verify_nx_web.sh 8/8 PASS（Critic 实跑证据）
- git diff 确认 nx_motion_node.py / panel.html / map.js 零改动

未满足 → Critic 退回 Generator 继续协商，无轮次上限。
