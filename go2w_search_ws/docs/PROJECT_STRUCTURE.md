# Go2W 搜索系统 — 项目结构

> 2026-08 重写。本文档以**当前源码**为准（非历史文档），是逐文件状态的权威入口。
> 覆盖范围：`go2w_search_ws/` 全部自研代码；vendored 第三方包（`src/FAST_LIO_ROS2`、`src/livox_ros_driver2`、`src/ros2_livox_simulation`、`src/sophus`）不在此列。

## 0. 权威架构（唯一资源所有者）

运行时的核心约束是"每种有副作用的资源只有一个所有者"，全部落在载荷 NX：

```text
PC 语音(tools/voice_console.py)/浏览器
  -> HTTP POST /api/command | /api/search_room | /api/navigate | WS:8001 反馈
  -> NxWebNode (nx_web_server.py, 唯一 Web 主程序 + 唯一装配者)
       -> 确定性产品命令解析 (nx_product_command) -> LLM 规划 (nx_llm_planner, fail-closed)
       -> TaskManager 单 worker 线程 -> RoomSearchOrchestrator / FetchAction / move 等
       -> 唯一 NavigationArbiter (motion owner: point/tasks/manual)
            -> 唯一 NavigationGateway (owner 互斥) -> PointNavigationController
                 -> Nav2 /navigate_to_pose -> /cmd_vel_nav
  -> 唯一 Go2WMotionMachine (纯策略) -> MotionController (actor 线程)
       -> 唯一 SportGatewayClient -> Unix socket -> 唯一 SportGatewayServer
            -> 唯一 SportClient(enableLease=True) -> 狗主控
```

| 责任 | 唯一权威实现 |
|---|---|
| 宇树运动调用（唯一 lease） | `src/go2w_bridge/go2w_bridge/nx_sport_gateway.py`（独立 systemd 进程，Unix socket `/run/go2w-sport-gateway/sport.sock`） |
| 产品运动状态机 | `motion_types.py` + `motion_machine.py`（纯策略，9 态：BOOT_HOLD/PARKED/ACTIVATING/MANUAL_ACTIVE/NAV_ACTIVE/STOPPING/PARKING/ESTOP/FAULT） |
| 运动编排 | `nx_motion_node.py`，ROS 回调只入队，单 actor 线程串行执行 SDK 效果 |
| 运动协议 | `motion_protocol.py`：intent v1、feedback v2、status v4 |
| 运动安全 | `motion_safety.py`（ScanFreshnessWatchdog / DriveExecutionWatchdog / 蠕变补偿）+ `nx_safety_observer.py`（只读） |
| Nav2 目标所有权 | `web/nx_navigation_gateway.py`（owner: point/mission 互斥）+ `web/nx_navigation_arbiter.py`（运动所有权 point/tasks/manual） |
| 点选导航 | `web/nx_point_nav.py`（迟到接受/取消隔离 + 健康门禁） |
| 任务 schema | `web/nx_mission_schema.py`（SearchMissionRequest，语音/文本/HTTP 同一 schema） |
| 房间编排 | `web/nx_room_orchestrator.py`（三路状态机：SELECT_ROOM 搜索 / frontier_explore / next_best_view） |
| 探索规划 | `web/nx_frontier_planner.py` + `nx_exploration_manager.py` + `nx_visibility_coverage.py` + `nx_global_search_state.py` |
| 感知同步 | `web/nx_observation_sync.py`（按拍摄时间插值位姿匹配扫描/点云） |
| 目标去重/物证 | `web/nx_person_mission.py`（空间 0.7m + 外观余弦 ≥0.90 融合去重，照片/report.json 落盘） |
| 产品命令解析 | `web/nx_product_command.py`（确定性解析 + `resolve_current_room`） |
| API 鉴权 | `web/nx_control_auth.py`（**2026-07-16 起短路为 auth_disabled**，token 校验逻辑为不可达死代码） |
| 原子发布 | `docker/build_release.sh` + `docker/deploy_release.sh` |
| 发布门禁/验收 | `tools/verify_release.py`、`tools/verify_release_artifact.py`、`tools/nx_release_probe.py`、`tools/nav2_preflight.py`、`tools/perception_preflight.py` |

发布目录 `/home/nx/go2w/releases/<release_id>`，`/home/nx/go2w/current` 原子软链指向不可变 payload；systemd 服务从 `current/payload` 启动，读取 `/etc/go2w/release.env` 与 `/etc/go2w/hardware.env`。

规范搜索任务示例：

```json
{"schema_version":1,"request_id":"voice-001","room":"current_room","target_classes":["person"],"search_strategy":"frontier_explore","require_photos":true,"mark_on_map":true,"max_radius_m":6.0,"max_time_s":480.0}
```

---

## 1. systemd 服务全景（NX 常驻，`docker/*.service`）

| 服务 | ExecStart | 职责 |
|---|---|---|
| `livox-mid360-net` | 配置 MID360 网口 IP 192.168.1.200/32 + 路由 | 雷达网络持久化 |
| `livox-mid360-driver` | `msg_MID360_launch.py` | MID360 驱动（/livox/lidar + /livox/imu） |
| `livox-mid360-watchdog` | `tools/livox_stream_watchdog.py` | 雷达数据陈旧重启驱动 |
| `go2w-fastlio` | FAST_LIO | 3D 建图（After livox） |
| `go2w-sport-gateway` | `nx_sport_gateway.py` | **唯一 lease 持有者**，Unix socket 网关，0.25s 空闲零保持 |
| `go2w-motion` | `nx_motion_node.py` | 控狗状态机（Requires sport-gateway） |
| `go2w-sensor` | `nx_sensor_node.py`（`publish_imu:=false publish_scan:=false publish_odom:=true publish_odom_tf:=false odom_topic:=/wheel_odom`，先等狗主控 161 链路就绪） | 读狗传感器（当前只发轮式 odom） |
| `go2w-safety-observer` | `nx_safety_observer.py` | 只读安全快照持久化 |
| `go2w-web` | `web/start_go2w_web.sh` → `nx_web_server.py` | 唯一 Web（HTTP:8000 + WS:8001），After motion |
| `costmap-bridge` | `web/costmap_bridge.py` | 订阅 costmap/plan 降采样写 `/tmp/*.json`（隔离 rclpy wait_set） |
| `go2w-slam-nav`（oneshot） | `docker/bringup_slam_nav2.sh --no-shm` | SLAM/Nav2/探索 bringup；内部 `systemd-run` 起 transient units：fastlio、map-odom-fuser、slam-online、nav2-3d、mid360-nav-bridge、map-padding、nav-health-supervisor |
| `99-go2w-rmem.conf` | sysctl | UDP 缓冲调优（非服务） |

启动顺序依赖：`livox-mid360-net → livox-mid360-driver → go2w-fastlio`；`go2w-sport-gateway → go2w-motion → go2w-web`；`go2w-slam-nav` 独立 bringup。

---

## 2. 目录结构

### `web/` — Web 服务 + 搜索栈（全部注入 `nx_web_server.py` 同进程，除标注独立进程）

**入口与独立进程**

| 文件 | 类型 | 职责 |
|---|---|---|
| `nx_web_server.py` | 主进程（systemd go2w-web） | HTTP:8000 + WS:8001 + 内嵌 rclpy；TaskManager 任务队列；NxRobotBridge 控狗桥；装配所有组件 |
| `nx_c13_image_node.py` | 独立进程 | C13 云台相机（`/c13/image_raw` + `/c13/camera_info`，bringup_livo.sh 启动） |
| `costmap_bridge.py` | 独立进程 | costmap/plan → /tmp JSON |
| `mock_dog_state_publisher.py` / `mock_nav2_action.py` | 仅验证 | 无狗 mock（发 `/dog_state /imu /scan /odom` / fake Nav2 action），勿部署 |
| `voice_command.py` / `verify_voice_search.py` | PC 端入口 | PC 端 NLU 验证/入口（发 `/api/search_room`），与 `tools/voice_console.py` 链路并存 |
| `start_go2w_web.sh` | 启动脚本 | go2w-web wrapper（剥离 ws_livox LD_LIBRARY_PATH 解锁 NVDEC 硬解） |
| `start_pc_browser.sh` | 启动脚本 | PC 端提示浏览器打开 NX |

**导航链路模块**

| 文件 | 职责 |
|---|---|
| `nx_navigation_arbiter.py` | 运动所有权仲裁（point/tasks/manual），drain→activate→设 owner；emergency_stop 独立锁 |
| `nx_navigation_gateway.py` | 唯一 Nav2 目标端口（owner: point/mission 互斥），含路径预检 |
| `nx_point_nav.py` | 点选导航控制器（Nav2 action client） |
| `nx_move_executor.py` | move_relative 执行（angular 闭环 / linear 分段 fail-closed） |
| `nx_motion_intent.py` | 运动意图 envelope（start_manual/start_nav/park/estop/clear_estop） |
| `nx_slam_map.py` | ObstacleGridAccumulator 障碍栅格累积（前端稳定占用图） |

**任务编排模块**

| 文件 | 职责 |
|---|---|
| `nx_mission_schema.py` | SearchMissionRequest（frozen dataclass）+ 校验 + legacy 迁移 + 任务规范化 |
| `nx_room_orchestrator.py` | 房间搜索三路状态机 + mission_report 落盘 |
| `nx_active_search.py` | 产品搜索 next-best-view 规划（信息增益 + 视锥覆盖） |
| `nx_exploration_manager.py` | frontier 探索周期管理（候选选择/失败记忆/blacklist） |
| `nx_frontier_planner.py` | 前沿候选提取（地图 + 视觉覆盖 + lidar） |
| `nx_visibility_coverage.py` | 视锥可见覆盖跟踪 |
| `nx_global_search_state.py` | 全局搜索状态分析（门禁推断/穷尽确认） |
| `nx_coverage_metrics.py` | ROI 覆盖率计算（REPORT 阶段验证） |
| `nx_person_mission.py` | 目标物证：去重/照片/report.json + mission store |
| `nx_person_localizer.py` | 检测 + Lidar 距离 → 世界坐标 |
| `nx_fetch_action.py` / `nx_action_plugin.py` | 取物插件（S1-S5 状态机）/ 插件意图解析注册表 |
| `nx_product_command.py` | 确定性产品命令解析（房间/人搜索 + move） |
| `nx_llm_planner.py` | DeepSeek propose-verify 规划（动作白名单，≤4 步，fail-closed） |
| `nx_landmarks.py` | 地标管理（landmarks.yaml 持久化，LLM prompt 动态注入） |

**感知/媒体模块（注入组件）**

| 文件 | 职责 |
|---|---|
| `nx_ai_node.py` | AI 引擎（YOLO detector / VLM / locate-anything / cloud LLM，3 daemon 线程，懒加载） |
| `nx_gimbal_node.py` | C13 云台 RTSP 双流（可见光+红外，gst/ffmpeg 双后端） |
| `nx_lidar_node.py` | `/mid360/points_nav` → 2D 鸟瞰 png（真机自建独立 rclpy context） |
| `nx_sim_video_node.py` | 仿真视频（GO2W_SIM 分支） |
| `nx_observation_sync.py` | 时间同步观测（拍摄时刻位姿插值） |
| `nx_camera_calibration.py` | 相机标定参数 |

**前端与测试**

| 路径 | 说明 |
|---|---|
| `static/panel.html` + `static/map.js` | 前端页面 + 地图 Canvas 渲染（ws_latest 背压：reliable FIFO 256 + stream 最新值） |
| `static/locate_anything_demo*.png` / `mock_person.png` | 静态资源 |
| `missions/large-room-16/` | 任务物证落盘示例 |
| `scripts/` | verify_nx_web.sh / verify_nx_ai.sh / verify_stage_e.sh / verify_product_room_person_search.sh / diagnose_*.sh |
| `tests/` | web 契约测试 |

### `src/go2w_bridge/` — 运动控制层（ROS2 包）

| 文件 | 职责 |
|---|---|
| `go2w_bridge/nx_motion_node.py` | ROS 适配器：回调只 enqueue，actor 线程独占 SDK 效果 |
| `go2w_bridge/nx_sensor_node.py` | 读狗 lowstate/sportmodestate → `/imu /odom /scan /wheel_feedback`（20Hz）+ TF |
| `go2w_bridge/nx_sport_gateway.py` | **唯一 lease** SportClient + MotionSwitcherClient，Unix socket 服务 |
| `go2w_bridge/nx_safety_observer.py` | 只读安全快照 → SafetyEventRecorder 持久化 |
| `go2w_bridge/sport_gateway_server.py` | 网关服务器（0.25s 空闲零保持、断连持续零、单 owner 409） |
| `go2w_bridge/sport_gateway_client.py` | 运动侧同步 adapter（失败不重放，599 transport 错误） |
| `go2w_bridge/sport_gateway_protocol.py` | JSON-lines 本地协议（version 1，6 操作白名单） |
| `go2w_bridge/motion_machine.py` | **纯策略状态机**（9 态，Effect 列表输出） |
| `go2w_bridge/motion_controller.py` | actor 线程编排（clamp 限速、nav 反向禁行、命令超时归零） |
| `go2w_bridge/motion_protocol.py` | 意图/状态/反馈 schema + legacy 兼容 |
| `go2w_bridge/motion_safety.py` | ScanFreshnessWatchdog / DriveExecutionWatchdog / 蠕变补偿 |
| `go2w_bridge/motion_types.py` | Effect/Telemetry/SessionState 等领域值 |
| `go2w_bridge/map_odom_fuser.py` | FAST_LIO 位姿主干 → `/odom /localization_pose` + TF（wheel odom 仅诊断） |
| `go2w_bridge/map_padding_bridge.py` | SLAM 网格 → Nav2 网格（2m padding + 动态障碍清除）；同文件带 MapMarginGate CLI |
| `go2w_bridge/mid360_nav_bridge.py` | MID360 → `/scan_mid360` + `/mid360/points_nav`（安全扫描，障碍源） |
| `go2w_bridge/safety_event_recorder.py` | 旋转 JSONL 事件记录（4MB×4、指纹去重、critical fsync） |
| `go2w_bridge/build_info.py` | 构建信息 |
| `test/` | 13 个测试文件：状态机/控制器/安全/协议/网关契约/集成（详见 §5） |

### `src/go2w_nav/` — Nav2 + SLAM

| 文件 | 职责 |
|---|---|
| `launch/nav2_3d.launch.py` + `config/nav2_params_3d.yaml` | **真机主力**（隔离自主速度通道） |
| `launch/nav2_slim.launch.py` + `config/nav2_params_slim.yaml` | 阶段 F 降级路径 |
| `launch/slam_online.launch.py` + `config/slam_toolbox_online.yaml` | 持久在线 SLAM → `/map_frontier_raw` |
| `launch/slam.launch.py` / `nav2.launch.py` | 早期 2D 建图 / 早期 Nav2（边界，可清理） |
| `src/mid360_nav_bridge.cpp` | 校准 `/scan_mid360` 滤波（唯一 owner，Python 版是 sim/fallback） |
| `behavior_trees/navigate_{to_pose,through_poses}_dynamic_safe.xml` | 动态安全 BT |
| `config/fastlio_low_latency/mid360.yaml`、`c13_intrinsic.yaml` | 参数 |

### `src/go2w_sim/` — 仿真包

| 文件 | 职责 |
|---|---|
| `launch/sim_full_bringup.launch.py` | 真机一致全栈（fastlio+Nav2+motion+telemetry+web:8000；WSL2 SIGFPE workaround） |
| `launch/sim_fastlio_bringup.launch.py` | 建图验证（`GO2W_NO_GAZEBO=1` 纯 mock） |
| `launch/sim_nav_bringup.launch.py` | Nav2 点选闭环 |
| `launch/sim_spawn_only.launch.py` | 仅 spawn 场景 |
| `go2w_sim/nodes/sim_sport_gateway.py` | GO2W_SIM=1 时代替 SportGatewayClient |
| `go2w_sim/nodes/mock_planar_move_node.py` 等 7 节点 | 运动学/scan/telemetry/odom mock |
| `worlds/`（indoor_empty/indoor_rooms/warehouse）+ `urdf/` | Gazebo 场景与 URDF |

### `ai/` — AI 推理模块

| 文件 | 职责 |
|---|---|
| `config.py` | 全局配置（CUDA/YOLO/VLM 路径双份 PC+NX/音频），全部 `GO2W_*` 可覆盖 |
| `detector.py` | YOLOv8 闭集 / YOLO-World 开放词汇双模式 |
| `vlm.py` | Qwen2.5-VL-3B 视觉定位（locate/track_target） |
| `locate_anything.py` | locate-anything.cpp CLI 包装（中文目标归一化） |
| `tracker.py` | 目标跟踪状态机 IDLE→SEARCHING→TRACKING→RECOVERING |
| `cloud_llm.py` | DeepSeek 云端 LLM（fail-closed） |
| `voice.py` | **遗留**：Audio-Interaction 语音方案，无生产调用方，import 即 NameError（引用 config 未定义的 VOICE_MODEL_NAME/VOICE_QUANT） |

### `tools/` — 运维工具（60+ 脚本）

| 类别 | 代表文件 | 用途 |
|---|---|---|
| 诊断（只读） | `diag_fastlio_map.py` `diag_livox_raw.py` `diag_mid360_filter.py` `diag_nav2_plan.py` `diag_occupancy_layers.py` `diag_scan_tf.py` `diag_sport_state.py` `diag_wheel_dq.py` `diag_frontier_preflight.py` | 人工排障：话题频率/TF/滤波链/plan 审计 |
| 安全门控 | `fastlio_latency_gate.py` `topic_rate_gate.py` `nav_health_supervisor.py` `nav_health_gate.py` `livox_stream_watchdog.py` `tf_static_intruder_watch.py` `sport_gateway_bootstrap_preflight.py` | fail-closed 启动门/守护（bringup 与 systemd 引用） |
| 发布验证 | `verify_release.py` `verify_release_artifact.py` `nav2_preflight.py` `nav2_benchmark.py` `perception_preflight.py` `nx_release_probe.py` `search_validate_after_charge.py` | 离线门禁/部署后只读验收 |
| 语音 | `voice_console.py` | PC 端 Vosk STT + 确定性解析 + 可选本地 LLM → NX `/api/command` |
| 运维 | `capture_map_pose.py` `publish_pose_once.py` `nav_shuttle.py` `monitor_obstacle_nav.py` `wait_lifecycle_active.py` `gen_mock_person.py` `generate_control_token.py` `kill_sim.sh` `restart_sim.sh` `retry_navigate.sh` `fix_amcl_navigate.sh` | 标定/穿梭/监控/仿真管理 |
| 测试 | `test_*.py`（round2/stage_e/voice_gate_follow 等）+ `conftest.py` | 脚本式验证（conftest 让 pytest 跳过模块级 print+sys.exit 的脚本） |

### `docker/` — 部署

| 文件 | 职责 |
|---|---|
| `*.service` / `*.conf` | 12 个 systemd 单元（见 §1） |
| `build_release.sh` | 原子构建（SUBSYSTEM=all/nav/motion/web/ai），打包 gate/preflight |
| `deploy_release.sh` | 原子部署 + preflight 门 + 完整回滚 |
| `bringup_slam_nav2.sh` / `bringup_livo.sh` | NX bringup（transient units 托管） |
| `deploy_nx.sh` / `deploy_nx_web.sh` / `deploy_nav2_bprime.sh` | **已 retired**：旧部署双轨，仅历史排障参考 |
| `deploy_fastlio.sh` / `deploy_fastlivo2.sh` / `deploy_nx_ai.sh` / `prepare_fastlio_low_latency.sh` | 专项部署/准备 |

### `config/rooms.yaml` — 房间标定坐标

`frame_id: map`、`default_search_spacing: 2.5`、`default_search_pattern: lawnmower`。3 房间均为**占位坐标**且 `calibrated: false`（阻塞向占位坐标发导航目标）：客厅 5×4m（spacing 1.5）、卧室 4×3.5m（spacing 1.2，`target_classes:["person"]`）、厨房 3×3m（spacing 1.0，`pattern: spiral`）。标定流程：遥控记录 /odom → 填 nav_pose → 估算矩形 → `curl POST /api/reload_rooms` 热加载 → `/api/search_room?room=客厅` 验证。

---

## 3. 指令 → 控狗链路

```
语音/前端 -> /api/command?text= 或 /api/search_room (SearchMissionRequest JSON)
  -> _parse_product_command 确定性解析链（顺序）：
     1) nx_product_command.parse_product_command（房间/人搜索 + move）
     2) nx_action_plugin.parse_plugin_intent（插件链，fetch）
     3) parse_go_landmark（"去大门"）
     4) parse_follow_command（"跟着穿黑衣服的人"）
     5) LLMPlanner.plan（DeepSeek propose-verify，动作白名单 search_room/move_relative/go_landmark/follow/fetch，
                        ≤4 步，任一步非法整体拒绝 fail-closed，透出 type=llm_reasoning）
  -> canonicalize_move_tasks / canonicalize_search_tasks -> TaskManager.add_list/add_plan
  -> TaskManager._worker 单 worker 线程分发：
     move/navigate -> robot.move          stop -> stop+clear
     follow -> _execute_follow            search_area -> _execute_search
     search_room -> RoomSearchOrchestrator.run（三路状态机）
     go_landmark -> _execute_go_landmark  fetch -> FetchActionPlugin.execute（S1-S5）
  -> MissionNavigationPort.send_goal_and_wait（owner="mission"，超时默认 90s，motion_unhealthy 调 arbiter.recover_task_motion）
     -> NavigationGateway.submit（owner 互斥 navigation_owner_busy）
     -> PointNavigationController.submit -> Nav2 /navigate_to_pose（路径预检 /compute_path_to_pose）
  -> NavigationArbiter（drain 旧 owner -> _activate_drive("nav"/"manual") -> 设新 owner）
  -> NxRobotBridge.move -> /cmd_vel（manual）| _navigate_blocking（task 导航端口）
  -> nx_motion_node（消费 /cmd_vel /cmd_vel_nav /motion_session /cmd_pose）
  -> Go2WMotionMachine -> MotionController -> SportGatewayClient -> SportGatewayServer -> 狗主控
```

**搜索策略两分支**（RoomSearchOrchestrator.run）：
- `frontier_explore`（当前房间无预建图探索）：INIT_SLAM →（FRONTIER_DETECT → NAVIGATING → DETECT）* → REPORT；哨兵房间 `__frontier__`
- `next_best_view`（房间产品搜索）：NAVIGATE → NAVIGATING → ARRIVED → NEXT_BEST_VIEW → DETECT → REPORT；ActiveSearchPlanner 视锥覆盖评分
- 其他（指定房间）：SELECT_ROOM → NAVIGATE → NAVIGATING → ARRIVED → SEARCH(逐航点) → DETECT → REPORT

---

## 4. Web 接口（HTTP:8000 / WS:8001）

**GET**：`/` `/index.html` `/map.js` `/missions/*`（物证 jpg/json）`/api/detection_snapshot` `/api/video_frame` `/api/foxglove` `/api/status` `/api/version` `/api/rooms` `/api/reload_rooms` `/api/landmarks` `/api/reload_landmarks`

**POST**（控制类，全部经 `authorize_request`——当前恒放行）：

| 路由 | 行为 |
|---|---|
| `/api/connect|stand|confirm_stand|balance|adopt_stand|confirm_balance|sit|reset_drive_fault` | `arbiter.run_operator_action` |
| `/api/activate` | start_drive_session("nav") + wait_drive_ready |
| `/api/manual_stop|stop` | release_manual / stop_all |
| `/api/e_stop` | `arbiter.emergency_stop`（独立锁） |
| `/api/fetch_confirm` | fetch 插件 S4 装载确认 |
| `/api/clear_all` | 清轨迹 + costmap clear（subprocess ros2 CLI） |
| `/api/move?vx=&vy=&vyaw=` | 手动移动（上限 0.4/0.3/0.5，全零=释放） |
| `/api/landmarks` | 注册/更新地标 |
| `/api/command?text=` | task_mgr.submit_command（202/409） |
| `/api/navigate` | 点选导航（定位健康 + 半径 ≤20.5m 预检 → arbiter.start_point_goal） |
| `/api/locate?target=` | locate-anything，WS 推 type=locate |
| `/api/search` | 地图选区搜索（Task search_area） |
| `/api/search_room` | SearchMissionRequest.from_api_payload → Task search_room |

**WS 消息**：stream 类（可替换快照）`gimbal / lidar / slam / costmap / costmap_global / occupancy_map / plan / detections / status / frame`；reliable 类（FIFO 256，满则 code 1013 断连重连）`tasks / vlm / mission_report / search_room / search / person_found / locate / follow / go_landmark / move_result / fetch / llm_reasoning`。

---

## 5. 测试覆盖

| 范围 | 位置 | 内容 |
|---|---|---|
| 运动状态机核心 | `src/go2w_bridge/test/test_motion_machine.py`（17KB） | boot 采纳/编码器噪声/parked_state_lost/激活/handoff/ESTOP/超时/epoch 重启/授权不变量 |
| 控制器/安全/协议 | `test_motion_controller.py` `test_motion_safety.py` `test_motion_protocol.py` `test_motion_types.py` | 回调不直触 SDK/clamp/scan 门/蠕变补偿/schema 往返 |
| 网关 | `test_sport_gateway_{server,client,protocol}.py` `test_gateway_motion_restart_integration.py` | 断连持续零/单 owner 409/失败不重放/租约跨重启 |
| 源码契约 | `test_motion_node_v2_contract.py` `test_nx_sport_gateway_contract.py` `test_sensor_feedback_v2.py` | AST/文本断言架构不变量 |
| web 契约 | `web/tests/` | HTTP/WS/前端契约 |

**已知缺口**：`map_odom_fuser.py` / `map_padding_bridge.py` / `mid360_nav_bridge.py` 三个建图节点**无任何测试**；`nx_sensor_node.py` 仅 AST 契约测试无运行时测试；`test_motion_node_v2_contract.py` 仍 import 已删除的 `unitree_sport_adapter`（会导致该契约测试失败）。

---

## 6. 已知遗留 / 死代码 / 待清理

| 项 | 位置 | 说明 |
|---|---|---|
| `ai/voice.py` import 即 NameError | `ai/voice.py:16-17` | 引用 config 未定义 VOICE_MODEL_NAME/VOICE_QUANT；无生产调用方 |
| 鉴权死代码 | `web/nx_control_auth.py:54-70` | auth_disabled 短路后的 token 校验不可达 |
| 契约测试死引用 | `test_motion_node_v2_contract.py:139` | import 已删的 unitree_sport_adapter |
| 死函数 | `motion_safety.py:39 battery_allows_drive`、`:321 observe_low_state`、`:264 nav_must_stop`（仅测试用） | 无运行时调用者 |
| 规划器三处重复 | `nx_web_server.plan_lawnmower/plan_spiral`、`nx_room_orchestrator._fallback_planner`、`nx_active_search` 栅格 | 需统一 |
| 循环依赖 | `nx_room_orchestrator.py:3131` | 懒 import nx_web_server 规避顶层环 |
| 双导入样板 | `go2w_bridge` 10 处 | try/except 双 import 重复 |
| 自屏蔽 footprint 不一致 | sensor `(-0.25,0.30,-0.20,0.20)` vs mid360 `(-0.45,0.45,-0.32,0.32)` | 需跨包核对 |
| sim launch 引用不存在 map | `sim_full_bringup.launch.py:141`、`sim_nav_bringup.launch.py:78` | maps/warehouse_map.yaml 等文件不存在 |
| Nav2 配置三份并存 | `nav2_params{,_3d,_slim}.yaml` + 3 个 launch 变体 | 早期版可清理 |
| 旧部署脚本 | `deploy_nx.sh` / `deploy_nx_web.sh` / `deploy_nav2_bprime.sh` | retired，仅转发/不可达 |
| `setup_jetson.sh` / `hardware/SETUP_GUIDE.md` | 旧栈（JetPack 4.6 + Galactic + CycloneDDS） | 与当前 humble + FastDDS + systemd 原子发布脱节 |

> 详细优化方案见 [`OPTIMIZATION_PLAN.md`](OPTIMIZATION_PLAN.md)。
