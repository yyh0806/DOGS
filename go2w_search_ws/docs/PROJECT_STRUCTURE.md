# Go2W 搜索系统 — 项目结构

## 2026-07-14 当前权威架构

本节取代下方历史架构描述。运行时的核心约束是“每种有副作用的资源只有一个所有者”：

```text
PC 语音/浏览器
  -> 带 Bearer Token 的规范 search_room 任务
  -> NX Web / 任务编排
       -> 持久化前沿探索 -> 唯一 NavigationGateway -> 唯一 NavigateToPose action 端口
       -> 时间同步观测 -> YOLO/YOLO-World -> 目标去重 -> 地图标注/照片
  -> Nav2 cmd_vel
  -> 唯一 Go2WMotionMachine -> 唯一 UnitreeSportAdapter -> 唯一 leased SportClient
```

| 责任 | 唯一权威实现 |
|---|---|
| 宇树运动调用 | `src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py` |
| 产品运动状态机 | `motion_types.py` + `motion_machine.py` |
| ROS 运动节点 | `nx_motion_node.py`，回调只入队，单 actor 串行执行 SDK |
| 运动协议 | `motion_protocol.py`：intent v1、feedback v2、status v4，含发布指纹 |
| Nav2 目标所有权 | `web/nx_navigation_gateway.py`，点选和搜索任务共享一个 action 端口 |
| 点选导航实现 | `web/nx_point_nav.py`，含迟到接受/取消隔离与健康门禁 |
| 探索 | `web/nx_frontier_planner.py` + `nx_exploration_manager.py`，按地图 revision 持久化 visited/blacklist/budget |
| 搜索任务协议 | `web/nx_mission_schema.py`，语音、文本和 HTTP 使用同一 schema |
| 感知同步 | `web/nx_observation_sync.py`，以拍摄时间插值位姿并匹配扫描/点云 |
| 目标去重 | `web/nx_person_mission.py`，按 mission/class 去重并保留质量最高的同步位置 |
| API 安全 | `web/nx_control_auth.py`，所有控制 POST 在读 body 前校验 Bearer Token |
| 离线发布门禁 | `tools/verify_release.py` |
| 发布包严格校验 | `tools/verify_release_artifact.py`，校验归档路径、类型、完整清单、hash 与 release ID |
| 部署后只读验收 | `tools/nx_release_probe.py`、`nav2_preflight.py`、`perception_preflight.py`，分别写 safe-park/release、单链 TF/action/双 costmap/障碍桥、开放词汇感知凭证 |
| 地图坐标采集 | `tools/capture_map_pose.py`，只读 `map -> base_link` TF 并生成房间标定片段 |
| 原子发布 | `docker/build_release.sh` + `docker/deploy_release.sh` |

发布目录是 `/home/nx/go2w/releases/<release_id>`，`/home/nx/go2w/current` 通过原子符号链接指向完整不可变 payload。归档显式包含 `ai/` 运行时和部署验收工具；严格校验器会拒绝哈希正确但缺关键文件的包。systemd 的 motion/web/nav/sensor 服务必须从 `current/payload` 启动，并读取 `/etc/go2w/release.env` 与自动探测生成的 `/etc/go2w/hardware.env`。旧的 `deploy_nx*.sh` 和直接覆盖 `~/go2w_ws` 的脚本只作历史排障参考，不再是生产发布入口。

规范搜索任务示例：

```json
{"schema_version":1,"request_id":"voice-001","room":"current_room","target_classes":["person"],"search_strategy":"frontier_explore","require_photos":true,"mark_on_map":true,"max_radius_m":6.0,"max_time_s":480.0}
```

将 `target_classes` 改为 `["dining table"]` 即搜索桌子；任意通过 schema 校验的英文 YOLO-World 类别都走同一条任务、探索、定位、去重和标注链路。VLM 只允许生成一个 `search_room`，加载失败、超时或非法输出均返回带 `parse_error` 的空任务，不会回退生成运动步骤。

> 2026-06-30 重写。反映 **NX 中心化架构**（旧 PC 中心化架构已于本日全部删除）。
> 旧版描述的 `panel.py` 主前端 / `cmd_publisher` / `ros_to_json` / PC go2w_humble 容器 **均已废弃删除**，勿再参考。

## 一、架构（NX 中心化 — 全部重活在载荷 NX）

```
PC (浏览器, 瘦客户端)
  └─ 访问 http://<NX_IP>:8000  (键盘控制 / 地图 / 视频 / 任务队列)
       │ HTTP:8000 + WebSocket:8001 (低频状态/指令, 非实时)
       ▼
载荷 NX (Orin NX 16GB, Humble — 跑所有重活, 本机闭环)
  ├─ nx_web_server.py    主程序 (HTTP+WS, 内嵌 rclpy 订阅/发布本机话题)
  ├─ nx_ai_node.py       VideoClient 取狗帧 + YOLO/VLM (注入 nx_web_server 同进程)
  ├─ nx_sensor_node.py   读狗 IMU/雷达 → /imu /scan /odom + odom→base_link TF
  ├─ nx_motion_node.py   持 lease 控狗 (订阅 /cmd_vel, 崩溃自启夺 lease)
  ├─ slam_toolbox        2D 建图 (消费 /scan + /odom)
  └─ nav2_slim           导航 (消费 /odom + /scan + 地图)
       │ unitree_sdk2py (CycloneDDS, USB 网线 enxc8a362616c4c)
       ▼
狗主控 192.168.123.161 (宇树出厂系统, 只收 SDK 指令, 不跑我们的程序)

[可选支线] 独立 Livox MID360 ──网线──> NX (enP8p1s0) ──> FAST_LIO 3D建图
           (SDK2 + livox_ros_driver2 + FAST_LIO 已编译装在 ~/ws_livox/, 待雷达点亮)
```

**核心原则**：NX 本机闭环（雷达→建图→Nav2→控狗 全本机 ROS2，零跨网延迟）；PC 是瘦客户端（断了狗也能自主）；热点只传低频状态/指令。

## 二、目录结构（删除旧架构后）

### `web/` — NX Web 服务 ✅ active
| 文件 | 用途 |
|------|------|
| `nx_web_server.py` | **NX 主程序**（HTTP+WS+rclpy，端口/契约原样照搬自已删的 panel.py）|
| `nx_ai_node.py` | 视频(YOLO/VLM) 引擎，作为组件注入 nx_web_server 同进程 |
| `nx_gimbal_node.py` | C13 云台 RTSP 双流(可见光+红外)，gst 硬解/ffmpeg 双后端 |
| `nx_lidar_node.py` | Livox MID360 `/livox/lidar` → 2D 鸟瞰 png 桥接 |
| `nx_slam_map.py` | ObstacleGridAccumulator 障碍栅格累积(前端稳定占用图) |
| `nx_room_orchestrator.py` | 房间搜索任务编排 |
| `mock_dog_state_publisher.py` / `mock_nav2_action.py` | NX 无狗/无 Nav2 测试 mock（deploy_nx_web.sh 拷贝）|
| `static/panel.html` | **前端页面**（合同，nx_web_server 服务它）|
| `static/map.js` | 地图 Canvas 渲染（合同）|
| `static/mock_person.png` | mock 视频用的 COCO 人物裁图 |
| `verify_nx_web.sh` / `verify_nx_ai.sh` / `verify_stage_e.sh` | NX 验证脚本 |
| `start_nx_web.sh` / `start_pc_browser.sh` | NX 启动 / PC 开浏览器访问 NX |

### `src/go2w_bridge/` — 狗 SDK 桥 ✅ active（仅 2 节点）
| 文件 | 用途 |
|------|------|
| `nx_sensor_node.py` | 读狗 IMU/雷达 → /imu /scan /odom + TF（**只读不控狗**）|
| `nx_motion_node.py` | 持 lease 控狗（订阅 /cmd_vel，状态机，看门狗）|

### `src/go2w_nav/` — SLAM + Nav2 ✅ active
| 文件 | 用途 |
|------|------|
| `slam.launch.py` + `config/slam_toolbox.yaml` | 2D 建图（mode:=mapping\|localization）|
| `nav2_slim.launch.py` + `config/nav2_params_slim.yaml` | 导航 slim（降级路径，无 amcl/FAST_LIO TF 桥）|
| `nav2_3d.launch.py` + `config/nav2_params_3d.yaml` | 3D 导航（FAST_LIO 路线，备用）|
| `nav2.launch.py` + `config/nav2_params.yaml` | 早期 2D 配置（边界，可后续清理）|

### `ai/` — AI 推理模块 ✅ active（被 nx_ai_node import）
| 文件 | 用途 |
|------|------|
| `config.py` | 全局配置（CUDA/模型路径）|
| `detector.py` | YOLO 检测（yolov8n，可降级 TensorRT）|
| `vlm.py` | Qwen2.5-VL 推理后端；输出由 `nx_ai_node.py` 强制校验成统一搜索任务，失败关闭为空任务 |
| `tracker.py` | 视觉跟踪状态机 |
| `locate_anything.py` | 开放词汇定位(locate-anything.cpp CLI 包装，gguf)，慢路径按需触发 |

### `docker/` — 部署 ✅ active
| 文件 | 用途 |
|------|------|
| `go2w-{motion,sensor,web,slam-nav}.service` | systemd 服务，从 `/home/nx/go2w/current/payload` 启动并读取 release 指纹 |
| `build_release.sh` / `deploy_release.sh` | 当前唯一生产发布入口；内容寻址、显式安全 Token 首次创建、狗/MID360/AI 自动探测、最终 release 前缀构建、主/辅助 unit 安装、MID360 网络/驱动/watchdog 在 Nav2 前显式重启、配置/unit/enable 完整回滚、全量三项只读 probe |
| `deploy_nx.sh` / `deploy_nx_web.sh` / `deploy_nav2_bprime.sh` | 兼容入口，只负责转发到原子 builder/deployer；旧脚本主体不可达 |
| `deploy_fastlio.sh` | Livox MID360 + FAST_LIO 部署（**前提：雷达物理连 NX**）|

### `config/rooms.yaml` — 房间标定坐标

### `docs/` — 文档
SDK_CAPABILITIES（狗能力调研）/ REFACTOR_NX_CENTRIC（架构方向）/ slam_runbook / nav2_3d_runbook / TROUBLESHOOTING / DECISIONS / TECH_DECISIONS / STAGE0_DEBUG_GUIDE / room_calibration

## 三、2026-06-30 已删除的旧 PC 中心化架构

以下均为 "PC 跑重活、跨热点桥接" 旧架构的遗留，新架构零依赖，已删：

- `web/{panel,server,cmd_publisher,ros_to_json}.py` — PC 后端 + 跨热点 /cmd_vel、状态桥
- `web/static/index.html`、`web/run_panel.sh`、`web/start_ros2.sh.legacy` — PC 启动/老前端
- `web/test_{e2e,vlm_commands,vlm_pipeline}.py`、`test_standalone.py` — 旧架构测试
- `src/go2w_bridge/{bridge_node,sport_client,odom_publisher,lidar_publisher,nx_panel_bridge}.py` — PC 直连狗的老桥
- `src/go2w_{orchestrator,detector,web,bringup,interfaces}/` — 整包废弃（被 web/ + ai/ + nx_* 取代）
- `docker/ros_humble.sh`、`audio/` — PC 容器辅助 / 旧语音

**回滚**：`git restore web/ src/ docker/ test_standalone.py audio/`

## 四、功能状态（REFACTOR_NX_CENTRIC 阶段进度）

| 阶段 | 内容 | 状态 |
|------|------|------|
| 0 移动控制 | BalanceStand 修复 | ✅ **实车验证** |
| 1 NX 本机闭环 | nx_web_server + 3 个 systemd service | ✅ 完成（视频/控制通）|
| 2 建图 | slam_toolbox 2D / FAST_LIO 3D | ⏳ odom 缺口已补 + MID360 已点亮(2026-07-01)，**待实车建图验证**|
| 3 Nav2 自主导航 | nav2_slim | ⏳ 待建图完成 |
| 4 AI 迁移 | nx_ai_node（YOLO/VLM） | ✅ 视频通，检测待接 locateanything |

## 五、建图阻塞已解除（2026-07-01 更新）

> 原 P0 两项均已落地代码/硬件，待实车联调验证。

1. ✅ **`/odom` xy 轮速积分已实现**（`nx_sensor_node.py:186-197`，原"xy 占位0"注释已废弃）：
   `motor_state[12-15]` 4 轮 dq 平均 × 轮径(0.065) = 线速度，沿 IMU yaw 积分得 xy。室内硬地够 slam_toolbox。
2. ✅ **MID360 雷达已点亮**（2026-07-01，见长期记忆 `mid360-livox-online`）：
   IP 192.168.1.160，独立 5V/3A 供电，USB-网口 `enx207bd2edf780` 连 NX（host 192.168.1.200/32），`/livox/lidar` 10Hz + `/livox/imu` 正常。

**新待办**：
- wheel odom 轮径(0.065) 待建图后标定；轮足切换/打滑会漂，FAST_LIO 是更精确备选。
- ✅ 雷达 NX 网络已由 `livox-mid360-net.service` 持久化；部署器自动探测或读取 `LIVOX_INTERFACE`，接口不存在时 fail-closed 并由 systemd 重试。

## 六、启动方式

**NX 端**：全量 Nav 模式启用 `go2w-{motion,web,slam-nav}.service`；完整 `go2w-sensor.service` 保留为 fallback 但默认禁用，Nav bringup 只启动受限 `/wheel_odom` 实例。狗与 MID360 USB 网卡均从 `/etc/go2w/hardware.env` 读取。

**PC 端**：浏览器访问 `http://<NX_IP>:8000`（或 `bash web/start_pc_browser.sh`）。

**首次/后续生产部署**：先运行 `python tools/verify_release.py`，再运行 `bash docker/build_release.sh all`，最后在狗放稳且场地安全时执行 `NX_HOST=<IP> NX_USER=nx bash docker/deploy_release.sh dist/<artifact>-all.tar.gz --allow-motion-restart --control-token-file control-token.txt`。
