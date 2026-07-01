# Go2W 搜索系统 — 项目结构

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
| `vlm.py` | Qwen2.5-VL 指令解析（加载失败降级关键词）|
| `tracker.py` | 视觉跟踪状态机 |
| `locate_anything.py` | 开放词汇定位(locate-anything.cpp CLI 包装，gguf)，慢路径按需触发 |

### `docker/` — 部署 ✅ active
| 文件 | 用途 |
|------|------|
| `go2w-{motion,sensor,web}.service` | systemd 服务（崩溃自启；含 ExecStartPre 等网卡就绪）|
| `deploy_nx.sh` / `deploy_nx_web.sh` / `deploy_nx_ai.sh` | NX 分阶段部署脚本 |
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
- 雷达 NX 网络 `ip addr/route add` 重启丢失，待写 systemd oneshot 持久化。

## 六、启动方式

**NX 端**：`go2w-{motion,sensor,web}.service` 已 enabled，开机自启（ExecStartPre 自动等 USB 网卡就绪）。

**PC 端**：浏览器访问 `http://<NX_IP>:8000`（或 `bash web/start_pc_browser.sh`）。

**首次部署新 NX**：`NX_HOST=<IP> bash docker/deploy_nx.sh` → `deploy_nx_web.sh` → `deploy_nx_ai.sh`。
