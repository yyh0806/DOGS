# Go2W 搜索系统 — 项目结构

> 基于 2026-06-26 实际代码梳理。标注每个模块**当前是否在用**，区分"活跃代码"与"历史遗留"。

## 一、系统运行架构（当前实际链路）

```
┌── PC (Ubuntu 20.04) ──────────────────────────────────────────┐
│  web/panel.py (主前端, HTTP:8000 / WS:8001)                   │
│    ├─ RosRobotBridge → docker exec go2w_humble/cmd_publisher   │
│    ├─ ai/detector.py (YOLO 检测, 给视频画框)                    │
│    └─ ai/vlm.py (Qwen 指令解析, 加载失败则降级关键词匹配)       │
│                                                                │
│  Docker: go2w_humble 容器 (--net=host)                         │
│    ├─ cmd_publisher.py (stdin←panel → 发 /cmd_vel /cmd_pose)   │
│    └─ ros_to_json.py (订阅 NX 话题 → 写 dog_state.json)        │
└────────────────────────┬───────────────────────────────────────┘
                         │ ROS2 Humble DDS (手机热点 192.168.43.0/24)
                         ▼
┌── 载荷 NX (Orin NX, 22.04, Humble) ───────────────────────────┐
│  go2w-motion.service (systemd, 崩溃自启)                       │
│    └─ nx_motion_node.py (订阅 /cmd_vel → 持 lease 控狗)        │
│  nx_sensor_node.py (读狗 IMU/雷达 → 发 /imu /scan /odom)       │
└────────────────────────┬───────────────────────────────────────┘
                         │ unitree SDK (CycloneDDS, USB网线直连)
                         ▼
                   狗主控 192.168.123.161
```

**核心要点**：当前活跃的只有 `web/`（PC）+ `go2w_bridge` 的两个 NX 节点 + `ai/`。`src/` 下其余 5 个 ROS2 包是历史遗留，未在运行链路中。

---

## 二、目录结构

### `web/` — PC 端 Web 服务（当前主程序）✅ 在用

| 文件 | 用途 | 状态 |
|------|------|------|
| `panel.py` | **主前端后端**。HTTP+WS 服务，ROS2 模式用 RosRobotBridge 控狗，import detector/vlm/tracker | ✅ 在用 |
| `static/panel.html` | **当前前端页面**（键盘控制、地图、视频、任务队列）。访问 `/` 实际返回它 | ✅ 在用 |
| `static/map.js` | 地图 Canvas 渲染（狗位姿、轨迹、雷达点、选区） | ✅ 在用 |
| `cmd_publisher.py` | 容器内常驻进程，stdin 收 JSON → 发 /cmd_vel /cmd_pose | ✅ 在用 |
| `ros_to_json.py` | 容器内桥接，订阅 /imu /scan /odom → 写 dog_state.json | ✅ 在用 |
| `run_panel.sh` | 后台启动 panel.py（setsid 脱离会话） | ✅ 在用 |
| `start_ros2.sh` | 一键启动：起容器 + 桥接 + panel | ✅ 在用 |
| `server.py` | 老的单体后端（50KB，panel.py 的前身） | ❌ 废弃 |
| `static/index.html` | 老前端（server.py 用的） | ❌ 废弃 |
| `test_*.py` | VLM/端到端测试 | 测试 |
| `dog_state.json` | ros_to_json 实时写的数据（gitignore，不入库） | 运行时 |

### `ai/` — AI 推理模块（被 panel.py import）✅ 在用

| 文件 | 用途 | 模型 | 状态 |
|------|------|------|------|
| `config.py` | 全局配置（CUDA、模型路径、音频参数） | — | ✅ 在用 |
| `detector.py` | 目标检测 | YOLOv8 (yolov8n.pt) | ✅ 在用 |
| `vlm.py` | 视觉语言引擎（指令解析/定位） | Qwen2.5-VL-3B | ✅ 在用（加载失败降级关键词） |
| `tracker.py` | 视觉跟踪状态机（IDLE→SEARCHING→TRACKING） | 调 VLM.locate | ✅ 在用 |
| `voice.py` | 语音理解 | — | ❌ 损坏（引用了 config 不存在的变量） |

### `src/go2w_bridge/` — 狗 SDK 桥接 ✅ 部分在用

| 文件 | 用途 | 状态 |
|------|------|------|
| `nx_motion_node.py` | **NX 控狗节点**（持 lease、状态机、看门狗） | ✅ 在用（systemd 服务） |
| `nx_sensor_node.py` | **NX 读狗传感器** → 发 /imu /scan /odom | ✅ 在用 |
| `bridge_node.py` | 老的单体桥（PC 直连狗） | ❌ 废弃（被 NX 架构取代） |
| `sport_client.py` | SportClient 封装（bridge_node 用） | ❌ 随 bridge_node 废弃 |

### `src/` 其余包 — ❌ 全部不在用（历史遗留）

| 包 | 用途 | 为何废弃 |
|----|------|----------|
| `go2w_nav/` | SLAM Toolbox + Nav2 导航 | 被 panel.py 内联路径规划取代 |
| `go2w_orchestrator/` | 任务编排/VLM/语音 | 被 panel.py + ai/ + audio/ 取代 |
| `go2w_detector/` | YOLO 检测 ROS2 节点 | 被 ai/detector.py 取代 |
| `go2w_web/` | ROS2 版 Web 桥 | 被 web/panel.py 取代 |
| `go2w_bringup/` | 全栈 launch | 被 NX+panel 架构取代 |
| `go2w_interfaces/` | 自定义 msg/srv | 定义存在但无消费者 |
| `{go2w_interfaces/` | 垃圾目录（mkdir 失败产物） | 空壳，可删 |

### `docker/` — 部署文件 ✅ 在用

| 文件 | 用途 |
|------|------|
| `deploy_nx.sh` | **NX 一键部署**（自动探测网卡，拷代码+装服务） |
| `go2w-motion.service` | systemd 服务定义（崩溃自启夺 lease） |
| `ros_humble.sh` | PC 容器辅助脚本 |

### `docs/` — 文档 ✅ 在用

| 文件 | 内容 |
|------|------|
| `REFACTOR.md` | NX 架构设计（三机协同拓扑、IP 配置清单） |
| `TROUBLESHOOTING.md` | 踩坑记录（网卡、DDS 单向故障等） |
| `PROJECT_STRUCTURE.md` | 本文件 |
| `DECISIONS.md` | 决策记录 |
| `SDK_CAPABILITIES.md` | unitree SDK 能力调研 |

### 其他

- `audio/` — 语音捕获（whisper STT 相关，被 server.py 用，panel 不直接用）
- `hardware/` — 硬件相关（待确认）
- `yolov8n.pt` — YOLO 模型权重（6.5MB，gitignore）
- `setup_jetson.sh` — **⚠️ 过时**（galactic 版，与当前 NX 的 humble 不符，勿用）

---

## 三、当前可用功能 vs 待解决

| 功能 | 状态 | 关键文件 |
|------|------|----------|
| Web 前端（键盘+按钮） | ✅ | web/panel.py, static/panel.html |
| 狗站立/坐下/急停 | ✅ | nx_motion_node._do_stand/_do_sit/_do_estop |
| 乱跑防护 | ✅ | go2w-motion.service (崩溃自启) |
| 后滑(旧"修复"实为bug根因) | ⚠️已纠正 | _do_stand **加回** BalanceStand (对齐 panel.py, 见 TECH_DECISIONS §一) |
| **狗移动（前进后退转向）** | ⏳ 待实车 | _do_stand 加 BalanceStand 后逻辑对齐 panel.py, 待硬件装完验证 |
| 地图/雷达显示 | ⚠️ 部分 | nx_sensor_node 数据流（需确认在跑） |
| YOLO 检测 | ✅ | ai/detector.py（视频画框） |
| VLM 指令解析 | ⚠️ 降级 | ai/vlm.py 加载失败，用关键词匹配兜底 |
| 自动搜索/跟踪 | ❌ 待验证 | 依赖移动控制先解决 |

---

## 四、启动方式

**PC 端**（每次开机后）：
```bash
cd go2w_search_ws
bash web/start_ros2.sh    # 起容器+桥接+panel，浏览器开 localhost:8000
```

**NX 端**（go2w-motion.service 已 enabled，开机自启，无需手动）。

**首次部署新 NX**：
```bash
NX_HOST=<IP> bash docker/deploy_nx.sh
```
