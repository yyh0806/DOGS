# Go2W Search & Discover

通过一条指令让 Go2W 轮足机器狗自动搜索指定区域，发现并报告目标。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                   Jetson Xavier NX (车载)                     │
│                                                              │
│  ┌──────────────────┐  ROS2  ┌──────────────────┐           │
│  │  search_commander │◄──────►│   go2w_bridge     │           │
│  │  (Rust/rclrs)     │        │   (Python)        │           │
│  │                   │        │                    │           │
│  │  · 路径规划       │ 话题   │  · SDK→ROS2 桥接  │           │
│  │  · 导航控制       │◄──────►│  · 速度指令转发   │           │
│  │  · 任务管理       │        │  · 状态发布       │           │
│  └────────┬─────────┘        └────────┬──────────┘           │
│           │                            │ DDS/CycloneDDS       │
│  ┌────────┴─────────┐                 │                      │
│  │  go2w_detector    │                 │                      │
│  │  (Python)         │                 │                      │
│  │                    │                 │                      │
│  │  · YOLO 检测      │                 │                      │
│  │  · TensorRT FP16  │                 │                      │
│  │  · 摄像头流       │                 │                      │
│  └──────────────────┘                 │                      │
│                                        │                      │
│              ROS2 Galactic + CycloneDDS│                      │
└────────────────────────────────────────┼─────────────────────┘
                                         │ USB-Ethernet
                                ┌────────┴──────────┐
                                │   Unitree Go2W     │
                                │   192.168.123.161  │
                                │                    │
                                │  · 前置摄像头      │
                                │  · mid-360 LiDAR   │
                                │  · IMU             │
                                │  · 轮式/步行模式   │
                                └───────────────────┘
```

## 硬件需求

| 组件 | 说明 |
|------|------|
| Unitree Go2W | 轮足版机器狗 |
| Jetson Xavier NX | 16GB，运行 ROS2 + YOLO |
| USB-Ethernet 转接器 | RTL8156B 千兆，连接 Go2W |
| DC 供电线 | Go2W 外部 12V 供电 → NX |
| 3D 打印支架 | 固定 NX 到 Go2W 背部 |

详细接线方案见 `hardware/SETUP_GUIDE.md`。

## 快速开始

### 1. Jetson NX 部署 (首次)

```bash
# 克隆项目到 Jetson NX
cd ~
git clone <repo_url> go2w_search_ws
cd go2w_search_ws

# 一键部署 (安装 ROS2 + Rust + 依赖 + 编译)
chmod +x setup_jetson.sh
./setup_jetson.sh
```

### 2. 连接 Go2W

```bash
# 确认网络连通
ping 192.168.123.161

# 确认 RTSP 摄像头流
ffplay rtsp://192.168.123.161:8554/camera
```

### 3. 一条指令启动搜索

```bash
# 加载环境
source install/setup.bash

# 搜索 10x10 米区域，割草机模式
ros2 launch go2w_bringup search.launch.py \
    area_width:=10.0 area_height:=10.0 pattern:=lawnmower
```

任务启动后自动:
1. Go2W 站立 → 切换轮式模式
2. 生成搜索路径 (航点)
3. 逐航点导航 + 实时检测
4. 发现目标 → 记录位置 + 保存图片
5. 搜索完成 → 返回起点 → 生成报告

### 4. 交互控制

```bash
# 手动触发搜索 (launch 启动后)
ros2 service call /go2w/start_search go2w_interfaces/srv/StartSearch \
    "{width: 20.0, height: 15.0, pattern: 'lawnmower', spacing: 2.0, target_classes: ['person']}"

# 查看任务状态
ros2 topic echo /go2w/mission_status

# 查看检测结果
ros2 topic echo /go2w/detections

# 手动停止
ros2 service call /go2w/stop_search go2w_interfaces/srv/StopSearch \
    "{mission_id: '', return_to_start: true}"
```

## 项目结构

```
go2w_search_ws/
├── src/
│   ├── go2w_interfaces/           # ROS2 自定义消息/服务
│   │   ├── msg/
│   │   │   ├── SearchArea.msg     # 搜索区域
│   │   │   ├── Waypoint.msg       # 航点
│   │   │   ├── PathPlan.msg       # 路径规划
│   │   │   ├── TargetDetection.msg# 目标检测
│   │   │   ├── MissionStatus.msg  # 任务状态
│   │   │   ├── RobotState.msg     # 机器人状态
│   │   │   └── MissionReport.msg  # 任务报告
│   │   └── srv/
│   │       ├── StartSearch.srv    # 开始搜索
│   │       └── StopSearch.srv     # 停止搜索
│   │
│   ├── go2w_search_rust/          # Rust 核心节点
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── commander_main.rs  # 指挥节点入口
│   │       ├── planner.rs         # 路径规划 (割草机/螺旋)
│   │       ├── navigator.rs       # 导航控制
│   │       └── types.rs           # 类型定义
│   │
│   ├── go2w_bridge/               # Go2W SDK 桥接 (Python)
│   │   └── go2w_bridge/
│   │       ├── bridge_node.py     # ROS2 桥接节点
│   │       └── sport_client.py    # Go2W 运动控制封装
│   │
│   ├── go2w_detector/             # 目标检测 (Python)
│   │   └── go2w_detector/
│   │       └── detector_node.py   # YOLO + TensorRT 节点
│   │
│   └── go2w_bringup/              # 启动配置
│       ├── launch/
│       │   └── search.launch.py   # 一键启动文件
│       └── config/
│           └── default.yaml       # 默认参数
│
├── hardware/
│   └── SETUP_GUIDE.md             # 硬件安装指南
│
├── setup_jetson.sh                # Jetson NX 一键部署
└── README.md
```

## ROS2 话题/服务接口

### 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/go2w/cmd_vel` | geometry_msgs/Twist | Commander → Bridge | 速度指令 |
| `/go2w/robot_state` | go2w_interfaces/RobotState | Bridge → Commander | 机器人位姿/速度 |
| `/go2w/detections` | go2w_interfaces/TargetDetection | Detector → Commander | 检测到的目标 |
| `/go2w/mission_status` | go2w_interfaces/MissionStatus | Commander 发布 | 任务进度 |
| `/go2w/path_plan` | go2w_interfaces/PathPlan | Commander 发布 | 规划路径 |
| `/go2w/mission_report` | go2w_interfaces/MissionReport | Commander 发布 | 最终报告 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/go2w/start_search` | go2w_interfaces/StartSearch | 启动搜索任务 |
| `/go2w/stop_search` | go2w_interfaces/StopSearch | 停止搜索任务 |

## 搜索模式

### 割草机模式 (Lawnmower)

```
→ → → → → → → → → →
                    |
← ← ← ← ← ← ← ← ← ←
|
→ → → → → → → → → →
                    |
← ← ← ← ← ← ← ← ← ←
```

- 适合矩形区域
- 效率最高，覆盖均匀
- 行间距可配置 (默认 2.5m)

### 螺旋模式 (Spiral)

```
    ┌───────────┐
    │ → → → → ┐ │
    │ ┌─────┐ │ │
    │ │ → → ┘ │ │
    │ └───────┘ │
    └───────────┘
```

- 从中心向外扩展
- 适合不确定目标大致位置的情况

## 检测能力

- **模型**: YOLOv8n (默认) / YOLOv8s / 自定义模型
- **加速**: TensorRT FP16 on Jetson NX (~25ms/帧, ~40 FPS)
- **类别**: COCO 80类 (person, car, dog 等) 或自定义
- **输出**: 目标类别、置信度、边界框、机器人位置、标注图片

## 配置修改

编辑 `src/go2w_bringup/config/default.yaml`:

```yaml
# 修改检测目标类别
detector:
  ros__parameters:
    target_classes: ["person", "car", "truck"]
    confidence: 0.5

# 修改移动速度
commander:
  ros__parameters:
    drive_speed: 1.5    # 降低速度增加安全性
    waypoint_tolerance: 0.3
```

## 开发

```bash
# 仅编译特定包
colcon build --packages-select go2w_interfaces
colcon build --packages-select go2w_bridge

# Rust 单独编译
cd src/go2w_search_rust
cargo build --release
cargo test

# 运行单元测试
colcon test --packages-select go2w_search_rust

# 查看日志
ros2 topic echo /rosout
```

## 常见问题

**Q: 连接 Go2W 失败**
- 检查 USB-Ethernet 是否识别: `ip link`
- 检查 IP 配置: `ip addr show`
- 确认 ping 通: `ping 192.168.123.161`
- 检查 CycloneDDS 网卡配置

**Q: 摄像头无画面**
- 检查 RTSP 流: `ffplay rtsp://192.168.123.161:8554/camera`
- 确认 Go2W 已开机且网络连通
- 尝试重启 Go2W

**Q: TensorRT 导出失败**
- 确保 JetPack 完整安装
- 先用 PyTorch 模型运行: `use_tensorrt: false`
- 手动导出: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='engine', half=True)"`

**Q: Rust 编译失败**
- 确认 ros2_rust 已克隆到 src/
- 检查 Rust 版本: `rustc --version` (需要 1.60+)
- 先构建 go2w_interfaces 再构建 Rust 包
