# 重构方案：NX 中心化架构（全部本机通信）

> 2026-06-26 制定。这次重构把"重活全在 PC"改成"重活全在 NX，PC 仅调试"。
> 这是方向性文档，确认后分步实施。推翻了之前 PC↔NX 跨热点桥接的方案。

## 一、为什么重构（之前方案的问题）

之前（已实现并提交）的架构是 **PC 跑重活、NX 只控狗**，数据跨手机热点传：
```
panel.py(PC) → cmd_publisher → /cmd_vel ──热点DDS──> nx_motion_node(NX)
NX 传感器 ──热点DDS──> ros_to_json(PC) → dog_state.json → panel.py
```

**问题**：
1. 点云/视频跨热点传，**带宽和延迟都吃紧**（热点 RTT 可达 50ms+）
2. PC 断网，狗就失去建图/感知能力，**无法真正自主**
3. MID360 点云本该在 NX 本地处理，跨机传是浪费

## 二、新架构（目标）

```
┌── PC (仅调试/监控) ──────────────────────────┐
│  panel.py 网页控制台 (瘦客户端)               │
│  → 只发高层指令(导航目标/搜索区域)            │
│  → 只接收 NX 推送的状态(地图/位姿/检测)        │
└────────────────────┬─────────────────────────┘
                     │ 热点 WiFi (低频状态/指令, 非实时)
                     ▼
┌── 我们的外置 NX (载荷, 全部重活) ─────────────┐
│  ① MID360 驱动 → /cloud (点云, 本地)          │
│  ② FAST_LIO → /odom /map (3D SLAM, 本地)      │
│  ③ Nav2 → /cmd_vel (规划, 本地)               │
│  ④ nx_motion_node → 持 lease 控狗 (本地)      │
│  ⑤ nx_sensor_node → 狗 IMU (本地)             │
│  ⑥ YOLO/VLM → 检测/指令解析 (本地, Jetson GPU)│
│                                               │
│  全部 ROS2 本机通信, 不走热点!                 │
└────────────────────┬─────────────────────────┘
                     │ USB 网线 + unitree SDK (CycloneDDS)
                     ▼
┌── 狗自带 NX (主控 192.168.123.161) ───────────┐
│  出厂系统, 只接收 SDK 指令, 不跑我们的程序     │
└───────────────────────────────────────────────┘
```

**核心原则**：
- **NX 本机闭环**：雷达→建图→Nav2→控狗 全在 NX 本机 ROS2，零跨网延迟
- **PC 是瘦客户端**：只做网页监控 + 发高层目标，断了狗也能自主
- **热点只传低频数据**：地图缩略图、位姿、检测结果（KB 级），不传点云/视频流

## 三、重构范围（要改/新增/废弃什么）

### 新增（NX 上）
| 组件 | 作用 | 来源 |
|------|------|------|
| MID360 驱动节点 | 发 /cloud 点云 | livox_ros2_driver |
| FAST_LIO 节点 | 3D SLAM，发 /odom /map | FAST_LIO (需适配 Go2W IMU) |
| Nav2 栈 | 路径规划，发 /cmd_vel | 复用 src/go2w_nav，改吃 3D |
| YOLO/VLM 节点 | 检测/指令 | 从 ai/ 迁移，启用 Jetson TensorRT |

### 改造
| 组件 | 改动 |
|------|------|
| `nx_motion_node` | 保持，但 /cmd_vel 来源从"PC 热点"变成"本机 Nav2 + 本机网页" |
| `panel.py` | 从"PC 重活"瘦身为"NX 上跑的网页服务 + 瘦客户端模式" |
| `go2w_nav` | slam_toolbox 2D → FAST_LIO 3D；nav2 costmap 改吃点云 |

### 废弃
| 组件 | 原因 |
|------|------|
| `web/cmd_publisher.py` | 不再跨热点，Nav2 本机直接发 /cmd_vel |
| `web/ros_to_json.py` | 不再跨热点，NX 本机直接有 /odom 等 |
| `web/dog_state.json` | 不再文件桥接 |
| PC 的 go2w_humble 容器 | 不再需要（NX 本机就是 Humble） |

## 四、实施顺序（分阶段，每阶段可独立验证）

### 阶段 0：移动控制（地基，必须先做）⚠️
**Go2W 轮式步态切换正确参数**。Nav2 发的 /cmd_vel 必须能驱动轮子，否则一切白搭。
- 待 NX 恢复，查清 SwitchGait / Move 的正确用法

### 阶段 1：NX 本机 ROS2 闭环（去掉跨热点依赖）
- nx_motion_node + nx_sensor_node 改为纯本机通信
- 在 NX 上加一个轻量 web 服务（panel 的瘦版），PC 访问 NX 的网页
- 验证：NX 上发 /cmd_vel（本机），狗能动；PC 网页能看到狗状态

### 阶段 2：MID360 + FAST_LIO 建图
- NX 上装 livox_ros2_driver，发 /cloud
- 接 FAST_LIO，融合 MID360 点云 + 狗 IMU，输出 /odom /map
- 验证：推着狗走，NX 上能建出 3D 地图

### 阶段 3：Nav2 自主导航
- Nav2 costmap 改吃点云（3D → 2D 投影）
- 用 FAST_LIO 的 /odom 做 localization
- 验证：Nav2 发 /cmd_vel，狗自主走到目标点

### 阶段 4：AI 迁移
- YOLO/VLM 迁到 NX，启用 TensorRT
- 验证：NX 上跑检测，结果本机用于搜索/跟踪

## 五、与之前工作的关系

之前几轮做的（已提交 git）：
- ✅ **nx_motion_node + systemd 服务**：保留，仍是控狗核心
- ✅ **乱跑修复 / 后滑修复**：保留，这是 NX 控狗的基础
- ✅ **键盘控制前端**：保留思路，但 panel.py 要瘦身迁移到 NX
- ⚠️ **cmd_publisher / ros_to_json / dog_state.json**：阶段 1 废弃
- ⚠️ **PC go2w_humble 容器**：阶段 1 后不再需要

## 六、风险

1. **NX 算力**：FAST_LIO + Nav2 + YOLO 全在 NX，需监控负载（Orin NX 16GB 应该够，但要测）
2. **FAST_LIO 适配**：需融合 Go2W IMU 和 MID360，坐标系/时间同步要调
3. **移动控制未解决**：阶段 0 是硬前提，必须先突破
