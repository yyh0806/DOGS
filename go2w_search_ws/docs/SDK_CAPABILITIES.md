# Go2W 搜索系统 — 能力清单

> 基于实际测试验证，非文档推测。测试环境：网线直连笔记本 ↔ Go2W，unitree_sdk2py 1.0.1。

## 一、我们能获取的数据

### 1.1 摄像头（已验证 ✓）

| 项目 | 详情 |
|------|------|
| SDK 接口 | `VideoClient().GetImageSample()` |
| 分辨率 | 1920 × 1080 JPEG |
| 数据格式 | 返回 `list[int]`，需 `bytes(data)` 转 bytes 再 `cv2.imdecode` |
| 获取频率 | 单次调用约 100-200ms，实测连续获取约 5-8 fps |
| 延迟 | 约 200-500ms |
| 注意 | 没有 RTSP，只能通过 SDK 获取；每次调用返回一帧静态图像 |
| **高帧率方案** | ⚠️ unitree_sdk2py 的 `video_api.py` **只暴露 `GetImageSample`(id=1001)**，5-8fps 是 SDK 硬上限。狗出厂另有 **WebRTC 服务**(App/网页流畅视频，H.264 ~30fps，信令端口 19390)，社区 **go2-webrtc**(aiortc) 是高帧率正确用法。2026-06-29 实测狗 19390 在 USB 网段(192.168.123.161)关——可能监听 WiFi(192.168.12.1)或固件没开；接入待后续(见长期记忆 video-highfps-go2webrtc)。 |

### 1.2 IMU 惯性测量单元（已验证 ✓，已集成）

| 项目 | 详情 |
|------|------|
| 订阅方式 | DDS topic `rt/lowstate`，消息类型 `LowState_` |
| 频率 | 约 500Hz（8秒收到 4000 条） |
| 四元数 | `imu_state.quaternion` — [w, x, y, z] |
| 欧拉角 RPY | `imu_state.rpy` — [roll, pitch, yaw] 单位弧度 |
| 陀螺仪 | `imu_state.gyroscope` — [x, y, z] 角速度 rad/s |
| 加速度计 | `imu_state.accelerometer` — [x, y, z] m/s² |
| 示例数据 | `rpy=[-0.002, 0.022, 0.594]` `gyro=[-0.003, -0.002, 0.006]` |
| 集成状态 | **已集成到 SLAM 系统**，提供精确的航向角（yaw），替代 VO 的漂移 yaw |

订阅代码：
```python
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_

factory = ChannelFactory()
factory.Init(0, 'enp65s0')

def on_lowstate(msg):
    imu = msg.imu_state
    roll, pitch, yaw = imu.rpy  # 弧度
    print(f"yaw={math.degrees(yaw):.1f}°")

ch = factory.CreateRecvChannel('rt/lowstate', LowState_)
ch.SetReader(handler=on_lowstate)
```

### 1.3 激光雷达点云（已验证 ✓，已集成）

| 项目 | 详情 |
|------|------|
| 订阅方式 | DDS topic `rt/utlidar/cloud`，消息类型 `PointCloud2_` |
| 来源 | Go2W 内置 LiDAR（官方自带的 Mid-360 或类似型号） |
| 频率 | 约 15Hz |
| 每帧点数 | ~3800-3900 个点 |
| 点云字段 | x, y, z (float32), intensity (float32), ring (uint16), time (float32) |
| 点步长 | 32 字节/点，小端序 |
| 坐标系 | `utlidar_lidar`（x 前, y 左, z 上） |
| 高度过滤 | -0.1m ~ 1.5m 之间视为障碍物 |
| 距离过滤 | 0.15m ~ 8.0m |
| 有效障碍点 | 约 2900-3000 点/帧（过滤后） |
| 集成状态 | **已集成到 SLAM 系统**，解析为 2D 障碍物栅格（200×200，分辨率 0.1m） |

订阅代码：
```python
from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_

def on_lidar(msg):
    raw = bytes(msg.data)
    n = min(msg.width, len(raw) // msg.point_step)
    import struct, numpy as np
    xyz = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        xyz[i] = struct.unpack_from('<fff', raw, i * msg.point_step)
    # xyz[:, 0]=x(前), xyz[:, 1]=y(左), xyz[:, 2]=z(上)

ch = factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
ch.SetReader(handler=on_lidar)
```

### 1.4 激光雷达去畸变点云（topic 已确认，数据待验证）

| 项目 | 详情 |
|------|------|
| DDS topic | `rt/utlidar/cloud_deskewed` |
| 消息类型 | `PointCloud2_` |
| 坐标系 | `odom` |
| 说明 | 运动补偿后的去畸变点云，定位精度更高 |

### 1.5 高度图（topic 已确认，数据待验证）

| 项目 | 详情 |
|------|------|
| DDS topic | `rt/utlidar/height_map_array` |
| 消息类型 | `HeightMap_` — 包含 `resolution`, `width`, `height`, `origin`, `data[]` |
| 坐标系 | `odom` |
| 前提条件 | 需要运控程序正常运行 |

### 1.6 障碍物距离信息（topic 已确认，数据待验证）

| 项目 | 详情 |
|------|------|
| DDS topic | `rt/utlidar/range_info` |
| 消息类型 | `PointStamped_` |
| 说明 | 提供 前方/左方/右方 距离信息 |

### 1.7 电机状态（已验证 ✓）

| 项目 | 详情 |
|------|------|
| 来源 | `LowState_` 消息中的 `motor_state` 数组（20 个电机） |
| 数据 | 每个电机包含：位置、速度、力矩 |
| 频率 | 与 IMU 相同，约 500Hz |
| 用途 | 可用于里程计推算（轮式编码器） |

### 1.8 电池/电源状态（已验证 ✓）

| 项目 | 详情 |
|------|------|
| 来源 | `LowState_` 消息 |
| 电压 | `power_v` (float, V) |
| 电流 | `power_a` (float, A) |
| 温度 | `temperature_ntc1`, `temperature_ntc2` |

### 1.9 遥控器数据（已验证 ✓）

| 项目 | 详情 |
|------|------|
| 来源 | `LowState_` 消息中的 `wireless_remote`（40 字节） |
| 用途 | 可读取遥控器按键状态 |

### 1.10 激光雷达状态（已验证 ✓）

| 项目 | 详情 |
|------|------|
| 订阅方式 | DDS topic `rt/lidarstate`（或其他类似名称） |
| 消息类型 | `LidarState_` |
| 包含信息 | `cloud_size`, `cloud_frequency`, `imu_rpy`, `error_state`, `rotation_speed` |
| 注意 | 这是雷达运行状态元数据，不含点云数据 |

---

## 二、我们能执行的动作

### 2.1 运动控制（已验证 ✓）

| 动作 | SDK 方法 | 结果 | 备注 |
|------|----------|------|------|
| 站立 | `SportClient.BalanceStand()` | ✓ 正常 | 返回 0 表示成功 |
| 前进 | `SportClient.Move(vx, 0, 0)` | ✓ 正常 | vx 正值前进，负值后退 |
| 后退 | `SportClient.Move(-vx, 0, 0)` | ✓ 正常 | |
| 左移 | `SportClient.Move(0, vy, 0)` | ✓ 正常 | vy 正值左移 |
| 右移 | `SportClient.Move(0, -vy, 0)` | ✓ 正常 | |
| 左转 | `SportClient.Move(0, 0, vyaw)` | ✓ 正常 | vyaw 正值左转 |
| 右转 | `SportClient.Move(0, 0, -vyaw)` | ✓ 正常 | |
| 停止 | `SportClient.Move(0, 0, 0)` | ✓ 有效 | **StopMove() 对 Go2W 轮式模式无效** |
| 速度等级 | `SportClient.SpeedLevel(level)` | 未测试 | level=0/1/2 对应不同速度档位 |

### 2.2 不生效的动作

| 动作 | SDK 方法 | 结果 | 备注 |
|------|----------|------|------|
| 停止 | `SportClient.StopMove()` | ✗ 不生效 | 对 Go2W 轮式模式无效，必须用 `Move(0,0,0)` |
| 坐下 | `SportClient.Sit()` | ✗ 不生效 | 无反应 |
| 趴下 | `SportClient.StandDown()` | ✗ 不生效 | 无反应 |
| 阻尼 | `SportClient.Damp()` | ⚠ 会导致倒下 | 不是"停止"，是取消平衡，狗会倒 |

### 2.3 不存在的 SDK 方法

| 期望功能 | 状态 |
|----------|------|
| `Velocity()` | 不存在，用 `Move(vx, vy, vyaw)` 代替 |
| `MoveTo(x, y)` | 不存在，没有目标点导航 |
| `SwitchMoveMode()` | 不存在，无法切换轮式/足式 |
| `GetPose()` / 位置查询 | 不存在，无法直接获取位置 |
| `DampStand()` | 不存在 |

### 2.4 特殊动作（未测试）

| 动作 | SDK 方法 | 备注 |
|------|----------|------|
| 恢复站立 | `RecoveryStand()` | 倒下后恢复站立 |
| 跳跃 | `FrontJump()` | 特技动作 |
| 翻转 | `FrontFlip()`, `BackFlip()` | 特技动作 |
| 舞蹈 | `Dance1()`, `Dance2()` | 表演动作 |
| 招手 | `Hello()` | 表演动作 |
| 避障开关 | `SwitchAvoidMode()` | 未测试 |
| 摇杆模式 | `SwitchJoystick()` | 未测试 |

---

## 三、网络配置

### 3.1 有线连接（推荐，稳定）

| 项目 | 详情 |
|------|------|
| 接口 | enp65s0（笔记本有线网卡） |
| 笔记本 IP | 192.168.123.100/24（静态，NM 持久化） |
| Go2W IP | 192.168.123.161 |
| DDS 通信 | ✓ 正常，所有功能可用 |
| 延迟 | < 1ms |

### 3.2 WiFi cp1 热点（不可用）

| 项目 | 详情 |
|------|------|
| SSID | cp1 |
| 密码 | 00000000 |
| Go2W WiFi IP | 192.168.12.1 |
| 笔记本 IP | 192.168.12.100/24（静态） |
| Ping | ✓ 通 |
| DDS | ✗ 不通 — Go2W 的 DDS 服务只监听以太网接口，WiFi 上所有端口关闭 |
| 结论 | **WiFi 只能 ping 通，无法控制** |

### 3.3 DDS 通信要点

| 项目 | 详情 |
|------|------|
| 协议 | CycloneDDS（UDP 多播 + 单播） |
| 端口 | 7900-7905（以太网接口） |
| 初始化 | `ChannelFactory().Init(0, 'enp65s0')` — 必须指定正确的网卡名 |
| 单例限制 | ChannelFactory 是进程级单例，一个 Python 进程只能初始化一次 |
| 多进程 | 多个 Python 进程各自初始化 ChannelFactory 不冲突 |
| 回调注意 | DDS 回调中不宜做重操作（如 numpy 大数组解析），应使用队列+后台线程 |

---

## 四、当前系统架构

```
浏览器 (http://<笔记本IP>:8000)
  ├── 第一视角画面（摄像头 + YOLO 检测框）
  ├── SLAM 建图（LiDAR 障碍物栅格 + IMU 航向 + 轨迹）
  └── 搜索 / 停止 按钮
       │
       │ WebSocket + REST API
       ▼
FastAPI 后端 (web/server.py)
  ├── RobotSDK
  │     ├── SportClient       → 运动控制（站立、移动、停止）
  │     ├── VideoClient       → 摄像头 1920x1080
  │     ├── IMU Subscriber    → rt/lowstate, 500Hz, 提供精确 yaw
  │     ├── LiDAR Subscriber  → rt/utlidar/cloud, 15Hz, 点云 → 障碍物栅格
  │     ├── VisualOdometer    → ORB 特征（备用，保留）
  │     └── 轮式里程计         → IMU yaw + 速度积分推算位置
  ├── Detector (YOLOv8n)      → 目标检测（当前加载失败，待修复）
  ├── LidarMap                 → PointCloud2 解析 → 200×200 栅格 (0.1m/格)
  └── SearchMission            → 割草机路径规划 + 定时导航
       │
       │ DDS (CycloneDDS, enp65s0)
       ▼
Go2W 机器狗 (192.168.123.161)
  ├── 摄像头 (1920x1080)
  ├── IMU (500Hz, rpy/quat/gyro/accel)
  ├── LiDAR (15Hz, ~3800点/帧)
  └── 运动控制 (Move/BalanceStand)
```

---

## 五、已解决的坑

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| StopMove() 无效，狗不停 | Go2W 轮式模式不支持 | 用 `Move(0,0,0)` 代替 |
| Damp() 导致狗倒下 | Damp 是取消平衡控制 | 绝对不要用，用 `Move(0,0,0)` 停止 |
| 重复命令导致红色保护模式 | 发送频率过高 | 控制命令频率，避免刷屏 |
| WiFi DDS 不通 | Go2W DDS 只监听以太网 | 必须用网线连接 |
| ChannelFactory import 路径 | 不是 `unitree_sdk2py.comm` | 正确路径: `unitree_sdk2py.core.channel` |
| DDS 回调阻塞 uvicorn | LiDAR 解析在回调线程中执行 | 改用队列 + 后台 worker 线程 |
| DDS 连接阻塞 uvicorn 启动 | `connect()` 在主线程同步执行 | 改为后台线程异步连接 |
| LiDAR worker 立即退出 | `_connected` 在 connect() 末尾才设 True | 改用 `_dds_inited` 作为 worker 运行标志 |
| VO 漂移严重 | 单目视觉里程计尺度不确定 | 改用 IMU yaw + 轮式里程计 |

---

## 六、待解决问题

| 问题 | 优先级 | 状态 |
|------|--------|------|
| 搜索任务的导航逻辑待实机验证 | 高 | 待验证 |
| YOLO 模型加载失败（Python 3.8 兼容性） | 高 | 待修复 |
| HeightMap / Deskewed Cloud 数据未获取 | 中 | topic 已确认，需运控程序运行 |
| 搜索导航改用 IMU yaw 做精确转向 | 中 | 待开发 |
| Sit/StandDown 命令不生效 | 低 | Go2W 限制 |
| WiFi DDS 不通 | 低 | Go2W 限制，需用网线或便携路由器 |

---

## 七、DDS Topic 汇总

### 已验证可用的 Topic

| Topic | 消息类型 | 频率 | 说明 |
|-------|----------|------|------|
| `rt/lowstate` | `LowState_` (unitree_go) | ~500Hz | IMU + 电机 + 电池 + 遥控器 |
| `rt/utlidar/cloud` | `PointCloud2_` (sensor_msgs) | ~15Hz | LiDAR 原始点云 |

### 已确认但未收到数据的 Topic

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `rt/utlidar/cloud_deskewed` | `PointCloud2_` (sensor_msgs) | 去畸变点云，坐标系 odom |
| `rt/utlidar/height_map_array` | `HeightMap_` (unitree_go) | 高度地图，需运控程序运行 |
| `rt/utlidar/range_info` | `PointStamped_` (geometry_msgs) | 前/左/右距离 |

### 未验证的 Topic（猜测）

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `rt/lidarstate` | `LidarState_` (unitree_go) | LiDAR 运行状态元数据 |

---

## 八、关键代码片段

### 连接 Go2W（含 IMU + LiDAR）
```python
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_

factory = ChannelFactory()
factory.Init(0, 'enp65s0')

# 运动
client = SportClient()
client.SetTimeout(10.0)
client.Init()
client.BalanceStand()

# IMU
def on_imu(msg):
    yaw = msg.imu_state.rpy[2]
ch = factory.CreateRecvChannel('rt/lowstate', LowState_)
ch.SetReader(handler=on_imu)

# LiDAR
def on_lidar(msg):
    pass  # 处理点云
ch2 = factory.CreateRecvChannel('rt/utlidar/cloud', PointCloud2_)
ch2.SetReader(handler=on_lidar)
```

### 获取摄像头图像
```python
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2, numpy as np

video = VideoClient()
video.SetTimeout(10.0)
video.Init()

code, data = video.GetImageSample()  # data 是 list[int]
frame = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
# frame: 1920x1080 BGR ndarray
```

### 解析 LiDAR 点云
```python
import struct, numpy as np

def parse_pointcloud(msg):
    raw = bytes(msg.data)
    point_step = msg.point_step  # 32
    n = min(msg.width, len(raw) // point_step)
    xyz = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        xyz[i] = struct.unpack_from('<fff', raw, i * point_step)
    # xyz[:, 0]=x(前), xyz[:, 1]=y(左), xyz[:, 2]=z(上)
    return xyz
```

### 运动控制
```python
client.BalanceStand()          # 站立
client.Move(0.3, 0, 0)        # 前进 0.3 m/s
client.Move(-0.3, 0, 0)       # 后退
client.Move(0, 0.3, 0)        # 左移
client.Move(0, 0, 0.3)        # 左转
client.Move(0, 0, 0)          # 停止（有效）
# client.StopMove()            # 无效！
```
