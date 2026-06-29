# Go2W 改造架构 — 载荷 NX + 多机协同

> 基于实测验证,非推测。测试时间 2026-06-24。
> 测试环境:PC(Ubuntu 20.04, Docker Humble) + 载荷NX(Jetson Orin NX 16GB, Ubuntu 22.04, Humble) + Go2W 狗主控(出厂系统)。

## 一、改造背景

原架构(`go2w_bridge`)是**单机闭环**:PC 直接通过 unitree_sdk2py + CycloneDDS 连狗主控(`192.168.123.161`),既读狗传感器(IMU/LiDAR/相机)、又发 `/cmd_vel` 控狗。

新增硬件后改为**三机协同**:
- 载荷 Jetson Orin NX 16GB(装在狗上,网线直连狗主控)
- MID360 3D 激光雷达(接载荷NX)
- 云台相机(接载荷NX)
- PX4 飞控(待接,作 IMU/姿态源)

## 二、核心架构决策

### 决策 1:不新开项目,原地重构 `go2w_search_ws`

理由:同一个机器人、同一套搜索/检测/编排任务,只是传感器和算力升级。新开仓库会割裂 `go2w_interfaces`(msg/srv 要跨设备共享)、割裂已跑通的 orchestrator/web/nav2。ROS2 工作空间天然支持"一套源码、多机部署",用 launch 区分运行位置即可。

### 决策 2:三段链路三套技术(关键)

系统版本犬牙交错,**不能用一套 ROS2 串通所有链路**:

```
PC(Ubuntu 20.04) ──ROS2 Humble DDS──> 载荷NX(Ubuntu 22.04) ──unitree SDK──> 狗主控(出厂系统)
   Galactic(已EOL)                         Humble                    CycloneDDS(非ROS)
        ↑用Docker跑Humble                    ↑原生Humble              ↑狗主控跑不了Humble
```

| 链路 | 系统 | 通信方式 | 理由 |
|---|---|---|---|
| PC ↔ 载荷NX | 20.04 ↔ 22.04 | **ROS2 Humble DDS** | 两端 Humble 对齐,跨机 DDS 双向通 |
| 载荷NX 内部 | 22.04 | ROS2 Humble | 本机节点通信 |
| 载荷NX ↔ 狗主控 | 22.04 ↔ 出厂 | **unitree_sdk2py**(CycloneDDS,非ROS) | 狗主控跑不了 Humble,且本就是 DDS 世界,直连 SDK 最稳 |

### 决策 3:`go2w_bridge` 退役,职责迁移

`go2w_bridge` 的三职责全部被新架构替代:

| `go2w_bridge` 现职责 | 新架构 |
|---|---|
| 读狗传感器(IMU/LiDAR/相机) | 传感器来自载荷NX(MID360/云台相机/PX4) |
| `/cmd_vel` → Go2W SDK(lease/看门狗) | 迁到载荷NX `go2w_motion`,网线直连狗 |
| 发 `/scan` `/odom` `/camera` | 来自 NX 各驱动包 |

## 三、目标拓扑

```
┌──── PC (Ubuntu 20.04 + Docker Humble) ────┐         ┌── 载荷 NX (Orin NX, Ubuntu 22.04, Humble) ──┐
│                                            │         │                                             │
│  nav2 / FAST_LIO / orchestrator / web     │  ROS2   │  MID360 驱动 → /cloud                        │
│  ─────────────────────────────────────    │◀═DDS══▶│  云台相机 → /camera                           │
│        只发高层 /cmd_vel + /nav2 目标      │ (热点)  │  PX4/imu → /imu                              │
│                                            │         │  YOLO(TensorRT) → /detections                │
│  Docker: go2w_humble 容器 (--net=host)    │         │                                             │
└────────────────────────────────────────────┘         │  go2w_motion (← go2w_bridge 迁移)           │
                                                       │  订阅 /cmd_vel → unitree_sdk2py             │
                                                       └───────────────┬─────────────────────────────┘
                                                                       │ USB转网口 enxc8a362616c4c
                                                                       │ (192.168.123.100, 持久化)
                                                                       │ CycloneDDS
                                                                       ▼
                                                              ┌── 狗主控 (出厂系统) ──┐
                                                              │  192.168.123.161      │
                                                              │  不跑我们的常驻程序   │
                                                              │  只接收 SDK 指令      │
                                                              │  (看门狗在载荷NX)    │
                                                              └───────────────────────┘

    手机热点 192.168.43.0/24 (华为)
    PC: 192.168.43.35   载荷NX: 192.168.43.41   (WiFi)
```

## 四、已验证的链路状态(实测)

| 链路 | 验证方式 | 结果 |
|---|---|---|
| PC ↔ 载荷NX ROS2 DDS | 双向心跳话题互发 | ✅ 双向数据通 |
| 华为热点 AP 隔离 | 跨机 DDS 正常 | ✅ 无隔离 |
| PC Docker Humble | `go2w_humble` 容器 | ✅ 运行中 |
| 载荷NX → 狗主控 ping | ping 192.168.123.161 | ✅ 0%丢包 RTT 0.27ms |
| 狗主控身份确认 | SSH banner + DDS 抓包 | ✅ OpenSSH 8.2, 发 UDP 组播 |
| 读狗 IMU | 订阅 rt/lowstate | ✅ ~500Hz, RPY 数据正常 |
| 读狗 LiDAR | 订阅 rt/utlidar/cloud | ✅ ~10Hz, 1200-1400点/帧 |

## 五、关键 IP / 配置清单

| 设备 | 接口 | IP | 说明 |
|---|---|---|---|
| PC | wlp66s0 | 192.168.43.35 | 华为热点 WiFi |
| 载荷NX | wlP1p1s0 | 192.168.43.41 | 华为热点 WiFi |
| 载荷NX | enxc8a362616c4c | 192.168.123.100 | **USB转网口, 连狗主控, nmcli持久化(con-name: go2-dog)** |
| 载荷NX | enP8p1s0 | 192.168.144.36 | 板载网口, 连狗MCU(144.108), 暂未用 |
| 狗主控 | - | 192.168.123.161 | 宇树默认主控IP |
| 狗MCU | - | 192.168.144.108 | 运动控制器, 只响应ICMP/telnet23, 不发DDS |

### ⚠️ 重要:狗主控在 USB 转网口,不在板载网口

载荷NX 有两个有物理连接的网口:
- **板载 `enP8p1s0`(144.36)** → 连狗的 MCU(运动控制器 144.108),**不是狗主控**
- **USB转网口 `enxc8a362616c4c`** → **这才是连狗主控(123.161)的口**,之前没配IP导致连不上

(详见 `docs/TROUBLESHOOTING.md` 问题1)

## 六、迁移路径(分阶段)

1. ✅ **链路地基**:PC↔NX DDS + NX→狗 SDK(已完成)
2. ⏳ **MID360 + 云台相机**:在载荷NX上装驱动,发 ROS2 话题
3. ⏳ **FAST_LIO**:PC 上接 MID360 点云做 3D SLAM
4. ⏳ **nav2 3D化**:costmap 改吃点云(替换原 `/scan`)
5. ⏳ **PX4→EKF**:robot_localization 融合里程计+IMU
6. ⏳ **检测迁NX**:YOLO 启用 Jetson TensorRT
7. ⏳ **`go2w_motion`**:运动控制迁载荷NX,退役 `go2w_bridge`

## 七、安全备忘

- **看门狗必须在载荷NX的 `go2w_motion` 里**(不能只在PC),否则 PC↔NX 热点断开时狗会执行最后一条 `/cmd_vel` 直到撞墙
- 连狗时 `SportClient(enableLease=True)` 会接管狗控制权,危险;**只读验证只用 ChannelFactory + RecvChannel,不碰 SportClient**
- 任何运动指令测试前,确认狗趴下、周围安全
