# DOGS 系统架构文档

> 本文档描述 Go2W 轮足机器狗自主搜索系统的整体架构，作为代码阅读与开发的权威入口。
> 创建于 2026-08 优化分支（`codex/optimize-project-structure`）；2026-08-14 按源码复核修正（systemd 依赖、感知链路、网络拓扑）。
> 逐文件状态以 [`PROJECT_STRUCTURE.md`](PROJECT_STRUCTURE.md) 为准。

---

## 1. 系统总览

**DOGS（Dog Object Guard System）**：基于 Unitree Go2W 轮足机器狗的自主搜索与发现系统。
核心能力：前端/语音发出搜索指令 → 机器狗自动建图（FastLIO）→ Nav2 自主导航 → YOLO-World 目标检测 → 去重标注并报告。

### 架构模式：NX 中心化

```
┌──────────────┐   HTTP:8000 / WS:8001   ┌──────────────────────────────┐
│  PC (瘦客户端) │ ◄──────────────────────► │  Jetson Orin NX (机载大脑)      │
│  浏览器面板    │                          │  ├─ go2w-web    (Web+编排+感知) │
└──────────────┘                          │  ├─ go2w-motion (控狗, lease)   │
                                          │  ├─ go2w-sensor (读狗传感器)     │
                                          │  ├─ go2w-slam-nav (SLAM+Nav2)   │
                                          │  └─ livox-*    (MID360 雷达)    │
                                          └──────────────┬───────────────┘
                                                         │ USB-网口 192.168.123.x
                                                 ┌───────▼────────┐
                                                 │ Go2W 狗主控     │
                                                 │ 192.168.123.161 │
                                                 └────────────────┘
```

- **NX**：所有重活（感知、建图、规划、控狗、搜索编排）跑在机载 Jetson 上。
- **PC**：仅浏览器访问 `http://<NX_IP>:8000`，不跑任何 ROS 节点。
- **网络拓扑**：NX → 狗主控 `192.168.123.100↔.161`（USB-Ethernet）；NX → MID360 `192.168.1.200↔.160`（独立网口）；PC ↔ NX 走手机热点 Wi-Fi（NX DHCP 动态，可用 ARP 扫描发现）。

## 2. 顶层目录职责

| 路径 | 职责 | 说明 |
|------|------|------|
| `go2w_search_ws/` | 主工作空间 | ROS2 包 + Web + Docker + Docs |
| `go2w_search_ws/web/` | Web 服务 + 前端 | `nx_web_server.py` 主入口；`static/` 前端页面 |
| `go2w_search_ws/src/go2w_bridge/` | 狗 SDK 桥 | `nx_sensor_node.py`（只读）/ `nx_motion_node.py`（控狗） |
| `go2w_search_ws/src/go2w_nav/` | SLAM + Nav2 | launch 文件 + 参数配置 |
| `go2w_search_ws/src/go2w_sim/` | 仿真包 | Gazebo 世界 + URDF + 仿真节点 |
| `go2w_search_ws/docker/` | NX 部署 | systemd service 文件 + 部署脚本 + 部署契约测试 |
| `go2w_search_ws/tools/` | 运维工具 | 诊断/标定/验证脚本（`diag_*`、`nav_*`、`verify_*`） |
| `go2w_search_ws/docs/` | 文档 | 架构/排障/部署/标定手册 |
| `docs/archive/` | 历史归档 | 已退役代码快照（如 `nx_deploy_snapshot/`） |
| `QUICKSTART.md` | 快速上手 | 仿真（路径 A）/ 真狗（路径 B） |

## 3. 核心数据流

### 3.1 感知链路

```
MID360 雷达 ──► /livox/lidar (CustomMsg)
                ├──► mid360_nav_bridge ──► /scan_mid360 (LaserScan) ──► Nav2 costmap
                ├──► FAST_LIO ──► /Odometry ──► map_odom_fuser ──► /odom → TF(map→odom→base_link)
                └──► LidarBridge (Web 内嵌) ──► WS type=lidar（2026-08 起前端停用）
狗自带 utlidar ──► rt/utlidar/cloud (PointCloud2) ──► nx_sensor_node ──► /scan（诊断用，自屏蔽盒内编码 NaN）

### 3.1.1 室外扩展链（2026-08 新增，代码就绪待实机标定）

```
GPS 天线/PX4 ──► /gps/fix (NavSatFix) 或串口 NMEA
                   └─► nx_gps_nav (web 进程内): GpsFixGate 质量门 → ENU+北向标定
                       → GpsRouteController 逐航点 → PointNavigationController
                       → Nav2 (复用室内点导航链, 不动 TF/odom)
                   ──► /api/gps/route (受理) / /api/gps/calibrate (标定)
                   ──► WS type=gps_route (状态推送); fail-closed 条款见 docs/GPS_NAV.md §3

Nooploop AOA 基站 (串口 NLink) ──► uwb_serial_bridge (B-1, mock 可降级测试)
                   └─► latest() 快照 → nx_uwb_bridge 适配 (fix 合同, 隐式 mock 拒绝)
                       → UwbFollowController (B-2, 五态状态机)
                       → 避障闸门(directional_clearance) → robot.move(manual)
                       → arbiter manual 所有权通道 (零速 handoff)
                   ──► /api/uwb_follow/{start,stop,status,params} + 前端跟随按钮
                   ──► WS type=uwb_follow; 安全条款见 nx_uwb_follow.py 模块头
```
```

### 3.2 控制链路

```
Web 面板点击地图 ──► /api/navigate ──► navigation_arbiter ──► Nav2 /navigate_to_pose
        Nav2 输出 /cmd_vel_nav ──► nx_motion_node（经 SportGateway 控狗，lease 在 sport-gateway）──► 狗主控
```

### 3.3 WebSocket 推送（前端数据合同）

| WS type | 内容 | 来源 |
|---------|------|------|
| `slam` | 位姿/轨迹/扫描点/检测 | `broadcast_loop`（nx_web_server.py） |
| `costmap` | local_costmap 障碍网格 | costmap_bridge → /tmp JSON |
| `costmap_global` | global_costmap | costmap_bridge |
| `occupancy_map` | 持久墙体 | map_frontier_walls.json |
| `plan` | Nav2 规划路线 | /plan |
| `gimbal` | C13 云台双流（可见光+红外） | nx_gimbal_node |
| `detections` | YOLO 检测结果 | nx_ai_node |
| `lidar` | MID360 鸟瞰 PNG | **2026-08 起前端停用**（双源交叉显示修复） |
| `uwb_follow` | UWB 跟随状态（state/reason） | UwbFollowController state_callback |
| `gps_route` | GPS 航线状态（航点进度/健康） | GpsRouteController state_callback |

## 4. systemd 服务依赖

```
livox-mid360-net ─► livox-mid360-driver ─► go2w-fastlio
                                   go2w-sport-gateway（唯一 lease 持有者，独立）
                                          └─► go2w-motion（Requires）─► go2w-web（After）
  go2w-sensor / go2w-safety-observer（独立，只读）
  go2w-slam-nav（独立 oneshot bringup：FastLIO→mid360 bridge→fuser→Nav2→slam，
                内部 systemd-run 起 transient units：fastlio / mid360-nav-bridge /
                map-odom-fuser / nav2-3d / slam-online / map-padding / nav-health-supervisor）
  costmap-bridge（独立，读 costmap 写 JSON）
```

- `go2w-web` 依赖 `go2w-motion`（`After=`）。
- `go2w-slam-nav` 为一次性 bringup 脚本；`livox-mid360-watchdog` 守护雷达流（数据陈旧即重启驱动）。

## 5. 关键设计决策（摘要）

1. **NX 中心化**：PC 不跑 ROS，所有节点同机 → DDS 走 UDP loopback（禁 SHM，见 `fastdds_udp.xml`），根治 SHM 损坏导致订阅静默失效。
2. **双雷达源**：MID360 供 SLAM+Nav2（`/scan_mid360`），狗自带 utlidar 供诊断（`/scan` 已停发）。前端 2026-08 起只渲染 `/scan_mid360`。
3. **原子发布**：`build_release.sh` 构建 content-addressed 发布包 → `deploy_release.sh` 原子切换 `current` 软链。
4. **安全护栏**：`go2w-safety-observer` 只读安全快照持久化；`go2w-sport-gateway` 0.25s 空闲零保持 + 断连持续零（租约跨策略重启存活）；`nav_health_supervisor` 监控导航健康；电池 <20% 强制 FAULT。
5. **已知性能瓶颈**：`map_odom_fuser.py`（Python）输出仅 ~0.6Hz，低于 bringup 要求的 5Hz，2026-08 起 bringup 跳过该检查（见 `bringup_slam_nav2.sh`；待实车复核）。
6. **API 鉴权现状**：`nx_control_auth.authorize_request` 自 2026-07-16 被用户要求短路为 `auth_disabled`（局域网免 Token），其后 token 校验逻辑为不可达死代码；CORS 仍按 `GO2W_PANEL_ORIGINS` 白名单精确匹配。

## 6. 扩展指引

- **新增 Web 功能**：在 `web/` 下新增 `nx_*.py` 模块，由 `nx_web_server.py` 导入；前端在 `static/panel.html` 注册 WS 消息类型。
- **新增 ROS 节点**：放 `src/go2w_bridge/`（Python，直接运行）或 `src/go2w_nav/`（C++，colcon 编译）。
- **新增 systemd 服务**：在 `docker/` 写 `.service` + 部署脚本，并在 `docs/NX_REDEPLOY.md` 登记。
- **测试**：Python 单测 `web/tests/`、`docker/tests/`、`src/go2w_bridge/test/`；端到端验证 `web/scripts/verify_*.sh` 与 `tools/verify_*.py`。
