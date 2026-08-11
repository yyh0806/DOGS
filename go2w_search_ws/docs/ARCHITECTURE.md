# DOGS 系统架构文档

> 本文档描述 Go2W 轮足机器狗自主搜索系统的整体架构，作为代码阅读与开发的权威入口。
> 创建于 2026-08 优化分支（`codex/optimize-project-structure`）。

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

- **NX**：所有重活（感知、建图、规划、控狗）跑在机载 Jetson 上。
- **PC**：仅浏览器访问 `http://<NX_IP>:8000`，不跑任何 ROS 节点。
- **网络拓扑**：NX → 狗主控 `192.168.123.100↔.161`（USB-Ethernet）；NX → MID360 `192.168.1.200↔.160`；PC ↔ NX 走 Wi-Fi（RK3568 网段 192.168.1.x）。

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
狗自带 utlidar ──► /lidar/points (PointCloud2) ──► nx_sensor_node ──► /scan（诊断用）
```

### 3.2 控制链路

```
Web 面板点击地图 ──► /api/navigate ──► navigation_arbiter ──► Nav2 /navigate_to_pose
        Nav2 输出 /cmd_vel_nav ──► nx_motion_node（持 lease）──► SportGateway ──► 狗主控
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

## 4. systemd 服务依赖

```
livox-mid360-net ─► livox-mid360-driver ─► go2w-sensor ─► go2w-sport-gateway
                                                        └─► go2w-motion ─► go2w-web
                                          go2w-slam-nav（独立 bringup：FastLIO→mid360 bridge→fuser→Nav2→slam）
                                          costmap-bridge（独立，读 costmap 写 JSON）
```

- `go2w-web` 依赖 `go2w-motion`（`After=`）。
- `go2w-slam-nav` 为一次性 bringup 脚本，内部用 `systemd-run` 起 transient units（fastlio / mid360-nav-bridge / map-odom-fuser / nav2-3d / slam-online / map-padding / nav-health-supervisor）。

## 5. 关键设计决策（摘要）

1. **NX 中心化**：PC 不跑 ROS，所有节点同机 → DDS 走 UDP loopback（禁 SHM，见 `fastdds_udp.xml`），根治 SHM 损坏导致订阅静默失效。
2. **双雷达源**：MID360 供 SLAM+Nav2（`/scan_mid360`），狗自带 utlidar 供诊断（`/scan` 已停发）。前端 2026-08 起只渲染 `/scan_mid360`。
3. **原子发布**：`build_release.sh` 构建 content-addressed 发布包 → `deploy_release.sh` 原子切换 `current` 软链。
4. **安全护栏**：`go2w-safety-observer` 看门狗超时停狗；`nav_health_supervisor` 监控导航健康；电池 <20% 强制 FAULT。
5. **已知性能瓶颈**：`map_odom_fuser.py`（Python）输出仅 ~0.6Hz，低于 bringup 要求的 5Hz，2026-08 起 bringup 跳过该检查（见 `bringup_slam_nav2.sh`）。

## 6. 扩展指引

- **新增 Web 功能**：在 `web/` 下新增 `nx_*.py` 模块，由 `nx_web_server.py` 导入；前端在 `static/panel.html` 注册 WS 消息类型。
- **新增 ROS 节点**：放 `src/go2w_bridge/`（Python，直接运行）或 `src/go2w_nav/`（C++，colcon 编译）。
- **新增 systemd 服务**：在 `docker/` 写 `.service` + 部署脚本，并在 `docs/NX_REDEPLOY.md` 登记。
- **测试**：Python 单测 `web/test_*.py` / `docker/test_*.py`；前端契约 `web/test_*.js`；端到端验证 `tools/verify_*.sh`。
