# 仿真 ↔ 真机切换指南 (spec 2026-07-25-real-fidelity-simulation-design)

目标: 仿真栈与真机除"狗的运动模型"外完全一致, 仿真验证过的代码上真机可直接用。

## 仿真全栈启动 (PC WSL2 Ubuntu-22.04 + Gazebo Classic 11 + ROS2 Humble)

```bash
source ~/go2w_ws/install/setup.bash
ros2 launch go2w_sim sim_full_bringup.launch.py
```

- 默认 world = `indoor_rooms.world` (4 房间 + 中央十字门洞, frontier 探索有内容)
- 切 MVP 单房: `world:=$(ros2 pkg prefix go2w_sim)/share/go2w_sim/worlds/indoor_empty.world`
- 浏览器: http://localhost:8000 (panel 状态灯/点选导航) / ws://localhost:8001 (实时推送)
- launch 自动设 `GO2W_SIM=1` + `FASTRTPS_DEFAULT_PROFILES_FILE` (禁 SHM)

## 启动的模块 (全部 real 代码, 非桩)

| 模块 | 仿真来源 | 真机来源 |
|------|----------|----------|
| FastLIO | `fast_lio mapping.launch.py` (mid360.yaml) | 同 (systemd mid360-nav-bridge) |
| Livox CustomMsg | `libros2_livox.so` (gzserver 插件) | `livox_ros_driver2` (实机 MID360) |
| Nav2 | `nav2_bringup bringup_launch.py` | 同 (systemd nav2-3d) |
| motion_machine | `nx_motion_node` (real) | 同 (systemd go2w-motion) |
| web + ExplorationManager | `nx_web_server.py` (real) | 同 (systemd go2w-web) |
| TF odom→base | `sim_odom_tf` (FastLIO /Odometry → TF) | FastLIO 直接发 |

## 唯一简化 (动力学层, 用户允许)

- 真机: `nx_motion_node` → `SportGatewayClient` (sport lease socket) → 真实电机
- 仿真: `nx_motion_node GO2W_SIM=1` → `SimSportGateway` (Effect → Twist → `/cmd_vel` → `libgazebo_ros_planar_move`)
- `planar_move` 是简化运动模型 (仅 vx + wz, 忽略关节/俯仰)
- `SimTelemetryBridge`: `/odom_planar` → `/wheel_feedback` (sport_mode 速度推断:
  停→6/JOINT_LOCK 推进 BOOT_HOLD→PARKED; 动→3/WHEEL 推进 ACTIVATING→NAV_ACTIVE)

## 上真机 checklist

1. NX 部署 (`deploy_nx.sh`): `~/go2w_ws/` 裸 python (`web/`) + colcon install (`go2w_bridge` / `go2w_nav`)
2. systemd 起: `mid360-nav-bridge` / `nav2-3d` / `go2w-motion` / `go2w-web`
3. **不设 `GO2W_SIM`** → `nx_motion_node` 走真机 `SportGatewayClient` (socket → sport lease)
4. 真机 `/wheel_feedback` 来自 `nx_sensor_node` (LowState 反推, 非 `SimTelemetryBridge`)
5. 真机视频: C13 RTSP `192.168.144.108` + Go2 VideoClient
6. FastRTPS SHM 真机也坑 (memory `slam-nav2-bringup-gotchas`), 建议 systemd env 设 `FASTRTPS_DEFAULT_PROFILES_FILE`

## 关键验证点 (仿真)

- Task2: `/Odometry` 静止三轴 max<0.02m (FastLIO ESKF 收敛) ✅
- Task3: `SimSportGateway` 7/7 契约测试 ✅
- `nx_web_server` :8000 HTTP 200 + /api/status JSON ✅ (阶段B AI + 阶段E RoomOrchestrator 注入)
- `sim_telemetry_bridge` 单独 `/wheel_feedback` PUB_COUNT=1, WF_RX~140/7s ✅
- 全栈 `/wheel_feedback` 跨进程 DDS 发现 + `dog_state` BOOT_HOLD→PARKED 推进 ✅
  (session=parked, physical_mode=joint_lock, actual_motion=stopped, telemetry_fresh=true, motion_service=ai-w, fault=null)

## Nav2 初始定位 (启动后执行一次)

amcl 需初始 pose 才发 map→odom TF (否则 costmap 报 `Invalid frame ID "map"`). 真机同理 (操作员 rviz/panel 设):

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}}}"
```

发后 amcl 收敛, global_costmap 正常, 即可点选导航 (`/goal_pose` → Nav2 → `/cmd_vel` → motion → SimSportGateway → planar_move).

## 已知限制 (WSL2 硬件)

- Gazebo Classic RTF~0.01 (WSL2 CPU 争用), 狗物理到达目标耗时 (非模块缺陷)
- 仿真时间非确定 (WSL2 调度), 不适合闭环时序硬实时测试
- 视频/YOLO 帧仿真不提供 (前端显示"等待视频"); C13 RTSP 桥仿真下连不上退化 (主服务不受影响)
