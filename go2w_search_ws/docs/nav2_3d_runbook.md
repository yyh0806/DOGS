# Go2W 阶段D Nav2 服务端 Runbook (运维 SOP)

> NX 上部署 Nav2 服务端 stack 的步骤 + 阶段C (FAST_LIO) 就绪后的联调流程 +
> 常见 TF / costmap / lifecycle 问题排查。
>
> 适用: `src/go2w_nav/launch/nav2_3d.launch.py` + `config/nav2_params_3d.yaml`。
> 后置依赖: 阶段C FAST_LIO 装好 + 硬件 (MID360 + Go2W) 就绪才能实跑。

---

## 1. 前置检查 (阶段C 就绪判据)

阶段D 是纯服务端配置, 实跑要求阶段C 先就绪。逐项确认:

```bash
# (1) livox_ros_driver2 编译安装 (TECH_DECISIONS 第二节步骤)
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # 终端1
ros2 topic list | grep -E '/livox/(lidar|imu)'       # 应有 /livox/lidar + /livox/imu

# (2) FAST_LIO 编译安装 (Ericsii/FAST_LIO_ROS2)
ros2 launch fast_lio mid360.launch.py                # 终端2
ros2 topic list | grep -E '/Odometry|/cloud_registered'   # 应有 /Odometry (~100Hz)

# (3) TF 链通 (FAST_LIO 发 camera_init→body, 本 launch 加 TF 桥到 map→base_link)
ros2 run tf2_ros tf2_echo map base_link              # 应持续输出非零 transform

# (4) 阶段A 控狗节点 (nx_motion_node) + 阶段A/B/E web 节点已部署
ros2 run go2w_bridge nx_motion_node                  # 终端3
python3 web/nx_web_server.py                         # 终端4
```

任一项不满足 → 先回去补阶段C / 阶段A, 不要硬启 Nav2。

---

## 2. 启动顺序 (5 个终端, 见 spec 决策 4)

```bash
# 终端1: 雷达驱动 (阶段C launch)
ros2 launch livox_ros_driver2 msg_MID360_launch.py    # 发 /livox/lidar + /livox/imu

# 终端2: FAST_LIO (阶段C launch)
ros2 launch fast_lio mid360.launch.py                  # 发 /Odometry + camera_init→body TF

# 终端3: 阶段A 控狗 (订阅 /cmd_vel 控狗)
ros2 run go2w_bridge nx_motion_node

# 终端4: 阶段A/B/E web (Nav2 action client 在此进程内)
python3 web/nx_web_server.py

# 终端5: 阶段D Nav2 服务端 (本阶段产出)
ros2 launch go2w_nav nav2_3d.launch.py                # Nav2 + p2l + TF 桥 + lifecycle_manager
```

**顺序关键**: 终端1→2→3→4→5。Nav2 (终端5) 最后起, 启动时 lookup `map→base_link`
要靠 FAST_LIO 的 TF + 本 launch 的 TF 桥先就绪 (TimerAction 延迟 2s 起也是为此)。

---

## 3. 验证步骤 (启动后逐项确认)

### 3.1 action / node / topic 基本面

```bash
# (1) Nav2 action server 暴露 (阶段E 客户端 wait_for_server 才不超时)
ros2 action list | grep navigate_to_pose              # 应有 /navigate_to_pose

# (2) Nav2 节点齐全 + 不含 amcl (决策 3 红线)
ros2 node list | grep -E 'controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager_navigation'
ros2 node list | grep -i amcl                          # **应无输出** (amcl 不能起)

# (3) map→odom TF 发布者唯一 (只 FAST_LIO + TF 桥, 不能有 amcl/nav2)
ros2 topic info /tf -v                                 # publisher 列表不能含 amcl
```

### 3.2 感知链路 (pointcloud_to_laserscan + costmap)

```bash
# (4) /scan 有数据 (pointcloud_to_laserscan 工作 + TF 链通)
ros2 topic echo /scan --once                           # 应输出 LaserScan, ranges 非空

# (5) local_costmap / global_costmap 有障碍 (rviz 可视化)
ros2 run rviz2 rviz2                                   # 加 LocalPool/GlobalPool/costmap 显示
# 或:
ros2 topic echo /local_costmap/costmap --once         # data 数组应有非 0 值
```

### 3.3 端到端单点导航 (CLI 测试, 不走 web)

```bash
# (6) 单点导航: 狗真走到 (1.0, 0.0), 方向正确 (无乱转)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0},
   orientation: {z: 0, w: 1}}}}"
# 期望: status=SUCCEEDED, 狗前移 1m, 转向正确 (决策 5 零反转验证)
```

### 3.4 阶段E 联调 (浏览器端到端)

```bash
# (7) kill 阶段E 的 mock Nav2 (决策 5 解耦: 编排零改动切真 Nav2)
pkill -f mock_nav2_action.py

# (8) 浏览器 (PC) 开 http://<NX_IP>:8000, F12 Console, 输入:
#     "搜索客厅"  (经 /api/command) 或直接 fetch('/api/search_room?room=客厅',{method:'POST'})
# Console 应看到 type=search_room 的 phase 推进序列:
#   SELECT_ROOM → NAVIGATE → NAVIGATING → ARRIVED → SEARCH → DETECT → REPORT → DONE
# 地图上狗箭头随 Nav2 导航移动 (Nav2 发 /cmd_vel → 狗动 → /Odometry 变 → broadcast_loop 推 slam)
# 最终收到 type=mission_report (含 room/waypoints_visited/detections)
```

---

## 4. 常见故障排查

### 4.1 TF `map→base_link` 断链

**症状**: `ros2 run tf2_ros tf2_echo map base_link` 报 "could not find chain",
pointcloud_to_laserscan / scan 为空, costmap 全空, Nav2 启动报 TF lookup 失败。

**排查**:
1. FAST_LIO (终端2) 是否在跑? `ros2 topic hz /Odometry` 应 ~100Hz。
2. 本 launch 的 tf_bridge_map / tf_bridge_body 是否起?
   `ros2 node list | grep tf_` 应有 `tf_map_to_camera_init` + `tf_body_to_base`。
3. 链路分段 echo:
   `tf2_echo map camera_init` (TF 桥, 应 0,0,0 identity)
   `tf2_echo camera_init body` (FAST_LIO 发, 应非零)
   `tf2_echo body base_link` (TF 桥, 应 0,0,0)
   `tf2_echo odom <...>` → **注意**: 本 launch 临时方案 map→odom→base_link 是用
   `map→camera_init→body→base_link` 近似的 (即 map==camera_init, body==base_link),
   "odom" 在此方案下等价于 camera_init→body 的中间帧。阶段C 就绪后优化为标准 map→odom→base_link。

### 4.2 costmap 全空 (狗不避障)

**症状**: `ros2 topic echo /local_costmap/costmap` data 全 0, 狗规划穿过障碍。

**排查**:
1. `/scan` 是否有数据? (依赖 4.1 的 TF 链 + pointcloud_to_laserscan)
2. pointcloud_to_laserscan 的 `target_frame: base_link` 能否 lookup?
   (TF 链不通时 lookup 失败, scan 为空, 见 4.1)
3. `transform_tolerance: 0.05` 是否够? (MID360 + FAST_LIO 频率高, 一般够; TF 抖动可加到 0.1)
4. obstacle_layer / voxel_layer 的 topic 名对不对?
   `/scan` (来自 p2l) + `/livox/lidar` (MID360 原始)。
   若用狗自带 utlidar, voxel_layer.topic 改 `/utlidar/cloud_base`, p2l.cloud_in 也改。

### 4.3 狗乱转 / 反向 (Critical, 决策 5)

**症状**: 发 goal 后狗原地乱转, 或前进/转向方向反。

**排查** (rubric #23/#24):
1. `grep cmd_vel src/go2w_nav/launch/nav2_3d.launch.py`
   **应只有 yaml 引用注释, 无 remap 行**。出现
   `remappings=[('/cmd_vel', '/cmd_vel_reversed')]` 之类 = 立刻删。
2. `grep cmd_vel_topic src/go2w_nav/config/nav2_params_3d.yaml`
   应是 `/cmd_vel` (默认), 不能改成 `/cmd_vel_nav2` 之类。
3. 链路核实 (决策 5 表):
   Nav2 DWB `angular.z` 正=左转 (REP-103) → nx_motion_node `_vyaw = msg.angular.z` (不反转)
   → SDK `Move(x,y,vyaw)` z 正=左转 (SDK_CAPABILITIES §2.1)。**0 次反转**。
4. 若仍乱转, 实车用 LowState 读轮速确认 SDK Move 方向 (TECH_DECISIONS 第一节待验证项)。

### 4.4 Nav2 节点不 activate (goal 卡在等待)

**症状**: `ros2 action send_goal` 一直 pending, `ros2 lifecycle get /bt_navigator`
显示 `unconfigured` 而非 `active`。

**排查** (rubric #31-34):
1. lifecycle_manager_navigation 是否起? `ros2 node list | grep lifecycle_manager_navigation`
2. `autostart: True`? (nav2_3d.launch.py 的 lifecycle_manager parameters)
3. `node_names` 含 `bt_navigator`? (否则 /navigate_to_pose 不暴露)
4. `node_names` **不含 amcl**? (若含 amcl 但 amcl 节点没起, lifecycle_manager 卡住)
5. 看 lifecycle_manager 日志: `ros2 node info /lifecycle_manager_navigation`

### 4.5 goal 被拒 (frame 不匹配)

**症状**: `ros2 action send_goal` 立即 reject, Nav2 报 "goal frame mismatch"。

**排查** (rubric #1/#37):
1. `grep global_frame src/go2w_nav/config/nav2_params_3d.yaml` 在 bt_navigator 段应 = `map`。
2. goal pose 的 `header.frame_id` 应 = `map` (与 rooms.yaml 的 frame_id 一致)。
3. 若 goal 用了 `odom` 或 `base_link`, 改成 `map` 再发。

### 4.6 Nav2 + 阶段A 手柄 /cmd_vel 冲突

**症状**: 导航中按手柄 (nx_web /api/move) 不响应, 或反过来。

**原因** (spec §8.3): Nav2 controller_server 和 nx_web 都发 `/cmd_vel`,
DDS 多 publisher, nx_motion_node 收到"谁后发谁覆盖"。

**约定** (阶段D 不实现仲裁, 文档约定):
- Nav2 导航中 (RoomSearchOrchestrator 的 NAVIGATING/SEARCH 阶段), **用户自觉别按手柄**。
- 要干预先 `curl -X POST /api/e_stop` 取消导航, 再手柄控。
- 未来 Sprint 加 `/cmd_vel_nav2` vs `/cmd_vel_manual` 优先级仲裁。

---

## 5. 故障注入 (验证 Nav2 的报错路径)

```bash
# (1) kill FAST_LIO → Nav2 报 TF 断链, costmap 停止更新, goal 卡 NAVIGATING
pkill -f fast_lio
ros2 topic echo /tf  # map→camera_init 停 (FAST_LIO 不发了)
# 恢复: 重启 FAST_LIO, TF 桥 + Nav2 自动恢复

# (2) kill pointcloud_to_laserscan → costmap obstacle_layer 无源, 全空
ros2 node kill /pointcloud_to_laserscan
ros2 topic echo /scan  # 无新消息
# 注意: voxel_layer 还吃 /livox/lidar (双保险), 不会完全瞎
# 恢复: ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node (或重启 launch)

# (3) kill lifecycle_manager → Nav2 节点停在 active 不再受管 (但不影响已激活节点)
ros2 node kill /lifecycle_manager_navigation
# 恢复: 重启 launch
```

---

## 6. 阶段C 就绪后的 TF 桥优化 (本 launch 临时方案的退役)

当前 TF 桥 (nav2_3d.launch.py 的 tf_bridge_map / tf_bridge_body) 是临时 identity 桥接。
阶段C FAST_LIO 配置就绪后, 二选一优化:

**方案 A (推荐, 改 FAST_LIO 配置)**: FAST_LIO 的 frame 名参数
`camera_init → map`, `body → odom`, 另发 `odom → base_link`。
删掉本 launch 的两个 tf_bridge 节点。

**方案 B (用 robot_localization EKF)**: FAST_LIO 发 `odom → base_link`,
EKF 融合发 `map → odom`。删 tf_bridge_map, 保留 tf_bridge_body 或改 EKF 配置。

退役后本 launch 顶部 "TF 桥临时方案" 注释段落一并删除。

---

## 7. 相关文件

- `src/go2w_nav/config/nav2_params_3d.yaml` — Nav2 全栈参数 (本 runbook 的核心配置)
- `src/go2w_nav/launch/nav2_3d.launch.py` — Nav2 服务端 launch (本 runbook 的启动入口)
- `docs/TECH_DECISIONS.md` 第三节 — Nav2 3D 决策依据 (路线A + VoxelLayer 双保险)
- `gan-harness/spec-stage-d.md` — 阶段D 完整规格
- `gan-harness/spec-stage-e.md` — 阶段E Nav2 客户端 (本服务端的消费方)
- `docs/room_calibration.md` — 房间标定 SOP (rooms.yaml 的 pose 来源)
