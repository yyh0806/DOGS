# Product Specification: Go2W 阶段D — Nav2 自主导航（服务端配置）

> Generated from brief: "为 Go2W 阶段D：Nav2 自主导航（服务端配置）产出完整实现规格（GAN Planner 角色）"
> 主管角色: GAN Planner（架构/规格，不含 yaml/launch 代码实现——由 Generator 写）
> 状态: 待 Generator 实现，待 Critic 静态审
> 范围: **纯软件先写配置**（硬件装完 + 阶段C FAST_LIO 就绪后实跑）。本阶段产出 Nav2 服务端 stack 的参数文件 + launch 文件 + TF 桥接方案，**静态审对照 Nav2 Humble 官方规范 + TECH_DECISIONS 第三节**，不实车。
> 前置: 阶段A（web 上移 NX，已 gan 收敛）+ 阶段B（AI 上移 NX，已 gan 收敛）+ 阶段E（房间搜索编排，已 gan 收敛，含 Nav2 **客户端**）
> 后置依赖: 阶段C（FAST_LIO 3D SLAM）就绪后才能实跑 Nav2（FAST_LIO 发 `/Odometry` + `map→odom` TF）

---

## 0. 规格阅读约定

- 所有路径均为**相对仓库根** `go2w_search_ws/`。
- 每个文件给三段：**职责 / 关键内容要点 / 实现约束**。要点是契约，约束是红线。
- **本阶段不写 Python 代码**（yaml/launch 由 Generator 写，本 spec 只定结构和参数表）。
- **阶段A/B/E 红线继续生效**：
  - 禁止改 `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py` 的现有逻辑（阶段E 已 gan 收敛，D 服务端要**兼容**阶段E 的客户端契约：同 action 名 `/navigate_to_pose`、同 TF 树 `map→odom→base_link`、同 frame_id `map`）
  - 禁止改 `nx_motion_node.py` / `nx_sensor_node.py` 的现有逻辑（阶段A 红线）
- **阶段D 新红线**：
  - **不启动 amcl**（FAST_LIO 已替代定位职能，amcl 会抢 `map→odom` 发布权——Critical）
  - **不启动 slam_toolbox**（与 FAST_LIO 互斥，二者都发 `map→odom` 会 TF 冲突——Critical）
  - **不复用 `src/go2w_bringup/launch/search.launch.py`** 的 Nav2 段（那是 2D + slam_toolbox 路线，与阶段D 3D + FAST_LIO 路线冲突；search.launch.py 的 pointcloud_to_laserscan 参数可参考但**不直接复用**其 Nav2 lifecycle 节点列表——它用了已废弃的 `nav2_recoveries/recoveries_server`，Humble 已迁移到 `nav2_behaviors`）
  - **不自己发 `/odom` 或 `map→odom` TF**（FAST_LIO 负责，Nav2 只消费）
  - **Nav2 lifecycle 不激活 amcl / map_server / slam_toolbox**（这些是 2D 路线组件，3D 路线不用）

---

## 1. Vision（阶段D 目标态）

载荷 NX 上跑一套 **Nav2 服务端 stack**（controller_server / planner_server / behavior_server / bt_navigator / waypoint_follower / lifecycle_manager），吃 FAST_LIO 的 `/Odometry` 做定位、吃 MID360 点云（经 `pointcloud_to_laserscan` 转 `/scan` + local_costmap 直接吃 PointCloud2 双保险）做避障，发 `/cmd_vel` 给 `nx_motion_node` 控狗走到目标点。

阶段E 的 `RoomSearchOrchestrator` 已经实现了 Nav2 **客户端**（`nav2_msgs/action/NavigateToPose` action client，发 `/navigate_to_pose` goal）。阶段D 的服务端配置要让这个 goal **被处理**：bt_navigator 接 goal → planner_server 全局规划 → controller_server 局部控制 → 发 `/cmd_vel` → nx_motion_node 控狗 → FAST_LIO 反馈位姿 → 到达。

一句话验收：**NX 上启动 livox_ros_driver2 + FAST_LIO（阶段C）+ 阶段D 的 nav2_3d.launch.py + nx_motion_node（阶段A）+ nx_web_server（阶段A/B/E），`ros2 action list` 含 `/navigate_to_pose`，`ros2 run tf2_tools view_frames` 显示完整 TF 树 `map→odom→base_link`（无断链、无多源发布），浏览器发"搜索客厅"后狗真走到客厅入口并完成房间内覆盖搜索。**（实跑依赖阶段C + 硬件，本阶段只交付配置 + 静态审）

---

## 2. 阶段E（客户端） → 阶段D（服务端）契约对齐（Generator 必读）

阶段E 已 gan 收敛，D 服务端**必须兼容**阶段E 客户端的以下契约（否则破坏阶段E）：

| 契约项 | 阶段E 客户端约定（已固化） | 阶段D 服务端必须满足 | 出处 |
|---|---|---|---|
| **Action 名** | `/navigate_to_pose`（`nx_room_orchestrator.py:287` 默认值） | bt_navigator 必须暴露 `/navigate_to_pose` action server（Nav2 默认即此名，**不要 remap**） | spec-stage-e 决策 1 |
| **Action 类型** | `nav2_msgs/action/NavigateToPose` | Nav2 Humble 自带，版本对齐 | 同上 |
| **Goal frame_id** | `rooms.yaml` 的 `frame_id`（默认 `map`，`nx_room_orchestrator.py:345` 默认参数） | Nav2 `bt_navigator.global_frame` 必须 = `map`（与 goal frame 一致，否则 `bt_navigator` 拒绝 goal 报 frame 不匹配） | spec-stage-e §6.1 |
| **Goal pose** | `PoseStamped`：`position.x/y` + `orientation`（yaw→四元数 `qz=sin(yaw/2), qw=cos(yaw/2)`，REP-103） | Nav2 标准接收，无需特殊处理 | nx_room_orchestrator.py:360-363 |
| **TF 树** | 假设 `map→odom→base_link` 完整（goal 在 map frame，Nav2 要能 lookup `map→base_link`） | FAST_LIO + TF 桥提供 `map→odom→base_link`（见 §7） | spec-stage-e §2 |
| **Feedback** | 读 `NavigateToPose.Feedback.distance_remaining`（`nx_room_orchestrator.py` progress 推送用） | Nav2 bt_navigator 标准 feedback，无需特殊配置 | 同上 |
| **Result status** | `status==4`（STATUS_SUCCEEDED）判到达（`nx_room_orchestrator.py:271` 注释） | Nav2 标准 GoalStatus，到达发 SUCCEEDED | 同上 |
| **Cancel** | `handle.cancel_goal_async()`（`nx_room_orchestrator.py` cancel_current） | Nav2 标准 cancel 接口 | 同上 |
| **超时** | goal 接受 5s，导航完成 120s（`nx_room_orchestrator.py:287` 默认） | Nav2 必须在 120s 内完成单航点导航（室内 + 0.6m/s 速度 + 5m costmap 足够；若规划失败及时 abort 而非挂起） | 同上 |

**结论**：阶段D 服务端**零改动阶段E 客户端代码**，只需让 Nav2 stack 的 action server / TF / frame 对齐阶段E 假设。kill 阶段E 的 `mock_nav2_action.py`，启动阶段D 的真 Nav2，阶段E 编排自动用真 Nav2（spec-stage-e 决策 5 的解耦价值）。

---

## 3. 关键设计决策（已拍板，给推荐 + 理由，全部基于 TECH_DECISIONS 第三节）

### 决策 1：参数文件组织 → **推荐 (b) 复用并更新 `src/go2w_nav/config/nav2_params_3d.yaml`（休眠文件，已有雏形但有多处错误需修正）**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 新建 `nav2_params_3d.yaml`（与休眠文件重名覆盖） | 干净起点 | 丢弃休眠文件已有的大量正确参数（critics/footprint/Go2W 室内参数），重复劳动；git 历史断裂 | ❌ 不选 |
| **(b) 复用并更新 `src/go2w_nav/config/nav2_params_3d.yaml`** | 休眠文件已 80% 正确（footprint/Go2W 室内参数/差速起步/VoxelLayer 雏形都在），只需修正错误项（见 §6.1 修正清单）；与 TECH_DECISIONS 第三节"路线A + VoxelLayer 双保险"已对齐；git 历史连续 | 需识别并修正休眠文件的错误（`odom_topic`、local_costmap 缺 ObstacleLayer、TF 桥粗糙等） | ✅ **推荐** |
| (c) 保留休眠 `nav2_params.yaml`（2D + slam_toolbox 路线）做主文件 | 与 bringup search.launch.py 一致 | 违反 TECH_DECISIONS 第三节"不用 amcl/slam_toolbox，用 FAST_LIO"；2D 路线已废弃 | ❌ 不选，违反决策 |

**理由总结**：(b) 是"修正而非重写"。休眠 `nav2_params_3d.yaml` 的 footprint `[ [0.30,0.20], [0.30,-0.20], [-0.25,-0.20], [-0.25,0.20] ]`、`max_vel_x: 0.6`、`max_vel_theta: 1.0`、`xy_goal_tolerance: 0.20`、`yaw_goal_tolerance: 0.15`、差速起步（`max_vel_y: 0.0, vy_samples: 0`）**全部正确**（与 TECH_DECISIONS 第三节逐字对齐）。错误项集中在：① `odom_topic: /Odometry`（FAST_LIO 原生 topic 名对，但 frame 不对——见 §7 TF 桥）；② local_costmap 只有 VoxelLayer 没 ObstacleLayer（违反"双保险"——TECH_DECISIONS 第三节明确"local_costmap 额外加 VoxelLayer 直接吃 PointCloud2"，意为 ObstacleLayer 吃 /scan 为主 + VoxelLayer 吃点云为辅，休眠文件理解反了）；③ global_costmap 没 VoxelLayer（全局规划也应能吃点云，至少 ObstacleLayer 吃 /scan）；④ `behavior_server` 用 `nav2_behaviors` 包名正确（休眠文件这部分对，bringup search.launch.py 的 `nav2_recoveries` 是旧名已废弃）；⑤ 缺 `velocity_controller` / `velocity_smoother`（可选，见 §6.1 决策 7）。

### 决策 2：costmap 路线 → **推荐 路线A（pointcloud_to_laserscan → /scan → ObstacleLayer）为主 + VoxelLayer 直接吃 PointCloud2 为辅（双保险）**

TECH_DECISIONS 第三节已拍板"路线A + VoxelLayer 双保险"。具体配置：

| costmap | layers（从下到上叠加顺序） | 数据源 | 理由 |
|---|---|---|---|
| **global_costmap** | `static_layer`（静态地图，可选——FAST_LIO 不发 OccupancyGrid，此层可禁用或留空地图） + `obstacle_layer`（吃 `/scan`，ObstacleLayer，2D 射线追踪） + `inflation_layer`（膨胀） | `/scan`（来自 pointcloud_to_laserscan） | 全局规划用 2D 投影足够；static_layer 在无 occupancy map 时靠 obstacle_layer 实时累积 |
| **local_costmap** | `obstacle_layer`（吃 `/scan`，ObstacleLayer，主） + `voxel_layer`（吃 `/livox/lidar` PointCloud2，VoxelLayer，辅——3D 体素更精细抓桌腿/人腿） + `inflation_layer`（膨胀） | `/scan` + `/livox/lidar` | 双保险：ObstacleLayer 2D 快速、VoxelLayer 3D 精细；rolling_window 5×5m 实时避障 |

**关键约束**：
1. **local_costmap 的 obstacle_layer 必须有**（休眠文件删了，错误）——它是 2D 射线追踪的主力，比 VoxelLayer 快。VoxelLayer 是"辅"，处理 ObstacleLayer 漏检的低矮/细长障碍（桌腿）。
2. **layer 顺序**：`plugins` 数组顺序决定叠加顺序，底层（static/obstacle）在前，上层（inflation）在后。`inflation_layer` 必须最后（它在所有障碍层之上膨胀）。
3. **VoxelLayer 的 `max_obstacle_height: 1.5` + `min_obstacle_height: 0.05`**：与 SDK_CAPABILITIES 的 MID360 高度过滤 `-0.1~1.5m` 对齐（VoxelLayer 的 min 稍高 0.05 避免地面噪声）。
4. **ObstacleLayer 的 `obstacle_max_range: 7.0` + `obstacle_min_range: 0.2`**：与 SDK_CAPABILITIES 的 MID360 距离过滤 `0.15~8.0m` 对齐（稍保守）。
5. **pointcloud_to_laserscan 的 `min_height: 0.10, max_height: 1.20`**：切 0.1~1.2m 高度的点投影成 2D scan（抓桌腿/人腿/墙，剔地面和头顶）。休眠 nav2_3d.launch.py 已用此值（正确）。**注意**：与 VoxelLayer 的 `min_obstacle_height: 0.05` 略不同——pointcloud_to_laserscan 切的是"投影成 scan 的点"，VoxelLayer 切的是"直接进体素的点"，两者独立。

### 决策 3：localization 不用 amcl → **推荐 lifecycle 不激活 amcl，`map→odom` 由 FAST_LIO 发布**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 启动 amcl 做 localization | Nav2 官方教程默认 | **amcl 发 `map→odom` TF**，与 FAST_LIO 抢发布权 → TF 跳变/狗乱动；amcl 需要 OccupancyGrid 地图（FAST_LIO 不发）；amcl 是 2D 粒子滤波，3D 点云建图场景不适合 | ❌ 不选，TECH_DECISIONS 第三节明确"不用 amcl" |
| **(b) 不启动 amcl，`map→odom` 由 FAST_LIO 发布** | FAST_LIO 已做 3D LIO 定位，发 `/Odometry`（`camera_init→body`）；通过 TF 桥（阶段C 配置）改名发布 `map→odom`；Nav2 只消费 `/Odometry` 做 controller 的速度反馈，不抢 TF | 需要阶段C 的 FAST_LIO + TF 桥正确配好（`camera_init`→`map`、`body`→`base_link` 静态桥，或 EKF 拆分） | ✅ **推荐** |
| (c) 启动 amcl 但禁用其 TF 发布（`set_initial_pose` only） | 兼容 Nav2 教程 | 复杂、无收益（FAST_LIO 已提供定位）；amcl 的粒子滤波在无 occupancy map 时无意义 | ❌ 不选 |

**理由总结**：(b) 是 TECH_DECISIONS 第三节拍板方案。**Nav2 lifecycle 配置关键**：`lifecycle_manager_navigation` 的 `node_names` 列表**不含 amcl**（休眠 nav2_3d.launch.py 用的是 `nav2_bringup/navigation_launch`，它默认**不启 amcl**——amcl 是 `nav2_bringup/bringup_launch` 才启的；用 `navigation_launch` 即可避开 amcl）。`bt_navigator.global_frame: map` + `odom_topic: /Odometry`（FAST_LIO 发的），Nav2 自己**不发** `map→odom`，靠 FAST_LIO + TF 桥提供。

**Critical 检查**：`ros2 topic info /tf` 的 publisher 列表里**不能有 amcl / lifecycle_manager / nav2 任何节点发 `map→odom`**——只有 FAST_LIO（或其 TF 桥）发。Critic 静态审必须确认 launch 文件不启动 amcl。

### 决策 4：launch 组织 → **推荐 (b) 复用并更新 `src/go2w_nav/launch/nav2_3d.launch.py`（休眠雏形），不集成进 NX 统一启动**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 复活 `src/go2w_nav/launch/nav2.launch.py`（2D + slam_toolbox） | 与 bringup 一致 | 违反 TECH_DECISIONS（2D 路线废弃）；slam_toolbox 与 FAST_LIO 冲突 | ❌ 不选 |
| **(b) 复用并更新 `src/go2w_nav/launch/nav2_3d.launch.py`** | 休眠文件已有 pointcloud_to_laserscan + TF 桥 + Nav2 bringup 雏形；只启 Nav2 + p2l + TF 桥（不含 livox/FAST_LIO/nx_motion，那些是阶段A/C 独立 launch），职责单一；与阶段A 的 nx_web/nx_motion/nx_sensor launch **并存不冲突**（不同 ROS2 node 名、不同 topic 发布权） | 休眠文件的 TF 桥用 `static_transform_publisher`（map↔camera_init、body↔base_link）是粗糙近似——阶段C FAST_LIO 装好后应改用 EKF 或 FAST_LIO 配置直接发对 frame（见 §7） | ✅ **推荐** |
| (c) 把 Nav2 集成进 NX 统一启动（一个 launch 起所有） | 一键启动 | 单个 launch 文件巨大、调试困难（Nav2 起不来时不知是 Nav2 还是 FAST_LIO 问题）；违反"分阶段独立验证"原则；阶段A/B/E 都是独立 launch | ❌ 不选，过度集中 |

**理由总结**：(b) 是"独立 launch + 人工编排启动顺序"。NX 上跑全套的启动顺序（写入 docs/nav2_3d_runbook.md，运维 SOP）：
```
# 终端1: 雷达驱动 (阶段C 提供 launch)
ros2 launch livox_ros_driver2 msg_MID360_launch.py    # 发 /livox/lidar + /livox/imu

# 终端2: FAST_LIO (阶段C 提供 launch)
ros2 launch fast_lio mid360.launch.py                  # 发 /Odometry + camera_init→body TF

# 终端3: 阶段A 控狗
ros2 run go2w_bridge nx_motion_node                    # 订阅 /cmd_vel 控狗

# 终端4: 阶段A/B/E web
python3 web/nx_web_server.py                           # Nav2 客户端在此进程内

# 终端5: 阶段D Nav2 服务端 (本阶段产出)
ros2 launch go2w_nav nav2_3d.launch.py                 # Nav2 + p2l + TF 桥
```
**关键约束**：
1. **不启动 livox_ros_driver2 / FAST_LIO / nx_motion_node / nx_web**（这些是阶段A/C 的 launch，阶段D 的 nav2_3d.launch.py 只管 Nav2 + p2l + TF 桥）。
2. **TF 桥的 static_transform_publisher 是临时方案**——阶段C FAST_LIO 装好后，FAST_LIO 可配置直接发 `map→odom`（而非 `camera_init→body`），届时删掉 static_transform_publisher。Generator 在 nav2_3d.launch.py 顶部注释写明此依赖关系。
3. **与 nx_web 共存**：nx_web 的 NxWebNode 发 `/cmd_vel`（手柄控狗），Nav2 的 controller_server 也发 `/cmd_vel`（自主导航）——**两个 publisher 共存于同一 topic**，DDS 层面不冲突，但**语义冲突**：谁在控狗？见 §8 `/cmd_vel` 仲裁。

### 决策 5：/cmd_vel 坐标核实 → **推荐 结论：Nav2 输出与 nx_motion_node 期望约定一致，无双重反转**

这是 **Critical**（双重反转=狗乱转）。逐层核实：

**Nav2 controller_server 输出（DWBLocalPlanner）**：
- 发布 `geometry_msgs/Twist` 到 `/cmd_vel`
- 约定（ROS REP-103 + Nav2 标准）：`linear.x` = 前进速度（正=前进）、`linear.y` = 横移（差速为 0）、`angular.z` = 角速度（正=左转/逆时针）
- DWB 的 `max_vel_x: 0.6, max_vel_theta: 1.0, max_vel_y: 0.0`（差速）→ 只发 `linear.x` 和 `angular.z`

**nx_motion_node 消费（`nx_motion_node.py:114-123`）**：
```python
def _on_cmd_vel(self, msg):
    # 坐标系: vx前后 vy左右 vyaw旋转(正=左转)
    # 实测 Go2W SDK Move(x,y,z): z正=左转 (与cmd_vel angular.z约定一致, 无需反转)
    self._vx = msg.linear.x      # 前后
    self._vy = msg.linear.y      # 左右 (差速为0)
    self._vyaw = msg.angular.z   # 旋转, 正=左转
```
- `_ctrl_loop:179` 调 `self._sport.Move(vx, vy, vyaw)` → SDK Move(x, y, z)
- SDK_CAPABILITIES.md §2.1 实测：`Move(0,0,vyaw)` vyaw 正=左转 ✓
- **结论**：`/cmd_vel.angular.z`（Nav2，正=左转）→ `nx_motion_node._vyaw`（正=左转）→ `Move(0,0,vyaw)`（正=左转）。**三层透传，零反转**。

**双重反转风险点排查**：
| 风险点 | 是否反转 | 证据 |
|---|---|---|
| Nav2 DWB `angular.z` 正负 | 正=左转（REP-103） | Nav2 Humble 源码 dwb_core 默认约定 |
| nx_motion_node `_on_cmd_vel` | 不反转（直接 `self._vyaw = msg.angular.z`） | nx_motion_node.py:120 + 注释 |
| nx_motion_node `Move(vx,vy,vyaw)` | 不反转（SDK 实测 z 正=左转） | SDK_CAPABILITIES.md §2.1 + nx_motion_node.py:117 注释 |
| **合计** | **0 次反转** | **狗转向正确** |

**与阶段A 手柄控狗的对比**：nx_web_server.py:288 `publish_cmd_vel(vx,vy,vyaw)` 注释明确"前端 vyaw 正=左转, /cmd_vel.angular.z 正=左转 (ROS REP-103), 直接透传不反转"——阶段A 已经走通"前端→/cmd_vel→nx_motion_node→狗"链路且零反转。Nav2 走的是**同一条 `/cmd_vel` 链路的后半段**（Nav2→/cmd_vel→nx_motion_node→狗），前半段（Nav2 内部规划→Twist）已是 REP-103 标准。**因此 Nav2 链路与阶段A 链路在 `/cmd_vel` 之后的语义完全一致，零额外反转**。

**Critical 结论（写入 eval-rubric-stage-d）**：Nav2 controller_server 发的 `/cmd_vel` 直接被 nx_motion_node 正确消费，**不需要任何 remap 或反转**。Generator **禁止**在 launch 里加 `remappings=[('/cmd_vel', '/cmd_vel_reversed')]` 之类的反转操作。Critic 静态审必须确认 launch 文件的 controller_server 节点**无 /cmd_vel remap**（或 remap 目标仍是 `/cmd_vel`）。

### 决策 6：pointcloud_to_laserscan 参数 → **推荐 休眠 nav2_3d.launch.py 的值（已对齐 SDK_CAPABILITIES）**

休眠 nav2_3d.launch.py 的 pointcloud_to_laserscan 参数：
```yaml
target_frame: base_link          # 输出 scan 的 frame (转成 base_link 系, costmap 直接用)
transform_tolerance: 0.05
min_height: 0.10                 # 切 0.1m 以上点 (剔地面)
max_height: 1.20                 # 切 1.2m 以下点 (剔头顶, 抓桌腿/人腿/墙)
angle_min: -3.14159              # 全周
angle_max: 3.14159
angle_increment: 0.0087266       # 0.5° 分辨率 (~720 点/scan, MID360 够密)
scan_time: 0.1                   # 10Hz scan
range_min: 0.2                   # 与 SDK_CAPABILITIES 的 0.15m 对齐 (稍保守)
range_max: 20.0                  # MID360 最远 ~40m, 室内 20m 够
use_inf: True                    # 无回波用 inf (costmap 正确处理)
inf_epsilon: 0.001
```

**对照 SDK_CAPABILITIES.md §1.3 的 MID360 特征**：
| 参数 | 休眠值 | SDK_CAPABILITIES 实测 | 是否对齐 |
|---|---|---|---|
| 高度过滤 | min 0.10 / max 1.20 | -0.1 ~ 1.5m 视为障碍 | ✓（scan 切 0.1~1.2m 抓主要障碍，VoxelLayer 用 0.05~1.5m 补） |
| 距离过滤 | range 0.2 ~ 20.0 | 0.15 ~ 8.0m 有效 | ✓（scan range_max 20 比 8 大，但 costmap 的 `obstacle_max_range: 7.0` 会再裁一次） |
| 角度分辨率 | 0.0087266 rad (0.5°) | MID360 ~360 个扫描线 | ✓（720 点/scan 足够密） |

**推荐**：复用休眠值，不改。`target_frame: base_link` 是关键——pointcloud_to_laserscan 把点云转到 base_link 系再切高度投影，这样狗转弯时 scan 跟着 base_link 转（local_costmap 的 rolling_window 需要）。**注意**：这要求 TF 树 `map→odom→base_link` 完整（FAST_LIO + TF 桥提供），否则 pointcloud_to_laserscan 的 `transform_tolerance: 0.05` 内 lookup 失败 → scan 为空 → costmap 无障碍 → 狗撞墙。Critic 静态审必须确认 TF 链完整。

### 决策 7：velocity_smoother → **推荐 不加（DWB 内置速度约束足够，差速起步阶段保持简单）**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 加 `nav2_velocity_smoother` 节点 | 平滑加速度、限速、deadzone | 多一个节点 + lifecycle；DWB 已有 `acc_lim_x/acc_lim_theta` + `decel_lim`；Go2W 室内 0.6m/s 速度不需要额外平滑 | ❌ 不选，过度工程 |
| **(b) 不加，靠 DWB 的 acc_lim 约束** | 简单；DWB 的 `acc_lim_x: 0.5, acc_lim_theta: 2.0, decel_lim_x: -1.0` 已限制急停急转；nx_motion_node 的看门狗 1s 超时停狗是最后保险 | 加速度边界靠 DWB 参数调，需实测微调 | ✅ **推荐** |

**理由**：差速起步阶段（max_vel_y=0.0）目标是"跑通"，velocity_smoother 是优化项。Go2W 轮式狗对急停敏感（TECH_DECISIONS §一 "重复命令导致红色保护模式"），但 DWB 的 acc_lim 已足够柔和。**未来 Sprint**（全向模式 max_vel_y=0.3）再评估加 velocity_smoother。

---

## 4. 关键设计决策总表（Generator 速查）

| # | 决策点 | 推荐 | 一句话理由 |
|---|---|---|---|
| 1 | 参数文件组织 | 复用更新 `nav2_params_3d.yaml`（休眠文件 80% 正确，修正错误项） | 避免重复劳动，git 历史连续 |
| 2 | costmap 路线 | global: static+obstacle(/scan)+inflation；local: obstacle(/scan)+voxel(/livox/lidar)+inflation | 双保险（TECH_DECISIONS 第三节） |
| 3 | localization | 不启动 amcl，FAST_LIO 发 map→odom | TECH_DECISIONS 第三节，避免抢 TF |
| 4 | launch 组织 | 复用更新 `nav2_3d.launch.py`，独立 launch 不集成统一 | 分阶段独立验证，与阶段A/C launch 并存 |
| 5 | /cmd_vel 坐标 | 零反转，Nav2 直发 /cmd_vel 给 nx_motion_node | 三层透传已核实（REP-103 全程一致） |
| 6 | pointcloud_to_laserscan 参数 | 复用休眠值（min 0.10/max 1.20/range 0.2~20） | 对齐 SDK_CAPABILITIES MID360 特征 |
| 7 | velocity_smoother | 不加（DWB acc_lim 够） | 差速起步保持简单 |

---

## 5. 新建/修改文件清单（文件级 + 内容要点，Generator 直接实现）

### 5.1 修改文件

#### `src/go2w_nav/config/nav2_params_3d.yaml`（核心，休眠文件修正）
**职责**：Nav2 全栈参数（controller/planner/behavior/bt_navigator/waypoint_follower/costmap）。

**修正项**（休眠文件 → 正确值）：

| 节点 | 参数 | 休眠值（错） | 正确值 | 理由 |
|---|---|---|---|---|
| `bt_navigator` | `odom_topic` | `/Odometry` | `/odom`（**或**保留 `/Odometry` 但 launch 里 remap FAST_LIO 的 `/Odometry` → `/odom`） | Nav2 默认期望 `/odom`；FAST_LIO 发 `/Odometry`（大写）。推荐 launch 里 remap `/Odometry`→`/odom` 而非改 Nav2 默认 topic 名（Nav2 各节点默认都订 `/odom`，改一个不保险）。**Generator 二选一**：(i) `bt_navigator` + `controller_server` 的 `odom_topic` 都显式设 `/Odometry`（与 FAST_LIO 原生一致）；(ii) launch 里全局 remap。**推荐 (i)**——显式声明，避免 remap 链断裂。 |
| `bt_navigator` | `global_frame` | `map` | `map`（不变） | 与阶段E 客户端 goal frame 一致 |
| `bt_navigator` | `robot_base_frame` | `base_link` | `base_link`（不变） | 与 TF 树一致 |
| `bt_navigator` | `plugin_lib_names` | **缺失** | 必须加（Humble 的 bt_navigator 要显式声明 BT plugin 库列表） | Humble 版 bt_navigator 需 `plugin_lib_names` 列表（NavigateToPose/NavigateThroughPoses/ComputePathToPose/... 等 ~20 个 lib），否则 bt_navigator 启动报错找不到 plugin。从 Nav2 Humble 官方 nav2_humble_params.yaml 复制标准列表 |
| `controller_server` | 各速度/加速度 | 已对齐 TECH_DECISIONS | 不变 | 休眠值正确 |
| `local_costmap` | `plugins` | `["voxel_layer", "inflation_layer"]`（缺 obstacle_layer） | `["obstacle_layer", "voxel_layer", "inflation_layer"]` | 双保险：obstacle_layer 吃 /scan（主）+ voxel_layer 吃点云（辅） |
| `local_costmap.obstacle_layer` | **缺失** | — | 加 ObstacleLayer 吃 `/scan`（同 global_costmap 的 obstacle_layer 配置） | 主力 2D 障碍层 |
| `local_costmap.voxel_layer.pointcloud.topic` | `/livox/lidar` | `/livox/lidar`（不变）或 `/utlidar/cloud_base` | 取决于阶段C 用哪个 LiDAR 驱动。**MID360 用 livox_ros_driver2 发 `/livox/lidar`**；若复用狗自带 utlidar 则 `/utlidar/cloud_base`。Generator 根据 §7 TF 架构注释说明二选一。**推荐 `/livox/lidar`**（外置 MID360，与 TECH_DECISIONS 第二节"用 MID360 自带 IMU"一致） |
| `global_costmap` | `plugins` | `["static_layer", "obstacle_layer", "inflation_layer"]` | 加 `voxel_layer`（可选）或保持 | global 主要靠 static + obstacle(/scan)，voxel 可选。**推荐保持休眠值**（global 不需要 3D 体素，2D 投影足够规划） |
| `global_costmap.static_layer` | `map_subscribe_transient_local` | `true` | `true`（但无 map_server 时此层不报错，仅不叠加静态地图） | FAST_LIO 不发 OccupancyGrid，static_layer 订阅 `/map` 无数据，costmap 靠 obstacle_layer 实时累积。**可选**：阶段D 暂不加 map_server，static_layer 留着无害 |
| `planner_server` | `GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | 可选改 `nav2_smac_planner/SmacPlanner2D`（更优）或保持 Navfn | Navfn 简单够用，室内小地图 OK。**推荐保持 Navfn**（起步简单），未来换 Smac |
| `behavior_server` | 各 `plugin` | `nav2_behaviors/Spin` 等 | 不变（休眠值正确，已用新包名 nav2_behaviors） | Humble 已从 nav2_recoveries 迁移到 nav2_behaviors |
| `behavior_server` | `global_frame` | `odom` | `odom`（不变） | recovery 行为在 odom frame |
| 全节点 | `use_sim_time` | 部分缺失 | launch 里统一传 `use_sim_time: false`（实车不用 sim time） | 实车模式 |

**新增项**（休眠文件完全没有）：
- `bt_navigator.plugin_lib_names`：Humble 标准 BT plugin 库列表（约 20 项，从 Nav2 官方复制）
- `local_costmap.obstacle_layer`：完整 ObstacleLayer 配置（吃 /scan）
- `controller_server.FollowPath.critics`：休眠值已对（保留）
- **可选** `velocity_controller` 节点参数：决策 7 不加，跳过

**实现约束**：
- **不删**休眠文件已有的正确参数（footprint / Go2W 室内速度 / 差速起步 / VoxelLayer 雏形 / behavior_server 新包名）。
- **保留** `local_costmap.voxel_layer`（它是双保险的"辅"），只补 `obstacle_layer`（"主"）。
- **layer 顺序**：`plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]`（obstacle 在 voxel 前，inflation 最后）。

#### `src/go2w_nav/launch/nav2_3d.launch.py`（核心，休眠文件修正）
**职责**：启动 Nav2 stack + pointcloud_to_laserscan + TF 桥（临时 static_transform）。

**修正项**（休眠文件 → 正确结构）：

| 项 | 休眠值/结构 | 正确值/结构 | 理由 |
|---|---|---|---|
| Nav2 启动方式 | `nav2_bringup.navigation_launch` 单节点 | **保留** `navigation_launch`（它默认不启 amcl，符合决策 3） | navigation_launch 是 Humble 推荐的"无 amcl/无 map_server"纯 Nav2 stack 入口 |
| lifecycle_manager | 缺失（navigation_launch 内部自带？） | **显式加** lifecycle_manager 节点（navigation_launch 不自带 lifecycle_manager，要单独启） | 休眠文件只起 navigation_launch 没 lifecycle_manager，Nav2 节点不会自动 activate → bt_navigator 不接 action goal。**Critical**：必须加 lifecycle_manager_navigation 节点，`autostart: True, node_names: [controller_server, planner_server, behavior_server, bt_navigator, waypoint_follower]`（**不含 amcl / map_server**） |
| pointcloud_to_laserscan | 已有（参数正确） | 保留，复用决策 6 的参数 | 正确 |
| TF 桥 | `static_transform_publisher` × 2（map↔camera_init, body↔base_link） | **保留**（临时方案），顶部注释说明"阶段C FAST_LIO 装好后改用 FAST_LIO 直接发 map→odom，删此 static_transform" | 临时方案，阶段C 就绪后优化 |
| `/Odometry` → `/odom` remap | 缺失 | **加 remap**：FAST_LIO 发 `/Odometry`，Nav2 期望 `/odom`——在 launch 里对 navigation_launch 加 `remappings=[('/odom', '/Odometry')]`（把 Nav2 订阅的 `/odom` 重定向到 FAST_LIO 的 `/Odometry`）。**或**在 nav2_params_3d.yaml 显式设 `odom_topic: /Odometry`（决策 1 推荐此）。二选一，**推荐 yaml 显式设**（更显式） | Nav2 各节点默认订 `/odom`，统一改 yaml 比逐节点 remap 保险 |
| `use_sim_time` | `false` | `false`（实车） | 正确 |
| 启动顺序 | 无 TimerAction | 加 TimerAction 让 pointcloud_to_laserscan + TF 桥先起（period 0s），Nav2 后起（period 2s，等 TF 就绪） | Nav2 启动时 lookup TF `map→base_link` 失败会报错；TF 桥先起避免 |

**最终 launch 结构**（Generator 实现）：
```
LaunchDescription:
  - DeclareLaunchArgument: params_file (default nav2_params_3d.yaml), use_sim_time (false)
  - Node: pointcloud_to_laserscan (period 0s, 决策 6 参数)
  - Node: static_transform_publisher map→camera_init (period 0s)
  - Node: static_transform_publisher body→base_link (period 0s)
  - TimerAction(period=2.0):
    - Node: nav2_bringup navigation_launch (params_file, use_sim_time)
    - Node: nav2_lifecycle_manager lifecycle_manager_navigation (autostart, node_names=[controller_server, planner_server, behavior_server, bt_navigator, waypoint_follower])  # 不含 amcl
```

**实现约束**：
- **不启动** livox_ros_driver2 / FAST_LIO / nx_motion_node / nx_web（阶段A/C 独立 launch）。
- **不启动** amcl / map_server / slam_toolbox（决策 3 + 阶段D 红线）。
- **不加** `/cmd_vel` remap（决策 5，零反转）。
- TF 桥的 static_transform_publisher 是**临时**方案，顶部注释写明"阶段C FAST_LIO 装好后，配置 FAST_LIO 直接发 map→odom，删掉这两个 static_transform_publisher"。

### 5.2 新建文件

#### `docs/nav2_3d_runbook.md`（运维 SOP，约 60 行）
**职责**：阶段D 实跑 runbook（硬件装完 + 阶段C 就绪后用）。

**内容要点**：
1. **前置检查**：livox_ros_driver2 编译安装（TECH_DECISIONS 第二节步骤）、FAST_LIO 编译安装、NX 上 `ros2 topic list` 含 `/livox/lidar` `/Odometry`、`ros2 run tf2_ros tf2_echo map base_link` 有输出（TF 链通）。
2. **启动顺序**（5 个终端，见决策 4）。
3. **验证步骤**：
   - `ros2 action list` 含 `/navigate_to_pose`
   - `ros2 node list` 含 controller_server/planner_server/bt_navigator/lifecycle_manager_navigation，**不含 amcl**
   - `ros2 topic info /tf -v` 的 publisher 列表**不含 amcl/nav2**（map→odom 只 FAST_LIO 发）
   - `ros2 topic echo /scan --once` 有数据（pointcloud_to_laserscan 工作）
   - `ros2 run nav2_costmap_2d nav2_costmap_2d` 或 rviz2 看 local_costmap/global_costmap 有障碍
   - `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0, w: 1}}}}"` 狗真走到 (1,0)
4. **常见故障**：
   - TF `map→base_link` 断链 → 检查 FAST_LIO 是否发 `/Odometry` + TF 桥 static_transform 是否起
   - costmap 全空 → pointcloud_to_laserscan 的 `target_frame: base_link` lookup 失败（TF 不通）
   - 狗乱转 → 检查 launch 无 `/cmd_vel` remap（决策 5）
   - Nav2 节点不 activate → lifecycle_manager 的 node_names 列表对（含 bt_navigator）
   - goal 被拒 → bt_navigator.global_frame 与 goal.frame_id 不一致（都应 `map`）
5. **阶段E 联调**：kill `web/mock_nav2_action.py`，启动阶段D 真 Nav2，浏览器发"搜索客厅"，狗真走。

#### `gan-harness/eval-rubric-stage-d.md`（Critic 消费，约 150 行）
**职责**：阶段D 静态审 rubric（见本 spec §11 + 独立文件）。

### 5.3 不动文件清单（Generator 勿碰，Critic 会核对）

- `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py`（阶段E 红线，D 服务端兼容客户端契约即可）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py` / `nx_sensor_node.py`（阶段A 红线）
- `src/go2w_nav/config/nav2_params.yaml`（2D 路线休眠文件，不动——它是历史留档，阶段D 用 _3d 版）
- `src/go2w_nav/config/slam_toolbox.yaml` / `launch/slam.launch.py`（slam_toolbox 路线，阶段D 不用 FAST_LIO，不动）
- `src/go2w_nav/launch/nav2.launch.py`（2D Nav2 launch，不动）
- `src/go2w_bringup/launch/search.launch.py`（旧 2D 全系统 launch，不动——它是阶段0/1 的历史入口，阶段D 用独立 nav2_3d.launch.py）
- `src/go2w_nav/package.xml` / `CMakeLists.txt`（已 install config/launch，新增/修改文件自动覆盖，无需改）

### 5.4 文件改动量预估

| 文件 | 类型 | 预估改动 | 改动性质 |
|---|---|---|---|
| `src/go2w_nav/config/nav2_params_3d.yaml` | 修改 | ~60 行（修正 + 新增 obstacle_layer + plugin_lib_names） | 参数修正 |
| `src/go2w_nav/launch/nav2_3d.launch.py` | 修改 | ~30 行（加 lifecycle_manager + TimerAction + remap 注释） | launch 结构修正 |
| `docs/nav2_3d_runbook.md` | 新建 | ~60 行 | 运维 SOP |
| `gan-harness/eval-rubric-stage-d.md` | 新建 | ~150 行 | Critic 消费 |

---

## 6. costmap 配置详解（global/local 的 layers）

### 6.1 global_costmap（全局规划用）

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0          # 1Hz 更新（全局，慢）
      publish_frequency: 1.0
      global_frame: map              # 全局 frame（与 goal frame 一致）
      robot_base_frame: base_link
      footprint: "[ [0.30, 0.20], [0.30, -0.20], [-0.25, -0.20], [-0.25, 0.20] ]"  # Go2W 室内 footprint
      resolution: 0.05               # 5cm 栅格
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]  # 顺序: static→obstacle→inflation
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true   # 订阅 /map (FAST_LIO 不发, 此层空, 无害)
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan                # 来自 pointcloud_to_laserscan
          data_type: "LaserScan"
          marking: True
          clearing: True
          max_obstacle_height: 1.5    # 与 SDK_CAPABILITIES 对齐
          min_obstacle_height: -0.1
          obstacle_max_range: 7.0     # MID360 有效距离内
          obstacle_min_range: 0.2
          raytrace_max_range: 8.0
          raytrace_min_range: 0.15
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7         # Go2W footprint 外膨胀 0.7m (室内走廊够走)
      always_send_full_costmap: true
```

**关键**：
- global 不加 voxel_layer（2D 投影足够规划，3D 体素是 local 的双保险）。
- `static_layer` 留着无害（FAST_LIO 不发 `/map`，此层订阅无数据，costmap 靠 obstacle_layer 实时累积）。**未来**加 map_server 发静态 occupancy map 时此层自动生效。
- `inflation_radius: 0.7`：Go2W footprint 最大 0.30m（前后），膨胀 0.7m 让狗离墙 0.4m 以上（室内门框/家具间隙够走）。

### 6.2 local_costmap（实时避障用，双保险）

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0           # 5Hz 更新（实时）
      publish_frequency: 2.0
      global_frame: odom              # local frame（rolling window 在 odom 系）
      robot_base_frame: base_link
      rolling_window: true            # 滚动窗口（狗动跟着动）
      width: 5                        # 5×5m 局部地图
      height: 5
      resolution: 0.05
      footprint: "[ [0.30, 0.20], [0.30, -0.20], [-0.25, -0.20], [-0.25, 0.20] ]"
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]  # 双保险: obstacle(主)+voxel(辅)+inflation
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: True
          clearing: True
          max_obstacle_height: 1.5
          min_obstacle_height: -0.1
          raytrace_max_range: 8.0
          raytrace_min_range: 0.15
          obstacle_max_range: 7.0
          obstacle_min_range: 0.2
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true       # rviz 可视化 3D 体素
        origin_z: 0.0
        z_resolution: 0.10            # 10cm 垂直分辨率
        z_voxels: 16                  # 16×0.10=1.6m 高度
        max_obstacle_height: 1.5      # 与 SDK_CAPABILITIES 对齐
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /livox/lidar         # MID360 原始点云 (或 /utlidar/cloud_base)
          max_obstacle_height: 1.5
          min_obstacle_height: 0.05   # 稍高于地面噪声
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 8.0
          raytrace_min_range: 0.15
          obstacle_max_range: 7.0
          obstacle_min_range: 0.2
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.7
      always_send_full_costmap: true
```

**关键**：
- **layer 顺序**：`obstacle_layer` 在 `voxel_layer` 前（叠加顺序，obstacle 先标 2D 障碍，voxel 再补 3D 细节）。
- `voxel_layer.pointcloud.topic` 二选一：`/livox/lidar`（外置 MID360，推荐）或 `/utlidar/cloud_base`（狗自带）。Generator 在 yaml 注释说明。
- `z_voxels: 16` × `z_resolution: 0.10` = 1.6m 高度，覆盖 Go2W 站立高度 + 桌面障碍。

---

## 7. TF 架构图（map→odom→base_link，谁发布）

### 7.1 TF 树（阶段D 目标态）

```
map                         (全局 frame, Nav2 global_costmap + goal pose 用)
 │
 │  ← 发布者: FAST_LIO (阶段C) 发 camera_init→body
 │    + TF 桥 (static_transform_publisher, 本阶段临时):
 │        map == camera_init (identity transform)
 │    或: 阶段C 优化后 FAST_LIO 直接配 map→odom
 │
 ▼
odom                        (里程计 frame, Nav2 local_costmap + controller_server 用)
 │
 │  ← 发布者: FAST_LIO 发 /Odometry (camera_init→body)
 │    + TF 桥 (static_transform_publisher, 本阶段临时):
 │        body == base_link (identity transform, 雷达装狗中心时成立)
 │    或: 阶段C 优化后用 robot_localization EKF 拆 odom→base_link
 │
 ▼
base_link                   (机器人本体 frame, costmap robot_base_frame + pointcloud_to_laserscan target_frame)
 │
 ├── livox_frame (或 utlidar_lidar)   ← LiDAR 物理安装偏移 (static_transform, 阶段C 提供)
 └── (其他传感器 frame)
```

### 7.2 谁发布哪个 TF（Critical：不能多源）

| TF 段 | 发布者 | topic/方式 | 频率 | 备注 |
|---|---|---|---|---|
| `map → odom` | FAST_LIO（经 TF 桥改名） | `/tf` (camera_init→body 改名为 map→odom) 或 static_transform | FAST_LIO ~100Hz / static 一次性 | **Critical**：只 FAST_LIO 发，Nav2/amcl 不发。阶段D 临时用 static_transform_publisher 发 `map→camera_init`（identity）让 `map==camera_init`，FAST_LIO 发 `camera_init→body`，组合起来 `map→body`。但 Nav2 要 `map→odom→base_link`，所以还要 `body→base_link`（identity static） |
| `odom → base_link` | FAST_LIO（经 TF 桥改名） | 同上 | 同上 | 同上，临时用 static_transform_publisher 发 `body→base_link`（identity） |
| `base_link → livox_frame` | static_transform_publisher | `/tf` | 一次性 | LiDAR 物理安装偏移（阶段C 提供，阶段D launch 可不含，假设阶段C 已发） |

### 7.3 临时 TF 桥方案（本阶段，阶段C 未就绪时）

休眠 nav2_3d.launch.py 的临时方案：
```
static_transform_publisher: map → camera_init  (0,0,0, 0,0,0)  # identity, 让 map==camera_init
static_transform_publisher: body → base_link   (0,0,0, 0,0,0)  # identity, 让 body==base_link
```
效果：`map → camera_init → body → base_link`，其中 `map→camera_init` 和 `body→base_link` 是 identity（零偏移），`camera_init→body` 是 FAST_LIO 发的真实里程计。

**前提假设**（顶部注释写明）：
1. 雷达装在狗中心（`body==base_link`，零偏移）——若雷达有物理偏移（如装在头部），static_transform 要填真实偏移。
2. `map==camera_init`——FAST_LIO 的全局原点 = map 原点（建图起始点）。

**阶段C 就绪后的优化方案**（本阶段不做，文档说明）：
- 配置 FAST_LIO 直接发 `map→odom`（改 FAST_LIO 的 frame 名参数，`camera_init`→`map`、`body`→`odom`，再单独发 `odom→base_link`）；或
- 用 `robot_localization` EKF 拆分：FAST_LIO 发 `odom→base_link`，EKF 融合发 `map→odom`。

### 7.4 Critic 静态审 TF 检查项

1. `map→odom` 发布者**唯一**（只 FAST_LIO 或其 TF 桥，Nav2/amcl 不发）。
2. `odom→base_link` 发布者**唯一**。
3. pointcloud_to_laserscan 的 `target_frame: base_link` 能 lookup（TF 链 `map→odom→base_link` 完整）。
4. Nav2 `bt_navigator.global_frame: map` + `robot_base_frame: base_link`，能 lookup `map→base_link`。
5. **无 TF 冲突**：amcl 不启动（决策 3），slam_toolbox 不启动（阶段D 红线）。

---

## 8. /cmd_vel 坐标核实结论（Critical，详见决策 5）

### 8.1 核实结论

**Nav2 controller_server 输出的 `/cmd_vel` 与 nx_motion_node 期望的约定完全一致，零反转，无需 remap。**

| 层 | 接口 | 约定 | 反转次数 |
|---|---|---|---|
| Nav2 DWBLocalPlanner | 发布 `geometry_msgs/Twist` 到 `/cmd_vel` | `linear.x` 正=前进, `angular.z` 正=左转（REP-103） | 0 |
| `/cmd_vel` topic | DDS 传输 | 透传 | 0 |
| nx_motion_node `_on_cmd_vel` | `self._vx=msg.linear.x; self._vyaw=msg.angular.z` | 直接赋值，无反转（nx_motion_node.py:118-120） | 0 |
| nx_motion_node `_ctrl_loop` | `self._sport.Move(vx, vy, vyaw)` | SDK Move(x,y,z)，z 正=左转（SDK_CAPABILITIES §2.1 实测） | 0 |
| **合计** | | | **0 次反转** |

### 8.2 双重反转风险排查

| 风险点 | 是否反转 | 证据 |
|---|---|---|
| Nav2 DWB `angular.z` 正负 | 否（正=左转） | Nav2 Humble 源码默认 REP-103 |
| nx_motion_node `_on_cmd_vel` | 否 | nx_motion_node.py:120 `self._vyaw = msg.angular.z` |
| nx_motion_node `Move(vx,vy,vyaw)` | 否 | SDK_CAPABILITIES.md §2.1 实测 + nx_motion_node.py:117 注释 |
| launch `/cmd_vel` remap | **禁止反转 remap** | 决策 5 红线 |

### 8.3 与阶段A 手柄控狗的共存（/cmd_vel 仲裁）

**问题**：nx_web_server.py:219 的 NxWebNode 发 `/cmd_vel`（手柄控狗，前端 vx/vy/vyaw），Nav2 的 controller_server 也发 `/cmd_vel`（自主导航）。两个 publisher 共存于同一 topic。

**ROS2 DDS 行为**：多 publisher 共存不报错，subscriber（nx_motion_node）收到的是**最后一个 publisher 的消息**（按时间戳）——即"谁后发谁覆盖"。这导致：
- 手柄控狗时，用户按前进 → nx_web 发 `/cmd_vel` → 但 Nav2 的 controller_server 也在 10Hz 发 `/cmd_vel`（导航中）→ nx_motion_node 收到的是 Nav2 的（用户手柄被覆盖）。
- 自主导航时，Nav2 发 `/cmd_vel` → 正常。

**推荐仲裁策略**（阶段D 文档说明，不强制实现）：
- **方案 A（默认，简单）**：Nav2 导航中（RoomSearchOrchestrator 的 NAVIGATING/SEARCH 阶段），前端手柄控狗 UI 禁用（前端禁用按钮，阶段F+ 实现）。用户要干预先 `/api/e_stop` 取消导航，再手柄控。
- **方案 B（未来优化）**：Nav2 和 nx_web 发不同 topic（`/cmd_vel_nav2` / `/cmd_vel_manual`），nx_motion_node 加优先级仲裁（手柄优先 / 或 Nav2 优先）。**阶段D 不做**，复杂度不值。

**阶段D 结论**：launch 里 controller_server 的 `/cmd_vel` **不 remap**（直接发 `/cmd_vel`），与 nx_web 共存。仲裁靠"导航中禁用手柄"（前端约定，阶段E 编排 NAVIGATING 时 ws 推 `type=search_room` 的 phase，前端可据此禁用——但阶段E spec 决策是不改前端，所以**当前阶段用户自觉**：导航中别按手柄）。文档 `docs/nav2_3d_runbook.md` 写明此约定。

### 8.4 Critic 静态审 /cmd_vel 检查项

1. launch 文件 controller_server 节点**无 `/cmd_vel` remap**（或 remap 目标仍是 `/cmd_vel`，即 no-op remap）。
2. nav2_params_3d.yaml 的 controller_server `cmd_vel_topic`（若有）= `/cmd_vel`（默认）。
3. **禁止**出现 `remappings=[('/cmd_vel', '/cmd_vel_reversed')]` 或类似反转。
4. **禁止**在 yaml 里设 `cmd_vel_topic: /cmd_vel_nav2`（除非决策 B 实施，但阶段D 不做）。

---

## 9. Nav2 lifecycle 配置（autostart 节点列表，amcl 不启动）

### 9.1 lifecycle_manager 配置

```yaml
# 在 launch 文件里 (nav2_3d.launch.py):
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'use_sim_time': False,
        'autostart': True,
        'node_names': [
            'controller_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
        ],
        # ⚠️ 不含 amcl (决策 3, FAST_LIO 替代)
        # ⚠️ 不含 map_server (FAST_LIO 不发 OccupancyGrid, 用 obstacle_layer 实时累积)
        # ⚠️ 不含 slam_toolbox (与 FAST_LIO 互斥)
    }],
)
```

### 9.2 节点启动列表（含/不含）

| 节点 | 启动 | 理由 |
|---|---|---|
| `controller_server` | ✅ | DWB 局部规划，发 /cmd_vel |
| `planner_server` | ✅ | Navfn 全局规划 |
| `behavior_server` | ✅ | recovery 行为（spin/backup/wait） |
| `bt_navigator` | ✅ | 行为树编排，暴露 `/navigate_to_pose` action |
| `waypoint_follower` | ✅ | 航点跟随（阶段E 编排虽自己逐航点发 goal，但 Nav2 stack 完整起） |
| `lifecycle_manager_navigation` | ✅ | 自动 activate 上述节点 |
| **`amcl`** | ❌ **不启动** | 决策 3，FAST_LIO 替代定位，amcl 会抢 map→odom TF |
| **`map_server`** | ❌ **不启动** | FAST_LIO 不发 OccupancyGrid；global_costmap 靠 obstacle_layer 实时累积（static_layer 订阅 /map 无数据，无害） |
| **`slam_toolbox`** | ❌ **不启动** | 阶段D 红线，与 FAST_LIO 互斥 |

### 9.3 Critic 静态审 lifecycle 检查项

1. `lifecycle_manager_navigation.node_names` 含 `bt_navigator`（否则 `/navigate_to_pose` action 不暴露，阶段E 客户端 `wait_for_server` 超时）。
2. `node_names` **不含 amcl**（Critical，决策 3）。
3. `autostart: True`（否则节点停在 unconfigured 状态，不接 goal）。
4. launch 不启动 `map_server` / `slam_toolbox` 节点（grep launch 文件无 `package='nav2_map_server'` / `'slam_toolbox'`）。

---

## 10. Anti-slop / 反模式清单（Generator 自查）

- ❌ 不要启动 amcl（决策 3，会抢 map→odom TF）
- ❌ 不要启动 slam_toolbox（阶段D 红线，与 FAST_LIO 互斥）
- ❌ 不要启动 map_server（FAST_LIO 不发 OccupancyGrid，global_costmap 靠 obstacle_layer 实时累积）
- ❌ 不要在 launch 加 `/cmd_vel` 反转 remap（决策 5，零反转）
- ❌ 不要把 `/Odometry` remap 成 `/odom` 时漏节点（决策 1，推荐 yaml 显式设 `odom_topic: /Odometry` 而非 launch remap）
- ❌ 不要用 `nav2_recoveries/recoveries_server`（已废弃，Humble 用 `nav2_behaviors`）
- ❌ 不要用 `nav2_bringup/bringup_launch`（它启 amcl + map_server），用 `navigation_launch`（纯 Nav2 stack）
- ❌ 不要在 local_costmap 删 obstacle_layer（决策 2，双保险要求 obstacle 主 + voxel 辅）
- ❌ 不要在 global_costmap 加 voxel_layer（2D 投影足够规划，3D 体素是 local 的事）
- ❌ 不要改阶段E 的 nx_room_orchestrator.py / mock_nav2_action.py（红线，D 服务端兼容客户端契约）
- ❌ 不要改阶段A 的 nx_motion_node.py / nx_sensor_node.py（红线）
- ❌ 不要在 nav2_3d.launch.py 启动 livox_ros_driver2 / FAST_LIO / nx_motion_node / nx_web（阶段A/C 独立 launch，阶段D 只 Nav2 + p2l + TF 桥）
- ❌ 不要省略 `bt_navigator.plugin_lib_names`（Humble 必填，否则 bt_navigator 启动报错）
- ❌ 不要省略 lifecycle_manager（navigation_launch 不自带，必须单独启，否则节点不 activate）
- ❌ 不要把 TF 桥的 static_transform 当永久方案（顶部注释写明"阶段C 就绪后删，FAST_LIO 直接发 map→odom"）
- ❌ 不要在 footprint 用 `robot_radius`（Go2W 是非圆形 footprint，用 `footprint` 多边形）
- ❌ 不要把 `inflation_radius` 设太大（室内 0.7m 够，太大走廊走不动）
- ❌ 不要把 `max_vel_x` 设超过 0.6（TECH_DECISIONS 第三节 Go2W 室内参数，安全速度）
- ❌ 不要把 `max_vel_y` 设非零（差速起步，决策 TECH_DECISIONS 第三节）
- ❌ 不要忘了 `use_sim_time: false`（实车不用 sim time）

---

## 11. 静态审清单（对照 Nav2 Humble 规范 + TECH_DECISIONS，Critic 用）

### 11.1 参数正确性（对照 Nav2 Humble 官方 nav2_params.yaml）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 1 | `bt_navigator.global_frame` | = `map`（与阶段E goal frame 一致） | Critical（frame 不匹配 goal 被拒） |
| 2 | `bt_navigator.robot_base_frame` | = `base_link` | Critical |
| 3 | `bt_navigator.odom_topic` | = `/Odometry`（FAST_LIO 原生）或 launch remap 一致 | High |
| 4 | `bt_navigator.plugin_lib_names` | 存在且含 NavigateToPose 等标准 BT plugin（Humble 必填） | Critical（bt_navigator 启动失败） |
| 5 | `controller_server.FollowPath.max_vel_x` | = 0.6（TECH_DECISIONS 第三节） | High |
| 6 | `controller_server.FollowPath.max_vel_y` | = 0.0（差速起步） | High |
| 7 | `controller_server.FollowPath.max_vel_theta` | = 1.0 | High |
| 8 | `controller_server` 的 `xy_goal_tolerance` / `yaw_goal_tolerance` | = 0.20 / 0.15（TECH_DECISIONS 第三节） | High |
| 9 | `local_costmap.footprint` / `global_costmap.footprint` | = `[ [0.30,0.20], [0.30,-0.20], [-0.25,-0.20], [-0.25,0.20] ]` | High |
| 10 | `local_costmap.plugins` | 含 obstacle_layer + voxel_layer + inflation_layer（双保险） | Critical（缺层=撞墙） |
| 11 | `local_costmap.obstacle_layer.scan.topic` | = `/scan` | High |
| 12 | `local_costmap.voxel_layer.pointcloud.topic` | = `/livox/lidar` 或 `/utlidar/cloud_base`（注释说明） | High |
| 13 | `local_costmap.global_frame` | = `odom`（rolling window） | High |
| 14 | `global_costmap.plugins` | 含 static_layer + obstacle_layer + inflation_layer | High |
| 15 | `global_costmap.global_frame` | = `map` | Critical |
| 16 | `behavior_server` 各 plugin | 用 `nav2_behaviors/Spin` 等（新包名，非 nav2_recoveries） | High |
| 17 | `planner_server.GridBased.plugin` | = `nav2_navfn_planner/NavfnPlanner`（或 Smac，注释说明） | Medium |

### 11.2 TF 一致性

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 18 | `map→odom` 发布者唯一 | launch 不启 amcl，只 FAST_LIO + TF 桥发 | Critical（多源 TF 跳变） |
| 19 | `odom→base_link` 发布者唯一 | 同上 | Critical |
| 20 | pointcloud_to_laserscan `target_frame` | = `base_link`（能 lookup） | High |
| 21 | TF 桥 static_transform 存在 | map→camera_init + body→base_link（临时方案，注释说明） | High（无 TF 桥 TF 断链） |
| 22 | slam_toolbox 不启动 | launch 无 slam_toolbox 节点 | Critical |

### 11.3 坐标系无反转（Critical）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 23 | launch 无 `/cmd_vel` 反转 remap | controller_server 节点 remappings 不含 `('/cmd_vel', ...)` 或目标是 `/cmd_vel` | **Critical（狗乱转）** |
| 24 | yaml 无 `cmd_vel_topic` 改名 | controller_server `cmd_vel_topic` 默认 `/cmd_vel`（若显式设，必须 `/cmd_vel`） | Critical |
| 25 | `/Odometry` → `/odom` 处理一致 | yaml 显式 `odom_topic: /Odometry` 或 launch 全局 remap，二选一不混用 | High |

### 11.4 costmap layers 完整

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 26 | local_costmap 有 obstacle_layer | ObstacleLayer 吃 /scan（主） | Critical（缺层=撞墙） |
| 27 | local_costmap 有 voxel_layer | VoxelLayer 吃 PointCloud2（辅，双保险） | High |
| 28 | local_costmap 有 inflation_layer | InflationLayer 最后（plugins 数组最后） | High |
| 29 | global_costmap 有 obstacle_layer | ObstacleLayer 吃 /scan | High |
| 30 | layer 顺序正确 | static→obstacle→voxel→inflation（叠加顺序） | Medium |

### 11.5 Nav2 lifecycle 正确

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 31 | lifecycle_manager 存在 | launch 含 lifecycle_manager_navigation 节点 | Critical（节点不 activate） |
| 32 | lifecycle_manager.autostart | = True | Critical |
| 33 | lifecycle_manager.node_names 含 bt_navigator | 否则 /navigate_to_pose 不暴露 | Critical（阶段E 客户端超时） |
| 34 | lifecycle_manager.node_names 不含 amcl | 决策 3 | Critical |
| 35 | launch 不启 map_server | grep 无 nav2_map_server | High |

### 11.6 阶段A/B/E 契约不破坏

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 36 | 阶段E action 名对齐 | Nav2 默认 `/navigate_to_pose`（不 remap） | Critical（阶段E 客户端找不到 server） |
| 37 | 阶段E goal frame 对齐 | bt_navigator.global_frame = map（与 rooms.yaml 默认 frame_id 一致） | Critical |
| 38 | 不改 nx_motion_node / nx_room_orchestrator / mock_nav2_action | git diff 这三个文件为空 | Critical |
| 39 | 不改 nx_web_server / nx_sensor_node | git diff 为空（阶段A/B 红线） | Critical |

---

## 12. 分 Sprint 实现顺序（配置先写，阶段C 就绪后联调）

### Sprint 1：nav2_params_3d.yaml 修正（纯配置，不依赖硬件/阶段C）
**目标**：休眠文件修正后，静态对照 Nav2 Humble 官方规范 + TECH_DECISIONS 第三节，参数名/类型/值全部正确。
**Features**：
- 修正 `bt_navigator`（加 plugin_lib_names、确认 odom_topic/global_frame）
- 修正 `local_costmap`（补 obstacle_layer，调 layer 顺序）
- 确认 `global_costmap`（保持 static+obstacle+inflation）
- 确认 `controller_server` Go2W 室内参数（速度/加速度/footprint/tolerance）
- 确认 `behavior_server` 用 nav2_behaviors 新包名
**Definition of Done**：
- yaml 静态审 §11.1 全过（17 项）
- `ros2 param dump` 能加载（语法正确，无 YAML 错）
- **不依赖硬件**：纯文件审
- **不依赖阶段C**：参数文件独立

### Sprint 2：nav2_3d.launch.py 修正（纯配置，不依赖硬件/阶段C）
**目标**：launch 文件结构正确，能被 `ros2 launch` 解析（即使节点起不来因依赖未就绪）。
**Features**：
- 加 lifecycle_manager_navigation 节点（autostart, node_names 不含 amcl）
- 加 TimerAction（TF 桥 + p2l 先起，Nav2 后起）
- 确认 TF 桥 static_transform（map→camera_init, body→base_link）
- 确认 pointcloud_to_laserscan 参数（决策 6）
- 确认无 `/cmd_vel` remap（决策 5）
- 顶部注释写明阶段C 依赖 + TF 桥临时性
**Definition of Done**：
- launch 静态审 §11.2-11.5 全过（18 项）
- `ros2 launch go2w_nav nav2_3d.launch.py --show-args` 能解析（语法正确）
- `ros2 launch` dry-run 不报 launch 语法错（节点起不来是阶段C 未就绪，不算 FAIL）
- **不依赖硬件**：纯 launch 结构审

### Sprint 3：docs/nav2_3d_runbook.md + eval-rubric-stage-d.md（文档 + rubric）
**目标**：运维 SOP + Critic rubric 齐全。
**Features**：
- runbook：启动顺序 + 验证步骤 + 常见故障（§5.2）
- rubric：§11 静态审清单 + 权重（独立文件 eval-rubric-stage-d.md）
**Definition of Done**：
- runbook 覆盖 5 终端启动 + 6 验证步骤 + 5 常见故障
- rubric 覆盖 §11 全部 39 检查项 + 权重
- **不依赖硬件**：纯文档

### Sprint 4（阶段C 就绪后联调）：FAST_LIO + TF 桥 + Nav2 实跑
**目标**：阶段C FAST_LIO 装好，NX 上启动全套，狗真走到目标点。
**Features**：
- 阶段C FAST_LIO 配置发 `map→odom`（或保留 TF 桥 static_transform）
- 启动顺序按 runbook（5 终端）
- `ros2 action send_goal /navigate_to_pose` 测试单点导航
- 浏览器"搜索客厅"端到端（阶段E 编排 + 阶段D 真 Nav2）
**Definition of Done**：
- `ros2 action list` 含 `/navigate_to_pose`
- `ros2 node list` 含 controller_server 等，不含 amcl
- `ros2 topic info /tf -v` map→odom 只 FAST_LIO 发
- 单点导航：狗真走到 (1,0)，方向正确（无乱转）
- 房间搜索：狗走到客厅入口 + 房间内覆盖搜索 + YOLO 检测
- **依赖硬件**：是
- **依赖阶段C**：是

---

## 13. 不依赖硬件/阶段C 的验证方法（Generator 必须实现并跑通）

### 13.1 静态验证（Sprint 1-3，纯配置审）

**yaml 语法验证**：
```bash
# 1. YAML 语法
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/nav2_params_3d.yaml'))"

# 2. 关键参数 grep
grep -E "global_frame|robot_base_frame|odom_topic|max_vel_x|max_vel_y|footprint|plugins:" src/go2w_nav/config/nav2_params_3d.yaml
```

**launch 语法验证**：
```bash
# 3. launch 能解析
ros2 launch go2w_nav nav2_3d.launch.py --show-args

# 4. 无 amcl / map_server / slam_toolbox
grep -E "amcl|map_server|slam_toolbox" src/go2w_nav/launch/nav2_3d.launch.py  # 应无输出

# 5. 无 /cmd_vel remap
grep -E "cmd_vel" src/go2w_nav/launch/nav2_3d.launch.py  # 应无 remap 行
```

**静态审清单**：Critic 按 §11 的 39 项逐条核对（人工 + grep），输出 PASS/FAIL 表。

### 13.2 半实跑验证（Sprint 4，阶段C 就绪后）

详见 `docs/nav2_3d_runbook.md`（§5.2），核心：
- `ros2 action send_goal /navigate_to_pose ...` 单点导航测试
- 浏览器"搜索客厅"端到端
- 故障注入：kill FAST_LIO 看 Nav2 报 TF 断链；kill pointcloud_to_laserscan 看 costmap 全空

---

## 14. Evaluation Criteria（见 `gan-harness/eval-rubric-stage-d.md`，权重已定）

详见独立 `gan-harness/eval-rubric-stage-d.md`，Critic 直接消费。核心四维：

- **参数正确性（0.30）**：对照 Nav2 Humble 官方规范 + TECH_DECISIONS 第三节，参数名/类型/值正确（§11.1 的 17 项）
- **TF 与坐标系一致性（0.30）**：TF 树完整无多源、map→odom 只 FAST_LIO 发、/cmd_vel 零反转（§11.2-11.4 的 13 项，含 Critical 反转检查）
- **costmap 完整性（0.20）**：local 双保险（obstacle+voxel+inflation）、global 三层、layer 顺序正确（§11.4 的 5 项）
- **阶段A/B/E 契约不破坏（0.20）**：不改 nx_motion/orchestrator/mock_nav2、action 名/frame 对齐（§11.6 的 4 项）
