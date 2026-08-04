# Product Specification: Go2W 阶段F — slam_toolbox 2D 建图（降级路径，替代未就绪的 FAST_LIO）

> Generated from brief: "为 Go2W 建图模块（slam_toolbox 2D）产出完整实现规格（GAN Planner 角色）"
> 主管角色: GAN Planner（架构/规格，不含 yaml/launch 代码实现——由 Generator 写）
> 状态: 待 Generator 实现，待 Critic 静态审
> 范围: **纯软件先写配置**（NX 恢复 + 推狗走后实跑建图）。本阶段产出 slam_toolbox 参数文件 + launch 文件 + 阶段D Nav2 适配方案，**静态审对照 slam_toolbox Humble 官方规范 + `nx_sensor_node.py` 实测 frame/QoS + `TECH_DECISIONS.md`**，不实车。
> 前置: 阶段A（nx_sensor 发 `/scan /imu /odom` + odom→base_link TF，已部署 `go2w-sensor.service`，实测 `/scan 10Hz /imu 50Hz /odom`）+ 阶段D（Nav2 配置 commit 9b0c397，但假设 FAST_LIO 就绪——本阶段用 slam_toolbox 替代）+ 阶段E（房间搜索编排，吃 Nav2 `/navigate_to_pose`，已 gan 收敛）
> 后置依赖: NX 上线（推狗走建图）+ 阶段0 移动控制（推狗走）才实跑。本阶段交付配置 + 静态审。

---

## 0. 规格阅读约定

- 所有路径均为**相对仓库根** `go2w_search_ws/`。
- 每个文件给三段：**职责 / 关键内容要点 / 实现约束**。要点是契约，约束是红线。
- **本阶段不写 Python 代码**（yaml/launch 由 Generator 写，本 spec 只定结构和参数表）。
- **阶段A/B/E 红线继续生效**（与阶段D spec §0 完全一致）：
  - 禁止改 `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py`（阶段E 红线）
  - 禁止改 `src/go2w_bridge/go2w_bridge/nx_motion_node.py` / `nx_sensor_node.py`（阶段A 红线，`/scan /imu /odom` 来源 = 红线中的红线）
- **阶段F（本阶段）新红线**：
  - **不改 nx_sensor_node.py**（`/scan` 是 slam_toolbox 的命脉，改 frame/QoS 会破坏阶段A 实测链路——见决策 3/4 QoS 章节的"不对称修复"原则：QoS 不匹配**只改 slam_toolbox 订阅端**，不动 nx_sensor 发布端）
  - **不复用 `src/go2w_bringup/launch/search.launch.py`**（旧 2D 全系统，slam_toolbox + amcl + Nav2 混起，互斥风险——见决策 5）
  - **slam_toolbox 与阶段D FAST_LIO 路线互斥**：建图/定位时只启其一，二者都发 `map→odom` 会 TF 冲突（与阶段D spec 决策 3 + 红线一致）
  - **slam_toolbox 与 amcl 互斥**：定位阶段用 slam_toolbox 的 **localization 模式**替代 amcl（不发粒子滤波，载入 `.posegraph` + `.data` 持续 scan-matching），**不启 amcl**
- **降级路径定位**：本阶段是 FAST_LIO/MID360 未就绪时的**降级方案**，用狗自带 2D 雷达（`nx_sensor` 投影的 `/scan`）替代 3D LIO。等 MID360 装好 + FAST_LIO 就绪后，可切回阶段D 原路线。两条路线的切换通过**选不同 launch**实现，不混用。

---

## 1. Vision（阶段F 目标态）

载荷 NX 上跑一套 **slam_toolbox 2D SLAM stack**，吃 `nx_sensor` 的 `/scan`（狗自带 LiDAR 投影的 2D LaserScan，10Hz，frame=`base_link`），用 **online async 模式**实时建图，发 `/map`（`nav_msgs/OccupancyGrid`）+ `map→odom` TF。`nx_sensor` 继续 `/odom` + odom→base_link TF（不变）。阶段D 的 Nav2 吃 slam_toolbox 的 `/map`（global_costmap static_layer 终于有数据）+ 完整 TF 链 `map→odom→base_link`，自主导航避障。

**两阶段工作流**：
1. **建图阶段（mapping）**：`slam_toolbox` online async 模式，人推狗走遍房间，`/map` 实时生长，**完成后用 `/slam_toolbox/save_map` service 存 `.posegraph` + `.data`**（slam_toolbox 原生序列化，非 ROS `nav2_map_server` 的 pgm/yaml）。
2. **定位阶段（localization）**：`slam_toolbox` localization 模式（`lifelong_slam_toolbox_node` 或 `localization_slam_toolbox_node`），载入建好的 `.posegraph` + `.data`，持续 scan-matching 发 `map→odom`（**替代 amcl**），Nav2 此时跑纯导航。

一句话验收：**NX 上启动 nx_sensor（go2w-sensor.service）+ 阶段F 的 slam.launch.py（mode=mapping）+ 阶段D 的 nav2_slim.launch.py（适配版），`ros2 topic echo /map --once` 有 OccupancyGrid 数据，`ros2 run tf2_ros tf2_echo map odom` 有输出（slam_toolbox 发），`ros2 run tf2_ros tf2_echo odom base_link` 有输出（nx_sensor 发），推狗走一圈 `/map` 覆盖房间；切 localization 模式载入地图后，浏览器发"搜索客厅"狗真走。**（实跑依赖 NX 恢复 + 阶段0，本阶段只交付配置 + 静态审）

---

## 2. 关键约束核实：nx_sensor_node.py 实测契约（Generator 必读，全部已 grep 确认）

本阶段最危险的陷阱是**对 nx_sensor 的 `/scan` 做错误假设**。以下全部基于 `src/go2w_bridge/go2w_bridge/nx_sensor_node.py` 源码逐行核实：

| 契约项 | 实测值（源码行号） | 对 slam_toolbox 的影响 |
|---|---|---|
| `/scan` 发布 QoS | `self.create_publisher(LaserScan, '/scan', 10)`（**第 107 行**）—— 裸 `depth=10`，即 rclpy **默认 QoS = RELIABLE + VOLATILE**（第 104 行注释明确"QoS 用默认 RELIABLE"） | ⚠️ **致命**：slam_toolbox 的 `scan_topic` 订阅默认用 **sensor_data QoS（BEST_EFFORT + VOLATILE）**。RELIABLE 发布者 vs BEST_EFFORT 订阅者 → **QoS 不兼容，slam_toolbox 收不到任何 scan**，建图静默失败（无报错，`/map` 一直空）。见决策 4 修复。 |
| `/scan` frame_id | `scan.header.frame_id = 'base_link'`（**第 228 行**）—— **不是独立的 `laser_frame`/`laser_link`** | slam_toolbox 的 `base_frame` 设 `base_link`，且 scan 已在 `base_link` 系 → **不需要** `base_link→laser` static_transform（激光与本体重合，nx_sensor 投影时已假设零偏移）。 |
| `/scan` 角度/距离 | `angle_min=-π, angle_max=π, angle_increment=2π/360`（第 229-231 行，360 个射线），`range_min=0.15, range_max=10.0`（第 234-235 行） | slam_toolbox `max_laser_range` 设 ≤ 10.0（与 nx_sensor 一致，见决策 6）。360 射线够密，scan_matching 充分。 |
| `/scan` 内容来源 | `_on_lidar` 把狗 DDS `rt/utlidar/cloud`（PointCloud2，SDK_CAPABILITIES §1.3 ~3800 点/帧，15Hz）XY 投影成 360 距离数组（第 133-156 行），`_publish_scan` 10Hz 定时发（第 116 行） | scan 是**狗自带 LiDAR 投影**（非外置 MID360），距离 ≤ 10m，无高度信息（已是 2D 投影）。slam_toolbox 是 2D，正合适。 |
| `/odom` 发布 | `self._odom_pub = self.create_publisher(Odometry, '/odom', 10)`（第 106 行），`header.frame_id='odom'`, `child_frame_id='base_link'`（第 201-202 行） | slam_toolbox `odom_frame: odom`，吃 `/odom`（默认 topic 名一致，**无需 remap**）。**注意**：阶段D 的 `nav2_params_3d.yaml` `odom_topic: /Odometry`（大写，FAST_LIO 原生）—— 阶段F 路线**没有 `/Odometry`**，只有 nx_sensor 的 `/odom`，这是阶段D params 不能直接复用的根因（见决策 7）。 |
| `/imu` 发布 | `/imu` 50Hz，`frame_id='imu_link'`（第 190 行） | slam_toolbox **不用 /imu**（2D SLAM 只靠 scan + odom）。nx_sensor 继续发，无害。 |
| odom→base_link TF | nx_sensor **自己用 TransformBroadcaster 发**（第 211-218 行，`/tf` topic） | ⚠️ **关键**：slam_toolbox **不能也发 odom→base_link**（否则多源 TF 跳变）。slam_toolbox 默认只发 `map→odom`（它假设 odom→base_link 由里程计发），这与 nx_sensor 的行为正好匹配。**不要**在 slam launch 加 odom→base_link 的 static_transform。 |
| nx_sensor 不发 map→odom | nx_sensor **完全不碰 map frame**（grep 确认无 `map` 字样） | `map→odom` 只能 slam_toolbox 发（建图 async 模式 + 定位 localization 模式都发）。无冲突。 |

**Generator 必记的三条铁律**（来自上表）：
1. **QoS 不匹配会静默失败**（决策 4 的核心问题，spec 头号风险）。
2. **scan 已在 `base_link` 系**，slam_toolbox `base_frame=base_link`，无 `laser_frame`，无 base_link→laser static_transform。
3. **odom→base_link 由 nx_sensor 发**，slam_toolbox 只发 map→odom，二者职责清晰，TF 链无多源。

---

## 3. 关键设计决策（已拍板，给推荐 + 理由）

### 决策 1：slam_toolbox 模式 → **推荐 (b) 同包内 launch arg 切换 mapping/localization 两个 executable，不写死**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| (a) 只写 async 建图，定位交给 amcl | 简单 | amcl 要 OccupancyGrid pgm/yaml，slam_toolbox 存的是 `.posegraph`+.data，格式不通；amcl 是粒子滤波，2D scan 场景不如 slam_toolbox 自带的 scan-matching localization 精确；TECH_DECISIONS 第三节"不用 amcl"原则 | ❌ 不选 |
| **(b) launch arg `mode:=mapping\|localization` 切换 executable** | 一份 launch 文件两用；建图用 `async_slam_toolbox_node`，定位用 `localization_slam_toolbox_node`（Humble 自带）；共用同一份 params；运维只改一个 arg | Generator 要写 `PythonExpression`/`IfCondition` 逻辑切 executable | ✅ **推荐** |
| (c) 写两个 launch 文件（slam_mapping.launch.py + slam_localization.launch.py） | 各自独立简单 | 重复（params 加载、QoS remap、node name 都一样），维护两份易漂移 | ❌ 不选，重复 |

**理由总结**：(b) 用 `LaunchConfiguration('mode')` + `IfCondition`/`OpaqueFunction` 在 `async_slam_toolbox_node`（mapping）和 `localization_slam_toolbox_node`（localization）间切换。两者**参数名几乎一致**（slam_toolbox 同源代码），共用一份 `slam_toolbox.yaml`。定位模式额外参数 `map_file_name` + `map_start_pose`（载入建图产物）。详见 §6.2。

**关键约束**：
- 建图模式 (`async_slam_toolbox_node`)：从头建图，`/map` 实时生长，发 `map→odom`。完成后调 `/slam_toolbox/save_map` service（`slam_toolbox/srv/SaveMap`，存 `.posegraph` + `.data` 到磁盘）。
- 定位模式 (`localization_slam_toolbox_node`)：启动即载入 `map_file_name` 指向的 `.posegraph` + `.data`，**initial_pose** 设建图起始点，持续 scan-matching 发 `map→odom`，`/map` 发布**静态**载入的地图（global_costmap static_layer 终于有数据）。
- **二者互斥**：NX 上同一时刻只跑其一（建图时不导航，导航时不建图）。launch 顶部注释写明。

### 决策 2：map→odom 归属 → **推荐 slam_toolbox 独占（不用 amcl，不用 FAST_LIO，nx_sensor 不碰 map frame）**

| 方案 | map→odom 发布者 | 问题 | 结论 |
|---|---|---|---|
| (a) amcl | amcl 粒子滤波 | 见决策 1（格式不通 + 精度差 + TECH_DECISIONS 禁 amcl） | ❌ |
| **(b) slam_toolbox** | slam_toolbox（async 模式建图时 + localization 模式定位时都发） | nx_sensor 发 `/odom` 和 odom→base_link TF（不变），slam_toolbox 在其上加 `map→odom`，无多源 | ✅ **推荐** |
| (c) FAST_LIO | camera_init→body 改名 | MID360 没装，FAST_LIO 跑不起来（本阶段前提就是降级） | ❌（硬件不具备） |
| (d) 多源（slam_toolbox + static_transform_publisher 兜底） | 二者都发 | TF 跳变，狗乱动 | ❌ **Critical 禁止** |

**理由总结**：(b) 是唯一可行方案。slam_toolbox 默认行为就是发 `map→odom`（它假设 odom→base_link 由里程计节点发，正好是 nx_sensor）。**Critical 检查**：`ros2 topic info /tf -v` 的 publisher 列表里，`map→odom` 只能出现 `slam_toolbox` 一个节点——**不能**有 `static_transform_publisher`、`amcl`、`nav2` 任何节点同时发。

**与阶段D nav2_3d.launch.py 的冲突**：阶段D launch 有两个 `static_transform_publisher`（`map→camera_init`、`body→base_link`），是为 FAST_LIO 的 `camera_init→body` TF 桥接的。**阶段F 不能复用 nav2_3d.launch.py**——它的 TF 桥会与 slam_toolbox 的 `map→odom` 冲突（`map→camera_init→body→base_link` 这条链假设 FAST_LIO 发 `camera_init→body`，但阶段F 没有 FAST_LIO，链断裂 + slam_toolbox 发的 `map→odom` 与 static `map→camera_init` 抢 `map` 的子节点）。所以阶段F 需要一份**适配版 Nav2 launch**（见决策 7）。

### 决策 3：frame/TF 架构 → **推荐 map→odom（slam_toolbox）→base_link（nx_sensor），scan 在 base_link 系，无 laser_frame**

阶段F TF 树（目标态）：

```
map                         (全局 frame, Nav2 global_costmap + 阶段E goal pose 用)
 │
 │  ← 发布者: slam_toolbox (async 或 localization 模式)
 │    topic: /tf, 频率: 随 scan-matching (≈10Hz, 跟 /scan)
 │    备注: map→odom 是 slam_toolbox 的 scan-matching 校正量
 │          (里程计漂移的补偿, 启动时 identity)
 │
 ▼
odom                        (里程计 frame, Nav2 local_costmap + behavior_server 用)
 │
 │  ← 发布者: nx_sensor_node (阶段A, go2w-sensor.service)
 │    topic: /tf + /odom (nav_msgs/Odometry)
 │    频率: 50Hz (odom_rate 参数)
 │    备注: 死推算 (IMU yaw + 暂无平移, xy 暂为 0)
 │          ⚠️ xy=0 是已知限制, slam_toolbox 的 scan-matching 会补偿
 │
 ▼
base_link                   (机器人本体 frame, slam_toolbox base_frame + Nav2 robot_base_frame
 │                          + nx_sensor /scan 的 frame_id, 三者一致)
 │
 │  ← /scan 直接在 base_link 系 (nx_sensor 投影时假设激光与本体零偏移)
 │    无 base_link→laser static_transform (激光与本体重合)
 │
 └── imu_link               (nx_sensor /imu 的 frame, slam_toolbox 不用)
```

**谁发布哪个 TF（Critical：不能多源）**：

| TF 段 | 唯一发布者 | topic/方式 | 频率 | 备注 |
|---|---|---|---|---|
| `map → odom` | **slam_toolbox** | `/tf` | ≈10Hz（跟 scan） | scan-matching 校正量。**严禁** static_transform_publisher/amcl/nav2/FAST_LIO 同时发 |
| `odom → base_link` | **nx_sensor_node** | `/tf` | 50Hz | 死推算（阶段A 红线，不动）。**严禁** slam_toolbox/static_transform_publisher 同时发 |
| `base_link → laser` | **不需要** | — | — | `/scan` frame_id 直接是 `base_link`（nx_sensor 第 228 行），激光与本体重合 |
| `base_link → imu_link` | （可选）static_transform | `/tf` | 一次性 | nx_sensor 发 `/imu` frame=`imu_link`，若 rviz 要看 IMU 可加；slam_toolbox/Nav2 都不需要，**可不加** |

**Critic 静态审 TF 检查项**：
1. slam launch 文件**不含** `static_transform_publisher` 发 `map→odom` 或 `odom→base_link`（多源 Critical）。
2. slam_toolbox params `base_frame: base_link` + `odom_frame: odom` + `map_frame: map`（与上树一致）。
3. Nav2 params（适配版，决策 7）`bt_navigator.global_frame: map` + `robot_base_frame: base_link`（能 lookup `map→base_link`，链 `map→odom→base_link` 完整）。
4. **不启 amcl**（与阶段D 一致，决策 2），lifecycle `node_names` 不含 amcl。

### 决策 4：/scan QoS 匹配 → **推荐 (a) 让 slam_toolbox 订阅用 RELIABLE（匹配 nx_sensor 发布端），不动 nx_sensor**

这是本阶段**头号风险**（spec §2 已证 QoS 不匹配会静默失败）。修复路径：

| 方案 | 做法 | 优点 | 缺点 | 结论 |
|---|---|---|---|---|
| **(a) 改 slam_toolbox 订阅 QoS 为 RELIABLE（匹配 nx_sensor）** | 在 slam launch 里给 slam_toolbox 节点加 `ros_arguments=['--ros-args', '-r', ...]` 不够；slam_toolbox 的 scan 订阅 QoS 由参数 `scan_topic` 的 QoS profile 决定。slam_toolbox Humble 源码：默认 `rclcpp::SensorDataQoS()`（BEST_EFFORT）。**Humble slam_toolbox 支持参数 `use_sensor_data_qos`（bool，默认 true）**——设 `false` 即切回默认 RELIABLE QoS。 | 不动 nx_sensor（阶段A 红线保住）；一行参数；slam_toolbox 官方支持 | 若未来 /scan 改回 sensor_data QoS，要同步改回——文档注明 | ✅ **推荐** |
| (b) 改 nx_sensor 发布 QoS 为 sensor_data（BEST_EFFORT） | 改 nx_sensor_node.py 第 107 行 `create_publisher(LaserScan, '/scan', qos_profile_sensor_data)` | 与 ROS 惯例（传感器 BEST_EFFORT）一致 | **违反阶段A 红线**（改 nx_sensor）；nx_sensor 的 `/scan` 还被其他订阅者（Nav2 costmap obstacle_layer，RELIABLE）消费，改 QoS 可能破坏 Nav2；阶段A 实测链路（commit 1218088/86887a7）要重测 | ❌ 不选（红线） |
| (c) 双向改：nx_sensor 发 sensor_data，slam_toolbox 默认 | 同 (b) 缺点 | — | ❌ |
| (d) 加 qos 命令行参数覆盖 | `ros2 run ... --ros-args --qos-reliability best_effort` 对 publisher 无效（nx_sensor 用代码 create_publisher） | — | 不解决根因 | ❌ |

**理由总结**：(a) 是唯一不动 nx_sensor 的方案。**Generator 实现**：在 `slam_toolbox.yaml` 加 `use_sensor_data_qos: false`（slam_toolbox Humble 参数，见 §6.1）。**验证方法**（写入 runbook）：启动后 `ros2 topic info /scan -v` 看 slam_toolbox 订阅者 QoS 是 `RELIABILITY: RELIABLE`（与 nx_sensor 发布端 `RELIABLE` 匹配），且 `ros2 topic hz /map` 有数据（建图在工作）。

**⚠️ Generator 必读的反直觉点**：slam_toolbox 文档大量示例用 `use_sensor_data_qos: true`（默认），因为大多数 LiDAR 驱动发 BEST_EFFORT。但 **nx_sensor 发的是 RELIABLE**（代码默认 QoS + 第 104 行注释明确"用默认 RELIABLE"）。所以这里**必须显式设 `false`**，否则踩 QoS 不匹配的静默失败坑——这是 Critic 静态审的 **Critical** 项（见 eval-rubric-slam #4）。

**额外保护**（可选，Generator 评估）：在 slam launch 里用 `ros_arguments` 强制 slam_toolbox 节点订阅 QoS——但 slam_toolbox 的 scan 订阅在节点内部用 `create_subscription`，外部 `--ros-args` 无法覆盖单个 topic 的 QoS，只能靠 `use_sensor_data_qos` 参数。所以**只走 (a)**。

### 决策 5：launch 组织 → **推荐 (a) 复用并更新 `src/go2w_nav/launch/slam.launch.py`（休眠雏形），独立 launch，不集成 Nav2**

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| **(a) 复用并更新 `slam.launch.py`（休眠，commit 1218088 前的 2D 路线雏形）** | 休眠文件已有 `async_slam_toolbox_node` 加载 `slam_toolbox.yaml` 的最小骨架（已读，27 行），只需加 `mode` arg 切 executable + `use_sensor_data_qos: false` + `initial_pose`（定位模式）；与 nx_sensor/Nav2 launch **并存不冲突**（不同 node 名、不同 TF 发布权） | 休眠文件只支持 async 模式，要加 localization 分支 | ✅ **推荐** |
| (b) 复活 `src/go2w_bringup/launch/search.launch.py`（旧 2D 全系统） | 一键起 slam + nav2 + amcl | search.launch.py 混起 slam_toolbox + amcl（互斥）+ 已废弃的 `nav2_recoveries`；阶段D/E spec 已明确不复用（违反 TECH_DECISIONS + 旧包名） | ❌ 不选（阶段D spec §0 红线 + 决策 3 amcl 互斥） |
| (c) 把 slam_toolbox 集成进 nav2_3d.launch.py | 一键起全套 | nav2_3d.launch.py 的 TF 桥（map→camera_init, body→base_link）与 slam_toolbox 抢 map→odom（决策 2）；建图和导航混在一个 launch，调试困难 | ❌ 不选（TF 冲突 + 职责混杂） |

**理由总结**：(a) 是"独立 launch + 人工编排启动顺序"。NX 上跑全套的启动顺序（写入 `docs/slam_runbook.md`）：
```
# 终端1: nx_sensor (阶段A, systemd 自启或手动)
sudo systemctl start go2w-sensor.service    # 发 /scan /imu /odom + odom→base_link TF
# 验证: ros2 topic hz /scan  (应 ~10Hz)

# 终端2 (建图模式): slam_toolbox async
ros2 launch go2w_nav slam.launch.py mode:=mapping
# 验证: ros2 topic hz /map  (应 ≈0.5Hz, map_update_interval=2.0)
# 验证: ros2 run tf2_ros tf2_echo map odom  (应有输出)

# 推狗走遍房间, /map 在 rviz 实时生长

# 建图完成, 存图:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: data, timeout: 5000}"
# 产物: data.posegraph + data.data (slam_toolbox 原生格式, 在当前目录)

# 终端3 (定位模式, 替换终端2): slam_toolbox localization
ros2 launch go2w_nav slam.launch.py mode:=localization map_file:=/path/to/data
# 载入建好的图, 持续 scan-matching 发 map→odom, /map 发静态载入图

# 终端4: Nav2 (阶段D 适配版, 见决策 7)
ros2 launch go2w_nav nav2_slim.launch.py
# 吃 slam_toolbox 的 /map + map→odom→base_link TF, 自主导航

# 终端5: 阶段A 控狗 + 阶段A/B/E web
ros2 run go2w_bridge nx_motion_node
python3 web/nx_web_server.py
```

**关键约束**：
1. **不启动 nx_sensor / nx_motion_node / nx_web**（阶段A 独立 launch / systemd）。
2. **不启动 amcl / map_server / Nav2**（slam.launch.py 只管 slam_toolbox；Nav2 走独立的 nav2_slim.launch.py）。
3. **建图 vs 定位互斥**：终端2（mapping）和终端3（localization）只跑其一。launch 顶部注释写明。
4. **mode arg 默认值**：`mode:=mapping`（建图是主流程，定位是建完图后切换）。

### 决策 6：slam_toolbox 参数 → **推荐 复用并更新 `slam_toolbox.yaml`（休眠雏形），对齐 nx_sensor 的 /scan 特征**

休眠 `slam_toolbox.yaml`（已读，72 行）大部分参数合理（Ceres solver、loop closing、resolution 0.05）。需修正/新增项：

| 参数 | 休眠值（错/缺） | 正确值 | 理由 |
|---|---|---|---|
| `base_frame` | `base_link`（正确） | `base_link`（不变） | nx_sensor `/scan` frame_id=`base_link`（第 228 行），决策 3 |
| `odom_frame` | `odom`（正确） | `odom`（不变） | nx_sensor `/odom` + odom→base_link TF frame=`odom`（第 201/213 行） |
| `map_frame` | `map`（正确） | `map`（不变） | Nav2 global_costmap.global_frame + 阶段E goal frame |
| `scan_topic` | `/scan`（正确） | `/scan`（不变） | nx_sensor 发的 topic 名 |
| **`use_sensor_data_qos`** | **缺失**（默认 true → BEST_EFFORT） | **`false`**（RELIABLE） | 决策 4 头号风险：nx_sensor `/scan` 发 RELIABLE（第 107/104 行），不匹配会静默失败 |
| `max_laser_range` | `8.0`（错，超 nx_sensor） | **`10.0`**（与 nx_sensor `/scan range_max=10.0` 第 235 行一致） | 休眠值 8.0 是按 MID360 写的（SDK_CAPABILITIES §1.3 0.15~8.0m）；nx_sensor 投影的 `/scan range_max=10.0`，slam_toolbox 要能用到全部 10m 数据 |
| `minimum_travel_distance` | `0.3` | `0.3`（不变，室内合理） | 推狗走时每 0.3m 加一个节点 |
| `minimum_travel_heading` | `0.3` | `0.3`（不变，≈17°） | 室内转弯密度合理 |
| `resolution` | `0.05` | `0.05`（不变） | 与 Nav2 costmap resolution 0.05 一致（阶段D nav2_params_3d.yaml 第 124/181 行），地图栅格对齐 |
| `map_update_interval` | `2.0` | `2.0`（不变） | 0.5Hz /map 发布，Nav2 global_costmap update_frequency 1.0Hz 够消化 |
| `transform_timeout` | `0.2` | `0.2`（不变，与 Nav2 一致） | TF lookup 容忍 |
| `tf_buffer_duration` | `30.0` | `30.0`（不变） | slam_toolbox scan-matching 要历史 scan，长 buffer |
| `mode`（仅 localization 用） | 缺失 | mapping 模式不用；localization 模式由 executable 决定（`localization_slam_toolbox_node`） | 见决策 1 |
| `map_file_name`（仅 localization） | 缺失 | localization 模式 launch arg 传入（`.posegraph` 路径，无扩展名） | slam_toolbox localization 启动载入 |
| `map_start_pose`（仅 localization） | 缺失 | `[0.0, 0.0, 0.0]`（建图起始点，identity） | 建图起始位姿 = map 原点 |
| `use_sim_time` | 缺失 | launch 传 `false`（实车） | 实车不用 sim time |

**新增项**（休眠文件完全没有，必须加）：
- `use_sensor_data_qos: false`（决策 4，头号风险）
- localization 模式专属：`map_file_name`、`map_start_pose`、`mode: localization`（由 launch 按 mode arg 注入）

**实现约束**：
- **不删**休眠文件已有的正确参数（Ceres solver / loop closing / resolution / map_update_interval）。
- **max_laser_range 从 8.0 改 10.0**（对齐 nx_sensor，休眠值是 MID360 遗留）。
- **加 use_sensor_data_qos: false**（头号风险）。
- localization 专属参数由 launch 的 `IfCondition`/`OpaqueFunction` 按 mode 注入（不写死在 yaml，避免 mapping 模式误载入地图）。

### 决策 7：和阶段D Nav2 协作 → **推荐 新建 `nav2_slim.launch.py` + 复用 `nav2_params_3d.yaml` 做 minimal diff 派生 `nav2_params_slim.yaml`，不污染阶段D 原文件**

阶段D 的 `nav2_params_3d.yaml` + `nav2_3d.launch.py` 是为 FAST_LIO 路线写的，**不能直接用**于 slam_toolbox 路线。具体不兼容点：

| 阶段D 假设（FAST_LIO 路线） | 阶段F 现实（slam_toolbox 路线） | 阶段F 适配 |
|---|---|---|
| `bt_navigator.odom_topic: /Odometry`（大写，FAST_LIO 原生，nav2_params_3d.yaml 第 23 行） | nx_sensor 发 `/odom`（小写，第 106 行），**没有 `/Odometry`** | 适配版 yaml 改 `odom_topic: /odom` |
| `nav2_3d.launch.py` 启 `static_transform_publisher` × 2（`map→camera_init`、`body→base_link`，为 FAST_LIO 的 `camera_init→body` 桥接） | slam_toolbox 直接发 `map→odom`，nx_sensor 直接发 `odom→base_link`，**不需要 TF 桥** | 适配版 launch **删掉**两个 static_transform_publisher |
| `nav2_3d.launch.py` 启 `pointcloud_to_laserscan`（MID360 点云 → /scan） | nx_sensor 已发 `/scan`（狗自带 LiDAR 投影），**不需要 pointcloud_to_laserscan** | 适配版 launch **删掉** pointcloud_to_laserscan 节点（否则它订 `/livox/lidar` 无数据，无害但多余；且若误订 `/scan` 会循环） |
| `global_costmap.static_layer` 订 `/map` 但 FAST_LIO 不发 OccupancyGrid（空层，nav2_params_3d.yaml 第 187-189 行注释） | slam_toolbox **发 `/map`**（OccupancyGrid），static_layer 终于有数据 | 适配版 yaml **保留** static_layer（nav2_params_3d.yaml 已有，无需改，但注释更新："slam_toolbox 路线下此层生效"） |
| `local_costmap.voxel_layer.pointcloud.topic: /livox/lidar`（MID360 点云） | 阶段F **没有 MID360**，只有 `/scan` | 适配版 yaml：voxel_layer **禁用**（`enabled: false`）或**删除**；local_costmap 退化为 obstacle_layer(/scan) + inflation_layer（单保险，因为没 3D 点云源） |

**方案选择**：

| 方案 | 做法 | 优点 | 缺点 | 结论 |
|---|---|---|---|---|
| (a) 直接改 `nav2_params_3d.yaml` + `nav2_3d.launch.py` 适配 slam_toolbox | 改 odom_topic、删 TF 桥、删 p2l、禁 voxel_layer | 一份文件 | **破坏阶段D FAST_LIO 路线**（commit 9b0c397 已 gan 收敛，改了要重审）；未来 MID360 装好要再改回 | ❌ 不选（破坏已收敛阶段） |
| **(b) 新建 `nav2_params_slim.yaml` + `nav2_slim.launch.py`（slim = 降级/精简，slam_toolbox 路线专属）** | 派生自 3d 版做 minimal diff；launch 不含 TF 桥/p2l；yaml 改 odom_topic=`/odom`、禁 voxel_layer | 阶段D 原文件零改动（红线保住）；slim 命名清晰表达"降级"；未来切回 FAST_LIO 路线只需换 launch | 多两份文件（slim params + slim launch） | ✅ **推荐** |
| (c) 用 launch arg 在 nav2_3d.launch.py 里切 FAST_LIO/slam_toolbox 两套配置 | 一个 launch 两模式 | 单文件 | launch 逻辑复杂（IfCondition 嵌套）；阶段D launch 已 gan 收敛，加分支要重审 | ❌ 不选（复杂 + 破坏收敛） |

**理由总结**：(b) 是"兄弟文件"策略。`nav2_params_slim.yaml` 是 `nav2_params_3d.yaml` 的 **slam_toolbox 适配派生版**，diff 极小（4 处：odom_topic、voxel_layer.enabled、static_layer 注释、可能删 voxel_layer 整段）。`nav2_slim.launch.py` 比 `nav2_3d.launch.py` **简单**（无 TF 桥、无 p2l，只 Nav2 + lifecycle_manager）。详见 §5.1、§6.3。

**关键约束**：
- **不改** `nav2_params_3d.yaml` / `nav2_3d.launch.py`（阶段D 红线，commit 9b0c397 已 gan 收敛）。
- **nav2_slim.launch.py 不启** TF 桥、pointcloud_to_laserscan、amcl、map_server、slam_toolbox（slam_toolbox 由 slam.launch.py 启）。
- **nav2_slim.launch.py 不启** nx_sensor / nx_motion_node / nx_web（阶段A 独立 launch）。
- **lifecycle_manager.node_names 不含 amcl / map_server / slam_toolbox**（slam_toolbox 由独立 launch 启；map_server 不需要，slam_toolbox 自己发 `/map`）。

---

## 4. 关键设计决策总表（Generator 速查）

| # | 决策点 | 推荐 | 一句话理由 |
|---|---|---|---|
| 1 | slam_toolbox 模式 | launch arg `mode:=mapping\|localization` 切 executable（async / localization node） | 一份 launch 两用，共用 params |
| 2 | map→odom 归属 | slam_toolbox 独占（不用 amcl/FAST_LIO/nx_sensor） | nx_sensor 不碰 map frame，无多源 |
| 3 | frame/TF 架构 | map→odom（slam_toolbox）→base_link（nx_sensor）；scan 在 base_link 系，无 laser_frame | 决策 2 + nx_sensor /scan frame=base_link |
| 4 | /scan QoS 匹配 | slam_toolbox `use_sensor_data_qos: false`（匹配 nx_sensor RELIABLE） | nx_sensor 发 RELIABLE，不匹配静默失败（头号风险） |
| 5 | launch 组织 | 复用更新 `slam.launch.py`，独立 launch 不集成 Nav2 | 与 nx_sensor/Nav2 并存，职责单一 |
| 6 | slam_toolbox 参数 | 复用更新 `slam_toolbox.yaml`，max_laser_range 8.0→10.0，加 use_sensor_data_qos:false | 对齐 nx_sensor /scan 特征 |
| 7 | 和阶段D Nav2 协作 | 新建 `nav2_slim.launch.py` + `nav2_params_slim.yaml`（slam_toolbox 适配派生），不改阶段D 原文件 | 保住阶段D 红线，diff 极小 |

---

## 5. 新建/修改文件清单（文件级 + 内容要点，Generator 直接实现）

### 5.1 修改文件

#### `src/go2w_nav/config/slam_toolbox.yaml`（核心，休眠文件修正 + 新增 QoS）
**职责**：slam_toolbox 参数（mapping + localization 共用）。

**修正/新增项**（休眠文件 → 正确值）：

| 参数 | 休眠值 | 正确值 | 理由 |
|---|---|---|---|
| `use_sensor_data_qos` | **缺失** | `false` | 决策 4 头号风险：nx_sensor `/scan` 发 RELIABLE（nx_sensor_node.py:107/104），slam_toolbox 默认 BEST_EFFORT 会静默失败 |
| `max_laser_range` | `8.0` | `10.0` | 对齐 nx_sensor `/scan range_max=10.0`（nx_sensor_node.py:235）；休眠值 8.0 是 MID360 遗留 |
| `base_frame` / `odom_frame` / `map_frame` | `base_link` / `odom` / `map`（不变） | 同 | 决策 3，与 nx_sensor 实测 frame 一致 |
| `scan_topic` | `/scan`（不变） | 同 | nx_sensor 发的 topic 名 |
| 其余（Ceres/loop/resolution/map_update_interval） | 休眠值合理 | 不变 | resolution 0.05 与 Nav2 costmap 对齐 |

**新增（localization 模式专属，由 launch 按 mode 注入，不写死 yaml）**：
- `map_file_name`：`.posegraph` 路径（无扩展名），launch arg `map_file` 传入
- `map_start_pose`：`[0.0, 0.0, 0.0]`（建图起始点）
- `mode: localization`（由 executable 决定，yaml 可不显式写）

**实现约束**：
- **不删**休眠文件已有的正确参数。
- **max_laser_range 改 10.0**（对齐 nx_sensor）。
- **加 `use_sensor_data_qos: false`**（Critical）。
- localization 专属参数由 launch 注入（mapping 模式不载入地图）。

#### `src/go2w_nav/launch/slam.launch.py`（核心，休眠文件重写为双模式）
**职责**：启动 slam_toolbox（mapping 或 localization），加 QoS 参数。

**休眠文件现状**（已读，27 行）：只起 `async_slam_toolbox_node` 加载 yaml，无 mode 切换、无 QoS 参数、无 localization 分支。

**重写要点**：

| 项 | 休眠值/结构 | 正确值/结构 | 理由 |
|---|---|---|---|
| `mode` arg | 缺失 | `DeclareLaunchArgument('mode', default_value='mapping')`（可选值 mapping/localization） | 决策 1 |
| executable | 写死 `async_slam_toolbox_node` | `IfCondition(mode==mapping)` → `async_slam_toolbox_node`；`IfCondition(mode==localization)` → `localization_slam_toolbox_node` | 决策 1 双模式 |
| `map_file` arg | 缺失 | `DeclareLaunchArgument('map_file', default_value='')`（localization 模式用，path 无扩展名） | localization 载入建图产物 |
| `use_sensor_data_qos` | 缺失（默认 true） | yaml 已设 false（决策 4），launch 不覆盖 | 由 yaml 统一 |
| localization 额外 params | 缺失 | `mode==localization` 时注入 `{'map_file_name': map_file, 'map_start_pose': [0.0,0.0,0.0]}` | 决策 1 |
| TF 桥 static_transform | 缺失（正确，不应有） | **不加**（决策 3，slam_toolbox 发 map→odom，nx_sensor 发 odom→base_link，无 TF 桥） | 避免多源 |
| `use_sim_time` | 默认 false | 保留 `false` | 实车 |

**最终 launch 结构**（Generator 实现，示意）：
```
LaunchDescription:
  - DeclareLaunchArgument: mode (default 'mapping'), map_file (default ''), use_sim_time (false)
  - Node (IfCondition mode==mapping):
      package=slam_toolbox, executable=async_slam_toolbox_node, name=slam_toolbox
      parameters=[slam_toolbox.yaml, {use_sim_time}]
      output=screen
  - Node (IfCondition mode==localization):
      package=slam_toolbox, executable=localization_slam_toolbox_node, name=slam_toolbox
      parameters=[slam_toolbox.yaml, {use_sim_time, map_file_name: map_file, map_start_pose: [0,0,0]}]
      output=screen
```

**实现约束**：
- **不启动** nx_sensor / nx_motion_node / nx_web / Nav2 / amcl / map_server（独立 launch）。
- **不加** TF 桥 static_transform（决策 3）。
- **mapping vs localization 互斥**：两个 Node 用 `IfCondition` 互斥（同一 name=slam_toolbox，不会同时起）。launch 顶部注释写明。

### 5.2 新建文件

#### `src/go2w_nav/config/nav2_params_slim.yaml`（阶段D 适配派生版，slam_toolbox 路线专属）
**职责**：Nav2 全栈参数，派生自 `nav2_params_3d.yaml`，做 slam_toolbox 适配 minimal diff。

**diff（相对 nav2_params_3d.yaml）**：

| 节点 | 参数 | 3d 版（FAST_LIO 路线） | slim 版（slam_toolbox 路线） | 理由 |
|---|---|---|---|---|
| `bt_navigator` | `odom_topic` | `/Odometry`（大写，FAST_LIO 原生） | **`/odom`**（小写，nx_sensor 发） | 决策 7，nx_sensor 发 `/odom`（nx_sensor_node.py:106） |
| `local_costmap` | `plugins` | `["obstacle_layer", "voxel_layer", "inflation_layer"]`（双保险） | **`["obstacle_layer", "inflation_layer"]`**（单保险，删 voxel_layer） | 决策 7，slam_toolbox 路线无 3D 点云源，voxel_layer 订 `/livox/lidar` 无数据 |
| `local_costmap.voxel_layer` | 整段 | 存在 | **删除** | 同上 |
| `global_costmap.static_layer` | 注释 | "FAST_LIO 不发 OccupancyGrid，此层订阅 /map 无数据" | **更新注释**："slam_toolbox 发 /map（OccupancyGrid），此层生效" | 决策 7，slam_toolbox 路线下 static_layer 有数据 |
| `global_costmap.static_layer.map_subscribe_transient_local` | `true` | `true`（不变） | slam_toolbox 发 /map 用 TRANSIENT_LOCAL QoS（latched），static_layer 要 TRANSIENT_LOCAL 才能收到 |
| 其余（controller 速度/footprint/planner/behavior/bt plugin_lib_names） | 3d 版值 | **不变**（逐字复制） | 阶段D 已 gan 收敛，slim 版保持一致 |

**实现约束**：
- **不改** `nav2_params_3d.yaml`（阶段D 红线）。
- slim yaml 是 3d yaml 的**派生**（Generator 可先 cp 再改 4 处 diff），不是重写。
- **保留** `bt_navigator.plugin_lib_names`（Humble 必填，3d 版已正确，slim 复制）。
- **保留** controller_server Go2W 室内参数（max_vel_x=0.6 / footprint / tolerance，与 TECH_DECISIONS 第三节一致）。
- **保留** `local_costmap.obstacle_layer`（吃 /scan，主力），只删 voxel_layer。

#### `src/go2w_nav/launch/nav2_slim.launch.py`（阶段D 适配派生版 launch）
**职责**：启动 Nav2 stack（适配 slam_toolbox 路线），无 TF 桥、无 p2l。

**diff（相对 nav2_3d.launch.py）**：

| 项 | 3d 版 | slim 版 | 理由 |
|---|---|---|---|
| `pointcloud_to_laserscan` 节点 | 有（MID360 点云 → /scan） | **删除** | nx_sensor 已发 /scan（狗自带 LiDAR 投影） |
| `static_transform_publisher` × 2（map→camera_init, body→base_link） | 有（FAST_LIO TF 桥） | **删除** | slam_toolbox 发 map→odom，nx_sensor 发 odom→base_link，无 TF 桥 |
| `nav2_bringup.navigation_launch` params_file | `nav2_params_3d.yaml` | **`nav2_params_slim.yaml`** | 决策 7 |
| `lifecycle_manager.node_names` | 同 3d 版（controller/planner/behavior/bt_navigator/waypoint_follower，不含 amcl/map_server/slam_toolbox） | **不变** | slam_toolbox 由 slam.launch.py 启，map_server 不需要（slam_toolbox 发 /map） |
| TimerAction 编排 | 3d 版：p2l + TF 桥先起，Nav2 后起 | slim 版：**直接起 Nav2 + lifecycle_manager**（无前置依赖） | 无 p2l/TF 桥，无前置 |
| `/cmd_vel` remap | 无（零反转） | **无**（不变，决策 5） | 与阶段D 一致 |

**最终 launch 结构**（Generator 实现，示意）：
```
LaunchDescription:
  - DeclareLaunchArgument: params_file (default nav2_params_slim.yaml), use_sim_time (false)
  - Node: nav2_bringup navigation_launch (params_file, use_sim_time)
  - Node: nav2_lifecycle_manager lifecycle_manager_navigation
      (autostart=True, node_names=[controller_server, planner_server, behavior_server,
       bt_navigator, waypoint_follower])  # 不含 amcl/map_server/slam_toolbox
```

**实现约束**：
- **不改** `nav2_3d.launch.py`（阶段D 红线）。
- **不加** `/cmd_vel` remap（零反转，阶段D 决策 5 继承）。
- **不启** amcl / map_server / slam_toolbox（slam_toolbox 由 slam.launch.py 启）。
- **不启** nx_sensor / nx_motion_node / nx_web（阶段A 独立 launch）。

#### `docs/slam_runbook.md`（运维 SOP，约 80 行）
**职责**：阶段F 实跑 runbook（NX 恢复后用）。

**内容要点**：
1. **前置检查**：`go2w-sensor.service` 运行中、`ros2 topic hz /scan` ≈10Hz、`ros2 topic hz /odom` ≈50Hz、`ros2 topic info /scan -v` 看 nx_sensor 发布 QoS = RELIABLE。
2. **建图流程**（mode=mapping）：
   - 启动顺序（5 终端，见决策 5）。
   - 推狗走遍房间（阶段0 移动控制就绪后可用遥控，否则人推）。
   - rviz2 看 `/map` 实时生长（add display OccupancyGrid topic=/map）。
   - 建图完成，调 `/slam_toolbox/save_map` service 存 `.posegraph` + `.data`。
3. **定位流程**（mode=localization）：
   - kill 建图终端，启 `slam.launch.py mode:=localization map_file:=/path/to/data`。
   - 验证 `ros2 topic echo /map --once` 有载入的静态地图。
   - 验证 `ros2 run tf2_ros tf2_echo map odom` 有输出（slam_toolbox scan-matching）。
4. **Nav2 联调**（mode=localization 时）：
   - 启 `nav2_slim.launch.py`。
   - `ros2 action list` 含 `/navigate_to_pose`。
   - `ros2 topic info /tf -v`：map→odom 只 slam_toolbox 发，odom→base_link 只 nx_sensor 发。
   - `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"` 单点导航。
   - 浏览器"搜索客厅"端到端（阶段E 编排）。
5. **常见故障**：
   - `/map` 一直空 → **QoS 不匹配**（决策 4）：检查 `slam_toolbox.yaml` 有 `use_sensor_data_qos: false`；`ros2 topic info /scan -v` 看 slam_toolbox 订阅 QoS = RELIABLE。
   - TF `map→odom` 断 → slam_toolbox 没起或 mode 错；检查 `ros2 node list` 含 slam_toolbox。
   - TF `odom→base_link` 断 → nx_sensor（go2w-sensor.service）没起；`systemctl status go2w-sensor`。
   - Nav2 global_costmap 全空 → static_layer 没收到 /map（slam_toolbox 没在 localization 模式发 /map，或 QoS 不匹配）。
   - 狗乱转 → 检查 nav2_slim.launch.py 无 `/cmd_vel` remap（阶段D 决策 5 继承）。
   - 建图扭曲 → 推狗走太快（>0.5m/s）或 `/scan` 帧率掉；慢推 + 检查 `ros2 topic hz /scan`。
6. **切回 FAST_LIO 路线**（MID360 装好后）：kill slam + nav2_slim，启 nav2_3d.launch.py（阶段D 原文件）+ FAST_LIO launch（阶段C）。两路线互斥。

#### `gan-harness/eval-rubric-slam.md`（Critic 消费，约 180 行）
**职责**：阶段F 静态审 rubric（见本 spec §11 + 独立文件）。

### 5.3 不动文件清单（Generator 勿碰，Critic 会核对）

- `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py`（阶段E 红线）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py` / `nx_sensor_node.py`（阶段A 红线，`/scan` 来源 = 红线中的红线）
- `src/go2w_nav/config/nav2_params_3d.yaml` / `launch/nav2_3d.launch.py`（阶段D 红线，commit 9b0c397 已 gan 收敛）
- `src/go2w_nav/config/nav2_params.yaml` / `launch/nav2.launch.py`（2D 路线休眠，不动）
- `src/go2w_bringup/launch/search.launch.py`（旧 2D 全系统，不动）
- `src/go2w_nav/package.xml` / `CMakeLists.txt`（已 install config/launch，新增文件自动覆盖，**但要确认 install 目录更新**——Generator 若新加 `nav2_params_slim.yaml`，CMakeLists 的 install(DIRECTORY config ...) 通常 glob，无需改；但 Generator 要 `colcon build` 让新文件进 install share）

### 5.4 文件改动量预估

| 文件 | 类型 | 预估改动 | 改动性质 |
|---|---|---|---|
| `src/go2w_nav/config/slam_toolbox.yaml` | 修改 | ~10 行（加 use_sensor_data_qos + 改 max_laser_range + localization 参数注释） | 参数修正 |
| `src/go2w_nav/launch/slam.launch.py` | 修改（重写） | ~50 行（双模式 IfCondition + map_file arg） | launch 重写 |
| `src/go2w_nav/config/nav2_params_slim.yaml` | 新建 | ~200 行（派生自 3d 版，改 4 处 diff） | 派生 |
| `src/go2w_nav/launch/nav2_slim.launch.py` | 新建 | ~50 行（派生自 3d 版，删 p2l + TF 桥） | 派生 |
| `docs/slam_runbook.md` | 新建 | ~80 行 | 运维 SOP |
| `gan-harness/eval-rubric-slam.md` | 新建 | ~180 行 | Critic 消费 |

---

## 6. 配置详解（slam_toolbox.yaml + mode 切换 + nav2_slim diff）

### 6.1 slam_toolbox.yaml 关键段（Generator 实现）

```yaml
slam_toolbox:
  ros__parameters:
    # Frame 架构 (决策 3, 与 nx_sensor 实测一致)
    odom_frame: odom              # nx_sensor /odom + odom→base_link TF (nx_sensor_node.py:201,213)
    map_frame: map                # Nav2 global_costmap.global_frame + 阶段E goal frame
    base_frame: base_link         # nx_sensor /scan frame_id (nx_sensor_node.py:228), 无 laser_frame
    scan_topic: /scan             # nx_sensor 发的 topic 名

    # ⚠️ 头号风险 (决策 4): QoS 匹配 nx_sensor 发布端
    # nx_sensor /scan 用默认 RELIABLE QoS (nx_sensor_node.py:107, 第104行注释明确"默认 RELIABLE")
    # slam_toolbox 默认 use_sensor_data_qos=true (BEST_EFFORT), 不匹配会静默失败 (收不到 scan)
    use_sensor_data_qos: false    # 切回 RELIABLE, 匹配 nx_sensor

    # 运动阈值 (推狗走室内合理)
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3

    # 扫描缓冲 + 范围
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    max_laser_range: 10.0         # 对齐 nx_sensor /scan range_max=10.0 (nx_sensor_node.py:235)
                                  # (休眠值 8.0 是 MID360 遗留, 改 10.0)

    # 匹配/loop/求解器 (休眠值合理, 保留)
    use_scan_matching: true
    use_scan_barycenter: true
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    # ... (loop 参数 + 搜索空间 + 惩罚 + 角度搜索, 保留休眠值)

    # 地图更新
    map_update_interval: 2.0      # 0.5Hz /map, Nav2 global_costmap update 1Hz 够消化
    resolution: 0.05              # 与 Nav2 costmap resolution 0.05 对齐 (nav2_params_slim 复用 3d 值)
    transform_timeout: 0.2        # 与 Nav2 一致
    tf_buffer_duration: 30.0      # scan-matching 要历史 scan

    # 求解器 (保留)
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    # ... (其余 ceres 参数保留)

    # localization 模式专属 (由 launch 按 mode 注入, mapping 模式不写):
    #   map_file_name: /path/to/data  (无扩展名, .posegraph + .data)
    #   map_start_pose: [0.0, 0.0, 0.0]
    #   mode: localization
```

### 6.2 launch mode 切换逻辑（Generator 实现，slam.launch.py 核心）

```python
# 伪代码 (Generator 写真实 launch)
mode = LaunchConfiguration('mode')
is_mapping = IfCondition(PythonExpression(["'", mode, " == 'mapping'"]))
is_localization = IfCondition(PythonExpression(["'", mode, " == 'localization'"]))

mapping_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',     # 建图
    name='slam_toolbox',
    condition=is_mapping,
    parameters=[slam_yaml, {'use_sim_time': use_sim_time}],
    output='screen',
)

localization_node = Node(
    package='slam_toolbox',
    executable='localization_slam_toolbox_node',  # 定位 (替代 amcl)
    name='slam_toolbox',
    condition=is_localization,
    parameters=[slam_yaml, {
        'use_sim_time': use_sim_time,
        'map_file_name': map_file,               # .posegraph 路径 (无扩展名)
        'map_start_pose': [0.0, 0.0, 0.0],       # 建图起始点
        # 'mode': ' localization'  (executable 自带, 可不写)
    }],
    output='screen',
)
```

### 6.3 nav2_params_slim.yaml diff（相对 nav2_params_3d.yaml，4 处）

```yaml
# diff 1: bt_navigator.odom_topic
bt_navigator:
  ros__parameters:
    odom_topic: /odom            # 3d 版是 /Odometry (FAST_LIO), slim 改 /odom (nx_sensor)
    # 其余 (global_frame/robot_base_frame/plugin_lib_names) 不变

# diff 2: local_costmap.plugins (删 voxel_layer)
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]   # 3d 版是 [obstacle, voxel, inflation]
      # voxel_layer 整段删除 (slam_toolbox 路线无 3D 点云源)
      obstacle_layer: ...           # 不变 (吃 /scan)
      inflation_layer: ...          # 不变

# diff 3: global_costmap.static_layer 注释更新
global_costmap:
  global_costmap:
    ros__parameters:
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true    # 不变, slam_toolbox /map 是 TRANSIENT_LOCAL (latched)
        # 注释更新: "slam_toolbox localization 模式发 /map (OccupancyGrid), 此层生效"
        # (3d 版注释是 "FAST_LIO 不发 OccupancyGrid, 此层空")

# diff 4: 其余 (controller/planner/behavior/waypoint) 逐字复制 3d 版
```

---

## 7. TF 架构图（map→odom→base_link，谁发布）+ 与阶段D 对比

### 7.1 阶段F TF 树（slam_toolbox 路线）

```
map ──(slam_toolbox, /tf, ≈10Hz)──▶ odom ──(nx_sensor, /tf, 50Hz)──▶ base_link
                                         │
                                         └── /scan 直接在 base_link 系 (无 laser_frame)
```

### 7.2 与阶段D TF 树（FAST_LIO 路线）对比

| TF 段 | 阶段F（slam_toolbox） | 阶段D（FAST_LIO） |
|---|---|---|
| `map → odom` | slam_toolbox 发（scan-matching） | FAST_LIO 发 camera_init→body + TF 桥 static（map→camera_init, body→base_link）改名 |
| `odom → base_link` | nx_sensor 发（死推算，50Hz） | FAST_LIO + TF 桥（body→base_link identity） |
| `base_link → laser` | 不需要（/scan 在 base_link 系） | 不需要（pointcloud_to_laserscan target_frame=base_link） |
| TF 桥 static_transform | **无** | 2 个（map→camera_init, body→base_link） |

**关键差异**：阶段F **不需要任何 TF 桥**（slam_toolbox + nx_sensor 直接覆盖 map→odom→base_link），比阶段D 简单。这也是 nav2_slim.launch.py 比 nav2_3d.launch.py 简单的根因。

### 7.3 Critic 静态审 TF 检查项

1. slam.launch.py **不含** `static_transform_publisher`（无 TF 桥，决策 3）。
2. nav2_slim.launch.py **不含** `static_transform_publisher`（无 TF 桥，决策 7）。
3. slam_toolbox params `map_frame: map` + `odom_frame: odom` + `base_frame: base_link`。
4. Nav2 params（slim）`bt_navigator.global_frame: map` + `robot_base_frame: base_link`。
5. **map→odom 唯一发布者 = slam_toolbox**（`ros2 topic info /tf -v`）。
6. **odom→base_link 唯一发布者 = nx_sensor**（同上）。
7. **不启 amcl**（lifecycle node_names 不含 amcl；launch 无 amcl 节点）。

---

## 8. /scan QoS 匹配详解（头号风险，详见决策 4）

### 8.1 问题

- nx_sensor `/scan` 发布：`create_publisher(LaserScan, '/scan', 10)` → 默认 QoS = **RELIABLE + VOLATILE**（nx_sensor_node.py:107，第 104 行注释明确）。
- slam_toolbox `/scan` 订阅：默认 `rclcpp::SensorDataQoS()` = **BEST_EFFORT + VOLATILE**（slam_toolbox Humble 源码默认）。
- ROS2 QoS 兼容规则：RELIABLE publisher + BEST_EFFORT subscriber = **不兼容**（订阅者收不到任何消息，且无报错）。

### 8.2 症状（静默失败）

- slam_toolbox 启动正常，无报错。
- `ros2 topic hz /scan`（从 nx_sensor 看）≈10Hz 正常。
- `ros2 topic hz /map` = 0（slam_toolbox 没收到 scan，不建图，不发 map）。
- rviz 看 /map 一直空。
- `ros2 run tf2_ros tf2_echo map odom` 无输出（slam_toolbox 没建图，不发 map→odom）。

### 8.3 修复（决策 4 (a)）

`slam_toolbox.yaml` 加 `use_sensor_data_qos: false`。验证：
- `ros2 topic info /scan -v`：slam_toolbox 订阅者 QoS = `RELIABILITY: RELIABLE`（与 nx_sensor 发布端匹配）。
- `ros2 topic hz /map` ≈0.5Hz（map_update_interval=2.0）。
- rviz /map 有数据。

### 8.4 为什么不改 nx_sensor（决策 4 (b) 否决理由）

- nx_sensor 是阶段A 红线（commit 1218088/86887a7 已实测收敛）。
- nx_sensor `/scan` 还被 Nav2 costmap obstacle_layer 消费（nav2_params_slim.yaml obstacle_layer scan topic=/scan），Nav2 的 costmap 订阅 LaserScan 默认用 sensor_data QoS（BEST_EFFORT）—— **等等，这是另一个潜在 QoS 坑**：Nav2 costmap obstacle_layer 订 /scan 默认 QoS 也是 sensor_data（BEST_EFFORT），与 nx_sensor RELIABLE 不匹配，**Nav2 也收不到 scan**！

**⚠️ Generator 必须额外验证**（写入 eval-rubric-slam）：Nav2 costmap obstacle_layer 订 `/scan` 的 QoS。若 Nav2 默认 BEST_EFFORT，则 nav2_params_slim.yaml 的 obstacle_layer scan 段要加 `reliability: reliable`（Nav2 Humble obstacle_layer 支持参数 `observation_persistence`/`expected_update_rate`，QoS 由 `data_type` 隐含——LaserScan 默认 sensor_data QoS）。**修复方案**：要么 nx_sensor 改发 BEST_EFFORT（违反红线），要么 Nav2 obstacle_layer 显式订 RELIABLE（Nav2 Humble 支持 `reliability` 参数？需 Generator 查 Nav2 Humble 源码确认）。

**推荐处理**（Generator 二选一，写入 rubric Medium 项）：
- (i) 若 Nav2 Humble obstacle_layer 支持显式 QoS 参数 → nav2_params_slim.yaml obstacle_layer scan 段加 `reliability: reliable`。
- (ii) 若不支持 → **本 spec 决策 4 升级为双向 QoS 调整**：nx_sensor `/scan` 改发 BEST_EFFORT（违反阶段A 红线，但这是 ROS 传感器惯例，且 slam_toolbox + Nav2 都默认 BEST_EFFORT，统一改一次比各自适配省事）。**此选项需用户/Planner 批准放宽阶段A 红线**——Generator 在实现前先验证 (i) 是否可行，不可行则上报。

**Planner 预判**：Nav2 Humble costmap obstacle_layer 的 `observation_sources` 在 Humble 版**支持 `reliability` 和 `durability` 参数**（nav2_costmap_2d Observation 类）。所以 (i) 可行，slim yaml obstacle_layer scan 段加：
```yaml
scan:
  topic: /scan
  reliability: reliable         # 匹配 nx_sensor 发布端 (RELIABLE)
  durability: volatile
  # 其余不变
```
（global_costmap + local_costmap 的 obstacle_layer scan 段都要加。）Generator 实现时确认 Nav2 Humble 版本支持（`ros2 pkg xml nav2_costmap_2d` 看版本 ≥ Humble）。

---

## 9. slam_toolbox 模式切换（mapping → localization）

### 9.1 mapping 模式（建图）

- executable: `async_slam_toolbox_node`
- 输入: `/scan`（实时）+ `/odom`（nx_sensor）+ odom→base_link TF
- 输出: `/map`（OccupancyGrid，实时生长，map_update_interval=2.0）+ `map→odom` TF
- 完成动作: 调 `/slam_toolbox/save_map` service（`slam_toolbox/srv/SaveMap`），存 `.posegraph` + `.data`
- 交互: slam_toolbox 提供 `/slam_toolbox/loop_closure`、`/slam_toolbox/pause_new_measurements` 等 service（rviz plugin 可用）

### 9.2 localization 模式（定位，替代 amcl）

- executable: `localization_slam_toolbox_node`
- 输入: `/scan`（实时）+ `/odom`（nx_sensor）+ odom→base_link TF + 启动载入 `map_file_name`（.posegraph + .data）
- 输出: `/map`（OccupancyGrid，**静态**载入的地图，TRANSIENT_LOCAL QoS latched）+ `map→odom` TF（scan-matching 持续校正）
- 初始化: `map_start_pose: [0.0, 0.0, 0.0]`（建图起始位姿 = map 原点，identity）
- 优势 vs amcl: scan-matching 比粒子滤波精确；格式与 mapping 同源（.posegraph）；TECH_DECISIONS 第三节"不用 amcl"原则

### 9.3 模式切换运维流程（写入 runbook）

```
# Step 1: 建图
ros2 launch go2w_nav slam.launch.py mode:=mapping
# 推狗走遍房间, rviz 看 /map 生长
# 建图完成, 存图:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: /home/nx/maps/room, timeout: 5000}"
# 产物: /home/nx/maps/room.posegraph + room.data

# Step 2: Ctrl-C 停建图

# Step 3: 定位
ros2 launch go2w_nav slam.launch.py mode:=localization \
  map_file:=/home/nx/maps/room
# 载入图, 持续 scan-matching

# Step 4: Nav2 导航
ros2 launch go2w_nav nav2_slim.launch.py
```

---

## 10. 和阶段D Nav2 协作（/map 接线 + odom_topic + amcl 不启动）

### 10.1 /map 接线

- slam_toolbox（mapping 或 localization）发 `/map`（nav_msgs/OccupancyGrid）。
- Nav2 global_costmap.static_layer 订 `/map`（`map_subscribe_transient_local: true`，匹配 slam_toolbox 的 TRANSIENT_LOCAL latched QoS）。
- **阶段F 的 global_costmap 终于有静态地图**（阶段D FAST_LIO 路线下 static_layer 是空层）。

### 10.2 odom_topic 改动

- 阶段D `nav2_params_3d.yaml`: `odom_topic: /Odometry`（FAST_LIO 大写）。
- 阶段F `nav2_params_slim.yaml`: `odom_topic: /odom`（nx_sensor 小写）。
- **slim yaml 是独立文件**，不改 3d 版（决策 7）。

### 10.3 amcl 不启动

- slam_toolbox localization 模式替代 amcl（决策 1/2）。
- nav2_slim.launch.py 的 lifecycle `node_names` 不含 amcl（与阶段D 一致）。
- launch 不启 map_server（slam_toolbox 自己发 /map）。

### 10.4 /cmd_vel 零反转（继承阶段D 决策 5）

- nav2_slim.launch.py 的 controller_server **无 `/cmd_vel` remap**。
- Nav2 发 /cmd_vel → nx_motion_node 消费 → Move(vx,vy,vyaw) → 狗动（REP-103 全程一致，零反转）。
- **Critical**：Critic 静态审必须确认 nav2_slim.launch.py 无 `/cmd_vel` remap。

### 10.5 与阶段E 客户端契约（继承阶段D §2）

- action 名 `/navigate_to_pose`（不 remap）。
- goal frame `map`（bt_navigator.global_frame=map，与 rooms.yaml frame_id 一致）。
- TF 链 map→odom→base_link 完整（slam_toolbox + nx_sensor）。
- 阶段F **零改动阶段E 客户端**（nx_room_orchestrator.py 不动）。

---

## 11. Anti-slop / 反模式清单（Generator 自查）

- ❌ 不要改 nx_sensor_node.py（阶段A 红线，/scan 来源）
- ❌ 不要改 nav2_params_3d.yaml / nav2_3d.launch.py（阶段D 红线，commit 9b0c397 已收敛）
- ❌ 不要漏 `use_sensor_data_qos: false`（决策 4，头号风险，QoS 不匹配静默失败）
- ❌ 不要漏 Nav2 obstacle_layer scan 的 `reliability: reliable`（决策 8.4，第二个 QoS 坑）
- ❌ 不要在 slam.launch.py 加 TF 桥 static_transform（决策 3，slam_toolbox 发 map→odom，无桥）
- ❌ 不要在 slam.launch.py 同时起 mapping + localization（互斥，IfCondition）
- ❌ 不要在 nav2_slim.launch.py 加 TF 桥 / pointcloud_to_laserscan（决策 7，slim 路线不需要）
- ❌ 不要在 nav2_slim.launch.py 加 `/cmd_vel` remap（决策 10.4，零反转）
- ❌ 不要启动 amcl（决策 1/2，slam_toolbox localization 替代）
- ❌ 不要启动 map_server（slam_toolbox 自己发 /map）
- ❌ 不要把 `max_laser_range` 留 8.0（决策 6，nx_sensor /scan range_max=10.0，改 10.0）
- ❌ 不要把 `base_frame` 设成 `laser_frame`/`base_scan`（决策 3，nx_sensor /scan frame_id=base_link）
- ❌ 不要在 localization 模式漏 `map_file_name`（决策 1，载入建图产物）
- ❌ 不要把 localization 模式的 `map_start_pose` 设非零（建图起始点 = map 原点，identity）
- ❌ 不要复用 search.launch.py（阶段D spec §0 红线 + 决策 5）
- ❌ 不要把 slim 和 3d launch 混起（TF 冲突，map→odom 多源）
- ❌ 不要忘 `colcon build`（新增 nav2_params_slim.yaml 要进 install share，否则 ros2 launch 找不到）
- ❌ 不要把建图和导航混在一个 launch（决策 5，slam.launch.py 只管 slam_toolbox）

---

## 12. 静态审清单（对照 slam_toolbox Humble + nx_sensor 实测，Critic 用）

### 12.1 QoS 与数据流（5 项，权重最高——头号风险区）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 1 | `slam_toolbox.yaml` 含 `use_sensor_data_qos: false` | grep 有，且值=false | **Critical**（QoS 不匹配静默失败） |
| 2 | `slam_toolbox.yaml` `scan_topic: /scan` | = `/scan`（nx_sensor 发的 topic 名） | Critical |
| 3 | `nav2_params_slim.yaml` obstacle_layer scan 段含 `reliability: reliable` | grep 有（local + global costmap 两处） | **High**（Nav2 收不到 scan = 撞墙） |
| 4 | nx_sensor_node.py 未改 | `git diff` 为空 | **Critical**（阶段A 红线） |
| 5 | slam_toolbox `max_laser_range: 10.0` | = 10.0（对齐 nx_sensor /scan range_max） | Medium（8.0 不致错，但浪费数据） |

### 12.2 frame/TF 一致性（6 项）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 6 | `slam_toolbox.yaml` `base_frame: base_link` | = base_link（nx_sensor /scan frame_id） | Critical |
| 7 | `slam_toolbox.yaml` `odom_frame: odom` | = odom（nx_sensor /odom frame） | Critical |
| 8 | `slam_toolbox.yaml` `map_frame: map` | = map（Nav2 global_frame + 阶段E goal frame） | Critical |
| 9 | slam.launch.py 无 static_transform_publisher | grep 无（无 TF 桥，决策 3） | Critical（多源 TF） |
| 10 | nav2_slim.launch.py 无 static_transform_publisher | grep 无 | Critical |
| 11 | nav2_slim.launch.py 无 pointcloud_to_laserscan | grep 无（nx_sensor 已发 /scan） | High |

### 12.3 mode 切换正确（4 项）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 12 | slam.launch.py 含 `mode` arg（default mapping） | DeclareLaunchArgument mode | High |
| 13 | mode=mapping 起 `async_slam_toolbox_node` | IfCondition + executable | High |
| 14 | mode=localization 起 `localization_slam_toolbox_node` | IfCondition + executable | High |
| 15 | localization 注入 `map_file_name` + `map_start_pose` | parameters 含 | High |

### 12.4 Nav2 slim 适配（6 项）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 16 | `nav2_params_slim.yaml` `bt_navigator.odom_topic: /odom` | = /odom（小写，nx_sensor） | **Critical**（3d 版是 /Odometry，slim 必须改） |
| 17 | `nav2_params_slim.yaml` local_costmap.plugins 不含 voxel_layer | grep plugins 数组 = [obstacle, inflation] | Medium（voxel_layer 订 /livox/lidar 无数据，无害但浪费） |
| 18 | nav2_slim.launch.py lifecycle node_names 不含 amcl | grep 无 amcl | Critical（决策 2） |
| 19 | nav2_slim.launch.py lifecycle node_names 不含 map_server | grep 无 map_server | High（slam_toolbox 发 /map） |
| 20 | nav2_slim.launch.py 无 `/cmd_vel` remap | grep 无 remap | **Critical**（狗乱转，继承阶段D 决策 5） |
| 21 | nav2_slim.launch.py params_file 默认 = nav2_params_slim.yaml | DeclareLaunchArgument | High |

### 12.5 阶段D/A/E 契约不破坏（4 项）

| # | 检查项 | 通过标准 | 严重度 |
|---|---|---|---|
| 22 | 不改 nav2_params_3d.yaml / nav2_3d.launch.py | `git diff` 为空 | **Critical**（阶段D 红线） |
| 23 | 不改 nx_motion_node.py / nx_room_orchestrator.py / mock_nav2_action.py | `git diff` 为空 | Critical（阶段A/E 红线） |
| 24 | 不改 nx_web_server.py / nx_sensor_node.py | `git diff` 为空 | Critical（阶段A 红线） |
| 25 | bt_navigator.global_frame = map（slim yaml） | = map（阶段E goal frame 一致） | Critical |

---

## 13. 分 Sprint 实现顺序（配置先写，NX 恢复后联调）

### Sprint 1：slam_toolbox.yaml + slam.launch.py（纯配置，不依赖硬件/NX）
**目标**：slam_toolbox 配置正确，静态对照 slam_toolbox Humble + nx_sensor 实测。
**Features**：
- 改 `slam_toolbox.yaml`（加 use_sensor_data_qos:false + max_laser_range 10.0）
- 重写 `slam.launch.py`（mode arg + IfCondition 切 executable + map_file arg）
**Definition of Done**：
- 静态审 §12.1-12.3 全过（15 项）
- `ros2 launch go2w_nav slam.launch.py --show-args` 能解析
- **不依赖硬件**：纯配置审
- **不依赖 NX**：launch 结构审

### Sprint 2：nav2_params_slim.yaml + nav2_slim.launch.py（纯配置，不依赖硬件/NX）
**目标**：Nav2 适配版配置正确，diff 自 3d 版 minimal。
**Features**：
- 派生 `nav2_params_slim.yaml`（改 odom_topic + 删 voxel_layer + 加 obstacle_layer reliability + 注释更新）
- 派生 `nav2_slim.launch.py`（删 p2l + TF 桥 + 改 params_file）
**Definition of Done**：
- 静态审 §12.4-12.5 全过（10 项）
- `ros2 launch go2w_nav nav2_slim.launch.py --show-args` 能解析
- **不依赖硬件**：纯配置审
- **不依赖 NX**：launch 结构审

### Sprint 3：docs/slam_runbook.md + eval-rubric-slam.md（文档 + rubric）
**目标**：运维 SOP + Critic rubric 齐全。
**Features**：
- runbook：建图流程 + 定位流程 + Nav2 联调 + 常见故障 + 切回 FAST_LIO
- rubric：§12 静态审清单 + 权重（独立文件 eval-rubric-slam.md）
**Definition of Done**：
- runbook 覆盖 5 终端启动 + 建图/定位/导航 3 流程 + 6 常见故障
- rubric 覆盖 §12 全部 25 检查项 + 权重
- **不依赖硬件**：纯文档

### Sprint 4（NX 恢复后联调）：建图 + 定位 + Nav2 实跑
**目标**：NX 上线，推狗建图，载入图后 Nav2 自主导航。
**Features**：
- NX 上启动 nx_sensor + slam.launch.py mode=mapping
- 推狗走遍房间，rviz 看 /map 生长
- 调 /slam_toolbox/save_map 存图
- 切 mode=localization 载入图
- 启 nav2_slim.launch.py
- `ros2 action send_goal /navigate_to_pose` 单点导航
- 浏览器"搜索客厅"端到端（阶段E）
**Definition of Done**：
- `ros2 topic hz /map` ≈0.5Hz（建图中）
- `ros2 topic info /scan -v` slam_toolbox 订阅 QoS=RELIABLE（决策 4 验证）
- 建图完成，`/map` 覆盖房间
- localization 模式 `ros2 topic echo /map --once` 有载入图
- `ros2 topic info /tf -v` map→odom 只 slam_toolbox 发，odom→base_link 只 nx_sensor 发
- 单点导航：狗走到目标点，方向正确（无乱转）
- 房间搜索：狗走到客厅入口 + 房间内覆盖搜索
- **依赖硬件**：是（NX + 狗）
- **依赖阶段0**：是（推狗走 / 遥控）

---

## 14. 不依赖狗硬件/NX 的验证方法（Generator 必须实现并跑通）

### 14.1 静态验证（Sprint 1-3，纯配置审）

**yaml 语法验证**：
```bash
# slam_toolbox.yaml 语法
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/slam_toolbox.yaml'))"

# nav2_params_slim.yaml 语法
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/nav2_params_slim.yaml'))"

# 关键参数 grep
echo "=== QoS 头号风险 ==="
grep "use_sensor_data_qos" src/go2w_nav/config/slam_toolbox.yaml   # 应有, =false
echo "=== frame 架构 ==="
grep -E "base_frame|odom_frame|map_frame|scan_topic" src/go2w_nav/config/slam_toolbox.yaml
echo "=== max_laser_range ==="
grep "max_laser_range" src/go2w_nav/config/slam_toolbox.yaml   # 应 =10.0
echo "=== Nav2 slim odom_topic ==="
grep "odom_topic" src/go2w_nav/config/nav2_params_slim.yaml   # 应 =/odom
echo "=== Nav2 slim obstacle_layer reliability ==="
grep -A1 "reliability" src/go2w_nav/config/nav2_params_slim.yaml   # 应有 reliable (local+global)
```

**launch 语法验证**：
```bash
# launch 能解析
ros2 launch go2w_nav slam.launch.py --show-args
ros2 launch go2w_nav nav2_slim.launch.py --show-args

# 禁止项检查（应无输出）
echo "=== slam.launch 禁止 TF 桥 ==="
grep "static_transform_publisher" src/go2w_nav/launch/slam.launch.py
echo "=== nav2_slim 禁止 TF 桥/p2l ==="
grep -E "static_transform_publisher|pointcloud_to_laserscan" src/go2w_nav/launch/nav2_slim.launch.py
echo "=== nav2_slim 禁止 /cmd_vel remap ==="
grep "cmd_vel" src/go2w_nav/launch/nav2_slim.launch.py
echo "=== 禁止 amcl/map_server ==="
grep -E "amcl|map_server" src/go2w_nav/launch/slam.launch.py src/go2w_nav/launch/nav2_slim.launch.py
```

**红线文件未改**：
```bash
git diff --name-only HEAD
# 期望输出只含:
#   src/go2w_nav/config/slam_toolbox.yaml
#   src/go2w_nav/launch/slam.launch.py
#   src/go2w_nav/config/nav2_params_slim.yaml
#   src/go2w_nav/launch/nav2_slim.launch.py
#   docs/slam_runbook.md
#   gan-harness/eval-rubric-slam.md
# 出现以下任一 = Critical FAIL:
#   web/nx_web_server.py / nx_room_orchestrator.py / mock_nav2_action.py
#   src/go2w_bridge/go2w_bridge/nx_motion_node.py / nx_sensor_node.py
#   src/go2w_nav/config/nav2_params_3d.yaml / launch/nav2_3d.launch.py
```

### 14.2 mock /scan 建图（半实跑，不依赖狗硬件/NX）

**可选验证**（Generator 评估，若 NX 未恢复可做）：在 PC 上用 `ros2 run dummy_slam dummy_laser` 或自写一个 mock node 发 `/scan`（固定或缓慢旋转的 LaserScan）+ mock `/odom` + odom→base_link TF，启动 slam.launch.py mode=mapping，验证：
- slam_toolbox 收到 scan（QoS 匹配，决策 4 验证）
- `/map` 生长（即使 mock scan 简单）
- `map→odom` TF 发布

**注意**：mock 验证**不能完全替代**真实建图（mock scan 无真实环境特征，scan-matching 可能失败）。Sprint 4 的真实建图是最终验收。mock 仅用于**QoS + launch 结构**验证（Sprint 1-2）。

**Generator 不强制实现 mock**（若工作量超 1 天，跳过，直接等 NX）。但 runbook 要写明 mock 验证步骤（供调试用）。

### 14.3 半实跑验证（Sprint 4，NX 恢复后）

详见 `docs/slam_runbook.md`（§5.2），核心：
- `ros2 topic info /scan -v` 看 QoS 匹配（决策 4 验证）
- `ros2 topic hz /map` 建图在工作
- `ros2 action send_goal /navigate_to_pose` 单点导航
- 故障注入：kill slam_toolbox 看 Nav2 报 TF 断链；kill nx_sensor 看 odom→base_link 断

---

## 15. Evaluation Criteria（见 `gan-harness/eval-rubric-slam.md`，权重已定）

详见独立 `gan-harness/eval-rubric-slam.md`，Critic 直接消费。核心四维：

- **QoS 与数据流（0.30）**：use_sensor_data_qos=false + Nav2 obstacle_layer reliability=reliable + nx_sensor 不改（§12.1 的 5 项，含 2 个 Critical QoS 项）
- **frame/TF 一致性（0.25）**：base/odom/map frame 与 nx_sensor 实测一致 + 无 TF 桥（无多源）（§12.2 的 6 项）
- **slim 适配正确（0.25）**：nav2_params_slim 的 odom_topic=/odom + 删 voxel_layer + Nav2 lifecycle 不含 amcl/map_server + 零 /cmd_vel remap（§12.4 的 6 项）
- **阶段D/A/E 契约不破坏（0.20）**：不改 nav2_3d/nx_sensor/nx_motion/orchestrator + global_frame=map（§12.5 的 4 项）
