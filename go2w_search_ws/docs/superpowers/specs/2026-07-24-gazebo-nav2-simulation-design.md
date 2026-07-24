# Gazebo 仿真验证 Nav2 导航闭环

> 日期: 2026-07-24
> 状态: 设计稿 v2（经对抗式自审修订，待用户审）
> 范围: 原"全栈端到端仿真"收口到"Nav2 导航闭环 + frontier 探索"
> 运行环境: WSL2 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11（开发机 RTX 4090 Laptop 16GB）
> 不在范围: YOLO/人检测/语音/room search 感知层、四足步态、部署到 NX、云端 CI（见"不在范围"节）

## 目标

不连真狗的前提下，在开发机 WSL2 里用 Gazebo 仿真复刻 DOGS 的 Nav2 导航栈（Livox MID360 + FastLIO + Nav2 nav2-3d + map_odom_fuser + motion_machine + web 节点 + frontier 探索），让两条核心链路能跑通并回归：

1. **点选导航闭环**：发目标点（rviz 2D Nav Goal / web 点地图）→ Nav2 规划执行 → Go2 在仿真里自主导航到达。
2. **frontier 探索覆盖**：复用现有 `ExplorationManager` 自动选目标，Nav2 导航，验证 frontier 耗尽终止与物理几何覆盖率。

间接对标实机痛点（costmap 假障碍、TF 时序、cmd_vel 抢占、park_not_allowed、DWB 旋转不前进），为后续故障注入回归集奠基。

## 背景：为什么是这套技术栈

经 5 路并行调研（workflow Run `wf_11561314-b4f`，2026-07-24，205k tokens / 97 工具调用 / 6 agent），三个直觉被证据推翻：

1. **Unitree 官方无 Go2 Gazebo 插件**。扫 `github.com/unitreerobotics` 全部 52 个仓库，零个含 "gazebo"。官方仿真器只有 `unitree_mujoco`（README 明说"只支持低级开发"，无步态）、`unitree_ros`（Gazebo Classic 8 + ROS1，README 明说"不能行走"，但有 `go2_description`/`go2w_description` URDF 可借用）、`unitree_sim_isaaclab`（Isaac Lab，行走需 RL 策略）。
2. **Gazebo Harmonic + ROS2 Humble 是错配**。官方兼容表对 Humble+Harmonic 标 "advanced users only"（Humble 的 supported 配对是 Fortress，Harmonic 的 supported 配对是 Jazzy）；`gz_ros2_control` 源码构建 for Humble+Harmonic 是 WONTFIX（Issue #394）；`ros-humble-ros-gz*` 与 `gz-harmonic` 包 apt 冲突；相机通过 `ros_gz` 桥接是已知失败模式（lidar 通、camera 常死）。
3. **Livox MID360 的 CustomMsg 插件只在 Gazebo Classic 成熟**。`stm32f303ret6/livox_laser_simulation_RO2`（218 ⭐，ROS2 Humble + Gazebo Classic 11）发布真正的 `livox_ros_driver2/msg/CustomMsg` + 非重复玫瑰扫描 + PointCloud2，FastLIO 零改动直接消费。Harmonic 路径（lxrobotics gpu_lidar）只发 PointCloud2 + 旋转栅格（720×40），需 1–2 周自写转换器或移植 C++ 插件。

结论：**Gazebo Classic 11 + ROS2 Humble（官方兼容表标 supported，非 advanced-only）+ kinematic-base 运动模型**。

## 关键决策

1. **Gazebo Classic 11 而非 Harmonic**。Classic 与 Humble 在官方兼容表标 supported；Livox CustomMsg 插件只在此成熟；`gz_ros2_control` 不需要（运动学底盘不依赖关节控制器）。Classic 2025 EOL 是缺点，但"现在能跑通"优先于"用最新版"，且等 Livox 官方移植到 Harmonic（Issue #34/#28 OPEN）或升 ROS2 Jazzy 时再迁。
2. **kinematic-base 运动模型而非四足步态**。实机控制链是 `web/motion_machine → /cmd_vel → [Go2 固件步态控制器] → 腿`，步态在狗固件黑盒里，代码栈停在 `/cmd_vel`。仿真用 `libgazebo_ros_planar_move.so` 挂 `base_link` 直接吃 `/cmd_vel` 平移旋转机器人模型，**正好对应实机控制边界**。四足步态（CHAMP）是实机不存在的平行架构，测它零价值且引入物理噪声（CHAMP Issue #32 摔倒、`gazebo_ros2_control` #54/133 关节站不住）。
3. **仿真话题与实机同名，真实业务逻辑代码不改**。`/livox/lidar`(CustomMsg)、`/Odometry`、`/cmd_vel`、`/scan`、`/map` 等话题名与实机完全一致。FastLIO / Nav2 nav2-3d / web 节点 / `motion_machine` / `ExplorationManager` 的**业务逻辑一行不改**，仅在**接口层 / mock 层 / launch 层**做适配（新增 `motion_sdk_mock.py`、launch 参数切源、个别 web 节点识别 `GO2W_SIM` 跳过实机专属检查）。仿真验证结果可直接外推到实机。
4. **Livox CustomMsg 直供 FastLIO**。用 `stm32f303ret6/livox_laser_simulation_RO2` 发 CustomMsg(+PointCloud2)，IMU 用 stock `libgazebo_ros_imu_sensor.so` 单独挂。FastLIO 消费端不动。Livox 插件 pin 具体 commit（M0 第一任务：跑通该仓库 README demo，把当时 HEAD 的 commit hash 回填到本 spec 与 `go2_sim/CMakeLists.txt` 的 `GIT_TAG` 字段，验证后锁定；该仓库仅 6 commits，搜索空间小）。
5. **IMU：仿真无需 ×9.80665 缩放**。实机 Livox SDK 输出 IMU 加速度单位为 g，项目在 SDK 层 `lddc.cpp:493` 加 ×9.80665 转成 m/s²（见 `fastlio-imu-unit-fixed.md`）。**仿真路径走 Gazebo `libgazebo_ros_imu_sensor.so`，原生输出 SI 单位 m/s²（含重力分量），不经过 `lddc.cpp`**，与 FastLIO 期望一致。因此 `fastlio_sim.yaml` **不引入任何 ×9.80665 factor**（该 factor 只存在于实机 `lddc.cpp:493`，仿真侧无对应代码路径，"移除/反转"在物理上无字段可操作）。`fastlio_sim.yaml` 仅需确认：(a) `imu_topic: /livox/imu`；(b) `gravity_init: 9.80665` 保持不变；(c) `acc_cov`/`gyr_cov` 与实机一致。若 FastLIO 静止原点漂移，第一排查项为 Gazebo IMU 的 `<orientation_reference_frame>` 重力分量配置。
6. **motion_machine 真实 + SDK mock，契约以源码为准**。`motion_machine` 状态机（PARK/ESTOP/FAULT/恢复）原样跑，Unitree SDK2 接口层用 `motion_sdk_mock.py` 替代。**cmd_vel 路径消歧**：`/cmd_vel` 唯一生产者为 `motion_machine`（经 `nx_navigation_arbiter` 路由），`motion_sdk_mock` 是仿真 `planar_move` 插件上游的**唯一消费者**；Nav2 发 goal → `motion_machine` 处理 lifecycle（激活/恢复运动、drive_session 判断）→ mock 转发给 planar_move。rviz 2D Nav Goal 同样经 `motion_machine` lifecycle，**不绕过**。这样能真实对标 `park_not_allowed` / cmd_vel 抢占痛点。**Mock 契约**：以 `nx_motion_node.py` 当前实现为权威（双 `/cmd_vel` + `/cmd_vel_nav` 订阅、`drive_session_owner`、`ScanFreshnessWatchdog`、`DriveExecutionWatchdog`、`SportModeState` 字段 mode/velocity/progress），mock 需对照源码 line-by-line 复刻，并加 unit test pin 该契约（改动源码契约时 mock 测试同步失败）。具体 watchdog 超时阈值 / 心跳 Hz / owner token 规则不在此 spec 编造，以源码读出值为准回填到 `motion_mock.yaml` 注释。
7. **YOLO/感知层不在 MVP**。C13 相机 + YOLO 检测 + 人标注 + 语音全砍出 MVP（domain gap 大、与"仿 nav2 导航"目标偏离）。远期再加。

## 架构

```
┌─────────────────── Gazebo Classic 11 世界 (WSL2 Ubuntu 22.04) ───────────────────┐
│                                                                                  │
│   室内场景 world (空房间 10×10m → 多房间+障碍)                                     │
│        │                                                                         │
│        ├─→ Go2 URDF (借 unitree_ros/go2_description)                              │
│        │        ├─→ libgazebo_ros_planar_move.so (挂 base_link, 唯一吃 /cmd_vel)  │
│        │        ├─→ stm32f303ret6 Livox MID360 插件 ─→ /livox/lidar (CustomMsg)  │
│        │        └─→ libgazebo_ros_imu_sensor.so ───→ /livox/imu (m/s², 含重力)   │
│        │                                                                         │
│        └─→ [不在 MVP] C13 camera（远期 YOLO，见决策#7；MVP world 不含此插件）     │
│                                                                                  │
└──────────────────────────────────────────────────────────────────────────────────┘
                 │ 仿真话题（与实机同名）
                 ▼
┌─────────────── 真实代码栈（业务逻辑不改，接口/mock/launch 层适配） ────────────────┐
│  FastLIO (吃 CustomMsg + /livox/imu，原生 m/s² 无需缩放)                           │
│       └─→ /Odometry ─→ map_odom_fuser(复用实机) ─→ /odom + odom→base_link TF      │
│                                                                                  │
│  pointcloud_to_laserscan ─→ /scan  （与实机 nav2_params.yaml 一致，单一 scan 源） │
│                                                                                  │
│  Nav2 nav2-3d (现有 nav2_params.yaml + sim 覆盖 scan/odom 源)                     │
│       └─→ goal 经 motion_machine lifecycle → /cmd_vel                             │
│                                                                                  │
│  motion_machine (状态机原样跑, SportModeState 由 mock 回报)                       │
│       └─→ motion_sdk_mock.py ─→ /cmd_vel 转发 planar_move（唯一消费者）           │
│                                                                                  │
│  web: nx_web_server / nx_navigation_arbiter / nx_room_orchestrator                │
│       nx_exploration_manager / nx_frontier_planner + frontier v3                  │
└──────────────────────────────────────────────────────────────────────────────────┘
```

**数据流关键点**：
- Livox 插件发 CustomMsg → FastLIO 消费。
- IMU（`libgazebo_ros_imu_sensor.so`）发 `/livox/imu`（m/s²，含重力分量）→ FastLIO **同时消费 CustomMsg + `/livox/imu`**；仿真无需 ×9.80665（见决策#5）。
- FastLIO 发 `/Odometry` → `map_odom_fuser`（复用实机代码，sim-only 配置仅切输入话题源）→ `/odom` + `odom→base_link` TF。
- Nav2 订阅 `/scan`（**单一源**：`pointcloud_to_laserscan` 从 Livox 点云转出，与实机现有 `nav2_params.yaml` 一致；不引入 costmap pointcloud layer）+ TF → 规划 → 发 goal。
- goal 经 `motion_machine` lifecycle → `/cmd_vel` → `motion_sdk_mock` → 仿真 `planar_move` 实际移动模型（见决策#6 路径消歧）。
- `ExplorationManager` 订阅 `/map_frontier` → 选目标 → 经 Nav2 发 goal。

## 验证范围

| 场景 | MVP | 内容 | 验收（定量定义见下"验收阈值"） |
|---|---|---|---|
| **S1 点选导航闭环** | ✅ | rviz 2D Nav Goal / web 点地图 → Nav2 规划 → Go2 导航到达 | 位置误差 < 0.3m；TF 稳定；不撞墙 |
| **S2 frontier 探索** | ✅ | `ExplorationManager` 自动选目标，Nav2 导航 | 物理几何覆盖率 ≥ 70%（95% 为 soft goal）；frontier 耗尽终止；探索步数 > 8；不死锁 |
| ~~S3 搜索房间里的人~~ | ❌ | YOLO + 标注 + marker | 远期 |
| ~~S4 语音指令~~ | ❌ | ASR/NLU + motion | 远期 |
| **故障注入回归**（M3 起步） | 部分 | TF stamp 抖动注入回归（必做）；costmap 假障碍（远期，需建模云台几何） | M3 见里程碑 |

## 验收阈值（客观可测，集中定义）

- **位置误差**：goal reached 后取 3s 窗口最大位移 < 0.05m 时的位置为终态；误差 = 终态 `base_link` TF 位置与 goal 位置的欧氏距离 < 0.3m。
- **TF 稳定**：60s 窗口内最大 transform age < 0.1s，且日志无 `TF_ERROR`、无 "timestamp earlier than transform cache" 报错。
- **不撞墙**：订阅 Gazebo `/contacts`，世界几何接触 normal force > 1N 的事件计数 = 0。
- **静止原点稳定**（M1 IMU 验证）：静止 30s 后 `/Odometry` position 三轴各取 max(|·|) < 0.02m（对标 `fastlio-imu-unit-fixed.md` 实测 `[0.002, 0.005, 0]` 量级并留余量）。
- **物理几何覆盖率**（S2）：world bbox（M2 world 尺寸写入 `indoor_multiroom.world` 并记入此 spec）内 free space cell 中被 `/map` 标记为 explored 的比例，用 ground-truth occupancy grid 对照求 IoU。分母 = world 外墙内 − inflation(0.30m) 后的可通行 free cell 总数。`ExplorationManager.bounded_explored_ratio` 仅作辅助诊断指标，不作硬门禁。
- **frontier 耗尽终止**：`ExplorationManager` 报告 `frontier_count == 0`（或等价的 `reachable_frontiers_exhausted`）持续 10s。
- **不回归定义**：锁定 baseline commit hash（M2 实施时回填），不回归 = 物理几何覆盖率下降 < 2 个百分点 AND frontier 终止判据命中。

## 代码包结构

新建独立 colcon 包 `go2w_search_ws/src/go2w_sim/`，不污染现有代码：

```
go2w_sim/
├─ worlds/
│    ├─ indoor_empty.world         # M1: 空房间 10×10m + 四壁
│    └─ indoor_multiroom.world     # M2: 多房间 + 内部障碍 + 走廊（bbox 尺寸记入 spec）
├─ models/
│    ├─ go2_sim/                   # 借 unitree_ros/go2_description，加 planar_move + 基础碰撞 mesh
│    └─ livox_mid360/              # stm32f303ret6 插件 + ros2 wrapper
├─ launch/
│    ├─ sim_nav_bringup.launch.py      # 一键起: world + Go2 + Livox + IMU + Nav2 + FastLIO + fuser
│    └─ sim_explore_bringup.launch.py  # S2: 上述 + ExplorationManager + frontier
├─ config/
│    ├─ fastlio_sim.yaml           # imu_topic=/livox/imu; gravity_init=9.80665; 不引入 ×9.80665
│    ├─ nav2_sim_params.yaml       # 必须覆盖: scan 源(/scan from pointcloud_to_laserscan)、odom 源(/odom)、inflation_radius、obstacle persistence
│    └─ motion_mock.yaml           # motion_machine SDK mock 配置（watchdog/heartbeat/owner 以源码为准回填）
├─ nodes/
│    └─ motion_sdk_mock.py         # 吃 /cmd_vel，转发 planar_move；mock SportModeState 回报；契约 pin 自 nx_motion_node.py
└─ test/
     ├─ test_sim_nav_goal.py       # S1 pytest: 发 goal → 断言狗到达 → headless
     ├─ test_sim_explore_coverage.py  # S2 pytest: 跑探索 → 断言覆盖率/终止
     ├─ test_motion_mock_contract.py  # pin mock 契约 vs nx_motion_node.py 源码
     └─ conftest.py                # Gazebo headless bringup fixture
```

## 集成原则

1. **话题同名**：所有仿真发的话题与实机一致（见架构图）。
2. **launch 参数切源（权威）**：`sensor_source:=sim|real`（默认 `real`）为权威切源参数；`GO2W_SIM=1` 仅作便捷别名，由 launch 文件读取并映射到 `sensor_source:=sim`（二者等价、不冲突）。现有 `nav2.launch.py` 不改，sim launch 包一层参数覆盖。
3. **复用现有配置**：`nav2_sim_params.yaml` 继承 `nav2_params.yaml`，**必须覆盖**：(a) scan 源（`/scan`，来自 `pointcloud_to_laserscan`）；(b) odom 源（`/odom`，来自 fuser）；(c) costmap `inflation_radius`（从实机 0.30m 适配仿真狗身体碰撞 mesh，默认起点 0.25m，按 go2 URDF 实际边长 + 安全裕度回填）；(d) `obstacle_layer.observation_persistence`（复刻实机 0.3s，见 `costmap-gimbal-echo.md`）。FastLIO config 见决策#5。
4. **motion SDK mock 契约对齐**：见决策#6。`motion_sdk_mock.py` 实现与 `nx_motion_node` 相同的 `/cmd_vel` + `/cmd_vel_nav` 双订阅、`drive_session_owner`、watchdog、`SportModeState` 回报契约（对照 `TEST_PLAN.md` 状态机 v2 + `web-nav-gil-drivesession-fix.md`），转发给仿真 `planar_move`。`test_motion_mock_contract.py` pin 该契约。
5. **GO2W_SIM 环境变量**：`GO2W_SIM=1` 让 web 节点 / arbiter 知道在仿真模式，跳过实机专属检查。**预计落点**：`nx_web_server.py` / `nx_navigation_arbiter.py` 的 SDK 连接健康检查（仿真无真 SDK 连接，需跳过 `connected:false` 判定）；具体落点以源码 grep `sdk_ready` / `connected` 为准，M1c 实施时列出。

## 里程碑

| 里程碑 | 内容 | 工期 | 验收门禁 |
|---|---|---|---|
| **M0 环境就绪** | 假设开发机已有 WSL2 + Ubuntu 22.04 + ROS2 Humble 桌面；若从零装则工期上浮到 2 天。装 Gazebo Classic 11 + colcon；clone stm32f303ret6 + 借 go2_description；跑通 stm32f303ret6 README demo，回填 commit hash 到 spec + `GIT_TAG` | 0.5–1 天（前提满足时） | `gzserver` 起空世界；`ros2 topic list` 看到仿真话题 |
| **M1a teleop 底盘** | world(空房间) + Go2 URDF + planar_move + 基础碰撞 mesh | 2–3 天 | 键盘 teleop 狗能动且不穿墙（`/contacts` force>1N 计数=0） |
| **M1b 感知与建图** | Livox MID360 插件 + FastLIO（IMU 原生 m/s²，无需缩放）+ map_odom_fuser 复用 | 2–3 天 | 静止原点稳定 < 0.02m；TF 稳定（60s age<0.1s 无 TF_ERROR） |
| **M1c 运动契约 + 导航** | motion_sdk_mock 契约对齐（pin test）+ Nav2 nav2-3d 接入 + GO2W_SIM 落点 | 2–3 天 | **S1 pytest 全绿**（位置误差<0.3m + 不撞墙） |
| **M2 多房间 + frontier 探索** | 多房间 world（bbox 记入 spec）；接 `ExplorationManager`；跑探索闭环；锁 baseline commit | 2–3 天 | **S2 pytest 全绿**（覆盖率≥70% + frontier 耗尽 + 步数>8 + 不死锁 + 不回归） |
| **M3 headless 回归** | pytest fixture headless 化；**TF stamp 抖动注入回归（必做）**；`colcon test` 全绿 | 2–3 天 | `colcon test` 全绿；TF 抖动注入下 FastLIO 门禁按设计响应 |

总工期约 2–2.5 周（M0–M3）。**CI 化列为 M4 远期**（见"不在范围"）。

## 已识别风险

1. **IMU 单位**（已消歧，见决策#5）：仿真无需 ×9.80665；若静止原点漂移，排查 Gazebo IMU `<orientation_reference_frame>` 重力分量。验收：M1b 静止原点 < 0.02m。
2. **WSL2 GPU 渲染**（中）：RTX 4090 dGPU 应 OK，但 WSLg 非官方测试；OGRE2 可能崩溃（gz-rendering #662），备选切 OGRE1。headless 模式（`gzserver`）绕过渲染问题，CI/M3 用 headless。
3. **costmap 假障碍不自动复现**（中）：`costmap-utlidar-self-blocked.md` / `costmap-gimbal-echo.md` 要复现必须建模 C13 云台 + 狗身体碰撞几何。MVP 先不建模云台（狗身体基础碰撞 mesh 要有，M1a），costmap 几何建模 + 假障碍回归列为 M3 后远期项。
4. **Livox 插件版本 pin**（低）：`stm32f303ret6` 仅 6 commits，M0 回填 commit hash；`LCAS/livox_laser_simulation_ros2`（12 ⭐）为 fallback，同 profile。
5. **NTP 跳变发散不自动复现**（低）：仿真时钟默认单调。`fastlio-ntp-jump-divergence.md` 根因复现需显式故障注入（M3 的 TF stamp 抖动是起步，完整 NTP 跳变列为远期）。
6. **cmd_vel 契约对齐**（中，已消歧）：`motion_sdk_mock.py` 必须忠实复刻 `nx_motion_node.py` 的双订阅/watchdog/owner 契约（决策#6），由 `test_motion_mock_contract.py` pin。

## 不在范围（YAGNI）

- **M4 云端 CI**：本仓库零 CI（`.github/workflows` 不存在），从零搭云端 runner 跑 Gazebo 物理仿真是周级工程（xvfb + runner 镜像 + flaky 重试）。M3 仅做本地 headless `colcon test` + docker-compose 复现，云端 CI 列为 M4 远期。
- 四足步态（CHAMP / khaledgabr77 Jazzy+Harmonic）—— 远期，仅当"步态研究"成为目标
- YOLO / C13 相机检测 / 人标注 / room search 感知层 —— 远期
- 语音 ASR/NLU —— 远期
- Isaac Sim 高保真渲染 / 合成训练数据 —— 远期
- 部署到 NX（NX 算力不够跑 Gazebo + 全栈渲染）
- 跨楼层导航
- costmap 云台几何建模 + 假障碍回归（M3 后远期）

## 参考仓库

| 仓库 | 用途 | 备注 |
|---|---|---|
| `stm32f303ret6/livox_laser_simulation_RO2` | Livox MID360 CustomMsg 主力 | 218 ⭐，ROS2 Humble + Classic 11，pin commit |
| `unitreerobotics/unitree_ros` | Go2 URDF 来源 | `robots/go2_description` |
| `LihanChen2004/pb_rm_simulation` | Mid360+FAST-LIO+Nav2 端到端参考接线 | RoboMaster Ackermann，wiring 参考价值高 |
| `LCAS/livox_laser_simulation_ros2` | Livox fallback | 12 ⭐，同 profile |
| `khaledgabr77/unitree_go2_ros2` | Go2 URDF/SDF + 远期步态参考 | 146 ⭐，Jazzy+Harmonic |
| `IntelligentRoboticsLabs/go2_robot` | Humble 实机 driver 接口参考 | 323 ⭐，cmd_vel/sport_mode 契约参考 |

## 与现有栅格仿真的关系

现有 `web/test_unknown_room_exploration_sim.py` + `tools/sim_strategy_compare.py` 是**纯 Python 栅格级**仿真（已知 truth grid + BFS 理想规划器 `_KnownFreePlanner`，即理想建图 + 理想规划），验证探索**决策层**。本仿真是**物理 ROS2 级**仿真（真 FastLIO/Nav2/motion，有漂移/规划失败/遮挡死角），验证**栈层**。两者互补：

- 改探索算法逻辑 → 先跑栅格仿真（秒级）快迭代。
- 改栈（Nav2/FastLIO/motion/cmd_vel）→ 跑物理仿真（分钟级）验真实链路。
- **栅格仿真进 CI 回归**（秒级、易跑）；**物理仿真 CI 化列为 M4 远期**（本地 headless `colcon test` 是 M3）。
- 注意：栅格基线的 95% 是"理想规划 + 理想建图"值，物理栈天然低于此（已知栈损耗，记为指标而非失败），故 S2 硬门禁放宽到 70% + frontier 耗尽。

---

*spec 状态：v2 已修 5 处 must-fix（IMU 单位 / motion mock 契约+cmd_vel 路径 / 覆盖率定义+门禁放宽 / 验收阈值量化 / CI 拆 M4）。待用户审 → 通过后 invoke superpowers:writing-plans 拆实现计划。*
