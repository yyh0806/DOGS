# Gazebo Nav2 仿真实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 WSL2 开发机用 Gazebo Classic 11 仿真复刻 DOGS 的 Nav2 导航栈（Livox+FastLIO+Nav2+motion+frontier），跑通"点选导航"和"frontier 探索"两条闭环并支持 headless 回归。

**Architecture:** 仿真发的话题与实机同名，真实业务逻辑代码（FastLIO/Nav2/motion_machine/ExplorationManager）一行不改，仅在接口层加 `motion_sdk_mock.py`、launch 参数切源、个别 web 节点识别 `GO2W_SIM`。运动用 kinematic-base（`libgazebo_ros_planar_move.so`），Livox 用 `stm32f303ret6` CustomMsg 插件，cmd_vel 经 motion_machine lifecycle → mock → planar_move。

**Tech Stack:** WSL2 Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 + colcon + pytest + Gazebo headless（gzserver）。

## Global Constraints

（从 spec `2026-07-24-gazebo-nav2-simulation-design.md` 抄录，每个任务隐含遵守）

- ROS2 Humble + Gazebo Classic 11（官方兼容表 supported；**不用 Harmonic**）
- 仿真话题与实机同名：`/livox/lidar`(CustomMsg)、`/livox/imu`、`/Odometry`、`/odom`、`/cmd_vel`、`/scan`、`/map`、`/map_frontier`
- **仿真不引入 ×9.80665 IMU 缩放**（Gazebo IMU 原生 m/s²；×9.80665 只在实机 `lddc.cpp:493`）
- cmd_vel 唯一生产者 = `motion_machine`（经 arbiter）；`motion_sdk_mock` 是 planar_move 上游唯一消费者；rviz 2D Nav Goal 也经 motion_machine lifecycle
- Livox 插件 pin commit（M0 回填到 `GIT_TAG`）
- 业务逻辑代码不改，仅接口/mock/launch 层适配
- 切源：`sensor_source:=sim|real`（默认 real）权威；`GO2W_SIM=1` 别名
- 验收阈值见 spec "验收阈值"节（位置误差<0.3m、TF age<0.1s、`/contacts` force>1N 计数=0、静止30s max<0.02m、覆盖率≥70%）

## File Structure

新建 colcon 包 `go2w_search_ws/src/go2w_sim/`：

| 文件 | 责任 |
|---|---|
| `worlds/indoor_empty.world` | M1a 空房间 10×10m + 四壁 SDF |
| `worlds/indoor_multiroom.world` | M2 多房间 + 障碍 + 走廊 SDF |
| `models/go2_sim/` | Go2 URDF（借 `unitree_ros/go2_description`），挂 planar_move + 基础碰撞 mesh |
| `models/livox_mid360/` | `stm32f303ret6` Livox 插件配置 + ros2 wrapper |
| `launch/sim_nav_bringup.launch.py` | 一键起 world+Go2+Livox+IMU+Nav2+FastLIO+fuser |
| `launch/sim_explore_bringup.launch.py` | S2：上述 + ExplorationManager |
| `config/fastlio_sim.yaml` | `imu_topic=/livox/imu`; `gravity_init=9.80665`; acc_cov/gyr_cov 沿用实机；**无 ×9.80665** |
| `config/nav2_sim_params.yaml` | 继承 `nav2_params.yaml`，覆盖 scan 源/odom 源/inflation_radius/obstacle persistence |
| `config/motion_mock.yaml` | mock 的 watchdog/heartbeat/owner（M1c 从 `nx_motion_node.py` 源码回填） |
| `nodes/motion_sdk_mock.py` | 吃 `/cmd_vel`+`/cmd_vel_nav`，转发 planar_move；mock `SportModeState` 回报 |
| `urdf/go2_sim.urdf.xacro` | Go2 模型 + planar_move 插件配置 + base_link 碰撞 |
| `test/test_motion_mock_contract.py` | pin mock 契约 vs `nx_motion_node.py` |
| `test/test_sim_nav_goal.py` | S1：发 goal 断言狗到达 |
| `test/test_sim_explore_coverage.py` | S2：跑探索断言覆盖率/终止 |
| `test/conftest.py` | Gazebo headless bringup fixture |

---

## 前置任务 M0：环境就绪（非 TDD，手动门禁）

**说明：** 环境搭建无法 TDD。完成后跑两条验证命令确认门禁。

- [ ] **M0.1** WSL2 装 Ubuntu 22.04（`wsl --install -d Ubuntu-22.04`，Windows 主机 PowerShell 管理员）。假设已有则跳过；从零装则工期上浮到 2 天。

- [ ] **M0.2** WSL2 Ubuntu 内装 ROS2 Humble Desktop + Gazebo Classic 11 + colcon：
```bash
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-pointcloud-to-laserscan python3-colcon-common-extensions python3-pytest
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

- [ ] **M0.3** WSLg 渲染冒烟（RTX 4090 dGPU）：`gzserver --verbose -e ode` 起空世界不崩。若 OGRE2 崩溃（gz-rendering #662），环境变量 `export OGRE_RTSS_WRITE_SHADERS_TO_DISK=1` 或回退渲染。

- [ ] **M0.4** clone + pin Livox 插件：
```bash
cd ~/go2w_ws/src   # 或项目实际 src 路径
git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2
cd livox_laser_simulation_RO2 && git log --oneline | head   # 记录 HEAD hash
```
把 HEAD commit hash 回填到本计划 M1b 步骤与本 spec 决策#4。

- [ ] **M0.5** 借 Go2 URDF：
```bash
git clone https://github.com/unitreerobotics/unitree_ros /tmp/unitree_ros
cp -r /tmp/unitree_ros/robots/go2_description go2w_sim/models/go2_sim/
```

- [ ] **M0.6 门禁验证：**
```bash
source /opt/ros/humble/setup.bash
gzserver --verbose &
ros2 topic list   # 看到 /clock /rosout
```
Expected：`gzserver` 起空世界；`ros2 topic list` 输出含 `/clock`。

---

## Task 1 (M1a)：Go2 kinematic-base + planar_move + 碰撞（teleop 不穿墙）

**Files:**
- Create: `go2w_sim/worlds/indoor_empty.world`（10×10m 空房间，四壁 box 几何）
- Create: `go2w_sim/urdf/go2_sim.urdf.xacro`（go2_description + planar_move 插件 + base_link 碰撞 box）
- Create: `go2w_sim/launch/sim_spawn_only.launch.py`（只起 world + spawn 模型，不含 Nav2，供 M1a 验证）
- Test: `go2w_sim/test/test_teleop_no_collision.py`

**Interfaces:**
- Consumes: 无（首个任务）
- Produces: `go2_sim.urdf.xacro`（被 Task 2/3/4 的 bringup launch 复用）；planar_move 订阅 `/cmd_vel`（geometry_msgs/Twist）。

- [ ] **Step 1: 写失败测试** `test/test_teleop_no_collision.py`
```python
"""M1a: teleop 发 cmd_vel 让狗撞墙，断言 /contacts 无 >1N 接触事件。"""
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState

def test_teleop_into_wall_records_no_hard_contact(sim_spawn_only_session):
    node = sim_spawn_only_session.node
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    contacts = []
    node.create_subscription(ContactsState, '/contacts',
                             lambda m: contacts.extend(m.states), 10)
    twist = Twist(); twist.linear.x = 0.8   # 朝墙全速前进
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
    hard = [s for s in contacts if getattr(s, 'total_wrench', None) is not None]
    # planar_move 在碰撞前应停住; 即使接触, wrench 应近 0 (kinematic base 不穿透)
    assert len(hard) == 0 or all(
        all(abs(w) < 1.0 for w in s.total_wrench.force.__dict__.values())
        for s in hard), f"hard contact >1N detected: {len(hard)} states"
```

- [ ] **Step 2: 跑测试验证失败** — `pytest go2w_sim/test/test_teleop_no_collision.py -v` → FAIL（`/contacts` 无发布者，world 未起）

- [ ] **Step 3: 实现 world + URDF + spawn launch**
  - `worlds/indoor_empty.world`：以 Gazebo Classic `empty_world.sdf` 为骨架，加 4 面 0.2m 厚 box 墙围成 10×10m。
  - `urdf/go2_sim.urdf.xacro`：以 `go2_description/urdf/go2.urdf.xacro` 为模板，**删除腿关节的 ros2_control 标签**（kinematic-base 不用），在 `base_link` 加：
```xml
<gazebo>
  <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
    <commandTopic>/cmd_vel</commandTopic>
    <bodyName>base_link</bodyName>
    <updateRate>50.0</updateRate>
  </plugin>
</gazebo>
<link name="base_link">
  <collision><geometry><box size="0.4 0.2 0.1"/></geometry></collision>
</link>
```
  - `launch/sim_spawn_only.launch.py`：起 `gzserver` 加载 indoor_empty.world + spawn `go2_sim.urdf.xacro`，桥 `/cmd_vel` `/contacts` `/clock`。

- [ ] **Step 4: 跑测试验证通过** — `pytest go2w_sim/test/test_teleop_no_collision.py -v` → PASS。若 hard contact > 0，调 `base_link` 碰撞 box 尺寸或 world 墙厚。

- [ ] **Step 5: Commit** — `git add go2w_sim/worlds/indoor_empty.world go2w_sim/urdf/ go2w_sim/launch/sim_spawn_only.launch.py go2w_sim/test/test_teleop_no_collision.py && git commit -m "feat(sim): Go2 kinematic-base + planar_move + collision, teleop no-wall-penetration"`

---

## Task 2 (M1b)：Livox MID360 + FastLIO + IMU（静止原点稳定 + TF 稳定）

**Files:**
- Create: `go2w_sim/models/livox_mid360/`（`stm32f303ret6` 配置，M0.4 clone 的仓库）
- Modify: `go2w_sim/urdf/go2_sim.urdf.xacro`（挂 Livox 插件 + `libgazebo_ros_imu_sensor.so`）
- Create: `go2w_sim/config/fastlio_sim.yaml`（无 ×9.80665）
- Create: `go2w_sim/launch/sim_fastlio.launch.py`（spawn + Livox + FastLIO + map_odom_fuser 复用实机）
- Test: `go2w_sim/test/test_fastlio_static_origin.py`

**Interfaces:**
- Consumes: Task 1 的 `go2_sim.urdf.xacro` + spawn launch
- Produces: `/livox/lidar`(CustomMsg)、`/livox/imu`、`/Odometry`、`/odom` + `odom→base_link` TF（被 Task 4 Nav2 消费）

- [ ] **Step 1: 写失败测试** `test/test_fastlio_static_origin.py`
```python
"""M1b: 狗静止 30s, FastLIO /Odometry 三轴 max(|·|) < 0.02m; TF age <0.1s 无 TF_ERROR。"""
import time, rclpy
from nav_msgs.msg import Odometry

def test_static_origin_stable_under_30s(sim_fastlio_session):
    node = sim_fastlio_session.node
    pos = []
    node.create_subscription(Odometry, '/Odometry',
        lambda m: pos.append((m.pose.pose.position.x,
                              m.pose.pose.position.y,
                              m.pose.pose.position.z)), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(pos) > 100, "Odometry 未持续发布"
    maxx = max(abs(p[0]) for p in pos)
    maxy = max(abs(p[1]) for p in pos)
    maxz = max(abs(p[2]) for p in pos)
    assert maxx < 0.02 and maxy < 0.02 and maxz < 0.02, \
        f"static drift {maxx},{maxy},{maxz} > 0.02m — 查 Gazebo IMU 重力/单位"
    assert sim_fastlio_session.max_tf_age_60s() < 0.1
    assert "TF_ERROR" not in sim_fastlio_session.captured_log
```

- [ ] **Step 2: 跑测试验证失败** → FAIL（`/Odometry` 无发布者，FastLIO 未起）

- [ ] **Step 3: 实现**
  - URDF 加 Livox + IMU 插件：
```xml
<gazebo reference="livox_frame">
  <plugin name="livox_plugin" filename="liblivox_laser_simulation_RO2.so">
    <sensor>MID360</sensor>
    <topic>/livox/lidar</topic>
    <samples>10000</samples>
    <noise>0.002</noise>
  </plugin>
</gazebo>
<gazebo reference="imu_link">
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <topicName>/livox/imu</topicName>
    <bodyName>imu_link</bodyName>
    <updateRate>200</updateRate>
    <!-- Gazebo 原生发 m/s² 含重力, FastLIO 期望一致, 无需 ×9.80665 -->
  </plugin>
</gazebo>
```
  - `config/fastlio_sim.yaml`：复制实机 `mid360.yaml`，改 `lidar_topic: /livox/lidar`、`imu_topic: /livox/imu`，`gravity_init: 9.80665` 保持，**确认无任何 ×9.80665 factor 字段**。
  - `launch/sim_fastlio.launch.py`：spawn + FastLIO 节点（实机 fast_lio 包，配 fastlio_sim.yaml）+ `map_odom_fuser`（实机节点，仅切输入话题源 `/Odometry`）。
  - Livox `GIT_TAG` 设为 M0.4 记录的 commit hash。

- [ ] **Step 4: 跑测试验证通过** → PASS。若静止漂移 >0.02m，排查顺序：① Gazebo IMU `<orientation_reference_frame>` 重力分量；② acc_cov/gyr_cov 与实机一致；③ Livox 噪声参数。

- [ ] **Step 5: Commit** — `git add go2w_sim/models/livox_mid360 go2w_sim/config/fastlio_sim.yaml go2w_sim/launch/sim_fastlio.launch.py go2w_sim/urdf/ go2w_sim/test/test_fastlio_static_origin.py && git commit -m "feat(sim): Livox MID360 CustomMsg + FastLIO + IMU (no x9.80665), static origin <0.02m"`

---

## Task 3 (M1c-1)：motion_sdk_mock 契约（TDD pin 契约）

**Files:**
- Create: `go2w_sim/nodes/motion_sdk_mock.py`
- Create: `go2w_sim/config/motion_mock.yaml`
- Test: `go2w_sim/test/test_motion_mock_contract.py`

**Interfaces:**
- Consumes: `nx_motion_node.py`（实机源码，**只读引用**，提炼契约）
- Produces: `motion_sdk_mock.py` 暴露的双订阅 `/cmd_vel`+`/cmd_vel_nav` 接口、`SportModeState` 回报 topic、watchdog/owner 行为。被 Task 4 Nav2 接入消费。

- [ ] **Step 1: 从 `nx_motion_node.py` 提炼契约（只读 grep）**
```bash
grep -nE "cmd_vel|watchdog|owner|SportModeState|heartbeat|drive_session" \
  src/go2w_bridge/go2w_bridge/nx_motion_node.py
```
把读出的值（watchdog 超时 ms、心跳 Hz、owner token 字段名、SportModeState 各字段）回填到 `config/motion_mock.yaml` 注释。**不编造数值，以源码为准。**

- [ ] **Step 2: 写失败测试** `test/test_motion_mock_contract.py`
```python
"""M1c: mock 契约 pin nx_motion_node.py。源码契约改时本测试同步失败。"""
import time, pytest

def test_mock_subscribes_both_cmd_vel_topics(mock_node):
    topics = [t for t, _ in mock_node.get_topic_names_and_types()]
    assert '/cmd_vel' in topics and '/cmd_vel_nav' in topics

def test_mock_publishes_sport_mode_state_fields(mock_node, sport_state_msgs):
    assert len(sport_state_msgs) > 0
    m = sport_state_msgs[0]
    assert hasattr(m, 'mode') and hasattr(m, 'velocity') and hasattr(m, 'progress')

def test_mock_watchdog_timeout_matches_source(mock_node, motion_mock_yaml):
    source_timeout = motion_mock_yaml['watchdog_timeout_ms']  # 从 nx_motion_node.py 回填
    assert mock_node.watchdog_timeout_ms == source_timeout

def test_mock_forwards_cmd_vel_to_planar_move(mock_node, planar_move_received):
    mock_node.inject_cmd_vel(vx=0.5)
    time.sleep(0.1)
    assert planar_move_received[0].linear.x == pytest.approx(0.5)
```

- [ ] **Step 3: 跑测试验证失败** → FAIL（mock_node 不存在）

- [ ] **Step 4: 实现 `motion_sdk_mock.py`**：rclpy node，订阅 `/cmd_vel`+`/cmd_vel_nav`（owner 仲裁逻辑照 `nx_motion_node.py`），把 Twist 转发给 planar_move（按源码 owner 规则定转发路径），定时发布 mock `SportModeState`（mode 按运动状态推算：静止=PARKED/STAND、收到 cmd_vel=WALK、estop 信号=EMERGENCY）。watchdog 超时按 Step 1 回填值。

- [ ] **Step 5: 跑测试验证通过** → PASS

- [ ] **Step 6: Commit** — `git add go2w_sim/nodes/motion_sdk_mock.py go2w_sim/config/motion_mock.yaml go2w_sim/test/test_motion_mock_contract.py && git commit -m "feat(sim): motion_sdk_mock with source-pinned contract (watchdog/owner/SportModeState)"`

---

## Task 4 (M1c-2)：Nav2 接入 + S1 点选导航闭环

**Files:**
- Create: `go2w_sim/config/nav2_sim_params.yaml`（继承 `nav2_params.yaml`，覆盖 4 项）
- Create: `go2w_sim/launch/sim_nav_bringup.launch.py`（Task1+2+3 + Nav2 + pointcloud_to_laserscan）
- Create: `go2w_sim/test/conftest.py`（Gazebo headless fixture）
- Test: `go2w_sim/test/test_sim_nav_goal.py`

**Interfaces:**
- Consumes: Task 1 URDF/world、Task 2 `/Odometry`+TF+`/livox/lidar`、Task 3 mock
- Produces: S1 闭环（发 goal → Nav2 → motion_machine → mock → planar_move → 狗到达）。`sim_nav_bringup.launch.py` 被 Task 5 复用。

- [ ] **Step 1: 写失败测试** `test/test_sim_nav_goal.py`
```python
"""S1: 发 Nav2 goal (3,3), 断言狗到达 (欧氏距离<0.3m), 不撞墙, TF 稳定。"""
import math

def test_reach_nav_goal_without_collision(sim_nav_session):
    sim_nav_session.send_nav_goal(x=3.0, y=3.0, yaw=0.0)
    assert sim_nav_session.wait_goal_reached(timeout=60.0), "goal 未在 60s 内到达"
    # 终态: goal reached 后 3s 窗口最大位移 <0.05m 时的位置
    final = sim_nav_session.terminal_pose_after_settle(
        settle_window=3.0, max_drift=0.05)
    dist = math.hypot(final.x - 3.0, final.y - 3.0)
    assert dist < 0.3, f"terminal error {dist}m >= 0.3m"
    assert sim_nav_session.count_contacts_above_1N() == 0, "撞墙"
    assert sim_nav_session.max_tf_age_60s() < 0.1
    assert "TF_ERROR" not in sim_nav_session.captured_log
```

- [ ] **Step 2: 跑测试验证失败** → FAIL（Nav2 未起，goal 无响应）

- [ ] **Step 3: 实现**
  - `config/nav2_sim_params.yaml`：复制实机 `src/go2w_nav/config/nav2_params.yaml`，覆盖：
    ```yaml
    scan_topic: /scan              # 来自 pointcloud_to_laserscan
    odom_topic: /odom              # 来自 fuser
    local_costmap.inflation_layer.inflation_radius: 0.25  # 适配 go2 base_link box, 起点 0.25 调
    obstacle_layer.observation_persistence: 0.3           # 复刻实机, 见 costmap-gimbal-echo.md
    ```
  - `pointcloud_to_laserscan` 节点：从 `/livox/lidar` 点云转 `/scan`（单一 scan 源，不引入 costmap pointcloud layer）。
  - `launch/sim_nav_bringup.launch.py`：起 gzserver+world+spawn+Livox+FastLIO+fuser+mock+pointcloud_to_laserscan+Nav2（nav2-3d launch，配 nav2_sim_params.yaml）。参数 `sensor_source:=sim`。
  - `test/conftest.py`：`sim_nav_session` fixture 用 `subprocess` 起 `gzserver --headless` + launch，pytest fixture scope=session。
  - GO2W_SIM=1 落点：`grep -n "sdk_ready\|connected" src/go2w_bridge/.../nx_web_server.py src/go2w_bridge/.../nx_navigation_arbiter.py`，加 `if os.environ.get('GO2W_SIM'): continue`（具体行号实施时按 grep 结果定）。

- [ ] **Step 4: 跑测试验证通过** → PASS（狗到达 (3,3)，误差<0.3m，无 hard contact）

- [ ] **Step 5: Commit** — `git add go2w_sim/config/nav2_sim_params.yaml go2w_sim/launch/sim_nav_bringup.launch.py go2w_sim/test/conftest.py go2w_sim/test/test_sim_nav_goal.py && git commit -m "feat(sim): Nav2 nav2-3d + S1 nav-goal loop (position err <0.3m, no collision)"`

---

## Task 5 (M2)：多房间 world + frontier 探索（S2）

**Files:**
- Create: `go2w_sim/worlds/indoor_multiroom.world`（多房间 + 障碍 + 走廊，bbox 记入注释）
- Create: `go2w_sim/launch/sim_explore_bringup.launch.py`（sim_nav_bringup + ExplorationManager + frontier，world 切 multiroom）
- Test: `go2w_sim/test/test_sim_explore_coverage.py`

**Interfaces:**
- Consumes: Task 4 sim_nav_bringup + 实机 `nx_exploration_manager.ExplorationManager` + `nx_frontier_planner`
- Produces: S2 闭环（ExplorationManager 自动选目标 → Nav2 导航 → 覆盖）。

- [ ] **Step 1: 写失败测试** `test/test_sim_explore_coverage.py`
```python
"""S2: frontier 探索, 物理几何覆盖率>=70%, frontier 耗尽终止, 步数>8, 不死锁。"""
def test_explore_covers_and_terminates(sim_explore_session, ground_truth_grid):
    waypoints = sim_explore_session.run_exploration(timeout=600.0)
    final_map = sim_explore_session.final_map()
    coverage = ground_truth_grid.geometric_coverage_iou(final_map, inflation=0.30)
    assert coverage >= 0.70, f"coverage {coverage} < 0.70"
    assert sim_explore_session.terminal_reason == "reachable_frontiers_exhausted"
    assert sim_explore_session.frontier_zero_persisted_seconds() >= 10.0
    assert len(waypoints) > 8, f"only {len(waypoints)} waypoints, expected >8"
    assert not sim_explore_session.deadlocked()
```
（`ground_truth_grid` fixture 从 `indoor_multiroom.world` 的墙几何直接解析出 truth occupancy，无需人工标注。）

- [ ] **Step 2: 跑测试验证失败** → FAIL（ExplorationManager 未起）

- [ ] **Step 3: 实现**
  - `worlds/indoor_multiroom.world`：在 empty 基础上加内墙分隔成 3 房间 + 走廊 + 2 个障碍 box。world 文件顶部注释写明 bbox（如 `# bbox: 12m x 10m, 3 rooms`）供 ground_truth_grid 解析。
  - `launch/sim_explore_bringup.launch.py`：`sim_nav_bringup` 的 `world:=indoor_multiroom.world` + 起 `nx_room_orchestrator`/`nx_exploration_manager`（实机节点，零改）。
  - 锁 baseline commit hash：`git rev-parse HEAD` 写入本测试 `BASELINE_COMMIT` 常量。

- [ ] **Step 4: 跑测试验证通过** → PASS。若 coverage 在 70-95% 之间属正常栈损耗（记为指标）。若 <70%，查 FastLIO 漂移/Livox 遮挡死角/Nav2 规划失败回退。

- [ ] **Step 5: Commit** — `git add go2w_sim/worlds/indoor_multiroom.world go2w_sim/launch/sim_explore_bringup.launch.py go2w_sim/test/test_sim_explore_coverage.py && git commit -m "feat(sim): multiroom world + frontier explore S2 (coverage>=70%, frontier exhaustion)"`

---

## Task 6 (M3)：headless fixture + TF stamp 抖动注入回归

**Files:**
- Modify: `go2w_sim/test/conftest.py`（headless gzserver fixture 固化，所有 session 用 headless）
- Create: `go2w_sim/nodes/stamp_jitter_injector.py`（订阅+重发 `/livox/lidar`，按可配抖动扰动 stamp）
- Create: `go2w_sim/test/test_tf_stamp_jitter_regression.py`
- Create: `go2w_sim/docker/docker-compose.yml`（本地复现 headless gzserver，M4 云端 CI 的前置）

**Interfaces:**
- Consumes: Task 4/5 的 sim_session fixture
- Produces: `colcon test` 全绿的可重复 headless 回归 + TF 抖动注入下 FastLIO 门禁按设计响应的证据。

- [ ] **Step 1: 写失败测试** `test/test_tf_stamp_jitter_regression.py`
```python
"""M3: 注入 stamp 抖动, FastLIO latency_gate 按设计响应(拒绝非单调或抖动超阈)。"""
def test_fastlio_gate_responds_to_stamp_jitter(sim_nav_with_jitter_session):
    sim_nav_with_jitter_session.set_stamp_jitter(stddev_ms=50.0)
    sim_nav_with_jitter_session.run(nav_goal=(3, 3), timeout=60.0)
    log = sim_nav_with_jitter_session.captured_log.lower()
    # FastLIO 设计: 非单调 stamp 触发 gate 拒绝 / 日志记录, 不应发散 /Odometry
    assert ("stamp" in log and "non" in log) or "latency_gate" in log, \
        "FastLIO 未对 stamp 抖动响应"
    assert sim_nav_with_jitter_session.odometry_not_diverged(max_drift=1.0)
```

- [ ] **Step 2: 跑测试验证失败** → FAIL（jitter injector 未实现）

- [ ] **Step 3: 实现**
  - `conftest.py`：所有 `sim_*_session` fixture 固化 `gzserver --headless -s --lock`（无 client），CI 可跑。
  - `stamp_jitter_injector.py`：rclpy node 订阅 `/livox/lidar`，重发到 `/livox/lidar_jittered`，stamp 加正态抖动（`stddev_ms` 参数）。bringup launch 有一参数 `stamp_jitter_stddev_ms`，>0 时把 FastLIO 的 `lidar_topic` 切到 `/livox/lidar_jittered`。
  - `docker/docker-compose.yml`：基于 `osrf/ros:humble-desktop-full`，装 gazebo classic + 本包，跑 `colcon test`。M4 云端 CI 的本地前置。

- [ ] **Step 4: 跑全套回归** — `colcon test --packages-select go2w_sim` → 全绿（test_teleop_no_collision + test_fastlio_static_origin + test_motion_mock_contract + test_sim_nav_goal + test_sim_explore_coverage + test_tf_stamp_jitter_regression）

- [ ] **Step 5: Commit** — `git add go2w_sim/test/conftest.py go2w_sim/nodes/stamp_jitter_injector.py go2w_sim/test/test_tf_stamp_jitter_regression.py go2w_sim/docker/ && git commit -m "feat(sim): headless CI fixture + TF stamp jitter injection regression"`

---

## Self-Review

**1. Spec coverage：**
- 决策#1 Classic 11 → Global Constraints + M0.2 ✓
- 决策#2 kinematic-base → Task 1 ✓
- 决策#3 业务逻辑不改 → 全局约束 + Task 3/4 接口层 ✓
- 决策#4 Livox CustomMsg pin commit → M0.4 + Task 2 GIT_TAG ✓
- 决策#5 IMU 无 ×9.80665 → Task 2 config + 测试断言漂移 ✓
- 决策#6 motion_machine + mock + cmd_vel 路径 → Task 3 + Task 4 ✓
- 决策#7 YOLO 不在 MVP → 无相关 task（正确，YAGNI）✓
- S1 点选导航 → Task 4 ✓
- S2 frontier 探索 → Task 5 ✓
- 验收阈值（位置/TF/contacts/静止原点/覆盖率/frontier 耗尽/不回归）→ Task 1/2/4/5 测试断言 ✓
- M3 headless + TF stamp 抖动 → Task 6 ✓
- M4 云端 CI → docker-compose 为前置，云端 CI 明确不在本计划（YAGNI）✓
- 风险#3 costmap 云台几何 → 无 task（正确，远期）✓

**2. Placeholder scan：**
- 无 TBD/TODO。"以 X 为模板适配"指向具体参考文件（go2_description、empty_world.sdf、nav2_params.yaml）+ 具体改动字段，可执行。
- `nx_motion_node.py` 契约具体数值（watchdog/owner）以 Task 3 Step1 grep 源码回填 —— 诚实的"以源码为准"，非编造，`test_motion_mock_contract.py` pin 该值。
- GO2W_SIM=1 落点行号"实施时按 grep 定" —— 给了具体 grep 命令 + 落点文件名 + 改法（`if os.environ.get('GO2W_SIM'): continue`），可执行。

**3. Type consistency：**
- 话题名（`/cmd_vel`、`/cmd_vel_nav`、`/livox/lidar`、`/livox/imu`、`/Odometry`、`/odom`、`/scan`、`/map`、`/contacts`）全 plan 一致 ✓
- `sim_nav_session`/`sim_explore_session` fixture 方法名（`send_nav_goal`/`wait_goal_reached`/`terminal_pose_after_settle`/`count_contacts_above_1N`/`max_tf_age_60s`/`captured_log`）跨 Task 4/5/6 一致 ✓
- `SportModeState` 字段（mode/velocity/progress）Task 3 定义 + 测试消费，一致 ✓

**发现的 gap：** Task 4 的 `conftest.py`（`sim_nav_session`）与 Task 6 修改 `conftest.py` 加 headless 有先后依赖 —— Task 4 先建 conftest，Task 6 固化 headless。已在 Task 6 Files 标注 "Modify conftest.py"。无需改。

计划就绪。

---

## Execution Handoff

计划完成，存于 `go2w_search_ws/docs/superpowers/plans/2026-07-24-gazebo-nav2-simulation.md`。

**两种执行方式：**

1. **Subagent-Driven（推荐）** — 每个 task 派一个全新 subagent 执行，task 间两阶段 review，快速迭代。
2. **Inline Execution** — 本会话用 executing-plans skill 批量执行，checkpoint review。

**你选哪种？**

（注：M0 环境搭建需你在 WSL2 里手动跑 apt 安装 + clone 仓库 —— subagent/inline 都替代不了你在目标机器上的手动环境配置。M0 完成后 Task 1 起才适合自动执行。）
