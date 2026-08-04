# Evaluation Rubric: Go2W 阶段D — Nav2 自主导航（服务端配置）

> Critic 直接消费。静态审对照 Nav2 Humble 官方规范 + `docs/TECH_DECISIONS.md` 第三节 + `gan-harness/spec-stage-d.md`。
> 范围：纯配置审（yaml + launch + 文档），**不实车**（硬件装完 + 阶段C FAST_LIO 就绪后实跑联调是 Sprint 4 的事）。
> 通过标准：39 项检查全 PASS（Critical 0、High ≤ 2 且有解释），否则打回 Generator 修正。

---

## 0. 审查对象（Generator 交付物）

| 文件 | 类型 | 审查重点 |
|---|---|---|
| `src/go2w_nav/config/nav2_params_3d.yaml` | 修改 | 参数名/类型/值正确性、costmap layers 完整、footprint/速度对齐 TECH_DECISIONS |
| `src/go2w_nav/launch/nav2_3d.launch.py` | 修改 | lifecycle_manager 存在且不含 amcl、无 /cmd_vel remap、TF 桥临时方案有注释 |
| `docs/nav2_3d_runbook.md` | 新建 | 启动顺序 + 验证步骤 + 常见故障覆盖 |
| `gan-harness/eval-rubric-stage-d.md` | 本文件 | — |

**不动文件清单**（git diff 必须为空，否则 Critical）：
- `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py`（阶段E 红线）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py` / `nx_sensor_node.py`（阶段A 红线）
- `src/go2w_nav/config/nav2_params.yaml` / `slam_toolbox.yaml`（2D 路线休眠，不动）
- `src/go2w_nav/launch/nav2.launch.py` / `slam.launch.py`（2D 路线休眠，不动）
- `src/go2w_bringup/launch/search.launch.py`（旧 2D 全系统，不动）

---

## 1. 四维权重

| 维度 | 权重 | 核心问题 |
|---|---|---|
| 参数正确性 | 0.30 | 对照 Nav2 Humble 官方规范 + TECH_DECISIONS 第三节，参数名/类型/值是否正确 |
| TF 与坐标系一致性 | 0.30 | TF 树完整无多源、map→odom 只 FAST_LIO 发、/cmd_vel 零反转（Critical 集中区） |
| costmap 完整性 | 0.20 | local 双保险（obstacle+voxel+inflation）、global 三层、layer 顺序 |
| 阶段A/B/E 契约不破坏 | 0.20 | 不改红线文件、action 名/frame 对齐阶段E 客户端 |

**通过门槛**：
- Critical 项（标 C）：必须全 PASS，任一 FAIL = 整体 FAIL，打回 Generator
- High 项（标 H）：≤ 2 项 FAIL 且有书面解释可放过，否则打回
- Medium 项（标 M）：≤ 4 项 FAIL 可放过（记录待优化）

---

## 2. 检查清单（39 项，逐条核对）

### 2.1 参数正确性（17 项，权重 0.30）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 1 | `bt_navigator.global_frame` | C | = `map` | `grep "global_frame" nav2_params_3d.yaml` 在 bt_navigator 段 = map |
| 2 | `bt_navigator.robot_base_frame` | C | = `base_link` | 同上 |
| 3 | `bt_navigator.odom_topic` | H | = `/Odometry`（与 FAST_LIO 一致）或 launch remap 一致 | grep；若 yaml 是 `/Odometry` 则 launch 不 remap；若 yaml 是 `/odom` 则 launch remap `/Odometry`→`/odom`。**二选一不混用** |
| 4 | `bt_navigator.plugin_lib_names` | C | 存在且含 NavigateToPose/ComputePathToPose/FollowPath 等标准 BT plugin（Humble 必填，约 20 项） | grep `plugin_lib_names`；对比 Nav2 Humble 官方 nav2_params.yaml 的标准列表 |
| 5 | `controller_server.FollowPath.max_vel_x` | H | = `0.6`（TECH_DECISIONS 第三节 Go2W 室内） | grep |
| 6 | `controller_server.FollowPath.max_vel_y` | H | = `0.0`（差速起步） | grep；**禁止非零**（全向模式是未来 Sprint） |
| 7 | `controller_server.FollowPath.max_vel_theta` | H | = `1.0` | grep |
| 8 | `controller_server` 的 `xy_goal_tolerance` / `yaw_goal_tolerance` | H | = `0.20` / `0.15`（FollowPath + general_goal_checker 两处都查） | grep |
| 9 | `local_costmap.footprint` / `global_costmap.footprint` | H | = `[ [0.30,0.20], [0.30,-0.20], [-0.25,-0.20], [-0.25,0.20] ]`（两处都有） | grep `footprint`；**禁止用 robot_radius**（Go2W 非圆形） |
| 10 | `local_costmap.plugins` | C | 含 `obstacle_layer` + `voxel_layer` + `inflation_layer`（双保险，TECH_DECISIONS 第三节） | grep `plugins:` 在 local_costmap 段；**缺 obstacle_layer = Critical**（休眠文件此错） |
| 11 | `local_costmap.obstacle_layer.scan.topic` | H | = `/scan` | grep |
| 12 | `local_costmap.voxel_layer.pointcloud.topic` | H | = `/livox/lidar`（外置 MID360，推荐）或 `/utlidar/cloud_base`（狗自带），注释说明二选一 | grep；topic 名与阶段C LiDAR 驱动输出对齐 |
| 13 | `local_costmap.global_frame` | H | = `odom`（rolling window 在 odom 系） | grep |
| 14 | `global_costmap.plugins` | H | 含 `static_layer` + `obstacle_layer` + `inflation_layer`（不加 voxel_layer，2D 投影足够） | grep |
| 15 | `global_costmap.global_frame` | C | = `map` | grep |
| 16 | `behavior_server` 各 plugin | H | 用 `nav2_behaviors/Spin` / `BackUp` / `DriveOnHeading` / `Wait`（新包名），**禁止 `nav2_recoveries`**（已废弃） | grep `behavior_server` 段的 plugin 行 |
| 17 | `planner_server.GridBased.plugin` | M | = `nav2_navfn_planner/NavfnPlanner`（推荐，简单）或 `nav2_smac_planner/SmacPlanner2D`（注释说明） | grep |

### 2.2 TF 一致性（5 项，权重并入 0.30）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 18 | `map→odom` 发布者唯一 | C | launch 不启 amcl，只 FAST_LIO + TF 桥发 | grep launch 无 `amcl`；grep nav2_params_3d.yaml 无 amcl 段 |
| 19 | `odom→base_link` 发布者唯一 | C | 同上（FAST_LIO 经 TF 桥改名） | 同上 |
| 20 | pointcloud_to_laserscan `target_frame` | H | = `base_link` | grep launch 的 pointcloud_to_laserscan 段 |
| 21 | TF 桥 static_transform 存在 | H | launch 含 `map→camera_init` + `body→base_link` 两个 static_transform_publisher（临时方案，顶部注释写明阶段C 就绪后删） | grep launch 的 static_transform_publisher；检查有注释说明临时性 |
| 22 | slam_toolbox 不启动 | C | launch 无 `slam_toolbox` 节点 | grep launch 无 `'slam_toolbox'` |

### 2.3 坐标系无反转（3 项，权重并入 0.30，**Critical 集中区**）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 23 | launch 无 `/cmd_vel` 反转 remap | **C** | controller_server 节点 `remappings` 不含 `('/cmd_vel', ...)` 或目标是 `/cmd_vel`（no-op）。**禁止** `('/cmd_vel', '/cmd_vel_reversed')` 之类 | grep launch 的 `cmd_vel`；人工审 remap 方向 |
| 24 | yaml 无 `cmd_vel_topic` 改名 | C | controller_server `cmd_vel_topic` 默认 `/cmd_vel`（若显式设，必须 `/cmd_vel`） | grep yaml 的 `cmd_vel_topic` |
| 25 | `/Odometry` → `/odom` 处理一致 | H | yaml 显式 `odom_topic: /Odometry` 或 launch 全局 remap，二选一不混用（决策 1） | grep yaml `odom_topic` + launch `remap`；确认一致 |

### 2.4 costmap layers 完整（5 项，权重 0.20）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 26 | local_costmap 有 obstacle_layer | C | ObstacleLayer 吃 /scan（主） | grep local_costmap 段的 `obstacle_layer` + `plugin: "nav2_costmap_2d::ObstacleLayer"` |
| 27 | local_costmap 有 voxel_layer | H | VoxelLayer 吃 PointCloud2（辅，双保险） | grep local_costmap 段的 `voxel_layer` + `plugin: "nav2_costmap_2d::VoxelLayer"` |
| 28 | local_costmap 有 inflation_layer | H | InflationLayer 存在且在 plugins 数组**最后** | grep local_costmap 段的 `inflation_layer`；检查 plugins 数组顺序 |
| 29 | global_costmap 有 obstacle_layer | H | ObstacleLayer 吃 /scan | grep global_costmap 段 |
| 30 | layer 顺序正确 | M | static→obstacle→voxel→inflation（叠加顺序，inflation 最后） | 人工审 plugins 数组顺序 |

### 2.5 Nav2 lifecycle 正确（5 项，权重并入 0.20）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 31 | lifecycle_manager 存在 | C | launch 含 `lifecycle_manager_navigation` 节点（navigation_launch 不自带） | grep launch 的 `lifecycle_manager` |
| 32 | lifecycle_manager.autostart | C | = `True` | grep `autostart` |
| 33 | lifecycle_manager.node_names 含 bt_navigator | C | node_names 列表含 `bt_navigator`（否则 /navigate_to_pose 不暴露，阶段E 客户端超时） | grep node_names 内容 |
| 34 | lifecycle_manager.node_names 不含 amcl | C | node_names 列表**不含 amcl**（决策 3） | grep；**amcl 出现 = Critical FAIL** |
| 35 | launch 不启 map_server | H | grep launch 无 `nav2_map_server` | grep |

### 2.6 阶段A/B/E 契约不破坏（4 项，权重 0.20）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 36 | 阶段E action 名对齐 | C | Nav2 bt_navigator 默认暴露 `/navigate_to_pose`（launch 不 remap action 名） | grep launch 无 `/navigate_to_pose` remap |
| 37 | 阶段E goal frame 对齐 | C | `bt_navigator.global_frame` = `map`（与 `config/rooms.yaml` 的 `frame_id: map` + `nx_room_orchestrator.py:345` 默认参数一致） | 交叉核对本 rubric #1 + rooms.yaml |
| 38 | 不改 nx_motion_node / nx_room_orchestrator / mock_nav2_action | C | `git diff` 这三个文件为空 | `git diff --name-only` 检查 |
| 39 | 不改 nx_web_server / nx_sensor_node | C | `git diff` 这两个文件为空 | 同上 |

---

## 3. 静态验证命令（Generator 自检 + Critic 复核）

### 3.1 YAML 语法 + 关键参数
```bash
# YAML 语法正确
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/nav2_params_3d.yaml')); print('YAML OK')"

# 关键参数逐项核对
echo "=== bt_navigator ==="
grep -A2 "^bt_navigator:" src/go2w_nav/config/nav2_params_3d.yaml | head -20
echo "=== controller 速度 ==="
grep -E "max_vel_x|max_vel_y|max_vel_theta|xy_goal_tolerance|yaw_goal_tolerance" src/go2w_nav/config/nav2_params_3d.yaml
echo "=== footprint ==="
grep "footprint" src/go2w_nav/config/nav2_params_3d.yaml
echo "=== costmap plugins ==="
grep -A1 "plugins:" src/go2w_nav/config/nav2_params_3d.yaml
echo "=== behavior plugins ==="
grep -E "nav2_behaviors|nav2_recoveries" src/go2w_nav/config/nav2_params_3d.yaml
```

### 3.2 launch 结构核对
```bash
# launch 能解析
ros2 launch go2w_nav nav2_3d.launch.py --show-args 2>&1 | head -5

# 禁止项检查（应无输出）
echo "=== 禁止 amcl ==="
grep -i "amcl" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 禁止 map_server ==="
grep -i "map_server" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 禁止 slam_toolbox ==="
grep -i "slam_toolbox" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 禁止 /cmd_vel remap（只允许 no-op 或无）==="
grep "cmd_vel" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 禁止 nav2_recoveries（应用 nav2_behaviors）==="
grep "nav2_recoveries" src/go2w_nav/launch/nav2_3d.launch.py src/go2w_nav/config/nav2_params_3d.yaml

# 必须项检查
echo "=== 必须有 lifecycle_manager ==="
grep "lifecycle_manager" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 必须有 pointcloud_to_laserscan ==="
grep "pointcloud_to_laserscan" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== 必须有 TF 桥 static_transform ==="
grep "static_transform_publisher" src/go2w_nav/launch/nav2_3d.launch.py
echo "=== node_names 应含 bt_navigator ==="
grep -A10 "node_names" src/go2w_nav/launch/nav2_3d.launch.py
```

### 3.3 红线文件未改
```bash
git diff --name-only HEAD
# 期望输出只含:
#   src/go2w_nav/config/nav2_params_3d.yaml
#   src/go2w_nav/launch/nav2_3d.launch.py
#   docs/nav2_3d_runbook.md
#   gan-harness/eval-rubric-stage-d.md
#   (gan-harness/spec-stage-d.md 若 Planner 也提交)
# 出现以下任一 = Critical FAIL:
#   web/nx_web_server.py / nx_room_orchestrator.py / mock_nav2_action.py
#   src/go2w_bridge/go2w_bridge/nx_motion_node.py / nx_sensor_node.py
```

---

## 4. 评分规则

### 4.1 单项判定
- **PASS**：检查项完全满足通过标准
- **FAIL**：不满足（按严重度计入门槛）
- **SKIP**：不适用（如阶段C 未就绪时 #12 的 topic 名二选一，记录待联调验证）

### 4.2 整体判定
- **通过（APPROVED）**：所有 Critical（C）项 PASS；High（H）项 ≤ 2 FAIL 且有书面解释；Medium（M）项 ≤ 4 FAIL
- **打回（REJECTED）**：任一 Critical FAIL；或 High FAIL > 2；或 Medium FAIL > 4

打回时 Critic 列出所有 FAIL 项 + 修正建议，Generator 修正后重新审。

### 4.3 加分项（可选，不计入门槛）
- runbook 覆盖故障注入场景（kill FAST_LIO / kill p2l 看 Nav2 报错）
- yaml 注释详尽（每个参数说明为何此值）
- launch 支持 `params_file` 参数覆盖（方便调试时换 yaml）

---

## 5. 重点 Critical 项速查（Critic 先看这 13 项，任一 FAIL 直接打回）

| # | 项 | 一句话 |
|---|---|---|
| 1 | bt_navigator.global_frame = map | 与阶段E goal frame 一致 |
| 4 | bt_navigator.plugin_lib_names 存在 | Humble 必填，否则 bt_navigator 启动失败 |
| 10 | local_costmap.plugins 含 obstacle_layer | 休眠文件此错（缺主障碍层） |
| 15 | global_costmap.global_frame = map | — |
| 18 | map→odom 发布者唯一（无 amcl） | 决策 3 核心 |
| 19 | odom→base_link 发布者唯一 | 同上 |
| 22 | slam_toolbox 不启动 | 与 FAST_LIO 互斥 |
| 23 | **launch 无 /cmd_vel 反转 remap** | **狗乱转 Critical** |
| 24 | yaml 无 cmd_vel_topic 改名 | 同上 |
| 26 | local_costmap 有 obstacle_layer | 双保险主层 |
| 31 | lifecycle_manager 存在 | navigation_launch 不自带 |
| 33 | node_names 含 bt_navigator | 否则 /navigate_to_pose 不暴露 |
| 34 | node_names 不含 amcl | 决策 3 |
| 36 | action 名不 remap（默认 /navigate_to_pose） | 阶段E 客户端契约 |
| 37 | global_frame = map | 与 rooms.yaml 一致 |
| 38 | 不改 nx_motion/orchestrator/mock_nav2 | 阶段A/E 红线 |
| 39 | 不改 nx_web/nx_sensor | 阶段A/B 红线 |

---

## 6. Critic 报告模板

```markdown
# 阶段D 静态审报告

## 总判定: [APPROVED / REJECTED]

## Critical 项 (C): [N/13 PASS]
- [PASS/FAIL] #1 bt_navigator.global_frame = map: ...
- [PASS/FAIL] #4 plugin_lib_names 存在: ...
...

## High 项 (H): [N/14 PASS]
- ...

## Medium 项 (M): [N/4 PASS]
- ...

## FAIL 项修正建议
- #XX [项名]: 当前值 X，应改为 Y，理由 Z

## 加分项
- [有/无] runbook 故障注入
- [有/无] yaml 注释详尽

## 结论
[通过 / 打回 + 一句话理由]
```
