# Evaluation Rubric: Go2W 阶段F — slam_toolbox 2D 建图（降级路径）

> Critic 直接消费。静态审对照 slam_toolbox Humble 官方规范 + `src/go2w_bridge/go2w_bridge/nx_sensor_node.py` 实测 frame/QoS + `gan-harness/spec-slam.md` + `docs/TECH_DECISIONS.md`。
> 范围：纯配置审（yaml + launch + 文档），**不实车**（NX 恢复 + 阶段0 就绪后实跑联调是 Sprint 4 的事）。
> 通过标准：25 项检查全 PASS（Critical 0、High ≤ 2 且有解释），否则打回 Generator 修正。

---

## 0. 审查对象（Generator 交付物）

| 文件 | 类型 | 审查重点 |
|---|---|---|
| `src/go2w_nav/config/slam_toolbox.yaml` | 修改 | use_sensor_data_qos=false（头号风险）、max_laser_range=10.0、base/odom/map frame 与 nx_sensor 实测一致 |
| `src/go2w_nav/launch/slam.launch.py` | 重写 | mode arg 切 executable（async/localization）、无 TF 桥 static_transform、localization 注入 map_file_name |
| `src/go2w_nav/config/nav2_params_slim.yaml` | 新建 | odom_topic=/odom（非 /Odometry）、删 voxel_layer、obstacle_layer scan 段 reliability=reliable |
| `src/go2w_nav/launch/nav2_slim.launch.py` | 新建 | 无 TF 桥、无 pointcloud_to_laserscan、无 /cmd_vel remap、lifecycle 不含 amcl/map_server |
| `docs/slam_runbook.md` | 新建 | 建图流程 + 定位流程 + Nav2 联调 + 常见故障（含 QoS 不匹配症状）+ 切回 FAST_LIO |
| `gan-harness/eval-rubric-slam.md` | 本文件 | — |

**不动文件清单**（git diff 必须为空，否则 Critical）：
- `web/nx_web_server.py` / `web/nx_room_orchestrator.py` / `web/mock_nav2_action.py`（阶段E 红线）
- `src/go2w_bridge/go2w_bridge/nx_motion_node.py` / `nx_sensor_node.py`（阶段A 红线，/scan 来源 = 红线中的红线）
- `src/go2w_nav/config/nav2_params_3d.yaml` / `launch/nav2_3d.launch.py`（阶段D 红线，commit 9b0c397 已 gan 收敛）
- `src/go2w_nav/config/nav2_params.yaml` / `launch/nav2.launch.py`（2D 路线休眠，不动）
- `src/go2w_bringup/launch/search.launch.py`（旧 2D 全系统，不动）

---

## 1. 四维权重

| 维度 | 权重 | 核心问题 |
|---|---|---|
| QoS 与数据流 | 0.30 | use_sensor_data_qos=false（头号风险）+ Nav2 obstacle_layer reliability=reliable（第二个 QoS 坑）+ nx_sensor 不改 |
| frame/TF 一致性 | 0.25 | base/odom/map frame 与 nx_sensor 实测一致（base_link/odom/map）+ 无 TF 桥（无多源 map→odom） |
| slim 适配正确 | 0.25 | nav2_params_slim 的 odom_topic=/odom + 删 voxel_layer + Nav2 lifecycle 不含 amcl/map_server + 零 /cmd_vel remap |
| 阶段D/A/E 契约不破坏 | 0.20 | 不改 nav2_3d/nx_sensor/nx_motion/orchestrator + global_frame=map + 阶段E action 名不 remap |

**通过门槛**：
- Critical 项（标 C）：必须全 PASS，任一 FAIL = 整体 FAIL，打回 Generator
- High 项（标 H）：≤ 2 项 FAIL 且有书面解释可放过，否则打回
- Medium 项（标 M）：≤ 4 项 FAIL 可放过（记录待优化）

---

## 2. 检查清单（25 项，逐条核对）

### 2.1 QoS 与数据流（5 项，权重 0.30 —— 头号风险区）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 1 | `slam_toolbox.yaml` 含 `use_sensor_data_qos: false` | **C** | grep 有此参数，且值=false | `grep "use_sensor_data_qos" src/go2w_nav/config/slam_toolbox.yaml`；**缺失或=true = Critical FAIL**（QoS 不匹配静默失败，slam_toolbox 收不到 nx_sensor 的 RELIABLE /scan） |
| 2 | `slam_toolbox.yaml` `scan_topic: /scan` | C | = `/scan`（nx_sensor 发的 topic 名） | grep；**禁止** /base_scan /laser_scan 等别名 |
| 3 | `nav2_params_slim.yaml` obstacle_layer scan 段含 `reliability: reliable` | H | local_costmap + global_costmap 的 obstacle_layer scan 段都有 `reliability: reliable`（匹配 nx_sensor RELIABLE 发布） | `grep -A2 "reliability" src/go2w_nav/config/nav2_params_slim.yaml`（应 2 处）；若 Nav2 Humble obstacle_layer 不支持 reliability 参数，Generator 需书面说明替代方案（如改 nx_sensor 发 BEST_EFFORT，但这违反阶段A 红线，需 Planner 批准） |
| 4 | nx_sensor_node.py 未改 | **C** | `git diff src/go2w_bridge/go2w_bridge/nx_sensor_node.py` 为空 | `git diff --name-only HEAD \| grep nx_sensor_node` 应无输出；**改动 = Critical FAIL**（阶段A 红线 + /scan 来源） |
| 5 | `slam_toolbox.yaml` `max_laser_range: 10.0` | M | = 10.0（对齐 nx_sensor /scan range_max=10.0，nx_sensor_node.py:235） | grep；8.0 不致错（MID360 遗留值）但浪费数据，应改 10.0 |

### 2.2 frame/TF 一致性（6 项，权重 0.25）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 6 | `slam_toolbox.yaml` `base_frame: base_link` | C | = base_link（nx_sensor /scan frame_id，nx_sensor_node.py:228） | grep；**禁止** laser_frame / base_scan / laser_link（nx_sensor /scan 直接在 base_link 系） |
| 7 | `slam_toolbox.yaml` `odom_frame: odom` | C | = odom（nx_sensor /odom frame，nx_sensor_node.py:201/213） | grep |
| 8 | `slam_toolbox.yaml` `map_frame: map` | C | = map（Nav2 global_frame + 阶段E goal frame） | grep |
| 9 | slam.launch.py 无 `static_transform_publisher` | C | grep 无（slam_toolbox 发 map→odom，nx_sensor 发 odom→base_link，无 TF 桥） | `grep "static_transform_publisher" src/go2w_nav/launch/slam.launch.py` 应无输出；**有 = Critical FAIL**（多源 TF，与 slam_toolbox 抢 map→odom） |
| 10 | nav2_slim.launch.py 无 `static_transform_publisher` | C | grep 无（slim 路线无 TF 桥，与阶段D nav2_3d.launch.py 的 TF 桥不同） | grep；**有 = Critical FAIL**（slim 路线不需要，加了与 slam_toolbox/nx_sensor 抢 TF） |
| 11 | nav2_slim.launch.py 无 `pointcloud_to_laserscan` | H | grep 无（nx_sensor 已发 /scan，不需要 p2l） | grep；**有 = High FAIL**（p2l 订 /livox/lidar 无数据，无害但多余；且若误订 /scan 会循环） |

### 2.3 mode 切换正确（4 项，权重并入 0.25）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 12 | slam.launch.py 含 `mode` arg（default `mapping`） | H | DeclareLaunchArgument mode，default=mapping | grep `DeclareLaunchArgument.*mode` |
| 13 | mode=mapping 起 `async_slam_toolbox_node` | H | IfCondition(mode==mapping) + executable=async_slam_toolbox_node | grep executable；检查 IfCondition 配对 |
| 14 | mode=localization 起 `localization_slam_toolbox_node` | H | IfCondition(mode==localization) + executable=localization_slam_toolbox_node | grep；**禁止** mapping/localization 用同一 executable |
| 15 | localization 注入 `map_file_name` + `map_start_pose` | H | localization Node 的 parameters 含 map_file_name（来自 map_file arg）+ map_start_pose=[0.0,0.0,0.0] | grep parameters；map_start_pose 非零 = FAIL（建图起始点 = map 原点 identity） |

### 2.4 Nav2 slim 适配（6 项，权重 0.25）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 16 | `nav2_params_slim.yaml` `bt_navigator.odom_topic: /odom` | **C** | = /odom（小写，nx_sensor 发）；**禁止** /Odometry（大写，那是阶段D FAST_LIO 路线） | `grep "odom_topic" src/go2w_nav/config/nav2_params_slim.yaml`；**=/Odometry = Critical FAIL**（slim 路线无 FAST_LIO，Nav2 收不到里程计） |
| 17 | `nav2_params_slim.yaml` local_costmap.plugins 不含 voxel_layer | M | plugins 数组 = ["obstacle_layer", "inflation_layer"]（删 voxel_layer，slim 路线无 3D 点云源） | grep local_costmap 段 plugins；**含 voxel_layer = Medium FAIL**（voxel_layer 订 /livox/lidar 无数据，无害但浪费） |
| 18 | nav2_slim.launch.py lifecycle node_names 不含 amcl | C | grep 无 amcl（slam_toolbox localization 替代） | grep；**有 amcl = Critical FAIL**（与 slam_toolbox 抢 map→odom） |
| 19 | nav2_slim.launch.py lifecycle node_names 不含 map_server | H | grep 无 map_server（slam_toolbox 自己发 /map） | grep；**有 = High FAIL**（map_server 会发 /map 与 slam_toolbox 冲突） |
| 20 | nav2_slim.launch.py 无 `/cmd_vel` remap | **C** | grep 无 cmd_vel remap（零反转，继承阶段D 决策 5） | `grep "cmd_vel" src/go2w_nav/launch/nav2_slim.launch.py` 应无 remap 行；**反转 remap = Critical FAIL（狗乱转）** |
| 21 | nav2_slim.launch.py params_file 默认 = nav2_params_slim.yaml | H | DeclareLaunchArgument params_file default 指向 nav2_params_slim.yaml（非 nav2_params_3d.yaml） | grep；**指向 3d 版 = High FAIL**（用了 FAST_LIO 假设的参数） |

### 2.5 阶段D/A/E 契约不破坏（4 项，权重 0.20）

| # | 检查项 | 严重度 | 通过标准 | 验证方法 |
|---|---|---|---|---|
| 22 | 不改 nav2_params_3d.yaml / nav2_3d.launch.py | **C** | `git diff` 这两个文件为空 | `git diff --name-only HEAD \| grep -E "nav2_params_3d\|nav2_3d.launch"` 应无输出；**改动 = Critical FAIL**（阶段D commit 9b0c397 已 gan 收敛） |
| 23 | 不改 nx_motion_node.py / nx_room_orchestrator.py / mock_nav2_action.py | C | `git diff` 为空 | git diff --name-only；阶段A/E 红线 |
| 24 | 不改 nx_web_server.py / nx_sensor_node.py | C | `git diff` 为空（阶段A 红线） | 同上 |
| 25 | `nav2_params_slim.yaml` `bt_navigator.global_frame: map` | C | = map（与 rooms.yaml frame_id + 阶段E goal pose 一致） | grep；**非 map = Critical FAIL**（阶段E 客户端 goal 被拒） |

---

## 3. 静态验证命令（Generator 自检 + Critic 复核）

### 3.1 YAML 语法 + 关键参数
```bash
# YAML 语法正确
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/slam_toolbox.yaml')); print('slam YAML OK')"
python3 -c "import yaml; yaml.safe_load(open('src/go2w_nav/config/nav2_params_slim.yaml')); print('slim YAML OK')"

# === 头号风险: QoS ===
echo "=== slam use_sensor_data_qos (应 false) ==="
grep "use_sensor_data_qos" src/go2w_nav/config/slam_toolbox.yaml
echo "=== slim obstacle_layer reliability (应 reliable, 2 处) ==="
grep -B1 -A1 "reliability" src/go2w_nav/config/nav2_params_slim.yaml

# === frame 架构 ===
echo "=== slam frame ==="
grep -E "base_frame|odom_frame|map_frame|scan_topic|max_laser_range" src/go2w_nav/config/slam_toolbox.yaml

# === slim 关键 diff ===
echo "=== slim odom_topic (应 /odom 小写) ==="
grep "odom_topic" src/go2w_nav/config/nav2_params_slim.yaml
echo "=== slim global_frame (应 map) ==="
grep "global_frame" src/go2w_nav/config/nav2_params_slim.yaml
echo "=== slim local_costmap plugins (应无 voxel_layer) ==="
grep -A1 "plugins:" src/go2w_nav/config/nav2_params_slim.yaml
```

### 3.2 launch 结构核对
```bash
# launch 能解析
ros2 launch go2w_nav slam.launch.py --show-args 2>&1 | head -5
ros2 launch go2w_nav nav2_slim.launch.py --show-args 2>&1 | head -5

# 禁止项检查（应无输出）
echo "=== slam.launch 禁止 TF 桥 ==="
grep "static_transform_publisher" src/go2w_nav/launch/slam.launch.py
echo "=== nav2_slim 禁止 TF 桥 / p2l ==="
grep -E "static_transform_publisher|pointcloud_to_laserscan" src/go2w_nav/launch/nav2_slim.launch.py
echo "=== nav2_slim 禁止 /cmd_vel remap ==="
grep "cmd_vel" src/go2w_nav/launch/nav2_slim.launch.py
echo "=== 禁止 amcl / map_server ==="
grep -E "amcl|map_server" src/go2w_nav/launch/slam.launch.py src/go2w_nav/launch/nav2_slim.launch.py
echo "=== 禁止 nav2_recoveries (应 nav2_behaviors) ==="
grep "nav2_recoveries" src/go2w_nav/config/nav2_params_slim.yaml

# 必须项检查
echo "=== slam.launch 必须有 mode arg ==="
grep -E "DeclareLaunchArgument.*mode|IfCondition" src/go2w_nav/launch/slam.launch.py
echo "=== slam.launch 必须切 async/localization executable ==="
grep -E "async_slam_toolbox_node|localization_slam_toolbox_node" src/go2w_nav/launch/slam.launch.py
echo "=== nav2_slim 必须有 lifecycle_manager + node_names 含 bt_navigator ==="
grep -A8 "node_names" src/go2w_nav/launch/nav2_slim.launch.py
echo "=== nav2_slim params_file 默认指向 slim yaml ==="
grep "nav2_params_slim" src/go2w_nav/launch/nav2_slim.launch.py
```

### 3.3 红线文件未改
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

---

## 4. 评分规则

### 4.1 单项判定
- **PASS**：检查项完全满足通过标准
- **FAIL**：不满足（按严重度计入门槛）
- **SKIP**：不适用（如 NX 未恢复时实跑项，记录待联调验证）

### 4.2 整体判定
- **通过（APPROVED）**：所有 Critical（C）项 PASS；High（H）项 ≤ 2 FAIL 且有书面解释；Medium（M）项 ≤ 4 FAIL
- **打回（REJECTED）**：任一 Critical FAIL；或 High FAIL > 2；或 Medium FAIL > 4

打回时 Critic 列出所有 FAIL 项 + 修正建议，Generator 修正后重新审。

### 4.3 加分项（可选，不计入门槛）
- runbook 覆盖 QoS 不匹配故障注入（kill nx_sensor 看 slam_toolbox /map 停止生长）
- runbook 覆盖 mock /scan 建图步骤（供 NX 未恢复时调试）
- slam_toolbox.yaml 注释详尽（每个参数说明为何此值，特别是 use_sensor_data_qos 的反直觉解释）
- launch 支持 `map_file` / `mode` arg 覆盖（方便联调时切换）

---

## 5. 重点 Critical 项速查（Critic 先看这 13 项，任一 FAIL 直接打回）

| # | 项 | 一句话 |
|---|---|---|
| 1 | **slam use_sensor_data_qos = false** | **头号风险**：nx_sensor /scan 发 RELIABLE，slam_toolbox 默认 BEST_EFFORT 不匹配静默失败 |
| 2 | slam scan_topic = /scan | nx_sensor 发的 topic 名 |
| 4 | **不改 nx_sensor_node.py** | **阶段A 红线 + /scan 来源** |
| 6 | slam base_frame = base_link | nx_sensor /scan frame_id（非 laser_frame） |
| 7 | slam odom_frame = odom | nx_sensor /odom frame |
| 8 | slam map_frame = map | Nav2 + 阶段E 一致 |
| 9 | **slam.launch 无 TF 桥** | slam_toolbox 发 map→odom，无 static_transform |
| 10 | **nav2_slim 无 TF 桥** | slim 路线与阶段D nav2_3d 不同，无 TF 桥 |
| 16 | **slim odom_topic = /odom（小写）** | nx_sensor 发 /odom（非 FAST_LIO 的 /Odometry） |
| 18 | nav2_slim lifecycle 不含 amcl | slam_toolbox localization 替代 |
| 20 | **nav2_slim 无 /cmd_vel remap** | **狗乱转 Critical**（继承阶段D 决策 5） |
| 22 | **不改 nav2_params_3d / nav2_3d.launch** | **阶段D 红线** |
| 23 | 不改 nx_motion / orchestrator / mock_nav2 | 阶段A/E 红线 |
| 24 | 不改 nx_web / nx_sensor | 阶段A 红线 |
| 25 | slim global_frame = map | 阶段E goal frame 一致 |

---

## 6. Critic 报告模板

```markdown
# 阶段F 静态审报告

## 总判定: [APPROVED / REJECTED]

## Critical 项 (C): [N/15 PASS]
- [PASS/FAIL] #1 slam use_sensor_data_qos=false: ...
- [PASS/FAIL] #2 slam scan_topic=/scan: ...
- [PASS/FAIL] #4 nx_sensor_node.py 未改: ...
- [PASS/FAIL] #6 slam base_frame=base_link: ...
- [PASS/FAIL] #7 slam odom_frame=odom: ...
- [PASS/FAIL] #8 slam map_frame=map: ...
- [PASS/FAIL] #9 slam.launch 无 TF 桥: ...
- [PASS/FAIL] #10 nav2_slim 无 TF 桥: ...
- [PASS/FAIL] #16 slim odom_topic=/odom: ...
- [PASS/FAIL] #18 nav2_slim lifecycle 不含 amcl: ...
- [PASS/FAIL] #20 nav2_slim 无 /cmd_vel remap: ...
- [PASS/FAIL] #22 nav2_params_3d / nav2_3d.launch 未改: ...
- [PASS/FAIL] #23 nx_motion/orchestrator/mock_nav2 未改: ...
- [PASS/FAIL] #24 nx_web/nx_sensor 未改: ...
- [PASS/FAIL] #25 slim global_frame=map: ...

## High 项 (H): [N/7 PASS]
- [PASS/FAIL] #3 slim obstacle_layer reliability=reliable: ...
- [PASS/FAIL] #11 nav2_slim 无 pointcloud_to_laserscan: ...
- [PASS/FAIL] #12 slam.launch mode arg: ...
- [PASS/FAIL] #13 mode=mapping 起 async node: ...
- [PASS/FAIL] #14 mode=localization 起 localization node: ...
- [PASS/FAIL] #15 localization 注入 map_file_name + map_start_pose: ...
- [PASS/FAIL] #19 nav2_slim lifecycle 不含 map_server: ...
- [PASS/FAIL] #21 nav2_slim params_file 默认 slim yaml: ...

## Medium 项 (M): [N/2 PASS]
- [PASS/FAIL] #5 slam max_laser_range=10.0: ...
- [PASS/FAIL] #17 slim local_costmap 不含 voxel_layer: ...

## FAIL 项修正建议
- #XX [项名]: 当前值 X，应改为 Y，理由 Z

## 加分项
- [有/无] runbook QoS 故障注入
- [有/无] runbook mock /scan 建图步骤
- [有/无] yaml 注释详尽（含 use_sensor_data_qos 反直觉解释）

## 结论
[通过 / 打回 + 一句话理由]
```

---

## 7. 与阶段D rubric 的对照（Critic 跨阶段一致性检查）

阶段F 与阶段D 都是 Nav2 + localization 配置，但有 5 处关键差异（Critic 审阶段F 时要确认这些差异是**有意为之**，不是抄阶段D 抄错）：

| 项 | 阶段D（FAST_LIO 路线） | 阶段F（slam_toolbox 路线） | 差异理由 |
|---|---|---|---|
| odom_topic | /Odometry（大写，FAST_LIO） | /odom（小写，nx_sensor） | 数据源不同 |
| TF 桥 | 有 2 个 static_transform（map→camera_init, body→base_link） | **无**（slam_toolbox + nx_sensor 直接覆盖） | localization 源不同 |
| pointcloud_to_laserscan | 有（MID360 → /scan） | **无**（nx_sensor 已发 /scan） | scan 源不同 |
| local_costmap voxel_layer | 有（吃 /livox/lidar，双保险） | **无**（无 3D 点云源，单保险） | 硬件不同 |
| localization 源 | FAST_LIO（camera_init→body + TF 桥） | slam_toolbox localization node（直接发 map→odom） | 决策 2 |
| amcl | 不启（FAST_LIO 替代） | 不启（slam_toolbox localization 替代） | 一致 |
| /cmd_vel remap | 无（零反转） | 无（零反转） | 一致（继承） |
| global_frame | map | map | 一致（阶段E 契约） |
| action 名 | /navigate_to_pose（不 remap） | /navigate_to_pose（不 remap） | 一致（阶段E 契约） |

**Critic 注意**：阶段F 的 nav2_slim 若与阶段D nav2_3d **完全相同**（没做上述 diff），那是抄错——Critical FAIL（odom_topic / TF 桥 / p2l / voxel_layer 至少要 diff）。
