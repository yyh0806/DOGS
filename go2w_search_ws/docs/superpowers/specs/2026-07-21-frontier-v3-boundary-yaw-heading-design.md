# Frontier 探索 v3：边界感知 + yaw 优化 + 直线优先

> 日期: 2026-07-21
> 状态: design approved（待写实现 plan）
> 前置: v0.95（commit `541fc07`, frontier v2 mixed-utility + 并行 probe + 封闭判据 + trap-escape）
> 取代: 无（v0.95 的演进，非推翻）
> Tech Stack: Python ≥ 3.10（NX 部署机 3.10.12）
> 范围: 算法层（exploration_manager / frontier_planner / visibility_coverage）。第 4 点「速度只看前方扇区」触碰 nav2 DWB controller，**分拆到独立 spec** `nav2-controller-forward-bias`，本 spec 不含。

## 背景

v0.95 的 mixed-utility 打分 `α·size + β·visual_gain − γ·path_cost` 解决了「最近优先」导致大远 frontier 饿死的问题，但有三个盲区（用户 2026-07-21 提出）：

1. **无边界概念**：`nx_frontier_planner.py` 只有 free/unknown/occupied 三态，无「墙/边界」语义；`detect_room_enclosure` 用周界墙占比判封闭是**被动终止判据**，不是主动「优先确定边界」。
2. **yaw 固定**：frontier 的 yaw 写死成 `atan2(dy,dx)`（朝 frontier，`nx_frontier_planner.py:446`）。真视锥 frustum 建模已做好（`_visible_buckets` FOV + raycast + 墙遮挡 + 朝向敏感，`nx_visibility_coverage.py:480-529`）但 yaw 不参与优化——视锥中心可能对着已观测区，扫不到最大未观测面积。
3. **heading 是 tiebreaker**：`heading_change` 在 mixed 模式是第 4 优先级 tiebreaker，不是主成本。狗 `max_vel_theta = 1.0 rad/s`（`nav2_params_3d.yaml`），原地转 90° = 1.57s、180° = 3.14s 纯静止时间；一场 20 waypoint 任务累计转身浪费可达 ~20s。

第 4 点「速度只看前方扇区」本质是改 nav2 DWB controller / costmap inflation，突破上一轮 design doc 定的「搜索层只发 goal pose，不碰 `/cmd_vel`」边界，风险高且与搜索算法耦合度低，**分拆到独立 spec**。

## 目标

在 v0.95 mixed-utility 框架内，三个改进点合体：

1. **边界感知**（第 1 点）：frontier + `wall_proximity_bonus` 加权，优先探墙边/墙角 frontier，尽早确定房间轮廓。
2. **yaw 优化**（第 2 点）：yaw 从固定 `atan2` 提升为 mixed-utility 的优化变量，每个 frontier 试 K 个 yaw 候选选最优。
3. **直线优先**（第 3 点）：heading 从 tiebreaker 升为主成本项，用时间归一化（秒）统一 path_cost 和 heading 量纲。

三者合体的关键：**yaw 作为优化变量**——mixed-utility 在「选 yaw」和「选 frontier」两层都同时用 visual_gain（收益）和 heading（成本），避免两者在 sort key 层面拉锯。

## 核心设计

### 统一打分公式（v3）

```
utility(candidate, yaw) =
      α  · information_gain            # frontier size（v0.95, mixed_frontier_weight）
    + β  · visual_gain(yaw)            # frustum 未观测 cell 数（v0.95, yaw 变优化变量）
    + δ  · wall_proximity_bonus(x,y)   # 【新·第1点】mixed_wall_bonus
    − k_time · (t_travel + t_turn)     # 【新·第3点】时间归一化主成本
```

其中：
- `t_travel = path_cost / max_vel_x`（秒）
- `t_turn = abs(angle_delta(yaw, robot_yaw)) / max_vel_theta`（秒）
- `path_cost`：preflight 阶段用 euclidean distance，Nav2 probe 后用真实 `path_length`（两阶段排序，同 v0.95）
- `max_vel_x` / `max_vel_theta`：从 `nav2_params_3d.yaml` 读或 env 注入（默认 1.5 / 1.0）

**与 v0.95 的差异**：
- `−γ·path_cost`（米）→ `−k_time·t_travel`（秒）：量纲从米变秒
- `heading_change` 从 tiebreaker → `−k_time·t_turn` 主成本
- 新增 `+δ·wall_proximity_bonus`
- `visual_gain(yaw)`：yaw 从固定变优化变量

### 两层优化结构

```
外层（选 frontier）:
  for frontier in candidates:
      frontier.best_yaw, frontier.utility = argmax_yaw utility(frontier, yaw)
  sort candidates by frontier.utility

内层（选 yaw）:
  yaw_candidates = { robot_yaw, robot_yaw±60°, atan2(dy,dx) }  # K≤4, 去重
  for yaw in yaw_candidates:
      vg = _visible_buckets(frontier_pos, yaw).difference(_observed)
      hc = abs(angle_delta(yaw, robot_yaw))
      wp = wall_proximity_bonus(frontier)            # 见下
      t  = path_cost / max_vel_x + hc / max_vel_theta
      u  = α·size + β·|vg| + δ·wp − k_time·t
      keep max
```

**关键性质**：
- `robot_yaw` 进候选集 = 「不转身」永远备选。若 visual_gain 差距不够大，`k_time·t_turn` 占优 → 选不转身。第 3 点「直线优先」的内生表达。
- `coverage_candidates` 已有的 4-yaw 优化（`nx_visibility_coverage.py:295-310`）被统一吸收进内层，frontier 和 coverage 走同一套 yaw 优化。
- yaw 优化发生在 `_select_candidates` / `rank_candidates` 阶段（preflight，Nav2 probe 前），`path_cost` 用 euclidean 近似；probe 后的 re-rank 阶段（`reachable.sort`）用真实 `path_length` 再排——与 v0.95 两阶段排序一致。

## 三个新项的具体形式

### wall_proximity（第 1 点）—— 邻域代理，不 BFS

`find_frontier_clusters` 提取 frontier 时本来就在扫 support_radius 邻域（`nx_frontier_planner.py:294-311`），顺手统计 candidate 周围 occupied cell 数，记成 `adjacent_wall_count`。**零额外遍历成本**。

```python
adjacent_wall_count = sum(1 for n in neighbors(candidate, support_radius)
                          if grid.value(n) >= obstacle_threshold)
# wall_proximity_bonus 是 [0,1] 几何指示（二值），δ（mixed_wall_bonus）是公式里的权重。
# 公式 +δ·wall_proximity_bonus 即 δ·1（墙角）或 δ·0（开阔）。
wall_proximity_bonus = 1.0 if adjacent_wall_count >= 2 else 0.0
```

- `adjacent_wall_count ≥ 2` = 墙角/沿墙 frontier（三面被围）
- `= 0` = 朝开阔区

正好奖励「边界 frontier」。不用「到最近墙的 BFS 距离」（O(W·H) per candidate 太贵）。

### yaw 候选集（第 2 点）

```python
yaw_candidates = {
    robot_yaw,                  # 不转身（第3点内生：永远备选）
    robot_yaw + radians(60),
    robot_yaw - radians(60),
    atan2(dy, dx),              # 朝 frontier（传统）
}  # 去重后 K ≤ 4
```

步长 60°（用户定），最大单次转身 60° ≈ 1.05s。步长 env 可配（`GO2W_FRONTIER_YAW_STEP_DEG`）。不包含 ±90°/180°（侧身扫描 visual_gain 收益不抵转身代价）。

### heading 时间归一化（第 3 点）

`t_turn = abs(angle_delta(yaw, robot_yaw)) / max_vel_theta` 把弧度换算成「转身秒数」，与 `t_travel = path_cost / max_vel_x`（行进秒数）同量纲。`k_time` 是统一的「每秒时间惩罚」。

**物理意义**：utility = 收益 − 时间成本。狗主动权衡「为多看 X 个未观测 cell，值不值得花 T 秒转身/行进」。

## 参数与默认启用

| 参数 | 含义 | 默认 | env |
|---|---|---|---|
| `utility_mode` | nearest/mixed | `nearest`（代码默认，回归保护）| `GO2W_FRONTIER_UTILITY_MODE` |
| `k_time` | 每秒时间惩罚（收益单位/秒）| sim 标定 | `GO2W_FRONTIER_TIME_PENALTY` |
| `α` (size weight) | frontier 规模 | 0.5 | `GO2W_FRONTIER_MIXED_FRONTIER_WEIGHT` |
| `β` (visual_gain weight) | 视锥收益 | 1.0 | `GO2W_FRONTIER_MIXED_VISUAL_GAIN_WEIGHT` |
| `δ` (wall_bonus) | 墙奖励 | sim 标定 | `GO2W_FRONTIER_MIXED_WALL_BONUS` |
| `yaw_step_deg` | yaw 步长 | 60 | `GO2W_FRONTIER_YAW_STEP_DEG` |
| `max_vel_x` | 前向速度上限 (m/s) | 1.5 | `GO2W_FRONTIER_MAX_VEL_X` |
| `max_vel_theta` | 角速度上限 (rad/s) | 1.0 | `GO2W_FRONTIER_MAX_VEL_THETA` |

**默认启用方式**：代码默认仍是 `nearest`（保护 v0.95 的 50 单测）；`bringup_slam_nav2.sh` / deploy env 设 `GO2W_FRONTIER_UTILITY_MODE=mixed`，部署层启用 v3。代码层向后兼容，部署层一键开。

**权重标定**：`sim_strategy_compare.py` 加 `v3` 列，`(k_time, δ)` 网格搜索，目标相同覆盖率下 `total_path_m` ↓ + `total_turn_rad` ↓。

## 错误处理 / 退化

| 场景 | 退化行为 |
|---|---|
| `visibility_tracker is None`（nearest / 测试 stub） | yaw 固定 atan2；wall_bonus 仍由 frontier_planner `adjacent_wall_count` 算（不依赖 tracker） |
| nav2 `max_vel_x` / `max_vel_theta` 读不到 | env 注入默认（1.5 / 1.0） |
| `adjacent_wall_count` 全 0（开阔区） | wall_bonus 全 0，退化为 `α·size + β·visual_gain − k_time·t` |
| 所有 yaw 的 Nav2 probe 失败 | 沿用 v0.95 serial fallback + trap-escape |
| yaw 优化 raycast 计算量 | K≤4 × N≤32 ≤ 128 次 `_visible_buckets`，参考现状 0.05s/50 测试，<50ms |
| mixed 模式 visual_gain==0 的 frontier | yaw 优化可能救活（换 yaw 扫到新区域，visual_gain 变正），扩大有效候选池 |

## 测试策略

**单元测试**（`test_exploration_manager.py`，+6）：

1. `test_mixed_yaw_optimization_picks_visual_gain_yaw` — robot_yaw 朝已观测区，±60° 朝未观测区 → 选 ±60° yaw
2. `test_mixed_heading_prefers_no_turn_when_visual_gain_close` — 两 frontier visual_gain 接近 → 选不转身的（验证 heading 内生）
3. `test_mixed_wall_bonus_prefers_corner_frontier` — size/visual_gain/path 相同，`adjacent_wall_count` 3 vs 0 → 选墙角
4. `test_mixed_time_normalization_respects_max_vel_theta` — `max_vel_theta` 调小 → heading 惩罚变大 → 更偏好不转身
5. `test_nearest_mode_unchanged_by_v3` — nearest 模式 yaw 固定 + 无 wall_bonus（回归保护，50 旧测继续绿）
6. `test_mixed_yaw_optimization_parallel_probe_compatible` — yaw 优化 + `parallel_probe_workers>0` 不冲突

**sim 扩展**（`sim_strategy_compare.py`）：+v3 列 + `(k_time, δ)` 网格搜索 + 新指标 `total_turn_rad = Σ|heading_change|`。

**实机**：NX 在线时同一封闭房间 nearest vs v3，对比覆盖率/路径/转身。

## 成功判据（v3 相对 v0.95 的增量）

| 指标 | v0.95 基线 | v3 目标 |
|---|---|---|
| `bounded_explored_ratio` | ≥ 0.90 | ≥ 0.90（不退步） |
| `total_path_m`（同覆盖率） | 基线 | ↓ |
| `total_turn_rad` | 基线 | ↓ 30%+ |
| 墙角 frontier 访问顺序 | 随机 | 轮廓优先确定 |
| nearest 模式 50 单测 | 绿 | 绿（零回归） |

## 复用清单 + 偏离说明

| 文件 | 改动 | 偏离 |
|---|---|---|
| `nx_frontier_planner.py` | `find_frontier_clusters` 输出加 `adjacent_wall_count` 字段 | **偏离 07-19 doc**（原「无改动」）；只增字段不改接口，向后兼容 |
| `nx_visibility_coverage.py` | `rank_candidates` 加 yaw 优化 + `wall_proximity_bonus`；统一吸收 coverage 的 4-yaw 逻辑 | 扩展现有 |
| `nx_exploration_manager.py` | `_mixed_utility_sort_key` 时间归一化 + wall + yaw 集成；新增参数（k_time/δ/yaw_step/max_vel） | v0.95 mixed 路径内 |
| `nx_room_orchestrator.py` | 构造 ExplorationManager / VisibilityCoverageTracker 时注入 max_vel_x/max_vel_theta | 从 nav2 config 读或 env |
| `test_exploration_manager.py` | +6 测试 | — |
| `tools/sim_strategy_compare.py` | +v3 列 + 网格搜索 + total_turn_rad | — |
| `bringup_slam_nav2.sh` / deploy env | `GO2W_FRONTIER_UTILITY_MODE=mixed` | 部署层启用 |

## 备选方案（已否决，留追溯）

- **第 1 点 A 沿墙范式（wall-following）**：改变探索范式，矩形房间与 frontier 等价，仅 L 形受益，回归风险大。选 B 加权。
- **第 1 点 C 墙段提取**：感知层重活，与 07-19 非目标「门检测/房间分割」同类。否决。
- **第 2 点 B 参数校准**：视锥建模已做好，校准收益小。选 A yaw 优化。
- **第 2 点 C 信息熵加权**：实现复杂，cell 数够用。否决（留 stretch）。
- **第 3 点 B 路径累积转角**：probe 后才能算，preflight 阶段无数据。选 A 端点 delta（抓主要痛点：到达后对齐转身）。累积转角留 stretch。
- **第 3 点 C 改 nav2 max_vel_theta**：属于第 4 点 controller 层，已分拆。

## 非目标（本 spec 明确不做）

- 第 4 点速度扇区（→ `nav2-controller-forward-bias` spec）
- 门检测 / 不出门几何约束 / 在线房间分割（07-19 非目标）
- 路径累积转角（stretch）
- 信息熵加权 visual_gain（stretch）
- coverage path 补漏（07-19 非目标）
- 找人 recall 导向 / viewpoint 打分加「期望新增 person」项（07-19 非目标）
- 重写 frontier 栈 / 替换为 `mrtsp_exploration_ros2` 等外部包

## 验证计划

**离线（纯 python）**：
- 6 个新单元测试绿
- sim v3 列 vs nearest/mixed：H1（覆盖率不退步）+ 新指标（path ↓ / turn ↓）

**实机（NX 在线）**：
- 同封闭房间 nearest vs v3，对比覆盖率/路径/转身
- `bounded_explored_ratio ≥ 0.90`，`total_turn_rad` 明显下降
- nearest 模式 50 单测全绿（零回归）

## 保证边界

「直线优先」和「yaw 优化」是在 v0.95 mixed-utility 框架内的**打分权重重标定 + yaw 提升为优化变量**，不改变「搜索层只发 goal pose、避障外包 Nav2 DWB」的架构边界。狗仍不发 `/cmd_vel`，所有移动走 Nav2 goal pose。完成度仍以 frontier 耗尽 + `bounded_explored_ratio` 为准，本 spec 不保证墙后/封闭障碍内部/传感器盲区/物理不可达区域的覆盖——这些通过 `completed_with_gaps` + `enclosed_unknown_regions` 暴露，不伪装。
