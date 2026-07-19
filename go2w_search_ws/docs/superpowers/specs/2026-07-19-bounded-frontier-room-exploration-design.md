# 封闭区域 Frontier 探索（标人副产品）

> 日期: 2026-07-19
> 状态: v3 已实现（大房间动态扩展 / 6m 分块 / 稳定耗尽 / 有界覆盖率）
> 取代: 早期"饱和驱动主动找人"构想（目标错位，已弃）
> Tech Stack: Python ≥ 3.10（NX 部署机 3.10.12；非 3.12）

## 目标

从起点位姿出发，对未知封闭区域做 frontier-based 探索，把四周墙壁内所有可达地方都跑一遍，直到 frontier 耗尽。探索过程中 YOLO 顺带把看到的人标到 map 上。

## 问题模型

- **起点**：进门后的某个位姿（`mission_origin`）。
- **环境**：四周墙壁包围的封闭区域，内部有障碍物（家具等）。
- **避障**：由 Nav2 costmap + DWB 负责（已外包给导航团队）。搜索算法只发 goal pose，不碰 `/cmd_vel`。
- **终止**：搜索半径扩展到安全上限后，连续 3 个选择周期没有可达 frontier，自然收敛。

这是经典的 Yamauchi 1997 frontier-based autonomous exploration 问题在封闭区域上的实例。

## 关键决策

1. **纯 frontier 探索，只加任务安全包络**。墙是天然物理边界，算法不做在线门检测；`max_radius_m` 只限定最远探索范围，防止定位或开门环境导致无界外溢。
2. **避障不是本方案职责**。Nav2 算路失败（障碍挡住某 frontier）时，搜索层换下一个 frontier——这是"避障"在搜索层的唯一体现。
3. **YOLO 标人是副产品**。它订阅 detection + 当前 pose，看到人就在 map 上画 marker，**不参与 viewpoint 选择**，不影响路径。
4. **最大化复用现有栈**：`nx_frontier_planner`、`nx_exploration_manager`、`nx_person_localizer`、`TargetMissionStore`、`RoomSearchOrchestrator._run_frontier_explore` 全部沿用，不重写。

## 算法

### Frontier 探索循环（核心，已有）

```
loop:
    map_msg ← 最新 /map_frontier 帧
    robot_pose ← 实时位姿
    target ← ExplorationManager.choose_next(map_msg, robot_pose)
    if target is None:
        break  # frontier 耗尽
    Nav2.send_goal_and_wait(target.x, target.y, target.yaw)
    if 失败:
        ExplorationManager.mark_navigation_failed(reason, target)
        continue  # 黑名单该 cell，换下一个
    ExplorationManager.mark_visited(target)
```

底层组件职责：
- `nx_frontier_planner.find_frontier_clusters`：occupancy grid 上 free/unknown 边界的连通分量聚类，代表点取**离 robot 最近的 free cell**（不是 centroid，避免大 frontier ring 的质心落在未知区）。
- `nx_frontier_planner.select_frontier_candidates`：cost-distance 打分 `gain / (1 + α·path_cost) - β·heading_change - γ·failures`。
- `nx_exploration_manager.ExplorationManager`：持久状态、预算、黑名单、path preflight（`compute_path_to_pose` 预算内探测可达性）、`reject_map_edge`（滤掉 rolling-window 边界伪 frontier）。

### 不可达 frontier 处理（已有）

障碍物挡住的 frontier，Nav2 算路会失败。处理链：
- `mark_navigation_failed` → 记录到 `_blacklist[(map_revision, cell)]`
- `max_failures_per_cell` 次失败后该 cell 不再被选
- `max_plan_probes_per_cycle`（默认 12）限制单次选择的 Nav2 预规划次数；累计 probe 只作为遥测，不再导致整场任务提前终止。
- 空间失败记录不绑定原始地图 revision，避免同一不可达 cell 因栅格轻微变化被无限重试；成功移动或扩展半径后允许重新评估。

### 大房间动态扩展与分块

`current_room` 不再固定搜索 6m 半径。任务以 `initial_radius_m=6m` 开始；局部 frontier 耗尽时按 `radius_step_m=6m` 扩展，直到 `max_radius_m=30m` 安全上限。时间预算默认 `1800s`，waypoint 安全上限默认 `200`。

活跃范围按 `tile_size_m=6m` 划分。管理器优先耗尽当前 tile 的 frontier，再选择距离机器人最近的候选 tile。分块是调度优先级，不是硬墙：Nav2 仍在统一 costmap 上规划，障碍物与可达性仍由 Nav2 决定。

这种组合解决两类提前停止：动态半径避免只探索入口附近；分块避免远处高 gain frontier 反复抢占任务，使相邻区域逐块收敛。它不依赖预先存在的房间地图。

### 覆盖率度量（ROI 内成功验证）

REPORT 阶段扫一遍最终 occupancy grid，但**只在任务 ROI 内**计数 free cell（值 `0`）、occupied cell（值 `100`）、unknown cell（值 `-1`），计算 `explored_ratio = (free + occupied) / (free + occupied + unknown)`。

**ROI 定义**：`mission_origin` 为圆心 + 任务结束时 `active_radius_m` 的圆（最大不超过默认 30m 安全包络）；命名房间用 `room_polygon`。**必须限定 ROI**——`/map_frontier` 被 `map_padding_bridge.py` 在四周加了 2m unknown padding，整图统计会把 padding 灌进分母（实测整图覆盖率仅 ~31.66%），不代表任务覆盖。

**enclosed_unknown_regions**（取代旧名 `dead_zones`，审核 #4）：ROI 内的连通 unknown 区域，且满足全部三条：(a) 不接任何"可达 free cell"（从 mission_origin 对膨胀后 free space 做 flood-fill 得到）；(b) 不接触 ROI 边界；(c) 不接触地图边界。接触 ROI/地图边界的 unknown 可能是墙体内部/建筑外/padding，**不报告**。每个区域输出 world-frame bounding box `{min_x, min_y, max_x, max_y, cell_count}`。

**有界覆盖率**：unknown 连通域若接触 ROI 或地图边界，分类为 `exterior_unknown_cells`，不进入房间内部完成率分母；其余归为 `interior_unknown_cells`。完成判定优先使用 `bounded_explored_ratio = (free + occupied) / (free + occupied + interior_unknown)`。原始 `explored_ratio` 保留用于诊断和兼容，不能单独代表闭合房间完成度。

地图不可用或 ROI 为空时，`compute_coverage` 返回 `None`，`completion_status="coverage_unverified"`——**不伪装 0.0**。

### 终止判据（极简）

主判据：活跃半径已达到 `max_radius_m`，并连续 `stable_exhaustion_cycles=3` 个选择周期没有可达 frontier 后，输出 `reachable_frontiers_exhausted`。半径扩展、tile 切换、单轮 probe 用尽和稳定性确认等待都是继续状态，不是任务终止。
兜底：时间预算 `max_time_s`、距离预算 `max_distance_m`、电量预留 `battery_reserve_percent`（全部 `ExplorationManager` 已有）。

不使用：覆盖率阈值**触发停止**（停止信号是 frontier 耗尽，不是覆盖率达标）、找人饱和判据、最少 viewpoint 数。

**`completion_status` 四态**（REPORT 输出，停止信号与完成状态分离，审核 #5）：

| 条件 | status |
|---|---|
| 地图不可用 / ROI 无效 | `coverage_unverified` |
| 时间/距离/规划预算耗尽退出 | `incomplete` |
| 稳定 frontier 耗尽 AND 有界覆盖率 ≥ 阈值 AND 无 enclosed | `completed` |
| 稳定 frontier 耗尽 AND (有界覆盖率 < 阈值 OR 有 enclosed) | `completed_with_gaps` |

阈值默认 `0.90`（旧 0.95 因 padding 灌水实测不可达，已下调；env `GO2W_FRONTIER_COVERAGE_THRESHOLD` 可配）。覆盖率只用于 REPORT 阶段的完成状态判定，不触发停止。

### YOLO 标人（副产品）

- **触发**：移动中（导航期间周期采样）+ 到点（每次到达 frontier 后）。
- **流程（到点）**：等新鲜帧 → 复用 `observation_sync.bundle_for_detection(captured_at)` 时间对齐 → `localize_target_detection` → `TargetMissionStore.add_observation`（range_lidar）/ `add_unresolved_observation`（bearing_only）。
- **流程（移动中 / en-route）**：**复用同一个 bundle 路径**（不事后读最新 scan，避免时间错位，审核 #1）；worker 线程按 detection 帧 `captured_at` 去重（同帧只处理一次），bounded queue ≤12（防 100 张 720p 堆积，审核 #2）；只收 `range_lidar`，bearing_only 留到点稳态；worker **不写 store**，主线程 join 后串行 ingest。
- **关键约束**：标人结果**不进入 viewpoint 打分**，不改路径。
- **不保证 recall**：背后/遮挡的人可能漏标；本方案是"沿途可见人员 best-effort 标注"，不是"室内所有人无漏"。云台多角度扫描作为可选 stretch（见非目标）。

## 7/15 实测遗留的实机改进点

来自 `voice-search-e2e-test-2026-07-15`：

1. **移动中连续检测**（取代到点单次采样）—— 现状 `_run_frontier_explore` 在 `send_goal_and_wait` 期间不检测，狗经过的人会漏。改成导航期间开 observer 周期采样（`τ=0.4s`，env 可配），**复用 observation_sync bundle 时间对齐**，capture_stamp 去重 + bounded queue，到达后 join + 串行 ingest。
2. **预算参数调整** —— `current_room` 默认 `max_time=1800s`、`max_frontiers=200`，并保持可配置；物理运行仍受电量、距离和最大半径约束。
3. **单 goal 超时调整** —— `send_goal_and_wait` 内部 `120s` 太长，调短（如 `60s`），到不了就 fail 换下一个 frontier。
4. **`parked_state_lost` 恢复** —— motion 层问题（非本方案范围），但搜索层要能容忍 pose 临时丢失、等 pose 恢复后再继续。

## 复用清单

| 模块 | 用途 | 改动 |
|---|---|---|
| `nx_frontier_planner.py` | frontier 提取 + 打分 | 无 |
| `nx_exploration_manager.py` | 持久探索策略 + 预算 + 黑名单 | 加动态半径、tile 调度、每轮 probe 预算和稳定耗尽 |
| `nx_room_orchestrator.py::_run_frontier_explore` | 探索状态机 + Nav2 调用 + DETECT | 接入自适应参数；REPORT 阶段加有界 occupancy grid 覆盖率统计 |
| `nx_person_localizer.py` | bbox bearing × range 定位 | 无 |
| `nx_person_mission.py::TargetMissionStore` | 标记去重 + 照片 + 报告 | 无 |
| `nx_navigation_gateway.py` | Nav2 action client facade | 无 |

## 非目标（明确砍掉）

- 门检测 / 不出门几何约束 / 在线房间分割
- 覆盖率阈值终止
- 找人 recall 导向（viewpoint 打分加"期望新增 person"项）
- bearing_only 主动确认（侧移三角化）
- in-place sweep / 云台扫描
- coverage path 补漏（frontier 耗尽后去未观测栅格补点）
- 重写 frontier 栈、替换为 `mrtsp_exploration_ros2` 等外部包

## 验证计划

离线（纯逻辑，已有测试基础）：
- frontier 提取在合成 occupancy grid 上正确聚类
- `ExplorationManager` 黑名单 + 预算 + path preflight 行为正确
- 移动中连续检测的 observer 在 mock Nav2 + mock detection 上不漏帧、不阻塞导航

仿真/集成：
- 封闭房间 + 内部障碍物，frontier 探索从起点到 frontier 耗尽
- 移动中检测：路径经过的人被标注
- 不可达 frontier：障碍后的 frontier 被黑名单跳过，不卡死

实机验收：
- 起点位姿 → 探索完整个封闭区域 → frontier 耗尽退出
- 全程 YOLO best-effort 标记沿途可见的人（en-route + 到点；不保证 recall，背后/遮挡的人可能漏）
- 预算合理（默认 300s，不因预算太短提前退出）
- pose 临时丢失时能恢复继续，不 EMERGENCY

## 成功判据

1. **全屋覆盖（安全包络内）**：最大半径内所有传感器已建图且 Nav2 可达的 frontier 都被探索到。验证指标 `bounded_explored_ratio ≥ 0.90`；原始 `explored_ratio` 仅作诊断。剩余 enclosed_unknown_regions 在 `mission_report` 列出 world-frame bounding box。退出原因是连续 3 轮 frontier 耗尽，`completion_status ∈ {completed, completed_with_gaps}`；预算耗尽则 `incomplete`；地图缺失则 `coverage_unverified`。
2. 内部障碍物后的区域通过 frontier 绕行覆盖，或被黑名单合理跳过（不卡死、不无限重试）。
3. 移动中经过的人被 best-effort 标注到 map（不止标到点的）；**不保证 recall**，背后/遮挡的人可能漏。
4. 整个探索过程中狗不发 `/cmd_vel`，所有移动走 Nav2 goal pose。
5. frontier 耗尽时正常 REPORT，输出含 `completion_status` + coverage 的 mission_report。

## 保证边界

“全部探索完”的工程含义是：在定位、激光雷达、地图更新和 Nav2 正常的前提下，配置的 `max_radius_m` 内没有剩余可达 frontier，并且这一状态连续出现 3 次。它不能保证墙后、封闭障碍内部、传感器盲区、物理不可达区域或 30m 安全包络以外的空间；这些情况通过 `completed_with_gaps`、`enclosed_unknown_regions` 和原始/有界覆盖率一起暴露，不能伪装成已覆盖。
