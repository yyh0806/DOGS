# 封闭区域 Frontier 探索（标人副产品）

> 日期: 2026-07-19
> 状态: 设计待审
> 取代: 早期"饱和驱动主动找人"构想（目标错位，已弃）

## 目标

从起点位姿出发，对未知封闭区域做 frontier-based 探索，把四周墙壁内所有可达地方都跑一遍，直到 frontier 耗尽。探索过程中 YOLO 顺带把看到的人标到 map 上。

## 问题模型

- **起点**：进门后的某个位姿（`mission_origin`）。
- **环境**：四周墙壁包围的封闭区域，内部有障碍物（家具等）。
- **避障**：由 Nav2 costmap + DWB 负责（已外包给导航团队）。搜索算法只发 goal pose，不碰 `/cmd_vel`。
- **终止**：封闭区域 frontier 单调减少到 0，自然收敛。

这是经典的 Yamauchi 1997 frontier-based autonomous exploration 问题在封闭区域上的实例。

## 关键决策

1. **纯 frontier 探索，不加边界约束**。墙是天然边界，frontier 会在墙边自然停下。门不做任何特殊处理——frontier 是否从门溜出去此刻不纳入算法，先把封闭区域探索本身跑稳。
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
- `max_plan_probes` / `max_blacklist_entries` 防止预算耗尽在死胡同

### 覆盖率度量（成功验证）

REPORT 阶段扫一遍最终 occupancy grid，分别计数 free cell（值 `0`）、occupied cell（值 `100`）、unknown cell（值 `-1`），计算 `explored_ratio = (free + occupied) / (free + occupied + unknown)`。

frontier 耗尽时，剩余的 unknown 都是被障碍物完全围死的不可达区域（没有 free 邻居，否则会有 frontier），因此 `explored_ratio` 自然接近 1.0。这些不可达死角的大致位置（连通 unknown 区域的 bounding box）写入 `mission_report`，作为"已知未覆盖"信息呈现给用户，而非失败。

### 终止判据（极简）

主判据：`reachable_frontiers_exhausted`（无前沿可达）。封闭区域 frontier 单调减到 0，等价于所有可达区域被覆盖。
兜底：时间预算 `max_time_s`、距离预算 `max_distance_m`、电量预留 `battery_reserve_percent`（全部 `ExplorationManager` 已有）。

不使用：覆盖率阈值**触发停止**（停止信号是 frontier 耗尽，不是覆盖率达标）、找人饱和判据、最少 viewpoint 数。覆盖率只用于 REPORT 阶段的成功验证。

### YOLO 标人（副产品）

- **触发**：移动中（导航期间周期采样）+ 到点（每次到达 frontier 后）。
- **流程**：读 `ai_engine` detection snapshot → 取当前 `robot_pose` → 调 `nx_person_localizer.localize_target_detection`（bbox bearing × LaserScan range → map 坐标）→ `TargetMissionStore` 空间去重（0.7m 合并）+ 照片 artifact → `ws_broadcast` 发 `person_markers`。
- **关键约束**：标人结果**不进入 viewpoint 打分**，不改路径。

## 7/15 实测遗留的实机改进点

来自 `voice-search-e2e-test-2026-07-15`：

1. **移动中连续检测**（取代到点单次采样）—— 现状 `_run_frontier_explore` 在 `send_goal_and_wait` 期间不检测，狗经过的人会漏。改成导航期间开 observer 周期采样（`Δd=0.5m` 或 `τ=0.4s`），到达后 join。
2. **预算参数调整** —— `max_time=180s` 太短，调到合理值（如 `300s`，可配置）。
3. **单 goal 超时调整** —— `send_goal_and_wait` 内部 `120s` 太长，调短（如 `60s`），到不了就 fail 换下一个 frontier。
4. **`parked_state_lost` 恢复** —— motion 层问题（非本方案范围），但搜索层要能容忍 pose 临时丢失、等 pose 恢复后再继续。

## 复用清单

| 模块 | 用途 | 改动 |
|---|---|---|
| `nx_frontier_planner.py` | frontier 提取 + 打分 | 无 |
| `nx_exploration_manager.py` | 持久探索策略 + 预算 + 黑名单 | 无 |
| `nx_room_orchestrator.py::_run_frontier_explore` | 探索状态机 + Nav2 调用 + DETECT | 加移动中连续检测；REPORT 阶段加 occupancy grid 覆盖率统计 |
| `nx_person_localizer.py` | bbox bearing × range 定位 | 无 |
| `nx_person_mission.py::TargetMissionStore` | 标记去重 + 照片 + 报告 | 无 |
| `nx_navigation_gateway.py` | Nav2 action client facade | 无 |

## 非目标（明确砍掉）

- 门检测 / 不出门约束 / 起点半径 / 房间 polygon / 在线房间分割
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
- 全程 YOLO 标记可见的人，无漏标（移动中采样生效）
- 预算合理（不因 180s 太短提前退出）
- pose 临时丢失时能恢复继续，不 EMERGENCY

## 成功判据

1. **全屋覆盖**：封闭区域内所有可达 free space 都被探索到。验证指标 `explored_ratio ≥ 0.95`。剩余 ≤5% 为被障碍物围死的不可达死角，需在 `mission_report` 列出大致位置（bounding box），不算失败。退出原因是 frontier 耗尽，不是预算耗尽。
2. 内部障碍物后的区域通过 frontier 绕行覆盖，或被黑名单合理跳过（不卡死、不无限重试）。
3. 移动中经过的人被标注到 map（不是只标到点的）。
4. 整个探索过程中狗不发 `/cmd_vel`，所有移动走 Nav2 goal pose。
5. frontier 耗尽时正常 REPORT，输出 mission_report。
