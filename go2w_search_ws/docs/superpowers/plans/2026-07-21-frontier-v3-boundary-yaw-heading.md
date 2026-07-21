# Frontier v3 实现计划：边界感知 + yaw 优化 + 直线优先

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户指定实现走 ecc 流程（见末尾 Execution Handoff）。

**Goal:** 在 v0.95 mixed-utility 框架内实现 wall_proximity_bonus + yaw 全360°优化 + heading 时间归一化，降低转身/路径代价，nearest 模式零回归。

**Architecture:** 不改架构边界（搜索层只发 goal pose，不碰 `/cmd_vel`）。三个改进落在两处：`ExplorationManager._mixed_utility_sort_key`（打分公式：时间归一化 + wall_bonus）+ 新增 `_optimize_yaw_for_candidates`（yaw 作为优化变量，试 K 候选选 mixed-utility 最优）。`VisibilityCoverageTracker` 暴露 `visual_gain_at()` public 方法支持 yaw 优化。

**Tech Stack:** Python ≥ 3.10，纯逻辑（测试用 stub，不依赖 ROS），pytest。

## Global Constraints（从 spec 复制）

- Python ≥ 3.10（NX 部署机 3.10.12）
- **nearest 模式行为不变**（v0.95 的 50 单测锁死，零回归）
- `nx_frontier_planner.py` 只增 `adjacent_wall_count` 字段，不改现有接口
- 不碰 `/cmd_vel` / nav2 DWB controller（第 4 点分拆到 `nav2-controller-forward-bias` spec）
- env 全大写 `GO2W_FRONTIER_*` 前缀
- 所有改动只在 `utility_mode="mixed"` 路径内生效；nearest 路径不触发 yaw 优化、不算 wall_bonus
- 部署层启用：`GO2W_FRONTIER_UTILITY_MODE=mixed`（代码默认仍 nearest）

## File Structure

| 文件 | 责任 | 改动性质 |
|---|---|---|
| `nx_frontier_planner.py` | frontier 提取 | 增字段 `adjacent_wall_count`（向后兼容） |
| `nx_visibility_coverage.py` | 视锥覆盖 + 候选打分 | 增 public `visual_gain_at()`；`rank_candidates` 不改行为 |
| `nx_exploration_manager.py` | 持久策略 + mixed 打分 | 新参数 + `_mixed_utility_sort_key` 改时间归一化 + 新 `_optimize_yaw_for_candidates` |
| `nx_room_orchestrator.py` | 状态机 | 构造时注入 `max_vel_x` / `max_vel_theta` |
| `test_exploration_manager.py` | 单测 | +8 测试（含底座 adjacent_wall_count） |
| `tools/sim_strategy_compare.py` | 离线策略对比 | +v3 列 + 网格搜索 + `total_turn_rad` |

---

## Task 1: frontier_planner 输出 adjacent_wall_count

**Files:**
- Modify: `go2w_search_ws/web/nx_frontier_planner.py`（`find_frontier_clusters` 内 support_radius 扫描段）
- Test: `go2w_search_ws/web/test_exploration_manager.py`

**Interfaces:**
- Consumes: 无（底座）
- Produces: `find_frontier_clusters` 返回的每个 candidate dict 增加 `"adjacent_wall_count": int`（candidate 周围 support_radius 内 occupied cell 数）

- [ ] **Step 1: 写失败测试**

在 `test_exploration_manager.py` 末尾加：

```python
def test_find_frontier_clusters_reports_adjacent_wall_count():
    """墙角 frontier 的 adjacent_wall_count >= 1；朝开阔区的可能 == 0。

    10m x 10m 房间, 四周墙, robot 在 (5,5)。靠墙的 frontier 邻接 occupied。
    """
    from nx_frontier_planner import find_frontier_clusters
    resolution = 0.5
    width = height = 20
    data = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if row in {0, height - 1} or col in {0, width - 1}:
                data[row * width + col] = 100  # 墙
    data[10 * width + 14] = -1  # (7.0, 5.0) 附近 unknown 触发 frontier
    map_msg = type("M", (), {
        "info": type("I", (), {
            "resolution": resolution, "width": width, "height": height,
            "origin": type("O", (), {
                "position": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0}),
                "orientation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
            }),
        })(),
        "data": data,
    })()
    clusters = find_frontier_clusters(map_msg, (5.0, 5.0, 0.0), [], min_cluster_size=1)
    assert clusters, "应至少有一个 frontier cluster"
    assert all("adjacent_wall_count" in c for c in clusters), \
        "每个 cluster 必须带 adjacent_wall_count 字段"
    assert any(c["adjacent_wall_count"] >= 1 for c in clusters), \
        "靠墙 frontier 的 adjacent_wall_count 应 >= 1"
```

- [ ] **Step 2: 运行验证失败**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_find_frontier_clusters_reports_adjacent_wall_count -v
```
Expected: FAIL — `KeyError: 'adjacent_wall_count'`

- [ ] **Step 3: 实现**

在 `find_frontier_clusters` 的 support_count 循环里加 occupied 邻居计数（复用同一循环边界）：

```python
            support_count = 0
            wall_neighbor_count = 0  # 新增
            touches_map_edge = False
            obstacle_threshold = 50
            for row in range(
                    max(0, rep_row - support_radius_cells),
                    min(height, rep_row + support_radius_cells + 1)):
                dr_sq = (row - rep_row) ** 2
                for col in range(
                        max(0, rep_col - support_radius_cells),
                        min(width, rep_col + support_radius_cells + 1)):
                    if dr_sq + (col - rep_col) ** 2 > support_radius_sq:
                        continue
                    cell_value = data[row * width + col]
                    if cell_value >= obstacle_threshold:
                        wall_neighbor_count += 1  # 新增
                    if not component_mask[row * width + col]:
                        continue
                    support_count += 1
                    touches_map_edge = bool(
                        touches_map_edge
                        or row <= 1 or col <= 1
                        or row >= height - 2 or col >= width - 2)
            selected_cells.append(representative)
            result.append({
                "center_cell": representative,
                "center_world": (world_x, world_y),
                "size": support_count,
                "cluster_size": len(component),
                "information_gain": float(support_count),
                "distance": math.hypot(world_x - robot_x, world_y - robot_y),
                "touches_map_edge": touches_map_edge,
                "adjacent_wall_count": wall_neighbor_count,  # 新增
            })
```

- [ ] **Step 4: 运行验证通过**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_find_frontier_clusters_reports_adjacent_wall_count -v
```
Expected: PASS

- [ ] **Step 5: 回归 + commit**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py -q
```
Expected: 51 passed

```bash
git add go2w_search_ws/web/nx_frontier_planner.py go2w_search_ws/web/test_exploration_manager.py
git commit -m "feat(frontier-planner): emit adjacent_wall_count for wall-proximity scoring"
```

---

## Task 2: VisibilityCoverageTracker 暴露 visual_gain_at()

**Files:**
- Modify: `go2w_search_ws/web/nx_visibility_coverage.py`（`VisibilityCoverageTracker` 类）
- Test: `go2w_search_ws/web/test_exploration_manager.py`

**Interfaces:**
- Consumes: 无（现有 `_visible_buckets` + `_observed`）
- Produces: `tracker.visual_gain_at(map_msg, x, y, yaw) -> int`

- [ ] **Step 1: 写失败测试**

```python
def test_visual_gain_at_counts_unobserved_cells_for_yaw():
    """同一 (x,y)，不同 yaw 的 visual_gain > 0（朝向敏感）。"""
    from nx_visibility_coverage import VisibilityCoverageTracker
    resolution = 0.5
    width = height = 20
    data = [-1] * (width * height)
    for dr in range(-2, 3):
        for dc in range(-2, 3):
            data[(10 + dr) * width + (10 + dc)] = 0
    map_msg = type("M", (), {
        "info": type("I", (), {
            "resolution": resolution, "width": width, "height": height,
            "origin": type("O", (), {
                "position": type("P", (), {"x": 0.0, "y": 0.0, "z": 0.0}),
                "orientation": type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
            }),
        })(),
        "data": data,
    })()
    tracker = VisibilityCoverageTracker(camera_hfov_rad=1.2, visual_range_m=5.0)
    gain_east = tracker.visual_gain_at(map_msg, 5.0, 5.0, 0.0)
    gain_west = tracker.visual_gain_at(map_msg, 5.0, 5.0, math.pi)
    assert gain_east > 0, "朝东应扫到未观测 cell"
    assert gain_west > 0, "朝西应扫到未观测 cell"
```

- [ ] **Step 2: 运行验证失败**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_visual_gain_at_counts_unobserved_cells_for_yaw -v
```
Expected: FAIL — `AttributeError: ... has no attribute 'visual_gain_at'`

- [ ] **Step 3: 实现**

在 `VisibilityCoverageTracker` 类（`rank_candidates` 之后）加：

```python
    def visual_gain_at(
        self, map_msg: Any, x: float, y: float, yaw: float,
    ) -> int:
        """从 (x,y,yaw) 出发的视锥 frustum 内, 尚不在 _observed 的 bucket 数。

        供 ExplorationManager 的 yaw 优化调用。线程安全 (RLock)。
        """
        with self._lock:
            try:
                grid = _Grid(map_msg)
            except ValueError:
                return 0
            visible = self._visible_buckets(grid, (float(x), float(y), float(yaw)), None)
            return len(visible.difference(self._observed))
```

- [ ] **Step 4: 运行验证通过**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_visual_gain_at_counts_unobserved_cells_for_yaw -v
```
Expected: PASS

- [ ] **Step 5: commit**

```bash
git add go2w_search_ws/web/nx_visibility_coverage.py go2w_search_ws/web/test_exploration_manager.py
git commit -m "feat(visibility): expose visual_gain_at() for yaw optimization"
```

---

## Task 3: ExplorationManager 新参数 + mixed_utility 时间归一化

**Files:**
- Modify: `go2w_search_ws/web/nx_exploration_manager.py`（`__init__` + `_mixed_utility_sort_key` + `snapshot`）
- Test: `go2w_search_ws/web/test_exploration_manager.py`

**Interfaces:**
- Consumes: candidate 的 `information_gain` / `visual_gain` / `wall_proximity_bonus` / `path_length` / `distance` / `heading_change`
- Produces: `ExplorationManager(..., mixed_heading_penalty=, mixed_wall_bonus=, yaw_step_deg=, max_vel_x=, max_vel_theta=)`

- [ ] **Step 1: 写失败测试**

```python
def test_mixed_time_normalization_respects_max_vel_theta():
    """max_vel_theta 调小 → heading 惩罚变大 → 选不转身。"""
    candidates = [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": 0.0, "wall_proximity_bonus": 0.0},
        {"x": 0.0, "y": 3.0, "yaw": math.pi / 2, "size": 10, "center_cell": (6, 0),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": math.pi / 2, "wall_proximity_bonus": 0.0},
    ]
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in candidates],
        reject_map_edge=False, utility_mode="mixed",
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=10.0,
        max_vel_x=1.0, max_vel_theta=0.1,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["heading_change"] == pytest.approx(0.0, abs=1e-6)
    assert m.snapshot()["mixed_heading_penalty"] == 10.0
```

- [ ] **Step 2: 运行验证失败**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_mixed_time_normalization_respects_max_vel_theta -v
```
Expected: FAIL — `TypeError: unexpected keyword argument 'mixed_heading_penalty'`

- [ ] **Step 3: 实现**

(a) `__init__` 签名加参数（在 `mixed_path_cost_penalty` 之后）：

```python
        mixed_path_cost_penalty: float = 0.5,
        # v3 (2026-07-21): heading 时间归一化 + wall_bonus + yaw 优化
        mixed_heading_penalty: float = 0.0,
        mixed_wall_bonus: float = 0.0,
        yaw_step_deg: float = 45.0,
        max_vel_x: float = 1.5,
        max_vel_theta: float = 1.0,
```

(b) `__init__` body 加 env override（在 `self.mixed_path_cost_penalty = ...` 之后）：

```python
        self.mixed_path_cost_penalty = max(0.0, float(mixed_path_cost_penalty))
        self.mixed_heading_penalty = max(0.0, float(os.environ.get(
            "GO2W_FRONTIER_TIME_PENALTY", str(mixed_heading_penalty))))
        self.mixed_wall_bonus = max(0.0, float(os.environ.get(
            "GO2W_FRONTIER_MIXED_WALL_BONUS", str(mixed_wall_bonus))))
        self.yaw_step_deg = max(5.0, float(os.environ.get(
            "GO2W_FRONTIER_YAW_STEP_DEG", str(yaw_step_deg))))
        self.max_vel_x = max(0.1, float(os.environ.get(
            "GO2W_FRONTIER_MAX_VEL_X", str(max_vel_x))))
        self.max_vel_theta = max(0.05, float(os.environ.get(
            "GO2W_FRONTIER_MAX_VEL_THETA", str(max_vel_theta))))
```

(c) 替换 `_mixed_utility_sort_key`：

```python
    def _mixed_utility_sort_key(self, candidate: dict, robot_pose) -> tuple:
        """v3: α·size + β·visual_gain + δ·wall − k_time·(t_travel+t_turn)。

        path_cost 和 heading 都换算成秒 (时间归一化)。
        """
        try:
            information_gain = float(candidate.get(
                "information_gain", candidate.get("size", 0.0)))
        except (TypeError, ValueError, OverflowError):
            information_gain = 0.0
        try:
            visual_gain = float(candidate.get("visual_gain", 0.0))
        except (TypeError, ValueError, OverflowError):
            visual_gain = 0.0
        try:
            wall_bonus = float(candidate.get("wall_proximity_bonus", 0.0))
        except (TypeError, ValueError, OverflowError):
            wall_bonus = 0.0
        path_cost = self._path_cost_for_utility(candidate, robot_pose)
        try:
            heading_change = abs(float(candidate.get("heading_change", 0.0)))
        except (TypeError, ValueError, OverflowError):
            heading_change = 0.0
        t_travel = path_cost / max(self.max_vel_x, 1e-6)
        t_turn = heading_change / max(self.max_vel_theta, 1e-6)
        utility = (
            self.mixed_frontier_weight * information_gain
            + self.mixed_visual_gain_weight * visual_gain
            + self.mixed_wall_bonus * wall_bonus
            - self.mixed_heading_penalty * (t_travel + t_turn)
        )
        return (-utility,)
```

(d) `snapshot` 加字段（在 `"mixed_path_cost_penalty"` 之后）：

```python
            "mixed_path_cost_penalty": self.mixed_path_cost_penalty,
            "mixed_heading_penalty": self.mixed_heading_penalty,
            "mixed_wall_bonus": self.mixed_wall_bonus,
            "yaw_step_deg": self.yaw_step_deg,
            "max_vel_x": self.max_vel_x,
            "max_vel_theta": self.max_vel_theta,
```

- [ ] **Step 4: 运行验证通过**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_mixed_time_normalization_respects_max_vel_theta -v
```
Expected: PASS

- [ ] **Step 5: 回归（v0.95 mixed 测试不破）+ commit**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py -q
```
Expected: 52 passed（v0.95 mixed 测试在 `mixed_heading_penalty=0.0` 默认退化，仍绿）

```bash
git add go2w_search_ws/web/nx_exploration_manager.py go2w_search_ws/web/test_exploration_manager.py
git commit -m "feat(frontier-v3): mixed-utility 时间归一化 + wall_bonus 参数"
```

---

## Task 4: ExplorationManager._optimize_yaw_for_candidates（yaw 全360°优化）

**Files:**
- Modify: `go2w_search_ws/web/nx_exploration_manager.py`（新增方法 + `choose_next` 调用点）
- Test: `go2w_search_ws/web/test_exploration_manager.py`（4 个新测试）

**Interfaces:**
- Consumes: `tracker.visual_gain_at()`（Task 2）；`candidate.adjacent_wall_count`（Task 1）；`yaw_step_deg` / `mixed_*`（Task 3）
- Produces: candidate 的 `yaw` / `visual_gain` / `heading_change` / `wall_proximity_bonus` 更新为最优 yaw 的值

- [ ] **Step 1: 写失败测试（4 个）**

```python
def _corner_vs_open_candidates():
    return [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": 0.0, "adjacent_wall_count": 0},
        {"x": 0.0, "y": -3.0, "yaw": -math.pi / 2, "size": 10, "center_cell": (-6, 0),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": math.pi / 2, "adjacent_wall_count": 3},
    ]


def test_mixed_wall_bonus_prefers_corner_frontier():
    cands = _corner_vs_open_candidates()
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False, utility_mode="mixed",
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=0.0, mixed_wall_bonus=10.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["adjacent_wall_count"] == 3


def test_mixed_heading_prefers_no_turn_when_visual_gain_close():
    cands = [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": 0.0, "adjacent_wall_count": 0},
        {"x": 0.0, "y": 3.0, "yaw": math.pi / 2, "size": 10, "center_cell": (6, 0),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 5.0,
         "heading_change": math.pi / 2, "adjacent_wall_count": 0},
    ]
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False, utility_mode="mixed",
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=5.0, max_vel_x=1.0, max_vel_theta=1.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["heading_change"] == pytest.approx(0.0, abs=1e-6)


def test_mixed_yaw_optimization_can_select_180_when_front_blocked():
    cands = [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 0.0,
         "heading_change": 0.0, "adjacent_wall_count": 0},
        {"x": -3.0, "y": 0.0, "yaw": math.pi, "size": 10, "center_cell": (0, -30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 20.0,
         "heading_change": math.pi, "adjacent_wall_count": 0},
    ]
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False, utility_mode="mixed",
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=1.0, max_vel_x=1.0, max_vel_theta=1.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["x"] == pytest.approx(-3.0)


class _StubTracker:
    """记录 (x,y,yaw) → visual_gain 的 stub."""
    def __init__(self, gain_map):
        self._gain_map = gain_map
        self._observed = set()
    def visual_gain_at(self, map_msg, x, y, yaw):
        return self._gain_map.get((round(x, 1), round(y, 1), round(yaw, 2)), 0)
    def rank_candidates(self, map_msg, robot_pose, candidates):
        return list(candidates)
    def observe(self, map_msg, robot_pose, scan):
        return {}
    def snapshot(self, map_msg=None):
        return {}


def test_mixed_yaw_optimization_picks_visual_gain_yaw():
    cands = [
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 10, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 10.0, "visual_gain": 0.0,
         "heading_change": 0.0, "adjacent_wall_count": 0},
    ]
    tracker = _StubTracker({
        (3.0, 0.0, 0.0): 0,
        (3.0, 0.0, round(math.pi / 3, 2)): 40,
    })
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False, utility_mode="mixed",
        visibility_tracker=tracker,
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=1.0, max_vel_x=1.0, max_vel_theta=1.0,
        yaw_step_deg=60.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["visual_gain"] == 40
    assert selected["yaw"] == pytest.approx(math.pi / 3, abs=1e-6)
```

- [ ] **Step 2: 运行验证失败**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_mixed_wall_bonus_prefers_corner_frontier test_exploration_manager.py::test_mixed_heading_prefers_no_turn_when_visual_gain_close test_exploration_manager.py::test_mixed_yaw_optimization_can_select_180_when_front_blocked test_exploration_manager.py::test_mixed_yaw_optimization_picks_visual_gain_yaw -v
```
Expected: 4 FAIL

- [ ] **Step 3: 实现**

(a) 模块级辅助函数（文件顶部 import 之后）：

```python
def _abs_angle_delta(a: float, b: float) -> float:
    """abs 角度差, 结果在 [0, π]。"""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))
```

(b) 新增方法（`_mixed_utility_sort_key` 之后）：

```python
    def _optimize_yaw_for_candidates(
            self, candidates: list, robot_pose, map_msg) -> list:
        """v3 yaw 优化: 每个 candidate 试全360° K 个 yaw, 选 mixed-utility 最优.

        仅 mixed 模式 + visibility_tracker 存在时调用. 否则只填 wall_proximity_bonus.
        候选集: robot_yaw + ±k·yaw_step (覆盖全360°含 90/180) + 朝frontier方向.
        不硬排除大角度 — 靠 k_time·t_turn 加权偏好小角度, 前方受阻时 180° 自然胜出.
        """
        try:
            robot_yaw = float(robot_pose[2])
        except (TypeError, IndexError, ValueError):
            robot_yaw = 0.0
        try:
            rx = float(robot_pose[0]); ry = float(robot_pose[1])
        except (TypeError, IndexError, ValueError):
            rx = ry = 0.0
        for cand in candidates:
            awc = int(cand.get("adjacent_wall_count", 0))
            wall_bonus = 1.0 if awc >= 2 else 0.0
            if (self.utility_mode != "mixed"
                    or self.visibility_tracker is None):
                cand["wall_proximity_bonus"] = wall_bonus
                continue
            try:
                cx = float(cand["x"]); cy = float(cand["y"])
            except (KeyError, TypeError, ValueError):
                cand["wall_proximity_bonus"] = wall_bonus
                continue
            frontier_yaw = math.atan2(cy - ry, cx - rx)
            step = math.radians(max(5.0, float(self.yaw_step_deg)))
            yaw_offsets = [0.0]
            k = 1
            while k * step < math.pi - 1e-9:
                yaw_offsets.append(k * step)
                yaw_offsets.append(-k * step)
                k += 1
            yaw_offsets.append(math.pi)
            yaw_offsets.append(_abs_angle_delta(frontier_yaw, robot_yaw)
                               * (1.0 if frontier_yaw >= robot_yaw else -1.0))
            path_cost = self._distance_from_pose(cand, robot_pose)
            t_travel = path_cost / max(self.max_vel_x, 1e-6)
            try:
                base_ig = float(cand.get(
                    "information_gain", cand.get("size", 0.0)))
            except (TypeError, ValueError, OverflowError):
                base_ig = 0.0
            best = None  # (key_tuple, yaw, vg, hc)
            for offset in set(yaw_offsets):
                yaw = robot_yaw + offset
                vg = self.visibility_tracker.visual_gain_at(map_msg, cx, cy, yaw)
                hc = _abs_angle_delta(yaw, robot_yaw)
                t_turn = hc / max(self.max_vel_theta, 1e-6)
                utility = (
                    self.mixed_frontier_weight * base_ig
                    + self.mixed_visual_gain_weight * float(vg)
                    + self.mixed_wall_bonus * wall_bonus
                    - self.mixed_heading_penalty * (t_travel + t_turn)
                )
                key = (utility, -hc, -vg)
                if best is None or key > best[0]:
                    best = (key, yaw, vg, hc)
            if best is not None:
                _, yaw, vg, hc = best
                cand["yaw"] = yaw
                cand["visual_gain"] = vg
                cand["heading_change"] = hc
            cand["wall_proximity_bonus"] = wall_bonus
        return candidates
```

(c) `choose_next` 里调用（在 `candidates = self._select_candidates(...)` 每次之后，半径扩张循环内）：

```python
            candidates = self._select_candidates(
                candidate_selector, map_msg, robot_pose)
            # v3 yaw 优化 (仅 mixed + tracker 时生效)
            candidates = self._optimize_yaw_for_candidates(
                candidates, robot_pose, map_msg)
```

- [ ] **Step 4: 运行验证通过**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_mixed_wall_bonus_prefers_corner_frontier test_exploration_manager.py::test_mixed_heading_prefers_no_turn_when_visual_gain_close test_exploration_manager.py::test_mixed_yaw_optimization_can_select_180_when_front_blocked test_exploration_manager.py::test_mixed_yaw_optimization_picks_visual_gain_yaw -v
```
Expected: 4 PASS

- [ ] **Step 5: 回归 + commit**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py -q
```
Expected: 56 passed（52 + 4）

```bash
git add go2w_search_ws/web/nx_exploration_manager.py go2w_search_ws/web/test_exploration_manager.py
git commit -m "feat(frontier-v3): yaw 全360°优化 (K≤10, 含90/180, 加权不排除)"
```

---

## Task 5: nearest 零回归 + parallel probe 兼容

**Files:**
- Test: `go2w_search_ws/web/test_exploration_manager.py`（2 个新测试）

**Interfaces:**
- Consumes: Task 1-4 全部产出
- Produces: 回归保证

- [ ] **Step 1: 写测试**

```python
def test_nearest_mode_unchanged_by_v3():
    """nearest 模式: 选最近, 不受 wall_bonus 影响。"""
    cands = [
        {"x": 5.0, "y": 0.0, "yaw": 0.0, "size": 100, "center_cell": (0, 50),
         "distance": 5.0, "information_gain": 100.0, "adjacent_wall_count": 3},
        {"x": 1.5, "y": 0.0, "yaw": 0.0, "size": 5, "center_cell": (0, 15),
         "distance": 1.5, "information_gain": 5.0, "adjacent_wall_count": 0},
    ]
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="current_room", room_radius_m=8.0, initial_radius_m=8.0, tile_size_m=16.0,
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False,
        mixed_wall_bonus=10.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert selected["x"] == pytest.approx(1.5)


def test_mixed_yaw_optimization_parallel_probe_compatible():
    cands = [
        {"x": 1.5, "y": 0.0, "yaw": 0.0, "size": 5, "center_cell": (0, 15),
         "distance": 1.5, "information_gain": 5.0, "adjacent_wall_count": 0},
        {"x": 3.0, "y": 0.0, "yaw": 0.0, "size": 8, "center_cell": (0, 30),
         "distance": 3.0, "information_gain": 8.0, "adjacent_wall_count": 0},
    ]
    m = ExplorationManager(
        navigation_port=_PlannerPort(), mission_origin=(0.0, 0.0, 0.0),
        mode="whole_floor",
        candidate_selector=lambda *_a, **_k: [dict(c) for c in cands],
        reject_map_edge=False, utility_mode="mixed",
        parallel_probe_workers=2,
        mixed_frontier_weight=0.5, mixed_visual_gain_weight=1.0,
        mixed_heading_penalty=0.5, max_vel_x=1.0, max_vel_theta=1.0,
    )
    selected = m.choose_next(_two_room_map(), (0.0, 0.0, 0.0))
    assert selected is not None
    assert m.snapshot()["parallel_probe_workers"] == 2
```

- [ ] **Step 2: 运行验证**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py::test_nearest_mode_unchanged_by_v3 test_exploration_manager.py::test_mixed_yaw_optimization_parallel_probe_compatible -v
```
Expected: 2 PASS（若 FAIL，Task 4 的 guard 漏了 → 修）

- [ ] **Step 3: 全量回归**

```bash
cd go2w_search_ws/web && python -m pytest test_exploration_manager.py -q
```
Expected: 58 passed

- [ ] **Step 4: commit**

```bash
git add go2w_search_ws/web/test_exploration_manager.py
git commit -m "test(frontier-v3): nearest 零回归 + parallel probe 兼容"
```

---

## Task 6: orchestrator 注入 max_vel_x / max_vel_theta

**Files:**
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`（构造 `ExplorationManager` 处）

**Interfaces:**
- Consumes: env `GO2W_FRONTIER_MAX_VEL_X` / `GO2W_FRONTIER_MAX_VEL_THETA`（或默认 1.5/1.0）
- Produces: ExplorationManager 拿到真实速度上限

- [ ] **Step 1: 定位构造点**

```bash
cd go2w_search_ws/web && grep -n "ExplorationManager(" nx_room_orchestrator.py
```

- [ ] **Step 2: 构造时传速度参数**

在构造 `ExplorationManager(...)` 调用里加：

```python
            max_vel_x=float(os.environ.get("GO2W_FRONTIER_MAX_VEL_X", "1.5")),
            max_vel_theta=float(os.environ.get("GO2W_FRONTIER_MAX_VEL_THETA", "1.0")),
```

（env 已在 ExplorationManager `__init__` 内 override，这里显式传只是冗余保险；也可省略让 manager 自己读 env。）

- [ ] **Step 3: commit**

```bash
git add go2w_search_ws/web/nx_room_orchestrator.py
git commit -m "feat(frontier-v3): orchestrator 注入 max_vel 用于时间归一化"
```

---

## Task 7: sim_strategy_compare 加 v3 列 + 网格搜索 + total_turn_rad

**Files:**
- Modify: `go2w_search_ws/tools/sim_strategy_compare.py`
- Run: `python go2w_search_ws/tools/sim_strategy_compare.py`

- [ ] **Step 1: 加 _SimTracker（sim 专用 visual_gain_at）**

文件顶部（`_KnownFreePlanner` 之后）加：

```python
class _SimTracker:
    """Sim-only: visual_gain_at = 从 (x,y,yaw) raycast truth grid,
    返回 observed 还没揭示的 free cell 数。"""
    def __init__(self, truth, observed, width, height, resolution,
                 hfov=1.2, vrange=5.0):
        self.truth = truth; self.observed = observed
        self.width = width; self.height = height; self.resolution = resolution
        self.hfov = hfov; self.vrange = vrange
        self._observed = set()
    def visual_gain_at(self, map_msg, x, y, yaw):
        step = self.resolution * 0.5
        gain = 0
        seen = set()
        ray_count = max(3, int(math.ceil(self.hfov / math.radians(2.0))) + 1)
        for k in range(ray_count):
            offset = -self.hfov / 2 + self.hfov * k / max(1, ray_count - 1)
            angle = yaw + offset
            d = 0.0
            while d <= self.vrange:
                wx = x + d * math.cos(angle); wy = y + d * math.sin(angle)
                col = int(math.floor(wx / self.resolution))
                row = int(math.floor(wy / self.resolution))
                if not (0 <= row < self.height and 0 <= col < self.width):
                    break
                idx = row * self.width + col
                if self.truth[idx] >= 50:
                    break
                if self.observed[idx] == -1 and idx not in seen:
                    gain += 1; seen.add(idx)
                d += step
        return gain
    def observe(self, *a, **k): return {}
    def snapshot(self, *a, **k): return {}
    def rank_candidates(self, m, p, c): return list(c)
```

- [ ] **Step 2: _run_scenario 注入 tracker + total_turn_rad**

在 `_run_scenario` 里 `manager = ExplorationManager(**kwargs)` 前加：

```python
    sim_tracker = _SimTracker(truth, observed, width, height, resolution)
    kwargs["visibility_tracker"] = sim_tracker
```

在循环里累计转身（`goals.append(...)` 之后）：

```python
        target_yaw = float(target.get("yaw", 0.0))
        total_turn += abs(math.atan2(
            math.sin(target_yaw - pose[2]),
            math.cos(target_yaw - pose[2])))
```

`total_turn = 0.0` 在循环前初始化；return dict 加 `"total_turn_rad": round(total_turn, 2)`。

- [ ] **Step 3: main() 加 v3 场景 + 网格搜索**

results 列表加 v3：

```python
        _run_scenario(
            "v3",
            utility_mode="mixed", parallel_workers=0,
            mixed_weights={
                "mixed_frontier_weight": 0.5,
                "mixed_visual_gain_weight": 1.0,
                "mixed_heading_penalty": 1.0,
                "mixed_wall_bonus": 2.0,
                "yaw_step_deg": 45.0,
                "max_vel_x": 1.5, "max_vel_theta": 1.0,
            },
        ),
```

hypothesis 检查后加网格搜索：

```python
    print("\n=== v3 权重网格搜索 (k_time × δ) ===")
    grid_results = []
    for k_time in [0.5, 1.0, 2.0, 5.0]:
        for delta in [0.0, 1.0, 2.0, 5.0]:
            r = _run_scenario(
                f"k={k_time},δ={delta}",
                utility_mode="mixed", parallel_workers=0,
                mixed_weights={
                    "mixed_frontier_weight": 0.5,
                    "mixed_visual_gain_weight": 1.0,
                    "mixed_heading_penalty": k_time,
                    "mixed_wall_bonus": delta,
                    "yaw_step_deg": 45.0,
                })
            grid_results.append((k_time, delta, r))
    base_cov = results[0]["coverage_pct"]
    candidates_ok = [(k, d, r) for k, d, r in grid_results
                     if r["coverage_pct"] >= base_cov - 1.0]
    if candidates_ok:
        best = min(candidates_ok, key=lambda t: t[2]["total_turn_rad"])
        print(f"  推荐: k_time={best[0]}, δ={best[1]}, "
              f"turn={best[2]['total_turn_rad']}rad, cov={best[2]['coverage_pct']}%")
    else:
        print("  WARN: 所有 (k_time,δ) 组合 coverage 退步 > 1%, 需调参")
```

`_print_table` headers 加 `"turn_rad"`；rows 加 `r["total_turn_rad"]`。

- [ ] **Step 4: 运行 sim**

```bash
python go2w_search_ws/tools/sim_strategy_compare.py
```
Expected: 表含 v3 列 + turn_rad + 网格搜索推荐。H1（v3 cov ≥ nearest - 1%）+ `total_turn_rad(v3) < nearest`。

- [ ] **Step 5: commit**

```bash
git add go2w_search_ws/tools/sim_strategy_compare.py
git commit -m "feat(frontier-v3): sim v3 列 + (k_time,δ) 网格搜索 + total_turn_rad"
```

---

## Task 8: 权重标定 + 部署 env + tag

**Files:**
- Modify: `go2w_search_ws/docker/bringup_slam_nav2.sh`

- [ ] **Step 1: 跑 sim 标定**

```bash
python go2w_search_ws/tools/sim_strategy_compare.py 2>&1 | tail -25
```
记录推荐的 `k_time` / `δ`。

- [ ] **Step 2: deploy env 加 v3 启用**

`bringup_slam_nav2.sh` 的 export 段加：

```bash
# frontier v3 (2026-07-21)
export GO2W_FRONTIER_UTILITY_MODE=mixed
export GO2W_FRONTIER_TIME_PENALTY=1.0       # k_time, sim 标定后替换
export GO2W_FRONTIER_MIXED_WALL_BONUS=2.0   # δ, sim 标定后替换
export GO2W_FRONTIER_YAW_STEP_DEG=45
export GO2W_FRONTIER_MAX_VEL_X=1.5
export GO2W_FRONTIER_MAX_VEL_THETA=1.0
```

- [ ] **Step 3: 实机验证（NX 在线）**

部署到 NX，同房间 nearest vs v3：`bounded_explored_ratio ≥ 0.90` + `total_turn_rad` 明显下降 + 无卡死。

- [ ] **Step 4: commit + tag**

```bash
git add go2w_search_ws/docker/bringup_slam_nav2.sh
git commit -m "chore(frontier-v3): deploy env 启用 mixed + sim 标定权重"
git tag -a v0.96 -m "v0.96: frontier v3 — 边界感知 + yaw 优化 + 时间归一化"
```

---

## Self-Review（plan 自审）

**1. Spec 覆盖**：
- 第 1 点 wall_proximity → Task 1 + Task 3（mixed_wall_bonus）+ Task 4（wall_proximity_bonus 填充）✓
- 第 2 点 yaw 优化 → Task 2（visual_gain_at）+ Task 4（_optimize_yaw_for_candidates，全360°含90/180）✓
- 第 3 点 heading 时间归一化 → Task 3（t_travel+t_turn）✓
- 第 4 点 → 明确分拆，本 plan 不含 ✓
- 7 测试 + 底座 = Task 1(1)+Task 3(1)+Task 4(4)+Task 5(2) = 8 ✓
- sim v3 + 网格搜索 → Task 7 ✓
- nearest 零回归 → Task 5 ✓
- 部署 → Task 8 ✓

**2. Placeholder 扫描**：无 TBD/TODO；Task 8 的"sim 标定后替换"给了初始值 1.0/2.0 可跑。✓

**3. 类型一致性**：`adjacent_wall_count`（Task 1）→ Task 4 读 ✓；`visual_gain_at`（Task 2）→ Task 4 + Task 7 sim ✓；`mixed_heading_penalty`/`mixed_wall_bonus`/`yaw_step_deg`/`max_vel_*`（Task 3）→ Task 4/6/8 一致 ✓。

**4. 已知风险**：
- Task 4 yaw 优化 K≤10 × N≤32 ≤ 320 raycast，sim 实测后若 >500ms 考虑加候选缓存。
- Task 7 `_SimTracker.visual_gain_at` 是简化 raycast（与生产 `_visible_buckets` 不完全一致），sim 验证趋势，生产精度以实机为准。

## Execution Handoff

**用户已指定实现走 ecc 流程。** 计划保存于 `docs/superpowers/plans/2026-07-21-frontier-v3-boundary-yaw-heading.md`。

ecc 执行路径：
- `ecc:prp-implement` 或 `ecc:feature-dev`：按 Task 1-8 顺序 TDD（写测试→失败→实现→通过→commit）
- Task 7 跑 sim 标定 `(k_time, δ)`
- Task 8 部署 env + 实机验证 + tag v0.96

superpowers 备选（若 ecc 不可用）：`superpowers:subagent-driven-development`（每 task fresh subagent + 两阶段 review）或 `superpowers:executing-plans`（本 session 批量 + checkpoint）。
