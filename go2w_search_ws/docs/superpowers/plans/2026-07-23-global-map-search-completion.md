# Global Map Search Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让“搜索房间/搜索整层”以持久化全局地图为权威，把任务起点处的入口门作为唯一排除边，穿过其他所有安全可通行的开口继续探索，并且仅在没有其他可通行开口、已探索或可证明被障碍物遮挡的内部区域达到 95%、连续多帧无可达 frontier 时完成。

**Architecture:** 保留 `/map_frontier`（SLAM OccupancyGrid）作为全局拓扑，保留 `VisibilityCoverageTracker` 作为 C13 视锥证据，新建纯函数全局搜索状态分析层，把地图单元统一分类为 `reachable_free`、`observed_free`、`occupied_boundary`、`reachable_unknown_frontier`、`traversable_narrow_opening`、`enclosed_occluded_unknown`、`exterior_unknown`。墙/障碍直接来自占用栅格，不要求语义门识别。任务开始时以 `mission_origin + 初始朝向` 建立有限宽度的入口虚拟门线，只禁止再次跨越该门线到狗身后；其他门洞必须保持开放并被探索。常规 Nav2 因保守圆形包络拒绝窄门时，使用门前对齐、实时 MID360 侧向/前向净空保护的低速有界穿越流程，穿过后恢复普通 frontier/Nav2。

**Tech Stack:** Python 3、ROS 2 `nav_msgs/OccupancyGrid` 数据模型、现有 Nav2 规划端口、pytest、现有 HTML/JavaScript 地图状态渲染。

---

## Completion contract

完成必须同时满足：

1. `/map_frontier` 有效且任务起点能落到膨胀后的可达 free-space；
2. 除入口虚拟门线外，`traversable_opening_count == 0`；边界不使用百分比容忍门洞；
3. 不存在可达的 unknown frontier，也不存在“几何可通行但尚未成功穿越”的窄通道；
4. `explainable_coverage_ratio >= 0.95`，95% 只用于内部覆盖，其中分子只允许 `observed reachable free + occupied boundary + certified enclosed occluded unknown`；
5. 同一搜索域及上述条件连续 3 个新地图修订成立；
6. 任何地图/位姿/视锥证据缺失均返回未验证，不能宣告完成。

`enclosed_occluded_unknown` 只包括：不接触 ROI/地图边缘、不邻接机器人可达 free-space、被 occupied boundary 包围的 unknown 连通块。一次视线被挡住不构成“可证明遮挡”。入口门后的 unknown 记录为 `entrance_excluded`，既不进入覆盖分母，也不成为 frontier；任何其他安全宽度的开口都不得被归入遮挡。

## Task 1: Lock the global search-state contract with failing tests

**Files:**
- Create: `web/test_global_search_state.py`
- Reference: `web/test_coverage_metrics.py`
- Reference: `web/test_unknown_room_exploration_sim.py`

- [ ] Add a grid-fixture builder for rotated origins, padded unknown borders, walls, door gaps, internal obstacles and observed C13 buckets.
- [ ] Add a closed rectangular room test: all reachable free observed, boundary closed, no frontier, both ratios are 1.0, completion eligible.
- [ ] Add an open-wall/door-gap test: unknown frontier survives, `traversable_opening_count > 0`, and completion is false even when visual coverage is 100%.
- [ ] Add an entrance-gate test: a finite virtual gate at `mission_origin`, perpendicular to initial yaw, excludes only the rear crossing while an identical opening elsewhere remains a frontier.
- [ ] Add a wrong-side regression test: candidates and paths that cross the entrance gate toward the rear are rejected, while a route reaching the same area through another non-entry opening remains eligible.
- [ ] Add an internal obstacle test: occupied cells are boundary/obstacle evidence, not free visual coverage.
- [ ] Add an enclosed unknown pocket test: it is certified occluded only when it does not touch reachable free or the ROI/map edge.
- [ ] Add a chair-like partial occluder test: unknown behind a single obstacle but reachable from another side remains frontier and cannot count as occluded.
- [ ] Add stale/missing pose, invalid grid and map-padding tests; each must fail closed.
- [ ] Run `python -m pytest -q web/test_global_search_state.py` and confirm the tests fail because the module does not exist.

## Task 2: Implement the pure global map evidence model

**Files:**
- Create: `web/nx_global_search_state.py`
- Test: `web/test_global_search_state.py`
- Reference: `web/nx_frontier_planner.py`
- Reference: `web/nx_coverage_metrics.py`

- [ ] Implement validated OccupancyGrid geometry conversion shared by all calculations, including rotated map origins.
- [ ] Inflate occupied cells by the configured robot clearance and flood-fill only the free component connected to `mission_origin`/current pose.
- [ ] Infer the finite entrance cross-section from the nearest stable occupied support on the left/right of the initial pose, retain initial forward yaw as the search side, and fail closed when the gate cannot be bounded reliably.
- [ ] Make reachable-free flood fill and frontier adjacency treat only the entrance cross-section as a blocked graph edge; do not apply a global rear half-plane mask.
- [ ] Classify reachable-free perimeter samples as occupied wall/obstacle, entrance-excluded edge, normal unknown opening, or traversable narrow opening; expose exact counts and require zero non-entry traversable openings.
- [ ] Find unknown connected components and classify edge-connected components as exterior, reachable-free-adjacent components as frontier, and only fully interior non-adjacent components as certified occluded.
- [ ] Convert `VisibilityCoverageTracker.observed_cells` into map cells and count only observed cells intersecting reachable free-space.
- [ ] Compute `explainable_coverage_ratio` without allowing padded exterior unknown or arbitrary occupied area to inflate the result.
- [ ] Return a structured snapshot with `valid`, `completion_eligible`, all counts, entrance-gate geometry, threshold failures and compact overlay cells for the frontend.
- [ ] Keep the analysis O(number of map cells); add a 1,000 x 1,000 synthetic-grid test with a bounded runtime assertion that tolerates CI variance.
- [ ] Run `python -m pytest -q web/test_global_search_state.py` and confirm all tests pass.

## Task 3: Make stable global evidence the only successful stop gate

**Files:**
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Test: `web/test_exploration_manager.py`
- Test: `web/test_product_room_orchestrator.py`

- [ ] Add failing tests proving `reachable_frontiers_exhausted` alone cannot complete when any non-entry opening remains or internal coverage is below 95%.
- [ ] Add failing tests for three consecutive eligible map revisions and for streak reset when a frontier, opening or coverage regression appears.
- [ ] Cache global analysis by `map_revision + observed_cell_count + active ROI`; do not rescan the grid once per candidate/path probe.
- [ ] Feed the latest global state into `ExplorationManager.choose_next()` after frontier and visual candidate generation.
- [ ] Replace the disabled/unsafe entry-origin `detect_room_enclosure()` success path with the new evidence gate. Keep legacy code unreachable until removed in a later cleanup.
- [ ] If frontiers are exhausted but the global gate is not satisfied, return `global_coverage_pending` and re-observe/replan; stop only on a bounded safety/time budget, reported as incomplete.
- [ ] Change the default authoritative threshold from 0.90 to 0.95 and report the exact configured threshold.
- [ ] Preserve source priority `frontier -> coverage -> lidar`; no coverage fallback may pre-empt a reachable unknown frontier.
- [ ] Run `python -m pytest -q web/test_exploration_manager.py web/test_product_room_orchestrator.py web/test_unknown_room_exploration_sim.py`.

## Task 4: Add geometric narrow-passage traversal without weakening normal safety

**Files:**
- Create: `web/nx_doorway_topology.py`
- Create: `web/test_doorway_topology.py`
- Modify: `web/nx_global_search_state.py`
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_navigation_gateway.py`

- [ ] Add failing fixtures for a 0.8–1.2 m doorway between two rooms, a wide open-plan connection, a corridor, a chair gap, a diagonal doorway, a below-safe-width gap and a temporarily noisy wall.
- [ ] Compute a free-space clearance field and extract stable bottleneck cross-sections with occupied support on opposing sides; cluster adjacent cross-sections and require confirmation across at least 3 distinct map revisions.
- [ ] Classify width below the padded 0.64 m body plus configured margin as physically non-traversable; classify wider openings as exploration obligations even if the normal global planner rejects them.
- [ ] Preserve the normal global planner's orientation-independent 0.60 m radius for ordinary motion. Do not globally shrink the collision envelope to make a door test pass.
- [ ] For a traversable narrow opening, generate a pre-door alignment pose, gate-center heading and bounded post-door target. Reach and rotate at the pre-door pose while normal Nav2 still has clearance.
- [ ] Add a navigation-owner narrow-passage primitive that drives straight at low speed only while fresh MID360 front/left/right clearance proves the padded rectangular footprint fits; stop immediately on stale scan, insufficient clearance, unexpected yaw, timeout or distance bound.
- [ ] Mark the opening crossed only after localization proves the robot center passed the gate plus a safety margin and `/map_frontier` reveals a connected region on the far side; then resume ordinary frontier exploration.
- [ ] A safe-width opening that repeatedly fails traversal keeps completion `incomplete_with_blocked_opening`; it must never be converted into a wall or occluded coverage.
- [ ] Expose confidence, measured width, crossing phase and failure evidence for diagnostics; an uncertain doorway must remain open and must never raise completion confidence.
- [ ] Allow `rooms.yaml` explicit entrance gate or passage geometry to override auto-detection when a known site configuration exists.
- [ ] Run `python -m pytest -q web/test_doorway_topology.py web/test_global_search_state.py web/test_exploration_manager.py`.

## Task 5: Show the evidence in the frontend

**Files:**
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`
- Test: `web/test_map_contract.js`
- Test: `web/test_panel_navigation_contract.py`

- [ ] Broadcast `boundary_closed_ratio`, `explainable_coverage_ratio`, reachable-frontier count, certified-occluded count, stability streak and completion blockers.
- [ ] Render the entrance virtual gate, confirmed walls/obstacles, remaining openings, narrow-passage phase and certified occluded pockets with distinct legend entries.
- [ ] Show a concise Chinese status such as `边界 92% / 可解释覆盖 97% / 仍有 2 个开口` instead of a generic “搜索中”.
- [ ] Add payload-shape and stale-update regression tests so reconnect/refresh does not lose the latest search state.
- [ ] Run `node web/test_map_contract.js` and `python -m pytest -q web/test_panel_navigation_contract.py`.

## Task 6: Validate behavior and performance before NX deployment

**Files:**
- Modify: `web/test_unknown_room_exploration_sim.py`
- Create: `tools/verify_global_search.py`
- Modify: `TEST_PLAN.md`
- Modify: `tools/verify_release.py`

- [ ] Extend the closed-loop simulation with two connected rooms, an open door, internal furniture and an unreachable enclosed pocket.
- [ ] Assert unknown/frontier area is selected before already observed coverage waypoints.
- [ ] Assert the algorithm does not cross the initial entrance gate toward the rear, but does traverse every other safe-width opening in both `current_room` and `whole_floor` modes.
- [ ] Record per-cycle map-analysis latency, candidate selection latency, Nav2 preflight count and completion evidence; fail the verifier on regressions or unverified completion.
- [ ] Run `python tools/verify_release.py` and require the entire release gate to pass.
- [ ] Deploy to NX only after offline pass, then capture one real mission artifact containing `/map_frontier` snapshots, pose, frontier selections, coverage evidence and final reason.
- [ ] On NX, verify: no reachable frontier is skipped, replanning remains continuous, wall/door overlays match the actual room, and completion occurs only after at least 95% explainable coverage.

## Recommended delivery order

Tasks 1–3 are the safe first delivery: they fix global coverage, add the unique entrance exclusion and prevent premature completion. Task 4 solves the separate narrow-door traversal problem without weakening ordinary Nav2 safety; Tasks 5–6 make the decision inspectable and prove it on the real robot. Do not enable automatic narrow-passage execution in production before the noisy-map fixtures, clearance gates and NX evidence pass.
