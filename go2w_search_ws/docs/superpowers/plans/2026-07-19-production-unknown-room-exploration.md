# Production Unknown-Room Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Web “搜索房间” action start a bounded Nav2 frontier mission that keeps producing useful goals throughout a large unknown room instead of treating one visited point as completion of an entire long frontier.

**Architecture:** Keep `/api/search_room`, `RoomSearchOrchestrator`, and Nav2 as the only motion path. Improve the pure occupancy-grid planner in two ways: retain only frontier cells connected to the robot's known-free component, and spatially sample every long connected frontier into multiple independent candidates. Keep the existing adaptive radius, tile scheduler, path preflight, standoff goals, safety budgets, detection pipeline, and mission report.

**Tech Stack:** Python 3.10-compatible pure functions, pytest, ROS 2 OccupancyGrid data contracts, Nav2 through the existing `MissionNavigationPort`, HTML/JavaScript using the existing authenticated control fetch.

**Execution status (2026-07-19):** Implemented locally with TDD. The final design also uses world-distance matching across adjacent failure buckets, retains legacy callback compatibility, and uses bytearray/integer-index flood fill plus spatial buckets to bound noisy-map CPU cost. No NX connection, deployment, activation, or motion command was performed.

**Guarantee boundary:** “Unknown room” means the robot-reachable known/free component inside the configured 30 m safety envelope. With no prior semantic map, an open doorway is topologically connected to the current room, so the algorithm cannot mathematically distinguish “the current room” from connected space beyond that doorway; a calibrated `room_polygon` is required for a strict room boundary.

---

### Task 1: Prove the long-frontier and disconnected-island failures

**Files:**
- Modify: `web/test_frontier_explore.py`

- [ ] **Step 1: Add a failing long-frontier test**

Construct a 40×20 grid whose left half is known free and right half is unknown. Call `find_frontier_clusters(..., frontier_spacing_m=0.6)` and require several candidates spread along the same connected boundary. Mark the nearest candidate visited and require candidates at least 2 m away to remain:

```python
baseline = find_frontier_clusters(
    grid, robot_pose, [], min_cluster_size=3,
    revisit_radius=0.5, frontier_spacing_m=0.6)
assert len(baseline) >= 3
visited = [{"x": baseline[0]["center_world"][0],
            "y": baseline[0]["center_world"][1]}]
remaining = find_frontier_clusters(
    grid, robot_pose, visited, min_cluster_size=3,
    revisit_radius=0.5, frontier_spacing_m=0.6)
assert remaining
assert max(item["center_world"][1] for item in remaining) > 2.0
```

- [ ] **Step 2: Add a failing reachable-component test**

Build two disconnected known-free islands surrounded by unknown cells, put the robot in the first island, and assert every returned frontier representative belongs to that island:

```python
candidates = find_frontier_clusters(
    grid, robot_pose, [], min_cluster_size=1,
    frontier_spacing_m=0.5)
assert candidates
assert all(item["center_world"][0] < 3.0 for item in candidates)
```

- [ ] **Step 3: Run RED**

Run:

```powershell
python -m pytest web/test_frontier_explore.py -k "long_connected_frontier or disconnected_free_island" -q
```

Expected: the existing planner either rejects `frontier_spacing_m`, collapses the long boundary to one candidate, or returns the remote island.

### Task 2: Split frontier components and filter by robot reachability

**Files:**
- Modify: `web/nx_frontier_planner.py`
- Modify: `web/test_frontier_explore.py`

- [ ] **Step 1: Locate the robot's reachable free component**

Convert the robot world pose into a grid cell using the full map-origin transform, choose the nearest free seed within 1 m when the exact cell is not free, and flood-fill eight-connected known-free cells. Only cells in this set may become frontier cells.

```python
reachable_free = _reachable_free_cells(
    data, width, height, resolution,
    origin_x, origin_y, origin_yaw,
    robot_x, robot_y,
)
frontier_cells = [
    cell for cell in reachable_free
    if any(value(cell[0] + dr, cell[1] + dc) == -1
           for dr, dc in _NEIGHBORS_8)
]
```

- [ ] **Step 2: Sample each long component into independent candidates**

Greedily select deterministic representatives ordered by robot distance and cell coordinates. A candidate must be at least `frontier_spacing_m` from already selected representatives. Compute local information gain from component cells within that spacing and apply visited filtering per representative, never per whole component.

```python
for cell in ordered_component:
    wx, wy = cell_world(cell)
    if any(math.hypot(wx - sx, wy - sy) < spacing
           for sx, sy in selected_world):
        continue
    if _was_visited(wx, wy, visited, revisit_radius):
        continue
    selected_world.append((wx, wy))
    result.append(_candidate_for(cell, component, spacing))
```

Bound output with `max_candidates_per_cluster=64`, and compute `touches_map_edge` from each candidate's local support rather than the entire long component.

- [ ] **Step 3: Run GREEN**

Run the Task 1 command. Expected: both tests pass.

### Task 3: Carry spatial-frontier configuration through the mission

**Files:**
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_mission_schema.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `web/test_mission_schema.py`
- Modify: `web/test_frontier_explore.py`

- [ ] **Step 1: Add failing contract tests**

Require current-room requests and manager snapshots to expose `frontier_spacing_m=1.5`, and require the manager to pass it to its candidate selector:

```python
request = SearchMissionRequest.current_room(["person"])
assert request.frontier_spacing_m == pytest.approx(1.5)
assert request.to_task_params()["frontier_spacing_m"] == pytest.approx(1.5)
assert manager.snapshot()["frontier_spacing_m"] == pytest.approx(1.5)
```

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest web/test_mission_schema.py web/test_exploration_manager.py web/test_frontier_explore.py -q
```

Expected: missing field assertions fail.

- [ ] **Step 3: Add and validate the field**

Add `frontier_spacing_m: float = 1.5` to `SearchMissionRequest`; require it to be finite and positive; include it in parsing and current-room task params. Add the same parameter to `ExplorationManager`, include it in `snapshot()`, and pass it into `select_frontier_candidates`. In `_run_frontier_explore`, read it from task params with a 1.5 m default.

- [ ] **Step 4: Run GREEN**

Run the Task 2 command. Expected: all focused schema, manager, and frontier tests pass.

### Task 4: Lock the Web-to-algorithm contract and run regression

**Files:**
- Modify: `web/test_panel_navigation_contract.py`
- Verify: `web/static/panel.html`
- Verify: `web/nx_web_server.py`

- [ ] **Step 1: Add a source-level Web contract test**

Require the panel action to call the canonical endpoint directly and request current-room person search:

```python
panel = read("web/static/panel.html")
assert 'onclick="searchRoom()"' in panel
assert "controlFetch('/api/search_room'" in panel
assert "room: 'current_room'" in panel
assert "target_classes: ['person']" in panel
```

- [ ] **Step 2: Run the complete local regression**

Run:

```powershell
python -m pytest web/test_exploration_manager.py web/test_frontier_explore.py web/test_coverage_metrics.py web/test_mission_schema.py web/test_product_command.py web/test_voice_search_contract.py web/test_panel_navigation_contract.py tools/test_verify_release_artifact.py -q
```

Expected: zero failures. No NX deployment or motion command is part of this task because NX is disconnected.

---

## Self-Review

- Spec coverage: large connected frontiers, per-goal revisiting, reachable-space filtering, adaptive mission configuration, Web admission, and local regression each map to an explicit task.
- Placeholder scan: no deferred implementation markers are present; each task identifies exact behavior, files, commands, and expected results.
- Type consistency: `frontier_spacing_m` is a positive float from HTTP schema through task params, orchestrator, manager, planner, telemetry, and tests.
- Safety: the algorithm only emits Nav2 goal poses; it never publishes velocity. This plan performs no NX connection, deployment, activation, or physical movement.
