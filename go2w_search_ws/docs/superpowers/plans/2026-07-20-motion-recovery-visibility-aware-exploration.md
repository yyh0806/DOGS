# Motion Recovery and Visibility-Aware Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make an active room search recover safely when the wheel-drive session falls back to parked, and choose longer or shorter Nav2 goals from live LiDAR clearance, C13 horizontal field of view, occupancy-map occlusion, and already-seen visual coverage.

**Architecture:** Keep Nav2 as the sole velocity-producing navigation path. Add a bounded task-owner recovery hook between `MissionNavigationPort` and `NavigationArbiter`, then add a pure occupancy-grid visibility tracker that ray-casts the C13 frustum through known free space and stops at walls/unknown cells. `ExplorationManager` uses the tracker to estimate visual gain, select an adaptive step length, penalize unnecessary heading reversals, and schedule coverage viewpoints after occupancy frontiers are exhausted; `RoomSearchOrchestrator` publishes coverage telemetry that the existing Canvas map renders.

**Tech Stack:** Python 3.10, pytest, ROS 2/Nav2 action facade, OccupancyGrid and LaserScan snapshots, HTML5 Canvas JavaScript, existing release/deployment scripts.

---

### Task 1: Recover an active mission after a parked wheel session

**Files:**
- Modify: `web/nx_navigation_arbiter.py`
- Modify: `web/nx_navigation_gateway.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/test_navigation_arbitration.py`
- Modify: `web/test_navigation_gateway.py`

- [ ] **Step 1: Write failing recovery tests**

Add a navigation-arbiter test whose task owner remains active while robot feedback changes from `nav_active` to `parked`. Require `recover_task_motion()` to call the existing feedback-gated `_activate_drive("nav", ...)`, refresh point-navigation health, preserve `motion_owner == "tasks"`, and never cancel the task. Add a gateway test whose action port enters `waiting_health/motion_unhealthy`; require the mission wait loop to invoke one rate-limited recovery callback and subsequently return the resumed goal's success.

```python
result = arbiter.recover_task_motion("mission_motion_unhealthy")
assert result["ok"] is True
assert ("drive_start", "nav") in events
assert tasks.cancel_count == 0

result = mission.send_goal_and_wait(4.0, 0.0, 0.0)
assert result["ok"] is True
assert recoveries == ["motion_unhealthy"]
```

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest web/test_navigation_arbitration.py web/test_navigation_gateway.py -k "recover_task_motion or mission_recovers_motion" -q
```

Expected: failures because neither the arbiter recovery entry point nor mission recovery hook exists.

- [ ] **Step 3: Implement bounded recovery**

Implement `NavigationArbiter.recover_task_motion(reason)` under the arbiter lock. It must reject non-task ownership, return immediately if navigation readiness is already ready, only reactivate when readiness is `activatable`, call `_activate_drive("nav", "task_reactivation_failed")`, tick the shared navigation port once, and verify point health before returning success. Extend `MissionNavigationPort` with a recovery callback and have its terminal wait poll call the hook at most once per configured interval while the shared action reports `waiting_health` with `motion_unhealthy`. Hard recovery failures cancel the queued goal and return a concrete failure instead of waiting 40 seconds.

- [ ] **Step 4: Wire and run GREEN**

After constructing `NavigationArbiter` in `nx_web_server.main()`, inject `navigation_arbiter.recover_task_motion` into the mission port. Run the Step 2 command and expect all focused tests to pass.

### Task 2: Model LiDAR/C13 visibility and building occlusion

**Files:**
- Create: `web/nx_visibility_coverage.py`
- Create: `web/test_visibility_coverage.py`

- [ ] **Step 1: Write failing pure-model tests**

Create synthetic OccupancyGrid and scan snapshots proving:

```python
def test_wall_stops_camera_visibility(): ...
def test_open_scan_selects_long_step(): ...
def test_cluttered_scan_selects_short_step(): ...
def test_visual_gain_excludes_already_observed_cells(): ...
def test_coverage_viewpoints_target_occluded_unswept_space(): ...
```

The wall test must place known-free cells behind an occupied wall and assert they are absent from visible cells. The adaptive-step tests must use the same C13 HFOV but different LiDAR range distributions and assert an open step of at least 5 m and a cluttered step no greater than 2 m.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest web/test_visibility_coverage.py -q
```

Expected: import failure because `nx_visibility_coverage` does not exist.

- [ ] **Step 3: Implement the pure tracker**

Add `VisibilityCoverageTracker` with these public methods:

```python
tracker.observe(map_msg, robot_pose, scan_snapshot) -> dict
tracker.rank_candidates(map_msg, robot_pose, candidates) -> list[dict]
tracker.coverage_candidates(map_msg, robot_pose, visited, limit=32) -> list[dict]
tracker.snapshot(map_msg=None) -> dict
```

Use the full OccupancyGrid origin pose for world/grid transforms. Ray-cast across `camera_hfov_rad` at a bounded angular resolution; stop each ray on occupied or unknown cells so walls and unmapped space occlude the C13 view. Limit ray distance by both configured C13 visual range and the matching live LiDAR return. Store observed cells as stable world-space buckets so SLAM map-origin growth cannot erase coverage. Compute `scene_complexity`, `forward_clearance_m`, and `adaptive_step_m`; clamp steps to configured safe `[min_step_m, max_step_m]`.

- [ ] **Step 4: Run GREEN**

Run the Step 2 command and expect all visibility-model tests to pass.

### Task 3: Use visual gain and adaptive step in frontier exploration

**Files:**
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `web/test_frontier_explore.py`

- [ ] **Step 1: Write failing integration tests**

Add manager tests proving that an open profile clamps a distant frontier to a long intermediate goal, a cluttered profile uses a short intermediate goal, visual gain outranks an equally reachable already-covered candidate, and heading hysteresis avoids alternating between equivalent frontiers. Add an orchestrator test proving each selection cycle records the current scan/pose before choosing and publishes `observed_cells`, `visual_coverage_ratio`, `adaptive_step_m`, and candidate/visited viewpoints.

```python
goal = manager.choose_next(map_msg, (0.0, 0.0, 0.0))
assert goal["adaptive_step_m"] >= 5.0
assert goal["x"] >= 5.0
```

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest web/test_exploration_manager.py web/test_frontier_explore.py -k "visibility or adaptive_step or swept or heading_hysteresis" -q
```

Expected: failures because the manager has no visibility tracker and the orchestrator does not publish coverage state.

- [ ] **Step 3: Integrate the tracker**

Inject a tracker into `ExplorationManager`. Add `observe_environment(map_msg, robot_pose, scan_snapshot)`, rank occupancy frontiers by new visual gain, add a bounded adaptive intermediate approach before fixed standoffs, and use the last successful heading as a small hysteresis penalty against needless reversals. When occupancy frontiers are stably exhausted but visual coverage remains below threshold, request bounded coverage viewpoints from the tracker before reporting completion. Preserve Nav2 path preflight, room/radius bounds, standoff safety, failure buckets, time/distance/battery limits, and the existing frontier fallback.

In `_run_frontier_explore`, obtain the current `LaserScanSnapshot`, update visibility before selection and after arrival, and attach the tracker snapshot to every relevant `_phase()` payload and the final report.

- [ ] **Step 4: Run GREEN**

Run the Step 2 command and then the complete visibility/exploration subset. Expect zero failures.

### Task 4: Render swept visual coverage and deploy safely

**Files:**
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`
- Modify: `web/test_panel_navigation_contract.py`
- Modify: `tools/deploy_nx_complete.ps1` only if required by release verification

- [ ] **Step 1: Write failing frontend contract tests**

Require the map state parser to accept `observed_cells`, `coverage_cell_size_m`, `visual_coverage_ratio`, `adaptive_step_m`, and `scene_complexity` for frontier searches even when no rectangular `room_area` exists. Require rendering of the swept cells outside the `if (roomArea)` block and panel forwarding of live `room_nav.search_state` fields.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest web/test_panel_navigation_contract.py -k "swept or visibility_coverage" -q
```

Expected: assertions fail because observed cells are currently drawn only when `roomArea` exists.

- [ ] **Step 3: Implement and verify locally**

Draw observed C13-visible cells as a translucent green layer for unknown-room searches, retain the room boundary when present, and show a compact map label with visual coverage percentage, adaptive step, and complexity. Run all navigation, visibility, exploration, schema, and panel tests.

- [ ] **Step 4: Build and activate one consistent NX release**

Build a complete release containing web code and the ROS install payload. Verify the release id from `/api/status`, `/dog_state`, `/home/nx/go2w/current`, and both running process command lines agrees before testing. Restart only the required services, perform read-only localization/LiDAR/C13/battery/drive preflight, then start one bounded room-search mission from the Web endpoint.

- [ ] **Step 5: Validate the physical acceptance path**

Observe at least one long goal in open space and one shortened goal near clutter/walls; verify the map accumulates swept coverage, the dog does not oscillate between equivalent frontiers, person markers remain present, and a forced parked-session event recovers without canceling the mission. Stop the mission after evidence is collected and preserve the journal/API trace.

---

## Self-Review

- Spec coverage: parked-session recovery, live LiDAR clearance, C13 HFOV, occupancy occlusion, adaptive distance, oscillation suppression, visual coverage completion, front-end swept-area display, consistent deployment, and physical acceptance each map to an explicit task.
- Placeholder scan: no deferred implementation markers are present; every task has named files, tests, commands, and expected behavior.
- Type consistency: coverage fields use the same snake_case keys from tracker snapshots through orchestrator WebSocket payloads and the Canvas parser.
- Safety: only the existing Nav2 action path commands movement; recovery is task-owner scoped and refuses non-activatable/hard-fault states.
