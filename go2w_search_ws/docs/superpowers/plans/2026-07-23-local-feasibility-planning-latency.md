# Local Feasibility and Planning Latency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent frontier search from driving the Go2W into a locally immobile pose, stop repeated `No valid trajectories out of 602` loops, and reduce the measured frontier-selection pause from roughly 13 seconds to a bounded candidate-analysis pass.

**Architecture:** Keep Nav2 as the final collision authority, but propagate fresh MID360 path and turning-clearance evidence into each exploration candidate before any global-plan probe. A blocked scan must produce no forward step, and a pose with neither a safe forward staging move nor a safe initial turn must terminate as `motion_trapped` instead of rotating through frontier goals. Bound expensive visibility/yaw analysis to the candidates that can fit in the current planning budget, while retaining frontier priority and rotating past blacklisted candidates.

**Tech Stack:** Python 3, pytest, ROS 2 Humble Nav2/DWB configuration, Nav2 Behavior Tree XML, systemd deployment contracts.

---

### Task 1: Lock blocked-clearance semantics

**Files:**
- Modify: `web/test_visibility_coverage.py`
- Modify: `web/nx_visibility_coverage.py`

- [ ] **Step 1: Write failing scan-profile tests**

Add tests proving that a fresh obstacle closer than `obstacle_standoff_m + 0.35 m` produces `adaptive_step_m == 0`, `path_blocked is True`, and a stale scan also fails closed with a zero step. Add a candidate-ranking assertion that current-forward and full-turn clearance evidence is copied into the candidate.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py -k "blocked or stale or turn_clearance"
```

Expected: FAIL because the current implementation clamps every result to `min_step_m == 1.0` and publishes no blocked/turn fields.

- [ ] **Step 3: Implement the minimum safe-step model**

Add `minimum_motion_step_m=0.35` and `turn_swept_radius_m=0.57` tracker parameters. Return zero instead of one metre when usable clearance is below the minimum; include `path_blocked`, `turn_clearance_m`, and `turn_motion_blocked` in fresh and conservative profiles. Preserve long adaptive steps in open space.

- [ ] **Step 4: Run the complete visibility suite**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py
```

Expected: all tests pass with stale-scan expectations updated to fail closed.

### Task 2: Reject locally impossible starts and report a trap

**Files:**
- Modify: `web/test_exploration_manager.py`
- Modify: `web/test_frontier_explore.py`
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`

- [ ] **Step 1: Write failing local-feasibility tests**

Add tests for these contracts:

```python
assert manager.choose_next(map_msg, pose) is None
assert manager.snapshot()["last_selection_reason"] == "motion_trapped"
assert nav.probes == []
```

Use candidates whose direct path is blocked, whose heading requires the rotation shim, whose turn clearance is below `0.57 m`, and whose current forward staging path is also blocked. Add a second case proving that a blocked turn with a safe forward corridor selects one straight staging goal rather than three speculative `0/+30/-30` degree goals.

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py -k "motion_trapped or blocked_turn"
```

Expected: FAIL because blocked profiles currently fall back to the physical frontier and `_eligible_candidates` treats distance alone as executable.

- [ ] **Step 3: Implement local approach admission**

Copy current-forward evidence from the visibility tracker into ranked candidates. In `_candidate_approaches`, return no approach when both the initial turn and the current forward staging move are blocked; when forward staging is safe, emit only a straight, clearance-capped staging pose. Do not turn a locally rejected candidate set into frontier exhaustion: preserve `motion_trapped` and structured trap evidence in the manager snapshot.

- [ ] **Step 4: Make the orchestrator stop the retry loop**

When `choose_next()` returns `motion_trapped`, exit the loop as incomplete. If a Nav2 abort occurs before the scan gate catches it, pass the live post-failure pose to `mark_navigation_failed`; when the pose did not move and current forward/turn evidence is blocked, return `motion_trapped` after the first failed goal rather than submitting another frontier.

- [ ] **Step 5: Run manager and mission regressions**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py web/test_frontier_explore.py
```

Expected: all tests pass and no test expects a blocked pose to keep probing.

### Task 3: Bound expensive candidate and yaw analysis

**Files:**
- Modify: `web/test_exploration_manager.py`
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `docker/go2w-web.service`
- Modify: `docker/bringup_slam_nav2.sh`
- Modify: `docker/test_global_planner_contract.py`

- [ ] **Step 1: Write failing work-bound tests**

Use 98 synthetic frontier candidates and a counting visibility tracker. Assert that no more than 24 non-blacklisted candidates enter full visibility analysis and no more than 12 candidates enter all-yaw optimization. Prove that candidates after failed/blacklisted leading entries rotate into the next cycle instead of being hidden by the limit.

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py -k "analysis_limit or yaw_optimization_limit"
```

Expected: FAIL because all 98 candidates currently receive roughly nine ray-casts each.

- [ ] **Step 3: Add bounded analysis and timing evidence**

Add `candidate_analysis_limit=24` and `yaw_optimization_candidate_limit=12`, with environment overrides `GO2W_FRONTIER_ANALYSIS_LIMIT` and `GO2W_FRONTIER_YAW_CANDIDATE_LIMIT`. Apply mission bounds and spatial failure filtering before expensive visual analysis, optimize yaw only for the leading bounded set, retain the existing yaw/gain for the remaining analyzed candidates, move per-candidate logs to debug, and expose raw/analyzed/yaw counts plus elapsed milliseconds in `snapshot()`.

- [ ] **Step 4: Persist and contract-test deployment defaults**

Set both environment values in `go2w-web.service` and `bringup_slam_nav2.sh`; extend the deployment contract to require identical values.

- [ ] **Step 5: Run planning-policy regressions**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py docker/test_global_planner_contract.py
```

Expected: all tests pass, with the 98-candidate fixture bounded to 24 visibility analyses and 12 yaw optimizations.

### Task 4: Shorten known-impossible Nav2 retry latency

**Files:**
- Modify: `docker/test_persistent_exploration_contract.py`
- Modify: `src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml`
- Modify: `src/go2w_nav/behavior_trees/navigate_through_poses_dynamic_safe.xml`
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`

- [ ] **Step 1: Change contract expectations first**

Require one wait-and-clear retry instead of four and require production DWB trajectory debugging/evaluation publication to be disabled.

- [ ] **Step 2: Verify RED**

Run:

```powershell
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py
```

Expected: FAIL against the current `number_of_retries="4"` and debug-enabled DWB configuration.

- [ ] **Step 3: Implement bounded retry and production DWB settings**

Set `number_of_retries="1"` in both trees, preserving the single one-second hold that allows a transient obstacle/costmap update to settle. Set `debug_trajectory_details: false` and `publish_evaluation: false`; do not reduce collision critics, footprint size, or controller safety margins.

- [ ] **Step 4: Run Nav2 contract tests**

Run:

```powershell
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py docker/test_global_planner_contract.py
```

Expected: all tests pass.

### Task 5: Verify, deploy, and measure on NX

**Files:**
- Modify only if verification finds a defect in the preceding scoped files.

- [ ] **Step 1: Run the complete focused regression set**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py web/test_exploration_manager.py web/test_frontier_explore.py docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py docker/test_global_planner_contract.py
node web/test_panel_nav_state.js
git diff --check
```

- [ ] **Step 2: Review the exact diff and preserve the checkpoint**

Verify `demo` still resolves to `e5fcfae5819121679c67c7af45c2c0bcf2ad9c09` and that only the planned fix files changed after that tag.

- [ ] **Step 3: Deploy through the repository release path**

Build and deploy the Web/Nav2 scoped release using the existing release scripts; do not replace running services by copying ad-hoc files. Wait for all health gates.

- [ ] **Step 4: Collect live evidence without commanding unrequested motion**

Read the deployed release ID, runtime parameters, health state, and planning timing fields. Confirm the new service has a 24-candidate analysis bound, 12-candidate yaw bound, one BT retry, and DWB debugging disabled. The currently trapped robot must remain stopped; a bounded rear-clearance recovery move requires an explicit operator command before starting the next full search.

