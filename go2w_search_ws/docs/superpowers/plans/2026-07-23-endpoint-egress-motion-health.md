# Endpoint Egress and Motion Health Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent whole-room exploration from selecting one-way terminal poses, stop retrying controller-aborted frontiers, and remove false `motion_unhealthy` trips caused by the Go2W wheel-response latency.

**Architecture:** Extend the existing MID360 visibility profile with terminal egress evidence. A candidate may end in a narrow passage only when it retains a safe forward continuation; otherwise its approach is shortened so at least one minimum motion step remains, and an endpoint with neither turning room nor forward egress is rejected before Nav2. Treat controller aborts as hard failures for the current planning epoch. Keep the feedback-staleness timeout strict while giving wheel actuation its own measured startup grace.

**Tech Stack:** Python 3, pytest, ROS 2 Humble Nav2/DWB, Unitree Go2-W motion bridge, repository release verification scripts.

---

### Task 1: Preserve egress at every exploration endpoint

**Files:**
- Modify: `web/test_visibility_coverage.py`
- Modify: `web/nx_visibility_coverage.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `web/nx_exploration_manager.py`

- [ ] **Step 1: Write failing endpoint-profile tests**

Add a visibility test with a narrow forward corridor and a wall shortly after the requested candidate. Require the ranked candidate to publish:

```python
assert candidate["terminal_turn_blocked"] is True
assert candidate["terminal_forward_margin_m"] < 0.35
assert candidate["terminal_egress_limited"] is True
assert 0.35 <= candidate["terminal_safe_step_m"] < candidate["distance"]
```

Add a second test where the candidate is beyond the last safe step:

```python
assert candidate["terminal_safe_step_m"] == 0.0
assert candidate["terminal_egress_safe"] is False
```

Add a doorway/corridor case proving that a terminal pose without turning room remains eligible when at least `minimum_motion_step_m` of measured forward continuation exists.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py -k "terminal_egress or terminal_turn"
```

Expected: FAIL because candidates currently contain only current-pose turn clearance and no terminal egress contract.

- [ ] **Step 3: Implement terminal geometry and safe-step calculation**

In `VisibilityCoverageTracker.rank_candidates`, calculate candidate terminal clearance from known occupied grid cells and finite MID360 obstacle endpoints. Unknown map cells are not walls. Publish:

```python
{
    "terminal_turn_clearance_m": clearance,
    "terminal_turn_blocked": clearance < self.turn_swept_radius_m,
    "terminal_forward_margin_m": max(
        0.0,
        path_profile["forward_clearance_m"]
        - candidate_distance
        - self.obstacle_standoff_m,
    ),
    "terminal_safe_step_m": safe_step,
    "terminal_egress_safe": original_goal_is_safe,
    "terminal_egress_limited": safe_step < candidate_distance,
}
```

When turning is blocked, reserve `minimum_motion_step_m` beyond the selected endpoint. Do not require turning room inside a `0.8 m` doorway when the live forward corridor continues into the next space.

- [ ] **Step 4: Write manager RED tests**

Require `_candidate_approaches` to submit only the shortened safe endpoint and never fall back to the unsafe physical frontier:

```python
selected = manager.choose_next(map_msg, pose)
assert selected["approach_terminal_egress_m"] == pytest.approx(1.2)
assert all(probe[0] <= 1.2 for probe in nav.probes)
```

Require a candidate with `terminal_safe_step_m == 0.0` to be rejected without a Nav2 path probe while a different safe frontier remains selectable.

- [ ] **Step 5: Run manager RED**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py -k "terminal_egress"
```

Expected: FAIL because `_candidate_approaches` can still fall back to the unsafe frontier.

- [ ] **Step 6: Implement egress-limited approaches**

Clamp the adaptive approach by `terminal_safe_step_m`. If the clamp is below `min_goal_distance_m`, return no approaches. Mark a shortened goal with `approach_terminal_egress_m` and return only egress-safe approaches so a later fallback cannot reintroduce the unsafe endpoint.

- [ ] **Step 7: Run the full visibility and manager suites**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py web/test_exploration_manager.py
```

Expected: all tests pass.

### Task 2: Do not resubmit controller-aborted physical frontiers

**Files:**
- Modify: `web/test_exploration_manager.py`
- Modify: `web/nx_exploration_manager.py`

- [ ] **Step 1: Write a failing hard-failure test**

Record one `aborted` or `nav2_aborted` navigation failure, change the map revision, and require the same world-space frontier to be filtered immediately:

```python
manager.mark_navigation_failed("aborted", selected, robot_pose=pose)
assert manager.choose_next(changed_map, pose) is None
assert len(nav.probes) == probes_after_first_attempt
```

Keep `timeout` retriable once so transient delays do not permanently suppress a frontier.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py -k "controller_abort or hard_failure"
```

Expected: FAIL because `aborted` currently increments the spatial count by only one while the configured maximum is two.

- [ ] **Step 3: Implement hard structural failure accounting**

Make `aborted`, `nav2_aborted`, `controller_abort`, `controller_failed`, and `degenerate_plan` saturate the current spatial failure count at `max_failures_per_cell`. Preserve the existing world-space matching across map-origin and revision changes.

- [ ] **Step 4: Run the manager suite**

Run:

```powershell
python -m pytest -q web/test_exploration_manager.py
```

Expected: all tests pass.

### Task 3: Separate wheel startup grace from feedback freshness

**Files:**
- Modify: `src/go2w_bridge/test/test_motion_safety.py`
- Modify: `src/go2w_bridge/test/test_motion_controller.py`
- Modify: `src/go2w_bridge/go2w_bridge/motion_safety.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`

- [ ] **Step 1: Write failing watchdog timing tests**

Construct `DriveExecutionWatchdog(timeout=0.2, response_grace=0.7, ...)`. With fresh zero-wheel feedback, require no `wheel_no_response` at `0.64 s`, then require the fault only after `0.70 s`. Separately prove that stale/invalid feedback keeps the shorter `timeout` confirmation.

- [ ] **Step 2: Run RED**

Run:

```powershell
$env:PYTHONPATH='src/go2w_bridge'
python -m pytest -q src/go2w_bridge/test/test_motion_safety.py -k "response_grace"
```

Expected: FAIL because `response_grace` is not accepted and the current `0.6 s` timer serves two unrelated safety purposes.

- [ ] **Step 3: Implement reason-specific timing**

Add an optional `response_grace` constructor argument validated as finite and positive. Use it only for `wheel_no_response`; continue using `timeout` for stale/invalid feedback. Track the active fault reason so a reason change starts the correct confirmation interval. Clear reason/timer state on zero command, measured wheel response, unexpected gait handling, and `reset()`.

Wire `drive_response_grace` through `NxMotionNode` with a `1.0 s` default, retaining `drive_response_timeout=0.6 s` for stale feedback.

- [ ] **Step 4: Run motion bridge suites**

Run:

```powershell
$env:PYTHONPATH='src/go2w_bridge'
python -m pytest -q src/go2w_bridge/test/test_motion_safety.py src/go2w_bridge/test/test_motion_controller.py src/go2w_bridge/test/test_motion_machine.py
```

Expected: all tests pass.

### Task 4: Verify and deploy the scoped release

**Files:**
- Modify only if a verification contract exposes a scoped defect.

- [ ] **Step 1: Run focused regressions and static checks**

Run:

```powershell
python -m pytest -q web/test_visibility_coverage.py web/test_exploration_manager.py web/test_frontier_explore.py
$env:PYTHONPATH='src/go2w_bridge'
python -m pytest -q src/go2w_bridge/test/test_motion_safety.py src/go2w_bridge/test/test_motion_controller.py src/go2w_bridge/test/test_motion_machine.py
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py docker/test_global_planner_contract.py
node web/test_panel_nav_state.js
git diff --check
```

- [ ] **Step 2: Review the exact diff**

Confirm the `demo` tag is unchanged and only the plan, exploration endpoint policy, hard-failure accounting, watchdog timing, and their tests changed.

- [ ] **Step 3: Deploy through the repository release path**

Use the existing scoped release/deployment scripts. Do not copy files ad hoc. Wait for Web, Nav2, motion, mapping, and perception health gates.

- [ ] **Step 4: Verify live configuration without commanding motion**

Confirm the deployed Web and motion release IDs match, the dog remains stopped, navigation is activatable, and the motion node reports the new response-grace default. Do not start a new search or recovery movement without the operator initiating it.
