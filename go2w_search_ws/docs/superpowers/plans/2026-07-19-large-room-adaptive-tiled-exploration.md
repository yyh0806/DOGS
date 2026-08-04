# Large-Room Adaptive Tiled Exploration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed 6 m / 15-waypoint / 60-probe current-room search with a bounded, dynamically expanding, tile-prioritized frontier search that only reports completion after stable reachable-frontier exhaustion.

**Architecture:** Keep Nav2 as the only motion owner and keep the existing frontier detector. Treat `max_radius_m` as a safety ceiling, start at `initial_radius_m`, expand in `radius_step_m` increments, and prioritize frontiers from one `tile_size_m` tile at a time. Planning probes are bounded per selection cycle rather than across the whole mission, while cumulative metrics remain report-only. Final coverage exposes both the legacy raw circle ratio and a bounded-interior ratio that excludes exterior unknown components touching the ROI/map boundary.

**Tech Stack:** Python 3.10-compatible dataclasses and pure functions, pytest, ROS 2/Nav2 integration through the existing `MissionNavigationPort`, HTML/JavaScript frontend consuming the existing mission schema.

**Implementation status (2026-07-19):** Tasks 1–4 are implemented in commits `4a656de`, `e1d5307`, `bccf904`, and `f18c82c`. Task 5 documentation and regression are complete; NX deployment/preflight is the remaining operational step. The checklists below retain the executed TDD recipe for auditability.

---

### Task 1: Extend the current-room mission contract

**Files:**
- Modify: `go2w_search_ws/web/nx_mission_schema.py`
- Modify: `go2w_search_ws/web/nx_product_command.py`
- Modify: `go2w_search_ws/web/test_mission_schema.py`
- Modify: `go2w_search_ws/web/test_product_command.py`

- [ ] **Step 1: Write failing contract tests**

Add assertions that a default current-room mission serializes these parameters:

```python
assert request.max_radius_m == 30.0
assert request.max_time_s == 1800.0
assert request.initial_radius_m == 6.0
assert request.radius_step_m == 6.0
assert request.tile_size_m == 6.0
assert request.stable_exhaustion_cycles == 3
assert request.max_frontiers == 200
assert request.max_plan_probes_per_cycle == 12
```

Add validation cases for initial radius above maximum radius, non-positive tile size, non-positive radius step, and non-positive integer budgets.

- [ ] **Step 2: Run the focused tests and observe RED**

Run:

```powershell
python -m pytest web/test_mission_schema.py web/test_product_command.py -q
```

Expected: failures for missing adaptive-search fields and the obsolete 6 m / 480 s defaults.

- [ ] **Step 3: Implement the contract fields and validation**

Extend `SearchMissionRequest` with Python 3.10-compatible fields:

```python
initial_radius_m: float = 6.0
radius_step_m: float = 6.0
tile_size_m: float = 6.0
stable_exhaustion_cycles: int = 3
max_frontiers: int = 200
max_plan_probes_per_cycle: int = 12
```

Set current-room safety defaults to `max_radius_m=30.0` and `max_time_s=1800.0`. Validate finite positive floats, `initial_radius_m <= max_radius_m`, and positive integers. Include every field in `from_dict`, legacy `from_api_payload`, `to_dict`, and `to_task_params`; keep named-room behavior unchanged.

- [ ] **Step 4: Run focused tests and observe GREEN**

Run the Step 2 command. Expected: all mission-schema and product-command tests pass.

- [ ] **Step 5: Commit the isolated contract change**

```powershell
git add go2w_search_ws/web/nx_mission_schema.py go2w_search_ws/web/nx_product_command.py go2w_search_ws/web/test_mission_schema.py go2w_search_ws/web/test_product_command.py
git commit -m "feat(frontier): add large-room adaptive mission contract"
```

### Task 2: Make frontier selection dynamically expanding and tile-prioritized

**Files:**
- Modify: `go2w_search_ws/web/nx_exploration_manager.py`
- Modify: `go2w_search_ws/web/test_exploration_manager.py`

- [ ] **Step 1: Write failing manager tests**

Add tests with real `ExplorationManager` and deterministic candidate selectors proving:

```python
def test_current_room_expands_radius_when_local_frontiers_are_exhausted(): ...
def test_candidates_in_active_tile_are_exhausted_before_switching_tiles(): ...
def test_plan_probe_budget_resets_each_selection_cycle(): ...
def test_same_spatial_frontier_is_bounded_across_rapid_map_revisions(): ...
def test_exhaustion_requires_configured_stable_confirmations(): ...
```

Assertions must verify radius transitions `6 -> 12`, tile IDs in the selected candidates, more than 12 cumulative probes across separate cycles without returning `planning_budget_exhausted`, and stable exhaustion only on the third confirmation.

- [ ] **Step 2: Run manager tests and observe RED**

```powershell
python -m pytest web/test_exploration_manager.py -q
```

Expected: new constructor arguments, radius/tile snapshot fields, and per-cycle budget behavior are absent.

- [ ] **Step 3: Implement adaptive bounds and tile scheduling**

Add constructor inputs:

```python
initial_radius_m=6.0
radius_step_m=6.0
tile_size_m=6.0
stable_exhaustion_cycles=3
```

Maintain:

```python
self._active_radius_m
self._active_tile
self._visited_tiles
self._spatial_failures
self._exhaustion_streak
```

Use `room_radius_m` only as the maximum safety ceiling. Query candidates at `_active_radius_m`; when none exist, grow by `radius_step_m` and retry until either candidates appear or the safety ceiling is reached. Compute tile IDs relative to `mission_origin`, keep candidates in the active tile while available, then select the nearest candidate tile. Reset spatial failure counts after a successful navigation or radius expansion so genuinely changed geometry is reconsidered, but do not let raw occupancy hashes bypass the per-cell failure cap while the robot has not moved.

Replace the lifetime planning gate with a local counter:

```python
probes_this_cycle = 0
while probes_this_cycle < self.max_plan_probes:
    ...
```

Keep `_plan_probes` as cumulative telemetry only. When no eligible frontier remains at maximum radius, increment `_exhaustion_streak`; return `stability_confirmation_pending` until the configured count, then return `reachable_frontiers_exhausted`.

- [ ] **Step 4: Run manager tests and observe GREEN**

Run the Step 2 command. Expected: all manager tests pass.

- [ ] **Step 5: Commit the manager change**

```powershell
git add go2w_search_ws/web/nx_exploration_manager.py go2w_search_ws/web/test_exploration_manager.py
git commit -m "feat(frontier): dynamically expand and schedule exploration tiles"
```

### Task 3: Integrate adaptive scheduling into room orchestration

**Files:**
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`
- Modify: `go2w_search_ws/web/test_frontier_explore.py`

- [ ] **Step 1: Write failing orchestrator tests**

Add tests proving:

```python
def test_frontier_explore_continues_after_per_cycle_probe_budget(): ...
def test_frontier_explore_reports_dynamic_radius_and_tiles(): ...
def test_frontier_explore_waits_for_stable_exhaustion(): ...
def test_large_room_defaults_do_not_stop_at_fifteen_waypoints(): ...
```

The first test must reproduce the field failure: one cycle rejects its local candidates, the next simulated map/candidate set is reachable, and the mission continues instead of returning `planning_budget_exhausted`.

- [ ] **Step 2: Run focused orchestrator tests and observe RED**

```powershell
python -m pytest web/test_frontier_explore.py -q
```

Expected: the old lifetime probe check at the top of the loop terminates the task.

- [ ] **Step 3: Wire adaptive mission parameters**

Construct `ExplorationManager` using the canonical task parameters:

```python
room_radius_m=max_radius
initial_radius_m=params.get("initial_radius_m", 6.0)
radius_step_m=params.get("radius_step_m", 6.0)
tile_size_m=params.get("tile_size_m", 6.0)
stable_exhaustion_cycles=params.get("stable_exhaustion_cycles", 3)
max_plan_probes=params.get("max_plan_probes_per_cycle", 12)
```

Remove the orchestrator lifetime comparison `snapshot()["plan_probes"] >= max_plan_probes`. Treat `search_boundary_expanded`, `tile_transition_pending`, `retry_pending`, and `stability_confirmation_pending` as continue states. Keep time, distance, battery, cancellation, Nav2 failure, and waypoint safety ceilings as incomplete terminal conditions.

Report `active_radius_m`, `max_radius_m`, `tile_size_m`, `active_tile`, `visited_tiles`, `exhaustion_streak`, and cumulative planning telemetry. Use `active_radius_m` for the final circle ROI and `max_radius_m` as the safety envelope.

- [ ] **Step 4: Run frontier tests and observe GREEN**

Run the Step 2 command. Expected: all frontier orchestration tests pass.

- [ ] **Step 5: Commit the orchestration change**

```powershell
git add go2w_search_ws/web/nx_room_orchestrator.py go2w_search_ws/web/test_frontier_explore.py
git commit -m "feat(frontier): continue large-room search across planning cycles"
```

### Task 4: Report bounded-interior coverage instead of exterior unknown padding

**Files:**
- Modify: `go2w_search_ws/web/nx_coverage_metrics.py`
- Modify: `go2w_search_ws/web/test_coverage_metrics.py`
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`
- Modify: `go2w_search_ws/web/test_frontier_explore.py`

- [ ] **Step 1: Write failing coverage tests**

Create a closed-wall synthetic room inside a much larger circular ROI. Assert that legacy `explored_ratio` remains low because exterior unknown cells are counted, while the new bounded metric is complete:

```python
assert metrics["explored_ratio"] < 0.5
assert metrics["bounded_explored_ratio"] == pytest.approx(1.0)
assert metrics["exterior_unknown_cells"] > 0
```

Add an interior unknown pocket and assert it lowers `bounded_explored_ratio` and appears in `enclosed_unknown_regions`.

- [ ] **Step 2: Run coverage tests and observe RED**

```powershell
python -m pytest web/test_coverage_metrics.py -q
```

Expected: `bounded_explored_ratio` and `exterior_unknown_cells` do not exist.

- [ ] **Step 3: Implement bounded-interior metrics**

During unknown-component traversal, classify components touching the ROI or map boundary as exterior. Return:

```python
"exterior_unknown_cells": exterior_unknown_cells
"interior_unknown_cells": interior_unknown_cells
"bounded_total_cells": free + occupied + interior_unknown_cells
"bounded_explored_ratio": (free + occupied) / bounded_total_cells
```

Keep the existing raw fields for compatibility. In `_derive_completion_status`, use `bounded_explored_ratio` when present, falling back to `explored_ratio` for older payloads.

- [ ] **Step 4: Run coverage and frontier tests and observe GREEN**

```powershell
python -m pytest web/test_coverage_metrics.py web/test_frontier_explore.py -q
```

- [ ] **Step 5: Commit coverage semantics**

```powershell
git add go2w_search_ws/web/nx_coverage_metrics.py go2w_search_ws/web/test_coverage_metrics.py go2w_search_ws/web/nx_room_orchestrator.py go2w_search_ws/web/test_frontier_explore.py
git commit -m "fix(frontier): measure coverage inside inferred room bounds"
```

### Task 5: Regression, deployment, and safe NX preflight

**Files:**
- Modify: `go2w_search_ws/docs/superpowers/specs/2026-07-19-bounded-frontier-room-exploration-design.md`
- Verify: deployment scripts and NX runtime

- [ ] **Step 1: Update the design specification**

Document the adaptive radius, tile scheduler, per-cycle planning budget, stable exhaustion rule, bounded-interior coverage, and the explicit safety guarantee: completion covers all reachable frontiers inside the configured maximum radius under healthy localization/sensors; it does not guarantee inaccessible or occluded physical space.

- [ ] **Step 2: Run the full relevant regression suite**

```powershell
python -m pytest web/test_exploration_manager.py web/test_frontier_explore.py web/test_coverage_metrics.py web/test_mission_schema.py web/test_product_command.py -q
```

Expected: zero failures.

- [ ] **Step 3: Build a complete NX release**

Use the repository release/deploy scripts with a subsystem that includes the ROS `go2w_nav` install tree, not the prior partial web-only release. Verify the release contains `payload/install/setup.bash` and `payload/install/go2w_nav/lib/go2w_nav/mid360_nav_bridge_cpp` before switching `current`.

- [ ] **Step 4: Restart and perform read-only preflight**

Verify all service indicators are green, `/localization_pose` and `/odom` are fresh, `/mid360/points_nav` is publishing, Nav2 lifecycle nodes are active, C13 frames and YOLO are healthy, battery exceeds reserve, and the dog remains parked with velocity authorization false.

- [ ] **Step 5: Stop before physical exploration unless explicitly authorized**

Deployment verification ends at parked/healthy preflight. A new large-room physical search requires an explicit command because defaults allow up to 30 m radius and 30 minutes.

---

## Self-Review

- Spec coverage: mission contract, dynamic bounds, tile scheduling, probe reset, stable completion, coverage semantics, deployment completeness, and physical safety are each mapped to one task.
- Placeholder scan: no TBD/TODO/“similar to” steps; every code task includes named tests, exact files, commands, and expected outcomes.
- Type consistency: canonical fields use the same names from schema through task params, manager constructor, report, and tests.
- Existing uncommitted edits in `nx_navigation_arbiter.py` and `static/panel.html` remain out of this plan and must not be staged accidentally.
