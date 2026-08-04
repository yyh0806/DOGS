# Nav2 Route Consistency And Three-Trip Shuttle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Go2W use one coherent map/odom transform chain, execute the same forward-only velocity model that Nav2 plans, retain mapped obstacles across a 20 m corridor, and provide a six-leg `(0,0) <-> (20,0)` shuttle that advances only after each leg succeeds.

**Architecture:** SLAM Toolbox exclusively publishes `map -> odom`; `map_odom_fuser` exclusively publishes `odom -> base_link` and composes `/localization_pose` with SLAM's correction. The persistent SLAM grid is restored to the 50 m rolling global costmap while Navfn may traverse unknown cells. A standalone HTTP shuttle tool drives the existing authenticated `/api/navigate` owner one goal at a time and polls the matching point-navigation generation to terminal success.

**Tech Stack:** ROS 2 Humble, Nav2, SLAM Toolbox, Python 3, pytest, systemd, YAML, urllib HTTP client.

---

### Task 1: Enforce single TF ownership

**Files:**
- Modify: `docker/test_persistent_exploration_contract.py`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`
- Modify: `docker/test_mid360_only_contract.py`
- Modify: `docker/test_release_deploy_contract.py`
- Modify: `docker/test_nav_health_supervisor.py`
- Modify: `docker/build_release.sh`
- Modify: `docker/deploy_release.sh`
- Modify: `docker/go2w-sensor.service`
- Modify: `docker/go2w-slam-nav.service`
- Modify: `docker/bringup_slam_nav2.sh`
- Modify: `src/go2w_bridge/go2w_bridge/map_odom_fuser.py`

- [x] **Step 1: Write the failing ownership contracts**

Assert that the permanent sensor service publishes diagnostic `/wheel_odom` without TF, the bringup never starts a second `nx_sensor_node`, and the fuser is launched with:

```python
assert "-p publish_odom_tf:=false" in sensor_service
assert "-p odom_topic:=/wheel_odom" in sensor_service
assert "start_transient wheel-odom" not in bringup
assert "Conflicts=go2w-sensor.service" not in nav_service
assert "Wants=go2w-sensor.service" in nav_service
assert "disable go2w-sensor.service" not in deploy
assert "-p publish_map_to_odom:=false" in bringup
assert "-p use_slam_pose:=true" in bringup
assert "50_000_000" not in fuser
```

- [x] **Step 2: Run the focused tests and verify they fail for the current duplicate publishers**

Run:

```powershell
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py
```

Expected: ownership assertions fail on the current service/bringup/future-stamp configuration.

- [x] **Step 3: Make each transform edge have one publisher**

Configure `go2w-sensor.service` to run:

```text
-p publish_imu:=false -p publish_scan:=false -p publish_odom:=true
-p publish_odom_tf:=false -p odom_topic:=/wheel_odom
```

Remove the transient duplicate sensor from bringup, make `go2w-slam-nav.service` depend on (rather than conflict with) the bounded sensor service, and keep that service enabled/restarted for both `nav` and `all` releases. Launch the fuser with `publish_map_to_odom:=false` and `use_slam_pose:=true`, make those its fail-safe defaults, and stamp fuser output at callback time without the artificial 50 ms future offset.

- [x] **Step 4: Run the focused tests and verify the ownership contracts pass**

Run the command from Step 2 and require zero failures in the ownership tests changed by this task.

### Task 2: Align the planned and executable velocity/BT contracts

**Files:**
- Modify: `docker/test_mid360_only_contract.py`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `src/go2w_nav/launch/nav2_3d.launch.py`

- [x] **Step 1: Write failing contracts for forward-only navigation and the safe BT**

Require all autonomous layers to agree:

```python
assert "min_vel_x: 0.0" in params
assert "min_velocity: [0.0, 0.0, -0.5]" in params
assert "max_vel_x: 0.6" in params
assert "max_velocity: [0.6, 0.0, 0.5]" in params
assert "default_nav_to_pose_bt_xml" in launch
assert "navigate_to_pose_dynamic_safe.xml" in launch
```

Also require the active behavior server to expose only the non-motion `wait` recovery.

- [x] **Step 2: Run the focused tests and verify the new contracts fail**

Run:

```powershell
python -m pytest -q docker/test_mid360_only_contract.py docker/test_map_odom_fuser_performance_contract.py
```

Expected: fail on negative DWB velocity, mismatched smoother maximum, disabled custom BT, and unsafe behavior plugins.

- [x] **Step 3: Apply the minimal aligned configuration**

Set DWB `min_vel_x` to `0.0`, keep the validated indoor maximum at `0.6 m/s`, align DWB and the smoother to the existing stable `0.5 rad/s` turn cap, restore the launch rewrite to the installed safe BT, and remove `spin`, `backup`, and `drive_on_heading` from `behavior_plugins`.

- [x] **Step 4: Re-run the focused tests and require green**

Run the command from Step 2 and confirm the velocity/BT assertions pass.

### Task 3: Retain obstacles across the complete 20 m shuttle corridor

**Files:**
- Modify: `docker/test_persistent_exploration_contract.py`
- Modify: `docker/test_map_odom_fuser_performance_contract.py`
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`

- [x] **Step 1: Write the 20 m global-map contract**

Require a 50 m rolling window, a persistent static map plus live obstacle layer, unknown preservation, and an unknown-capable Navfn planner:

```python
assert "rolling_window: true" in global_section
assert float(width) >= 50.0
assert plugins == ["static_layer", "obstacle_layer", "inflation_layer"]
assert 'map_topic: "/map_frontier"' in global_section
assert "track_unknown_space: true" in global_section
assert "allow_unknown: true" in planner_section
```

- [x] **Step 2: Run the focused tests and observe failure on the missing persistent layer**

Run:

```powershell
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py
```

- [x] **Step 3: Restore the persistent static layer without shrinking the corridor window**

Keep the 50 m rolling window, add `static_layer` subscribed to `/map_frontier`, change `track_unknown_space` to true, and retain `allow_unknown: true` so the first outbound leg can enter not-yet-scanned cells while later legs reuse mapped obstacles.

- [x] **Step 4: Re-run the focused tests and require green for the map strategy**

Run the command from Step 2 and inspect every failure rather than weakening unrelated safety assertions.

### Task 4: Add a sequential three-trip shuttle client

**Files:**
- Create: `tools/nav_shuttle.py`
- Create: `tools/test_nav_shuttle.py`
- Modify: `docker/build_release.sh`

- [x] **Step 1: Write failing route and runner tests**

Cover six ordered legs, reverse yaw, stop-on-first-failure, generation matching, and dry-run safety:

```python
goals = build_shuttle_goals((0, 0), (20, 0), trips=3)
assert [(g.x, g.y) for g in goals] == [
    (20, 0), (0, 0), (20, 0), (0, 0), (20, 0), (0, 0)
]
assert runner.run(goals)["legs_completed"] == 6
```

- [x] **Step 2: Run the new test module and verify RED**

Run:

```powershell
python -m pytest -q tools/test_nav_shuttle.py
```

Expected: import failure because `tools/nav_shuttle.py` does not exist.

- [x] **Step 3: Implement the minimal safe HTTP runner**

Implement a dry-run-by-default CLI with explicit `--execute`, finite coordinate validation, positive bounded trip count, one `/api/navigate` POST per leg, `/api/status` polling until the matching generation is terminal, and `/api/stop` on transport/timeout failure.

- [x] **Step 4: Add the tool to release packaging and run its tests**

Require `tools/test_nav_shuttle.py` to pass and verify `docker/build_release.sh` copies `tools/nav_shuttle.py`.

### Task 5: Full verification

**Files:**
- Verify all modified files above.

- [x] **Step 1: Run focused navigation, motion, gateway, and shuttle tests**

```powershell
python -m pytest -q docker/test_persistent_exploration_contract.py docker/test_map_odom_fuser_performance_contract.py docker/test_mid360_only_contract.py src/go2w_bridge/test/test_motion_controller.py src/go2w_bridge/test/test_motion_safety.py web/test_navigation_gateway.py tools/test_nav_shuttle.py
```

- [x] **Step 2: Run broader offline regression suites**

```powershell
python -m pytest -q docker src/go2w_bridge/test web tools
```

Report any pre-existing or environment-only failures separately; do not claim full green unless the command exits zero.

- [x] **Step 3: Run static checks**

```powershell
python -m py_compile tools/nav_shuttle.py
git diff --check
git status --short
```

- [x] **Step 4: Review the final diff against the five root causes**

Confirm one TF owner per edge, one velocity model, the safe BT, persistent 20 m obstacle memory, and success-gated six-leg sequencing. Do not deploy or move the physical robot without a separate explicit execution request.
