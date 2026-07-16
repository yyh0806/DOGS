# Nav2 Low-Angular Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stabilize FastLIO during Nav2 recovery and prove a real 5 metre autonomous navigation run.

**Architecture:** Keep the complete Nav2 safety chain and the repaired velocity-smoother wiring. Limit DWB and recovery angular motion, relax the progress checker to match the robot's indoor speed, retain FastLIO online extrinsic estimation, and verify each layer from low-speed rotation through 1 metre and 5 metre goals.

**Tech Stack:** ROS 2 Humble, Nav2 DWB, nav2_velocity_smoother, FAST_LIO_ROS2, pytest contract tests, systemd, tf2.

---

### Task 1: Lock navigation stability parameters with a failing contract

**Files:**
- Modify: `docker/test_map_odom_fuser_performance_contract.py`
- Test: `docker/test_map_odom_fuser_performance_contract.py`

- [ ] **Step 1: Write the failing parameter contract**

Add a test that reads `src/go2w_nav/config/nav2_params_3d.yaml` and asserts these exact strings:

```python
def test_nav_recovery_is_limited_to_fastlio_safe_angular_motion():
    params = (ROOT / "src/go2w_nav/config/nav2_params_3d.yaml").read_text(
        encoding="utf-8"
    )
    assert "max_vel_theta: 0.3" in params
    assert "required_movement_radius: 0.2" in params
    assert "movement_time_allowance: 20.0" in params
    assert "max_velocity: [0.6, 0.0, 0.3]" in params
    assert "min_velocity: [-0.6, 0.0, -0.3]" in params
    assert "max_accel: [0.5, 0.0, 0.5]" in params
    assert "max_decel: [-1.0, 0.0, -0.5]" in params
    assert "max_rotational_vel: 0.3" in params
    assert "min_rotational_vel: 0.1" in params
    assert "rotational_acc_lim: 0.5" in params
```

- [ ] **Step 2: Run the contract and verify RED**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
```

Expected: the new test fails because the current angular limits are `1.0` and progress checking is `0.5 m / 10 s`.

### Task 2: Implement the approved Nav2 limits

**Files:**
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Test: `docker/test_map_odom_fuser_performance_contract.py`

- [ ] **Step 1: Apply the minimal parameter changes**

Set the controller parameters to:

```yaml
FollowPath:
  max_vel_theta: 0.3
progress_checker:
  required_movement_radius: 0.2
  movement_time_allowance: 20.0
```

Set the velocity smoother limits to:

```yaml
max_velocity: [0.6, 0.0, 0.3]
min_velocity: [-0.6, 0.0, -0.3]
max_accel: [0.5, 0.0, 0.5]
max_decel: [-1.0, 0.0, -0.5]
```

Set the recovery limits under `behavior_server.ros__parameters` to:

```yaml
max_rotational_vel: 0.3
min_rotational_vel: 0.1
rotational_acc_lim: 0.5
```

- [ ] **Step 2: Run tests and verify GREEN**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
python -m py_compile src/go2w_nav/launch/nav2_3d.launch.py src/go2w_bridge/go2w_bridge/nx_sensor_node.py src/go2w_bridge/go2w_bridge/map_odom_fuser.py
```

Expected: all contract tests pass and all Python files compile.

### Task 3: Deploy exact artifacts and verify runtime configuration

**Files:**
- Deploy: `src/go2w_nav/config/nav2_params_3d.yaml` to `/home/nx/go2w_ws/install/go2w_nav/share/go2w_nav/config/nav2_params_3d.yaml`
- Verify: `/home/nx/ws_livox/src/FAST_LIO_ROS2/config/mid360.yaml`

- [ ] **Step 1: Copy the Nav2 configuration to NX**

Use the established Paramiko/SFTP workflow and preserve file mode `0644`.

- [ ] **Step 2: Enforce the validated FastLIO setting**

Verify this exact line exists on NX:

```yaml
extrinsic_est_en: true
```

If it does not, change only that value before restarting FastLIO.

- [ ] **Step 3: Restart and verify runtime nodes**

Restart `fastlio` and `nav2-3d`, wait for initialization, then verify:

```bash
systemctl is-active fastlio map-odom-fuser nav2-3d go2w-motion
ros2 lifecycle get /velocity_smoother
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /controller_server progress_checker.required_movement_radius
ros2 param get /controller_server progress_checker.movement_time_allowance
ros2 param get /behavior_server max_rotational_vel
```

Expected: all services are `active`, smoother is `active [3]`, and values are `0.3`, `0.2`, `20.0`, and `0.3`.

### Task 4: Verify low-speed rotation does not destabilize FastLIO

**Files:**
- Runtime evidence: NX TF and `/tmp` diagnostic output only

- [ ] **Step 1: Capture starting poses**

Capture `odom -> base_link` and `camera_init -> body` before motion.

- [ ] **Step 2: Command a bounded rotation and stop**

Publish `angular.z: 0.2` at 10 Hz for 5 seconds, then publish a zero Twist.

- [ ] **Step 3: Validate localization stability**

Capture both transforms again. Require FastLIO translation change below 1 metre and absolute roll/pitch below 15 degrees. If either fails, stop execution and follow the design rollback section.

### Task 5: Run and evaluate the 1 metre Nav2 gate

**Files:**
- Runtime script: `/home/nx/go2w_ws/diagnose_nav2_goal.sh`

- [ ] **Step 1: Reset localization and costmaps**

Restart FastLIO, wait for `map -> base_link`, and clear both costmaps.

- [ ] **Step 2: Execute the 1 metre goal**

Run:

```bash
bash /home/nx/go2w_ws/diagnose_nav2_goal.sh 1 0 90
```

- [ ] **Step 3: Check the gate**

Require `linear.x > 0.01`, no `map` TF loss, no FastLIO metre-scale jump, and a successful or goal-tolerance-compliant arrival. Always publish zero Twist after the script.

### Task 6: Run final 5 metre navigation and completion audit

**Files:**
- Runtime evidence: NX `/tmp/nav2diag_*`

- [ ] **Step 1: Reset and capture authoritative start state**

Restart FastLIO, clear costmaps, and record starting `odom -> base_link`, `camera_init -> body`, fuser CPU, and service states.

- [ ] **Step 2: Execute the 5 metre goal**

Run:

```bash
bash /home/nx/go2w_ws/diagnose_nav2_goal.sh 5 0 120
```

- [ ] **Step 3: Audit every success requirement**

Require all of the following current evidence:

- NavigateToPose result is `SUCCEEDED`.
- Captured `/cmd_vel` contains `linear.x > 0.01`.
- Start/end `odom -> base_link` displacement is approximately 5 metres.
- `camera_init -> body` remains bounded and `map` TF remains available.
- `map_odom_fuser.py` CPU is below 20%.
- `fastlio`, `map-odom-fuser`, `nav2-3d`, and `go2w-motion` remain active.

- [ ] **Step 4: Run fresh local verification**

Run:

```powershell
python -m pytest docker/test_map_odom_fuser_performance_contract.py -q
git diff --check
```

Expected: tests pass and no whitespace errors are introduced.
