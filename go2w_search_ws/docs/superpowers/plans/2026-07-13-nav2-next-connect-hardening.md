# Go2W Next-Connect Nav2 Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 下次 NX 与狗上电后，操作员完成一次安全站立确认，即可在 Panel 地图点击任意 5m 内可达点，由 Nav2 完成转向、避障、到达并持续显示 MID360 建图与 costmap 障碍。

**Architecture:** 保留 MID360+FastLIO 作为定位/建图与障碍主链，C13 只负责视觉；狗的轮速/IMU仅作为 NX 无对应传感器时的短时里程计反馈。运动控制改为可观测、可中断的 `jointLock(6) → BalanceStand → damping(7) → Move(0) → balance/locomotion(1/3)` 握手，停止统一回到 `jointLock(6)`；任何握手失败都允许在确认锁轮后安全重置，不再重启 motion 并连带重启 SLAM。Web 的导航就绪状态统一包含定位、scan、SDK、电池、驱动故障和运动状态，Panel 只在完整 gate 通过时发点。

**Tech Stack:** ROS 2 Humble, Unitree SDK2 Python, Nav2 DWB, FAST_LIO_ROS2, MID360, Python 3/pytest, vanilla JavaScript/Node contract tests, systemd.

---

### Task 1: Make the wheel activation protocol observable and recoverable

**Files:**
- Modify: `src/go2w_bridge/go2w_bridge/nx_sensor_node.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Modify: `docker/test_motion_scan_watchdog.py`

- [ ] **Step 1: Write failing feedback-contract tests**

Add assertions that `/wheel_feedback` carries `sport_mode`, `sport_progress`, and `gait_type`, and that `/dog_state` exposes `sport_mode`, `wheel_activation_phase`, `wheel_activation_elapsed_sec`, and `wheel_activation_mode_sequence`.

- [ ] **Step 2: Verify the tests fail**

Run: `python -m pytest docker/test_motion_scan_watchdog.py -q -k "sport_mode or activation"`

Expected: FAIL because progress/gait/activation diagnostics are absent.

- [ ] **Step 3: Implement validated feedback and activation telemetry**

Store the three Unitree sport fields under the sensor lock; reject non-finite progress and out-of-range byte values. Track activation phases `locked`, `balance_requested`, `settling`, `zero_committed`, `wheel_ready`, `failed`, with monotonic elapsed time and a de-duplicated mode sequence.

- [ ] **Step 4: Add a safe `reset_drive_fault` command**

Only clear recoverable faults (`wheel_no_response`, `wheel_mode_activation_timeout`) when state is `STOPPED` or `EMERGENCY`, sport mode is 6, mean absolute wheel speed is below 0.15 rad/s, SDK is ready, battery is valid, and no e-stop is in progress. Keep velocity zero and transition to `STOOD`, so balance authorization remains explicit.

- [ ] **Step 5: Run focused and full motion tests**

Run: `python -m pytest docker/test_motion_scan_watchdog.py -q`

Expected: all tests pass with no `Damp` or `StopMove` call in the motion source.

### Task 2: Unify navigation readiness across motion and localization

**Files:**
- Modify: `web/nx_web_server.py`
- Modify: `web/test_panel_navigation_contract.py`
- Modify: `web/test_point_navigation.py`

- [ ] **Step 1: Write a failing readiness test**

Create a node fixture where motion is `STOPPED`, scan and SDK are ready, but localization is stale. Assert `get_navigation_readiness()` returns `ready=false`, `reason="localization_stale"`, and includes the localization health fields.

- [ ] **Step 2: Verify the test fails**

Run: `python -m pytest web/test_panel_navigation_contract.py -q -k localization`

Expected: FAIL because current readiness ignores localization.

- [ ] **Step 3: Gate readiness on localization**

Read localization outside the motion-state lock, then include `localization_healthy`, `localization_reason`, and `localization_age_sec`. Preserve the first-failure ordering: stale dog state, SDK, scan, drive fault, battery, motion state, then localization.

- [ ] **Step 4: Verify API admission and active-goal cancellation**

Test that `/api/navigate` returns 409 while localization is unhealthy and that an active PointNavigationController goal is canceled by the same combined health gate.

- [ ] **Step 5: Run Web navigation tests**

Run: `python -m pytest web/test_panel_navigation_contract.py web/test_point_navigation.py web/test_navigation_arbitration.py -q`

Expected: all tests pass.

### Task 3: Make Panel goal admission and recovery explicit

**Files:**
- Modify: `web/static/panel.html`
- Modify: `web/test_panel_nav_state.js`
- Modify: `web/test_panel_navigation_contract.py`
- Modify: `web/nx_web_server.py`

- [ ] **Step 1: Write failing Panel contract tests**

Assert a map click is rejected locally when `navigation.ready` is false, displays the backend reason, and never calls `/api/navigate`. Assert the recovery button calls `/api/reset_drive_fault` and is only enabled for recoverable locked faults.

- [ ] **Step 2: Verify the tests fail**

Run: `node web/test_panel_nav_state.js && python -m pytest web/test_panel_navigation_contract.py -q`

Expected: at least one new assertion fails.

- [ ] **Step 3: Cache and render the full navigation gate**

Update the cache from both WebSocket status and `/api/status`. Before `sendNavGoal`, require `ready=true`, healthy localization, a finite map pose, and no pending recovery. Keep the existing 5m radius limit and goal yaw pointing from the current robot pose toward the clicked point, including rear goals.

- [ ] **Step 4: Expose safe fault reset through the arbiter**

Add `NxRobotBridge.reset_drive_fault()`, POST `/api/reset_drive_fault`, and a Panel button. The endpoint must cancel/drain point and room navigation before publishing the pose command.

- [ ] **Step 5: Verify Panel contracts**

Run: `node web/test_panel_nav_state.js && node web/test_map_contract.js && python -m pytest web/test_panel_navigation_contract.py -q`

Expected: all tests pass.

### Task 4: Add a read-only next-connect preflight and bounded wheel probe

**Files:**
- Create: `tools/nav2_preflight.py`
- Modify: `tools/probe_angular_response.py`
- Create: `tools/test_nav2_preflight.py`
- Modify: `docker/deploy_nav2_bprime.sh`

- [ ] **Step 1: Write failing preflight parser tests**

Feed recorded `/api/status` and topic/action samples into a pure evaluator. Require checks for MID360 scan, FastLIO localization, map→base_link TF, Nav2 action server, costmap freshness, motion SDK, battery, sport mode 6, zero wheels, and no drive fault.

- [ ] **Step 2: Verify the tests fail**

Run: `python -m pytest tools/test_nav2_preflight.py -q`

Expected: FAIL because the evaluator does not exist.

- [ ] **Step 3: Implement read-only JSON preflight**

Default mode performs no robot command and exits nonzero on any failed hard gate. `--json` prints machine-readable checks; human output prints exact recovery actions. Do not instantiate SportClient.

- [ ] **Step 4: Keep the wheel probe opt-in and bounded**

Require `--execute`, cap angular speed at 0.15 rad/s, stop after 0.15 seconds of real wheel response, always publish repeated zero, and report sport-mode sequence plus final jointLock confirmation. Refuse to start unless the initial state is `STOPPED` and the preflight hard gates pass.

- [ ] **Step 5: Deploy tools contractually**

Add the files to the NX deployment script and test the deployment paths without contacting hardware.

### Task 5: Verify rear-turn, obstacle, and mapping configuration contracts

**Files:**
- Modify: `docker/test_map_odom_fuser_performance_contract.py`
- Modify: `web/test_map_contract.js`
- Modify: `docs/HANDOVER_2026-07-10.md`

- [ ] **Step 1: Add rear-goal configuration assertions**

Assert DWB `max_vel_theta=0.15`, `min_speed_theta=0.15`, `acc_lim_theta=1.5`, `decel_lim_theta=-1.5`, RotateToGoal is enabled, and both goal checker and controller tolerances match.

- [ ] **Step 2: Add obstacle/mapping display assertions**

Assert MID360 is the only costmap observation source, both local/global costmaps send full updates, `costmap_bridge` is part of the SLAM unit, forced WebSocket costmap messages bypass throttling, and Panel draws costmap before robot/goal overlays.

- [ ] **Step 3: Run configuration contracts**

Run: `python -m pytest docker/test_map_odom_fuser_performance_contract.py docker/test_costmap_bridge_service_contract.py -q && node web/test_map_contract.js`

Expected: all tests pass.

- [ ] **Step 4: Document the next physical acceptance sequence**

Record: power on → verify jointLock/zero wheels → operator stand confirmation → arm/confirm → bounded angular probe → rear Panel goal → obstacle Panel goal → verify SUCCEEDED, final mode 6, zero wheels, visible orange map and red obstacle overlay. Any unexpected motion uses `/api/e_stop`; never call `Damp` or `SwitchGait`.

### Task 6: Final offline verification

**Files:**
- Verify all modified files above.

- [ ] **Step 1: Run all software tests**

Run: `python -m pytest web docker tools/test_nav2_preflight.py -q`

Expected: all tests pass. The hardware-only `test_lidar_topics.py` is intentionally excluded because Unitree SDK is unavailable on Windows.

- [ ] **Step 2: Run JavaScript contracts**

Run: `node web/test_panel_nav_state.js && node web/test_map_contract.js`

Expected: both commands pass.

- [ ] **Step 3: Run static safety checks**

Run: `rg -n "Damp\(|StopMove\(|SwitchGait\(" src/go2w_bridge/go2w_bridge/nx_motion_node.py`

Expected: no executable call sites; only disabled documentation/helper references are allowed.

- [ ] **Step 4: Inspect the final diff**

Run: `git diff --check && git diff --stat`

Expected: no whitespace errors; unrelated user changes remain untouched. Do not commit or stage without the user's explicit request.

