# Go2W SDK-Aligned Motion State Machine Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task by task.

**Goal:** Refactor `nx_motion_node.py` so physical state is derived from Go2W telemetry, pose RPC results remain diagnostic, and nonzero wheel commands are accepted only after an explicit, feedback-confirmed balance transition.

**Architecture:** Keep the existing `/cmd_pose`, `/cmd_vel`, and `/dog_state` compatibility surface, but separate it from a ROS-independent layered state model. A firmware profile translates the raw, undocumented `SportModeState.mode` values observed on this robot into semantic physical modes. Pose commands open bounded transitions; feedback completes them. The legacy workflow state remains temporarily for the current frontend, while `/dog_state` also publishes the new connection, physical, motion, transition, and safety layers.

**Tech Stack:** Python 3, ROS 2 `rclpy`, Unitree `unitree_sdk2py`, pytest, Python AST contract tests.

---

### Task 1: Lock the layered state contract with offline tests

**Files:**
- Modify: `go2w_search_ws/docker/test_motion_scan_watchdog.py`
- Test: `go2w_search_ws/docker/test_motion_scan_watchdog.py`

- [x] Add an AST loader for the ROS-independent model and its enums.
- [x] Test the firmware-profile mapping: raw mode `6` is joint lock, `1/3` are wheel-capable, `7` is damping, and unknown values stay unknown.
- [x] Test that actual motion is computed from reported wheel speed, not from the requested velocity.
- [x] Test that a transition succeeds only after matching telemetry and that velocity authorization requires confirmed balance plus normal safety.
- [x] Test that a nonzero `BalanceStand` RPC result does not immediately abort or send an inverse pose action.
- [x] Run the focused tests and preserve the expected RED result before implementation.

### Task 2: Add the ROS-independent Go2W state model

**Files:**
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Test: `go2w_search_ws/docker/test_motion_scan_watchdog.py`

- [x] Define independent enums for link, physical mode, actual motion, transition, and safety.
- [x] Define an explicit `Go2WModeProfile` for robot-specific raw mode values; document that raw `SportModeState.mode` is not the SDK `GetState` gait enum.
- [x] Implement `Go2WStateModel.observe_feedback`, transition lifecycle methods, safety updates, and a serializable snapshot.
- [x] Keep all model code independent of ROS and the Unitree SDK so it can run in fault-injection tests.
- [x] Run the model tests and confirm GREEN.

### Task 3: Integrate authoritative feedback and observability

**Files:**
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Modify: `go2w_search_ws/docker/test_motion_scan_watchdog.py`

- [x] Create the model before SDK startup and mark the link online only when the SDK is ready.
- [x] Feed every validated wheel/sport feedback sample into the model.
- [x] Publish `state_model_version`, `link_state`, `physical_mode`, `motion_state`, `transition_state`, `transition_operation`, `safety_state`, `raw_sport_mode`, and `raw_gait_type` while retaining the legacy `state` field.
- [x] Ensure unknown or stale physical feedback fails closed for velocity.
- [x] Run focused tests.

### Task 4: Refactor pose and velocity transitions

**Files:**
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Modify: `go2w_search_ws/docker/test_motion_scan_watchdog.py`

- [x] Make `stand` and `balance` feedback-confirmed transitions; gate `sit` from the documented StandDown source modes and never write physical mode from an RPC result.
- [x] For `BalanceStand`, retain the RPC code for diagnostics and wait for bounded authoritative feedback even when the code is nonzero.
- [x] On balance timeout, latch the fault and keep zero velocity; do not automatically issue `StandUp`, `Damp`, or another inverse pose action.
- [x] Remove implicit `BalanceStand` from the MOVING loop; reject nonzero velocity unless wheel-capable feedback is already confirmed.
- [x] Require matching telemetry for startup adoption, `adopt_stand`, `adopt_balance`, and human confirmations.
- [x] Keep e-stop and stop paths zero-velocity-first without unloading the robot.
- [x] Run all motion safety tests.

### Task 5: Verify the offline refactor

**Files:**
- Verify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Verify: `go2w_search_ws/docker/test_motion_scan_watchdog.py`

- [x] Run `python -m pytest go2w_search_ws/docker/test_motion_scan_watchdog.py -q`.
- [x] Run `python -m py_compile go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`.
- [x] Review the final diff for accidental edits and forbidden startup/automatic pose commands.
- [x] Report that verification is offline only and that live deployment requires a separate, restrained hardware validation sequence.
