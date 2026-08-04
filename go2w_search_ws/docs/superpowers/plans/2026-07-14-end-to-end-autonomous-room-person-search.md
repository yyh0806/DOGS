# Go2-W End-to-End Autonomous Room Person Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make arrow-key driving, click-to-Nav2 obstacle avoidance, current-room search, person photography/localization, map annotation, and the exact Chinese voice command work as one feedback-confirmed Go2-W workflow.

**Architecture:** The motion node owns a single Unitree SDK lease and exposes an explicit drive-session lifecycle. Web/manual/Nav2/task producers request `manual_start`, `nav_start`, or `park`; the motion node confirms every transition from `/sportmodestate`, wheel `dq`, battery, scan freshness, and sport errors before authorizing velocity. `NavigationArbiter` owns drive sessions for both point goals and room tasks. The current-room voice intent uses bounded frontier exploration from the live map pose and does not trust uncalibrated placeholder rooms. Existing YOLO, LiDAR person localization, artifact storage, WebSocket markers, and map rendering remain the detection pipeline.

**Tech Stack:** Python 3, ROS 2 Humble/rclpy, Unitree Go2-W SDK 1.0.1, Nav2 `NavigateToPose`, Fast-LIO localization, Mid-360 LaserScan, YOLO/OpenCV, HTTP/WebSocket frontend, pytest/unittest contract tests.

---

### Task 0: Remove MID360 floor-carpet obstacles and establish performance gates

**Files:**
- Modify: `go2w_search_ws/docker/test_mid360_only_contract.py`
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/mid360_nav_bridge.py`
- Modify: `go2w_search_ws/docker/test_map_odom_fuser_performance_contract.py`
- Modify: `go2w_search_ws/src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `go2w_search_ws/tools/diag_mid360_filter.py`
- Modify: `go2w_search_ws/tools/diag_nav2_plan.py`

- [ ] Add a failing field-regression test with floor returns in `z=[-0.60,-0.50]` and obstacle returns above `z=-0.45`; require floor rejection while retaining the obstacle surface.
- [ ] Change the bridge height gate to the field-validated `-0.45 m` threshold and document that the leveled floor is centered near `-0.57 m`.
- [ ] Restart only `mid360-nav-bridge` and both Nav2 costmaps, then require the filtered point cloud floor-band ratio to fall below 5% and the scan minimum to describe the physical box rather than the fixed floor ring.
- [ ] Add failing configuration contracts for a click-go performance profile: planning response below 1 s on a 12 m rolling map, controller frequency at least 15 Hz, linear command up to 0.4 m/s, and a field-validated angular limit that never exceeds the stable Go2-W wheel-mode response.
- [ ] Tune one parameter group at a time and record planner latency, path collision samples, time-to-first-command, average speed, minimum obstacle clearance, terminal result, and final parked state.
- [ ] Do not raise velocity until a straight-behind-box goal produces a collision-free path and a monitored low-speed run succeeds.

### Task 0A: Separate persistent exploration mapping from live collision avoidance

**Files:**
- Modify: `go2w_search_ws/src/go2w_nav/config/slam_toolbox_online.yaml`
- Modify: `go2w_search_ws/src/go2w_nav/launch/slam_online.launch.py`
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/map_odom_fuser.py`
- Modify: `go2w_search_ws/web/costmap_bridge.py`
- Modify: `go2w_search_ws/web/test_frontier_explore.py`
- Modify: `go2w_search_ws/docker/test_map_odom_fuser_performance_contract.py`

- [ ] Add failing tests that the persistent `/map_frontier` source is not the rolling global costmap and that exactly one node owns `map->odom`.
- [ ] Feed the corrected `/scan_mid360` directly to one online SLAM instance; remove the obsolete `PointCloud2` conversion of Livox `CustomMsg`.
- [ ] Let online SLAM own `map->odom` during exploration while the wheel/IMU fuser owns only `odom->base_link`; publish localization from the resulting TF chain.
- [ ] Keep Nav2 local obstacle clearing live and bounded while the persistent occupancy grid grows across rooms and loop closures.
- [ ] Add frontier reachability filtering through `ComputePathToPose`, blacklist rejected frontiers with a bounded retry count, and finish only when reachable frontier information gain is exhausted or the mission budget expires.
- [ ] Verify a synthetic multi-room map produces a deterministic sequence of reachable frontiers without selecting rolling-window boundaries.

### Task 1: Specify the feedback-confirmed drive session

**Files:**
- Modify: `go2w_search_ws/docker/test_motion_scan_watchdog.py`
- Modify: `go2w_search_ws/src/go2w_bridge/go2w_bridge/nx_motion_node.py`

- [ ] Add failing tests for `DriveSessionModel`: parked mode is raw sport mode 6 with stopped wheels; active mode is raw mode 1/3; stale feedback, sport error, low battery, or missing scan cannot activate autonomous drive.
- [ ] Add failing tests that `manual_start`/`nav_start` request exactly one `BalanceStand`, wait for mode 1/3, and only then authorize velocity.
- [ ] Add failing tests that `park` sends zero then exactly one `StandUp`, waits for mode 6 plus stopped wheels, and does not repeatedly alternate pose modes.
- [ ] Add failing tests that startup adopts mode 6 as parked and parks an already-active mode 1/3 instead of leaving wheel balance active.
- [ ] Remove the disproven unleased second `SportClient`; route `Move`, `StopMove`, `BalanceStand`, and `StandUp` through the one leased client.
- [ ] Subscribe to `/motion_session` (`std_msgs/String`) and accept only `manual_start`, `manual_stop`, `nav_start`, `nav_stop`, `park`, and `estop`.
- [ ] Publish `drive_session`, `drive_session_owner`, `drive_session_phase`, and transition error in `/dog_state`.
- [ ] Run `python -m pytest go2w_search_ws/docker/test_motion_scan_watchdog.py -q`.

### Task 2: Split “can activate” from “currently drive-ready”

**Files:**
- Modify: `go2w_search_ws/web/test_navigation_arbitration.py`
- Modify: `go2w_search_ws/web/test_panel_navigation_contract.py`
- Modify: `go2w_search_ws/web/nx_web_server.py`

- [ ] Add failing tests that mode 6, stopped wheels, fresh dog state, SDK, scan, battery, no drive fault, and healthy localization returns `activatable: true` but `ready: false`.
- [ ] Add failing tests that mode 1/3 plus an active drive session returns both `activatable: true` and `ready: true`.
- [ ] Add failing tests that stale dog feedback, low battery, protection/fault, stale scan, or unhealthy localization returns `activatable: false` with a precise reason.
- [ ] Cache the new drive-session fields from `/dog_state` and expose them from `/api/status`.
- [ ] Add a `/motion_session` publisher and `NxRobotBridge.start_drive_session(owner)`, `park_drive_session(reason)`, and bounded `wait_drive_ready(timeout)` helpers.
- [ ] Keep transient zero `/cmd_vel_nav` inside an active Nav2 session as zero only; never map each zero command to `StandUp`.
- [ ] Run the two web test modules.

### Task 3: Give NavigationArbiter ownership of the physical drive session

**Files:**
- Modify: `go2w_search_ws/web/test_navigation_arbitration.py`
- Modify: `go2w_search_ws/web/nx_navigation_arbiter.py`
- Modify: `go2w_search_ws/web/nx_web_server.py`

- [ ] Add failing tests that a point goal first drains old task ownership, activates `nav`, waits for feedback-confirmed readiness, and only then submits to Nav2.
- [ ] Add failing tests that rejected activation never submits a goal and parks fail-closed.
- [ ] Add failing tests that `succeeded`, `aborted`, `rejected`, `timed_out`, `canceled`, quarantine, and health loss park only after action ownership is terminal.
- [ ] Add failing tests that a room/task batch holds one Nav2 session across all internal waypoints and parks once when the worker drains.
- [ ] Add failing tests that emergency stop bypasses normal handoff and operator stand cancels autonomy before parking.
- [ ] Wire the point-navigation state callback and task-worker finalizer to arbiter terminal notifications without calling callbacks under controller locks.
- [ ] Run `python -m pytest go2w_search_ws/web/test_navigation_arbitration.py go2w_search_ws/web/test_point_navigation.py -q`.

### Task 4: Make the arrow keys a bounded manual drive session

**Files:**
- Modify: `go2w_search_ws/web/test_panel_navigation_contract.py`
- Modify: `go2w_search_ws/web/nx_web_server.py`
- Modify: `go2w_search_ws/web/static/panel.html`

- [ ] Add failing contracts that the first non-zero `/api/move` requests `manual_start` and reports activation progress rather than silently dropping the command.
- [ ] Buffer the latest finite manual command during activation and publish it only after dog feedback says drive-ready.
- [ ] Treat key release/zero as `manual_stop`: publish zero immediately, then request one park transition after a short bounded debounce.
- [ ] Return explicit JSON (`accepted`, `phase`, `reason`) for the frontend and display activation/failure state.
- [ ] Preserve emergency-stop precedence and finite/clamped velocity validation.
- [ ] Run web and frontend contract tests.

### Task 5: Admit click-to-go from parked mode and verify obstacle inputs

**Files:**
- Modify: `go2w_search_ws/web/test_navigation_arbitration.py`
- Modify: `go2w_search_ws/web/test_panel_navigation_contract.py`
- Modify: `go2w_search_ws/web/nx_web_server.py`
- Modify: `go2w_search_ws/docker/diagnose_nav2_goal.sh`

- [ ] Change `/api/navigate` admission from `ready` to `activatable`; activation itself remains inside the arbiter.
- [ ] Return the goal generation only after drive activation succeeds and Nav2 accepts submission.
- [ ] Extend diagnostics to verify `/scan_mid360`, local/global costmaps, map→base_link TF, controller/planner lifecycle state, `/navigate_to_pose`, and non-stale `/cmd_vel_nav`.
- [ ] Add terminal parking and surface Nav2 result/reason in WebSocket and `/api/status`.
- [ ] Run point-navigation and panel navigation tests.

### Task 6: Route “这个房间” to bounded live-map exploration

**Files:**
- Modify: `go2w_search_ws/web/test_product_command.py`
- Modify: `go2w_search_ws/web/test_product_room_orchestrator.py`
- Modify: `go2w_search_ws/web/test_frontier_explore.py`
- Modify: `go2w_search_ws/web/nx_product_command.py`
- Modify: `go2w_search_ws/web/nx_web_server.py`
- Modify: `go2w_search_ws/web/nx_room_orchestrator.py`

- [ ] Add failing tests that “去搜索这个房间，把所有人标注出来” produces `search_room`, `room=__current__`, `search_strategy=frontier_explore`, photos, LiDAR range, and map marking.
- [ ] Never resolve `__current__` to an uncalibrated nearest placeholder room; named rooms still require `calibrated: true`.
- [ ] Capture the mission start pose and restrict current-room frontier candidates by a configurable radius and maximum mission time.
- [ ] Treat no remaining bounded frontier as successful search completion, not a navigation failure.
- [ ] Keep the same Nav2 session across frontier goals and park at the task terminal state.
- [ ] Run product command/orchestrator/frontier tests.

### Task 7: Verify person evidence and map annotation contracts

**Files:**
- Modify: `go2w_search_ws/web/test_ai_snapshot_contract.py`
- Modify: `go2w_search_ws/web/test_person_localizer.py`
- Modify: `go2w_search_ws/web/test_person_mission.py`
- Modify: `go2w_search_ws/web/test_map_contract.js`
- Modify as required: `go2w_search_ws/web/nx_room_orchestrator.py`
- Modify as required: `go2w_search_ws/web/static/map.js`

- [ ] Verify only YOLO `person` detections enter the mission snapshot and copied frames match bbox coordinates.
- [ ] Verify LiDAR range projection produces finite map-frame `(x,y)` using the live robot pose and rejects missing/stale scan.
- [ ] Verify deduplication, full-frame/crop artifacts, stable marker IDs, photo URLs, confidence, and mission IDs.
- [ ] Verify `person_markers` WebSocket updates render at map coordinates and persist in the mission report.
- [ ] Run the AI/person/map contract tests, including Node tests when Node is installed.

### Task 8: Deploy safely and perform staged physical acceptance

**Files:**
- Modify if required: `go2w_search_ws/docker/deploy_nx.sh`
- Modify if required: `go2w_search_ws/docker/deploy_nx_web.sh`
- Modify: `go2w_search_ws/web/verify_product_room_person_search.sh`

- [ ] Run the full local non-hardware suite and Python compilation before deployment.
- [ ] Copy files to timestamped NX backups, deploy, restart only affected services, and verify service logs contain no traceback.
- [ ] With no non-zero command, verify startup ends in mode 6, wheel `dq` below threshold, no sport error, and `drive_session=parked`.
- [ ] After the operator confirms the area is clear, test one low-speed arrow pulse; require mode 1/3 and wheel response, then key release must return mode 6 with stopped wheels.
- [ ] Test a short visible Nav2 goal; require action acceptance, obstacle-aware costmap/controller output, measured displacement, successful/explicit terminal result, and final mode 6 parking.
- [ ] Test bounded current-room search with a staged person; require photos, finite map marker, mission report, and terminal parking.
- [ ] Test the exact voice command end-to-end and retain `/api/status`, mission report, and service-log evidence.
