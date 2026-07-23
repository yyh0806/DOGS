# Voice, UI, Frontier Performance, and Frustum Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver reliable PC-local voice/LLM control, a self-healing low-latency panel, unknown-space-first wall-aware exploration, faster and more continuous planning/motion, and a geometrically correct camera frustum overlay.

**Architecture:** Keep the NX command contract deterministic and fail-closed: a PC-local LLM may normalize speech into one canonical Chinese command, but the existing parser/schema remains the sole authority that can admit motion. Replace unbounded WebSocket frame queuing with latest-value coalescing and render the map only when dirty. Preserve frontier exploration/Nav2 ownership while making unknown gain explicit, reusing parallel probe results, and aligning scoring velocities with the actual controller profile.

**Tech Stack:** Python 3.10/3.12, ROS 2 Humble/Nav2, Vosk, OpenAI-compatible or Ollama-local HTTP inference, vanilla JavaScript Canvas/WebSocket, pytest, Node.js contract tests, Playwright.

---

## Current evidence and guardrails

- Work is on `codex/product-room-person-search`, not `main`; the tree was clean at diagnosis time.
- The focused baseline has 205 passes and one stale timeout-contract failure. The broad baseline has 1012 passes and 18 pre-existing/stale contract failures; these must not be misreported as regressions.
- `tools/voice_console.py` currently uses Vosk only; no PC-local LLM is connected.
- `map.js` has no camera-frustum drawing code. The server publishes `visual_range_m` but not camera HFOV/yaw offset.
- forced `slam`/`gimbal` broadcasts bypass `_WS_PENDING`, while the browser redraws the complete map at 60 FPS even when nothing changed.
- frontier v3 computes wall and unknown-neighbor fields, but deployment leaves expansion weight at zero. The current simulator fails its v3 coverage gate (96.39% vs nearest 97.67%) and parallel mode uses 300 probes vs serial 66.
- frontier scoring assumes 1.5 m/s and 1.0 rad/s while Nav2 actually limits 0.6 m/s and 0.5 rad/s.
- Autonomous motion must remain cancelable, stop in `finally`, require fresh localization/scan data, and never bypass the shared navigation arbiter.

## File map

- `tools/local_llm_nlu.py` (create): small HTTP adapter for Ollama `/api/chat` and OpenAI-compatible `/v1/chat/completions`; returns text only.
- `tools/voice_console.py` (modify): optional local-LLM normalization, CLI/env configuration, fail-closed validation, accurate task acknowledgement.
- `web/nx_product_command.py` (modify): exact user utterance aliases such as “整个房间” and “房间”.
- `web/nx_move_executor.py`, `web/nx_web_server.py` (modify): actual bounded reverse motion with rear-sector scan safety.
- `web/nx_ws_latest.py` (create): ROS-free latest-value WebSocket outbox.
- `web/nx_web_server.py`, `web/nx_gimbal_node.py` (modify): coalesced streaming payloads plus reliable event delivery.
- `web/static/map.js`, `web/static/panel.html` (modify): dirty rendering, WS watchdog, polling de-overlap, frustum data/rendering.
- `web/nx_visibility_coverage.py`, `web/nx_room_orchestrator.py` (modify): publish the exact calibration used by the visibility model.
- `web/nx_exploration_manager.py`, `docker/bringup_slam_nav2.sh` (modify): explicit unknown-space tier, reusable parallel probes, tuned defaults.
- `src/go2w_nav/config/nav2_params_3d.yaml` (modify): guarded 0.8 m/s exploration-capable controller profile with matching smoother limits.

### Task 1: Lock down the requested voice command contract

**Files:**
- Modify: `web/test_product_command.py`
- Modify: `tools/test_voice_console.py`
- Modify: `web/nx_product_command.py`

- [x] **Step 1: add failing tests for every user utterance**

```python
@pytest.mark.parametrize("text,direction,amount", [
    ("往前走", "forward", 1.0),
    ("往后退", "backward", 1.0),
    ("左转", "left", 90.0),
    ("右转", "right", 90.0),
    ("向前走2米", "forward", 2.0),
])
def test_requested_motion_utterances(text, direction, amount):
    params = _move_task(text)
    assert params["direction"] == direction
    assert params.get("distance_m", params.get("angle_deg")) == amount

@pytest.mark.parametrize("text,target", [
    ("搜索整个房间，标注人", "person"),
    ("搜索房间，标注所有椅子", "chair"),
])
def test_requested_current_room_search_utterances(text, target):
    result = parse_product_command(text)
    task = result["tasks"][0]
    assert task["type"] == "search_room"
    assert task["params"]["room"] == "__current__"
    assert target in task["params"]["target_classes"]
```

- [x] **Step 2: run RED**

Run: `python -m pytest web/test_product_command.py -k requested -v`

Expected: the two abbreviated current-room searches fail because `房间/整个房间` are not current-room aliases.

- [x] **Step 3: minimally extend the deterministic aliases**

Add `"整个房间"`, `"整间房"`, and `"房间"` to `_CURRENT_ROOM_TERMS`, keeping current terms ordered longest-first through `_terms_re`. Do not weaken negation or arbitrary-room checks.

- [x] **Step 4: run GREEN and the complete parser suite**

Run: `python -m pytest web/test_product_command.py tools/test_voice_console.py -v`

Expected: all requested utterances produce exactly one canonical task.

### Task 2: Connect a PC-local LLM without giving it motion authority

**Files:**
- Create: `tools/local_llm_nlu.py`
- Create: `tools/test_local_llm_nlu.py`
- Modify: `tools/voice_console.py`
- Modify: `tools/test_voice_console.py`
- Modify: `requirements-voice.txt`
- Modify: `README.md`

- [x] **Step 1: write adapter RED tests using an injected transport**

```python
def test_ollama_response_returns_canonical_command():
    calls = []
    adapter = LocalLLMCommandNormalizer(
        url="http://127.0.0.1:11434/api/chat", model="qwen2.5:3b",
        transport=lambda url, body, timeout: calls.append((url, body)) or {
            "message": {"content": '{"command":"搜索房间标注所有椅子"}'}}
    )
    assert adapter.normalize("把屋里的凳子都圈出来") == "搜索房间标注所有椅子"
    assert calls[0][1]["stream"] is False

def test_invalid_or_multi_command_output_fails_closed():
    adapter = LocalLLMCommandNormalizer(
        url="http://127.0.0.1:11434/api/chat", model="qwen2.5:3b",
        transport=lambda _url, _body, _timeout: {
            "message": {"content": '{"commands":["前进","右转"]}'}}
    )
    assert adapter.normalize("随便走走") is None
```

- [x] **Step 2: run RED**

Run: `python -m pytest tools/test_local_llm_nlu.py -v`

Expected: import failure because the adapter does not exist.

- [x] **Step 3: implement the adapter**

`LocalLLMCommandNormalizer.normalize()` sends a strict system prompt listing only `前进/后退/左转/右转/搜索当前房间并标注目标`, requests one JSON object `{"command": string|null}`, accepts Ollama `message.content` or OpenAI-compatible `choices[0].message.content`, strips a single fenced block, rejects arrays/multiple commands/oversized output, and returns `None` on timeout/network/JSON errors. Use `urllib.request`; add no cloud SDK.

- [x] **Step 4: add dispatcher fallback tests**

```python
def test_dispatcher_uses_llm_only_after_deterministic_parser_rejects():
    normalizer = FakeNormalizer("搜索房间标注所有椅子")
    dispatcher = SearchCommandDispatcher(
        sender=fake_sender, command_normalizer=normalizer.normalize)
    result = dispatcher.dispatch("http://nx/api/command", "把屋里的凳子都圈出来")
    assert result["ok"] is True
    assert fake_sender.calls[-1][1] == "搜索房间标注所有椅子"

def test_llm_output_is_revalidated_before_send():
    dispatcher = SearchCommandDispatcher(
        sender=fake_sender, command_normalizer=lambda _: "执行系统命令")
    assert dispatcher.dispatch("http://nx/api/command", "乱说")["ok"] is False
    assert fake_sender.calls == []
```

- [x] **Step 5: wire CLI/env configuration**

Add `--llm-url`, `--llm-model`, `--llm-mode {off,fallback,always}`, and `--llm-timeout`; env equivalents are `GO2W_LOCAL_LLM_URL`, `GO2W_LOCAL_LLM_MODEL`, `GO2W_LOCAL_LLM_MODE`, and `GO2W_LOCAL_LLM_TIMEOUT`. Default mode is `fallback`; an empty URL disables model calls cleanly. Known commands stay deterministic and low-latency. Update TTS acknowledgements to say “移动任务已接收” or “搜索任务已接收” from the admitted task type.

- [x] **Step 6: run GREEN and fake HTTP integration**

Run: `python -m pytest tools/test_local_llm_nlu.py tools/test_voice_console.py web/test_product_command.py -v`

Expected: adapter formats both APIs correctly, and every model result is revalidated by `validate_voice_command` before POST.

### Task 3: Make “往后退” a real, bounded reverse movement

**Files:**
- Modify: `web/nx_move_executor.py`
- Modify: `web/test_move_executor.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/test_task_manager_move.py`

- [x] **Step 1: write RED tests for closed-loop translation**

```python
def test_reverse_stops_at_requested_displacement():
    poses = [(0.0, 0.0), (-0.25, 0.0), (-0.75, 0.0), (-0.98, 0.0)]
    index = {"value": 0}
    sent = []
    now = {"value": 0.0}
    def sleep(_seconds):
        index["value"] = min(index["value"] + 1, len(poses) - 1)
        now["value"] += 0.05
    result = run_linear_translation(
        read_xy=lambda: poses[index["value"]],
        read_clearance=lambda direction: 2.0,
        send_cmd_vel=lambda vx, vy, wz: sent.append((vx, vy, wz)),
        sleep=sleep, monotonic=lambda: now["value"],
        start_xy=(0.0, 0.0), direction="backward", distance_m=1.0)
    assert result == "succeeded"
    assert sent[0][0] < 0.0
    assert sent[-1] == (0.0, 0.0, 0.0)

def test_reverse_aborts_on_rear_obstacle_and_always_stops():
    sent = []
    result = run_linear_translation(
        read_xy=lambda: (0.0, 0.0),
        read_clearance=lambda direction: 0.3,
        send_cmd_vel=lambda vx, vy, wz: sent.append((vx, vy, wz)),
        sleep=lambda _: None, monotonic=lambda: 0.0,
        start_xy=(0.0, 0.0), direction="backward", distance_m=1.0,
        clearance_m=0.55)
    assert result == "obstacle"
    assert sent[-1] == (0.0, 0.0, 0.0)

def test_reverse_times_out_and_always_stops():
    sent = []
    ticks = iter((0.0, 0.0, 0.2, 0.4, 0.6))
    result = run_linear_translation(
        read_xy=lambda: (0.0, 0.0),
        read_clearance=lambda direction: 2.0,
        send_cmd_vel=lambda vx, vy, wz: sent.append((vx, vy, wz)),
        sleep=lambda _: None, monotonic=lambda: next(ticks),
        start_xy=(0.0, 0.0), direction="backward", distance_m=1.0,
        max_duration=0.3)
    assert result == "timed_out"
    assert sent[-1] == (0.0, 0.0, 0.0)
```

- [x] **Step 2: run RED**

Run: `python -m pytest web/test_move_executor.py -k linear_translation -v`

- [x] **Step 3: implement the pure primitive**

`run_linear_translation` commands at most 0.3 m/s in reverse, slows inside 0.25 m remaining distance, requires clearance greater than the padded-body stopping margin, measures Euclidean displacement from fresh localization, returns `succeeded|obstacle|localization_lost|timed_out`, and publishes zero in `finally`.

- [x] **Step 4: generalize scan clearance and route backward tasks**

Add `NxRobotBridge.directional_clearance(center_deg, half_fov_deg=30)` using the actual LaserScan angle metadata; `front_clearance` becomes a wrapper at 0°, and reverse uses 180°. `_execute_move_relative` keeps forward movement on Nav2 but sends backward movement through `run_linear_translation` with `manual=False`, so connectivity and obstacle guards remain active.

- [x] **Step 5: run GREEN**

Run: `python -m pytest web/test_move_executor.py web/test_task_manager_move.py -v`

### Task 4: Replace WebSocket backlog with latest-value streaming

**Files:**
- Create: `web/nx_ws_latest.py`
- Create: `web/test_ws_latest.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/nx_gimbal_node.py`

- [x] **Step 1: write RED tests**

Test that ten queued `gimbal` frames retain only the newest, reliable `mission_report/nav_goal/tasks/move_result` events keep FIFO order, a slow client cannot block another client, and closing a client removes its pending state.

- [x] **Step 2: run RED**

Run: `python -m pytest web/test_ws_latest.py -v`

- [x] **Step 3: implement `LatestValueOutbox`**

Use one bounded reliable-event deque plus a dict keyed by stream type (`gimbal`, `lidar`, `slam`, `costmap`, `costmap_global`, `occupancy_map`, `plan`, `detections`). A per-client sender always drains reliable events first, then the newest value of each stream. Updating a stream replaces its unsent value instead of scheduling another coroutine.

- [x] **Step 4: integrate and expose telemetry**

`ws_broadcast(data, force=False)` classifies by type; `force` no longer means unbounded. Add status telemetry `ws_stream_replaced`, `ws_reliable_depth`, and connected-client count. Keep JSON serialization off the ROS callback hot path where possible.

- [x] **Step 5: run GREEN plus server contract tests**

Run: `python -m pytest web/test_ws_latest.py web/test_panel_navigation_contract.py -v`

### Task 5: Make the browser self-healing and render only on change

**Files:**
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`
- Modify: `web/test_map_contract.js`
- Modify: `web/test_panel_nav_state.js`

- [x] **Step 1: write RED JS tests**

Add fake-RAF tests proving 100 unchanged animation ticks cause no additional `_draw`, multiple updates within one frame cause one draw, a changed occupancy map invalidates the correct cache, stale sockets close and reconnect, and status polling never overlaps.

- [x] **Step 2: run RED**

Run: `node web/test_map_contract.js && node web/test_panel_nav_state.js`

- [x] **Step 3: implement dirty scheduling**

Replace the permanent 60 FPS loop with `_markDirty()` that schedules at most one `requestAnimationFrame`; every state mutation calls it. Resize and interaction events invalidate transforms/caches and mark dirty. Keep video overlays independently RAF-coalesced by source so old image-load callbacks cannot repaint newer detections.

- [x] **Step 4: implement connection recovery**

Track `lastWsMessageAt`; a 2 s watchdog closes a socket with no data for 6 s. Reconnect uses capped exponential delay with jitter and one timer. On open, fetch `/api/status` with an abort timeout and rehydrate navigation, room mission, pose, detections, and stream-health metadata. Replace fixed `setInterval` polling with an awaited self-scheduling poll and `AbortController`.

- [x] **Step 5: run GREEN**

Run: `node web/test_map_contract.js && node web/test_panel_nav_state.js`

### Task 6: Publish and draw the real camera frustum

**Files:**
- Modify: `web/nx_visibility_coverage.py`
- Modify: `web/test_visibility_coverage.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/static/map.js`
- Modify: `web/test_map_contract.js`

- [x] **Step 1: write backend RED tests**

Assert `VisibilityCoverageTracker.snapshot()` contains `camera_hfov_deg`, `camera_yaw_offset_deg`, and `visible_cells`, and `_exploration_live_fields` forwards them without recomputing a different calibration.

- [x] **Step 2: write frontend geometry RED tests**

For pose `(0,0,0)`, HFOV 60°, offset 0°, range 2 m, assert frustum endpoints are `(sqrt(3), ±1)`. For robot yaw 90° and camera offset -10°, assert center bearing is 80°. Invalid/missing calibration must hide the cone rather than draw a guessed one.

- [x] **Step 3: implement contract and rendering**

Expose the exact tracker calibration in degrees and the current obstacle-clipped visible buckets. Normalize them in `_updateRoomSearch`. Draw a translucent wedge behind walls/costmap but above fog, with left/right rays, an arc, a centerline, and a `C13 77.4°` label. When `visible_cells` exists, clip/fill those exact cells; otherwise use the geometric wedge only when all calibration fields are valid.

- [x] **Step 4: run GREEN and Playwright visual check**

Run: `python -m pytest web/test_visibility_coverage.py web/test_frontier_explore.py -v && node web/test_map_contract.js`

Then render yaw 0°, 90°, -90°, and a +15° camera offset in Playwright; screenshots must show correct world-to-screen sign and no mirrored cone.

### Task 7: Make unknown-space exploration explicit and wall-aware

**Files:**
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `tools/sim_strategy_compare.py`
- Modify: `docker/bringup_slam_nav2.sh`

- [x] **Step 1: write RED selection tests**

Add cases where a nearby zero-unknown visual-coverage fallback competes with a farther frontier and the frontier wins; higher adjacent-unknown frontier wins within a bounded travel-time band; a wall-adjacent frontier wins only after the unknown tier is equal; coverage/lidar fallback is used only after no reachable frontier remains; termination requires three stable cycles with no reachable frontier/fallback.

- [x] **Step 2: run RED**

Run: `python -m pytest web/test_exploration_manager.py -k "unknown_priority or fallback_order or stable_exhaustion" -v`

- [x] **Step 3: implement lexicographic tiers**

Rank first by source (`frontier` before coverage/lidar), then by positive unknown gain, then by a bounded ETA band, then mixed utility. Normalize `adjacent_unknown_count` by the sampled support area so map resolution cannot dominate. Keep wall proximity as a same-tier tie-break/utility term; walls never replace the frontier requirement.

- [x] **Step 4: enable and report deployment defaults**

Set `GO2W_FRONTIER_MIXED_EXPANSION_BONUS=0.1`, report selected candidate `unknown_gain`, `wall_proximity_bonus`, ETA and source in snapshots/logs, and keep the three-cycle frontier-exhaustion rule.

- [x] **Step 5: run simulation gate**

Run: `python tools/sim_strategy_compare.py`

Expected: v3 coverage is no worse than nearest by more than 1%, reaches the far room boundary, and does not revisit coverage-only goals while a frontier is reachable.

### Task 8: Reuse parallel probes and eliminate standing replans

**Files:**
- Modify: `web/nx_exploration_manager.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `tools/sim_strategy_compare.py`
- Modify: `docker/bringup_slam_nav2.sh`

- [x] **Step 1: write RED tests**

Assert that parallel and serial selection choose the same candidate when the highest-priority candidate needs its second standoff approach; each first approach is probed at most once; total calls do not exceed `max_plan_probes`; and successful parallel results are reused rather than repeated serially.

- [x] **Step 2: run RED**

Run: `python -m pytest web/test_exploration_manager.py -k parallel_probe -v`

- [x] **Step 3: return ordered probe evidence**

Refactor the parallel helper to return candidate, approach list, first-result, and validation reason in original utility order. The serial continuation starts at approach index 1 for candidates already probed. It must exhaust higher-priority candidate fallbacks before accepting a lower-priority first approach, preserving serial semantics without duplicate calls.

- [x] **Step 4: enable a bounded worker pool**

Set `GO2W_FRONTIER_PROBE_WORKERS=4`. Keep the per-cycle probe cap at 12 and planning timeout at the existing bounded value.

- [x] **Step 5: run simulator performance gate**

Run: `python tools/sim_strategy_compare.py`

Expected: parallel output matches serial waypoint/coverage behavior and probe count is at most serial plus one speculative batch, not the current 300 vs 66 amplification.

### Task 9: Align and safely raise the physical exploration speed

**Files:**
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `docker/bringup_slam_nav2.sh`
- Modify: `docker/test_global_planner_contract.py`
- Modify: `tools/sim_strategy_compare.py`

- [x] **Step 1: write RED configuration contracts**

Assert controller and velocity smoother share `max_vel_x=0.8`, `max_speed_xy=0.8`, `max_velocity[0]=0.8`, and angular limit 0.5; frontier scoring env must match those exact values. Assert deceleration and local obstacle margins provide at least the calculated stopping distance plus one costmap cell.

- [x] **Step 2: run RED**

Run: `python -m pytest docker/test_global_planner_contract.py -k "velocity or stopping" -v`

- [x] **Step 3: update the guarded profile**

Raise linear max from 0.6 to 0.8 m/s in DWB and smoother, retain 1.2 m/s² acceleration and -1.0 m/s² deceleration, keep the full-body footprint and local 0.71 m inflation authority, and retain 0.5 rad/s turn speed to avoid the measured overshoot. Set frontier scoring env to 0.8/0.5.

- [x] **Step 4: retune time utility against the actual profile**

Extend simulator grid search around `k_time=15..25`; select only a setting meeting coverage and far-boundary gates, then require path length near nearest while turn radians decrease. Record the selected value in the bringup comment and simulation output.

- [x] **Step 5: run nav config and simulator gates**

Run: `python -m pytest docker/test_global_planner_contract.py web/test_exploration_manager.py -v`

Run: `python tools/sim_strategy_compare.py`

### Task 10: Full verification and deployment evidence

**Files:**
- Modify: `TEST_PLAN.md`
- Modify: `docs/OPTIMIZATION_COMPLETION_AUDIT.md`

- [ ] **Step 1: run all focused tests fresh**

```powershell
python -m pytest -q `
  web/test_product_command.py web/test_move_executor.py web/test_task_manager_move.py `
  tools/test_local_llm_nlu.py tools/test_voice_console.py `
  web/test_ws_latest.py web/test_visibility_coverage.py `
  web/test_exploration_manager.py web/test_frontier_explore.py `
  web/test_unknown_room_exploration_sim.py docker/test_global_planner_contract.py
node web/test_map_contract.js
node web/test_panel_nav_state.js
python tools/sim_strategy_compare.py
```

- [ ] **Step 2: run broad suite and classify only pre-existing failures**

Run: `python -m pytest -q web tools src/go2w_bridge/test docker -k "not test_lidar_topics"`

Compare against the recorded 1012-pass/18-failure baseline; no new failure is allowed. Do not call the broad suite green unless every stale contract is either reconciled or explicitly excluded by an authoritative current contract.

- [ ] **Step 3: browser performance and frustum proof**

Use Playwright with a mocked high-rate WS producer for at least 60 seconds. Verify the displayed state advances without refresh, no stream backlog grows, draw count remains bounded by state changes, reconnection rehydrates state, and capture screenshots for four frustum orientations.

- [ ] **Step 4: build release artifacts**

Run existing nav and web release builders and artifact verifiers. Do not deploy until all offline gates pass.

- [ ] **Step 5: NX acceptance checklist**

With the user-controlled NX online, run all seven spoken phrases, measure displacement/yaw, search a room with a person and chairs, compare frontier logs against map unknown/walls, record mean/p95 selection latency, total probes, path/turn totals, controller speed, WebSocket replacement counts, and panel FPS/reconnect behavior. Hardware-only evidence remains required before claiming the physical behaviors are fully verified.

## Self-review

- Spec coverage: all five numbered requirements map to Tasks 1-3, 4-6, 7, 8-9, and 6 respectively.
- Safety: model output never bypasses the deterministic parser/schema; reverse and turn paths have clearance/freshness/timeout/final stop; Nav2 remains autonomous path authority for forward/search travel.
- Performance: both planning-call amplification and browser frame amplification have explicit regression tests and numeric gates.
- Geometry: frontend uses the same HFOV/yaw offset/range values as backend visibility, not duplicated constants.
- Completion boundary: offline mocks prove software contracts; physical success still requires the documented NX acceptance run.
