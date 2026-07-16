# Go2W Project Architecture Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the current Go2W prototype into a deterministic, SDK-aligned system that boots safely, owns motion through one state machine, navigates through one action gateway, explores unknown space, time-aligns detections with localization/LiDAR, and exposes one canonical search-mission contract.

**Architecture:** Keep the NX-centric deployment and the proven `/cmd_vel_nav` isolation, but split the current god modules into pure policy/state components plus thin ROS/HTTP adapters. One event-driven motion machine owns every Unitree command; one navigation gateway owns every Nav2 goal; one timestamped observation pipeline owns target localization; one mission schema is shared by text, voice, HTTP, exploration, and the frontend. Every transition is confirmed by telemetry or a terminal action result, never by an RPC return code alone.

**Tech Stack:** Python 3, ROS 2 Humble/rclpy, Unitree SDK2 Python, Nav2, Fast-LIO/MID360, YOLO/YOLO-World, OpenCV/NumPy, HTTP/WebSocket, pytest, Node contract tests, systemd.

**Execution status (2026-07-15):** Tasks 1–13 are implemented and the current offline release gate passes (`696 passed` plus both Node contract suites). The strict, content-addressed all-system artifact is `dist/go2w-f10b14504edb-dirty-b2b7d5faa28e-all.tar.gz` (SHA-256 `DBA16D7E90ED765C39DB8FE10A354DF3BC4EE2621AEB480EF4F39A9BAD2D2A62`); its strict required set exactly covers all 83 payload files. It includes the AI runtime, automatic dog/MID360 interface and model commissioning, final-prefix Nav2 build, costmap/MID360 companion units, complete rollback, and read-only release/Nav2/perception deployment probes. The Nav-owned wheel/IMU process explicitly receives the commissioned dog interface and release fingerprint. The PC deployer can explicitly create a non-overwriting private first-deploy Token after local artifact verification; Web, voice, and NX preflight reject weak tokens; and deployment restarts the selected MID360 network, driver, and watchdog units before Nav2 so no already-active process can retain an old release path. Task 14 remains intentionally pending until NX and the powered, supported robot are available; no offline result is treated as hardware validation.

---

## Audit baseline and decisions

This plan supersedes the conflicting motion/startup portions of:

- `docs/superpowers/plans/2026-07-14-go2w-sdk-aligned-motion-state-machine.md`
- `docs/superpowers/plans/2026-07-14-end-to-end-autonomous-room-person-search.md`

The generic target work in `2026-07-14-generic-voice-target-search.md` remains useful, but its command schema will move behind the single mission contract in Task 8.

### Verified baseline

- `python -m pytest docker web tools -q` passes: `555 passed`.
- `node web/test_map_contract.js` and `node web/test_panel_nav_state.js` pass.
- `python -m compileall -q ai web src tools docker` passes.
- These tests establish compatibility, not architectural correctness. Existing motion tests simultaneously require startup to avoid pose commands and require first mode-1 feedback to queue `StandUp`; both pass because they inspect different local functions instead of the complete event sequence.

### Keep

- NX-local sensing/planning/control loop; PC remains a thin UI/voice client.
- One leased `SportClient` for all high-level motion commands.
- Separate manual `/cmd_vel` and autonomous `/cmd_vel_nav` channels.
- `/scan_mid360` freshness gate immediately before SDK `Move`.
- Telemetry-derived physical mode and measured wheel-speed motion state.
- Nav2 dynamic replanning tree without autonomous `Spin`/`BackUp` motion.
- Generic YOLO/YOLO-World target vocabulary, LiDAR bearing/range localization, mission artifacts, spatial/appearance de-duplication, and generic `target_markers` rendering.

### Replace or consolidate

- Replace `Go2WStateModel + DriveSessionModel + legacy integer workflow` with one pure event reducer.
- Replace direct SDK calls from callbacks/control branches with one serialized SDK command actor.
- Replace `PointNavigationController` plus `RoomSearchOrchestrator.Nav2ActionClient` with one navigation gateway.
- Split `nx_motion_node.py` (2,597 lines), `nx_web_server.py` (2,546 lines), `nx_room_orchestrator.py` (3,248 lines), and `nx_ai_node.py` (1,858 lines) by responsibility.
- Replace wall-clock “latest sample” target localization with timestamp-matched detection, scan/cloud, and map pose.
- Replace repeated command parsing in `nx_product_command.py`, `TaskManager`, `NxAiEngine`, `voice_command.py`, and `tools/voice_console.py` with one canonical request schema.
- Replace file-by-file deployment and unrelated motion restarts with versioned, subsystem-scoped releases.

### Unitree SDK contract used by this plan

- `SportClient.Init()` only registers APIs; startup does not imply a posture command.
- `StandUp`, `StandDown`, `BalanceStand`, `RecoveryStand`, `Damp`, and `StopMove` are separate request/reply APIs.
- `Move(vx, vy, vyaw)` uses `_CallNoReply`; a local zero return is transport evidence, not proof of physical motion.
- `MotionSwitcher.CheckMode()` is the read-only way to observe the selected motion service; the official example identifies `ai-w` as the wheeled mode.
- Raw `SportModeState.mode` is feedback. The current robot profile maps observed `1/3` to wheel-capable modes, `6` to joint lock, and `7` to damping; unknown values remain unknown and fail closed.

## Target runtime ownership

```text
HTTP / voice / panel
        |
        v
Canonical MissionRequest -----> MissionCoordinator
                                      |
                                      v
                              NavigationGateway
                                      |
                               Nav2 action server
                                      |
                                /cmd_vel_nav
                                      v
MotionIntent ------------> Go2WMotionMachine <----------- Unitree telemetry
                                | effects
                                v
                         UnitreeSportAdapter
                                |
                         one leased SportClient

Detection frame + stamped scan/cloud + pose history
        \              |               /
         ------> ObservationSynchronizer ----> TargetMissionStore ----> map markers
```

Only `Go2WMotionMachine` may authorize an SDK effect. Only `NavigationGateway` may own a Nav2 goal handle. Only `MissionCoordinator` may advance a search mission phase.

---

### Task 1: Freeze observable contracts and add a deployment fingerprint

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/build_info.py`
- Create: `docker/test_build_fingerprint_contract.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Modify: `web/nx_web_server.py`
- Modify: `docker/deploy_nx.sh`
- Modify: `docker/deploy_nx_web.sh`

- [ ] **Step 1: Write a failing fingerprint contract**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "go2w_bridge"))

def test_motion_and_web_expose_the_same_release_id():
    from go2w_bridge.build_info import release_id
    value = release_id({"GO2W_RELEASE_ID": "abc123"})
    assert value == "abc123"
    assert len(value) <= 64
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python -m pytest docker/test_build_fingerprint_contract.py -q`

Expected: FAIL because `go2w_bridge.build_info` does not exist.

- [ ] **Step 3: Implement the immutable release identifier**

```python
# src/go2w_bridge/go2w_bridge/build_info.py
import os

def release_id(env=None):
    source = os.environ if env is None else env
    value = str(source.get("GO2W_RELEASE_ID", "development")).strip()
    return value[:64] or "development"
```

Add `release_id` to `/dog_state`, `/api/status`, and `/api/version`. Do not derive it from a dirty runtime directory.

- [ ] **Step 4: Make deployment scripts generate and verify one ID**

Use `git rev-parse --short=12 HEAD` plus a `-dirty` suffix when local changes exist, export it through both systemd units, and compare `/dog_state` with `/api/version` before reporting success.

- [ ] **Step 5: Run contracts**

Run: `python -m pytest docker/test_build_fingerprint_contract.py web/test_ws_broadcast_contract.py -q`

Expected: PASS and both status surfaces contain the same non-empty `release_id`.

- [ ] **Step 6: Commit**

```bash
git add src/go2w_bridge/go2w_bridge/build_info.py docker/test_build_fingerprint_contract.py \
  src/go2w_bridge/go2w_bridge/nx_motion_node.py web/nx_web_server.py \
  docker/deploy_nx.sh docker/deploy_nx_web.sh
git commit -m "chore: expose one NX release fingerprint"
```

---

### Task 2: Define one SDK-aligned motion domain model

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/motion_types.py`
- Create: `src/go2w_bridge/test/conftest.py`
- Create: `src/go2w_bridge/test/test_motion_types.py`

- [ ] **Step 1: Write failing enum/profile tests**

Create the package-local pytest path bootstrap once:

```python
# src/go2w_bridge/test/conftest.py
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

```python
def test_go2w_profile_fails_closed_for_unknown_mode():
    from go2w_bridge.motion_types import PhysicalMode, Go2WModeProfile
    profile = Go2WModeProfile()
    assert profile.decode(1) is PhysicalMode.WHEEL_BALANCE
    assert profile.decode(3) is PhysicalMode.WHEEL_LOCOMOTION
    assert profile.decode(6) is PhysicalMode.JOINT_LOCK
    assert profile.decode(7) is PhysicalMode.DAMPING
    assert profile.decode(255) is PhysicalMode.UNKNOWN

def test_move_transport_result_is_not_motion_confirmation():
    from go2w_bridge.motion_types import CommandReceipt
    receipt = CommandReceipt("Move", 0, sequence=4)
    assert receipt.transport_ok is True
    assert receipt.physical_confirmed is False
```

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest src/go2w_bridge/test/test_motion_types.py -q`

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Add the domain types**

```python
from dataclasses import dataclass
from enum import Enum

class PhysicalMode(Enum):
    UNKNOWN = "unknown"
    IDLE = "idle"
    WHEEL_BALANCE = "wheel_balance"
    POSE = "pose"
    WHEEL_LOCOMOTION = "wheel_locomotion"
    LIE_DOWN = "lie_down"
    JOINT_LOCK = "joint_lock"
    DAMPING = "damping"
    RECOVERY = "recovery"

class SessionState(Enum):
    BOOT_HOLD = "boot_hold"
    PARKED = "parked"
    ACTIVATING = "activating"
    MANUAL_ACTIVE = "manual_active"
    NAV_ACTIVE = "nav_active"
    STOPPING = "stopping"
    PARKING = "parking"
    ESTOP = "estop"
    FAULT = "fault"

class MotionIntent(Enum):
    START_MANUAL = "start_manual"
    START_NAV = "start_nav"
    PARK = "park"
    ESTOP = "estop"

@dataclass(frozen=True)
class Effect:
    operation: str
    sequence: int

@dataclass(frozen=True)
class Telemetry:
    sample_id: int
    source_stamp: float
    received_at: float
    raw_mode: int
    wheel_dq: tuple[float, float, float, float]
    battery_soc: float
    error_code: int
    roll: float
    pitch: float
    motion_service: str | None
    motor_fault: bool

@dataclass(frozen=True)
class CommandReceipt:
    operation: str
    code: int
    sequence: int
    physical_confirmed: bool = False

    @property
    def transport_ok(self):
        return self.code == 0

class Go2WModeProfile:
    _MAP = {0: PhysicalMode.IDLE, 1: PhysicalMode.WHEEL_BALANCE,
            2: PhysicalMode.POSE, 3: PhysicalMode.WHEEL_LOCOMOTION,
            5: PhysicalMode.LIE_DOWN, 6: PhysicalMode.JOINT_LOCK,
            7: PhysicalMode.DAMPING, 8: PhysicalMode.RECOVERY}

    def decode(self, raw):
        return self._MAP.get(int(raw), PhysicalMode.UNKNOWN)
```

- [ ] **Step 4: Add telemetry validation tests**

Reject non-finite wheel speeds/attitude, invalid SOC, negative error codes, and stale samples. Do not silently coerce invalid samples to a safe mode.

- [ ] **Step 5: Run and commit**

Run: `python -m pytest src/go2w_bridge/test/test_motion_types.py -q`

Expected: PASS.

```bash
git add src/go2w_bridge/go2w_bridge/motion_types.py \
  src/go2w_bridge/test/conftest.py src/go2w_bridge/test/test_motion_types.py
git commit -m "refactor: define SDK-aligned Go2W motion types"
```

---

### Task 3: Implement the single pure motion state machine

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/motion_machine.py`
- Create: `src/go2w_bridge/test/test_motion_machine.py`
- Delete after migration: embedded `Go2WStateModel`, `DriveSessionModel`, and legacy workflow state from `nx_motion_node.py`

- [ ] **Step 1: Write the complete startup transition table as tests**

```python
import pytest

from go2w_bridge.motion_machine import Go2WMotionMachine
from go2w_bridge.motion_types import PhysicalMode, SessionState, Telemetry

@pytest.fixture
def machine():
    return Go2WMotionMachine(now=lambda: 100.0)

@pytest.fixture
def telemetry():
    def make(raw_mode, wheel_dq=(0.0, 0.0, 0.0, 0.0),
             motion_service="ai-w"):
        return Telemetry(
            sample_id=1,
            source_stamp=99.9,
            received_at=100.0,
            raw_mode=raw_mode,
            wheel_dq=tuple(float(value) for value in wheel_dq),
            battery_soc=80.0,
            error_code=0,
            roll=0.0,
            pitch=0.0,
            motion_service=motion_service,
            motor_fault=False,
        )
    return make

def test_boot_mode6_adopts_parked_without_pose_effect(machine, telemetry):
    effects = machine.observe(telemetry(raw_mode=6, wheel_dq=(0, 0, 0, 0)))
    assert machine.snapshot().session is SessionState.PARKED
    assert effects == []

def test_boot_stationary_wheel_mode_requests_one_park(machine, telemetry):
    first = machine.observe(telemetry(raw_mode=1, wheel_dq=(0, 0, 0, 0)))
    second = machine.observe(telemetry(raw_mode=1, wheel_dq=(0, 0, 0, 0)))
    assert [effect.operation for effect in first] == ["StandUp"]
    assert second == []
    assert machine.snapshot().session is SessionState.PARKING

def test_boot_moving_wheels_never_issue_pose(machine, telemetry):
    effects = machine.observe(telemetry(raw_mode=1, wheel_dq=(1, 1, 1, 1)))
    assert [effect.operation for effect in effects] == ["MoveZero"]
    assert "StandUp" not in [effect.operation for effect in effects]

def test_boot_unknown_or_wrong_motion_service_fails_closed(machine, telemetry):
    effects = machine.observe(telemetry(raw_mode=255, motion_service="normal"))
    assert effects == []
    assert machine.snapshot().velocity_authorized is False
```

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest src/go2w_bridge/test/test_motion_machine.py -q`

Expected: FAIL because the machine does not exist.

- [ ] **Step 3: Implement event-to-effect reduction**

The public surface is intentionally small: `sdk_ready(motion_service)`,
`observe(telemetry)`, `request(intent)`, `tick(now)`,
`record_receipt(receipt)`, and `snapshot()`. The first four methods return a
list of immutable `Effect` values; `snapshot()` returns an immutable
`MotionSnapshot`. No method performs SDK or ROS I/O.

Implement these exact rules:

| Current session | Event/feedback | Next session | SDK effect |
|---|---|---|---|
| BOOT_HOLD | mode 6 + stopped + healthy | PARKED | none |
| BOOT_HOLD | mode 1/3 + stopped + healthy | PARKING | one `StandUp` |
| BOOT_HOLD | mode 1/3 + moving | BOOT_HOLD | `MoveZero` only |
| BOOT_HOLD | stale/error/unknown/wrong motion service | FAULT | none |
| PARKED | explicit `start_manual` | ACTIVATING(manual) | one `BalanceStand` |
| PARKED | explicit `start_nav` + fresh scan | ACTIVATING(nav) | one `BalanceStand` |
| ACTIVATING | matching mode 1/3 feedback | MANUAL_ACTIVE or NAV_ACTIVE | none |
| ACTIVE | explicit park/terminal | STOPPING | `MoveZero`, then at most one profile-authorized `StopMove` |
| STOPPING | wheel speed below threshold | PARKING | one `StandUp` |
| PARKING | mode 6 + stopped | PARKED | none |
| any | estop | ESTOP | `MoveZero` only |
| transition | deadline exceeded | FAULT | `MoveZero` only |

Do not emit `Damp`, `RecoveryStand`, `StandDown`, or `BalanceStand` from
startup, watchdog, timeout, or error handling. Model `StopMove` as a terminal
movement operation, never as a repeating 0.5 s timer. Support two explicit
firmware profiles, `move_zero_only` and `move_zero_then_stop_move`; keep the
latter locked until the supported-robot experiment in Task 14 records that a
single `StopMove` clears the retained wheel target without changing posture.
Both profiles still require stopped-wheel feedback before parking.

- [ ] **Step 4: Add property-style invariant tests**

For every event sequence generated from the finite state/event set, assert:

```python
assert not snapshot.velocity_authorized or (
    snapshot.session in {SessionState.MANUAL_ACTIVE, SessionState.NAV_ACTIVE}
    and snapshot.physical_mode in {
        PhysicalMode.WHEEL_BALANCE,
        PhysicalMode.WHEEL_LOCOMOTION,
    }
    and snapshot.telemetry_fresh
    and snapshot.error_code == 0
)
```

Also assert one pose effect per transition ID and no inverse pose command on timeout.

- [ ] **Step 5: Run and commit**

Run: `python -m pytest src/go2w_bridge/test/test_motion_machine.py -q`

Expected: PASS.

```bash
git add src/go2w_bridge/go2w_bridge/motion_machine.py src/go2w_bridge/test/test_motion_machine.py
git commit -m "refactor: add single feedback-driven motion machine"
```

---

### Task 4: Serialize Unitree SDK effects behind one adapter

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py`
- Create: `src/go2w_bridge/test/test_unitree_sport_adapter.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`

- [ ] **Step 1: Write a failing serialization test**

```python
from go2w_bridge.motion_types import Effect
from go2w_bridge.unitree_sport_adapter import UnitreeSportAdapter

class FakeSport:
    def __init__(self, calls):
        self.calls = calls

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", (vx, vy, vyaw)))
        return 0

    def StandUp(self):
        self.calls.append(("StandUp", ()))
        return 0

class FakeSwitcher:
    def CheckMode(self):
        return 0, {"name": "ai-w"}

def test_adapter_uses_one_client_and_preserves_effect_order():
    calls = []
    adapter = UnitreeSportAdapter(FakeSport(calls), FakeSwitcher())
    adapter.execute(Effect("MoveZero", sequence=1))
    adapter.execute(Effect("StandUp", sequence=2))
    assert calls == [
        ("Move", (0.0, 0.0, 0.0)),
        ("StandUp", ()),
    ]
```

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest src/go2w_bridge/test/test_unitree_sport_adapter.py -q`

Expected: FAIL because the adapter does not exist.

- [ ] **Step 3: Implement the adapter**

The adapter owns exactly one `SportClient(enableLease=True)` and one read-only `MotionSwitcherClient`. On initialization it performs:

1. `SportClient.Init()`.
2. Immediate `Move(0,0,0)` twice.
3. Lease-settle wait.
4. `Move(0,0,0)` twice.
5. `MotionSwitcher.CheckMode()` and reports the result to the state machine.

Never call `SelectMode()` automatically. A non-`ai-w` service is a visible fault requiring an explicit maintenance operation.

- [ ] **Step 4: Make callbacks enqueue events, never call SDK methods**

Use one queue and one actor thread:

```python
while not stopped:
    event = queue.get(timeout=control_period)
    effects = machine.handle(event)
    for effect in effects:
        receipt = adapter.execute(effect)
        machine.record_receipt(receipt)
```

Set BOOT_HOLD before the SDK thread starts, and start the actor before creating ROS command subscriptions. This removes the current feedback/thread/startup-latch race.

- [ ] **Step 5: Run and commit**

Run: `python -m pytest src/go2w_bridge/test/test_unitree_sport_adapter.py docker/test_motion_scan_watchdog.py -q`

Expected: PASS with no SDK operation reachable from ROS callbacks.

```bash
git add src/go2w_bridge/go2w_bridge/unitree_sport_adapter.py \
  src/go2w_bridge/test/test_unitree_sport_adapter.py \
  src/go2w_bridge/go2w_bridge/nx_motion_node.py
git commit -m "refactor: serialize Unitree effects through one adapter"
```

---

### Task 5: Migrate ROS motion I/O and eliminate the three old state owners

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/motion_protocol.py`
- Create: `src/go2w_bridge/test/test_motion_protocol.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_sensor_node.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Modify: `docker/test_motion_scan_watchdog.py`
- Modify: `web/nx_web_server.py`

- [ ] **Step 1: Lock a versioned intent/status schema**

```python
intent = {
    "schema_version": 1,
    "request_id": "7f6d9f8e-5d35-4ea2-932a-31fa16cd31ce",
    "intent": "start_nav",
    "source": "navigation_arbiter",
}

status = {
    "schema_version": 4,
    "session": "parked",
    "physical_mode": "joint_lock",
    "actual_motion": "stopped",
    "velocity_authorized": False,
    "transition": None,
    "fault": None,
    "raw": {"sport_mode": 6, "error_code": 0},
}
```

Accept the six legacy strings during one migration release, translate them into the schema, and publish a deprecation counter. Reject all other strings.

- [ ] **Step 2: Add attitude and sample identity to feedback**

Extend `/wheel_feedback` with `sample_id`, source timestamps, `roll`, `pitch`, motor-loss flags, and motion-switcher service. The motion node must reject non-increasing or stale sample IDs.

- [ ] **Step 3: Remove old state classes and integer workflow**

Delete `DriveSessionModel`, the old `Go2WStateModel`, the `STANDING`, `STOOD`, and `BALANCE_UNCONFIRMED` workflow integers, `_synchronize_active_session_state`, `_adopt_startup_feedback`, and any method that independently changes authorization state.

- [ ] **Step 4: Add complete boot-order tests**

Test feedback arriving before SDK readiness, during lease settlement, immediately after actor start, and after subscriptions are active. All schedules must produce the same final state/effect list.

- [ ] **Step 5: Run and commit**

Run:

```bash
python -m pytest src/go2w_bridge/test docker/test_motion_scan_watchdog.py -q
python -m py_compile src/go2w_bridge/go2w_bridge/nx_motion_node.py
```

Expected: PASS; `nx_motion_node.py` contains only one state-machine instance and no legacy workflow constants.

```bash
git add src/go2w_bridge/go2w_bridge/motion_protocol.py \
  src/go2w_bridge/test/test_motion_protocol.py \
  src/go2w_bridge/go2w_bridge/nx_sensor_node.py \
  src/go2w_bridge/go2w_bridge/nx_motion_node.py \
  docker/test_motion_scan_watchdog.py web/nx_web_server.py
git commit -m "refactor: make motion protocol single-source-of-truth"
```

---

### Task 6: Unify all Nav2 goals behind one navigation gateway

**Files:**
- Create: `web/nx_navigation_gateway.py`
- Create: `web/test_navigation_gateway.py`
- Modify: `web/nx_point_nav.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_navigation_arbiter.py`
- Modify: `web/nx_web_server.py`

- [ ] **Step 1: Write failing single-owner tests**

```python
from nx_navigation_gateway import NavigationGateway

class FakeActionPort:
    def __init__(self):
        self.sent = []

    def send_goal(self, pose, feedback_cb=None):
        handle = object()
        self.sent.append((pose, feedback_cb, handle))
        return handle

def test_point_and_mission_goals_share_one_action_owner():
    gateway = NavigationGateway(action_port=FakeActionPort())
    first = gateway.submit(owner="point", pose=(1, 0, 0))
    second = gateway.submit(owner="mission", pose=(2, 0, 0))
    assert first.accepted is True
    assert second.accepted is False
    assert second.reason == "navigation_owner_busy"
```

Cover late goal acceptance, cancel acknowledgement, result arrival after timeout, health loss, and shutdown. Ownership is released only after a terminal action result or confirmed cancellation.

- [ ] **Step 2: Run and verify RED**

Run: `python -m pytest web/test_navigation_gateway.py -q`

Expected: FAIL because the gateway does not exist.

- [ ] **Step 3: Extract the proven action lifecycle**

Move the future/goal-handle/quarantine logic from `PointNavigationController` into `NavigationGateway`. Provide:

```python
submit(owner, pose, feedback_cb=None) -> SubmitResult
cancel(owner, reason) -> CancelResult
wait_terminal(owner, timeout) -> NavigationResult
compute_path(pose, timeout) -> PathResult
snapshot() -> dict
```

- [ ] **Step 4: Replace the second action client**

Delete `RoomSearchOrchestrator.Nav2ActionClient`. Inject the shared gateway into both point navigation and room exploration. Keep `NavigationArbiter` as the high-level producer/session owner; it must no longer coordinate two unrelated action implementations.

- [ ] **Step 5: Remove Referer-based stop semantics**

`/api/manual_release` releases a key hold; `/api/stop` always performs global zero/cancel/park. Never infer safety meaning from the HTTP `Referer` header.

- [ ] **Step 6: Run and commit**

Run:

```bash
python -m pytest web/test_navigation_gateway.py web/test_point_navigation.py \
  web/test_navigation_arbitration.py web/test_product_room_orchestrator.py -q
```

Expected: PASS and exactly one `NavigateToPose` action client is constructed in production.

```bash
git add web/nx_navigation_gateway.py web/test_navigation_gateway.py \
  web/nx_point_nav.py web/nx_room_orchestrator.py \
  web/nx_navigation_arbiter.py web/nx_web_server.py
git commit -m "refactor: route every Nav2 goal through one gateway"
```

---

### Task 7: Add measurable Nav2 performance gates before tuning speed

**Files:**
- Create: `tools/nav2_benchmark.py`
- Create: `tools/test_nav2_benchmark.py`
- Modify: `src/go2w_nav/config/nav2_params_3d.yaml`
- Modify: `src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml`
- Modify: `docker/diagnose_nav2_goal.sh`

- [ ] **Step 1: Define benchmark metrics in tests**

```python
from nav2_benchmark import evaluate_report

def test_reference_report_passes_acceptance_gate():
    report = {
        "plan_latency_ms": 500,
        "time_to_first_cmd_ms": 400,
        "min_obstacle_clearance_m": 0.20,
        "terminal_parked": True,
        "measured_displacement_m": 0.30,
    }
    assert evaluate_report(report).passed is True
```

- [ ] **Step 2: Implement a read-only recorder**

Record `/navigate_to_pose` acceptance/result, `/plan`, `/cmd_vel_nav`, `/localization_pose`, local/global costmaps, `/scan_mid360`, and `/dog_state` into one JSON report. Without `--execute`, never publish a goal.

- [ ] **Step 3: Tune in this fixed order**

1. Sensor/TF freshness and self-filter correctness.
2. Planner latency and reachable collision-free path.
3. Controller time-to-first-command.
4. Straight-line deadband and acceleration.
5. Turning response and creep compensation.
6. Only then raise maximum linear/angular speeds.

Do not change more than one parameter group per benchmark report. Preserve the current `/cmd_vel_nav` safety channel and no-motion recovery tree.

- [ ] **Step 4: Run offline tests**

Run: `python -m pytest tools/test_nav2_benchmark.py docker/test_mid360_only_contract.py docker/test_map_odom_fuser_performance_contract.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/nav2_benchmark.py tools/test_nav2_benchmark.py \
  src/go2w_nav/config/nav2_params_3d.yaml \
  src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml \
  docker/diagnose_nav2_goal.sh
git commit -m "test: add Nav2 latency and clearance benchmarks"
```

---

### Task 8: Introduce one canonical search-mission schema

**Files:**
- Create: `web/nx_mission_schema.py`
- Create: `web/test_mission_schema.py`
- Modify: `web/nx_product_command.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_ai_node.py`
- Modify: `tools/voice_console.py`
- Modify: `web/voice_command.py`

- [ ] **Step 1: Write failing schema tests**

```python
from nx_mission_schema import SearchMissionRequest

def test_person_and_table_commands_share_one_schema():
    person = SearchMissionRequest.current_room(["person"])
    table = SearchMissionRequest.current_room(["dining table"])
    assert person.to_dict()["target_classes"] == ["person"]
    assert table.to_dict()["target_classes"] == ["dining table"]
    assert person.search_strategy == table.search_strategy == "frontier_explore"
```

- [ ] **Step 2: Implement immutable validated request/result types**

```python
from dataclasses import asdict, dataclass
from uuid import uuid4

@dataclass(frozen=True)
class SearchMissionRequest:
    request_id: str
    room: str
    target_classes: tuple[str, ...]
    search_strategy: str
    require_photos: bool
    mark_on_map: bool
    max_radius_m: float
    max_time_s: float

    @classmethod
    def current_room(cls, target_classes):
        targets = tuple(str(value).strip() for value in target_classes)
        return cls(
            request_id=str(uuid4()),
            room="current_room",
            target_classes=targets,
            search_strategy="frontier_explore",
            require_photos=True,
            mark_on_map=True,
            max_radius_m=12.0,
            max_time_s=900.0,
        )

    def to_dict(self):
        value = asdict(self)
        value["target_classes"] = list(self.target_classes)
        return value
```

Reject empty targets, non-finite bounds, unknown strategies, and unsupported free-form detector names. Normalize aliases once (`桌子 -> dining table`, `人 -> person`).

- [ ] **Step 3: Remove duplicate parsers**

The deterministic parser is authoritative. VLM output must be validated into the same schema; on failure it returns a user-visible parse error rather than entering a second fallback parser. PC voice produces the same JSON and does not invent motion steps.

- [ ] **Step 4: Run and commit**

Run:

```bash
python -m pytest web/test_mission_schema.py web/test_product_command.py \
  web/test_voice_search_contract.py tools/test_voice_console.py -q
```

Expected: PASS for person and table missions with identical navigation/search semantics.

```bash
git add web/nx_mission_schema.py web/test_mission_schema.py \
  web/nx_product_command.py web/nx_web_server.py web/nx_room_orchestrator.py \
  web/nx_ai_node.py tools/voice_console.py web/voice_command.py
git commit -m "refactor: use one canonical search mission schema"
```

---

### Task 9: Time-align detections, LiDAR, and map localization

**Files:**
- Create: `web/nx_observation_sync.py`
- Create: `web/test_observation_sync.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/nx_ai_node.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_person_localizer.py`
- Modify: `web/nx_person_mission.py`

- [ ] **Step 1: Write failing synchronization tests**

```python
import pytest

from nx_observation_sync import ObservationSynchronizer

def test_observation_uses_pose_at_camera_capture():
    sync = ObservationSynchronizer()
    sync.add_pose(stamp=10.0, x=0.0, y=0.0, yaw=0.0)
    sync.add_pose(stamp=11.0, x=1.0, y=0.0, yaw=0.0)
    sync.add_scan(stamp=10.1, scan=object())
    bundle = sync.bundle_for_detection(stamp=10.1, tolerance=0.15)
    assert bundle.pose.x == pytest.approx(0.1, abs=0.02)

def test_unsynchronized_scan_is_rejected():
    sync = ObservationSynchronizer()
    sync.add_pose(stamp=10.0, x=0.0, y=0.0, yaw=0.0)
    sync.add_scan(stamp=20.0, scan=object())
    assert sync.bundle_for_detection(stamp=10.0, tolerance=0.15) is None
```

- [ ] **Step 2: Implement bounded sample histories**

Keep deques of stamped localization poses, MID360 scans/clouds, camera frames, and detections. Interpolate planar pose with wrapped yaw; select the nearest scan/cloud; reject any bundle outside tolerance. Use ROS header stamps when available and monotonic reception time only for freshness.

- [ ] **Step 3: Localize from the bundle**

Replace `latest detection + latest scan + current pose` in `_observe_people_at_viewpoint`. Persist every marker with capture stamp, pose stamp, scan stamp, time deltas, localization quality, and detector source.

- [ ] **Step 4: Preserve and strengthen de-duplication**

Keep class-specific spatial/appearance matching, but never merge across classes or missions. Record merge evidence and keep the highest-quality range observation as the canonical position.

- [ ] **Step 5: Run and commit**

Run:

```bash
python -m pytest web/test_observation_sync.py web/test_person_localizer.py \
  web/test_person_mission.py web/test_product_room_orchestrator.py -q
```

Expected: PASS; stale or unsynchronized data produces an unresolved observation, never a false finite map marker.

```bash
git add web/nx_observation_sync.py web/test_observation_sync.py \
  web/nx_web_server.py web/nx_ai_node.py web/nx_room_orchestrator.py \
  web/nx_person_localizer.py web/nx_person_mission.py
git commit -m "feat: synchronize target evidence before map localization"
```

---

### Task 10: Extract persistent exploration from the room orchestrator

**Files:**
- Create: `web/nx_frontier_planner.py`
- Create: `web/nx_exploration_manager.py`
- Create: `web/test_exploration_manager.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/test_frontier_explore.py`

- [ ] **Step 1: Write multi-room frontier tests**

Construct a synthetic occupancy map with two rooms, a doorway, an unreachable frontier, and a moved obstacle. Require deterministic selection of reachable frontiers, bounded retries, no rolling-window edge selection, and completion only when information gain is exhausted or the mission budget expires.

- [ ] **Step 2: Implement pure frontier scoring**

Score with path cost and information gain, not Euclidean distance alone:

```python
score = information_gain / (1.0 + path_length * distance_weight)
score -= heading_change * heading_weight
score -= failure_count * failure_penalty
```

Candidates must be free-space goals with a successful `ComputePathToPose` result. Failed candidates enter a bounded blacklist keyed by map cell and map revision.

- [ ] **Step 3: Add persistent exploration state**

`ExplorationManager` owns mission origin, map revision, visited/failed frontiers, budgets, and current goal. It receives the shared `NavigationGateway` and `ObservationSynchronizer`; it does not create ROS action clients or dynamic subscriptions inside `run()`.

- [ ] **Step 4: Separate current-room and whole-floor policies**

- Current room: radius/time bound plus optional calibrated polygon; crossing a detected doorway outside the polygon is rejected.
- Whole floor: no room radius, but use total time/distance/battery reserve and persistent frontier exhaustion.

- [ ] **Step 5: Run and commit**

Run: `python -m pytest web/test_exploration_manager.py web/test_frontier_explore.py web/test_active_search.py -q`

Expected: PASS; `RoomSearchOrchestrator` delegates exploration and contains no frontier clustering implementation.

```bash
git add web/nx_frontier_planner.py web/nx_exploration_manager.py \
  web/test_exploration_manager.py web/nx_room_orchestrator.py web/test_frontier_explore.py
git commit -m "refactor: extract bounded persistent frontier exploration"
```

---

### Task 11: Secure and simplify the control API

**Files:**
- Create: `web/nx_control_auth.py`
- Create: `web/test_control_auth.py`
- Modify: `web/nx_web_server.py`
- Modify: `web/static/panel.html`
- Modify: `docker/go2w-web.service`

- [ ] **Step 1: Write failing authorization tests**

```python
import pytest

from nx_control_auth import authorize_request

@pytest.mark.parametrize("path", [
    "/api/move", "/api/navigate", "/api/search_room",
    "/api/stand", "/api/balance", "/api/e_stop",
])
def test_control_endpoint_requires_token(path):
    decision = authorize_request(
        method="POST", path=path, headers={}, configured_token="test-token")
    assert decision.allowed is False
    assert decision.status_code == 401
```

Keep `/api/status`, `/api/version`, and mission media read-only. Decide explicitly whether e-stop is unauthenticated on the isolated LAN; if so, rate-limit it and document that exception in the test.

- [ ] **Step 2: Implement constant-time token verification**

Read `GO2W_CONTROL_TOKEN` from a root-readable environment file, accept
`Authorization: Bearer <token>`, restrict CORS to configured panel origins,
and reject state-changing requests without a valid token before parsing the
body.

- [ ] **Step 3: Make endpoints semantically explicit**

Remove fallback direct robot calls when `NavigationArbiter` is unavailable. Return `503 arbiter_unavailable`. Keep distinct `manual_release`, `stop`, `park`, and `e_stop` operations.

- [ ] **Step 4: Run and commit**

Run: `python -m pytest web/test_control_auth.py web/test_panel_navigation_contract.py -q`

Expected: PASS.

```bash
git add web/nx_control_auth.py web/test_control_auth.py web/nx_web_server.py \
  web/static/panel.html docker/go2w-web.service
git commit -m "security: authenticate robot control endpoints"
```

---

### Task 12: Make deployment atomic and subsystem-scoped

**Files:**
- Create: `docker/build_release.sh`
- Create: `docker/deploy_release.sh`
- Create: `docker/test_release_deploy_contract.py`
- Modify: `docker/go2w-motion.service`
- Modify: `docker/go2w-web.service`
- Modify: `docker/go2w-slam-nav.service`
- Retire after migration: `docker/deploy_nx.sh`, `docker/deploy_nx_web.sh`, `docker/deploy_nav2_bprime.sh`

- [ ] **Step 1: Write the release contract**

The artifact contains a manifest with release ID, hashes, subsystem, required services, and verification command. Installation goes to `/home/nx/go2w/releases/<release_id>/`; `/home/nx/go2w/current` is switched only after byte verification.

- [ ] **Step 2: Add subsystem restart rules**

| Changed subsystem | Services allowed to restart |
|---|---|
| motion | `go2w-motion` only, and only with explicit `--allow-motion-restart` |
| web/mission/perception | `go2w-web` only |
| Nav2 config/BT | `go2w-slam-nav`/Nav2 components; never motion |
| sensor bridge | affected sensor bridge only |

The current `deploy_nav2_bprime.sh` behavior that restarts motion during a Nav2 deployment must be deleted.

- [ ] **Step 3: Add rollback**

If health checks fail, restore the previous symlink and restart only the affected subsystem. Never use a motion restart as a general fault reset.

- [ ] **Step 4: Run and commit**

Run: `python -m pytest docker/test_release_deploy_contract.py docker/test_livox_deploy_contract.py -q`

Expected: PASS; contract tests prove Nav2/web releases cannot restart motion without an explicit flag.

```bash
git add docker/build_release.sh docker/deploy_release.sh \
  docker/test_release_deploy_contract.py docker/go2w-motion.service \
  docker/go2w-web.service docker/go2w-slam-nav.service \
  docker/deploy_nx.sh docker/deploy_nx_web.sh docker/deploy_nav2_bprime.sh
git commit -m "ops: deploy versioned NX subsystems atomically"
```

---

### Task 13: Run the offline release gate

**Files:**
- Create: `tools/verify_release.py`
- Modify: `docs/NX_REDEPLOY.md`
- Modify: `docs/PROJECT_STRUCTURE.md`
- Modify: `docs/SDK_CAPABILITIES.md`

- [ ] **Step 1: Encode one local gate**

`tools/verify_release.py` runs:

```bash
python -m compileall -q ai web src tools docker
python -m pytest docker web tools src/go2w_bridge/test -q
node web/test_map_contract.js
node web/test_panel_nav_state.js
```

It also rejects multiple production `NavigateToPose` clients, legacy motion state classes, unauthenticated control endpoints, absent release IDs, and Nav2 deploy scripts that restart motion.

- [ ] **Step 2: Update authoritative documentation**

Document the new ownership model, Unitree command table, startup policy, motion-service check, status schema, deployment layout, and staged hardware procedure. Mark old contradictory plans as superseded rather than deleting their history.

- [ ] **Step 3: Run the gate**

Run: `python tools/verify_release.py`

Expected: all Python/Node tests pass and the architecture checks report one motion machine, one SDK adapter, one navigation gateway, and one mission schema.

- [ ] **Step 4: Commit**

```bash
git add tools/verify_release.py docs/NX_REDEPLOY.md docs/PROJECT_STRUCTURE.md docs/SDK_CAPABILITIES.md
git commit -m "docs: define the optimized Go2W runtime contract"
```

---

### Task 14: Perform staged powered validation only after Tasks 1-13 pass

**Files:**
- Modify with recorded results: `docs/NX_REDEPLOY.md`
- Use read-only tools: `tools/diag_sport_requests.py`, `tools/diag_sport_state.py`, `tools/nav2_benchmark.py`

- [ ] **Step 1: Boot observation, no control command**

With the robot supported, area clear, and operator beside the hardware, record motion-switcher mode, sport mode, wheel `dq`, attitude, battery, errors, SDK API requests, and release ID from before motion service acquisition through PARKED. Acceptance:

- no `BalanceStand`, `Damp`, `RecoveryStand`, or non-zero `Move` during boot;
- if booted in mode 6, no `StandUp` is sent;
- if booted stationary in mode 1/3, exactly one `StandUp` is sent and mode 6 + stopped wheels is confirmed;
- if wheels remain moving, the node enters FAULT without a pose command.

- [ ] **Step 2: Characterize the terminal stop operation while supported**

With the wheels clear of the floor, compare `Move(0,0,0)` with one explicit
`StopMove()` after a bounded low-speed command. Record API receipts, sport mode,
wheel `dq`, and attitude. Enable `move_zero_then_stop_move` only if the single
operation clears the retained target without unloading or changing posture;
never validate it by repeating the call on a sliding floor robot.

- [ ] **Step 3: Manual session at minimum speed**

Require one `BalanceStand`, mode 1/3 feedback, then a bounded non-zero `Move`; release must zero first and park once. Verify measured displacement and final mode 6.

- [ ] **Step 4: Short Nav2 goal without obstacle**

Require healthy TF/scan/costmaps, action acceptance, non-zero `/cmd_vel_nav`, measured map displacement, explicit terminal result, and final PARKED.

- [ ] **Step 5: Box avoidance**

Place a movable box in the global path. Require it in `/scan_mid360` and both costmaps, a collision-free replanned path, measured clearance, no contact, and final PARKED.

- [ ] **Step 6: Bounded person then table mission**

For each target class, require synchronized evidence, finite map marker, stable deduplicated ID, annotated full frame/crop, mission report, and final PARKED.

- [ ] **Step 7: Voice last**

Only after all lower layers pass, send the exact PC voice command. Verify that its serialized mission request is byte-equivalent to the already-tested HTTP mission request.

---

## Priority, expected sequence, and stop conditions

1. **P0 safety:** Tasks 1-5 and 11-12, followed by their focused offline tests. Do not tune Nav2 or run missions before this gate.
2. **P1 navigation:** Tasks 6-7.
3. **P1 perception/exploration:** Tasks 8-10.
4. **Release gate:** Task 13 must pass only after Tasks 1-12 are complete.
5. **Powered validation:** Task 14 Steps 1-6, in order.
6. **P2 voice:** Task 14 Step 7 only after the lower layers pass.

Stop and return to diagnosis when any of these occur:

- an SDK pose operation appears without a matching explicit transition;
- wheel feedback remains non-zero after the bounded zero-hold window;
- motion-service mode is unknown or not `ai-w`;
- a Nav2 goal becomes terminal without measured displacement consistent with the goal;
- a marker is produced from unsynchronized detection/pose/range data;
- deployment hashes differ between source, NX manifest, `/dog_state`, and `/api/version`.

## Completion definition

The project is optimized only when the same release can demonstrate, with retained logs:

1. deterministic safe boot and process restart;
2. reliable manual motion and park;
3. click-to-go with obstacle avoidance and measured displacement;
4. bounded unknown-space exploration;
5. synchronized person/table localization with de-duplication and evidence;
6. map rendering and mission report persistence;
7. the voice command producing the same canonical mission request as the tested API path.
