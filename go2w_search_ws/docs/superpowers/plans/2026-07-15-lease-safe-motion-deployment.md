# Lease-Safe Motion Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the Unitree `sport_lease` continuously owned and force zero velocity while the motion policy, Web, Nav2, or release is restarted, while preserving raw pre-fault evidence across NX power cycles.

**Architecture:** Move the leased `SportClient` into a small, stable NX-local Unix-socket gateway that normal releases never restart. `nx_motion_node.py` remains the feedback-driven policy owner but executes its existing `Effect` objects through a synchronous local adapter; if that client disappears, the gateway continues renewing the lease and issuing zero velocity. A separate read-only process subscribes to raw sport/low state and writes a bounded JSONL incident trail outside journald, so the high-rate state stream cannot starve the one-second SDK lease.

**Tech Stack:** Python 3.10, Unitree SDK2 Python, Unix domain sockets, ROS 2 Humble/rclpy, systemd, pytest, Bash release tooling

---

## File structure

- Create `src/go2w_bridge/go2w_bridge/sport_gateway_protocol.py`: length-prefixed JSON request/response validation shared by gateway and client.
- Create `src/go2w_bridge/go2w_bridge/sport_gateway_server.py`: stable lease owner, SDK allowlist, zero-velocity watchdog, and Unix-socket server.
- Create `src/go2w_bridge/go2w_bridge/sport_gateway_client.py`: synchronous adapter implementing the existing `initialize()`, `check_motion_service()`, and `execute(Effect)` boundary.
- Create `src/go2w_bridge/go2w_bridge/safety_event_recorder.py`: bounded, durable state/command transition recorder with rotation.
- Create `src/go2w_bridge/go2w_bridge/nx_sport_gateway.py`: production entry point wiring only the Unitree control clients, command recorder, and server.
- Create `src/go2w_bridge/go2w_bridge/nx_safety_observer.py`: independent read-only raw DDS subscriber and state recorder.
- Modify `src/go2w_bridge/go2w_bridge/nx_motion_node.py`: remove direct SDK/lease construction and connect to the local gateway.
- Create `docker/go2w-sport-gateway.service`: stable service ordered before `go2w-motion.service`.
- Modify `docker/go2w-motion.service`: require the gateway socket service; restarting motion must not release the lease.
- Modify `docker/build_release.sh`: package the gateway runtime and service.
- Modify `docker/deploy_release.sh`: install but never restart an already-active gateway during normal `motion`/`all` deployment; start it only when absent on first installation.
- Modify `tools/verify_release.py`: enforce gateway packaging, ordering, and non-restart invariants.
- Create `src/go2w_bridge/test/test_sport_gateway_protocol.py`.
- Create `src/go2w_bridge/test/test_sport_gateway_server.py`.
- Create `src/go2w_bridge/test/test_sport_gateway_client.py`.
- Create `src/go2w_bridge/test/test_safety_event_recorder.py`.
- Modify `src/go2w_bridge/test/test_motion_node_v2_contract.py`.
- Modify `docker/test_release_deploy_contract.py`.

### Task 1: Define a strict local gateway protocol

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/sport_gateway_protocol.py`
- Test: `src/go2w_bridge/test/test_sport_gateway_protocol.py`

- [ ] **Step 1: Write the failing protocol tests**

```python
import pytest

from go2w_bridge.sport_gateway_protocol import (
    ProtocolError,
    decode_request,
    decode_response,
    encode_frame,
)


def test_request_accepts_only_versioned_allowlisted_effects():
    request = decode_request({
        "version": 1,
        "request_id": "req-1",
        "operation": "Move",
        "arguments": [0.1, 0.0, 0.0],
    })
    assert request.operation == "Move"
    assert request.arguments == (0.1, 0.0, 0.0)


@pytest.mark.parametrize("operation", ["Damp", "StandDown", "RecoveryStand"])
def test_request_rejects_unload_operations(operation):
    with pytest.raises(ProtocolError, match="unsupported operation"):
        decode_request({
            "version": 1,
            "request_id": "req-1",
            "operation": operation,
            "arguments": [],
        })


def test_response_requires_matching_request_id_and_integer_code():
    response = decode_response({
        "version": 1,
        "request_id": "req-1",
        "operation": "MoveZero",
        "code": 0,
        "motion_service": "ai-w",
    }, expected_request_id="req-1")
    assert response.code == 0
    assert encode_frame(response.to_dict()).endswith(b"\n")
```

- [ ] **Step 2: Run the protocol tests and verify RED**

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_protocol.py`

Expected: collection fails with `ModuleNotFoundError: go2w_bridge.sport_gateway_protocol`.

- [ ] **Step 3: Implement the minimal protocol**

Implement frozen `GatewayRequest`/`GatewayResponse` dataclasses, newline-delimited UTF-8 JSON framing capped at 4096 bytes, finite numeric validation, exact argument counts (`Move`=3; `MoveZero`, `StandUp`, `BalanceStand`, `StopMove`, `CheckMode`=0), and the fixed version `1`. The production allowlist must omit `Damp`, `StandDown`, and `RecoveryStand`.

- [ ] **Step 4: Run the protocol tests and verify GREEN**

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_protocol.py`

Expected: all protocol tests pass.

### Task 2: Build the stable lease owner and zero-command watchdog

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/sport_gateway_server.py`
- Test: `src/go2w_bridge/test/test_sport_gateway_server.py`

- [ ] **Step 1: Write failing server tests with a real Unix socket and fake SDK**

```python
import json
import socket
import time

from go2w_bridge.sport_gateway_server import SportGatewayServer


class FakeSport:
    def __init__(self):
        self.calls = []

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", (vx, vy, vyaw)))
        return 0

    def BalanceStand(self):
        self.calls.append(("BalanceStand", ()))
        return 0


class FakeSwitcher:
    def CheckMode(self):
        return 0, {"name": "ai-w"}


def test_client_disconnect_keeps_server_alive_and_repeats_zero(tmp_path):
    sport = FakeSport()
    server = SportGatewayServer(
        sport, FakeSwitcher(), socket_path=tmp_path / "sport.sock",
        command_timeout=0.05, zero_period=0.02,
    )
    server.start()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.connect(str(tmp_path / "sport.sock"))
        client.sendall((json.dumps({
            "version": 1, "request_id": "1", "operation": "Move",
            "arguments": [0.1, 0.0, 0.0],
        }) + "\n").encode())
        client.recv(4096)
    time.sleep(0.09)
    server.close()
    zero_calls = [call for call in sport.calls if call == ("Move", (0.0, 0.0, 0.0))]
    assert len(zero_calls) >= 2


def test_second_policy_client_cannot_execute_concurrently(tmp_path):
    sport = FakeSport()
    socket_path = tmp_path / "sport.sock"
    server = SportGatewayServer(
        sport, FakeSwitcher(), socket_path=socket_path,
        command_timeout=0.2, zero_period=0.05,
    )
    server.start()
    first = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    second = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        first.connect(str(socket_path))
        first.sendall((json.dumps({
            "version": 1, "request_id": "owner", "operation": "CheckMode",
            "arguments": [],
        }) + "\n").encode())
        assert json.loads(first.recv(4096))["code"] == 0

        second.settimeout(0.3)
        second.connect(str(socket_path))
        second.sendall((json.dumps({
            "version": 1, "request_id": "intruder", "operation": "Move",
            "arguments": [0.2, 0.0, 0.0],
        }) + "\n").encode())
        response = json.loads(second.recv(4096))
        assert response["code"] == 409
        assert ("Move", (0.2, 0.0, 0.0)) not in sport.calls
    finally:
        first.close()
        second.close()
        server.close()
```

- [ ] **Step 2: Run the server tests and verify RED**

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_server.py`

Expected: collection fails because `SportGatewayServer` does not exist.

- [ ] **Step 3: Implement the minimal serialized server**

Implement one SDK actor thread and one accepted policy connection. The actor owns every SDK call. It executes validated requests synchronously, returns the integer SDK code, and calls `Move(0,0,0)` at `zero_period` whenever no valid request has arrived for `command_timeout` or the policy socket disconnects. Socket/client errors must close only the policy connection; they must never exit the SDK actor or stop lease renewal.

- [ ] **Step 4: Add and pass transport-failure tests**

Add tests for malformed JSON, over-size frames, a fake SDK exception, and reconnect after disconnect. Each must receive a bounded error response or reconnect cleanly while later watchdog zero calls continue.

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_server.py`

Expected: all server tests pass.

### Task 3: Preserve the existing motion-controller adapter contract

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/sport_gateway_client.py`
- Modify: `src/go2w_bridge/go2w_bridge/nx_motion_node.py`
- Test: `src/go2w_bridge/test/test_sport_gateway_client.py`
- Test: `src/go2w_bridge/test/test_motion_node_v2_contract.py`

- [ ] **Step 1: Write failing adapter tests**

```python
from go2w_bridge.motion_types import Effect
from go2w_bridge.sport_gateway_client import SportGatewayClient


def test_gateway_client_maps_effect_and_receipt(fake_gateway):
    client = SportGatewayClient(fake_gateway.socket_path, timeout=0.2)
    initialized = client.initialize()
    assert initialized.code == 0
    assert initialized.motion_service == "ai-w"
    receipt = client.execute(Effect("MoveZero", sequence=7))
    assert receipt.operation == "MoveZero"
    assert receipt.sequence == 7
    assert receipt.code == 0


def test_gateway_client_rejects_unload_effect_before_transport(fake_gateway):
    client = SportGatewayClient(fake_gateway.socket_path, timeout=0.2)
    with pytest.raises(ValueError, match="unsupported SDK effect"):
        client.execute(Effect("Damp", sequence=8))
```

- [ ] **Step 2: Run the adapter tests and verify RED**

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_client.py`

Expected: import fails because the adapter does not exist.

- [ ] **Step 3: Implement the client and switch the node boundary**

Implement `SportGatewayClient` with a persistent Unix socket, monotonically unique request IDs, exact response-ID matching, and one reconnect attempt only for `CheckMode`. In `NxMotionNode._try_initialize_sdk`, replace `ChannelFactory`, `SportClient(enableLease=True)`, `MotionSwitcherClient`, and `UnitreeSportAdapter` with `SportGatewayClient(os.environ.get("GO2W_SPORT_GATEWAY_SOCKET", "/run/go2w-sport-gateway/sport.sock"))`. Keep `MotionController`, `Go2WMotionMachine`, callbacks, and `Effect` semantics unchanged.

- [ ] **Step 4: Strengthen the structural contract**

Add assertions that `nx_motion_node.py` contains `SportGatewayClient`, contains no `SportClient(` or `MotionSwitcherClient(`, and that only `nx_sport_gateway.py` constructs `SportClient(enableLease=True)`.

- [ ] **Step 5: Run adapter and motion tests and verify GREEN**

Run: `python -m pytest -q src/go2w_bridge/test/test_sport_gateway_client.py src/go2w_bridge/test/test_motion_node_v2_contract.py src/go2w_bridge/test/test_motion_machine.py`

Expected: all selected tests pass.

### Task 4: Persist raw incident evidence outside journald

**Files:**
- Create: `src/go2w_bridge/go2w_bridge/safety_event_recorder.py`
- Create: `src/go2w_bridge/go2w_bridge/nx_sport_gateway.py`
- Test: `src/go2w_bridge/test/test_safety_event_recorder.py`

- [ ] **Step 1: Write failing recorder tests**

```python
import json
import os

from go2w_bridge.safety_event_recorder import SafetyEventRecorder


def test_recorder_persists_only_changes_and_safety_events(tmp_path):
    path = tmp_path / "events.jsonl"
    recorder = SafetyEventRecorder(path, max_bytes=4096, backups=2)
    sample = {"mode": 6, "error_code": 0, "wheel_dq": [0, 0, 0, 0]}
    recorder.record_state(sample)
    recorder.record_state(sample)
    recorder.record_command("MoveZero", 0, reason="watchdog")
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["kind"] for row in rows] == ["state", "command"]


def test_nonzero_error_is_fsynced_and_rotation_keeps_backups(tmp_path, monkeypatch):
    path = tmp_path / "events.jsonl"
    fsync_calls = []
    real_fsync = os.fsync

    def record_fsync(fd):
        fsync_calls.append(fd)
        real_fsync(fd)

    monkeypatch.setattr(os, "fsync", record_fsync)
    recorder = SafetyEventRecorder(path, max_bytes=220, backups=2)
    recorder.record_state({
        "mode": 7, "error_code": 3104, "wheel_dq": [0, 0, 0, 0],
    })
    for index in range(12):
        recorder.record_state({
            "mode": 7 if index % 2 else 6,
            "error_code": 3104 + index,
            "wheel_dq": [float(index), 0, 0, 0],
        })
    assert fsync_calls
    assert path.with_name("events.jsonl.1").exists()
```

- [ ] **Step 2: Run the recorder tests and verify RED**

Run: `python -m pytest -q src/go2w_bridge/test/test_safety_event_recorder.py`

Expected: import fails because the recorder does not exist.

- [ ] **Step 3: Implement bounded durable recording**

Write one JSON object per line with wall-clock ISO timestamp, monotonic time, event kind, raw mode/error, roll/pitch, wheel speeds, motor lost flags, battery, SDK operation/code/reason, and gateway process epoch. Suppress identical normal samples, but always write mode changes, nonzero errors, unsafe attitude, motor loss, SDK failures, client connect/disconnect, gateway start/stop, and every nonzero motion command. Rotate at 4 MiB with four backups; `flush`+`fsync` safety events.

- [ ] **Step 4: Wire the production gateway**

`nx_sport_gateway.py` must initialize `ChannelFactory` once, construct exactly one `SportClient(enableLease=True)`, wait for a valid lease, and send zero twice before accepting a policy client. It writes command/event receipts to `/home/nx/go2w/safety-events/gateway.jsonl`; it must never import ROS, subscribe to high-rate raw state, or expose `Damp`, `StandDown`, or `RecoveryStand`.

`nx_safety_observer.py` independently subscribes read-only to `rt/lf/sportmodestate` and `rt/lowstate` and writes snapshots to `/home/nx/go2w/safety-events/events.jsonl`. It must never construct a `SportClient` or `MotionSwitcherClient`. This separation supersedes the original same-participant design after live NX measurements showed that raw LowState receive load caused repeated lease-renewal timeouts (`3104`, then `3206`) despite zero NIC loss.

- [ ] **Step 5: Run recorder and structural tests and verify GREEN**

Run: `python -m pytest -q src/go2w_bridge/test/test_safety_event_recorder.py src/go2w_bridge/test/test_motion_node_v2_contract.py`

Expected: all selected tests pass.

### Task 5: Make normal release deployment lease-continuous

**Files:**
- Create: `docker/go2w-sport-gateway.service`
- Modify: `docker/go2w-motion.service`
- Modify: `docker/build_release.sh`
- Modify: `docker/deploy_release.sh`
- Modify: `docker/test_release_deploy_contract.py`
- Modify: `tools/verify_release.py`

- [ ] **Step 1: Write failing deployment contract tests**

```python
def test_full_deploy_never_restarts_active_sport_gateway():
    source = text("deploy_release.sh")
    assert 'gateway_was_active="$(systemctl is-active' in source
    assert 'if [ "$gateway_was_active" != "active" ]' in source
    restart_loop = source[source.index("for service in $restart_units"):]
    assert "go2w-sport-gateway.service" not in restart_loop.split("done", 1)[0]


def test_motion_service_requires_stable_gateway():
    gateway = text("go2w-sport-gateway.service")
    motion = text("go2w-motion.service")
    assert "RuntimeDirectory=go2w-sport-gateway" in gateway
    assert "Restart=always" in gateway
    assert "Before=go2w-motion.service" in gateway
    assert "Requires=go2w-sport-gateway.service" in motion
    assert "After=go2w-sport-gateway.service" in motion
```

- [ ] **Step 2: Run deployment tests and verify RED**

Run: `python -m pytest -q docker/test_release_deploy_contract.py tools/test_verify_release.py`

Expected: failures report missing gateway service/order/non-restart markers.

- [ ] **Step 3: Add the stable systemd boundary**

The gateway service must use `RuntimeDirectory=go2w-sport-gateway`, `Restart=always`, `RestartSec=0.2`, `KillSignal=SIGINT`, the commissioned `DOG_INTERFACE`, and an `ExecStartPre` reachability gate. `go2w-motion.service` must point at the socket through `GO2W_SPORT_GATEWAY_SOCKET` and require/come after the gateway.

- [ ] **Step 4: Change release restart and rollback behavior**

Install the gateway unit with other managed units, but do not add it to `restart_units`, `rollback_units`, or `restore_active_state`. Before restarting motion, assert the gateway is active and the socket exists. On first installation only, start the gateway and wait for a read-only `CheckMode` response before starting motion. If a release later changes gateway code/unit, leave the active process untouched and write `/home/nx/go2w/pending-gateway-release` for activation at the next whole-machine reboot.

- [ ] **Step 5: Run deployment contracts and verify GREEN**

Run: `python -m pytest -q docker/test_release_deploy_contract.py tools/test_verify_release.py`

Expected: all release/deployment tests pass.

### Task 6: Offline end-to-end restart proof

**Files:**
- Create: `src/go2w_bridge/test/test_gateway_motion_restart_integration.py`
- Modify: `docker/test_release_deploy_contract.py`

- [ ] **Step 1: Write the failing integration test**

Start the fake gateway with a one-second fake lease term, connect a real `SportGatewayClient`, execute a nonzero `Move`, close the client to simulate `systemctl restart go2w-motion`, wait 1.5 seconds, reconnect a second client, and assert:

```python
assert fake_lease.renewal_gap_max < 0.45
assert fake_lease.expired_count == 0
assert fake_sport.nonzero_after_disconnect == 0
assert fake_sport.zero_calls_after_disconnect >= 10
assert second_client.initialize().motion_service == "ai-w"
```

- [ ] **Step 2: Run the integration test and verify RED**

Run: `python -m pytest -q src/go2w_bridge/test/test_gateway_motion_restart_integration.py`

Expected: fail until the gateway survives client replacement and continues renew/zero loops independently.

- [ ] **Step 3: Make the minimum lifecycle fixes and verify GREEN**

Adjust only gateway shutdown/reconnect/watchdog lifecycle revealed by the test. Do not alter motion state transitions.

Run: `python -m pytest -q src/go2w_bridge/test/test_gateway_motion_restart_integration.py`

Expected: the lease never expires and only zero commands occur between clients.

- [ ] **Step 4: Run the complete offline gate**

Run: `python -m pytest -q`

Run: `bash tools/ci_offline_gate.sh` if present; otherwise run `python tools/verify_release.py` and the repository's existing JavaScript contract tests.

Expected: all existing and new tests pass with no warnings attributable to this change.

- [ ] **Step 5: Stop before deployment**

Do not deploy or restart any NX service in this task. Capture the passing test output and request a separate maintenance window for the one-time gateway bootstrap. During that bootstrap the dog must be physically unloaded/supported or powered in a vendor-approved maintenance posture; “clear area” alone is not sufficient.

## Self-review

- Spec coverage: normal release restarts no longer release the lease; missing policy traffic fails to continuous zero; unsafe SDK calls remain absent; pre-fault evidence survives NX power cycles; deployment remains fail-closed.
- Placeholder scan: the plan contains no `TBD`, `TODO`, ellipses, or deferred implementation placeholders.
- Type consistency: protocol version, operation names, socket path, adapter methods, service name, and recorder path are consistent across all tasks.
