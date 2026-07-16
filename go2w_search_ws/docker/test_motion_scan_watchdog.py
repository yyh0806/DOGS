"""Release-level contracts for the consolidated motion safety architecture."""

from pathlib import Path
from types import SimpleNamespace
import math
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "src" / "go2w_bridge"
if str(BRIDGE) not in sys.path:
    sys.path.insert(0, str(BRIDGE))

from go2w_bridge.motion_safety import (  # noqa: E402
    ScanFreshnessWatchdog,
    compensate_pure_turn_creep,
)


def _scan(stamp=1, distance=2.0, frame_id="base_link"):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=stamp, nanosec=0),
        ),
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 180.0,
        range_min=0.05,
        range_max=30.0,
        ranges=[distance] * 360,
    )


def test_scan_gate_is_fail_closed_and_zero_is_always_safe():
    clock = [10.0]
    watchdog = ScanFreshnessWatchdog(timeout=1.8, clock=lambda: clock[0])

    assert watchdog.filter_nav_velocity((0.2, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert watchdog.filter_nav_velocity((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert watchdog.observe_scan(_scan()) is True
    assert watchdog.filter_nav_velocity((0.2, 0.0, 0.0)) == (0.2, 0.0, 0.0)
    clock[0] = 12.0
    assert watchdog.filter_nav_velocity((0.2, 0.0, 0.0)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("bad_scan", [
    None,
    _scan(frame_id="laser"),
    _scan(distance=float("nan")),
    _scan(distance=float("-inf")),
])
def test_malformed_or_wrong_frame_scan_never_opens_nav_gate(bad_scan):
    watchdog = ScanFreshnessWatchdog(timeout=1.8, clock=lambda: 10.0)
    assert watchdog.observe_scan(bad_scan) is False
    assert watchdog.nav_must_stop() is True


def test_pure_turn_requires_clearance_and_latches_guard():
    watchdog = ScanFreshnessWatchdog(
        timeout=1.8, clock=lambda: 10.0, pure_turn_clearance=0.95)
    assert watchdog.observe_scan(_scan(distance=0.7)) is True
    assert watchdog.filter_nav_velocity((0.0, 0.0, 0.1)) == (0.0, 0.0, 0.0)
    assert watchdog.nav_guard_reason() == "pure_turn_clearance"


def test_turn_creep_compensation_never_creates_reverse_velocity():
    result = compensate_pure_turn_creep(
        (0.0, 0.0, 0.12), gain=1.0, maximum=0.15,
        linear_epsilon=0.02, angular_threshold=0.05)
    assert result == (0.0, 0.0, 0.12)


def test_motion_node_has_one_machine_gateway_and_callback_event_boundary():
    source = (BRIDGE / "go2w_bridge" / "nx_motion_node.py").read_text(
        encoding="utf-8")

    assert source.count("machine = Go2WMotionMachine(") == 1
    assert source.count("adapter = SportGatewayClient(") == 1
    assert "UnitreeSportAdapter(" not in source
    assert "SportClient(" not in source
    assert 'self._enqueue("intent"' in source
    assert 'self._enqueue("scan"' in source
    assert 'LaserScan, "/scan_mid360"' in source
    assert '_parameter_value(self, "nav_scan_timeout", 1.8)' in source
    for forbidden in ("DriveSessionModel", "LayeredMotionState", ".Damp(",
                      ".RecoveryStand("):
        assert forbidden not in source


def test_startup_policy_contains_no_blind_pose_command():
    source = (BRIDGE / "go2w_bridge" / "nx_motion_node.py").read_text(
        encoding="utf-8")
    actor = source.split("def _actor_loop", 1)[1]
    before_loop = actor.split("while not stop_requested", 1)[0]

    assert "BalanceStand" not in before_loop
    assert "StandUp" not in before_loop
    assert "Damp" not in before_loop
    assert "RecoveryStand" not in before_loop


def test_nav_deploy_never_restarts_or_replaces_motion_runtime():
    source = (ROOT / "docker" / "deploy_nav2_bprime.sh").read_text(
        encoding="utf-8")
    assert "nx_motion_node.py" not in source
    assert "restart go2w-motion" not in source
    assert "stop go2w-motion" not in source
