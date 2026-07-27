"""SimSportGateway 契约 pin: Effect -> /cmd_vel Twist, 替代 SportGatewayClient socket.

spec 2026-07-25-real-fidelity-simulation-design.md §4.
单元测试用 mock publisher (不发真 ROS2 msg), 不依赖 Gazebo / livox build.
"""
import os
import sys
from unittest.mock import MagicMock

import pytest

# pytest 无 colcon install: sys.path 加 Python 包父目录
#   src/go2w_bridge  -> from go2w_bridge.X  (包在 src/go2w_bridge/go2w_bridge/)
#   src/go2w_sim     -> from go2w_sim.X     (包在 src/go2w_sim/go2w_sim/)
#   + go2w_bridge 内层 (motion_types.py 直 import 的 fallback)
_SRC = "/mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/src"
for _p in (os.path.join(_SRC, "go2w_bridge"),
           os.path.join(_SRC, "go2w_sim"),
           os.path.join(_SRC, "go2w_bridge", "go2w_bridge")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from go2w_sim.nodes.sim_sport_gateway import SimSportGateway  # noqa: E402
from go2w_bridge.motion_types import Effect  # noqa: E402


def _make_gateway():
    node = MagicMock()
    pub = MagicMock()
    node.create_publisher.return_value = pub
    return SimSportGateway(node), pub


def test_move_effect_publishes_velocity_twist():
    gw, pub = _make_gateway()
    receipt = gw.execute(Effect(operation="Move", sequence=1, arguments=(0.5, 0.0, 0.3)))
    assert receipt.code == 0
    assert receipt.transport_ok
    assert receipt.operation == "Move"
    assert receipt.sequence == 1
    pub.publish.assert_called_once()
    twist = pub.publish.call_args[0][0]
    assert twist.linear.x == pytest.approx(0.5)
    assert twist.linear.y == pytest.approx(0.0)
    assert twist.angular.z == pytest.approx(0.3)


def test_movezero_publishes_zero_twist():
    gw, pub = _make_gateway()
    receipt = gw.execute(Effect(operation="MoveZero", sequence=2))
    assert receipt.code == 0
    assert receipt.transport_ok
    assert receipt.operation == "MoveZero"
    twist = pub.publish.call_args[0][0]
    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0


def test_initialize_returns_ai_w_zero_code():
    gw, _ = _make_gateway()
    result = gw.initialize()
    assert result.code == 0
    assert result.motion_service == "ai-w"


def test_check_motion_service_matches_initialize():
    gw, _ = _make_gateway()
    a = gw.initialize()
    b = gw.check_motion_service()
    assert a.code == b.code
    assert a.motion_service == b.motion_service


def test_close_publishes_zero_twist():
    gw, pub = _make_gateway()
    gw.close()
    assert pub.publish.called
    twist = pub.publish.call_args[0][0]
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_unknown_operation_publishes_zero_safe_stop():
    """未知 operation 应安全止动 (零速), 不抛 — 同 SportGatewayClient 容错语义."""
    gw, pub = _make_gateway()
    receipt = gw.execute(Effect(operation="StandUp", sequence=3))
    assert receipt.code == 0
    twist = pub.publish.call_args[0][0]
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_partial_arguments_does_not_crash():
    """arguments 长度 < 3 应补零, 不 IndexError."""
    gw, pub = _make_gateway()
    receipt = gw.execute(Effect(operation="Move", sequence=4, arguments=(0.6,)))
    assert receipt.code == 0
    twist = pub.publish.call_args[0][0]
    assert twist.linear.x == pytest.approx(0.6)
    assert twist.linear.y == 0.0
    assert twist.angular.z == 0.0
