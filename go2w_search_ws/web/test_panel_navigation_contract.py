"""Cross-file and executable contracts for Panel click-to-Nav2 integration."""

import importlib
import http.client
import json
import math
import sys
import threading
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
_CONTROL_TOKEN = "panel-contract-token-0123456789-ABCDEF"


@pytest.fixture(autouse=True)
def _authenticated_control_requests(monkeypatch):
    """Exercise control routes through the same Bearer contract as the panel."""

    monkeypatch.setenv("GO2W_CONTROL_TOKEN", _CONTROL_TOKEN)
    original_request = http.client.HTTPConnection.request

    def request(connection, method, url, body=None, headers=None, *,
                encode_chunked=False):
        authorized_headers = dict(headers or {})
        if str(method).upper() not in {"GET", "HEAD", "OPTIONS"}:
            authorized_headers.setdefault(
                "Authorization", f"Bearer {_CONTROL_TOKEN}")
        return original_request(
            connection, method, url, body=body, headers=authorized_headers,
            encode_chunked=encode_chunked)

    monkeypatch.setattr(http.client.HTTPConnection, "request", request)


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_search_room_button_calls_canonical_current_room_mission_endpoint():
    panel = read("web/static/panel.html")

    assert 'onclick="searchRoom()"' in panel
    assert "controlFetch('/api/search_room'" in panel
    assert "room: 'current_room'" in panel
    # searchRoom 参数化 (spec §3.2): 默认 person, 可传任意类/多类组合/all_objects
    assert "let tc = targetClasses || ['person']" in panel
    assert "target_classes: tc" in panel
    assert "tc === 'all_objects'" in panel


def test_panel_has_search_target_buttons():
    """搜人/搜桌椅/搜物体 三按钮 (spec §3.2)."""
    panel = read("web/static/panel.html")
    assert "searchRoom(['person'])" in panel
    assert "searchRoom(['dining table','chair'])" in panel
    assert "searchRoom('all_objects')" in panel


def test_search_room_http_endpoint_admits_spatial_frontier_mission(
        monkeypatch, tmp_path):
    web = _load_web_server(monkeypatch)
    admitted = []

    class TaskManager:
        def add_list(self, tasks, *, reason):
            admitted.append((list(tasks), reason))
            return {"ok": True, "phase": "active", "owner": "tasks"}

    web.task_mgr = TaskManager()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(
            *server.server_address, timeout=1.0)
        connection.request(
            "POST", "/api/search_room",
            body=json.dumps({
                "room": "current_room",
                "target_classes": ["person"],
            }),
            headers={"Content-Type": "application/json"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        thread.join(timeout=1.0)
        server.server_close()

    assert response.status == 200
    assert payload["ok"] is True
    assert payload["mission_request"]["room"] == "current_room"
    assert payload["mission_request"]["frontier_spacing_m"] == pytest.approx(1.5)
    assert len(admitted) == 1
    tasks, reason = admitted[0]
    assert reason == "search_room"
    assert tasks[0].type == "search_room"
    assert tasks[0].params["search_strategy"] == "frontier_explore"
    assert tasks[0].params["frontier_spacing_m"] == pytest.approx(1.5)


def _install_ros_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        pass

    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")

    class Odometry:
        pass

    nav_msgs_msg.Odometry = Odometry
    nav_msgs.msg = nav_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class Imu:
        pass

    class LaserScan:
        pass

    sensor_msgs_msg.Imu = Imu
    sensor_msgs_msg.LaserScan = LaserScan
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        pass

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    nx_slam_map = types.ModuleType("nx_slam_map")

    class ObstacleGridAccumulator:
        def __init__(self, *args, **kwargs):
            self._points = []

        def update(self, points):
            self._points = list(points)
            return list(self._points)

        def points(self):
            return list(self._points)

    nx_slam_map.ObstacleGridAccumulator = ObstacleGridAccumulator

    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "nx_slam_map": nx_slam_map,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)


def _load_web_server(monkeypatch):
    _install_ros_stubs(monkeypatch)
    web_dir = str(ROOT / "web")
    if web_dir not in sys.path:
        sys.path.insert(0, web_dir)
    sys.modules.pop("nx_web_server", None)
    return importlib.import_module("nx_web_server")


def _new_localization_node(web, *, timeout=1.0, max_tilt_deg=10.0):
    node = web.NxWebNode.__new__(web.NxWebNode)
    node._lock = threading.Lock()
    node.localization_timeout = timeout
    node.localization_max_tilt = math.radians(max_tilt_deg)
    node._map_x = 0.0
    node._map_y = 0.0
    node._map_yaw = 0.0
    node._localization_count = 0
    node._localization_received_monotonic = None
    node._localization_frame_id = ""
    node._localization_child_frame_id = ""
    node._localization_stamp = {"sec": 0, "nanosec": 0}
    node._localization_valid = False
    node._localization_reason = "not_received"
    node._odom_x = 0.0
    node._odom_y = 0.0
    node._odom_z = 0.0
    node._odom_yaw = 0.0
    node._odom_count = 0
    node._odom_received_monotonic = None
    node._odom_frame_id = ""
    node._odom_child_frame_id = ""
    node._odom_stamp = {"sec": 0, "nanosec": 0}
    node._odom_valid = False
    node._odom_reason = "not_received"
    return node


class _LoggerStub:
    def warning(self, *args, **kwargs):
        pass


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return types.SimpleNamespace(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _localization_message(
    *,
    x=2.5,
    y=-1.25,
    roll=0.0,
    pitch=0.0,
    yaw=1.2,
    frame_id="map",
    child_frame_id="base_link",
):
    position = types.SimpleNamespace(x=x, y=y, z=0.0)
    pose = types.SimpleNamespace(
        position=position,
        orientation=_quat_from_rpy(roll, pitch, yaw),
    )
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id=frame_id,
            stamp=types.SimpleNamespace(sec=123, nanosec=456000000),
        ),
        child_frame_id=child_frame_id,
        pose=types.SimpleNamespace(pose=pose),
    )


def test_valid_localization_quaternion_becomes_map_yaw(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = _new_localization_node(web)
    now = {"value": 100.0}
    monkeypatch.setattr(web.time, "monotonic", lambda: now["value"])

    node._on_localization_pose(
        _localization_message(
            roll=math.radians(4.0),
            pitch=math.radians(-3.0),
            yaw=1.2,
        )
    )
    now["value"] = 100.25
    health = node.get_localization_health()

    assert health["healthy"] is True
    assert health["reason"] == "ok"
    assert health["frame_id"] == "map"
    assert health["child_frame_id"] == "base_link"
    assert health["stamp"] == {"sec": 123, "nanosec": 456000000}
    assert health["x"] == pytest.approx(2.5)
    assert health["y"] == pytest.approx(-1.25)
    assert health["yaw"] == pytest.approx(1.2)
    assert health["age_sec"] == pytest.approx(0.25)


def test_lio_odometry_is_exposed_as_diagnostic_pose_without_replacing_map_pose(
    monkeypatch,
):
    web = _load_web_server(monkeypatch)
    node = _new_localization_node(web)
    now = {"value": 200.0}
    monkeypatch.setattr(web.time, "monotonic", lambda: now["value"])
    node._on_localization_pose(_localization_message(x=8.37, y=-1.44, yaw=2.70))

    odom = _localization_message(
        x=11.17,
        y=5.39,
        yaw=1.76,
        frame_id="odom",
    )
    node._on_lio_odometry(odom)
    now["value"] = 200.2

    localization = node.get_localization_health()
    diagnostic = node.get_odometry_snapshot()
    assert localization["x"] == pytest.approx(8.37)
    assert localization["y"] == pytest.approx(-1.44)
    assert diagnostic["healthy"] is True
    assert diagnostic["frame_id"] == "odom"
    assert diagnostic["child_frame_id"] == "base_link"
    assert diagnostic["x"] == pytest.approx(11.17)
    assert diagnostic["y"] == pytest.approx(5.39)
    assert diagnostic["yaw"] == pytest.approx(1.76)
    assert diagnostic["age_sec"] == pytest.approx(0.2)


@pytest.mark.parametrize(
    ("message", "reason"),
    [
        (_localization_message(frame_id="odom"), "invalid_frame"),
        (_localization_message(child_frame_id="body"), "invalid_child_frame"),
        (_localization_message(x=float("nan")), "nonfinite_pose"),
        (_localization_message(pitch=math.radians(11.0)), "excessive_tilt"),
    ],
)
def test_invalid_localization_is_immediately_unhealthy(monkeypatch, message, reason):
    web = _load_web_server(monkeypatch)
    node = _new_localization_node(web)
    now = {"value": 10.0}
    monkeypatch.setattr(web.time, "monotonic", lambda: now["value"])
    node._on_localization_pose(_localization_message())

    now["value"] = 10.1
    node._on_localization_pose(message)
    health = node.get_localization_health()

    assert health["healthy"] is False
    assert health["reason"] == reason


def test_zero_and_nonfinite_localization_quaternions_are_rejected(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = _new_localization_node(web)
    monkeypatch.setattr(web.time, "monotonic", lambda: 10.0)

    zero = _localization_message()
    zero.pose.pose.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
    node._on_localization_pose(zero)
    assert node.get_localization_health()["reason"] == "invalid_quaternion"

    nonfinite = _localization_message()
    nonfinite.pose.pose.orientation.x = float("inf")
    node._on_localization_pose(nonfinite)
    assert node.get_localization_health()["reason"] == "nonfinite_pose"

    overflow = _localization_message()
    overflow.pose.pose.orientation = types.SimpleNamespace(
        x=1e308, y=1e308, z=1e308, w=1e308
    )
    node._on_localization_pose(overflow)
    assert node.get_localization_health()["reason"] == "invalid_quaternion"


@pytest.mark.parametrize(
    ("timeout", "max_tilt_deg"),
    [
        (float("nan"), 10.0),
        (float("inf"), 10.0),
        (0.0, 10.0),
        (1.0, float("nan")),
        (1.0, 0.0),
        (1.0, 90.0),
    ],
)
def test_localization_safety_parameters_reject_nonfinite_or_unsafe_values(
    monkeypatch, timeout, max_tilt_deg
):
    web = _load_web_server(monkeypatch)

    with pytest.raises(ValueError):
        web.NxWebNode._validate_localization_safety_parameters(
            timeout, max_tilt_deg
        )


def test_localization_health_uses_reception_monotonic_age(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = _new_localization_node(web, timeout=1.0)
    now = {"value": 50.0}
    monkeypatch.setattr(web.time, "monotonic", lambda: now["value"])
    node._on_localization_pose(_localization_message())

    now["value"] = 51.001
    health = node.get_localization_health()

    assert health["healthy"] is False
    assert health["reason"] == "stale"
    assert health["age_sec"] == pytest.approx(1.001)
    assert health["timeout_sec"] == pytest.approx(1.0)


def test_parked_dog_is_activatable_and_only_active_nav_session_is_ready(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.RLock(),
        _dog_state="UNKNOWN",
        _dog_vx=0.0,
        _dog_vy=0.0,
        _dog_vyaw=0.0,
        _motion_sdk_ready=False,
        _motion_nav_scan_fresh=False,
        _motion_battery_soc=None,
        _motion_drive_fault=None,
        _motion_sport_mode=None,
        _motion_wheel_dq=None,
        _motion_wheel_activation_phase="unknown",
        _motion_drive_session="startup",
        _motion_drive_session_owner=None,
        _motion_drive_session_phase="startup",
        _motion_drive_session_reason="waiting_for_feedback",
        _last_state_t=0.0,
        state_timeout=3.0,
        get_localization_health=lambda: {
            "healthy": True, "reason": "ok", "age_sec": 0.01
        },
        get_logger=lambda: _LoggerStub(),
    )
    node._on_dog_state = types.MethodType(web.NxWebNode._on_dog_state, node)
    node.get_navigation_readiness = types.MethodType(
        web.NxWebNode.get_navigation_readiness, node
    )
    now = {"value": 100.0}
    monkeypatch.setattr(web.time, "time", lambda: now["value"])

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STOPPED",
        "sdk_ready": False,
        "nav_scan_fresh": True,
        "battery_soc": 80,
        "drive_fault": None,
    })))
    bridge = web.NxRobotBridge(node)
    assert bridge.connected is True
    assert bridge.navigation_ready is False

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STOPPED",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "battery_soc": 80,
        "drive_fault": None,
        "sport_mode": 6,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "drive_session": "parked",
        "drive_session_owner": None,
        "drive_session_phase": "parked",
    })))
    readiness = node.get_navigation_readiness()
    assert readiness["activatable"] is True
    assert readiness["ready"] is False
    assert readiness["reason"] == "drive_session_parked"

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STOPPED",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "battery_soc": 80,
        "drive_fault": None,
        "sport_mode": 1,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "drive_session": "active",
        "drive_session_owner": "nav",
        "drive_session_phase": "active",
    })))
    assert bridge.navigation_ready is True

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STOPPED",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "battery_soc": 6,
        "drive_fault": None,
        "sport_mode": 1,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "drive_session": "active",
        "drive_session_owner": "nav",
        "drive_session_phase": "active",
    })))
    readiness = node.get_navigation_readiness()
    assert readiness["ready"] is False
    assert readiness["reason"] == "battery_low"

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STANDING",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "battery_soc": 80,
        "drive_fault": None,
        "sport_mode": 6,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "drive_session": "activating",
        "drive_session_owner": "nav",
        "drive_session_phase": "activating",
    })))
    assert bridge.navigation_ready is False

    now["value"] += 3.01
    assert bridge.connected is False
    assert bridge.navigation_ready is False


def test_navigation_ready_is_false_when_localization_is_stale(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.RLock(),
        _dog_state="STOPPED",
        _motion_sdk_ready=True,
        _motion_nav_scan_fresh=True,
        _motion_battery_soc=80,
        _motion_drive_fault=None,
        _last_state_t=100.0,
        state_timeout=3.0,
        motion_min_battery_soc=20.0,
        get_localization_health=lambda: {
            "healthy": False, "reason": "stale", "age_sec": 1.25
        },
    )
    node.get_navigation_readiness = types.MethodType(
        web.NxWebNode.get_navigation_readiness, node
    )
    monkeypatch.setattr(web.time, "time", lambda: 100.1)

    readiness = node.get_navigation_readiness()

    assert readiness["ready"] is False
    assert readiness["reason"] == "localization_stale"
    assert readiness["localization_healthy"] is False
    assert readiness["localization_reason"] == "stale"
    assert readiness["localization_age_sec"] == pytest.approx(1.25)


def test_navigation_v4_fails_closed_when_web_and_motion_releases_differ(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.RLock(),
        _dog_state="STOPPED",
        _motion_sdk_ready=True,
        _motion_nav_scan_fresh=True,
        _motion_nav_guard_reason=None,
        _motion_battery_soc=80,
        _motion_drive_fault=None,
        _motion_sport_mode=1,
        _motion_wheel_dq=[0.0, 0.0, 0.0, 0.0],
        _motion_wheel_activation_phase="wheel_ready",
        _motion_drive_session="nav_active",
        _motion_drive_session_owner="nav",
        _motion_drive_session_phase="nav_active",
        _motion_drive_session_reason="feedback_confirmed",
        _motion_physical_mode="wheel_balance",
        _motion_actual_motion="stopped",
        _motion_velocity_authorized=True,
        _motion_schema_version=4,
        _motion_release_id="different-release",
        _last_state_t=100.0,
        state_timeout=3.0,
        motion_min_battery_soc=20.0,
        get_localization_health=lambda: {
            "healthy": True, "reason": "ok", "age_sec": 0.01
        },
    )
    node.get_navigation_readiness = types.MethodType(
        web.NxWebNode.get_navigation_readiness, node
    )
    monkeypatch.setattr(web.time, "time", lambda: 100.1)

    readiness = node.get_navigation_readiness()

    assert readiness["ready"] is False
    assert readiness["activatable"] is False
    assert readiness["release_consistent"] is False
    assert readiness["reason"] == "release_mismatch"


def test_navigation_ready_surfaces_a_latched_motion_guard(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.RLock(),
        _dog_state="STOPPED",
        _dog_vx=0.0,
        _dog_vy=0.0,
        _dog_vyaw=0.0,
        _motion_sdk_ready=True,
        _motion_nav_scan_fresh=True,
        _motion_nav_guard_reason=None,
        _motion_battery_soc=80.0,
        _motion_drive_fault=None,
        _motion_sport_mode=1,
        _motion_wheel_dq=[0.0, 0.0, 0.0, 0.0],
        _motion_wheel_activation_phase="wheel_ready",
        _motion_drive_session="active",
        _motion_drive_session_owner="nav",
        _motion_drive_session_phase="active",
        _motion_drive_session_reason="wheel_mode_feedback_confirmed",
        _last_state_t=100.0,
        state_timeout=3.0,
        motion_min_battery_soc=20.0,
        get_localization_health=lambda: {
            "healthy": True, "reason": "ok", "age_sec": 0.01
        },
        get_logger=lambda: _LoggerStub(),
    )
    node._on_dog_state = types.MethodType(web.NxWebNode._on_dog_state, node)
    node.get_navigation_readiness = types.MethodType(
        web.NxWebNode.get_navigation_readiness, node
    )
    monkeypatch.setattr(web.time, "time", lambda: 100.1)

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "STOPPED",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "nav_guard_reason": "pure_turn_oscillation",
        "battery_soc": 80,
        "drive_fault": None,
        "sport_mode": 1,
        "wheel_dq": [0.0, 0.0, 0.0, 0.0],
        "drive_session": "active",
        "drive_session_owner": "nav",
        "drive_session_phase": "active",
    })))
    readiness = node.get_navigation_readiness()

    assert readiness["ready"] is False
    assert readiness["activatable"] is False
    assert readiness["reason"] == "pure_turn_oscillation"
    assert readiness["nav_guard_reason"] == "pure_turn_oscillation"


def test_dog_state_caches_drive_recovery_diagnostics(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.RLock(),
        _dog_state="UNKNOWN",
        _dog_vx=0.0,
        _dog_vy=0.0,
        _dog_vyaw=0.0,
        _motion_sdk_ready=False,
        _motion_nav_scan_fresh=False,
        _motion_battery_soc=None,
        _motion_drive_fault=None,
        _motion_sport_mode=None,
        _motion_wheel_dq=None,
        _motion_wheel_activation_phase="unknown",
        _motion_drive_session="startup",
        _motion_drive_session_owner=None,
        _motion_drive_session_phase="startup",
        _motion_drive_session_reason="waiting_for_feedback",
        _last_state_t=0.0,
        get_logger=lambda: _LoggerStub(),
    )
    node._on_dog_state = types.MethodType(web.NxWebNode._on_dog_state, node)

    node._on_dog_state(types.SimpleNamespace(data=json.dumps({
        "state": "EMERGENCY",
        "sdk_ready": True,
        "nav_scan_fresh": True,
        "battery_soc": 78,
        "drive_fault": "wheel_no_response",
        "sport_mode": 6,
        "wheel_dq": [0.01, -0.02, 0.0, 0.03],
        "wheel_activation_phase": "failed",
        "drive_session": "parked",
        "drive_session_owner": None,
        "drive_session_phase": "parked",
        "drive_session_reason": "operator_stop",
    })))

    assert node._motion_sport_mode == 6
    assert node._motion_wheel_dq == pytest.approx([0.01, -0.02, 0.0, 0.03])
    assert node._motion_wheel_activation_phase == "failed"
    assert node._motion_drive_session == "parked"
    assert node._motion_drive_session_owner is None
    assert node._motion_drive_session_reason == "operator_stop"


def test_reset_drive_fault_http_drains_autonomy_and_publishes_command(
    monkeypatch, tmp_path
):
    web = _load_web_server(monkeypatch)
    calls = []

    class Robot:
        def reset_drive_fault(self):
            calls.append("reset")

    class Arbiter:
        def run_operator_action(self, reason, action):
            calls.append(reason)
            action()
            return {"ok": True}

    web.robot = Robot()
    web.navigation_arbiter = Arbiter()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request("POST", "/api/reset_drive_fault")
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 200
    assert payload["ok"] is True
    assert calls == ["reset_drive_fault", "reset"]


def test_manual_move_http_activates_one_session_and_stop_parks_it(monkeypatch, tmp_path):
    web = _load_web_server(monkeypatch)
    calls = []

    class Robot:
        def move(self, vx, vy, vyaw, manual=False):
            calls.append(("move", vx, vy, vyaw, manual))

    class Arbiter:
        def run_manual_action(self, reason, action):
            calls.append(("manual_session", reason))
            action()
            return {"ok": True, "phase": "active"}

        def stop_all(self, reason):
            calls.append(("stop_all", reason))
            return {"ok": True, "phase": "parking"}

    web.robot = Robot()
    web.navigation_arbiter = Arbiter()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request("POST", "/api/move?vx=0.2&vy=0&vyaw=0")
        move_response = connection.getresponse()
        move_payload = json.loads(move_response.read().decode("utf-8"))
        connection.close()

        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request("POST", "/api/stop")
        stop_response = connection.getresponse()
        stop_payload = json.loads(stop_response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert move_response.status == 200 and move_payload["phase"] == "active"
    assert stop_response.status == 200 and stop_payload["phase"] == "parking"
    assert calls == [
        ("manual_session", "manual_move"),
        ("move", 0.2, 0.0, 0.0, True),
        ("stop_all", "operator_stop"),
    ]


def test_manual_release_http_does_not_use_global_operator_stop(monkeypatch, tmp_path):
    web = _load_web_server(monkeypatch)
    calls = []

    class Arbiter:
        def release_manual(self, reason):
            calls.append(("manual_release", reason))
            return {"ok": True, "phase": "unchanged", "ignored": True}

    web.navigation_arbiter = Arbiter()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request("POST", "/api/manual_stop")
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 200
    assert payload["ok"] is True
    assert calls == [("manual_release", "manual_release")]


def test_global_stop_cancels_any_autonomous_owner(monkeypatch, tmp_path):
    web = _load_web_server(monkeypatch)
    calls = []

    class Arbiter:
        def stop_all(self, reason):
            calls.append(("stop_all", reason))
            return {
                "ok": True,
                "phase": "parking",
                "owner": "operator",
            }

    web.navigation_arbiter = Arbiter()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request(
            "POST",
            "/api/stop",
            headers={"Referer": "http://127.0.0.1:8000/"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 200
    assert payload["phase"] == "parking"
    assert calls == [("stop_all", "operator_stop")]


def test_point_navigation_health_combines_localization_and_motion_readiness(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        get_localization_health=lambda: {
            "healthy": True, "reason": "ok", "x": 0.0, "y": 0.0},
        get_navigation_readiness=lambda: {
            "ready": False, "reason": "sdk_not_ready"},
    )
    assert web._point_navigation_health(node) is False
    assert web._point_navigation_health_sample(node) == {
        "healthy": False,
        "immediate": True,
        "reason": "motion_unhealthy",
        "motion_reason": "sdk_not_ready",
        "localization_reason": "ok",
    }
    node.get_navigation_readiness = lambda: {"ready": True}
    assert web._point_navigation_health(node) is True
    assert web._point_navigation_health_sample(node)["healthy"] is True
    assert "health_check=lambda: _point_navigation_health_sample(node)" in read(
        "web/nx_web_server.py"
    )


def test_parked_activatable_drive_uses_recovery_grace_not_immediate_cancel(
        monkeypatch):
    web = _load_web_server(monkeypatch)
    node = types.SimpleNamespace(
        get_localization_health=lambda: {
            "healthy": True, "reason": "ok"},
        get_navigation_readiness=lambda: {
            "ready": False,
            "activatable": True,
            "reason": "drive_session_parked",
            "drive_session": "parked",
        },
    )

    sample = web._point_navigation_health_sample(node)

    assert sample == {
        "healthy": False,
        "immediate": False,
        "reason": "motion_unhealthy",
        "motion_reason": "drive_session_parked",
        "localization_reason": "ok",
    }


def test_motion_readiness_loss_cancels_an_active_point_goal(monkeypatch):
    web = _load_web_server(monkeypatch)
    from test_point_navigation import accept, make_controller

    health = {"localization": True, "motion": True}
    node = types.SimpleNamespace(
        get_localization_health=lambda: {"healthy": health["localization"]},
        get_navigation_readiness=lambda: {"ready": health["motion"]},
    )
    controller, client, _ = make_controller(
        health_check=lambda: web._point_navigation_health_sample(node)
    )
    controller.submit(1.0, 0.0, 0.0)
    handle = accept(client)

    health["motion"] = False
    controller.tick()

    assert handle.cancel_calls == 1
    state = controller.get_state()
    assert state["status"] == "canceling"
    assert state["reason"] == "motion_unhealthy"


def test_transient_localization_loss_does_not_mask_motion_safety(monkeypatch):
    web = _load_web_server(monkeypatch)
    from test_point_navigation import accept, make_controller

    health = {"localization": True, "motion": True}
    node = types.SimpleNamespace(
        get_localization_health=lambda: {
            "healthy": health["localization"], "reason": "tf_stale"},
        get_navigation_readiness=lambda: {
            "ready": health["motion"], "reason": "ok"},
    )
    controller, client, monotonic = make_controller(
        health_check=lambda: web._point_navigation_health_sample(node),
        health_failure_grace=0.5,
    )
    controller.submit(1.0, 0.0, 0.0)
    handle = accept(client)

    health["localization"] = False
    controller.tick()
    monotonic.advance(0.3)
    health["localization"] = True
    controller.tick()

    assert handle.cancel_calls == 0
    state = controller.get_state()
    assert state["status"] == "active"
    assert state["healthy"] is True
    assert state["health_degraded"] is False


def test_navigate_http_rejects_fresh_dog_state_when_sdk_is_not_ready(
    monkeypatch, tmp_path
):
    web = _load_web_server(monkeypatch)
    calls = []

    web.node = types.SimpleNamespace(
        get_localization_health=lambda: {
            "healthy": True, "x": 0.0, "y": 0.0
        },
        get_navigation_readiness=lambda: {
            "ready": False,
            "reason": "sdk_not_ready",
            "sdk_ready": False,
            "nav_scan_fresh": True,
        },
    )
    web.point_nav = object()
    web.robot = types.SimpleNamespace(connected=True, navigation_ready=False)
    web.navigation_arbiter = types.SimpleNamespace(
        start_point_goal=lambda *args: calls.append(args) or {"ok": True}
    )
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request(
            "POST",
            "/api/navigate",
            body=json.dumps({"x": 1.0, "y": 2.0, "yaw": 0.0}),
            headers={"Content-Type": "application/json"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 503
    assert payload["navigation"]["reason"] == "sdk_not_ready"
    assert calls == []


def test_navigate_http_admits_feedback_confirmed_parked_dog_for_activation(
    monkeypatch, tmp_path
):
    web = _load_web_server(monkeypatch)
    calls = []
    web.node = types.SimpleNamespace(
        get_localization_health=lambda: {
            "healthy": True, "x": 0.0, "y": 0.0
        },
        get_navigation_readiness=lambda: {
            "ready": False,
            "activatable": True,
            "reason": "drive_session_parked",
            "drive_session": "parked",
            "sport_mode": 6,
        },
    )
    web.point_nav = object()
    web.robot = types.SimpleNamespace(connected=True, navigation_ready=False)
    web.navigation_arbiter = types.SimpleNamespace(
        start_point_goal=lambda *args: calls.append(args) or {
            "ok": True, "generation": 7
        }
    )
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request(
            "POST", "/api/navigate",
            body=json.dumps({"x": 1.0, "y": 0.0, "yaw": 0.0}),
            headers={"Content-Type": "application/json"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 202
    assert payload == {"ok": True, "generation": 7}
    assert calls == [(1.0, 0.0, 0.0)]


def test_point_goal_distance_guard_rejects_remote_or_invalid_targets(monkeypatch):
    web = _load_web_server(monkeypatch)
    localization = {"healthy": True, "x": 4.0, "y": -1.0}

    assert web._point_goal_within_local_radius(5.0, -1.0, localization, 20.0)
    assert not web._point_goal_within_local_radius(
        -61034.96, -9724.70, localization, 20.0
    )
    assert not web._point_goal_within_local_radius(
        float("nan"), 0.0, localization, 20.0
    )


def test_point_goal_default_radius_covers_twenty_metre_goal_tolerance(monkeypatch):
    web = _load_web_server(monkeypatch)
    localization = {"healthy": True, "x": -0.2, "y": 0.0}

    assert web.POINT_GOAL_MAX_DISTANCE >= 20.2
    assert web._point_goal_within_local_radius(20.0, 0.0, localization)


def test_task_manager_cancel_keeps_worker_owner_until_room_action_drains(monkeypatch):
    web = _load_web_server(monkeypatch)

    class Robot:
        def stop_move(self):
            pass

    class Room:
        def __init__(self):
            self.drained = False

        def cancel(self):
            pass

        def wait_drained(self, timeout):
            return self.drained

    room = Room()
    manager = web.TaskManager(Robot(), room_orchestrator=room)
    active = web.Task("search_room")
    active.status = "active"
    manager._active = active
    manager.cancel_all()

    assert manager._active is active
    assert active.status == "cancelled"
    assert manager.wait_drained(0.01) is False

    with manager._lock:
        manager._active = None
    assert manager.wait_drained(0.01) is False
    room.drained = True
    assert manager.wait_drained(0.05) is True


def test_task_admission_delegates_to_shared_navigation_arbiter(monkeypatch):
    web = _load_web_server(monkeypatch)
    calls = []

    class Robot:
        def stop_move(self):
            pass

    class Arbiter:
        def start_tasks(self, tasks, *, reason):
            calls.append((list(tasks), reason))
            return {"ok": True}

    manager = web.TaskManager(Robot())
    manager.set_navigation_arbiter(Arbiter())
    task = web.Task("search_room")
    result = manager.add_list([task], reason="search_room")

    assert result == {"ok": True}
    assert calls == [([task], "search_room")]
    assert manager._tasks == []


def test_product_voice_command_returns_confirmed_task_admission(monkeypatch):
    web = _load_web_server(monkeypatch)
    calls = []

    class Robot:
        def stop_move(self):
            pass

    class Arbiter:
        def start_tasks(self, tasks, *, reason):
            calls.append((list(tasks), reason))
            return {"ok": True, "phase": "active", "owner": "tasks"}

    manager = web.TaskManager(Robot())
    manager.set_navigation_arbiter(Arbiter())

    result = manager.submit_command(
        "去 搜索 这个 房间，把 所有 人 标注 出来"
    )

    assert result["ok"] is True
    assert result["accepted"] is True
    assert result["parser"] == "product"
    assert result["admission"] == {
        "ok": True, "phase": "active", "owner": "tasks"
    }
    assert len(result["tasks"]) == 1
    assert result["tasks"][0]["type"] == "search_room"
    assert result["tasks"][0]["params"]["room"] == "__current__"
    assert result["tasks"][0]["params"]["search_strategy"] == "frontier_explore"
    assert len(calls) == 1
    admitted_tasks, reason = calls[0]
    assert reason == "task_command"
    assert admitted_tasks[0].type == "search_room"


def test_product_voice_command_surfaces_navigation_admission_failure(monkeypatch):
    web = _load_web_server(monkeypatch)

    class Robot:
        def stop_move(self):
            pass

    class Arbiter:
        def start_tasks(self, tasks, *, reason):
            return {
                "ok": False,
                "reason": "stand_confirmation_required",
                "phase": "parked",
            }

    manager = web.TaskManager(Robot())
    manager.set_navigation_arbiter(Arbiter())

    result = manager.submit_command("去搜索这个房间，把所有人标注出来")

    assert result["ok"] is False
    assert result["accepted"] is False
    assert result["reason"] == "stand_confirmation_required"
    assert result["admission"]["phase"] == "parked"


def test_command_http_returns_the_real_product_admission_result(
    monkeypatch, tmp_path
):
    web = _load_web_server(monkeypatch)
    calls = []

    class TaskManager:
        def submit_command(self, text):
            calls.append(text)
            return {
                "ok": False,
                "accepted": False,
                "reason": "stand_confirmation_required",
            }

    web.task_mgr = TaskManager()
    server = web.create_server("127.0.0.1", 0, str(tmp_path))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(*server.server_address, timeout=1.0)
        connection.request(
            "POST",
            "/api/command",
            body=json.dumps({"text": "去搜索这个房间，把所有人标注出来"}),
            headers={"Content-Type": "application/json"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 409
    assert payload["ok"] is False
    assert payload["accepted"] is False
    assert payload["reason"] == "stand_confirmation_required"
    assert calls == ["去搜索这个房间，把所有人标注出来"]


def test_web_uses_validated_map_frame_localization_pose():
    server = read("web/nx_web_server.py")

    assert "from nx_point_nav import PointNavigationController" in server
    assert "'/localization_pose', self._on_localization_pose, 10" in server
    assert "'/odom', self._on_lio_odometry, 10" in server
    assert "def _on_localization_pose" in server
    assert "def _on_lio_odometry" in server
    assert "def get_odometry_snapshot" in server
    assert '"odometry": self.get_odometry_snapshot()' in server
    assert "header.frame_id" in server
    assert "child_frame_id" in server
    assert "_map_yaw" in server
    assert "def get_localization_health" in server
    assert "localization_timeout" in server


def test_panel_pose_header_uses_fresh_status_instead_of_backlogged_slam_frames():
    panel = read("web/static/panel.html")
    slam_branch = panel.split("data.type === 'slam'", 1)[1].split(
        "data.type === 'nav_goal'", 1
    )[0]

    assert "function updatePoseInfo(localization, odometry" in panel
    assert "updatePoseInfo(data.localization, data.odometry)" in panel
    assert "updatePoseInfo(d.localization, d.odometry)" in panel
    assert "poseInfo" not in slam_branch


def test_map_consumers_use_localization_yaw_not_raw_imu_yaw():
    server = read("web/nx_web_server.py")
    orchestrator = read("web/nx_room_orchestrator.py")
    broadcast = server.split("def broadcast_loop", 1)[1].split("# Main", 1)[0]

    assert "yaw = nx_node._map_yaw" in broadcast
    assert "yaw = nx_node._imu_yaw" not in broadcast
    assert 'getattr(self._node, "_imu_yaw"' not in orchestrator
    assert 'getattr(self._node, "_map_yaw"' in orchestrator


def test_web_exposes_point_navigation_and_arbitrates_motion_routes():
    server = read("web/nx_web_server.py")
    arbiter = read("web/nx_navigation_arbiter.py")

    assert "p.path == '/api/navigate'" in server
    assert "navigation_arbiter.start_point_goal(" in server
    assert "point_nav.get_state()" in server
    assert "navigation_gateway.tick" in server
    for reason in (
        "operator_stop",
        "manual_move",
        "sit",
        "stand",
    ):
        assert f'"{reason}"' in server
    assert "navigation_arbiter.emergency_stop()" in server
    assert "self._point_nav.cancel(reason)" in arbiter
    assert "self._tasks.cancel_all()" in arbiter
    assert "self._wait_point_drained" in arbiter
    assert "self._wait_tasks_drained" in arbiter
    assert "state_callback=_handle_point_nav_state" in server
    assert "navigation_arbiter.on_point_state(state)" in server
    assert "self._navigation_arbiter.on_tasks_drained()" in server


def test_status_api_exposes_perception_health_without_copying_frames():
    server = read("web/nx_web_server.py")
    ai = read("web/nx_ai_node.py")

    assert '"perception": _perception_health(robot)' in server
    assert "get_person_detection_health" in server
    assert "def get_person_detection_health" in ai
    assert 'health["map_annotation"]' in server
    assert "resolve_camera_calibration" in server


def test_websocket_status_rehydrates_persisted_room_markers():
    server = read("web/nx_web_server.py")
    panel = read("web/static/panel.html")

    broadcast = server.split("def broadcast_loop", 1)[1].split(
        "# Main", 1)[0]
    assert '"room_nav": room_orchestrator.get_navigation_state()' in broadcast
    assert "data.room_nav" in panel
    assert "target_markers" in panel


def test_http_serves_mission_artifacts_from_persistent_root(
    monkeypatch, tmp_path
):
    web = _load_web_server(monkeypatch)
    static_dir = tmp_path / "release" / "static"
    static_dir.mkdir(parents=True)
    mission_root = tmp_path / "persistent" / "missions"
    artifact = mission_root / "mission-1" / "person_001.json"
    artifact.parent.mkdir(parents=True)
    artifact.write_text('{"id":"person_001"}', encoding="utf-8")

    server = web.create_server(
        "127.0.0.1", 0, str(static_dir), mission_root=str(mission_root))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        connection = http.client.HTTPConnection(
            *server.server_address, timeout=1.0)
        connection.request(
            "GET", "/missions/mission-1/person_001.json")
        response = connection.getresponse()
        payload = json.loads(response.read().decode("utf-8"))
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1.0)

    assert response.status == 200
    assert payload == {"id": "person_001"}


def test_high_impact_control_requests_are_attributed_in_the_web_log():
    server = read("web/nx_web_server.py")
    arbiter = read("web/nx_navigation_arbiter.py")

    for path in (
        "/api/stand", "/api/balance", "/api/sit", "/api/e_stop",
        "/api/stop", "/api/manual_stop", "/api/navigate",
    ):
        assert path in server
    assert "control request path=%s client=%s" in server
    assert 'self.headers.get("User-Agent")' in server
    assert 'self.headers.get("Referer")' in server
    assert "def get_motion_owner(self)" in arbiter
    assert 'p.path == "/api/move"' in server
    assert 'get_motion_owner() != "manual"' in server


def test_point_controller_is_created_before_ros_spin_and_stopped_before_shutdown():
    server = read("web/nx_web_server.py")
    arbiter = read("web/nx_navigation_arbiter.py")

    controller_create = server.index("PointNavigationController(")
    room_client_create = server.index("room_orchestrator = RoomSearchOrchestrator(")
    arbiter_create = server.index("navigation_arbiter = NavigationArbiter(")
    spin_start = server.index("target=_spin_loop_yielding")
    arbiter_shutdown = server.rindex("navigation_arbiter.shutdown()")
    ros_shutdown = server.rindex("rclpy.shutdown()")
    assert controller_create < room_client_create < arbiter_create < spin_start
    assert arbiter_shutdown < ros_shutdown
    assert "self._point_nav.stop()" in arbiter
    assert "tasks_drained" in arbiter
    assert "self._emergency_stop()" in arbiter


def test_web_deployer_copies_point_navigation_module():
    deploy = read("docker/deploy_nx_web.sh")
    assert "web/nx_point_nav.py" in deploy
    assert "web/nx_navigation_arbiter.py" in deploy


def test_panel_posts_map_goal_and_restores_monotonic_ws_state():
    panel = read("web/static/panel.html")

    assert "onSelectGoal" in panel
    assert "function sendNavGoal" in panel
    assert "controlFetch('/api/navigate'" in panel
    assert "frame_id: 'map'" in panel
    assert "Math.atan2" in panel
    assert "data.type === 'nav_goal'" in panel
    assert "map.setNavGoal" in panel
    assert "navRequestSerial" in panel
    assert "lastNavGeneration" in panel
    assert "lastNavUpdatedMonotonic" in panel
    assert "navConnectionEpoch" in panel
    assert "function applyNavState" in panel
    assert "function applyNavStateForEpoch" in panel
    assert "function resetNavOrderingForNewConnection" in panel
    assert "socket.onopen" in panel
    assert "state.generation" in panel
    assert "d.point_nav" in panel
    assert "server_unavailable" in panel
    assert 'id="navStatus"' in panel


def test_panel_requires_explicit_visual_stand_confirmation_before_navigation():
    panel = read("web/static/panel.html")
    server = read("web/nx_web_server.py")

    assert 'id="confirmStandBtn"' in panel
    assert "function confirmStand()" in panel
    assert "controlFetch('/api/confirm_stand'" in panel
    assert "state !== 'STAND_UNCONFIRMED'" in panel
    assert "p.path == '/api/confirm_stand'" in server
    assert "def confirm_stand(self):" in server
    assert "publish_cmd_pose('confirm_stand')" in server
    assert 'id="balanceBtn"' in panel
    assert "function enterBalance()" in panel
    assert "controlFetch('/api/balance'" in panel
    assert "p.path == '/api/balance'" in server
    assert "def balance(self):" in server
    assert "state !== 'STOOD'" in panel
    assert 'id="confirmBalanceBtn"' in panel
    assert "function confirmBalance()" in panel
    assert "controlFetch('/api/confirm_balance'" in panel
    assert "p.path == '/api/confirm_balance'" in server
    assert "def confirm_balance(self):" in server


def test_web_roots_all_long_lived_ros_subscriptions():
    server = read("web/nx_web_server.py")

    assert "self._dog_state_sub = self.create_subscription(" in server
    assert "self._imu_sub = self.create_subscription(" in server
    assert "self._scan_sub = self.create_subscription(" in server
    assert "self._localization_sub = self.create_subscription(" in server


def test_panel_keyboard_control_uses_one_continuous_motion_path():
    panel = read("web/static/panel.html")
    keyboard_motion = panel.split("function sendMoveFromKeys()", 1)[1].split(
        "// 按钮拖拽控制", 1
    )[0]

    assert "else move(vx, vy, vyaw);" in keyboard_motion
    assert "fetch(`/api/move" not in keyboard_motion
    assert panel.count("document.addEventListener('keydown'") == 1
    assert panel.count("document.addEventListener('keyup'") == 1
    assert (
        "window.addEventListener('blur', () => { pressedKeys.clear(); "
        "resetKeyHighlight(); stopMove(); });"
    ) in panel


def test_panel_manual_release_and_startup_cannot_cancel_autonomy():
    panel = read("web/static/panel.html")
    stop_move = panel.split("function stopMove()", 1)[1].split(
        "function connect()", 1
    )[0]
    startup = panel.split("// ---- ", 1)[-1]

    assert "controlFetch('/api/manual_stop'" in stop_move
    assert "controlFetch('/api/stop'" not in stop_move
    assert "controlFetch('/api/stop'" not in startup


def test_panel_has_bare_move_buttons():
    """无参数运动按钮 (前进/后退/左转/右转) 各至少出现一次 (spec §3.2)."""
    panel = read("web/static/panel.html")
    for cmd in ("quickCmd('前进')", "quickCmd('后退')",
                "quickCmd('左转')", "quickCmd('右转')"):
        assert cmd in panel, f"缺无参数按钮: {cmd}"


def test_panel_keeps_parameterized_move_buttons():
    """原有带参数运动按钮不丢失."""
    panel = read("web/static/panel.html")
    assert "quickCmd('前进两米')" in panel
    assert "quickCmd('左转90度')" in panel
