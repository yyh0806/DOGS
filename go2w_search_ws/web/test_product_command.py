import importlib
import sys
import threading
import types
from pathlib import Path

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_product_command import parse_product_command, resolve_current_room


def _expected_task(room):
    return {
        "type": "search_room",
        "priority": 8,
        "params": {
            "room": room,
            "target_classes": ["person"],
            "require_photos": True,
            "mark_on_map": True,
            "search_strategy": "next_best_view",
            "use_lidar_person_range": True,
        },
    }


def test_parse_current_room_all_people_command():
    result = parse_product_command("去搜索这个房间，把所有人标注出来")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("__current__")],
    }


def test_parse_current_room_all_people_short_command():
    result = parse_product_command("搜索这个房间所有人")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("__current__")],
    }


def test_parse_named_room_people_command():
    result = parse_product_command("搜索客厅的人")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("客厅")],
    }


def test_parse_go_named_room_find_people_command():
    result = parse_product_command("去卧室找人")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("卧室")],
    }


def test_non_product_command_returns_none():
    assert parse_product_command("前进两米") is None


def test_non_room_people_command_returns_none():
    assert parse_product_command("找前面的人") is None


@pytest.mark.parametrize("command", [
    "找客厅的机器人",
    "去客厅找无人机",
    "客厅有人，找一下钥匙",
    "别找客厅的人",
])
def test_non_product_room_commands_return_none(command):
    assert parse_product_command(command) is None


def test_resolve_current_room_by_containing_area():
    rooms = [
        {
            "name": "客厅",
            "nav_pose": {"x": 5.0, "y": 5.0, "yaw": 0.0},
            "search_area": {"origin_x": 0.0, "origin_y": 0.0, "width": 4.0, "height": 3.0},
        },
        {
            "name": "卧室",
            "nav_pose": {"x": 10.0, "y": 10.0, "yaw": 0.0},
            "search_area": {"origin_x": 8.0, "origin_y": 8.0, "width": 3.0, "height": 3.0},
        },
    ]

    assert resolve_current_room(2.0, 1.0, rooms) == "客厅"


def test_resolve_current_room_by_nearest_nav_pose_when_outside_area():
    rooms = [
        {
            "name": "客厅",
            "nav_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "search_area": {"origin_x": 20.0, "origin_y": 20.0, "width": 2.0, "height": 2.0},
        },
        {
            "name": "卧室",
            "nav_pose": {"x": 5.0, "y": 1.0, "yaw": 0.0},
            "search_area": {"origin_x": 30.0, "origin_y": 30.0, "width": 2.0, "height": 2.0},
        },
    ]

    assert resolve_current_room(4.0, 1.0, rooms) == "卧室"


def test_resolve_current_room_returns_none_for_empty_rooms():
    assert resolve_current_room(0.0, 0.0, []) is None


def test_resolve_current_room_returns_none_when_no_room_has_usable_geometry():
    rooms = [
        {"name": "客厅", "search_area": {"origin_x": 0.0, "origin_y": 0.0}},
        {"name": "卧室", "nav_pose": {"yaw": 0.0}},
        {"name": "厨房"},
    ]

    assert resolve_current_room(0.0, 0.0, rooms) is None


def test_resolve_current_room_returns_none_for_non_finite_robot_pose():
    rooms = [
        {
            "name": "客厅",
            "nav_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "search_area": {"origin_x": -1.0, "origin_y": -1.0, "width": 2.0, "height": 2.0},
        },
    ]

    assert resolve_current_room(float("nan"), 0.0, rooms) is None


def test_resolve_current_room_ignores_non_finite_nav_pose():
    rooms = [
        {
            "name": "客厅",
            "nav_pose": {"x": float("inf"), "y": 0.0, "yaw": 0.0},
        },
        {
            "name": "卧室",
            "nav_pose": {"x": 5.0, "y": 0.0, "yaw": 0.0},
        },
    ]

    assert resolve_current_room(4.0, 0.0, rooms) == "卧室"


def test_resolve_current_room_returns_none_when_nav_pose_is_non_finite():
    rooms = [
        {
            "name": "客厅",
            "nav_pose": {"x": float("inf"), "y": 0.0, "yaw": 0.0},
        },
    ]

    assert resolve_current_room(4.0, 0.0, rooms) is None


def test_resolve_current_room_ignores_non_finite_search_area():
    rooms = [
        {
            "name": "客厅",
            "search_area": {
                "origin_x": float("nan"),
                "origin_y": -1.0,
                "width": 10.0,
                "height": 10.0,
            },
        },
        {
            "name": "卧室",
            "search_area": {"origin_x": 3.0, "origin_y": -1.0, "width": 3.0, "height": 3.0},
        },
    ]

    assert resolve_current_room(4.0, 0.0, rooms) == "卧室"


def test_latest_robot_map_pose_requires_received_odom(monkeypatch):
    nx_web_server = _import_nx_web_server_with_ros_stubs(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.Lock(),
        _odom_x=0.0,
        _odom_y=0.0,
        _odom_count=0,
    )
    manager = nx_web_server.TaskManager.__new__(nx_web_server.TaskManager)
    manager.robot = types.SimpleNamespace(_node=node)

    assert manager._latest_robot_map_pose() is None

    node._odom_count = 1
    assert manager._latest_robot_map_pose() == (0.0, 0.0)


def _import_nx_web_server_with_ros_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = type("Node", (), {})
    monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.qos_profile_sensor_data = object()
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_mod)

    for pkg_name, class_names in {
        "geometry_msgs.msg": ("Twist",),
        "nav_msgs.msg": ("Odometry",),
        "sensor_msgs.msg": ("Imu", "LaserScan"),
        "std_msgs.msg": ("String",),
    }.items():
        parent_name = pkg_name.split(".")[0]
        monkeypatch.setitem(sys.modules, parent_name, types.ModuleType(parent_name))
        msg_mod = types.ModuleType(pkg_name)
        for class_name in class_names:
            setattr(msg_mod, class_name, type(class_name, (), {}))
        monkeypatch.setitem(sys.modules, pkg_name, msg_mod)

    monkeypatch.delitem(sys.modules, "nx_web_server", raising=False)
    return importlib.import_module("nx_web_server")
