import importlib
import sys
import threading
import time
import types
from pathlib import Path

import pytest

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_product_command import (
    build_search_mission,
    parse_product_command,
    resolve_current_room,
)


def _expected_task(room):
    current = room == "__current__"
    params = {
        "mission_request": build_search_mission(room, "person").to_dict(),
        "room": room,
        "target_classes": ["person"],
        "require_photos": True,
        "mark_on_map": True,
        "search_strategy": "frontier_explore" if current else "next_best_view",
        "use_lidar_person_range": True,
    }
    if current:
        params.update({
            "max_radius_m": 30.0,
            "max_time": 1800.0,
            "initial_radius_m": 6.0,
            "radius_step_m": 6.0,
            "tile_size_m": 6.0,
            "stable_exhaustion_cycles": 3,
            "max_frontiers": 200,
            "max_plan_probes_per_cycle": 12,
        })
    return {
        "type": "search_room",
        "priority": 8,
        "params": params,
    }


def test_parse_current_room_all_people_command():
    result = parse_product_command("去搜索这个房间，把所有人标注出来")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("__current__")],
    }


def test_parse_current_room_all_tables_command():
    result = parse_product_command("去搜索这个房间，把所有桌子标记出来")

    assert result == {
        "response": "搜索当前房间并标注所有桌子",
        "tasks": [{
            "type": "search_room",
            "priority": 8,
            "params": {
                "mission_request": build_search_mission(
                    "__current__", "dining table").to_dict(),
                "room": "__current__",
                "target_classes": ["dining table"],
                "require_photos": True,
                "mark_on_map": True,
                "search_strategy": "frontier_explore",
                "use_lidar_target_range": True,
                "max_radius_m": 30.0,
                "max_time": 1800.0,
                "initial_radius_m": 6.0,
                "radius_step_m": 6.0,
                "tile_size_m": 6.0,
                "stable_exhaustion_cycles": 3,
                "max_frontiers": 200,
                "max_plan_probes_per_cycle": 12,
            },
        }],
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
        "response": "搜索客厅并标注所有人",
        "tasks": [_expected_task("客厅")],
    }


def test_parse_named_room_tables_command():
    result = parse_product_command("去会议室搜索餐桌并标注出来")

    assert result["response"] == "搜索会议室并标注所有桌子"
    assert result["tasks"][0]["params"] == {
        "mission_request": build_search_mission(
            "会议室", "dining table").to_dict(),
        "room": "会议室",
        "target_classes": ["dining table"],
        "require_photos": True,
        "mark_on_map": True,
        "search_strategy": "next_best_view",
        "use_lidar_target_range": True,
    }


def test_parse_go_named_room_find_people_command():
    result = parse_product_command("去卧室找人")

    assert result == {
        "response": "搜索卧室并标注所有人",
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
        _odom_t=0.0,
    )
    manager = nx_web_server.TaskManager.__new__(nx_web_server.TaskManager)
    manager.robot = types.SimpleNamespace(_node=node)

    assert manager._latest_robot_map_pose() is None

    node._odom_count = 1
    node._odom_t = time.time()
    assert manager._latest_robot_map_pose() == (0.0, 0.0)


def test_product_current_room_parser_keeps_current_when_odom_is_stale(monkeypatch):
    nx_web_server = _import_nx_web_server_with_ros_stubs(monkeypatch)
    node = types.SimpleNamespace(
        _lock=threading.Lock(),
        _odom_x=2.0,
        _odom_y=1.0,
        _odom_count=1,
        _odom_t=time.time() - 10.0,
    )
    manager = nx_web_server.TaskManager.__new__(nx_web_server.TaskManager)
    manager.robot = types.SimpleNamespace(_node=node)
    manager.room_orchestrator = types.SimpleNamespace(list_rooms_detail=lambda: [
        {
            "name": "room-a",
            "search_area": {
                "origin_x": 0.0,
                "origin_y": 0.0,
                "width": 4.0,
                "height": 3.0,
            },
        },
    ])
    result = {"tasks": [_expected_task("__current__")]}

    manager._resolve_product_current_room(result)

    assert result["tasks"][0]["params"]["room"] == "__current__"


def test_frontier_current_room_is_never_rewritten_to_uncalibrated_placeholder(
        monkeypatch):
    nx_web_server = _import_nx_web_server_with_ros_stubs(monkeypatch)
    node = types.SimpleNamespace(get_localization_health=lambda: {
        "healthy": True, "x": 0.1, "y": 0.1,
    })
    manager = nx_web_server.TaskManager.__new__(nx_web_server.TaskManager)
    manager.robot = types.SimpleNamespace(_node=node)
    manager.room_orchestrator = types.SimpleNamespace(list_rooms_detail=lambda: [{
        "name": "占位客厅",
        "calibrated": False,
        "nav_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "search_area": {
            "origin_x": 0.0, "origin_y": 0.0, "width": 2.0, "height": 2.0,
        },
    }])
    result = parse_product_command("去搜索这个房间，把所有人标注出来")

    manager._resolve_product_current_room(result)

    assert result["tasks"][0]["params"]["room"] == "__current__"
    assert result["tasks"][0]["params"]["search_strategy"] == "frontier_explore"


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
