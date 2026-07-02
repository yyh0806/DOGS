import sys
from pathlib import Path

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


def test_parse_named_room_people_command():
    result = parse_product_command("搜索客厅的人")

    assert result == {
        "response": "搜索当前房间并标注所有人",
        "tasks": [_expected_task("客厅")],
    }


def test_non_product_command_returns_none():
    assert parse_product_command("前进两米") is None


def test_non_room_people_command_returns_none():
    assert parse_product_command("找前面的人") is None


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
