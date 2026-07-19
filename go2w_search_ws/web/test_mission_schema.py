import math
from pathlib import Path

import pytest

from nx_mission_schema import (
    MissionValidationError,
    SearchMissionRequest,
    canonicalize_search_tasks,
)


def test_person_and_table_commands_share_one_schema():
    person = SearchMissionRequest.current_room(["person"], request_id="person-1")
    table = SearchMissionRequest.current_room(["桌子"], request_id="table-1")
    assert person.to_dict()["target_classes"] == ["person"]
    assert table.to_dict()["target_classes"] == ["dining table"]
    assert person.search_strategy == table.search_strategy == "frontier_explore"


def test_current_room_defaults_support_adaptive_large_room_search():
    request = SearchMissionRequest.current_room(["person"], request_id="large-room")

    assert request.max_radius_m == 30.0
    assert request.max_time_s == 1800.0
    assert request.initial_radius_m == 6.0
    assert request.radius_step_m == 6.0
    assert request.tile_size_m == 6.0
    assert request.stable_exhaustion_cycles == 3
    assert request.max_frontiers == 200
    assert request.max_plan_probes_per_cycle == 12


def test_schema_round_trip_is_stable_and_generates_orchestrator_params():
    request = SearchMissionRequest.from_dict({
        "schema_version": 1,
        "request_id": "request-1",
        "room": "current_room",
        "target_classes": ["person", "dining table", "person"],
        "search_strategy": "frontier_explore",
        "require_photos": True,
        "mark_on_map": True,
        "max_radius_m": 6.0,
        "max_time_s": 180.0,
        "initial_radius_m": 3.0,
        "radius_step_m": 2.0,
        "tile_size_m": 4.0,
        "stable_exhaustion_cycles": 4,
        "max_frontiers": 120,
        "max_plan_probes_per_cycle": 9,
    })
    assert request.target_classes == ("person", "dining table")
    assert SearchMissionRequest.from_dict(request.to_dict()) == request
    assert request.to_task_params() == {
        "mission_request": request.to_dict(),
        "room": "__current__",
        "target_classes": ["person", "dining table"],
        "search_strategy": "frontier_explore",
        "require_photos": True,
        "mark_on_map": True,
        "max_radius_m": 6.0,
        "max_time": 180.0,
        "initial_radius_m": 3.0,
        "radius_step_m": 2.0,
        "tile_size_m": 4.0,
        "stable_exhaustion_cycles": 4,
        "max_frontiers": 120,
        "max_plan_probes_per_cycle": 9,
        "use_lidar_target_range": True,
    }


def test_legacy_http_table_payload_is_migrated_to_the_same_schema():
    request = SearchMissionRequest.from_api_payload({
        "room": "__current__",
        "target_classes": ["桌子"],
    }, request_id="http-1")
    assert request == SearchMissionRequest.current_room(
        ["dining table"], request_id="http-1")


@pytest.mark.parametrize(
    "patch",
    [
        {"target_classes": []},
        {"target_classes": ["../bad"]},
        {"max_radius_m": math.nan},
        {"max_time_s": 0},
        {"initial_radius_m": 31.0},
        {"radius_step_m": 0},
        {"tile_size_m": 0},
        {"stable_exhaustion_cycles": 0},
        {"max_frontiers": 0},
        {"max_plan_probes_per_cycle": 0},
        {"search_strategy": "drive_forward_blindly"},
    ],
)
def test_invalid_missions_fail_closed(patch):
    value = SearchMissionRequest.current_room(["person"], request_id="base").to_dict()
    value.update(patch)
    with pytest.raises(MissionValidationError):
        SearchMissionRequest.from_dict(value)


def test_search_task_list_has_one_canonical_admission_shape():
    tasks = canonicalize_search_tasks([{
        "type": "search_room",
        "priority": 99,
        "params": {
            "room": "__current__",
            "target_classes": ["chair", "chair"],
            "require_photos": True,
            "mark_on_map": True,
        },
    }])

    assert len(tasks) == 1
    assert tasks[0]["type"] == "search_room"
    assert tasks[0]["priority"] == 10
    assert tasks[0]["params"]["target_classes"] == ["chair"]
    assert tasks[0]["params"]["mission_request"]["room"] == "current_room"


@pytest.mark.parametrize("tasks", [
    [],
    [{"type": "move", "params": {"vx": 0.2}}],
    [{"params": {"room": "__current__", "target_classes": ["person"]}}],
    [
        {"type": "search_room", "params": {}},
        {"type": "search_room", "params": {}},
    ],
])
def test_search_task_list_rejects_missing_legacy_or_ambiguous_tasks(tasks):
    with pytest.raises(MissionValidationError):
        canonicalize_search_tasks(tasks)


def test_task_manager_never_defaults_missing_task_type_to_move():
    source = (Path(__file__).resolve().parent / "nx_web_server.py").read_text(
        encoding="utf-8")

    assert 't.get("type", "move")' not in source
    assert "canonicalize_search_tasks(tasks)" in source
