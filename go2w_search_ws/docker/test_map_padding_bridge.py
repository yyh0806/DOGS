"""Tests for the SLAM-map padding and robot-boundary safety gate."""

import ast
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/go2w_bridge/go2w_bridge/map_padding_bridge.py"


def _load_helpers():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    wanted = {
        "clear_obstacles_visible_in_scan",
        "pad_occupancy_data",
        "shift_grid_origin",
        "point_boundary_margin",
    }
    definitions = [
        item
        for item in tree.body
        if isinstance(item, ast.FunctionDef) and item.name in wanted
    ]
    assert {item.name for item in definitions} == wanted
    namespace = {"math": math}
    exec(
        compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace


def _cell_index(x, y, *, origin=-2.0, resolution=0.1, width=40):
    return int(math.floor((y - origin) / resolution)) * width + int(
        math.floor((x - origin) / resolution)
    )


def test_live_finite_scan_clears_only_stale_occupied_cells_before_endpoint():
    clear = _load_helpers()["clear_obstacles_visible_in_scan"]
    data = [-1] * (40 * 40)
    for x in (0.5, 1.0, 1.4):
        data[_cell_index(x, 0.0)] = 100

    cleaned, count = clear(
        data=data,
        width=40,
        height=40,
        resolution=0.1,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        angle_min=0.0,
        angle_increment=0.1,
        ranges=[1.5],
        range_min=0.2,
        range_max=8.0,
        endpoint_margin=0.2,
    )

    assert cleaned[_cell_index(0.5, 0.0)] == 0
    assert cleaned[_cell_index(1.0, 0.0)] == 0
    assert cleaned[_cell_index(1.4, 0.0)] == 100
    assert count == 2


def test_live_scan_never_marks_unknown_free_or_clears_without_a_finite_hit():
    clear = _load_helpers()["clear_obstacles_visible_in_scan"]
    data = [-1] * (40 * 40)
    data[_cell_index(0.5, 0.0)] = 100

    cleaned, count = clear(
        data=data,
        width=40,
        height=40,
        resolution=0.1,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        angle_min=0.0,
        angle_increment=0.1,
        ranges=[math.inf, math.nan],
        range_min=0.2,
        range_max=8.0,
        endpoint_margin=0.2,
    )

    assert cleaned == data
    assert count == 0
    assert cleaned[_cell_index(0.3, 0.0)] == -1


def test_healthy_contiguous_no_return_sector_clears_only_nearby_ghosts():
    clear = _load_helpers()["clear_obstacles_visible_in_scan"]
    data = [0] * (40 * 40)
    ghost_angle = 0.6
    ghost_x = 0.5 * math.cos(ghost_angle)
    ghost_y = 0.5 * math.sin(ghost_angle)
    data[_cell_index(ghost_x, ghost_y)] = 100
    far_x = 1.2 * math.cos(ghost_angle)
    far_y = 1.2 * math.sin(ghost_angle)
    data[_cell_index(far_x, far_y)] = 100

    cleaned, count = clear(
        data=data,
        width=40,
        height=40,
        resolution=0.1,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        angle_min=0.0,
        angle_increment=0.2,
        ranges=[math.inf] * 7 + [2.0],
        range_min=0.2,
        range_max=8.0,
        endpoint_margin=0.2,
        no_return_min_run=7,
        no_return_clear_range=1.0,
    )

    assert cleaned[_cell_index(ghost_x, ghost_y)] == 0
    assert cleaned[_cell_index(far_x, far_y)] == 100
    assert count == 1


def test_short_no_return_run_cannot_erase_persistent_occupancy():
    clear = _load_helpers()["clear_obstacles_visible_in_scan"]
    data = [0] * (40 * 40)
    data[_cell_index(0.5, 0.0)] = 100

    cleaned, count = clear(
        data=data,
        width=40,
        height=40,
        resolution=0.1,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        angle_min=-0.3,
        angle_increment=0.1,
        ranges=[math.inf] * 6 + [2.0],
        range_min=0.2,
        range_max=8.0,
        endpoint_margin=0.2,
        no_return_min_run=7,
        no_return_clear_range=1.0,
    )

    assert cleaned == data
    assert count == 0


def test_live_scan_clearing_respects_robot_and_grid_rotation():
    clear = _load_helpers()["clear_obstacles_visible_in_scan"]
    data = [0] * (20 * 20)
    # The grid is rotated +90 degrees. Grid-local (0.5, 0.0) is world (0, 0.5).
    data[0 * 20 + 5] = 100

    cleaned, count = clear(
        data=data,
        width=20,
        height=20,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=math.pi / 2.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=math.pi / 2.0,
        angle_min=0.0,
        angle_increment=0.1,
        ranges=[1.0],
        range_min=0.1,
        range_max=8.0,
        endpoint_margin=0.2,
    )

    assert cleaned[5] == 0
    assert count == 1


def test_padding_centres_original_grid_and_fills_unknown_cells():
    pad = _load_helpers()["pad_occupancy_data"]

    data, width, height, pad_cells = pad(
        data=[0, 100], width=2, height=1, resolution=0.5, padding_m=1.0
    )

    assert (width, height, pad_cells) == (6, 5, 2)
    assert len(data) == 30
    assert data[2 * width + 2 : 2 * width + 4] == [0, 100]
    assert sum(value != -1 for value in data) == 2


def test_padding_rounds_up_to_whole_cells_and_rejects_malformed_maps():
    pad = _load_helpers()["pad_occupancy_data"]

    _, width, height, pad_cells = pad(
        data=[0], width=1, height=1, resolution=0.3, padding_m=1.0
    )
    assert (width, height, pad_cells) == (9, 9, 4)

    invalid = (
        dict(data=[], width=1, height=1, resolution=0.1, padding_m=1.0),
        dict(data=[0], width=0, height=1, resolution=0.1, padding_m=1.0),
        dict(data=[0], width=1, height=1, resolution=0.0, padding_m=1.0),
        dict(data=[0], width=1, height=1, resolution=0.1, padding_m=-1.0),
    )
    for kwargs in invalid:
        with pytest.raises(ValueError):
            pad(**kwargs)


def test_origin_shift_respects_grid_rotation():
    shift = _load_helpers()["shift_grid_origin"]

    assert shift(5.0, 7.0, 0.0, 2.0) == pytest.approx((3.0, 5.0))
    assert shift(5.0, 7.0, math.pi / 2.0, 2.0) == pytest.approx((7.0, 5.0))


def test_boundary_margin_respects_grid_rotation_and_detects_outside_pose():
    margin = _load_helpers()["point_boundary_margin"]

    assert margin(
        point_x=0.0,
        point_y=0.0,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        width=100,
        height=100,
        resolution=0.05,
    ) == pytest.approx(2.0)
    # A 90-degree grid whose local (1, 2) point is world (-2, 1).
    assert margin(
        point_x=-2.0,
        point_y=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=math.pi / 2.0,
        width=10,
        height=10,
        resolution=0.5,
    ) == pytest.approx(1.0)
    assert margin(
        point_x=-0.1,
        point_y=0.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        width=10,
        height=10,
        resolution=0.5,
    ) < 0.0


def test_bridge_has_transient_reliable_raw_to_padded_map_contract():
    source = SOURCE.read_text(encoding="utf-8")

    assert 'declare_parameter("input_topic", "/map_frontier_raw")' in source
    assert 'declare_parameter("output_topic", "/map_frontier")' in source
    assert 'declare_parameter("padding_m", 2.0)' in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "--check-margin" in source
    assert 'Odometry, "/localization_pose"' in source
    fuser = (
        ROOT / "src/go2w_bridge/go2w_bridge/map_odom_fuser.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher(Odometry, '/localization_pose', 10)" in fuser
    assert "self._map_subscription = self.create_subscription(" in source
    assert "self._pose_subscription = self.create_subscription(" in source
