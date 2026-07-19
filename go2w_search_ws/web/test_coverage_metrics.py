"""test_coverage_metrics.py — ROI 限定覆盖率 + enclosed_unknown 单测 (无 rclpy)。"""
import sys
from pathlib import Path
from types import SimpleNamespace

WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

from nx_coverage_metrics import compute_coverage  # noqa: E402


def _map(data, width, height, resolution=0.1, origin_x=0.0, origin_y=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        info=SimpleNamespace(
            resolution=resolution,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin_x, y=origin_y, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=list(data),
    )


def test_compute_coverage_none_map_returns_none():
    assert compute_coverage(None) is None


def test_compute_coverage_invalid_geometry_returns_none():
    bad = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        info=SimpleNamespace(
            resolution=0.0, width=10, height=10,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * 100,
    )
    assert compute_coverage(bad) is None


def test_compute_coverage_roi_circle_excludes_outer_padding():
    """整图 10x10, 四周 2 圈 unknown (padding), 中心 6x6 free.
    ROI 圆心 (0.5,0.5) 半径 0.3m (resolution=0.1 → 3 cell) → 只数中心 free,
    不被外围 padding 拉低 explored_ratio."""
    width = height = 10
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 7 and 2 <= c <= 7:
                data.append(0)
            else:
                data.append(-1)  # padding
    msg = _map(data, width, height, resolution=0.1, origin_x=0.0, origin_y=0.0)
    roi = {"type": "circle", "center": [0.5, 0.5], "radius": 0.35}
    result = compute_coverage(msg, roi=roi, mission_origin=(0.5, 0.5, 0.0))
    assert result is not None
    assert result["coverage_valid"] is True
    assert result["explored_ratio"] == 1.0  # ROI 内全 free
    assert result["unknown_cells"] == 0


def test_compute_coverage_whole_map_without_roi_marks_unverified():
    """roi=None → 仍计算但 coverage_valid=False (整图含 padding 不可信)."""
    msg = _map([0] * 100, 10, 10)
    result = compute_coverage(msg)
    assert result is not None
    assert result["coverage_valid"] is False
    assert result["roi"]["type"] == "whole_map"


def test_compute_coverage_walled_pocket_is_enclosed():
    """7x7: 中心 3x3 unknown 被 occupied 围死, 最外层 free.
    ROI = 整图圆覆盖; mission_origin 在最外层 free.
    中心 unknown 不接 reachable_free 也不接边界 → enclosed."""
    width = height = 7
    data = []
    for r in range(height):
        for c in range(width):
            if 2 <= r <= 4 and 2 <= c <= 4:
                data.append(-1)
            elif 1 <= r <= 5 and 1 <= c <= 5:
                data.append(100)
            else:
                data.append(0)
    msg = _map(data, width, height, resolution=0.5, origin_x=10.0, origin_y=20.0)
    roi = {"type": "circle", "center": [10.0 + 0.25, 20.0 + 0.25], "radius": 5.0}
    # mission_origin 在角落 free cell (row 0, col 0 → world ~ (10.25, 20.25))
    result = compute_coverage(
        msg, roi=roi, mission_origin=(10.25, 20.25, 0.0), inflation_radius_m=0.0)
    assert result["unknown_cells"] == 9
    assert len(result["enclosed_unknown_regions"]) == 1
    dz = result["enclosed_unknown_regions"][0]
    assert dz["cell_count"] == 9
    # cell-center local = (col+0.5)*0.5; col 2 → 1.25, col 4 → 2.25; origin (10,20)
    assert dz["min_x"] == 10.0 + 1.25
    assert dz["max_x"] == 10.0 + 2.25
    assert dz["min_y"] == 20.0 + 1.25
    assert dz["max_y"] == 20.0 + 2.25


def test_compute_coverage_unknown_touching_roi_boundary_not_enclosed():
    """ROI 边界处的 unknown 连通块不算 enclosed (可能是建筑外)."""
    # 5x5: 左上角 2x2 unknown 接触 ROI 边界, 其余 free
    width = height = 5
    data = []
    for r in range(height):
        for c in range(width):
            if r < 2 and c < 2:
                data.append(-1)
            else:
                data.append(0)
    msg = _map(data, width, height, resolution=0.5)
    roi = {"type": "circle", "center": [1.25, 1.25], "radius": 2.5}
    result = compute_coverage(
        msg, roi=roi, mission_origin=(2.0, 2.0, 0.0), inflation_radius_m=0.0)
    # 左上 unknown 接触 ROI/地图边界 → 不算 enclosed
    assert result["enclosed_unknown_regions"] == []


def test_compute_coverage_explored_ratio_rounded():
    data = [0, 0, 0, -1]  # 3 free, 1 unknown
    msg = _map(data, 2, 2)
    roi = {"type": "circle", "center": [0.1, 0.1], "radius": 1.0}
    result = compute_coverage(msg, roi=roi)
    assert result["explored_ratio"] == 0.75
