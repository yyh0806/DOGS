"""搜索路径规划器。

从 Rust planner.rs 移植，支持割草机和螺旋两种覆盖模式。
输出航点列表供编排器转为 Nav2 目标。
"""

import math
from typing import List, Dict


def plan_lawnmower(width: float, height: float, spacing: float = 2.5,
                   origin_x: float = 0.0, origin_y: float = 0.0) -> List[Dict]:
    """割草机/弓字形覆盖路径。

    Args:
        width: 区域宽度 (米)
        height: 区域高度 (米)
        spacing: 行间距 (米)
        origin_x, origin_y: 区域左下角坐标
    Returns:
        航点列表 [{"x", "y", "yaw", "is_scan"}, ...]
    """
    if spacing <= 0:
        spacing = 2.5

    waypoints = []
    num_rows = max(1, math.ceil(height / spacing))
    actual_spacing = height / num_rows

    for row in range(num_rows + 1):
        y = origin_y + min(row * actual_spacing, height)

        # 行间连接
        if row > 0 and waypoints:
            last = waypoints[-1]
            x_start = origin_x if row % 2 == 0 else origin_x + width
            if abs(last["x"] - x_start) > 0.01:
                waypoints.append({"x": x_start, "y": y, "yaw": 0.0, "is_scan": False})

        # 交替方向
        if row % 2 == 0:
            waypoints.append({"x": origin_x, "y": y, "yaw": 0.0, "is_scan": True})
            waypoints.append({"x": origin_x + width, "y": y, "yaw": 0.0, "is_scan": True})
        else:
            waypoints.append({"x": origin_x + width, "y": y, "yaw": math.pi, "is_scan": True})
            waypoints.append({"x": origin_x, "y": y, "yaw": math.pi, "is_scan": True})

    return waypoints


def plan_spiral(width: float, height: float, spacing: float = 2.5,
                origin_x: float = 0.0, origin_y: float = 0.0) -> List[Dict]:
    """螺旋形覆盖路径（从中心向外扩展）。

    Args: 同 plan_lawnmower
    Returns: 航点列表
    """
    if spacing <= 0:
        spacing = 2.5

    cx = origin_x + width / 2.0
    cy = origin_y + height / 2.0
    max_radius = math.sqrt(width ** 2 + height ** 2) / 2.0

    num_turns = max(3, math.ceil(max_radius / spacing))
    points_per_turn = 12
    total_points = num_turns * points_per_turn

    waypoints = []
    for i in range(total_points + 1):
        angle = i * 2.0 * math.pi / points_per_turn
        radius = (i / total_points) * max_radius if total_points > 0 else 0.0

        x = max(origin_x, min(cx + radius * math.cos(angle), origin_x + width))
        y = max(origin_y, min(cy + radius * math.sin(angle), origin_y + height))

        is_scan = (i % points_per_turn == 0)
        waypoints.append({"x": x, "y": y, "yaw": angle, "is_scan": is_scan})

    return waypoints


def compute_path_length(waypoints: List[Dict]) -> float:
    """计算路径总长度。"""
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for i in range(len(waypoints) - 1):
        dx = waypoints[i + 1]["x"] - waypoints[i]["x"]
        dy = waypoints[i + 1]["y"] - waypoints[i]["y"]
        total += math.sqrt(dx * dx + dy * dy)
    return total
