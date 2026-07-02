"""Room-bounded next-best-view active search planning."""

import math
from collections.abc import Iterable


_GRID_RATIO_TOLERANCE = 1e-9
_MAX_GRID_CANDIDATES = 100_000


class ActiveSearchPlanner:
    def __init__(self, spacing: float = 1.0, obstacle_clearance: float = 0.5):
        self.spacing = self._finite_positive_value(spacing, "spacing")
        self.obstacle_clearance = self._finite_positive_value(obstacle_clearance, "obstacle_clearance")
        self._blocked = set()
        self._visited = set()

    def generate_candidates(self, room_area: dict, robot_pose: tuple[float, float, float], obstacles) -> list[dict]:
        origin_x, origin_y, width, height = self._validate_room_area(room_area)
        x_steps, max_x, center_x = self._grid_axis(origin_x, width, "origin_x", "width")
        y_steps, max_y, center_y = self._grid_axis(origin_y, height, "origin_y", "height")
        self._validate_grid_size(x_steps, y_steps)
        obstacle_points = list(self._obstacle_points(obstacles))

        candidates = []
        for x_index in range(x_steps):
            x = self._grid_coordinate(origin_x, width, x_index, x_steps)
            for y_index in range(y_steps):
                y = self._grid_coordinate(origin_y, height, y_index, y_steps)
                rounded_x = round(x, 2)
                rounded_y = round(y, 2)
                key = (rounded_x, rounded_y)
                obstacle_distance = self._nearest_obstacle_distance(x, y, obstacle_points)
                if key not in self._blocked and not self._too_close_to_obstacle(obstacle_distance):
                    candidates.append(
                        {
                            "x": rounded_x,
                            "y": rounded_y,
                            "yaw": self._yaw_toward(center_x, center_y, x, y),
                            "information_gain": self._information_gain(x, y, origin_x, origin_y, max_x, max_y),
                            "visual_coverage_gain": self._visual_coverage_gain(
                                x,
                                y,
                                center_x,
                                center_y,
                                width,
                                height,
                            ),
                            "obstacle_risk_cost": self._obstacle_risk_cost(obstacle_distance),
                            "repeated_observation_penalty": 1.0 if key in self._visited else 0.0,
                        }
                    )

        return candidates

    def select_next_best(self, candidates: list[dict], robot_pose: tuple[float, float, float]) -> dict | None:
        if not candidates:
            return None

        robot_x = float(robot_pose[0])
        robot_y = float(robot_pose[1])

        def scored(candidate):
            path_cost = math.hypot(float(candidate["x"]) - robot_x, float(candidate["y"]) - robot_y)
            score = (
                float(candidate.get("information_gain", 0.0))
                + float(candidate.get("visual_coverage_gain", 0.0))
                - path_cost
                - float(candidate.get("obstacle_risk_cost", 0.0))
                - float(candidate.get("repeated_observation_penalty", 0.0))
            )
            return score, -path_cost

        selected = max(candidates, key=scored)
        score, _ = scored(selected)
        result = dict(selected)
        result["score"] = score
        return result

    def mark_blocked(self, candidate: dict) -> None:
        self._blocked.add(self._key(candidate))

    def mark_visited(self, candidate: dict) -> None:
        self._visited.add(self._key(candidate))

    def _key(self, candidate: dict) -> tuple[float, float]:
        return (round(float(candidate["x"]), 2), round(float(candidate["y"]), 2))

    def _too_close_to_obstacle(self, obstacle_distance: float | None) -> bool:
        return obstacle_distance is not None and obstacle_distance < self.obstacle_clearance

    def _obstacle_risk_cost(self, obstacle_distance: float | None) -> float:
        if obstacle_distance is None:
            return 0.0
        return max(0.0, self.obstacle_clearance * 2.0 - obstacle_distance)

    def _nearest_obstacle_distance(self, x: float, y: float, obstacles: list[tuple[float, float]]) -> float | None:
        if not obstacles:
            return None
        return min(math.hypot(point_x - x, point_y - y) for point_x, point_y in obstacles)

    def _obstacle_points(self, obstacles) -> Iterable[tuple[float, float]]:
        if obstacles is None:
            return
        if isinstance(obstacles, dict):
            point = self._normalize_obstacle_point(obstacles)
            if point is not None:
                yield point
            return
        if isinstance(obstacles, (str, bytes)):
            return
        try:
            iterator = iter(obstacles)
        except TypeError:
            return

        for obstacle in iterator:
            point = self._normalize_obstacle_point(obstacle)
            if point is not None:
                yield point

    def _normalize_obstacle_point(self, obstacle) -> tuple[float, float] | None:
        if isinstance(obstacle, dict):
            if "x" not in obstacle or "y" not in obstacle:
                return None
            return self._finite_point_or_none(obstacle["x"], obstacle["y"])
        if isinstance(obstacle, (str, bytes)):
            return None
        try:
            return self._finite_point_or_none(obstacle[0], obstacle[1])
        except (TypeError, IndexError, KeyError):
            return None

    def _finite_point_or_none(self, x_value, y_value) -> tuple[float, float] | None:
        try:
            x = float(x_value)
            y = float(y_value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return (x, y)

    def _validate_room_area(self, room_area: dict) -> tuple[float, float, float, float]:
        if not isinstance(room_area, dict):
            raise ValueError("room_area must be a mapping with origin_x, origin_y, width, and height")

        origin_x = self._finite_room_area_value(room_area, "origin_x")
        origin_y = self._finite_room_area_value(room_area, "origin_y")
        width = self._finite_room_area_value(room_area, "width")
        height = self._finite_room_area_value(room_area, "height")
        if width <= 0.0:
            raise ValueError("room_area.width must be a finite positive value")
        if height <= 0.0:
            raise ValueError("room_area.height must be a finite positive value")
        return origin_x, origin_y, width, height

    def _finite_room_area_value(self, room_area: dict, field: str) -> float:
        if field not in room_area:
            raise ValueError(f"room_area.{field} is required")
        value = self._finite_number(room_area[field], f"room_area.{field}")
        return value

    def _finite_positive_value(self, value, name: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite positive value") from exc
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return number

    def _finite_number(self, value, name: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} must be a finite number")
        return number

    def _grid_axis(
        self,
        origin: float,
        extent: float,
        origin_field: str,
        extent_field: str,
    ) -> tuple[int, float, float]:
        max_value = origin + extent
        center_value = origin + extent / 2.0
        if not (
            math.isfinite(max_value)
            and math.isfinite(center_value)
            and origin < center_value < max_value
        ):
            raise ValueError(
                f"room_area.{origin_field}/{extent_field} is too large or unsafe for grid generation"
            )

        ratio = extent / self.spacing
        if not math.isfinite(ratio) or ratio > _MAX_GRID_CANDIDATES:
            raise ValueError(f"room_area.{extent_field} is too large for grid generation")

        steps_after_origin = math.floor(ratio + _GRID_RATIO_TOLERANCE)
        step_count = int(steps_after_origin) + 1
        if step_count > 1 and origin + self.spacing <= origin:
            raise ValueError(
                f"room_area.{origin_field}/{extent_field} is too large or unsafe for grid generation"
            )
        return step_count, max_value, center_value

    def _validate_grid_size(self, x_steps: int, y_steps: int) -> None:
        if x_steps * y_steps > _MAX_GRID_CANDIDATES:
            raise ValueError("room_area grid is too large for grid generation")

    def _grid_coordinate(self, origin: float, extent: float, index: int, step_count: int) -> float:
        offset = index * self.spacing
        if index == step_count - 1 and math.isclose(
            offset,
            extent,
            rel_tol=0.0,
            abs_tol=abs(self.spacing) * _GRID_RATIO_TOLERANCE,
        ):
            offset = extent
        return origin + offset

    def _information_gain(self, x: float, y: float, min_x: float, min_y: float, max_x: float, max_y: float) -> float:
        distance_to_edge = min(x - min_x, max_x - x, y - min_y, max_y - y)
        return max(0.0, 2.0 - distance_to_edge)

    def _visual_coverage_gain(
        self,
        x: float,
        y: float,
        center_x: float,
        center_y: float,
        width: float,
        height: float,
    ) -> float:
        max_distance = max(math.hypot(width / 2.0, height / 2.0), 1e-9)
        distance_to_center = math.hypot(x - center_x, y - center_y)
        return max(0.0, 2.0 * (1.0 - distance_to_center / max_distance))

    def _yaw_toward(self, target_x: float, target_y: float, x: float, y: float) -> float:
        if math.isclose(target_x, x) and math.isclose(target_y, y):
            return 0.0
        return math.atan2(target_y - y, target_x - x)
