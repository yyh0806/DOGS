"""Canonical validated contract shared by text, voice, HTTP, and missions."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import re
from typing import Mapping
from uuid import uuid4


SCHEMA_VERSION = 1
CURRENT_ROOM_MAX_RADIUS_M = 120.0
CURRENT_ROOM_MAX_TIME_S = 7200.0
CURRENT_ROOM_INITIAL_RADIUS_M = 16.0
CURRENT_ROOM_RADIUS_STEP_M = 16.0
CURRENT_ROOM_TILE_SIZE_M = 16.0
CURRENT_ROOM_MAX_FRONTIERS = 1000
_REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9_.:@/-]{1,128}$")
_TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,63}$")
_ROOM_RE = re.compile(r"^[A-Za-z0-9_\-\u4e00-\u9fff]{1,64}$")
_STRATEGIES = frozenset({"frontier_explore", "next_best_view"})
_ALIASES = {
    # 人
    "人": "person",
    "人员": "person",
    "所有人": "person",
    "person": "person",
    "people": "person",
    # 桌
    "桌子": "dining table",
    "餐桌": "dining table",
    "所有桌子": "dining table",
    "table": "dining table",
    "dining table": "dining table",
    # 椅
    "椅子": "chair",
    "座椅": "chair",
    "凳子": "chair",
    "所有椅子": "chair",
    "chair": "chair",
    # 室内家具电器 (spec §2.2, YOLO-World 开放词汇, 新词立即可检)
    "沙发": "couch", "couch": "couch",
    "床": "bed", "bed": "bed",
    "电视": "tv", "tv": "tv",
    "冰箱": "refrigerator", "refrigerator": "refrigerator",
    "微波炉": "microwave", "microwave": "microwave",
    "烤箱": "oven", "oven": "oven",
    "笔记本": "laptop", "laptop": "laptop",
    "杯子": "cup", "cup": "cup",
    "瓶子": "bottle", "bottle": "bottle",
    "书": "book", "book": "book",
    "钟": "clock", "clock": "clock",
    "花瓶": "vase", "vase": "vase",
    "绿植": "potted plant", "盆栽": "potted plant", "potted plant": "potted plant",
    "背包": "backpack", "backpack": "backpack",
    "碗": "bowl", "bowl": "bowl",
    "键盘": "keyboard", "keyboard": "keyboard",
}


class MissionValidationError(ValueError):
    pass


def normalize_target_class(value: object) -> str:
    text = " ".join(str(value or "").strip().split()).casefold()
    text = _ALIASES.get(text, text)
    if not text or not _TARGET_RE.fullmatch(text):
        raise MissionValidationError("invalid target class")
    return text


def canonicalize_search_tasks(tasks: object) -> list[dict]:
    """Validate exactly one task and return its canonical admission shape."""
    if not isinstance(tasks, list) or len(tasks) != 1:
        raise MissionValidationError(
            "exactly one search_room task is required")
    task = tasks[0]
    if not isinstance(task, Mapping) or task.get("type") != "search_room":
        raise MissionValidationError("task type must be search_room")
    mission = SearchMissionRequest.from_api_payload(task.get("params", {}))
    try:
        priority = int(task.get("priority", 8))
    except (TypeError, ValueError, OverflowError) as exc:
        raise MissionValidationError("invalid task priority") from exc
    priority = max(0, min(10, priority))
    return [{
        "type": "search_room",
        "priority": priority,
        "params": mission.to_task_params(),
    }]


_MOVE_MODES = frozenset({"linear", "angular"})
_MOVE_DIRECTIONS = frozenset({"forward", "backward", "left", "right"})


def canonicalize_move_tasks(tasks: object) -> list[dict]:
    """Validate exactly one move_relative task and return its canonical shape."""
    if not isinstance(tasks, list) or len(tasks) != 1:
        raise MissionValidationError(
            "exactly one move_relative task is required")
    task = tasks[0]
    if not isinstance(task, Mapping) or task.get("type") != "move_relative":
        raise MissionValidationError("task type must be move_relative")
    raw = task.get("params", {})
    if not isinstance(raw, Mapping):
        raise MissionValidationError("move params must be an object")
    mode = str(raw.get("mode", ""))
    direction = str(raw.get("direction", ""))
    if mode not in _MOVE_MODES:
        raise MissionValidationError("invalid move mode")
    if direction not in _MOVE_DIRECTIONS:
        raise MissionValidationError("invalid move direction")
    params: dict = {"mode": mode, "direction": direction,
                    "clamped": bool(raw.get("clamped", False))}
    if mode == "linear":
        distance = _finite_or_raise(raw.get("distance_m"), "distance_m")
        if distance <= 0.0:
            raise MissionValidationError("distance_m must be positive")
        params["distance_m"] = distance
    else:
        angle = _finite_or_raise(raw.get("angle_deg"), "angle_deg")
        if angle <= 0.0:
            raise MissionValidationError("angle_deg must be positive")
        params["angle_deg"] = angle
    try:
        priority = int(task.get("priority", 5))
    except (TypeError, ValueError, OverflowError) as exc:
        raise MissionValidationError("invalid task priority") from exc
    priority = max(0, min(10, priority))
    return [{"type": "move_relative", "priority": priority, "params": params}]


def _finite_or_raise(value, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise MissionValidationError(f"{name} must be a finite number")
    return parsed


@dataclass(frozen=True)
class SearchMissionRequest:
    request_id: str
    room: str
    target_classes: tuple[str, ...]
    search_strategy: str
    require_photos: bool
    mark_on_map: bool
    max_radius_m: float
    max_time_s: float
    schema_version: int = SCHEMA_VERSION
    initial_radius_m: float = 6.0
    radius_step_m: float = 6.0
    tile_size_m: float = 6.0
    frontier_spacing_m: float = 1.5
    stable_exhaustion_cycles: int = 3
    max_frontiers: int = 200
    max_plan_probes_per_cycle: int = 12

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise MissionValidationError("unsupported mission schema_version")
        if not _REQUEST_ID_RE.fullmatch(str(self.request_id)):
            raise MissionValidationError("invalid request_id")
        room = str(self.room).strip()
        if not _ROOM_RE.fullmatch(room):
            raise MissionValidationError("invalid room")
        if self.search_strategy not in _STRATEGIES:
            raise MissionValidationError("unsupported search_strategy")
        targets = tuple(normalize_target_class(value) for value in self.target_classes)
        targets = tuple(dict.fromkeys(targets))
        if not targets:
            raise MissionValidationError("target_classes must not be empty")
        radius = float(self.max_radius_m)
        duration = float(self.max_time_s)
        initial_radius = float(self.initial_radius_m)
        radius_step = float(self.radius_step_m)
        tile_size = float(self.tile_size_m)
        frontier_spacing = float(self.frontier_spacing_m)
        if not math.isfinite(radius) or radius <= 0.0:
            raise MissionValidationError("max_radius_m must be finite and positive")
        if not math.isfinite(duration) or duration <= 0.0:
            raise MissionValidationError("max_time_s must be finite and positive")
        if not math.isfinite(initial_radius) or initial_radius <= 0.0:
            raise MissionValidationError(
                "initial_radius_m must be finite and positive")
        if initial_radius > radius:
            raise MissionValidationError(
                "initial_radius_m must not exceed max_radius_m")
        if not math.isfinite(radius_step) or radius_step <= 0.0:
            raise MissionValidationError(
                "radius_step_m must be finite and positive")
        if not math.isfinite(tile_size) or tile_size <= 0.0:
            raise MissionValidationError("tile_size_m must be finite and positive")
        if not math.isfinite(frontier_spacing) or frontier_spacing <= 0.0:
            raise MissionValidationError(
                "frontier_spacing_m must be finite and positive")
        integer_fields = {
            "stable_exhaustion_cycles": self.stable_exhaustion_cycles,
            "max_frontiers": self.max_frontiers,
            "max_plan_probes_per_cycle": self.max_plan_probes_per_cycle,
        }
        normalized_integers = {}
        for name, value in integer_fields.items():
            try:
                integer = int(value)
            except (TypeError, ValueError, OverflowError) as exc:
                raise MissionValidationError(f"{name} must be a positive integer") from exc
            if integer <= 0 or float(value) != float(integer):
                raise MissionValidationError(f"{name} must be a positive integer")
            normalized_integers[name] = integer
        if room == "current_room" and self.search_strategy != "frontier_explore":
            raise MissionValidationError("current_room requires frontier_explore")
        object.__setattr__(self, "room", room)
        object.__setattr__(self, "target_classes", targets)
        object.__setattr__(self, "max_radius_m", radius)
        object.__setattr__(self, "max_time_s", duration)
        object.__setattr__(self, "initial_radius_m", initial_radius)
        object.__setattr__(self, "radius_step_m", radius_step)
        object.__setattr__(self, "tile_size_m", tile_size)
        object.__setattr__(self, "frontier_spacing_m", frontier_spacing)
        for name, value in normalized_integers.items():
            object.__setattr__(self, name, value)
        object.__setattr__(self, "require_photos", bool(self.require_photos))
        object.__setattr__(self, "mark_on_map", bool(self.mark_on_map))

    @classmethod
    def current_room(cls, target_classes, *, request_id=None):
        return cls(
            request_id=str(request_id or uuid4()),
            room="current_room",
            target_classes=tuple(target_classes),
            search_strategy="frontier_explore",
            require_photos=True,
            mark_on_map=True,
            max_radius_m=CURRENT_ROOM_MAX_RADIUS_M,
            max_time_s=CURRENT_ROOM_MAX_TIME_S,
            initial_radius_m=CURRENT_ROOM_INITIAL_RADIUS_M,
            radius_step_m=CURRENT_ROOM_RADIUS_STEP_M,
            tile_size_m=CURRENT_ROOM_TILE_SIZE_M,
            max_frontiers=CURRENT_ROOM_MAX_FRONTIERS,
        )

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        if not isinstance(value, Mapping):
            raise MissionValidationError("mission request must be an object")
        targets = value.get("target_classes")
        if not isinstance(targets, (list, tuple)):
            raise MissionValidationError("target_classes must be a list")
        room = str(value.get("room", ""))
        current_room = room == "current_room"
        try:
            return cls(
                schema_version=int(value.get("schema_version", SCHEMA_VERSION)),
                request_id=str(value.get("request_id", "")),
                room=room,
                target_classes=tuple(targets),
                search_strategy=str(value.get("search_strategy", "")),
                require_photos=value.get("require_photos") is True,
                mark_on_map=value.get("mark_on_map") is True,
                max_radius_m=float(value.get("max_radius_m")),
                max_time_s=float(value.get("max_time_s")),
                initial_radius_m=float(value.get(
                    "initial_radius_m",
                    CURRENT_ROOM_INITIAL_RADIUS_M if current_room else 6.0)),
                radius_step_m=float(value.get(
                    "radius_step_m",
                    CURRENT_ROOM_RADIUS_STEP_M if current_room else 6.0)),
                tile_size_m=float(value.get(
                    "tile_size_m",
                    CURRENT_ROOM_TILE_SIZE_M if current_room else 6.0)),
                frontier_spacing_m=float(value.get(
                    "frontier_spacing_m", 1.5)),
                stable_exhaustion_cycles=int(value.get(
                    "stable_exhaustion_cycles", 3)),
                max_frontiers=int(value.get(
                    "max_frontiers",
                    CURRENT_ROOM_MAX_FRONTIERS if current_room else 200)),
                max_plan_probes_per_cycle=int(value.get(
                    "max_plan_probes_per_cycle", 12)),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            if isinstance(exc, MissionValidationError):
                raise
            raise MissionValidationError("invalid mission request") from exc

    @classmethod
    def from_api_payload(cls, value: Mapping[str, object], *, request_id=None):
        """Validate canonical JSON or migrate one legacy HTTP payload."""
        if not isinstance(value, Mapping):
            raise MissionValidationError("mission request must be an object")
        nested = value.get("mission_request")
        if isinstance(nested, Mapping):
            return cls.from_dict(nested)
        if "schema_version" in value:
            return cls.from_dict(value)
        room = str(value.get("room", "")).strip()
        if room in {"__current__", "current", "current_room"}:
            room = "current_room"
        targets = value.get("target_classes")
        if not isinstance(targets, (list, tuple)):
            raise MissionValidationError("target_classes must be a list")
        strategy = str(value.get("search_strategy") or (
            "frontier_explore" if room == "current_room"
            else "next_best_view"))
        try:
            return cls(
                request_id=str(request_id or value.get("request_id") or uuid4()),
                room=room,
                target_classes=tuple(targets),
                search_strategy=strategy,
                require_photos=value.get("require_photos", True) is True,
                mark_on_map=value.get("mark_on_map", True) is True,
                max_radius_m=float(value.get(
                    "max_radius_m", CURRENT_ROOM_MAX_RADIUS_M
                    if room == "current_room" else 12.0)),
                max_time_s=float(value.get(
                    "max_time_s", value.get(
                        "max_time", CURRENT_ROOM_MAX_TIME_S
                        if room == "current_room" else 900.0))),
                initial_radius_m=float(value.get(
                    "initial_radius_m", CURRENT_ROOM_INITIAL_RADIUS_M
                    if room == "current_room" else 6.0)),
                radius_step_m=float(value.get(
                    "radius_step_m", CURRENT_ROOM_RADIUS_STEP_M
                    if room == "current_room" else 6.0)),
                tile_size_m=float(value.get(
                    "tile_size_m", CURRENT_ROOM_TILE_SIZE_M
                    if room == "current_room" else 6.0)),
                frontier_spacing_m=float(value.get(
                    "frontier_spacing_m", 1.5)),
                stable_exhaustion_cycles=int(value.get(
                    "stable_exhaustion_cycles", 3)),
                max_frontiers=int(value.get(
                    "max_frontiers", CURRENT_ROOM_MAX_FRONTIERS
                    if room == "current_room" else 200)),
                max_plan_probes_per_cycle=int(value.get(
                    "max_plan_probes_per_cycle", 12)),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            if isinstance(exc, MissionValidationError):
                raise
            raise MissionValidationError("invalid mission request") from exc

    def to_dict(self) -> dict:
        value = asdict(self)
        value["target_classes"] = list(self.target_classes)
        return value

    def to_task_params(self) -> dict:
        params = {
            "mission_request": self.to_dict(),
            "room": "__current__" if self.room == "current_room" else self.room,
            "target_classes": list(self.target_classes),
            "search_strategy": self.search_strategy,
            "require_photos": self.require_photos,
            "mark_on_map": self.mark_on_map,
            "max_radius_m": self.max_radius_m,
            "max_time": self.max_time_s,
        }
        if self.target_classes == ("person",):
            params["use_lidar_person_range"] = True
        else:
            params["use_lidar_target_range"] = True
        if self.room == "current_room":
            params.update({
                "initial_radius_m": self.initial_radius_m,
                "radius_step_m": self.radius_step_m,
                "tile_size_m": self.tile_size_m,
                "frontier_spacing_m": self.frontier_spacing_m,
                "stable_exhaustion_cycles": self.stable_exhaustion_cycles,
                "max_frontiers": self.max_frontiers,
                "max_plan_probes_per_cycle": self.max_plan_probes_per_cycle,
            })
        return params


__all__ = [
    "MissionValidationError", "SCHEMA_VERSION", "SearchMissionRequest",
    "canonicalize_search_tasks", "canonicalize_move_tasks",
    "normalize_target_class",
]
