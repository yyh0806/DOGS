"""Canonical validated contract shared by text, voice, HTTP, and missions."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import re
from typing import Mapping
from uuid import uuid4


SCHEMA_VERSION = 1
_REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9_.:@/-]{1,128}$")
_TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,63}$")
_ROOM_RE = re.compile(r"^[A-Za-z0-9_\-\u4e00-\u9fff]{1,64}$")
_STRATEGIES = frozenset({"frontier_explore", "next_best_view"})
_ALIASES = {
    "人": "person",
    "人员": "person",
    "所有人": "person",
    "person": "person",
    "people": "person",
    "桌子": "dining table",
    "餐桌": "dining table",
    "table": "dining table",
    "dining table": "dining table",
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
        if not math.isfinite(radius) or radius <= 0.0:
            raise MissionValidationError("max_radius_m must be finite and positive")
        if not math.isfinite(duration) or duration <= 0.0:
            raise MissionValidationError("max_time_s must be finite and positive")
        if room == "current_room" and self.search_strategy != "frontier_explore":
            raise MissionValidationError("current_room requires frontier_explore")
        object.__setattr__(self, "room", room)
        object.__setattr__(self, "target_classes", targets)
        object.__setattr__(self, "max_radius_m", radius)
        object.__setattr__(self, "max_time_s", duration)
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
            max_radius_m=6.0,
            max_time_s=480.0,
        )

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        if not isinstance(value, Mapping):
            raise MissionValidationError("mission request must be an object")
        targets = value.get("target_classes")
        if not isinstance(targets, (list, tuple)):
            raise MissionValidationError("target_classes must be a list")
        try:
            return cls(
                schema_version=int(value.get("schema_version", SCHEMA_VERSION)),
                request_id=str(value.get("request_id", "")),
                room=str(value.get("room", "")),
                target_classes=tuple(targets),
                search_strategy=str(value.get("search_strategy", "")),
                require_photos=value.get("require_photos") is True,
                mark_on_map=value.get("mark_on_map") is True,
                max_radius_m=float(value.get("max_radius_m")),
                max_time_s=float(value.get("max_time_s")),
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
                    "max_radius_m", 6.0 if room == "current_room" else 12.0)),
                max_time_s=float(value.get(
                    "max_time_s", value.get(
                        "max_time", 480.0 if room == "current_room" else 900.0))),
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
        return params


__all__ = [
    "MissionValidationError", "SCHEMA_VERSION", "SearchMissionRequest",
    "canonicalize_search_tasks",
    "normalize_target_class",
]
