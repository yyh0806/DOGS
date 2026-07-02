"""Person marker state and mission media artifact storage."""

from __future__ import annotations

import copy
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np


class PersonMissionStore:
    """Track person observations for one room-search mission."""

    def __init__(self, mission_id: str, static_root: str | Path | None = None, merge_distance_m: float = 0.7):
        self.mission_id = str(mission_id)
        self.static_root = Path(static_root) if static_root is not None else Path(__file__).resolve().parent / "static"
        self.merge_distance_m = float(merge_distance_m)
        self._markers: list[dict[str, Any]] = []
        self._next_id = 1

    def add_observation(self, observation: dict, frame=None) -> dict:
        """Add one person observation and return the current marker state."""

        obs = copy.deepcopy(observation or {})
        match = self._find_matching_marker(obs)
        if match is None:
            marker = self._new_marker(obs)
            self._markers.append(marker)
            should_save_media = True
        else:
            marker = match
            old_confidence = _to_float(marker.get("confidence"), default=0.0)
            new_confidence = _to_float(obs.get("confidence"), default=0.0)
            should_save_media = new_confidence >= old_confidence or not marker.get("photo_url")
            self._merge_marker(marker, obs)

        if frame is not None and obs.get("position_quality") == "range_lidar" and should_save_media:
            self._save_artifacts(marker, obs, frame)
        return copy.deepcopy(marker)

    def markers(self) -> list[dict]:
        """Return marker copies so callers cannot mutate store state."""

        return copy.deepcopy(self._markers)

    def _find_matching_marker(self, obs: dict) -> dict[str, Any] | None:
        if obs.get("position_quality") != "range_lidar":
            return None
        obs_x = _to_float(obs.get("world_x"))
        obs_y = _to_float(obs.get("world_y"))
        if obs_x is None or obs_y is None:
            return None
        for marker in self._markers:
            if marker.get("position_quality") != "range_lidar":
                continue
            marker_x = _to_float(marker.get("world_x"))
            marker_y = _to_float(marker.get("world_y"))
            if marker_x is None or marker_y is None:
                continue
            if math.hypot(obs_x - marker_x, obs_y - marker_y) <= self.merge_distance_m:
                return marker
        return None

    def _new_marker(self, obs: dict) -> dict[str, Any]:
        marker_id = f"person_{self._next_id:03d}"
        self._next_id += 1
        marker = _marker_from_observation(marker_id, obs)
        marker["observation_count"] = 1
        return marker

    def _merge_marker(self, marker: dict[str, Any], obs: dict) -> None:
        marker["observation_count"] = int(marker.get("observation_count", 1)) + 1
        marker["last_observed_at"] = _to_float(obs.get("timestamp"), default=time.time())

        current_confidence = _to_float(marker.get("confidence"), default=0.0)
        new_confidence = _to_float(obs.get("confidence"), default=0.0)
        if new_confidence >= current_confidence:
            marker_id = marker["id"]
            observation_count = marker["observation_count"]
            marker.clear()
            marker.update(_marker_from_observation(marker_id, obs))
            marker["observation_count"] = observation_count
        else:
            marker["confidence"] = current_confidence

    def _save_artifacts(self, marker: dict[str, Any], obs: dict, frame) -> None:
        frame_array = _normalize_frame(frame)
        height, width = frame_array.shape[:2]
        bbox = _clamp_bbox(obs.get("bbox") or marker.get("bbox"), width, height)

        marker_id = marker["id"]
        mission_dir = self.static_root / "missions" / self.mission_id
        mission_dir.mkdir(parents=True, exist_ok=True)

        raw_path = mission_dir / f"{marker_id}_raw.jpg"
        annotated_path = mission_dir / f"{marker_id}_annotated.jpg"
        crop_path = mission_dir / f"{marker_id}_crop.jpg"
        json_path = mission_dir / f"{marker_id}.json"

        annotated = frame_array.copy()
        _draw_bbox(annotated, bbox)
        x1, y1, x2, y2 = bbox
        crop = frame_array[y1:y2, x1:x2].copy()

        _write_jpeg(raw_path, frame_array)
        _write_jpeg(annotated_path, annotated)
        _write_jpeg(crop_path, crop)

        marker.update({
            "bbox": list(bbox),
            "frame_width": width,
            "frame_height": height,
            "raw_url": f"/missions/{self.mission_id}/{marker_id}_raw.jpg",
            "photo_url": f"/missions/{self.mission_id}/{marker_id}_annotated.jpg",
            "crop_url": f"/missions/{self.mission_id}/{marker_id}_crop.jpg",
        })
        with json_path.open("w", encoding="utf-8") as fh:
            json.dump(_json_safe(marker), fh, ensure_ascii=False, indent=2, sort_keys=True)


def _marker_from_observation(marker_id: str, obs: dict) -> dict[str, Any]:
    marker = copy.deepcopy(obs)
    marker["id"] = marker_id
    marker.setdefault("class", "person")
    marker["confidence"] = _to_float(marker.get("confidence"), default=0.0)
    marker.setdefault("timestamp", time.time())

    world_x = _to_float(marker.get("world_x"))
    world_y = _to_float(marker.get("world_y"))
    if world_x is not None:
        marker["world_x"] = world_x
        marker["x"] = world_x
    if world_y is not None:
        marker["world_y"] = world_y
        marker["y"] = world_y
    return marker


def _to_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _normalize_frame(frame) -> np.ndarray:
    array = np.asarray(frame)
    if array.ndim not in (2, 3):
        raise ValueError("frame must be a 2D or 3D image array")
    if array.size == 0:
        raise ValueError("frame must not be empty")
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def _clamp_bbox(bbox, width: int, height: int) -> tuple[int, int, int, int]:
    if not bbox or len(bbox) < 4:
        return 0, 0, int(width), int(height)
    x1, y1, x2, y2 = (_to_float(v, default=0.0) for v in bbox[:4])
    left = max(0, min(int(round(x1)), int(width)))
    top = max(0, min(int(round(y1)), int(height)))
    right = max(0, min(int(round(x2)), int(width)))
    bottom = max(0, min(int(round(y2)), int(height)))
    if right <= left or bottom <= top:
        return 0, 0, int(width), int(height)
    return left, top, right, bottom


def _draw_bbox(image: np.ndarray, bbox: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = bbox
    try:
        import cv2

        cv2.rectangle(image, (x1, y1), (max(x1, x2 - 1), max(y1, y2 - 1)), (0, 255, 0), 2)
        return
    except Exception:
        pass

    from PIL import Image, ImageDraw

    pil_image = Image.fromarray(_array_for_pil(image))
    draw = ImageDraw.Draw(pil_image)
    draw.rectangle((x1, y1, max(x1, x2 - 1), max(y1, y2 - 1)), outline=(0, 255, 0), width=2)
    replacement = np.asarray(pil_image)
    if image.ndim == 2:
        image[:, :] = replacement
    else:
        image[:, :, :replacement.shape[2]] = replacement


def _write_jpeg(path: Path, image: np.ndarray) -> None:
    try:
        import cv2

        if cv2.imwrite(str(path), image):
            return
    except Exception:
        pass

    from PIL import Image

    Image.fromarray(_array_for_pil(image)).save(path, format="JPEG", quality=90)


def _array_for_pil(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    if image.shape[2] == 1:
        return image[:, :, 0]
    if image.shape[2] == 4:
        return image[:, :, :3]
    return image


def _json_safe(value):
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value
