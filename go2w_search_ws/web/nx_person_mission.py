"""Generic target marker state and mission media artifact storage."""

from __future__ import annotations

import copy
import json
import math
import re
import time
from pathlib import Path
from typing import Any

import numpy as np


_ARTIFACT_METADATA_KEYS = {"bbox", "frame_width", "frame_height", "raw_url", "photo_url", "crop_url"}
_CANONICAL_RANGE_KEYS = {
    "world_x", "world_y", "world_z", "x", "y", "z",
    "position_dimension", "position_quality", "position_weight",
    "range_source", "range_m", "height_m", "height_point_count",
    "capture_stamp", "pose_stamp", "scan_stamp", "cloud_stamp",
    "pose_delta_s", "scan_delta_s", "cloud_delta_s",
    "localization_quality", "canonical_observation_id",
    "canonical_range_quality_score",
}
_SAFE_SLUG_RE = re.compile(r"[^A-Za-z0-9_.-]+")


class TargetMissionStore:
    """Track and deduplicate target observations for one search mission."""

    def __init__(self, mission_id: str,
                 static_root: str | Path | None = None,
                 mission_root: str | Path | None = None,
                 merge_distance_m: float = 0.7,
                 appearance_merge_distance_m: float = 1.5,
                 appearance_match_threshold: float = 0.90,
                 appearance_separation_threshold: float = 0.45,
                 default_class: str = "person"):
        self.mission_id = str(mission_id)
        self.mission_slug = _safe_mission_slug(self.mission_id)
        self.static_root = Path(static_root) if static_root is not None else Path(__file__).resolve().parent / "static"
        self.mission_root = (
            Path(mission_root) if mission_root is not None
            else self.static_root / "missions"
        )
        self.merge_distance_m = float(merge_distance_m)
        self.appearance_merge_distance_m = float(appearance_merge_distance_m)
        self.appearance_match_threshold = float(appearance_match_threshold)
        self.appearance_separation_threshold = float(
            appearance_separation_threshold)
        self.default_class = _normalize_target_class(default_class, "person")
        self._markers: list[dict[str, Any]] = []
        self._appearance_by_id: dict[str, np.ndarray] = {}
        self._unresolved: list[dict[str, Any]] = []
        self._next_id = 1
        self._next_unresolved_id = 1
        self._next_observation_id = 1

    def add_observation(self, observation: dict, frame=None) -> dict:
        """Add one target observation and return the current marker state."""

        obs = copy.deepcopy(observation or {})
        observation_mission_id = str(obs.get("mission_id") or self.mission_id)
        if observation_mission_id != self.mission_id:
            raise ValueError("observation mission_id does not match mission store")
        obs["mission_id"] = self.mission_id
        if not str(obs.get("observation_id") or "").strip():
            obs["observation_id"] = (
                f"{self.mission_slug}-obs-{self._next_observation_id:04d}")
            self._next_observation_id += 1
        obs["class"] = _normalize_target_class(
            obs.get("class"), self.default_class)
        descriptor = (
            _appearance_descriptor(frame, obs.get("bbox"))
            if frame is not None else None
        )
        match, match_evidence = self._find_matching_marker(obs, descriptor)
        if match is None:
            marker = self._new_marker(obs)
            marker.update({
                "dedup_method": "new_track",
                "dedup_distance_m": None,
                "appearance_similarity": None,
                "merge_evidence": [],
            })
            should_save_media = True
        else:
            marker = copy.deepcopy(match)
            old_confidence = _to_float(marker.get("confidence"), default=0.0)
            new_confidence = _to_float(obs.get("confidence"), default=0.0)
            should_save_media = (
                new_confidence >= old_confidence
                or _range_quality_score(obs) > _canonical_quality_score(marker)
                or not marker.get("photo_url")
            )
            self._merge_marker(marker, obs)
            marker.update(match_evidence)
            evidence = {
                "observation_id": obs["observation_id"],
                "mission_id": self.mission_id,
                "class": obs["class"],
                "method": match_evidence.get("dedup_method"),
                "distance_m": match_evidence.get("dedup_distance_m"),
                "appearance_similarity": match_evidence.get(
                    "appearance_similarity"),
                "canonical_observation_id": marker.get(
                    "canonical_observation_id"),
                "range_quality_score": _range_quality_score(obs),
            }
            marker.setdefault("merge_evidence", []).append(evidence)

        try:
            if frame is not None and obs.get("position_quality") == "range_lidar" and should_save_media:
                self._save_artifacts(marker, obs, frame)
        except Exception:
            if match is None:
                self._next_id = max(1, self._next_id - 1)
            raise

        if match is None:
            self._markers.append(marker)
        else:
            self._markers[self._markers.index(match)] = marker
        if descriptor is not None:
            self._update_appearance(marker["id"], descriptor, obs)
        return copy.deepcopy(marker)

    def markers(self) -> list[dict]:
        """Return marker copies so callers cannot mutate store state."""

        return copy.deepcopy(self._markers)

    def add_unresolved_observation(self, observation: dict, frame=None) -> dict:
        """Persist a bearing-only observation without publishing a map marker."""

        obs = copy.deepcopy(observation or {})
        observation_mission_id = str(obs.get("mission_id") or self.mission_id)
        if observation_mission_id != self.mission_id:
            raise ValueError("observation mission_id does not match mission store")
        obs["mission_id"] = self.mission_id
        obs["class"] = _normalize_target_class(
            obs.get("class"), self.default_class)
        marker_id = f"unresolved_{self._next_unresolved_id:03d}"
        marker = _marker_from_observation(marker_id, obs)
        marker["observation_count"] = 1
        if frame is not None:
            self._save_artifacts(marker, obs, frame)
        self._unresolved.append(marker)
        self._next_unresolved_id += 1
        return copy.deepcopy(marker)

    def unresolved(self) -> list[dict]:
        return copy.deepcopy(self._unresolved)

    def resolve_unresolved(self, count: int = 1) -> int:
        resolved = min(max(int(count), 0), len(self._unresolved))
        if resolved:
            del self._unresolved[:resolved]
        return resolved

    def _find_matching_marker(self, obs: dict, descriptor=None):
        if obs.get("position_quality") != "range_lidar":
            return None, {}
        obs_x = _to_float(obs.get("world_x"))
        obs_y = _to_float(obs.get("world_y"))
        if obs_x is None or obs_y is None:
            return None, {}
        best_marker = None
        best_key = None
        best_evidence = {}
        obs_class = _normalize_target_class(
            obs.get("class"), self.default_class)
        for marker in self._markers:
            if marker.get("position_quality") != "range_lidar":
                continue
            marker_class = _normalize_target_class(
                marker.get("class"), self.default_class)
            if marker_class != obs_class:
                continue
            marker_x = _to_float(marker.get("world_x"))
            marker_y = _to_float(marker.get("world_y"))
            if marker_x is None or marker_y is None:
                continue
            distance = math.hypot(obs_x - marker_x, obs_y - marker_y)
            marker_descriptor = self._appearance_by_id.get(str(marker.get("id")))
            similarity = (
                _cosine_similarity(descriptor, marker_descriptor)
                if descriptor is not None and marker_descriptor is not None
                else None
            )

            method = None
            if distance <= self.merge_distance_m:
                if (similarity is not None
                        and similarity < self.appearance_separation_threshold):
                    continue
                method = "appearance_spatial" if similarity is not None else "spatial"
            elif (
                distance <= self.appearance_merge_distance_m
                and similarity is not None
                and similarity >= self.appearance_match_threshold
            ):
                method = "appearance_spatial"
            if method is None:
                continue

            # Prefer visual agreement when available, then the closest marker.
            key = (
                0 if similarity is not None else 1,
                -(similarity if similarity is not None else 0.0),
                distance,
                str(marker.get("id", "")),
            )
            if best_key is None or key < best_key:
                best_key = key
                best_marker = marker
                best_evidence = {
                    "dedup_method": method,
                    "dedup_distance_m": distance,
                    "appearance_similarity": similarity,
                }
        return best_marker, best_evidence

    def _update_appearance(self, marker_id: str, descriptor: np.ndarray,
                           obs: dict) -> None:
        marker_id = str(marker_id)
        current = self._appearance_by_id.get(marker_id)
        if current is None:
            self._appearance_by_id[marker_id] = descriptor.copy()
            return
        new_weight = max(_to_float(obs.get("confidence"), default=0.0), 0.01)
        marker = next(
            (item for item in self._markers if item.get("id") == marker_id), None)
        old_weight = max(
            _to_float((marker or {}).get("position_weight"), default=1.0), 0.01)
        combined = current * old_weight + descriptor * new_weight
        norm = float(np.linalg.norm(combined))
        self._appearance_by_id[marker_id] = (
            combined / norm if norm > 1e-12 else descriptor.copy())

    def _new_marker(self, obs: dict) -> dict[str, Any]:
        target_class = _normalize_target_class(
            obs.get("class"), self.default_class)
        obs["class"] = target_class
        marker_id = f"{_safe_target_slug(target_class)}_{self._next_id:03d}"
        self._next_id += 1
        marker = _marker_from_observation(marker_id, obs)
        marker["observation_count"] = 1
        marker["mission_id"] = self.mission_id
        marker["canonical_observation_id"] = obs.get("observation_id")
        marker["canonical_range_quality_score"] = _range_quality_score(obs)
        return marker

    def _merge_marker(self, marker: dict[str, Any], obs: dict) -> None:
        marker["observation_count"] = int(marker.get("observation_count", 1)) + 1
        last_observed_at = _to_float(obs.get("timestamp"), default=time.time())

        current_confidence = _to_float(marker.get("confidence"), default=0.0)
        new_confidence = _to_float(obs.get("confidence"), default=0.0)
        current_x = _to_float(marker.get("world_x"))
        current_y = _to_float(marker.get("world_y"))
        current_z = _to_float(marker.get("world_z"))
        new_x = _to_float(obs.get("world_x"))
        new_y = _to_float(obs.get("world_y"))
        new_z = _to_float(obs.get("world_z"))
        current_weight = _to_float(
            marker.get("position_weight"), default=max(current_confidence, 0.01))
        new_weight = max(new_confidence, 0.01)
        current_quality = _canonical_quality_score(marker)
        new_quality = _range_quality_score(obs)
        quality_delta = new_quality - current_quality
        canonical_snapshot = {
            key: copy.deepcopy(marker[key])
            for key in _CANONICAL_RANGE_KEYS
            if key in marker
        }

        if new_confidence >= current_confidence:
            marker_id = marker["id"]
            updates = _merge_updates_from_observation(marker_id, obs)
            marker.update(updates)
        marker["confidence"] = max(current_confidence, new_confidence)

        if quality_delta > 1e-9:
            marker_id = marker["id"]
            marker.update(_merge_updates_from_observation(marker_id, obs))
            marker["confidence"] = max(current_confidence, new_confidence)
            marker["canonical_observation_id"] = obs.get("observation_id")
            marker["canonical_range_quality_score"] = new_quality
            marker["position_weight"] = new_weight
            if new_z is None:
                for key in ("world_z", "z"):
                    marker.pop(key, None)
                marker["position_dimension"] = 2
        elif quality_delta < -1e-9:
            for key in _CANONICAL_RANGE_KEYS:
                marker.pop(key, None)
            marker.update(canonical_snapshot)
        elif None not in (current_x, current_y, new_x, new_y):
            total_weight = current_weight + new_weight
            fused_x = (current_x * current_weight + new_x * new_weight) / total_weight
            fused_y = (current_y * current_weight + new_y * new_weight) / total_weight
            marker.update({
                "world_x": fused_x,
                "world_y": fused_y,
                "x": fused_x,
                "y": fused_y,
                "position_weight": total_weight,
            })
            if current_z is not None and new_z is not None:
                fused_z = (
                    current_z * current_weight + new_z * new_weight
                ) / total_weight
                marker.update({
                    "world_z": fused_z,
                    "z": fused_z,
                    "position_dimension": 3,
                })
            elif new_z is not None:
                marker.update({
                    "world_z": new_z,
                    "z": new_z,
                    "position_dimension": 3,
                })
            elif current_z is not None:
                marker.update({
                    "world_z": current_z,
                    "z": current_z,
                    "position_dimension": 3,
                })
            marker["canonical_range_quality_score"] = current_quality
        marker["last_observed_at"] = last_observed_at

    def _save_artifacts(self, marker: dict[str, Any], obs: dict, frame) -> None:
        frame_array = _normalize_frame(frame)
        height, width = frame_array.shape[:2]
        bbox = _clamp_bbox(obs.get("bbox") or marker.get("bbox"), width, height)

        marker_id = marker["id"]
        mission_dir = self._mission_artifact_dir()
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
            "raw_url": f"/missions/{self.mission_slug}/{marker_id}_raw.jpg",
            "photo_url": f"/missions/{self.mission_slug}/{marker_id}_annotated.jpg",
            "crop_url": f"/missions/{self.mission_slug}/{marker_id}_crop.jpg",
        })
        with json_path.open("w", encoding="utf-8") as fh:
            json.dump(_json_safe(marker), fh, ensure_ascii=False, indent=2, sort_keys=True)

    def _mission_artifact_dir(self) -> Path:
        missions_root = self.mission_root.resolve()
        mission_dir = (missions_root / self.mission_slug).resolve()
        try:
            mission_dir.relative_to(missions_root)
        except ValueError as exc:
            raise ValueError("mission artifact directory escapes static missions root") from exc
        return mission_dir

    def save_report(self, report: dict[str, Any]) -> Path:
        """Atomically persist the mission report beside its media evidence."""

        report_data = _json_safe(copy.deepcopy(report or {}))
        if str(report_data.get("mission_id") or "") != self.mission_id:
            raise ValueError("report mission_id does not match mission store")
        mission_dir = self._mission_artifact_dir()
        mission_dir.mkdir(parents=True, exist_ok=True)
        report_path = mission_dir / "report.json"
        temporary_path = mission_dir / ".report.json.tmp"
        with temporary_path.open("w", encoding="utf-8") as fh:
            json.dump(
                report_data, fh, ensure_ascii=False, indent=2, sort_keys=True)
            fh.flush()
        temporary_path.replace(report_path)
        return report_path


class PersonMissionStore(TargetMissionStore):
    """Backward-compatible store whose default target class is ``person``."""

    def __init__(self, *args, **kwargs):
        kwargs.setdefault("default_class", "person")
        super().__init__(*args, **kwargs)


def load_latest_mission_report(
        mission_root: str | Path | None) -> dict[str, Any] | None:
    """Return the newest valid persisted report without trusting directory names."""

    if mission_root is None:
        return None
    root = Path(mission_root)
    if not root.is_dir():
        return None
    latest: tuple[float, dict[str, Any]] | None = None
    for report_path in root.glob("*/report.json"):
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
            if not isinstance(report, dict):
                continue
            mission_id = str(report.get("mission_id") or "").strip()
            detections = report.get("detections")
            if not mission_id or not isinstance(detections, list):
                continue
            try:
                ordering = float(report.get("end_time"))
            except (TypeError, ValueError):
                ordering = report_path.stat().st_mtime
            if not math.isfinite(ordering):
                ordering = report_path.stat().st_mtime
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            continue
        if latest is None or ordering > latest[0]:
            latest = ordering, report
    return None if latest is None else latest[1]


def _marker_from_observation(marker_id: str, obs: dict) -> dict[str, Any]:
    marker = copy.deepcopy(obs)
    marker["id"] = marker_id
    marker.setdefault("class", "person")
    marker["confidence"] = _to_float(marker.get("confidence"), default=0.0)
    marker.setdefault("timestamp", time.time())

    world_x = _to_float(marker.get("world_x"))
    world_y = _to_float(marker.get("world_y"))
    world_z = _to_float(marker.get("world_z"))
    if world_x is not None:
        marker["world_x"] = world_x
        marker["x"] = world_x
    if world_y is not None:
        marker["world_y"] = world_y
        marker["y"] = world_y
    if world_z is not None:
        marker["world_z"] = world_z
        marker["z"] = world_z
        marker["position_dimension"] = 3
    elif world_x is not None and world_y is not None:
        marker.setdefault("world_z", None)
        marker.setdefault("position_dimension", 2)
    if (
        marker.get("position_quality") == "range_lidar"
        and world_x is not None
        and world_y is not None
    ):
        marker["position_weight"] = max(marker["confidence"], 0.01)
    return marker


def _merge_updates_from_observation(marker_id: str, obs: dict) -> dict[str, Any]:
    updates = _marker_from_observation(marker_id, obs)
    for default_key in ("class", "confidence", "timestamp"):
        if default_key not in obs:
            updates.pop(default_key, None)
    for artifact_key in _ARTIFACT_METADATA_KEYS:
        updates.pop(artifact_key, None)
    return updates


def _canonical_quality_score(marker: dict) -> float:
    stored = _to_float(marker.get("canonical_range_quality_score"))
    return stored if stored is not None else _range_quality_score(marker)


def _range_quality_score(observation: dict) -> float:
    """Rank timestamp/range evidence independently from detector confidence.

    Equal legacy observations intentionally receive equal scores and retain the
    historical confidence-weighted position fusion. Synchronized observations
    outrank latest-pose estimates even when detector confidence is lower.
    """

    explicit = _to_float(observation.get("range_quality_score"))
    if explicit is not None:
        return explicit
    localization_rank = {
        "unsynchronized": 0.0,
        "latest_pose": 1.0,
        "timestamp_interpolated": 3.0,
        "timestamp_exact": 4.0,
    }.get(str(observation.get("localization_quality") or "").strip().lower(), 0.0)
    source_rank = 1.0 if str(
        observation.get("range_source") or "").strip().lower() == "lidar" else 0.0
    scan_delta = abs(_to_float(observation.get("scan_delta_s"), default=0.0))
    pose_delta = abs(_to_float(observation.get("pose_delta_s"), default=0.0))
    point_count = max(_to_float(
        observation.get("height_point_count"), default=0.0), 0.0)
    return (
        localization_rank * 100.0
        + source_rank * 10.0
        + min(point_count, 100.0) * 0.01
        - min(scan_delta + pose_delta, 1.0)
    )


def _safe_mission_slug(mission_id: str) -> str:
    basename = str(mission_id or "mission").replace("\\", "/").rstrip("/").split("/")[-1]
    slug = _SAFE_SLUG_RE.sub("-", basename).strip(".-_")
    return slug or "mission"


def _normalize_target_class(value, default: str = "person") -> str:
    normalized = " ".join(str(value or "").strip().split()).lower()
    if normalized:
        return normalized
    fallback = " ".join(str(default or "person").strip().split()).lower()
    return fallback or "person"


def _safe_target_slug(target_class: str) -> str:
    normalized = _normalize_target_class(target_class, "target")
    slug = re.sub(r"[^A-Za-z0-9]+", "_", normalized).strip("_").lower()
    return slug or "target"


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


def _appearance_descriptor(frame, bbox) -> np.ndarray:
    """Build a compact normalized color descriptor for cross-view dedup."""
    array = _normalize_frame(frame)
    height, width = array.shape[:2]
    x1, y1, x2, y2 = _clamp_bbox(bbox, width, height)
    crop = array[y1:y2, x1:x2]
    if crop.size == 0:
        raise ValueError("person crop must not be empty")
    if crop.ndim == 2:
        channels = [crop]
    else:
        channels = [crop[:, :, index] for index in range(min(3, crop.shape[2]))]
    features = []
    for channel in channels:
        histogram, _ = np.histogram(channel, bins=16, range=(0, 256))
        features.append(histogram.astype(np.float64))
    descriptor = np.concatenate(features)
    norm = float(np.linalg.norm(descriptor))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("person appearance descriptor is empty")
    return descriptor / norm


def _cosine_similarity(first, second) -> float | None:
    if first is None or second is None:
        return None
    try:
        a = np.asarray(first, dtype=np.float64).reshape(-1)
        b = np.asarray(second, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    if a.shape != b.shape or a.size == 0:
        return None
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    if not math.isfinite(denominator) or denominator <= 1e-12:
        return None
    value = float(np.dot(a, b) / denominator)
    return max(-1.0, min(1.0, value))


def _clamp_bbox(bbox, width: int, height: int) -> tuple[int, int, int, int]:
    if not bbox or len(bbox) < 4:
        return 0, 0, int(width), int(height)
    x1, y1, x2, y2 = (_to_float(v, default=0.0) for v in bbox[:4])
    left_value, right_value = sorted((x1, x2))
    top_value, bottom_value = sorted((y1, y2))
    left = max(0, min(int(round(left_value)), int(width)))
    top = max(0, min(int(round(top_value)), int(height)))
    right = max(0, min(int(round(right_value)), int(width)))
    bottom = max(0, min(int(round(bottom_value)), int(height)))
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
