"""Bounded timestamp matching for camera, localization, and MID360 evidence."""

from __future__ import annotations

from dataclasses import dataclass
import bisect
import math
import threading
from typing import Any, Optional


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StampedSample:
    stamp: float
    value: Any


@dataclass(frozen=True)
class ObservationBundle:
    capture_stamp: float
    pose: PoseSample
    scan: StampedSample
    cloud: Optional[StampedSample]
    frame: Optional[StampedSample]
    detection: Optional[StampedSample]
    pose_delta_s: float
    scan_delta_s: float
    cloud_delta_s: Optional[float]
    frame_delta_s: Optional[float]


class ObservationSynchronizer:
    def __init__(self, *, max_samples: int = 120, max_pose_gap: float = 2.0):
        if isinstance(max_samples, bool) or int(max_samples) < 2:
            raise ValueError("max_samples must be at least two")
        if not math.isfinite(float(max_pose_gap)) or float(max_pose_gap) <= 0:
            raise ValueError("max_pose_gap must be finite and positive")
        self._max_samples = int(max_samples)
        self._max_pose_gap = float(max_pose_gap)
        self._poses: list[PoseSample] = []
        self._scans: list[StampedSample] = []
        self._clouds: list[StampedSample] = []
        self._frames: list[StampedSample] = []
        self._detections: list[StampedSample] = []
        self._lock = threading.RLock()

    @staticmethod
    def _finite(value: object, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    def _insert(self, samples, sample) -> None:
        stamps = [item.stamp for item in samples]
        index = bisect.bisect_left(stamps, sample.stamp)
        if index < len(samples) and samples[index].stamp == sample.stamp:
            samples[index] = sample
        else:
            samples.insert(index, sample)
        del samples[:-self._max_samples]

    def add_pose(self, *, stamp, x, y, yaw) -> None:
        sample = PoseSample(
            self._finite(stamp, "stamp"),
            self._finite(x, "x"),
            self._finite(y, "y"),
            self._finite(yaw, "yaw"),
        )
        with self._lock:
            self._insert(self._poses, sample)

    def add_scan(self, *, stamp, scan) -> None:
        self._add_value(self._scans, stamp, scan)

    def add_cloud(self, *, stamp, cloud) -> None:
        self._add_value(self._clouds, stamp, cloud)

    def add_frame(self, *, stamp, frame) -> None:
        self._add_value(self._frames, stamp, frame)

    def add_detection(self, *, stamp, detection) -> None:
        self._add_value(self._detections, stamp, detection)

    def _add_value(self, history, stamp, value) -> None:
        sample = StampedSample(self._finite(stamp, "stamp"), value)
        with self._lock:
            self._insert(history, sample)

    @staticmethod
    def _nearest(samples, stamp: float, tolerance: float):
        if not samples:
            return None
        stamps = [item.stamp for item in samples]
        index = bisect.bisect_left(stamps, stamp)
        candidates = []
        if index < len(samples):
            candidates.append(samples[index])
        if index:
            candidates.append(samples[index - 1])
        sample = min(candidates, key=lambda item: (abs(item.stamp - stamp), item.stamp))
        return sample if abs(sample.stamp - stamp) <= tolerance else None

    def _pose_at(self, stamp: float, tolerance: float):
        if not self._poses:
            return None
        stamps = [item.stamp for item in self._poses]
        index = bisect.bisect_left(stamps, stamp)
        if index < len(self._poses) and self._poses[index].stamp == stamp:
            return self._poses[index]
        if index == 0 or index == len(self._poses):
            nearest = self._poses[0] if index == 0 else self._poses[-1]
            if abs(nearest.stamp - stamp) <= tolerance:
                return PoseSample(stamp, nearest.x, nearest.y, nearest.yaw)
            return None
        before = self._poses[index - 1]
        after = self._poses[index]
        span = after.stamp - before.stamp
        if span <= 0.0 or span > self._max_pose_gap:
            return None
        ratio = (stamp - before.stamp) / span
        delta_yaw = math.atan2(
            math.sin(after.yaw - before.yaw),
            math.cos(after.yaw - before.yaw),
        )
        yaw = before.yaw + ratio * delta_yaw
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return PoseSample(
            stamp=stamp,
            x=before.x + ratio * (after.x - before.x),
            y=before.y + ratio * (after.y - before.y),
            yaw=yaw,
        )

    def bundle_for_detection(self, *, stamp, tolerance=0.15):
        capture = self._finite(stamp, "stamp")
        tolerance = self._finite(tolerance, "tolerance")
        if tolerance <= 0.0:
            raise ValueError("tolerance must be positive")
        with self._lock:
            pose = self._pose_at(capture, tolerance)
            scan = self._nearest(self._scans, capture, tolerance)
            if pose is None or scan is None:
                return None
            cloud = self._nearest(self._clouds, capture, tolerance)
            frame = self._nearest(self._frames, capture, tolerance)
            detection = self._nearest(self._detections, capture, tolerance)
            return ObservationBundle(
                capture_stamp=capture,
                pose=pose,
                scan=scan,
                cloud=cloud,
                frame=frame,
                detection=detection,
                pose_delta_s=abs(pose.stamp - capture),
                scan_delta_s=abs(scan.stamp - capture),
                cloud_delta_s=(None if cloud is None else abs(cloud.stamp - capture)),
                frame_delta_s=(None if frame is None else abs(frame.stamp - capture)),
            )

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "pose_samples": len(self._poses),
                "scan_samples": len(self._scans),
                "cloud_samples": len(self._clouds),
                "frame_samples": len(self._frames),
                "detection_samples": len(self._detections),
            }


__all__ = [
    "ObservationBundle", "ObservationSynchronizer", "PoseSample",
    "StampedSample",
]
