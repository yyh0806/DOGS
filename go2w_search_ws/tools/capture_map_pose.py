#!/usr/bin/env python3
"""Capture one map->base_link pose without publishing or commanding motion."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion values must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def build_pose_receipt(
    *,
    map_frame: str,
    base_frame: str,
    x: float,
    y: float,
    quaternion: tuple[float, float, float, float],
    captured_at: float,
    room: str = "",
) -> dict:
    x = float(x)
    y = float(y)
    captured_at = float(captured_at)
    if not all(math.isfinite(value) for value in (x, y, captured_at)):
        raise ValueError("pose and timestamp values must be finite")
    yaw = quaternion_to_yaw(*quaternion)
    pose = {
        "x": round(x, 6),
        "y": round(y, 6),
        "yaw": round(yaw, 6),
    }
    receipt = {
        "read_only": True,
        "captured_at": captured_at,
        "frame_id": str(map_frame),
        "child_frame_id": str(base_frame),
        "pose": pose,
        "yaml_nav_pose": (
            f"nav_pose: {{x: {x:.5f}, y: {y:.5f}, yaw: {yaw:.5f}}}"),
    }
    if str(room).strip():
        receipt["room"] = str(room).strip()
    return receipt


def capture_transform(
    *, map_frame: str = "map", base_frame: str = "base_link",
    timeout: float = 5.0,
) -> dict:
    """Read one transform from tf2. ROS imports stay lazy for offline tests."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=None)
    node = Node("go2w_capture_map_pose")
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + float(timeout)
    last_error = "transform unavailable"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                message = buffer.lookup_transform(
                    str(map_frame), str(base_frame), Time())
            except TransformException as exc:
                last_error = str(exc)
                continue
            translation = message.transform.translation
            rotation = message.transform.rotation
            return {
                "x": float(translation.x),
                "y": float(translation.y),
                "quaternion": (
                    float(rotation.x), float(rotation.y),
                    float(rotation.z), float(rotation.w),
                ),
            }
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()
    raise RuntimeError(
        f"no {map_frame}->{base_frame} transform within {timeout:.1f}s: "
        f"{last_error}")


def _write_atomic(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(payload, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Read one map-frame robot pose; sends no robot command")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--room", default="")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0.5 <= args.timeout <= 30.0:
        parser.error("--timeout must be in [0.5, 30.0]")
    try:
        transform = capture_transform(
            map_frame=args.map_frame,
            base_frame=args.base_frame,
            timeout=args.timeout,
        )
        receipt = build_pose_receipt(
            map_frame=args.map_frame,
            base_frame=args.base_frame,
            x=transform["x"],
            y=transform["y"],
            quaternion=transform["quaternion"],
            captured_at=time.time(),
            room=args.room,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(json.dumps({
            "ok": False,
            "read_only": True,
            "error": str(exc),
        }, ensure_ascii=False))
        return 1
    payload = json.dumps(
        {"ok": True, **receipt}, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        _write_atomic(Path(args.output), payload)
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
