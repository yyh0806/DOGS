#!/usr/bin/env python3
"""Read-only post-deploy release and safe-park probe for the NX."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Mapping
from urllib.request import urlopen


class ReleaseProbeError(RuntimeError):
    pass


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ReleaseProbeError(f"{name} is not an object")
    return value


def validate_release_evidence(
    expected_release: object,
    version: object,
    dog_state: object,
    *,
    require_sdk_ready: bool,
) -> dict:
    expected = str(expected_release or "").strip()
    if not expected:
        raise ReleaseProbeError("expected release is empty")
    web = _mapping(version, "version evidence")
    motion = _mapping(dog_state, "dog state evidence")

    if str(web.get("release_id", "")) != expected:
        raise ReleaseProbeError("web release mismatch")
    if str(web.get("motion_release_id", "")) != expected:
        raise ReleaseProbeError("web/motion release mismatch")
    if web.get("release_consistent") is not True:
        raise ReleaseProbeError("web reports an inconsistent release")
    if str(motion.get("release_id", "")) != expected:
        raise ReleaseProbeError("dog release mismatch")
    try:
        schema = int(motion.get("schema_version", 0))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ReleaseProbeError("invalid dog state schema") from exc
    if schema < 4:
        raise ReleaseProbeError("dog state schema is older than v4")
    if require_sdk_ready and motion.get("sdk_ready") is not True:
        raise ReleaseProbeError("SDK is not ready")
    if motion.get("telemetry_fresh") is not True:
        raise ReleaseProbeError("telemetry is stale")
    if str(motion.get("motion_service", "")) != "ai-w":
        raise ReleaseProbeError("motion service is not ai-w")
    if motion.get("fault") not in (None, ""):
        raise ReleaseProbeError("motion fault is latched")
    if str(motion.get("session", "")) != "parked":
        raise ReleaseProbeError("motion session is not parked")
    if str(motion.get("physical_mode", "")) != "joint_lock":
        raise ReleaseProbeError("physical mode is not joint_lock")
    if str(motion.get("actual_motion", "")) != "stopped":
        raise ReleaseProbeError("wheels are not stopped")
    if motion.get("velocity_authorized") is not False:
        raise ReleaseProbeError("velocity remains authorized")

    return {
        "ok": True,
        "validated_at": datetime.now(timezone.utc).isoformat(),
        "release_id": expected,
        "web": {
            "release_id": web.get("release_id"),
            "motion_release_id": web.get("motion_release_id"),
            "release_consistent": web.get("release_consistent"),
        },
        "motion": {
            key: motion.get(key)
            for key in (
                "schema_version", "release_id", "sdk_ready",
                "telemetry_fresh", "motion_service", "session",
                "physical_mode", "actual_motion", "velocity_authorized",
                "fault", "transition", "raw",
            )
        },
    }


def _fetch_version(url: str, timeout: float) -> dict:
    with urlopen(url, timeout=max(0.1, float(timeout))) as response:
        if int(getattr(response, "status", 200)) != 200:
            raise ReleaseProbeError(
                f"version endpoint returned HTTP {response.status}")
        value = json.loads(response.read().decode("utf-8"))
    return dict(_mapping(value, "version response"))


def _wait_for_evidence(
    *,
    expected_release: str,
    version_url: str,
    timeout: float,
    require_sdk_ready: bool,
) -> dict:
    if not math.isfinite(float(timeout)) or float(timeout) <= 0.0:
        raise ReleaseProbeError("timeout must be finite and positive")
    deadline = time.monotonic() + float(timeout)
    version = None
    last_error: Exception | None = None
    while time.monotonic() < deadline and version is None:
        try:
            version = _fetch_version(
                version_url, min(2.0, deadline - time.monotonic()))
        except Exception as exc:  # HTTP service may still be starting.
            last_error = exc
            time.sleep(0.2)
    if version is None:
        raise ReleaseProbeError(
            f"version endpoint unavailable: {last_error or 'timeout'}")

    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as exc:
        raise ReleaseProbeError("rclpy/std_msgs is unavailable") from exc

    latest = None
    initialized_here = False
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            initialized_here = True
        node = rclpy.create_node("go2w_release_probe")

        def receive(message):
            nonlocal latest
            try:
                decoded = json.loads(message.data)
                if isinstance(decoded, dict):
                    latest = decoded
            except (AttributeError, json.JSONDecodeError):
                return

        node.create_subscription(String, "/dog_state", receive, 10)
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, remaining)))
            if latest is None:
                continue
            # The HTTP server can become reachable before its ROS callback has
            # cached the first motion status. Refresh the cross-process view
            # while waiting instead of pinning an early null motion release.
            try:
                version = _fetch_version(
                    version_url,
                    min(0.5, max(0.1, deadline - time.monotonic())),
                )
            except Exception as exc:
                last_error = exc
                continue
            try:
                return validate_release_evidence(
                    expected_release,
                    version,
                    latest,
                    require_sdk_ready=require_sdk_ready,
                )
            except ReleaseProbeError as exc:
                last_error = exc
        raise ReleaseProbeError(
            f"safe release evidence timed out: {last_error or 'no dog state'}")
    finally:
        if node is not None:
            node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def _write_report(path: object, value: Mapping[str, object]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected", required=True)
    parser.add_argument(
        "--version-url", default="http://127.0.0.1:8000/api/version")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--require-sdk-ready", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    try:
        report = _wait_for_evidence(
            expected_release=args.expected,
            version_url=args.version_url,
            timeout=args.timeout,
            require_sdk_ready=args.require_sdk_ready,
        )
    except ReleaseProbeError as exc:
        report = {
            "ok": False,
            "validated_at": datetime.now(timezone.utc).isoformat(),
            "release_id": args.expected,
            "reason": str(exc),
        }
        if args.output:
            _write_report(args.output, report)
        parser.exit(1, f"NX release probe failed: {exc}\n")
    if args.output:
        _write_report(args.output, report)
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
