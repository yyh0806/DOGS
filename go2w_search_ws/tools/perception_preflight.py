#!/usr/bin/env python3
"""Wait for fresh open-vocabulary perception using read-only HTTP status."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Mapping
import urllib.error
import urllib.request


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _check(name: str, ok: bool, detail: Any) -> dict[str, Any]:
    return {"name": name, "ok": bool(ok), "detail": detail}


def evaluate_perception_status(
    status: Mapping[str, Any], expected_release: str,
) -> dict[str, Any]:
    perception = status.get("perception")
    if not isinstance(perception, Mapping):
        perception = {}
    age = _finite(perception.get("age_sec"))
    max_age = _finite(perception.get("max_age_sec"))
    if max_age is None or max_age <= 0.0:
        max_age = 2.0
    source = str(perception.get("source") or "")
    checks = [
        _check("release_id", status.get("release_id") == expected_release, {
            "expected": expected_release,
            "actual": status.get("release_id"),
        }),
        _check(
            "release_consistent",
            status.get("release_consistent") is True
            and status.get("motion_release_id") == expected_release,
            {
                "consistent": status.get("release_consistent"),
                "motion_release_id": status.get("motion_release_id"),
            },
        ),
        _check("engine_running", perception.get("running") is True,
               perception.get("running")),
        _check(
            "detector_ready",
            perception.get("detector_initialized") is True
            and perception.get("detector_ready") is True,
            {
                "initialized": perception.get("detector_initialized"),
                "ready": perception.get("detector_ready"),
                "model": perception.get("detector_model"),
            },
        ),
        _check(
            "open_vocabulary",
            perception.get("detector_open_vocabulary") is True,
            perception.get("detector_open_vocabulary"),
        ),
        _check(
            "real_frame_source",
            perception.get("frame_available") is True
            and source not in {"", "mock"},
            {"frame_available": perception.get("frame_available"),
             "source": source},
        ),
        _check(
            "frame_fresh",
            age is not None and -0.5 <= age <= max_age,
            {"age_sec": age, "max_age_sec": max_age},
        ),
        _check("perception_health", perception.get("healthy") is True,
               perception.get("reason")),
    ]
    return {
        "ok": all(item["ok"] for item in checks),
        "read_only": True,
        "expected_release": expected_release,
        "checks": checks,
        "perception": dict(perception),
    }


def _load_status(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        value = json.loads(response.read().decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("status response must be an object")
    return value


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
        description="Read-only wait for YOLO-World and a fresh real camera frame")
    parser.add_argument("--expected", required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8000/api/status")
    parser.add_argument("--wait", type=float, default=120.0)
    parser.add_argument("--request-timeout", type=float, default=3.0)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    if not math.isfinite(args.wait) or not 0.0 <= args.wait <= 300.0:
        parser.error("--wait must be in [0, 300]")
    if (not math.isfinite(args.request_timeout)
            or not 0.2 <= args.request_timeout <= 15.0):
        parser.error("--request-timeout must be in [0.2, 15.0]")

    deadline = time.monotonic() + args.wait
    report: dict[str, Any] = {
        "ok": False,
        "read_only": True,
        "error": "status_not_sampled",
        "checks": [],
    }
    while True:
        try:
            report = evaluate_perception_status(
                _load_status(args.url, args.request_timeout), args.expected)
        except (OSError, ValueError, json.JSONDecodeError,
                urllib.error.URLError) as exc:
            report = {
                "ok": False,
                "read_only": True,
                "expected_release": args.expected,
                "error": str(exc),
                "checks": [],
            }
        if report.get("ok") or time.monotonic() >= deadline:
            break
        time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))

    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        _write_atomic(Path(args.output), payload)
    print(payload, end="")
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
