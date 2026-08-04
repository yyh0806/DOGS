#!/usr/bin/env python3
"""Wait on a local snapshot produced by nav_health_supervisor (no ROS client)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


def load_fresh(path: Path, max_age: float = 2.0) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    age = time.time() - float(data.get("updated_wall", 0.0))
    if age < -1.0 or age > max_age:
        raise ValueError(f"stale supervisor snapshot age={age:.3f}s")
    return data


def satisfied(data: dict, args: argparse.Namespace) -> tuple[bool, str]:
    if args.rate:
        topic, minimum = args.rate
        item = (data.get("topics") or {}).get(topic) or {}
        rate = item.get("rate_hz")
        samples = int(item.get("samples", 0))
        age_value = item.get("age_sec")
        age = math.inf if age_value is None else float(age_value)
        ok = rate is not None and float(rate) >= float(minimum) and samples >= 10 and age <= 1.0
        return ok, f"{topic} rate={rate} samples={samples} age={age:.3f}"
    if args.stamp_age:
        topic, maximum = args.stamp_age
        item = (data.get("topics") or {}).get(topic) or {}
        count = int(item.get("stamp_samples", 0))
        total = int(item.get("stamp_total", count))
        baseline = int(getattr(args, "stamp_baseline", 0) or 0)
        new_samples = max(0, total - baseline)
        median_age = item.get("stamp_median_sec")
        p95_age = item.get("stamp_p95_sec")
        ok = (
            count >= 20 and new_samples >= 20
            and median_age is not None and p95_age is not None
            and 0.0 <= float(median_age) <= float(maximum)
            and 0.0 <= float(p95_age) <= float(maximum)
        )
        return ok, (
            f"{topic} samples={count} new_samples={new_samples} median={median_age} "
            f"p95={p95_age} max={maximum}"
        )
    if args.message:
        item = (data.get("topics") or {}).get(args.message) or {}
        age_value = item.get("age_sec")
        age = math.inf if age_value is None else float(age_value)
        ok = int(item.get("samples", 0)) > 0 and age <= 3.0
        return ok, f"{args.message} samples={item.get('samples', 0)} age={item.get('age_sec')}"
    if args.tf:
        key = f"{args.tf[0]}->{args.tf[1]}"
        ok = bool((data.get("tf") or {}).get(key, False))
        return ok, f"tf {key}={ok}"
    if args.lifecycle:
        states = data.get("lifecycle") or {}
        missing = [name for name in args.lifecycle if int((states.get(name) or {}).get("id", -1)) != 3]
        return not missing, f"inactive={missing} states={states}"
    if args.action:
        ok = bool((data.get("actions") or {}).get(args.action, False))
        return ok, f"action {args.action}={ok}"
    if args.single_parent:
        child = args.single_parent
        dynamic = set((data.get("parents_dynamic") or {}).get(child) or [])
        static = set((data.get("parents_static") or {}).get(child) or [])
        parents = dynamic | static
        return len(parents) <= 1, f"child={child} parents={sorted(parents)}"
    if args.dog_ready:
        try:
            dog = json.loads(data.get("dog_state") or "{}")
        except json.JSONDecodeError:
            dog = {}
        ok = (
            dog.get("sdk_ready") is True and dog.get("nav_scan_fresh") is True
            and dog.get("session") == "parked"
            and dog.get("physical_mode") == "joint_lock"
            and dog.get("actual_motion") == "stopped"
            and dog.get("velocity_authorized") is False
            and dog.get("fault") is None
            and float(dog.get("battery_soc", 0)) >= 20
        )
        return ok, f"dog_ready={ok} state={dog}"
    raise ValueError("no gate selected")


def wait_gate(path: Path, timeout: float, args: argparse.Namespace) -> dict:
    deadline = time.monotonic() + timeout
    last = "snapshot unavailable"
    while time.monotonic() < deadline:
        try:
            data = load_fresh(path)
            if args.stamp_age and not hasattr(args, "stamp_baseline"):
                topic = args.stamp_age[0]
                item = (data.get("topics") or {}).get(topic) or {}
                args.stamp_baseline = int(
                    item.get("stamp_total", item.get("stamp_samples", 0)) or 0)
            ok, last = satisfied(data, args)
            if ok:
                return {"ok": True, "detail": last}
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            last = str(exc)
        time.sleep(0.1)
    return {"ok": False, "detail": last, "error": "timeout"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-file", required=True)
    parser.add_argument("--timeout", type=float, required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--rate", nargs=2, metavar=("TOPIC", "MIN_HZ"))
    group.add_argument("--stamp-age", nargs=2, metavar=("TOPIC", "MAX_SEC"))
    group.add_argument("--message")
    group.add_argument("--tf", nargs=2, metavar=("PARENT", "CHILD"))
    group.add_argument("--lifecycle", nargs="+")
    group.add_argument("--action")
    group.add_argument("--single-parent")
    group.add_argument("--dog-ready", action="store_true")
    args = parser.parse_args()
    if args.rate:
        args.rate[1] = float(args.rate[1])
    if args.stamp_age:
        args.stamp_age[1] = float(args.stamp_age[1])
    report = wait_gate(Path(args.state_file), args.timeout, args)
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
