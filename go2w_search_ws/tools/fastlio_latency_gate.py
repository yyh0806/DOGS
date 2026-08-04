#!/usr/bin/env python3
"""Fail-closed, subscription-only latency gate for raw FAST_LIO odometry."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from typing import NamedTuple, Sequence


class GateDecision(NamedTuple):
    passed: bool
    failures: tuple[str, ...]
    sample_count: int
    median_age_sec: float | None
    p95_age_sec: float | None


def _percentile_nearest_rank(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(float(value) for value in values)
    rank = max(1, math.ceil(float(percentile) * len(ordered)))
    return ordered[min(len(ordered), rank) - 1]


def evaluate_samples(
    samples,
    *,
    minimum_samples: int,
    max_median_age_sec: float,
    max_p95_age_sec: float,
) -> GateDecision:
    rows = list(samples)
    failures = []
    if len(rows) < int(minimum_samples):
        failures.append("insufficient_samples")

    stamps = []
    ages = []
    for stamp, age in rows:
        try:
            stamp = float(stamp)
            age = float(age)
        except (TypeError, ValueError, OverflowError):
            failures.append("nonfinite_age")
            continue
        if not math.isfinite(stamp):
            failures.append("nonfinite_stamp")
        if not math.isfinite(age):
            failures.append("nonfinite_age")
        stamps.append(stamp)
        ages.append(age)

    if len(stamps) >= 2 and any(
        current <= previous for previous, current in zip(stamps, stamps[1:])
    ):
        failures.append("nonmonotonic_stamp")

    finite_ages = [age for age in ages if math.isfinite(age)]
    median_age = (
        float(statistics.median(finite_ages)) if finite_ages else None
    )
    p95_age = (
        _percentile_nearest_rank(finite_ages, 0.95) if finite_ages else None
    )
    if median_age is not None and median_age > float(max_median_age_sec):
        failures.append("median_age")
    if p95_age is not None and p95_age > float(max_p95_age_sec):
        failures.append("p95_age")
    unique_failures = tuple(dict.fromkeys(failures))
    return GateDecision(
        not unique_failures,
        unique_failures,
        len(rows),
        median_age,
        p95_age,
    )


def _post_warmup_samples(samples, *, warmup_samples: int, sample_count: int):
    """Return a bounded steady-state window after startup transients."""
    rows = list(samples)
    start = max(0, int(warmup_samples))
    stop = start + max(0, int(sample_count))
    return rows[start:stop]


def record_samples(*, sample_count: int, timeout_sec: float):
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except ImportError as exc:  # pragma: no cover - exercised on the NX
        raise RuntimeError(f"ROS 2 odometry dependencies unavailable: {exc}") from exc

    samples = []

    class LatencyProbe(Node):
        def __init__(self):
            super().__init__("fastlio_latency_gate")
            # Keep the subscription alive for the full sampling window.  An
            # unreferenced rclpy entity may be garbage-collected immediately,
            # making this fail-closed gate report a nondeterministic 0 frames.
            self._subscription = self.create_subscription(
                Odometry, "/Odometry", self._on_odom, 10
            )

        def _on_odom(self, message):
            source_stamp = (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) / 1e9
            )
            now = self.get_clock().now().nanoseconds / 1e9
            samples.append((source_stamp, now - source_stamp))

    rclpy.init()
    node = LatencyProbe()
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    try:
        while (
            rclpy.ok()
            and len(samples) < int(sample_count)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return samples


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--warmup-samples", type=int, default=20)
    parser.add_argument("--minimum-samples", type=int, default=15)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--max-median-age", type=float, default=0.30)
    parser.add_argument("--max-p95-age", type=float, default=0.35)
    args = parser.parse_args(argv)
    recorded = record_samples(
        sample_count=max(0, args.warmup_samples) + max(0, args.samples),
        timeout_sec=args.timeout,
    )
    samples = _post_warmup_samples(
        recorded,
        warmup_samples=args.warmup_samples,
        sample_count=args.samples,
    )
    decision = evaluate_samples(
        samples,
        minimum_samples=args.minimum_samples,
        max_median_age_sec=args.max_median_age,
        max_p95_age_sec=args.max_p95_age,
    )
    print(json.dumps(decision._asdict(), sort_keys=True))
    return 0 if decision.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
