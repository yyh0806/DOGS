#!/usr/bin/env python3
"""Measure a ROS 2 topic with one persistent DDS participant."""

from __future__ import annotations

import argparse
import json
import math
import time


def evaluate_receive_times(times, *, minimum_hz: float, minimum_samples: int):
    rows = [float(value) for value in times]
    if len(rows) < int(minimum_samples):
        return False, None, "insufficient_samples"
    duration = rows[-1] - rows[0]
    if not math.isfinite(duration) or duration <= 0.0:
        return False, None, "invalid_duration"
    rate = (len(rows) - 1) / duration
    if not math.isfinite(rate) or rate < float(minimum_hz):
        return False, rate, "rate_below_minimum"
    return True, rate, None


def record(topic: str, type_name: str, *, samples: int, timeout_sec: float):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    message_type = get_message(type_name)
    receive_times = []

    class Probe(Node):
        def __init__(self):
            super().__init__("go2w_topic_rate_gate")
            qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE
            )
            self._subscription = self.create_subscription(
                message_type, topic, self._on_message, qos
            )

        def _on_message(self, _message):
            receive_times.append(time.monotonic())

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    try:
        while len(receive_times) < int(samples) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return receive_times


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--type", required=True, dest="type_name")
    parser.add_argument("--minimum-hz", required=True, type=float)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--minimum-samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.samples < 2 or args.minimum_samples < 2:
        parser.error("sample counts must be at least 2")
    times = record(
        args.topic, args.type_name,
        samples=args.samples, timeout_sec=args.timeout,
    )
    passed, rate, failure = evaluate_receive_times(
        times,
        minimum_hz=args.minimum_hz,
        minimum_samples=args.minimum_samples,
    )
    print(json.dumps({
        "failure": failure,
        "passed": passed,
        "rate_hz": rate,
        "sample_count": len(times),
        "topic": args.topic,
    }, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
