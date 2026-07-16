#!/usr/bin/env python3
"""Nav2 acceptance-gate evaluator and read-only report scaffold.

The default mode never publishes a goal.  On the NX, topic capture can fill
the same JSON fields before/after a separately authorized staged test.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Mapping


DEFAULT_GATES = {
    "max_plan_latency_ms": 1000.0,
    "max_time_to_first_cmd_ms": 600.0,
    "min_obstacle_clearance_m": 0.18,
    "min_measured_displacement_m": 0.10,
}


@dataclass(frozen=True)
class BenchmarkDecision:
    passed: bool
    failures: tuple[str, ...]


class BenchmarkRecorder:
    """Derive acceptance metrics from subscriptions without any publisher."""

    def __init__(self, *, started_at=None):
        self.started_at = float(
            time.monotonic() if started_at is None else started_at)
        self.action_started_at = None
        self.plan_at = None
        self.first_cmd_at = None
        self.first_pose = None
        self.last_pose = None
        self.min_clearance = None
        self.last_dog_state = None
        self.terminal_status = None
        self.samples = {
            "action_status": 0,
            "plan": 0,
            "cmd_vel_nav": 0,
            "localization_pose": 0,
            "local_costmap": 0,
            "global_costmap": 0,
            "scan_mid360": 0,
            "dog_state": 0,
        }

    @staticmethod
    def _stamp(value):
        stamp = float(value)
        if not math.isfinite(stamp):
            raise ValueError("event stamp must be finite")
        return stamp

    def observe_action_status(self, stamp, status):
        stamp = self._stamp(stamp)
        normalized = str(status).strip().lower()
        self.samples["action_status"] += 1
        if normalized in {"accepted", "executing"} and self.action_started_at is None:
            self.action_started_at = stamp
        if normalized in {"succeeded", "aborted", "canceled", "cancelled"}:
            self.terminal_status = normalized

    def observe_plan(self, stamp, *, poses=0):
        del poses
        stamp = self._stamp(stamp)
        self.samples["plan"] += 1
        if self.plan_at is None:
            self.plan_at = stamp

    def observe_cmd_vel(self, stamp, *, vx, vy, vyaw):
        stamp = self._stamp(stamp)
        velocity = tuple(float(value) for value in (vx, vy, vyaw))
        if not all(math.isfinite(value) for value in velocity):
            return
        self.samples["cmd_vel_nav"] += 1
        if self.first_cmd_at is None and any(abs(value) > 1e-6 for value in velocity):
            self.first_cmd_at = stamp

    def observe_pose(self, stamp, *, x, y):
        stamp = self._stamp(stamp)
        point = (float(x), float(y))
        if not all(math.isfinite(value) for value in point):
            return
        self.samples["localization_pose"] += 1
        if self.first_pose is None:
            self.first_pose = (stamp, point)
        self.last_pose = (stamp, point)

    def observe_scan(self, stamp, ranges):
        self._stamp(stamp)
        finite = []
        for raw in ranges:
            try:
                value = float(raw)
            except (TypeError, ValueError, OverflowError):
                continue
            if math.isfinite(value) and value > 0.0:
                finite.append(value)
        self.samples["scan_mid360"] += 1
        if finite:
            nearest = min(finite)
            self.min_clearance = (
                nearest if self.min_clearance is None
                else min(self.min_clearance, nearest))

    def observe_costmap(self, kind, stamp):
        self._stamp(stamp)
        key = f"{str(kind).strip().lower()}_costmap"
        if key in self.samples:
            self.samples[key] += 1

    def observe_dog_state(self, stamp, state):
        self._stamp(stamp)
        if isinstance(state, Mapping):
            self.last_dog_state = dict(state)
            self.samples["dog_state"] += 1

    def report(self) -> dict:
        baseline = self.action_started_at
        plan_latency = (
            None if baseline is None or self.plan_at is None
            else round((self.plan_at - baseline) * 1000.0, 3))
        first_cmd_latency = (
            None if baseline is None or self.first_cmd_at is None
            else round((self.first_cmd_at - baseline) * 1000.0, 3))
        displacement = None
        if self.first_pose is not None and self.last_pose is not None:
            displacement = round(math.hypot(
                self.last_pose[1][0] - self.first_pose[1][0],
                self.last_pose[1][1] - self.first_pose[1][1],
            ), 4)
        dog_state = self.last_dog_state or {}
        parked = (
            str(dog_state.get("session", "")).upper() == "PARKED"
            and str(dog_state.get("actual_motion", "")).upper() == "STOPPED"
            and self.terminal_status == "succeeded"
        )
        report = {
            "schema_version": 1,
            "captured_at": time.time(),
            "execute": False,
            "goal_published": False,
            "topics": list(read_only_template()["topics"]),
            "plan_latency_ms": plan_latency,
            "time_to_first_cmd_ms": first_cmd_latency,
            "min_obstacle_clearance_m": self.min_clearance,
            "terminal_parked": parked,
            "measured_displacement_m": displacement,
            "action_terminal_status": self.terminal_status,
            "samples": dict(self.samples),
        }
        decision = evaluate_report(report)
        report["acceptance"] = {
            "passed": decision.passed,
            "failures": list(decision.failures),
        }
        return report


def _finite(report: Mapping[str, object], name: str) -> float:
    try:
        value = float(report[name])
    except (KeyError, TypeError, ValueError, OverflowError):
        return float("nan")
    return value if math.isfinite(value) else float("nan")


def evaluate_report(report: Mapping[str, object], gates=None) -> BenchmarkDecision:
    limits = {**DEFAULT_GATES, **dict(gates or {})}
    failures = []
    if not _finite(report, "plan_latency_ms") <= limits["max_plan_latency_ms"]:
        failures.append("plan_latency")
    if not _finite(report, "time_to_first_cmd_ms") <= limits["max_time_to_first_cmd_ms"]:
        failures.append("first_command_latency")
    if not _finite(report, "min_obstacle_clearance_m") >= limits["min_obstacle_clearance_m"]:
        failures.append("obstacle_clearance")
    if report.get("terminal_parked") is not True:
        failures.append("terminal_not_parked")
    if not _finite(report, "measured_displacement_m") >= limits["min_measured_displacement_m"]:
        failures.append("insufficient_displacement")
    return BenchmarkDecision(not failures, tuple(failures))


def read_only_template() -> dict:
    return {
        "schema_version": 1,
        "captured_at": time.time(),
        "execute": False,
        "goal_published": False,
        "topics": [
            "/navigate_to_pose/_action/status", "/plan", "/cmd_vel_nav",
            "/localization_pose",
            "/local_costmap/costmap", "/global_costmap/costmap",
            "/scan_mid360", "/dog_state",
        ],
        "plan_latency_ms": None,
        "time_to_first_cmd_ms": None,
        "min_obstacle_clearance_m": None,
        "terminal_parked": None,
        "measured_displacement_m": None,
        "acceptance": {"passed": False, "failures": ["not_executed"]},
    }


def record_ros(duration_sec: float) -> dict:
    """Subscribe for a bounded interval; intentionally create no publisher."""

    try:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 recorder dependencies unavailable: {exc}") from exc

    recorder = BenchmarkRecorder()
    status_names = {
        1: "accepted", 2: "executing", 4: "succeeded",
        5: "canceled", 6: "aborted",
    }

    class RecorderNode(Node):
        def __init__(self):
            super().__init__("go2w_nav2_benchmark_recorder")
            self.create_subscription(
                GoalStatusArray, "/navigate_to_pose/_action/status",
                self.on_status, 10)
            self.create_subscription(Path, "/plan", self.on_plan, 10)
            self.create_subscription(Twist, "/cmd_vel_nav", self.on_cmd, 20)
            self.create_subscription(
                Odometry, "/localization_pose", self.on_pose, 20)
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap",
                lambda _msg: recorder.observe_costmap(
                    "local", time.monotonic()), 10)
            self.create_subscription(
                OccupancyGrid, "/global_costmap/costmap",
                lambda _msg: recorder.observe_costmap(
                    "global", time.monotonic()), 10)
            self.create_subscription(
                LaserScan, "/scan_mid360", self.on_scan,
                qos_profile_sensor_data)
            self.create_subscription(String, "/dog_state", self.on_dog, 20)

        def on_status(self, message):
            now = time.monotonic()
            for item in message.status_list:
                name = status_names.get(int(item.status))
                if name:
                    recorder.observe_action_status(now, name)

        def on_plan(self, message):
            recorder.observe_plan(time.monotonic(), poses=len(message.poses))

        def on_cmd(self, message):
            recorder.observe_cmd_vel(
                time.monotonic(), vx=message.linear.x,
                vy=message.linear.y, vyaw=message.angular.z)

        def on_pose(self, message):
            position = message.pose.pose.position
            recorder.observe_pose(
                time.monotonic(), x=position.x, y=position.y)

        def on_scan(self, message):
            recorder.observe_scan(time.monotonic(), message.ranges)

        def on_dog(self, message):
            try:
                payload = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                return
            recorder.observe_dog_state(time.monotonic(), payload)

    rclpy.init()
    node = RecorderNode()
    deadline = time.monotonic() + max(0.1, float(duration_sec))
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(
                0.1, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return recorder.report()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="nav2_benchmark_report.json")
    parser.add_argument(
        "--record", action="store_true",
        help="subscribe read-only to Nav2/safety topics for a bounded interval")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument(
        "--execute", action="store_true",
        help="reserved for an explicitly authorized staged NX run")
    args = parser.parse_args(argv)
    if args.execute:
        parser.error(
            "goal execution is intentionally delegated to the authenticated "
            "navigation API; run this recorder alongside that staged request")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = record_ros(args.duration) if args.record else read_only_template()
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
