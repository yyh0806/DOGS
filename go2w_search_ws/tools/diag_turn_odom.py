#!/usr/bin/env python3
"""Correlate Go2-W wheel feedback, commands, and wheel/fused odometry."""

import argparse
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def _yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_delta(start, end):
    return math.atan2(math.sin(end - start), math.cos(end - start))


class Recorder(Node):
    def __init__(self):
        super().__init__("diag_turn_odom")
        self.started = time.monotonic()
        self.wheels = []
        self.poses = {"wheel_odom": [], "odom": []}
        self.commands = {"cmd_vel": [], "cmd_vel_nav": []}
        self.create_subscription(String, "/wheel_feedback", self._wheel, 20)
        self.create_subscription(
            Odometry, "/wheel_odom", lambda msg: self._odom("wheel_odom", msg), 50)
        self.create_subscription(
            Odometry, "/odom", lambda msg: self._odom("odom", msg), 50)
        self.create_subscription(
            Twist, "/cmd_vel", lambda msg: self._cmd("cmd_vel", msg), 20)
        self.create_subscription(
            Twist, "/cmd_vel_nav", lambda msg: self._cmd("cmd_vel_nav", msg), 20)

    def _t(self):
        return time.monotonic() - self.started

    def _wheel(self, message):
        try:
            values = json.loads(message.data).get("wheel_dq")
            if values is not None and len(values) == 4:
                self.wheels.append((self._t(), *(float(value) for value in values)))
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _odom(self, name, message):
        pose = message.pose.pose.position
        twist = message.twist.twist
        self.poses[name].append(
            (self._t(), float(pose.x), float(pose.y), _yaw(message),
             float(twist.linear.x), float(twist.angular.z)))

    def _cmd(self, name, message):
        self.commands[name].append(
            (self._t(), float(message.linear.x), float(message.linear.y),
             float(message.angular.z)))


def _pose_summary(samples):
    if len(samples) < 2:
        return {"sample_count": len(samples)}
    first = samples[0]
    last = samples[-1]
    path_length = sum(
        math.hypot(cur[1] - prev[1], cur[2] - prev[2])
        for prev, cur in zip(samples, samples[1:])
    )
    return {
        "sample_count": len(samples),
        "start": [first[1], first[2], first[3]],
        "end": [last[1], last[2], last[3]],
        "delta_xy": [last[1] - first[1], last[2] - first[2]],
        "displacement": math.hypot(last[1] - first[1], last[2] - first[2]),
        "path_length": path_length,
        "yaw_delta": _angle_delta(first[3], last[3]),
        "linear_twist_range": [min(row[4] for row in samples),
                               max(row[4] for row in samples)],
        "angular_twist_range": [min(row[5] for row in samples),
                                max(row[5] for row in samples)],
    }


def _wheel_summary(samples):
    if not samples:
        return {"sample_count": 0}
    columns = [[row[index] for row in samples] for index in range(1, 5)]
    means = [sum(row[1:]) / 4.0 for row in samples]
    mean_abs = [sum(abs(value) for value in row[1:]) / 4.0 for row in samples]
    return {
        "sample_count": len(samples),
        "per_motor_median": [statistics.median(values) for values in columns],
        "per_motor_range": [[min(values), max(values)] for values in columns],
        "raw_mean_median": statistics.median(means),
        "raw_mean_range": [min(means), max(means)],
        "mean_abs_peak": max(mean_abs),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 30.0:
        raise SystemExit("duration must be in [1, 30] seconds")

    rclpy.init()
    node = Recorder()
    try:
        ready_deadline = time.monotonic() + 5.0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.wheels and all(node.poses.values()):
                break
        if not node.wheels or not all(node.poses.values()):
            raise RuntimeError("missing /wheel_feedback, /wheel_odom, or /odom")
        node.started = time.monotonic()
        node.wheels.clear()
        for samples in node.poses.values():
            samples.clear()
        for samples in node.commands.values():
            samples.clear()
        print("READY", flush=True)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        nonzero_manual = [
            sample for sample in node.commands["cmd_vel"]
            if any(abs(value) > 1e-6 for value in sample[1:])
        ]
        active_window = None
        if nonzero_manual:
            start_t = nonzero_manual[0][0]
            end_t = nonzero_manual[-1][0] + 0.25
            active_window = {
                "start_t": start_t,
                "end_t": end_t,
                "wheels": _wheel_summary([
                    row for row in node.wheels if start_t <= row[0] <= end_t
                ]),
                "wheel_odom": _pose_summary([
                    row for row in node.poses["wheel_odom"]
                    if start_t <= row[0] <= end_t
                ]),
                "odom": _pose_summary([
                    row for row in node.poses["odom"]
                    if start_t <= row[0] <= end_t
                ]),
            }
        print(json.dumps({
            "duration": args.duration,
            "wheels": _wheel_summary(node.wheels),
            "wheel_odom": _pose_summary(node.poses["wheel_odom"]),
            "odom": _pose_summary(node.poses["odom"]),
            "commands": {
                name: {"sample_count": len(samples), "samples": samples}
                for name, samples in node.commands.items()
            },
            "active_window": active_window,
        }, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
