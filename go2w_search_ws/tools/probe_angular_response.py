#!/usr/bin/env python3
"""Brief, bounded Go2W motion-response probe with an unconditional zero command."""

import argparse
import json
import math
import time

from nav2_preflight import collect_ros_probes, evaluate_status, load_status

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("probe_angular_response")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state = None
        self.yaw = None
        self.samples = []
        self.feedback = None
        self.mode_samples = []
        self.create_subscription(String, "/dog_state", self._state, 10)
        self.create_subscription(String, "/wheel_feedback", self._feedback, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _state(self, message):
        self.state = json.loads(message.data)
        wheel_dq = self.state.get("wheel_dq")
        if wheel_dq:
            self.samples.append((time.monotonic(), tuple(float(v) for v in wheel_dq)))

    def _feedback(self, message):
        self.feedback = json.loads(message.data)
        wheel_dq = self.feedback.get("wheel_dq")
        if wheel_dq:
            self.samples.append((time.monotonic(), tuple(float(v) for v in wheel_dq)))
        self.mode_samples.append((time.monotonic(), self.feedback.get("sport_mode")))

    def _odom(self, message):
        q = message.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def publish(self, angular, linear_x=0.0):
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular
        self.publisher.publish(message)


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def angle_delta(start, end):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("angular", type=float, nargs="?", default=0.0)
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.7)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--status-url", default="http://127.0.0.1:8000/api/status")
    args = parser.parse_args()
    if not args.execute:
        raise SystemExit(
            "refusing to publish: re-run with --execute after clearing the area")
    if (args.angular == 0.0) == (args.linear_x == 0.0):
        raise SystemExit("specify exactly one nonzero angular or --linear-x command")
    if args.linear_x != 0.0:
        if not (0.0 < abs(args.linear_x) <= 0.20):
            raise SystemExit("linear-x magnitude must be in (0, 0.20]")
        if not (0.1 <= args.duration <= 0.2):
            raise SystemExit("linear probe duration must be in [0.1, 0.2] seconds")
        if not abs(args.linear_x) * args.duration <= 0.04:
            raise SystemExit("linear probe nominal displacement limit is 0.04 m")
    else:
        if not (0.0 < abs(args.angular) <= 0.15):
            raise SystemExit("angular magnitude must be in (0, 0.15]")
        if not (0.1 <= args.duration <= 5.5):
            raise SystemExit("angular probe duration must be in [0.1, 5.5] seconds")

    preflight = evaluate_status(
        load_status(args.status_url, None, 4.0), collect_ros_probes(4.0))
    if not preflight["ok"]:
        raise SystemExit(
            "preflight failed; no command sent:\n" +
            json.dumps(preflight, ensure_ascii=False, indent=2))

    rclpy.init()
    node = Probe()
    try:
        if not spin_until(
                node,
                lambda: node.state is not None and node.feedback is not None and node.yaw is not None,
                5.0):
            raise RuntimeError("missing /dog_state, /wheel_feedback or /odom")
        initial = node.state
        if initial.get("state") != "STOPPED" or initial.get("drive_fault") is not None:
            raise RuntimeError(f"motion not ready: {initial}")
        initial_yaw = node.yaw
        node.samples.clear()
        node.mode_samples.clear()
        started = time.monotonic()
        response_since = None
        while True:
            remaining = args.duration - (time.monotonic() - started)
            if remaining <= 0.0:
                break
            node.publish(args.angular, args.linear_x)
            rclpy.spin_once(node, timeout_sec=min(0.01, remaining))
            feedback_wheels = (node.feedback or {}).get("wheel_dq") or ()
            mean_wheel_speed = (
                sum(abs(float(value)) for value in feedback_wheels) / len(feedback_wheels)
                if feedback_wheels else 0.0
            )
            if mean_wheel_speed >= 0.2:
                response_since = response_since or time.monotonic()
                if time.monotonic() - response_since >= 0.15:
                    break
            else:
                response_since = None
            if node.state and (
                node.state.get("drive_fault") is not None
                or node.state.get("state") not in {"MOVING", "STOPPED"}
            ):
                break
        active_duration = time.monotonic() - started
    finally:
        # Repeated zero commands make the stop independent of a single DDS packet.
        stop_deadline = time.monotonic() + 1.5
        while time.monotonic() < stop_deadline:
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

    final_yaw = node.yaw
    means = [sum(abs(v) for v in values) / 4.0 for _, values in node.samples]
    modes = []
    for _, mode in node.mode_samples:
        if not modes or modes[-1] != mode:
            modes.append(mode)
    final_wheels = (node.feedback or {}).get("wheel_dq") or ()
    final_mean_wheel_speed = (
        sum(abs(float(value)) for value in final_wheels) / len(final_wheels)
        if len(final_wheels) == 4 else None)
    final_joint_lock = (
        (node.feedback or {}).get("sport_mode") == 6
        and final_mean_wheel_speed is not None
        and final_mean_wheel_speed < 0.15)
    print(json.dumps({
        "command_angular": args.angular,
        "command_linear_x": args.linear_x,
        "command_duration_limit": args.duration,
        "command_duration_actual": active_duration,
        "yaw_delta": angle_delta(initial_yaw, final_yaw),
        "peak_mean_abs_wheel_dq": max(means, default=0.0),
        "sport_mode_sequence": modes,
        "final_joint_lock": final_joint_lock,
        "final_mean_abs_wheel_dq": final_mean_wheel_speed,
        "final_wheel_feedback": node.feedback,
        "final_state": node.state,
        "sample_count": len(means),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
