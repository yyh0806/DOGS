#!/usr/bin/env python3
"""Monitor one Panel Nav2 run and stop safely on proximity or no progress."""

import json
import math
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


STATUS_URL = "http://127.0.0.1:8000/api/status"
STOP_URL = "http://127.0.0.1:8000/api/stop"


def api_status():
    with urllib.request.urlopen(STATUS_URL, timeout=1.0) as response:
        return json.load(response)


def stop(reason):
    request = urllib.request.Request(STOP_URL, data=b"{}", method="POST")
    with urllib.request.urlopen(request, timeout=2.0) as response:
        print("SAFETY_STOP", reason, response.status, response.read().decode())


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor_obstacle_nav")
        self.pose = None
        self.front_min = math.inf
        self.safety_front_min = math.inf
        self.front_danger_streak = 0
        self.cmd = (0.0, 0.0)
        self.path = None
        self.trajectory = []
        self.minimum_front = math.inf
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_mid360", self._scan, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd, 10)
        self.create_subscription(Path, "/plan", self._path, 10)

    def _odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if not self.trajectory or math.hypot(
                p.x - self.trajectory[-1][0], p.y - self.trajectory[-1][1]) >= 0.01:
            self.trajectory.append((p.x, p.y))

    def _scan(self, message):
        values = []
        safety_values = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= math.radians(45) and math.isfinite(distance):
                values.append(float(distance))
            if abs(angle) <= math.radians(20) and math.isfinite(distance):
                safety_values.append(float(distance))
        self.front_min = min(values, default=math.inf)
        self.minimum_front = min(self.minimum_front, self.front_min)
        self.safety_front_min = min(safety_values, default=math.inf)
        if self.safety_front_min < 0.45:
            self.front_danger_streak += 1
        else:
            self.front_danger_streak = 0

    def _cmd(self, message):
        self.cmd = (message.linear.x, message.angular.z)

    def _path(self, message):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in message.poses]


def path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def max_line_deviation(points):
    if len(points) < 2:
        return 0.0
    start, end = points[0], points[-1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return 0.0
    return max(
        abs(dy * (x - start[0]) - dx * (y - start[1])) / norm
        for x, y in points
    )


def report_trajectory(node):
    points = node.trajectory
    print(
        "TRAJECTORY",
        "samples", len(points),
        "start", tuple(round(value, 3) for value in points[0]) if points else None,
        "end", tuple(round(value, 3) for value in points[-1]) if points else None,
        "length", round(path_length(points), 3),
        "line_deviation", round(max_line_deviation(points), 3),
        "minimum_front", round(node.minimum_front, 3),
        flush=True,
    )


def main():
    rclpy.init()
    node = Monitor()
    initial_state = api_status().get("point_nav", {})
    initial_generation = initial_state.get("generation")
    print("WAIT_GOAL generation", initial_generation, flush=True)
    wait_deadline = time.monotonic() + 60.0
    active_at = None
    progress_pose = None
    progress_at = None
    last_report = 0.0

    try:
        while time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            state = api_status().get("point_nav", {})
            if state.get("status") in {
                "pending", "waiting_server", "waiting_health", "active"
            } and (
                state.get("generation") != initial_generation
                or initial_state.get("status") in {
                    "pending", "waiting_server", "waiting_health", "active"
                }
            ):
                active_at = time.monotonic()
                progress_at = active_at
                progress_pose = node.pose
                print("GOAL", state, flush=True)
                break
        if active_at is None:
            print("NO_GOAL")
            return

        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            state = api_status().get("point_nav", {})
            if node.pose and progress_pose:
                moved = math.hypot(node.pose[0] - progress_pose[0], node.pose[1] - progress_pose[1])
                if moved >= 0.08:
                    progress_pose = node.pose
                    progress_at = now
            if now - last_report >= 1.0:
                print(
                    "T", round(now - active_at, 1),
                    "status", state.get("status"),
                    "pose", tuple(round(v, 3) for v in node.pose) if node.pose else None,
                    "front", round(node.front_min, 3),
                    "cmd", tuple(round(v, 3) for v in node.cmd),
                    "path_n", len(node.path) if node.path else 0,
                    flush=True,
                )
                last_report = now
            if node.front_danger_streak >= 5:
                stop("central_front_proximity_5_frames")
                report_trajectory(node)
                return
            if now - progress_at > 20.0:
                stop("no_progress_20s")
                report_trajectory(node)
                return
            if now - active_at > 60.0:
                stop("test_timeout_60s")
                report_trajectory(node)
                return
            if state.get("status") in {
                "succeeded", "aborted", "canceled", "failed", "rejected", "timed_out"
            }:
                print("TERMINAL", state, flush=True)
                report_trajectory(node)
                return
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
