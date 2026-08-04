#!/usr/bin/env python3
"""Send the current localization pose to NavigateToPose and audit velocity."""

import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class ZeroMotionAudit(Node):
    def __init__(self):
        super().__init__("diag_nav2_zero_motion")
        self.pose = None
        self.cmd_count = 0
        self.nonzero_count = 0
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.pose_subscription = self.create_subscription(
            Odometry, "/localization_pose", self._on_pose, 10
        )
        self.cmd_subscription = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd, 10
        )
        self.nav_cmd_subscription = self.create_subscription(
            Twist, "/cmd_vel_nav", self._on_cmd, 10
        )

    def _on_pose(self, message):
        self.pose = message

    def _on_cmd(self, message):
        self.cmd_count += 1
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if any(abs(value) > 1e-4 for value in values):
            self.nonzero_count += 1


def wait_for(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not predicate():
        raise TimeoutError("diagnostic timed out")


def main():
    rclpy.init()
    node = ZeroMotionAudit()
    try:
        wait_for(node, lambda: node.pose is not None, 5.0)
        if not node.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/navigate_to_pose action is unavailable")

        current = node.pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose = current.pose.pose
        future = node.client.send_goal_async(goal)
        wait_for(node, future.done, 5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("zero-motion navigation goal was rejected")

        result_future = handle.get_result_async()
        wait_for(node, result_future.done, 15.0)
        wrapped = result_future.result()
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        print(
            f"status={wrapped.status} cmd_messages={node.cmd_count} "
            f"nonzero_cmd_messages={node.nonzero_count}"
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"navigation action status={wrapped.status}")
        if node.nonzero_count:
            raise RuntimeError("zero-motion goal emitted a nonzero velocity")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
