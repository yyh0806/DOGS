#!/usr/bin/env python3
"""Request a Nav2 path without executing motion and audit obstacle avoidance."""

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class PlanAudit(Node):
    def __init__(self):
        super().__init__("diag_nav2_plan")
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.costmap = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.on_costmap, qos
        )

    def on_costmap(self, message):
        self.costmap = message


def wait_future(node, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("Nav2 future timed out")
    return future.result()


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
    return max(abs(dy * (x - start[0]) - dx * (y - start[1])) / norm
               for x, y in points)


def cost_at(costmap, x, y):
    metadata = costmap.metadata
    resolution = metadata.resolution
    cell_x = int(math.floor((x - metadata.origin.position.x) / resolution))
    cell_y = int(math.floor((y - metadata.origin.position.y) / resolution))
    if not 0 <= cell_x < metadata.size_x or not 0 <= cell_y < metadata.size_y:
        return None
    return int(costmap.data[cell_y * metadata.size_x + cell_x])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = PlanAudit()
    if not node.client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("/compute_path_to_pose action is unavailable")

    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = args.x
    goal.goal.pose.position.y = args.y
    goal.goal.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.goal.pose.orientation.w = math.cos(args.yaw / 2.0)
    goal.use_start = False

    handle = wait_future(node, node.client.send_goal_async(goal), args.timeout)
    if handle is None or not handle.accepted:
        raise RuntimeError("planning goal was rejected")
    wrapped = wait_future(node, handle.get_result_async(), args.timeout)
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f"planning failed with action status={wrapped.status}")

    points = [
        (pose.pose.position.x, pose.pose.position.y)
        for pose in wrapped.result.path.poses
    ]
    if len(points) < 2:
        raise RuntimeError(f"planner returned only {len(points)} path poses")

    deadline = time.monotonic() + 2.0
    while node.costmap is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    costs = [cost_at(node.costmap, x, y) for x, y in points] if node.costmap else []
    known_costs = [cost for cost in costs if cost is not None]
    # Nav2 cost encoding: 253=inscribed, 254=lethal, 255=unknown.  Unknown is
    # traversable here because the rolling global planner intentionally uses
    # allow_unknown while FAST_LIO maps new space.
    collision = sum(253 <= cost <= 254 for cost in known_costs)
    unknown = sum(cost == 255 for cost in known_costs)
    print(f"status=SUCCEEDED poses={len(points)}")
    print(
        f"start={points[0]} goal={points[-1]} "
        f"length_m={path_length(points):.3f} "
        f"max_line_deviation_m={max_line_deviation(points):.3f}"
    )
    print(
        f"cost_samples={len(known_costs)} collision_path_samples={collision} "
        f"unknown_path_samples={unknown} "
        f"max_path_cost={max(known_costs, default=None)}"
    )
    if collision:
        collision_points = [
            (index, round(points[index][0], 3), round(points[index][1], 3), cost)
            for index, cost in enumerate(costs)
            if cost is not None and 253 <= cost <= 254
        ]
        print(f"collision_points={collision_points}")
        raise RuntimeError("computed path intersects lethal costmap cells")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
