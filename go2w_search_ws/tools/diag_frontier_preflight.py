#!/usr/bin/env python3
"""Audit live frontier candidates through Nav2 without sending motion goals."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _wait_future(node, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result() if future.done() else None


def _path_length(path):
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _cost_at(costmap, x, y):
    metadata = costmap.metadata
    dx = float(x) - float(metadata.origin.position.x)
    dy = float(y) - float(metadata.origin.position.y)
    angle = -_yaw(metadata.origin.orientation)
    local_x = math.cos(angle) * dx - math.sin(angle) * dy
    local_y = math.sin(angle) * dx + math.cos(angle) * dy
    cell_x = int(math.floor(local_x / float(metadata.resolution)))
    cell_y = int(math.floor(local_y / float(metadata.resolution)))
    if not (0 <= cell_x < metadata.size_x and 0 <= cell_y < metadata.size_y):
        return None
    return int(costmap.data[cell_y * metadata.size_x + cell_x])


class FrontierAudit(Node):
    def __init__(self):
        super().__init__("diag_frontier_preflight")
        self.map = None
        self.pose = None
        self.costmap = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map_frontier", self._map, map_qos)
        self.create_subscription(Odometry, "/localization_pose", self._pose, 10)
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self._costmap, costmap_qos
        )
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def _map(self, message):
        self.map = message

    def _pose(self, message):
        self.pose = message

    def _costmap(self, message):
        self.costmap = message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=6.0)
    parser.add_argument("--limit", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--web-dir", default="/home/nx/go2w_ws/web")
    args = parser.parse_args()
    sys.path.insert(0, args.web_dir)
    from nx_room_orchestrator import select_frontier_candidates

    rclpy.init()
    node = FrontierAudit()
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.map is not None and node.pose is not None and node.costmap is not None:
                break
        if node.map is None or node.pose is None or node.costmap is None:
            raise RuntimeError("timed out waiting for map, pose, or global costmap")
        if not node.planner.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("/compute_path_to_pose unavailable")

        position = node.pose.pose.pose.position
        robot_yaw = _yaw(node.pose.pose.pose.orientation)
        origin = (float(position.x), float(position.y), robot_yaw)
        candidates = select_frontier_candidates(
            node.map,
            origin,
            [],
            origin_pose=origin,
            max_radius=args.radius,
        )
        print(
            f"pose=({position.x:.3f},{position.y:.3f},{robot_yaw:.3f}) "
            f"radius={args.radius:.2f} candidates={len(candidates)}"
        )
        reachable = 0
        for index, candidate in enumerate(candidates[: max(0, args.limit)]):
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = "map"
            goal.goal.header.stamp = node.get_clock().now().to_msg()
            goal.goal.pose.position.x = float(candidate["x"])
            goal.goal.pose.position.y = float(candidate["y"])
            goal.goal.pose.orientation.z = math.sin(float(candidate["yaw"]) / 2.0)
            goal.goal.pose.orientation.w = math.cos(float(candidate["yaw"]) / 2.0)
            handle = _wait_future(
                node, node.planner.send_goal_async(goal), args.timeout
            )
            status = "rejected"
            poses = 0
            length = None
            collisions = None
            max_cost = None
            if handle is not None and handle.accepted:
                wrapped = _wait_future(node, handle.get_result_async(), args.timeout)
                if wrapped is None:
                    status = "timeout"
                elif wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                    status = "reachable"
                    path = wrapped.result.path
                    poses = len(path.poses)
                    length = _path_length(path)
                    costs = [
                        _cost_at(node.costmap, pose.pose.position.x, pose.pose.position.y)
                        for pose in path.poses
                    ]
                    costs = [cost for cost in costs if cost is not None]
                    collisions = sum(253 <= cost <= 254 for cost in costs)
                    max_cost = max(costs, default=None)
                    if collisions == 0:
                        reachable += 1
                else:
                    status = f"status_{wrapped.status}"
            print(
                f"candidate[{index}] xy=({candidate['x']:.3f},{candidate['y']:.3f}) "
                f"score={candidate['score']:.3f} size={candidate['size']} "
                f"plan={status} poses={poses} length_m={length} "
                f"collision_samples={collisions} max_cost={max_cost}"
            )
        print(f"audited={min(len(candidates), max(0, args.limit))} reachable_safe={reachable}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
