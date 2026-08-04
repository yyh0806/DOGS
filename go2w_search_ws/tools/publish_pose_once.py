#!/usr/bin/env python3
"""Publish one validated maintenance pose command without shell YAML quoting."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COMMANDS = (
    "stand",
    "confirm_stand",
    "adopt_stand",
    "adopt_balance",
    "balance",
    "confirm_balance",
    "sit",
    "estop",
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=COMMANDS)
    args = parser.parse_args()

    rclpy.init()
    node = Node("publish_pose_once")
    publisher = node.create_publisher(String, "/cmd_pose", 10)
    deadline = time.monotonic() + 2.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("/cmd_pose has no subscriber")

    message = String()
    message.data = args.command
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.25)
    print(f"published {args.command}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
