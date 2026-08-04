"""M1a: teleop full-speed into +x wall; kinematic-base must stop before wall.

Walls at x=+-5, y=+-5. planar_move respects Gazebo collision, so the robot
cannot penetrate — final pose must stay within |x|<4.9, |y|<4.9.
"""
import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def test_teleop_into_wall_does_not_penetrate(sim_spawn_only_session):
    node = sim_spawn_only_session
    odom_pos = []
    node.create_subscription(
        Odometry, '/odom_planar',
        lambda m: odom_pos.append((m.pose.pose.position.x,
                                   m.pose.pose.position.y)), 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.8  # full speed toward +x wall (x=5)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(odom_pos) > 5, (
        "no /odom_planar received — planar_move plugin not attached?")
    final_x = odom_pos[-1][0]
    final_y = odom_pos[-1][1]
    assert abs(final_x) < 4.9, f"penetrated x wall: x={final_x}"
    assert abs(final_y) < 4.9, f"penetrated y wall: y={final_y}"
    # And it must actually have moved (not stuck at origin)
    assert odom_pos[-1][0] > 0.5, (
        f"robot did not move forward (final x={final_x}) — cmd_vel ignored?")
