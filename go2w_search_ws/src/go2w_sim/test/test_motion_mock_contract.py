"""Task3 (简化): motion_sdk_mock 契约 pin. 双订阅 + /cmd_vel_nav→/cmd_vel 转发.

完整版加 SportModeState 回报 (mode/velocity/progress) + watchdog timeout pin.
"""
import rclpy
from geometry_msgs.msg import Twist


def test_mock_dual_subscription_and_forward():
    rclpy.init()
    from go2w_sim.nodes.motion_sdk_mock import MotionSdkMock
    node = MotionSdkMock()
    try:
        topics = [t for t, _ in node.get_topic_names_and_types()]
        assert '/cmd_vel' in topics, "mock 未订阅 /cmd_vel"
        assert '/cmd_vel_nav' in topics, "mock 未订阅 /cmd_vel_nav"
        received = []
        node.create_subscription(Twist, '/cmd_vel', lambda m: received.append(m), 10)
        nav_pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
        twist = Twist(); twist.linear.x = 0.5
        for _ in range(5):
            nav_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) >= 1, "mock 未转发 /cmd_vel_nav → /cmd_vel"
        assert abs(received[-1].linear.x - 0.5) < 1e-6, "转发值不对"
    finally:
        node.destroy_node()
        rclpy.shutdown()
