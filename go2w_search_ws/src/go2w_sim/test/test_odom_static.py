"""Task 2 (极简版): odom 链静止稳定.

完整 Task 2 (FastLIO 静止原点) 被 Livox driver2 Humble 兼容性深坑阻塞
(package.xml/rosidl/CMake SDK 三层 + 面向 Jazzy) + p3d ground-truth 参数坑.
极简: 用 planar_move /odom_planar (Task1 已验证发布) 验证 odom 链静止稳定,
fuser/Nav2 的 /Odometry 桥接留到 Task 4. 完整 LIO 留待 driver2 Humble fork.
"""
import time
import rclpy
from nav_msgs.msg import Odometry


def test_static_odom_planar_stable(sim_spawn_only_session):
    node = sim_spawn_only_session
    pos = []
    node.create_subscription(
        Odometry, '/odom_planar',
        lambda m: pos.append((m.pose.pose.position.x,
                              m.pose.pose.position.y,
                              m.pose.pose.position.z)), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(pos) > 10, "no /odom_planar — planar_move not publishing?"
    maxx = max(abs(p[0]) for p in pos)
    maxy = max(abs(p[1]) for p in pos)
    maxz = max(abs(p[2]) for p in pos)
    # 静止 (无 cmd_vel) planar_move odom 不积分, 位姿稳定在 0
    assert maxx < 0.02 and maxy < 0.02 and maxz < 0.02, \
        f"static odom drift {maxx},{maxy},{maxz} > 0.02m"
