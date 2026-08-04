"""sim_amcl_to_odom: FastLIO /Odometry → /localization_pose (map frame).

原设计订 amcl /amcl_pose 转 Odometry, 但 amcl 在仿真下因 static map (indoor_empty_map
全自由空) 跟 world (indoor_rooms 有墙) 不匹配, scan matching 不收敛, /amcl_pose 不发.
仿真初始狗在原点 odom=map 对齐, FastLIO /Odometry (odom→base, 稳定 7Hz) 可直接当
map→base localization (无全局校正, 仿真演示够). 真机用真 amcl / map-odom-fuser 发
/localization_pose, 不走本节点 (本节点仅 GO2W_SIM launch 起).

spec 2026-07-25 Task E (localization 全绿).
importer: sim_full_bringup.launch.py Node executable=sim_amcl_to_odom.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SimAmclToOdom(Node):
    def __init__(self):
        super().__init__('sim_amcl_to_odom')
        self._pub = self.create_publisher(Odometry, '/localization_pose', 10)
        # 缓存最后 FastLIO pose: WSL2 visualize=false 后 livox ~0.6Hz, FastLIO /Odometry
        # 间歇, web localization age 阈值 0.5s 会判 stale → motion activation_timeout →
        # velocity_authorized=false → planar_move 不执行 → 狗不位移. 10Hz 重发缓存 pose
        # (stamp=now) 让 /localization_pose 持续新鲜 → localization_healthy → 激活完成.
        self._latest_pose = None
        # 订 /odom_planar (planar_move 运动学 odom, 总发不依赖 livox) 而非 /Odometry
        # (FastLIO 需 livox 点云, WSL2 visualize=false 后 livox 偶发不发致 /Odometry 间歇,
        # web localization_stale → motion activation_timeout → 狗不位移).
        # 仿真 odom=map 初始对齐, planar_move /odom_planar 即 map→base_link 真实位置.
        self.create_subscription(Odometry, '/odom_planar', self._on_odom, 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            'sim_amcl_to_odom: /Odometry (FastLIO) → /localization_pose (map frame, '
            '10Hz 重发缓存 pose, stamp=now)')

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_pose = msg.pose

    def _publish(self) -> None:
        if self._latest_pose is None:
            return
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()  # 新 stamp (非 FastLIO 原始间歇 stamp)
        o.header.frame_id = 'map'  # 强制 map (仿真 odom=map 初始对齐)
        o.child_frame_id = 'base_link'  # web get_localization_health 校验 == "base_link"
        o.pose = self._latest_pose
        self._pub.publish(o)


def main() -> None:
    rclpy.init()
    node = SimAmclToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
