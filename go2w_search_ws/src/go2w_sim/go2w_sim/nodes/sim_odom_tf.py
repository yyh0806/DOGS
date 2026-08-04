"""sim_odom_tf: TF map→odom (identity, 10Hz 定时广播).

仿真 odom=map 初始对齐 + FastLIO 无全局漂移, map→odom 用 identity 等效真机
amcl/map-odom-fuser 的 map→odom 修正 (静态空地图 amcl 不收敛).

odom→base_footprint 由 planar_move (libgazebo_ros_planar_move) 独占发布 ——
sim_odom_tf 不再发该段: 去 FASTRTPS profile 后 planar_move 恢复正常发 TF,
若 sim_odom_tf 同时发会触发 TF_OLD_DATA (两 authority stamp 不一致) →
p2l target_frame=base_link 查 TF 丢帧 → /scan 仅 0.4Hz → nav_scan_stale.

10Hz 定时广播 (不依赖 /Odometry): map→odom identity 不变, 低频够用,
避免 FastLIO 7Hz stamp 与 planar_move stamp 的链式时序耦合.

spec 2026-07-25 Task4 Nav2 集成 + Task E map frame + TF 冲突修复.
"""
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SimOdomTf(Node):
    def __init__(self):
        super().__init__('sim_odom_tf')
        self._br = TransformBroadcaster(self)
        # 10Hz 发 map→odom identity (不依赖 /Odometry, 避免与 planar_move stamp 冲突)
        self.create_timer(0.1, self._publish_map_odom)
        self.get_logger().info(
            'sim_odom_tf: map→odom identity (10Hz); '
            'odom→base_footprint 由 planar_move 独占')

    def _publish_map_odom(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        # identity: translation 全零 (默认), rotation 必须设 w=1 (默认 w=0 是无效四元数,
        # TF 会拒绝/警告). Point/Quaternion 不能整体赋值 (类型断言), 逐字段设.
        t.transform.rotation.w = 1.0
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = SimOdomTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
