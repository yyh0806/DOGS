"""容器内常驻 ROS2 指令桥 (PC → NX 的 nx_motion_node)。

为什么有它: panel.py 在宿主机跑 (Python3.8, 无 rclpy), 要发 ROS2 话题只能借
go2w_humble 容器。但每次 `docker exec ros2 topic pub` 有 ~200ms 进程启动开销,
20Hz 连续移动喂不动。所以让容器里常驻本进程, panel.py 往它的 stdin 写一行 JSON,
它立刻转成 /cmd_vel (Twist) 或 /cmd_pose (String) 发布, 零额外开销。

数据流:
  panel.py ──stdin(JSON)──> 本进程 ──ROS2话题──> NX nx_motion_node ──SDK──> 狗
  NX nx_motion_node ──/dog_state──> 本进程 ──stdout(JSON)──> panel.py (推前端)

stdin 协议 (每行一条 JSON):
  {"type":"vel","vx":0.4,"vy":0,"vyaw":0.5}   → 发布 /cmd_vel
  {"type":"pose","cmd":"stand"}               → 发布 /cmd_pose (stand/sit/estop)
  {"type":"stop"}                             → 发布 /cmd_vel 零速

stdout 协议 (收到 NX /dog_state 时写一行):
  {"type":"dog_state","state":"STOPPED","vx":0.0,"vy":0.0,"vyaw":0.0}

坐标系: 前端 vyaw 正=左转, 这里只透传, 真正的反转在 nx_motion_node 里做
(Go2W SDK z 正=右转, 需反转)。

运行 (容器内, 由 panel.py 用 subprocess 拉起):
  python3 /workspace/web/cmd_publisher.py
"""
import json
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        # 发布器: /cmd_vel 速度, /cmd_pose 姿态
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pose_pub = self.create_publisher(String, '/cmd_pose', 10)
        # 订阅 NX 发的 /dog_state, 转发给 panel.py
        self.create_subscription(String, '/dog_state', self._on_dog_state, 10)
        self.get_logger().info('cmd_publisher 就绪: 发 /cmd_vel /cmd_pose, 收 /dog_state')

    def _on_dog_state(self, msg):
        """NX 发布的狗状态 → 原样转给 panel.py (stdout)。"""
        try:
            d = json.loads(msg.data)
            out = {'type': 'dog_state',
                   'state': d.get('state', '?'),
                   'vx': d.get('vx', 0.0), 'vy': d.get('vy', 0.0), 'vyaw': d.get('vyaw', 0.0)}
            sys.stdout.write(json.dumps(out) + '\n')
            sys.stdout.flush()
        except Exception:
            pass

    def handle_line(self, line):
        """处理 panel.py 写来的一行 JSON 指令。"""
        line = line.strip()
        if not line:
            return
        try:
            cmd = json.loads(line)
        except Exception as e:
            self.get_logger().warning(f'解析指令失败: {line!r}: {e}')
            return
        t = cmd.get('type')
        if t == 'vel':
            tw = Twist()
            tw.linear.x = float(cmd.get('vx', 0.0))
            tw.linear.y = float(cmd.get('vy', 0.0))
            tw.angular.z = float(cmd.get('vyaw', 0.0))
            self._vel_pub.publish(tw)
        elif t == 'stop':
            self._vel_pub.publish(Twist())  # 全零
        elif t == 'pose':
            s = String()
            s.data = str(cmd.get('cmd', ''))
            self._pose_pub.publish(s)
        else:
            self.get_logger().warning(f'未知指令类型: {t}')


def main():
    rclpy.init()
    node = CmdPublisher()
    # ROS2 spin 放后台线程, 主线程读 stdin (panel.py 关闭 stdin 即退出)
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    try:
        for line in sys.stdin:
            node.handle_line(line)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
