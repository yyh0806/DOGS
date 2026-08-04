"""NX 本机闭环桥接节点 (阶段1)。

在"我们的NX"上本机运行, 替代跨热点的 cmd_publisher + ros_to_json 方案。
职责:
  1. 本机订阅 /dog_state (nx_motion_node 发) → 写 dog_state.json (供本机panel读)
  2. 提供 HTTP API 接收前端指令 → 本机发 /cmd_vel /cmd_pose
  3. 这样 NX 本机闭环, 不依赖热点, PC 断网狗也能控

架构 (NX本机, 全部ROS2本机通信):
  浏览器(PC/手机) ─HTTP→ 本节点 ─/cmd_vel─> nx_motion_node ─SDK─> 狗
  nx_motion_node ─/dog_state─> 本节点 ─HTTP/WS─> 浏览器

运行 (NX上, 取代PC的panel.py):
  source /opt/ros/humble/setup.bash
  ros2 run go2w_bridge nx_panel_bridge
  # 或裸python: python3 nx_panel_bridge.py
  浏览器开 http://<NX_IP>:8000

注意: 这是阶段1的本机版, PC端panel.py保持不变(调试用)。
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NxPanelBridge(Node):
    """NX本机桥: HTTP API ↔ ROS2 /cmd_vel /cmd_pose, 并转发 /dog_state。"""

    def __init__(self):
        super().__init__('nx_panel_bridge')
        # 本机发布器 (直接到 nx_motion_node, 不跨网)
        self._vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pose_pub = self.create_publisher(String, '/cmd_pose', 10)
        # 订阅狗状态 (nx_motion_node 发)
        self._latest_state = {'state': 'UNKNOWN', 'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}
        self._state_lock = threading.Lock()
        self.create_subscription(String, '/dog_state', self._on_dog_state, 10)
        self.get_logger().info('nx_panel_bridge 就绪 (本机 /cmd_vel /cmd_pose + /dog_state)')

    def _on_dog_state(self, msg):
        try:
            d = json.loads(msg.data)
            with self._state_lock:
                self._latest_state = d
        except Exception:
            pass

    def send_vel(self, vx, vy, vyaw):
        """发速度指令 (本机 /cmd_vel)。"""
        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.angular.z = float(vyaw)
        self._vel_pub.publish(tw)

    def send_stop(self):
        self._vel_pub.publish(Twist())

    def send_pose(self, cmd):
        """发姿态指令 stand/sit/estop (本机 /cmd_pose)。"""
        s = String()
        s.data = str(cmd)
        self._pose_pub.publish(s)

    def get_state(self):
        with self._state_lock:
            return dict(self._latest_state)


def main(args=None):
    rclpy.init(args=args)
    node = NxPanelBridge()
    # ROS2 spin 后台
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    # 轻量 HTTP 服务 (内联, 避免依赖 flask)
    from http.server import HTTPServer, BaseHTTPRequestHandler
    from urllib.parse import urlparse, parse_qs
    import os

    static_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'web', 'static')

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            p = urlparse(self.path)
            if p.path in ('/', '/index.html', '/panel.html'):
                f = os.path.join(static_dir, 'panel.html')
                self._serve(f, 'text/html')
            elif p.path == '/map.js':
                self._serve(os.path.join(static_dir, 'map.js'), 'application/javascript')
            elif p.path == '/api/status':
                self._json({'connected': True, 'dog_state': node.get_state()})
            else:
                self.send_error(404)

        def do_POST(self):
            p = urlparse(self.path); q = parse_qs(p.query)
            if p.path == '/api/stand':
                node.send_pose('stand'); self._json({'ok': True})
            elif p.path == '/api/sit':
                node.send_pose('sit'); self._json({'ok': True})
            elif p.path == '/api/e_stop':
                node.send_pose('estop'); self._json({'ok': True})
            elif p.path == '/api/stop':
                node.send_stop(); self._json({'ok': True})
            elif p.path == '/api/move':
                node.send_vel(float(q.get('vx', ['0'])[0]),
                              float(q.get('vy', ['0'])[0]),
                              float(q.get('vyaw', ['0'])[0]))
                self._json({'ok': True})
            else:
                self.send_error(404)

        def _json(self, d):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(d, ensure_ascii=False).encode())

        def _serve(self, path, ct):
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', ct)
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)

        def log_message(self, *a):
            pass

    port = int(os.environ.get('GO2W_PORT', '8000'))
    srv = HTTPServer(('0.0.0.0', port), H)
    print(f'NX本机panel: http://0.0.0.0:{port}  (本机ROS2闭环, 不依赖热点)')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
