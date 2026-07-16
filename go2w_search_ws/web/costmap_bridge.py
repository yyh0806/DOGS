#!/usr/bin/env python3
"""Bridge Nav2's global costmap to the frontend obstacle overlay.

为什么独立 (不在 nx_web_server 订阅):
  nx_web_server 的 rclpy 节点订阅数已临界 wait_set 上限, 加 OccupancyGrid 订阅触发
  "IndexError: wait set index too big" → spin 崩 → web rclpy 全瘫 (imu_count=0)。
  本节点独立 rclpy 节点 + 独立 wait_set, 不抢 web; 通过文件 IPC 把降采后 costmap 交给
  web broadcast_loop 读转发 WS 给前端 (map.js _renderCostmap 渲染)。

运行 (NX, 独立于 web):
  systemctl status costmap-bridge.service
  生产服务从 /home/nx/go2w/current/payload/web/costmap_bridge.py 启动，
  并跟随 go2w-slam-nav.service 生命周期；不要直接运行旧 workspace 副本。

前端: web broadcast_loop 读 /tmp/costmap_lite.json 推 type=costmap WS, map.js 已渲染。
搜索: 同一张带 unknown/free/occupied 语义的 OccupancyGrid 原样转发到
      /map_frontier；不启动第二个 SLAM，也不新增冲突的 map→odom TF。
"""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


def _downsample_values(values, width, height, step):
    """Max-pool occupancy blocks so thin lethal obstacles are never skipped."""
    output = []
    for y0 in range(0, height, step):
        for x0 in range(0, width, step):
            block = [
                int(values[y * width + x])
                for y in range(y0, min(y0 + step, height))
                for x in range(x0, min(x0 + step, width))
            ]
            known = [value for value in block if value >= 0]
            output.append(max(known) if known else -1)
    return output


class CostmapBridge(Node):
    def __init__(self):
        super().__init__('costmap_bridge')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._cb, qos)
        self.get_logger().info(
            'costmap_bridge: /global_costmap/costmap -> '
            '/tmp/costmap_lite.json (max-pool ~80x80)')

    def _cb(self, msg):
        # Nav2 already fused MID360 rays, unknown space and inflated obstacles
        # in map coordinates. Reusing it avoids a second SLAM TF publisher.
        info = msg.info
        w, h = info.width, info.height
        step = max(1, w // 80)  # 降采步长 (100x100 step=1 不抽; 大地图抽到 ~80 列)
        sw = (w + step - 1) // step
        sh = (h + step - 1) // step
        sub = _downsample_values(msg.data, w, h, step)
        out = {
            'w': sw, 'h': sh,
            'res': info.resolution * step,
            'ox': info.origin.position.x,
            'oy': info.origin.position.y,
            'vals': sub,
        }
        try:
            with open('/tmp/costmap_lite.json', 'w') as f:
                json.dump(out, f)
        except Exception as e:
            self.get_logger().warning(f'写 costmap_lite.json 失败: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
