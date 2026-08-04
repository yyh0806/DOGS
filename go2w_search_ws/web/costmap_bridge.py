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
from nav_msgs.msg import OccupancyGrid, Path


def _downsample_values(values, width, height, step):
    """Max-pool occupancy blocks so thin lethal obstacles are never skipped.

    Bounds the grid up front: the double loop is O(w*h), so capping only the
    output (or the step) cannot stop a runaway OccupancyGrid (config error or
    a rogue DDS publisher) from pinning the single-threaded callback spin.
    """
    try:
        w, h = int(width), int(height)
    except (TypeError, ValueError):
        return []
    if w <= 0 or h <= 0:
        return []
    if w * h > 10_000_000:  # 3160x3160 ≈ 100m×100m @ 3cm; past any real costmap
        import logging
        logging.getLogger("go2w.costmap_bridge").warning(
            "costmap grid %dx%d exceeds 10M cells, skipping downsample", w, h)
        return []
    step = max(1, int(step))
    output = []
    for y0 in range(0, h, step):
        for x0 in range(0, w, step):
            block = [
                int(values[y * w + x])
                for y in range(y0, min(y0 + step, h))
                for x in range(x0, min(x0 + step, w))
            ]
            known = [value for value in block if value >= 0]
            output.append(max(known) if known else -1)
    return output


def _extract_occupied_points(
        values, width, height, resolution, origin_x, origin_y,
        occupied_threshold=65, max_points=5000):
    """Return a bounded, evenly sampled persistent SLAM wall point cloud.

    Bounds the grid up front: the double loop is O(w*h), so capping only the
    output sample count cannot stop a runaway OccupancyGrid from pinning the
    single-threaded callback spin.
    """
    try:
        w, h = int(width), int(height)
    except (TypeError, ValueError):
        return []
    if w <= 0 or h <= 0:
        return []
    if w * h > 10_000_000:
        import logging
        logging.getLogger("go2w.costmap_bridge").warning(
            "costmap grid %dx%d exceeds 10M cells, skipping extraction", w, h)
        return []
    occupied = []
    for row in range(h):
        offset = row * w
        for col in range(w):
            if int(values[offset + col]) < int(occupied_threshold):
                continue
            occupied.append([
                float(origin_x) + (col + 0.5) * float(resolution),
                float(origin_y) + (row + 0.5) * float(resolution),
            ])
    limit = max(1, int(max_points))
    if len(occupied) <= limit:
        return occupied
    count = len(occupied)
    return [occupied[int(index * count / limit)] for index in range(limit)]


class CostmapBridge(Node):
    def __init__(self):
        super().__init__('costmap_bridge')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # local (DWB 避障用, 前端默认显示 —— 用户要看狗避障时实际用的 costmap)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            lambda msg: self._cb(msg, '/tmp/costmap_lite.json'), qos)
        # global (Navfn 规划用, 前端可切换看规划视野/ghost 对比)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            lambda msg: self._cb(msg, '/tmp/costmap_global.json'), qos)
        # Persistent SLAM walls are a display-only layer. Keeping them
        # separate from Nav2's live red costmap prevents old map artefacts
        # from becoming navigation authority while retaining room geometry.
        self.create_subscription(
            OccupancyGrid, '/map_frontier', self._map_cb, qos)
        # plan (nav2 规划路径, 前端 polyline 显示狗要走的路线)
        plan_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Path, '/plan', self._plan_cb, plan_qos)
        self.get_logger().info(
            'costmap_bridge: /local_costmap->costmap_lite.json (避障,默认) + '
            '/global_costmap->costmap_global.json (规划,切换) + '
            '/map_frontier->map_frontier_walls.json (持久墙体) + '
            '/plan->plan_lite.json (路线 polyline)')

    def _cb(self, msg, out_path):
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
            with open(out_path, 'w') as f:
                json.dump(out, f)
        except Exception as e:
            self.get_logger().warning(f'写 {out_path} 失败: {e}', throttle_duration_sec=5.0)

    def _map_cb(self, msg):
        info = msg.info
        points = _extract_occupied_points(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            occupied_threshold=65, max_points=5000)
        out = {
            'points': points,
            'resolution': float(info.resolution),
            'frame_id': str(msg.header.frame_id or 'map'),
        }
        try:
            with open('/tmp/map_frontier_walls.json', 'w') as f:
                json.dump(out, f)
        except Exception as e:
            self.get_logger().warning(
                f'写 /tmp/map_frontier_walls.json 失败: {e}',
                throttle_duration_sec=5.0)

    def _plan_cb(self, msg):
        """nav2 /plan (nav_msgs/Path) -> [[x,y],...] 抽稀写 plan_lite.json, 前端画 polyline."""
        poses = getattr(msg, 'poses', None) or []
        pts = []
        for pose in poses:
            try:
                p = pose.pose.position
                pts.append([round(float(p.x), 3), round(float(p.y), 3)])
            except (AttributeError, TypeError, ValueError):
                continue
        # 抽稀到 ~120 点 (路径可能几百点, 前端折线够画)
        if len(pts) > 120:
            stride = len(pts) / 120.0
            sparse, i = [], 0.0
            while i < len(pts):
                sparse.append(pts[int(i)]); i += stride
            if sparse and sparse[-1] != pts[-1]:
                sparse.append(pts[-1])
            pts = sparse
        try:
            with open('/tmp/plan_lite.json', 'w') as f:
                json.dump({'pts': pts}, f)
        except Exception as e:
            self.get_logger().warning(f'写 plan_lite.json 失败: {e}', throttle_duration_sec=5.0)


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
