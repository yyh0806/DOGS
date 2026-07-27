#!/usr/bin/env bash
# Check /scan near-range points (self-hit diagnosis for planner failure).
set +e
source ~/go2w_ws/install/setup.bash
echo "=== /scan stats ==="
timeout 10 python3 << "PYEOF"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init()
n = Node("scan_check2")
got = []
def cb(m):
    if not got:
        finite = [r for r in m.ranges if r == r and r < m.range_max]
        near = [r for r in finite if r < 0.5]
        mid = [r for r in finite if 0.5 <= r < 2.0]
        far = [r for r in finite if r >= 2.0]
        got.append({
            "total_ranges": len(m.ranges),
            "finite": len(finite),
            "near_lt0.5m": len(near),
            "near_min": round(min(near), 3) if near else None,
            "mid_0.5_2m": len(mid),
            "far_gt2m": len(far),
            "range_min": m.range_min,
            "range_max": m.range_max,
            "frame_id": m.header.frame_id,
        })
n.create_subscription(LaserScan, "/scan", cb, 10)
import time
t0 = time.time()
while time.time() - t0 < 6 and not got:
    rclpy.spin_once(n, timeout_sec=0.1)
if got:
    for k, v in got[0].items():
        print(f"  {k}: {v}")
else:
    print("  NO SCAN in 6s")
n.destroy_node()
rclpy.shutdown()
PYEOF
echo "=== END ==="
