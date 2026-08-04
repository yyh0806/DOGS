"""探针: SensorDataQoS 订阅 /livox/lidar, 输出点数 + 距离分布 (FastLIO 排障).

区分: MID360 故障(点数少) / 环境退化(远点多近点少) / 配置(blind 滤太多).
ros2 topic echo 收不到 CustomMsg 是因为默认 reliable, livox 用 best_effort.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg

rclpy.init()
n = Node("inspect_lidar")
got = [None]


def cb(m):
    if got[0] is None:
        got[0] = m


n.create_subscription(
    CustomMsg, "/livox/lidar", cb,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
)

end = time.time() + 8
while time.time() < end and got[0] is None:
    rclpy.spin_once(n, timeout_sec=0.1)

if got[0] is None:
    print("NO_MSG (8s 内未收到 CustomMsg — driver 断或 QoS?)")
else:
    pts = got[0].points
    N = len(pts)
    print(f"point_num field={got[0].point_num}  actual len={N}")
    if N > 0:
        ds = sorted(math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z) for p in pts)
        med = ds[N // 2]
        p90 = ds[int(N * 0.9)]
        near1 = sum(1 for d in ds if d < 1.0)
        near2 = sum(1 for d in ds if d < 2.0)
        far = sum(1 for d in ds if d > 20.0)
        print(f"距离: median={med:.2f}m  p90={p90:.2f}m  max={ds[-1]:.2f}m")
        print(f"近点: <1m={near1}({100*near1//N}%)  <2m={near2}({100*near2//N}%)  >20m={far}({100*far//N}%)")
        eff = sum(1 for d in ds if d > 0.5)
        print(f"blind>0.5m 剩 {eff}({100*eff//N}%) → point_filter÷3 ≈ {eff//3} → voxSurf0.5 降采样")
rclpy.shutdown()
