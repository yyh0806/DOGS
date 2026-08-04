#!/bin/bash
# nav2_timesync_verify.sh — 验证 nav2 假成功根治 (方案 B' 全链 lidar time)
# 在 NX 上跑。前提: bringup_livo.sh 或 bringup_slam_nav2.sh 已起完全栈。
# 验收 R1-R7: 见 .omc/plans/nav2-timesync-plan-B.md §5.2
set -uo pipefail

echo "===== Nav2 time-sync 根治验证 (方案 B') ====="

# 0. 前提: /livox/lidar stamp offset 统计 (决定 offset EMA 方案是否可行)
echo "[0/7] /livox/lidar stamp offset 统计 (10帧)..."
timeout 5 python3 -c "
import rclpy, statistics
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
rclpy.init()
n = Node('stamp_check')
offsets = []
def cb(msg):
    lidar_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
    wall_ns = n.get_clock().now().nanoseconds
    offsets.append((lidar_ns - wall_ns) / 1e9)
    if len(offsets) >= 10:
        print(f'  offset avg={statistics.mean(offsets):.3f}s stdev={statistics.stdev(offsets):.3f}s')
        print(f'  offsets={[round(o,3) for o in offsets]}')
        rclpy.shutdown()
n.create_subscription(PointCloud2, '/livox/lidar', cb, 10)
rclpy.spin(n)
" 2>/dev/null || echo "  [WARN] offset 统计失败 (python3/rclpy 不可用?)"
echo "  期望: avg ~-1.5s (lidar 旧), stdev < 0.1s (稳定). 若 stdev > 0.3s 需调 EMA alpha 或改方案"

# 1. 记录 goal 前位姿
echo "[1/7] 记录 goal 前位姿..."
POSE_BEFORE=$(ros2 topic echo /Odometry --once 2>/dev/null | grep -A3 "position:" | head -4)
X_BEFORE=$(echo "$POSE_BEFORE" | grep "x:" | awk '{print $2}')
echo "  before x: $X_BEFORE"

# 2. 发 goal x=1.0
echo "[2/7] 发 navigate_to_pose goal x=1.0..."
GOAL_RESULT=$(ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}}" \
  --feedback 2>&1)
echo "  result: $(echo "$GOAL_RESULT" | tail -3)"

# 3. 记录 goal 后位姿
echo "[3/7] 记录 goal 后位姿..."
POSE_AFTER=$(ros2 topic echo /Odometry --once 2>/dev/null | grep -A3 "position:" | head -4)
X_AFTER=$(echo "$POSE_AFTER" | grep "x:" | awk '{print $2}')
echo "  after x: $X_AFTER"

# 4. 算位移 (R1)
echo "[4/7] 计算位移 (R1)..."
if [ -n "$X_BEFORE" ] && [ -n "$X_AFTER" ]; then
  DELTA=$(python3 -c "print(abs($X_AFTER - $X_BEFORE))")
  echo "  Δx = ${DELTA}m (goal 1.0m, tolerance 0.20)"
  if awk -v d="$DELTA" 'BEGIN{exit !(d>0.8)}'; then
    echo "  [PASS R1] 位移 > 0.8m, 狗真到 goal"
  else
    echo "  [FAIL R1] 位移 < 0.8m, 假成功未根治"
  fi
else
  echo "  [WARN] 无法获取位姿, 手动检查"
fi

# 5. 查 nav2 日志 (R2/R3)
echo "[5/7] 检查 nav2 日志 (R2/R3)..."
NAV2_LOG=$(systemctl status nav2-3d 2>/dev/null | grep -oP 'log.*\.log' | head -1)
if [ -n "$NAV2_LOG" ] && [ -f "$NAV2_LOG" ]; then
  NO_VALID=$(grep -c "No valid trajectories" "$NAV2_LOG" 2>/dev/null || echo 0)
  DROP_FRAME=$(grep -c "dropping frame.*base_link" "$NAV2_LOG" 2>/dev/null || echo 0)
else
  NO_VALID=$(journalctl -u nav2-3d --since "5 min ago" --no-pager 2>/dev/null | grep -c "No valid trajectories" || echo 0)
  DROP_FRAME=$(journalctl -u nav2-3d --since "5 min ago" --no-pager 2>/dev/null | grep -c "dropping frame.*base_link" || echo 0)
fi
echo "  'No valid trajectories': $NO_VALID 条 (期望 0) [R2]"
echo "  'dropping frame base_link': $DROP_FRAME 条 (期望 0) [R3]"

# 6. TF stamp 同源 (R4)
echo "[6/7] TF stamp 同源检查 (R4)..."
echo "  map→base_link TF time:"
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep "At time" | head -1
echo "  /livox/lidar stamp:"
timeout 3 ros2 topic echo /livox/lidar --field header.stamp --once 2>/dev/null
echo "  (两者秒数应接近, 差 < 0.2s)"

# 7. costmap + 频率 (R5/R6/R7)
echo "[7/7] costmap + 频率 (R5/R6/R7)..."
echo "  /scan hz (期望 ~10Hz) [R6]:"
timeout 5 ros2 topic hz /scan 2>/dev/null | head -2
echo "  /tf hz (odom→base_link, 期望 ~50Hz) [R7]:"
timeout 5 ros2 topic hz /tf 2>/dev/null | head -2
echo "  costmap 有障碍 [R5]: rviz 看 /local_costmap/costmap_updates 非全 0, 或:"
timeout 3 ros2 topic echo /local_costmap/costmap_updates --once 2>/dev/null | head -5

echo "===== 验证完成 ====="
echo "R1=位移>0.8m  R2=No valid trajectories=0  R3=dropping frame=0"
echo "R4=TF stamp 同源  R5=costmap 有障碍  R6=/scan~10Hz  R7=/tf~50Hz"
echo "全 PASS = 假成功根治. 详见 .omc/plans/nav2-timesync-plan-B.md"
