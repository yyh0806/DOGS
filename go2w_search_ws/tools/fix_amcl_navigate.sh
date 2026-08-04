#!/usr/bin/env bash
# Diagnose planner failure + send AMCL initialpose + re-navigate.
set +e
source ~/go2w_ws/install/setup.bash

echo "=== map_server / global_costmap log ==="
grep -iE "map_server|map_loader|global_costmap|Map loaded|OccGrid|static_layer|received map|amcl.*initial|amcl.*pose" /tmp/sim_launch.log | grep -v "cannot publish\|set the initial" | tail -15

echo "=== /map metadata ==="
timeout 5 ros2 topic echo /map --once --field info 2>&1 | head -15

echo "=== /amcl_pose check ==="
timeout 4 ros2 topic echo /amcl_pose --once 2>&1 | head -6

echo "=== send AMCL initialpose (0,0,0) ==="
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068, 0.0, 0.0, 0.0, 0.0]}}" 2>&1 | tail -2

sleep 3
echo "=== /amcl_pose after initialpose ==="
timeout 4 ros2 topic echo /amcl_pose --once 2>&1 | head -6

echo "=== re-navigate (3,0) ==="
curl -s -X POST http://localhost:8000/api/navigate -H "Content-Type: application/json" -d '{"x":3.0,"y":0.0,"yaw":0.0}' 2>&1 | head -c 200
echo ""

sleep 12
echo "=== /cmd_vel ==="
timeout 4 ros2 topic echo /cmd_vel --once 2>&1 | head -8
echo "=== /odom_planar position ==="
timeout 4 ros2 topic echo /odom_planar --once 2>&1 | grep -A4 position | head -6
echo "=== /dog_state ==="
timeout 4 ros2 topic echo /dog_state --once 2>&1 | head -c 400
echo ""
echo "=== planner log latest ==="
grep -iE "planner_server|GridBased|compute_path|plan|navigate_to_pose" /tmp/sim_launch.log | tail -10
echo "=== END ==="
