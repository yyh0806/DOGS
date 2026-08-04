#!/usr/bin/env bash
# Full clean + launch + diagnose sim stack. Run inside wsl.
# Avoids inline $(...) which breaks via wsl.exe+GitBash quoting.
set +e
source ~/go2w_ws/install/setup.bash

echo "===KILL_STALE==="
kill -9 539 2>/dev/null
bash /mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/tools/kill_sim.sh >/dev/null 2>&1
pkill -9 -f nx_web_server 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f nx_motion_node 2>/dev/null
pkill -9 -f sim_telemetry 2>/dev/null
pkill -9 -f fastlio 2>/dev/null
pkill -9 -f nav2_container 2>/dev/null
pkill -9 -f component_container 2>/dev/null
pkill -9 -f ros2cli.daemon 2>/dev/null
pkill -9 -f "topic pub" 2>/dev/null
pkill -9 -f "topic echo" 2>/dev/null
sleep 3
rm -rf /tmp/gazebo* /dev/shm/gazebo* /dev/shm/fastrtps* /dev/shm/sem.* /dev/shm/iceoryx* /tmp/fast* 2>/dev/null

echo "===LAUNCH==="
WF=/root/go2w_ws/install/go2w_sim/share/go2w_sim/worlds/warehouse.world
setsid ros2 launch go2w_sim sim_full_bringup.launch.py world:=$WF > /tmp/sim_launch.log 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "LAUNCHED pid=$LAUNCH_PID"

echo "===WAIT_50s==="
sleep 50

echo "=== :8000 ==="
ss -tlnp 2>/dev/null | grep ":8000" || echo "NO :8000"

echo "=== web proc ==="
ps -eo pid,etime,cmd | grep nx_web_server | grep -v grep | head -3

echo "=== TF_STATIC map->odom ==="
timeout 4 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|sec:|nanosec" | head -10

echo "=== TF map->base_link ==="
timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -10

echo "=== /scan sample ==="
timeout 5 ros2 topic echo /scan --once 2>&1 | grep -E "frame_id|range_min|range_max" | head -6

echo "=== /livox/lidar_PointCloud2 hz ==="
timeout 6 ros2 topic hz /livox/lidar_PointCloud2 2>&1 | tail -3

echo "=== /scan hz ==="
timeout 6 ros2 topic hz /scan 2>&1 | tail -3

echo "=== /cmd_vel sample ==="
timeout 4 ros2 topic echo /cmd_vel --once 2>&1 | head -8

echo "=== /dog_state ==="
timeout 4 ros2 topic echo /dog_state --once 2>&1 | head -c 600

echo ""
echo "=== WEB /api/status ==="
curl -s http://localhost:8000/api/status 2>&1 | head -c 800

echo ""
echo "=== fastlio No-point count ==="
grep -c "No point" /tmp/sim_launch.log 2>/dev/null

echo "=== launch log tail ==="
tail -15 /tmp/sim_launch.log 2>/dev/null

echo "=== END ==="
