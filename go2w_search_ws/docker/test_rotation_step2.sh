#!/bin/bash
# 设计文档步骤2: 0.1 rad/s 旋转 ~57° 验证 FastLIO 稳定性
# 通过条件: map→base_link roll/pitch <2°(0.035rad); camera_init→body 平移变化 <1m
# 停止条件(设计文档): 平移跳变>1m / roll/pitch>5° / map TF 消失 → 零速度
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ws_livox/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0

echo "===== 步骤2: 0.1 rad/s 旋转测试 (10s 理论转 57°) ====="
echo ""

# 暂停 velocity_smoother (防它发零 /cmd_vel 抢占; 测完恢复)
echo "--- 暂停 velocity_smoother (独占 /cmd_vel) ---"
timeout 5 ros2 lifecycle set /velocity_smoother deactivate 2>&1 | head -2
sleep 1

echo "--- 起始 pose ---"
echo -n "camera_init→body Translation: "
CB0=$(timeout 6 ros2 run tf2_ros tf2_echo camera_init body 2>&1 | grep "Translation:" | head -1)
echo "$CB0"
echo -n "map→base_link RPY(rad): "
MB0=$(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep "RPY (radian)" | head -1)
echo "$MB0"

echo ""
echo "--- 后台采样 map→base_link (每1.5s, 旋转中) ---"
( for i in $(seq 1 8); do
    echo -n "[$(date +%H:%M:%S)] "
    timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep "RPY (radian)" | head -1
    sleep 1
  done ) > /tmp/rot_sample.txt &
SPID=$!

echo "--- 发 cmd_vel angular.z=0.1 @10Hz 持续 10s ---"
timeout 10 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.1}}" -r 10 >/dev/null 2>&1

# 零速度停 (连发 3 次确保)
for k in 1 2 3; do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
  sleep 0.3
done

wait $SPID 2>/dev/null
sleep 1

echo ""
echo "--- 旋转中 map→base_link 采样 ---"
cat /tmp/rot_sample.txt

echo ""
echo "--- 结束 pose ---"
echo -n "camera_init→body Translation: "
CB1=$(timeout 6 ros2 run tf2_ros tf2_echo camera_init body 2>&1 | grep "Translation:" | head -1)
echo "$CB1"
echo -n "map→base_link RPY(rad): "
MB1=$(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep "RPY (radian)" | head -1)
echo "$MB1"

echo ""
echo "--- 恢复 velocity_smoother (步骤3-4 需要) ---"
timeout 5 ros2 lifecycle set /velocity_smoother activate 2>&1 | head -2
sleep 1
echo -n "velocity_smoother: "
timeout 4 ros2 lifecycle get /velocity_smoother 2>&1 | head -1

echo ""
echo "===== 判读 ====="
echo "map→base_link roll/pitch 应 <2°(0.035rad):"
echo "  起: $MB0"
echo "  终: $MB1"
echo "camera_init→body 平移变化应 <1m:"
echo "  起: $CB0"
echo "  终: $CB1"
