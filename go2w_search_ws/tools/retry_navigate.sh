#!/usr/bin/env bash
# importer/caller: 人工执行 `bash retry_navigate.sh` (运维脚本, 非库模块, 无 python import).
# 受影响 API: 无 (纯进程管理 + curl/ros2 CLI). 用户原话: "让仿真跟真实一样, 点击哪通过nav2移动到哪,
#   点击搜索房间可以执行算法。还有前进后退。fastlio的激光雷达仿真全部接入。"
# 自动重试: gzserver WSL2 反复 SIGFPE 随机崩, 多次 launch 直到稳定窗口命中 (b463cl71d 模式),
# 立即测点击导航 + 验证狗移动 (cmd_vel + /Odometry 位置变化).
# 脚本文件隔离 cmdline (kill_sim 的 pkill -f 'sim_full_bringup.launch' 只杀 setsid ros2 launch
# 进程, 不杀本脚本 bash - bash retry_navigate.sh cmdline 不含 launch 名).
set +e
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/go2w_ws/install/setup.bash"
WF="/root/go2w_ws/install/go2w_sim/share/go2w_sim/worlds/indoor_empty.world"
KILL=/mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/tools/kill_sim.sh

for attempt in 1 2 3 4 5; do
  echo "===ATTEMPT_$attempt==="
  bash "$KILL" > /dev/null 2>&1
  rm -rf /dev/shm/fastrtps* /dev/shm/gazebo* /tmp/gazebo* 2>/dev/null
  sleep 1
  source "$ROS_SETUP" 2>/dev/null
  source "$WS_SETUP" 2>/dev/null
  setsid ros2 launch go2w_sim sim_full_bringup.launch.py world:="$WF" > /tmp/sim_launch.log 2>&1 < /dev/null &
  sleep 90
  GZ=$(pgrep gzserver 2>/dev/null | head -1)
  # 不依赖 gzserver (mock_planar_move/mock_scan 纯 ROS, gzserver SIGFPE 崩也能跑).
  # 只要 web 响应 + mock_planar_move 发 /odom_planar.
  WEB=$(curl -s --max-time 3 http://localhost:8000/api/status 2>/dev/null | head -c 20)
  if [ -z "$WEB" ]; then
    echo "ATTEMPT_$attempt: no_web_retry"
    continue
  fi
  ODOM=$(timeout 4 ros2 topic hz /odom_planar 2>&1 | head -1)
  if [ -z "$ODOM" ]; then
    echo "ATTEMPT_$attempt: no_odom_planar_retry (gz=$GZ)"
    continue
  fi
  echo "ATTEMPT_$attempt: STABLE gz=$GZ scan=$SCAN"
  # 搜索房间 (狗初始 parked, frontier_explore; slam_toolbox /map_frontier + ai 空检测)
  SEARCH_RESP=$(curl -s --max-time 8 -X POST http://localhost:8000/api/search_room -H "Content-Type: application/json" -d '{"target_classes":["person"],"room":"__current__","require_photos":false}')
  echo "SEARCH_ROOM_RESP: $SEARCH_RESP"
  sleep 10
  SEARCH_STATE=$(curl -s --max-time 4 http://localhost:8000/api/status 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);r=d.get('room_nav',{});print('search_phase=',r.get('search_phase'),'mission=',r.get('mission_id'),'state=',r.get('status'))" 2>/dev/null)
  echo "SEARCH_STATE: $SEARCH_STATE"
  # 点击导航 (狗初始 parked, sport_mode=6 → activatable=True)
  POS_BEFORE=$(timeout 3 ros2 topic echo /odom_planar --once 2>/dev/null | grep -A1 "position:" | head -2 | tr '\n' ' ')
  echo "POS_BEFORE: $POS_BEFORE"
  NAV_RESP=$(curl -s --max-time 10 -X POST http://localhost:8000/api/navigate -H "Content-Type: application/json" -d '{"frame_id":"map","x":1.5,"y":0.0,"yaw":0.0}')
  echo "NAVIGATE_RESP: $NAV_RESP"
  # 等 motion 激活 (SimTelemetryBridge 订 /cmd_vel BalanceStand → sport_mode=3 → NAV_ACTIVE)
  # + Nav2 NavigateToPose 规划 + controller cmd_vel + mock_planar_move 狗位移
  sleep 16
  CMD_VEL=$(timeout 4 ros2 topic hz /cmd_vel 2>&1 | head -1)
  echo "CMD_VEL_HZ: $CMD_VEL"
  POS_AFTER=$(timeout 3 ros2 topic echo /odom_planar --once 2>/dev/null | grep -A1 "position:" | head -2 | tr '\n' ' ')
  echo "POS_AFTER: $POS_AFTER"
  NAV_STATE=$(curl -s --max-time 5 http://localhost:8000/api/status 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);n=d.get('navigation',{});print('state=',n.get('state'),'session=',n.get('drive_session'),'reason=',n.get('reason'))" 2>/dev/null)
  echo "NAV_STATE: $NAV_STATE"
  GZ_AFTER=$(pgrep gzserver 2>/dev/null | head -1)
  echo "GZ_AFTER_NAV: ${GZ_AFTER:-dead}"
  echo "===SUCCESS_ATTEMPT_$attempt==="
  exit 0
done
echo "===ALL_5_ATTEMPTS_FAILED (gz never stable enough)==="
