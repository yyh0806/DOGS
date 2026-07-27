#!/usr/bin/env bash
# 重启仿真栈: 清进程 + 清 Gazebo 残留 + rebuild + launch.
# importer/caller: 人工执行 `bash /mnt/c/.../restart_sim.sh [world]` (运维脚本,
#   非库模块, 无 python import). 受影响 API: 无 (纯进程管理 + 残留清理).
# 用户原话: "让仿真跟真实一样, 点击哪通过nav2移动到哪, 点击搜索房间可以执行算法。
#   还有前进后退。fastlio的激光雷达仿真全部接入。"
# 用法: wsl -d Ubuntu-22.04 -- bash -lc 'bash /mnt/c/.../restart_sim.sh [world_basename]'
#   world_basename: indoor_rooms (默认) | indoor_empty
set +e
WORLD="${1:-indoor_rooms}"
WS=/mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/tools

echo "===KILL==="
bash "$WS/kill_sim.sh" 2>&1 | tr -d '\0'

echo "===CLEAN_GAZEBO_RESIDUE==="
# gzserver SIGFPE -8 根因之一: 上次崩溃残留 /dev/shm /tmp lock 冲突
rm -rf /tmp/gazebo* /dev/shm/gazebo* /tmp/gz* 2>/dev/null
# FastRTPS SHM 残留 (PUB_COUNT=0 静默坑)
rm -rf /dev/shm/fastrtps* 2>/dev/null
sleep 1

echo "===REBUILD==="
source ~/go2w_ws/install/setup.bash 2>/dev/null
cd ~/go2w_ws && colcon build --packages-select go2w_sim --symlink-install 2>&1 | tail -3
source ~/go2w_ws/install/setup.bash

echo "===LAUNCH world=$WORLD==="
WORLD_FILE=$(ros2 pkg prefix go2w_sim)/share/go2w_sim/worlds/${WORLD}.world
setsid ros2 launch go2w_sim sim_full_bringup.launch.py world:=$WORLD_FILE > /tmp/sim_launch.log 2>&1 < /dev/null &
echo "===LAUNCHED pid=$! ==="
sleep 2
