#!/usr/bin/env bash
# 清场脚本: 杀掉所有仿真栈进程 (避开 pkill -f 自杀陷阱 —— 本脚本进程 cmdline
# 仅 "bash .../kill_sim.sh", 不含下方任何目标模式串, 故 pkill -f 不自匹配).
# importer/caller: 人工 `bash /mnt/c/.../kill_sim.sh` (运维脚本, 非库模块, 无 python import)
# 受影响 API: 无 (纯系统进程管理). 用户原话: "让仿真跟真实一样, 点击哪通过nav2移动到哪,
#   点击搜索房间可以执行算法。还有前进后退。fastlio的激光雷达仿真全部接入。"
# 用法 (WSL): bash /mnt/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws/tools/kill_sim.sh
set +e
for pat in \
  'sim_full_bringup.launch' \
  'sim_fastlio_bringup' \
  'sim_spawn_only' \
  'sim_nav_bringup' \
  'fastlio_mapping' \
  'nav2_container' \
  'component_container' \
  'topic_tools.*relay' \
  'nx_motion_node' \
  'nx_web_server' \
  'sim_telemetry_bridge' \
  'sim_odom_tf' \
  'sim_amcl_to_odom' \
  'pointcloud_to_laserscan' \
  'gzserver' \
  'gzclient' \
  'robot_state_publisher' \
  ; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2
pkill -9 -f 'ros2.*topic_tools' 2>/dev/null
pkill -9 -f 'ros2.*run.*go2w' 2>/dev/null
sleep 1
echo "===KILL_DONE==="
echo "===REMAINING==="
pgrep -af 'ros2|gzserver|fastlio|nav2|nx_web|nx_motion|sim_' 2>/dev/null | grep -v 'kill_sim.sh' | head -20 || echo "none"
echo "===END==="
