#!/bin/bash
# diagnose_nav2_goal.sh — 发 Nav2 NavigateToPose goal, 并行采集 6 维证据, 定位"没到目标"原因
# ============================================================
# 用法 (NX 上, 先跑完 bringup_slam_nav2.sh):
#   bash ~/go2w_ws/docker/diagnose_nav2_goal.sh          # 默认目标 (5, 0), 超时 90s
#   bash ~/go2w_ws/docker/diagnose_nav2_goal.sh 3 2       # 目标 (3, 2)
#   bash ~/go2w_ws/docker/diagnose_nav2_goal.sh 5 0 60    # 目标 (5,0), 超时 60s
#
# 6 维证据 (goal 结束后汇总判读):
#   1. action 结果     SUCCEEDED / ABORTED / CANCELED / REJECTED + error_code
#   2. global path     /plan 有没有 (planner 规划成功没)
#   3. /cmd_vel 实测   linear.x 峰值 (区分"发过前进" vs "只发旋转")
#   4. 移动距离        起点→终点 (判"卡住" vs "偏离" vs "到不了")
#   5. TF map→base_link pitch  (倾斜证据, map_odom_fuser _R_level bug 复发)
#   6. nav2-3d 日志    planner/controller 报错 (no path / no valid trajectory / out of bounds)
# ============================================================
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f $HOME/ws_livox/install/setup.bash ] && source $HOME/ws_livox/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0

GX=${1:-5.0}; GY=${2:-0.0}; TIMEOUT=${3:-90}
TS=$(date +%H%M%S)
TMP=/tmp/nav2diag_$TS
mkdir -p "$TMP"

C_G=$'\033[32m'; C_R=$'\033[31m'; C_Y=$'\033[33m'; C_C=$'\033[36m'; C_OFF=$'\033[0m'
ok()   { echo "${C_G}[ OK ]${C_OFF} $1"; }
fail() { echo "${C_R}[FAIL]${C_OFF} $1"; }
warn() { echo "${C_Y}[WARN]${C_OFF} $1"; }

map_pose_xy() {
  timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null \
    | sed -n 's/.*Translation: \[\([^,]*\), \([^,]*\),.*/\1 \2/p' \
    | head -1
}

echo "${C_C}===== Nav2 goal 诊断: 目标 ($GX, $GY), 超时 ${TIMEOUT}s =====${C_OFF}"
echo "证据目录: $TMP"
echo ""

if ! ros2 node list 2>/dev/null | grep -q bt_navigator; then
  fail "bt_navigator 不在线 — Nav2 没起. 先跑 bringup_slam_nav2.sh"; exit 1
fi
ok "bt_navigator 在线"

START=$(map_pose_xy); echo "$START" > "$TMP/start.txt"
echo "起点 (map→base_link): $START"

# ---- 后台采集 (goal 期间并行) ----
( timeout $TIMEOUT ros2 topic echo /cmd_vel 2>/dev/null ) > "$TMP/cmd_vel.txt" &
( for i in $(seq 1 $((TIMEOUT/2 + 1))); do
    echo "[$(date +%H:%M:%S)]"
    timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | awk '/Rotation:/{f=1;next} f&&/^$/{exit} f'
    sleep 2
  done ) > "$TMP/tf.txt" &
TF_PID=$!
journalctl -u nav2-3d -f 2>/dev/null > "$TMP/nav2_log.txt" &
LOG_PID=$!
( timeout $TIMEOUT ros2 topic echo /plan --once 2>/dev/null ) > "$TMP/plan.txt" &
PLAN_PID=$!

echo "${C_C}--- 发 goal (阻塞, 最长 ${TIMEOUT}s) ---${C_OFF}"
GOAL_JSON="{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}"
timeout $TIMEOUT ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$GOAL_JSON" \
  > "$TMP/goal.txt" 2>&1
GOAL_RC=$?
echo ""

kill $TF_PID $LOG_PID $PLAN_PID 2>/dev/null
pkill -f "ros2 topic echo /cmd_vel" 2>/dev/null
pkill -f "ros2 topic echo /plan" 2>/dev/null
sleep 0.5

# ===== 汇总判读 =====
echo "${C_C}===== 证据汇总 =====${C_OFF}"

echo ""; echo "--- 1. action 结果 (rc=$GOAL_RC) ---"
if   grep -q "SUCCEEDED" "$TMP/goal.txt"; then ok "goal SUCCEEDED (到达目标)";
elif grep -q "ABORTED"   "$TMP/goal.txt"; then fail "goal ABORTED (planner/controller 放弃)";
elif grep -q "CANCELED"  "$TMP/goal.txt"; then warn "goal CANCELED (超时或被取消)";
elif grep -q "REJECTED"  "$TMP/goal.txt"; then fail "goal REJECTED (bt_navigator 拒绝)";
else warn "无明确结果 (ros2 action 超时? 看 $TMP/goal.txt)"; fi
grep -iE "error_code|result_code|status" "$TMP/goal.txt" | head -4

echo ""; echo "--- 2. global path /plan ---"
if [ -s "$TMP/plan.txt" ] && grep -q "poses:" "$TMP/plan.txt"; then
  NP=$(grep -c "position:" "$TMP/plan.txt")
  ok "/plan 有路径 ($NP 个 pose — planner 规划成功)"
else
  fail "/plan 空 — planner 没规划出来 (no path / costmap 全障碍 / goal 在 unknown?)"
fi

echo ""; echo "--- 3. /cmd_vel 实测 (linear.x 峰值) ---"
NL=$(grep -c "linear:" "$TMP/cmd_vel.txt" 2>/dev/null || echo 0)
if [ "$NL" -gt 1 ]; then
  MAXX=$(grep -A3 "linear:" "$TMP/cmd_vel.txt" | grep -oE 'x: [-]?[0-9.]+' | grep -oE '[-]?[0-9.]+$' \
         | sort -n | tail -1)
  echo "   收到 $NL 条 cmd_vel, linear.x 峰值 = ${MAXX:-0}"
  awk -v m="${MAXX:-0}" 'BEGIN{exit !(m+0>0.01)}' \
    && ok "linear.x 有 >0.01 — 发过前进速度" \
    || fail "linear.x 全 ~0 — DWB 只发旋转不发前进 (costmap/TF 让前进轨迹全判碰撞)"
else
  warn "/cmd_vel 几乎无数据 — controller_server 没发速度 (未 activate? TF extrapolation 超时?)"
fi

echo ""; echo "--- 4. 移动距离 (起点→终点) ---"
END=$(map_pose_xy)
sx=$(echo "$START" | awk '{print $1}'); sy=$(echo "$START" | awk '{print $2}')
ex=$(echo "$END"   | awk '{print $1}'); ey=$(echo "$END"   | awk '{print $2}')
if [ -n "$sx" ] && [ -n "$ex" ] && [ "$sx" != "" ]; then
  DIST=$(awk -v sx=$sx -v sy=$sy -v ex=$ex -v ey=$ey 'BEGIN{printf "%.2f", sqrt((ex-sx)^2+(ey-sy)^2)}')
  GDIST=$(awk -v sx=$sx -v sy=$sy -v gx=$GX -v gy=$GY 'BEGIN{printf "%.2f", sqrt((gx-sx)^2+(gy-sy)^2)}')
  echo "   起点($sx,$sy) → 终点($ex,$ey): 移动 ${DIST}m / 起点距目标 ${GDIST}m"
  awk -v d=$DIST 'BEGIN{exit !(d+0<0.3)}' \
    && fail "几乎没动 (<0.3m) — 底层执行断 (低电? lease 丢? Move 盲调未执行?)" \
    || ok "移动了 ${DIST}m"
else
  warn "TF map→base_link 取不到位姿 (TF 断? fuser 没发 map→odom?)"
fi

echo ""; echo "--- 5. TF map→base_link pitch (倾斜证据) ---"
if [ -s "$TMP/tf.txt" ]; then
  echo "   (完整采样见 $TMP/tf.txt, 末次 Rotation:)"
  tail -8 "$TMP/tf.txt"
  warn "若 quaternion 的 y ≈ ±0.17 (即 pitch ≈ ±20°=±0.35rad), = map_odom_fuser _R_level 倾斜 bug"
fi

echo ""; echo "--- 6. nav2-3d 日志报错 ---"
ERRS=$(grep -iE "fail|abort|error|no valid|out of bounds|no path|extrapolat" "$TMP/nav2_log.txt" 2>/dev/null \
       | grep -ivE "INFO|DEBUG|level=" | tail -10)
if [ -n "$ERRS" ]; then echo "$ERRS"; else echo "   (无 WARN/ERROR 级报错)"; fi

echo ""; echo "${C_C}===== 全部证据存于 $TMP =====${C_OFF}"
echo ""
echo "判读速查:"
echo "  ABORTED + /plan 空        → planner no path (costmap 全障碍 / map 倾斜投影错 / goal 在 unknown)"
echo "  ABORTED + linear.x 全 0   → DWB 只旋转不前进 (costmap/TF 让前进轨迹全判碰撞)"
echo "  ABORTED + 移动 <0.3m      → 底层执行断 (低电 / lease 丢失 / Move 盲调未执行)"
echo "  SUCCEEDED + 移动远 <目标   → goal tolerance 过大 或 定位漂移 (FastLIO 跟丢)"
echo "  TF pitch ≈ ±20°           → map_odom_fuser _R_level 倾斜 bug, map frame 不水平 → 障碍错位"
