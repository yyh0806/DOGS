#!/bin/bash
# ============================================================
# Go2W FastLIO + Nav2 3D 栈编排启动 (memory slam-nav2-bringup-gotchas 优先级1)
# ============================================================
# 职责: 按依赖顺序起 fastlio + nav2, 每步健康检查 gate, 失败即停 + 报错.
# 根治 memory 6 大坑: 启动顺序(坑1)/SHM(坑2)/lifecycle 时序(坑4)/IMU 断流(坑5)/
#                     systemd RMW(坑6). TF 桥(坑3) 本脚本不动, 禁 SHM 后观察是否同根消失.
#
# 前提 (systemd 永久服务, 本脚本只检查不重启):
#   livox-mid360-driver  → /livox/lidar + /livox/imu
#   go2w-sensor          → /scan + odom→base_link TF
#   go2w-motion          → /cmd_vel 控狗 (持有 lease)
#   go2w-web             → web 面板
#
# 本脚本起 (systemd transient, 重启 NX 会丢):
#   fastlio              → /Odometry + camera_init→body TF
#   nav2-3d              → Nav2 全栈 (controller/planner/behavior/bt_navigator)
#
# 用法 (NX 上, nx 用户):
#   bash bringup_slam_nav2.sh              # 默认: 跳过 SHM 清理 (不扰动已运行 DDS 会话)
#   bash bringup_slam_nav2.sh --no-shm     # 全局禁 SHM (SHM 反复损坏时切, 点云走 UDP)
#   bash bringup_slam_nav2.sh --watch-imu  # 后台监控 /livox/imu, 断流自动 restart driver
# ============================================================
set -o pipefail  # 去 -u: ROS setup.bash AMENT_* 变量未设会 unbound (ssh 非交互式)

# ---- 配置 (环境变量可覆盖) ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="${GO2W_RUNTIME_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
BRIDGE_RUNTIME="$RUNTIME_ROOT/src/go2w_bridge/go2w_bridge"
RUN_USER="$(id -un)"
FASTLIO_WS="${FASTLIO_WS:-$HOME/ws_livox}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-$RUNTIME_ROOT/src/go2w_nav/config/fastlio_low_latency}"  # absolute release-owned directory; launch appends mid360.yaml
FASTLIO_ENABLE="${FASTLIO_ENABLE:-1}"  # required physical-pose backbone; never integrate balancing wheels as pose
PROFILE_XML="${PROFILE_XML:-$RUNTIME_ROOT/docker/fastdds_udp.xml}"
IMU_MIN_HZ="${IMU_MIN_HZ:-50}"      # /livox/imu 断流阈值 (MID360 标称 200Hz)
LIDAR_MIN_HZ="${LIDAR_MIN_HZ:-5}"   # /livox/lidar 最低 Hz
SCAN_MIN_HZ="${SCAN_MIN_HZ:-3}"     # raw MID360 output; watchdog additionally enforces max age
ODOM_MIN_HZ="${ODOM_MIN_HZ:-5}"     # /Odometry (FastLIO) 最低 Hz
WHEEL_ODOM_MIN_HZ="${WHEEL_ODOM_MIN_HZ:-10}" # diagnostic-only wheel odom liveness gate
NAV_HEALTH_STATE="${NAV_HEALTH_STATE:-/tmp/go2w-nav-health-${UID}.json}"
NAV_HEALTH_GATE="$RUNTIME_ROOT/tools/nav_health_gate.py"

# FastLIO body→base_link 安装外参 pitch (设计文档 2026-07-10 §启动与持久化):
# MID360 模组安装外参为 -20° (-0.3490658504 rad). fuser 双侧共轭公式 (inv(T_bb)
# @ T_cb @ T_bb @ inv(T_ob)) 用此值把倾斜传感器系旋转映射为水平底盘 yaw. 环境变量可覆盖
# (如换狗/换安装角重标定后 BODY_TO_BASE_PITCH=-0.xx bash bringup_slam_nav2.sh).
BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:--0.3490658504}"   # rad, -20°

# go2w_nav colcon install WS 探测 (优先 env, 再常见路径)
GO2W_WS="${GO2W_WS:-}"
if [ -z "$GO2W_WS" ]; then
  for _w in "$RUNTIME_ROOT" "$HOME/go2w_ws" "$HOME/ros2_ws" "$HOME/dogs_ws" "$HOME/go2w_search_ws"; do
    [ -f "$_w/install/setup.bash" ] && GO2W_WS="$_w" && break
  done
fi

# ---- 参数解析 ----
MODE_NO_SHM=0
MODE_WATCH_IMU=0
for _a in "$@"; do
  case "$_a" in
    --no-shm) MODE_NO_SHM=1 ;;
    --watch-imu) MODE_WATCH_IMU=1 ;;
    *) echo "未知参数: $_a" >&2 ;;
  esac
done
unset _a _w

# ---- 颜色日志 ----
C_GREEN=$'\033[32m'; C_RED=$'\033[31m'; C_YEL=$'\033[33m'; C_OFF=$'\033[0m'
log()  { echo "[$(date +%H:%M:%S)] $*"; }
ok()   { echo "${C_GREEN}[OK]${C_OFF} $*"; }
warn() { echo "${C_YEL}[WARN]${C_OFF} $*"; }
die()  { echo "${C_RED}[FAIL]${C_OFF} $*"; exit 1; }

# ---- ROS 环境 (首次 source, 后续 no-op — critic M6: 避免 topic_hz 每次都 source 致 wait_hz 超时预算失真) ----
_ROS_ENV_DONE=0
ros_env() {
  [ "$_ROS_ENV_DONE" = "1" ] && return 0
  source /opt/ros/humble/setup.bash
  [ -f "$FASTLIO_WS/install/setup.bash" ] && source "$FASTLIO_WS/install/setup.bash"
  [ -n "$GO2W_WS" ] && [ -f "$GO2W_WS/install/setup.bash" ] && source "$GO2W_WS/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  export ROS2CLI_NO_DAEMON=1
  # frontier v3 (2026-07-21): 边界感知 + yaw 全360°优化 + 时间归一化
  # k_time=5.0 来自 sim 标定 (k,δ) 网格搜索最低 total_turn_rad;
  # δ=1.0 保留边界感知 (sim 简单矩形房间标 0, 实机复杂房间建议 1.0-2.0).
  export GO2W_FRONTIER_UTILITY_MODE=mixed
  export GO2W_FRONTIER_TIME_PENALTY=5.0
  export GO2W_FRONTIER_MIXED_WALL_BONUS=1.0
  export GO2W_FRONTIER_MIXED_EXPANSION_BONUS=0.1
  export GO2W_FRONTIER_PROBE_WORKERS=4
  export GO2W_FRONTIER_YAW_STEP_DEG=45
  export GO2W_FRONTIER_MAX_VEL_X=1.5
  export GO2W_FRONTIER_MAX_VEL_THETA=1.0
  _ROS_ENV_DONE=1
}

# ============================================================
# SHM 治理 (坑2)
# ============================================================
clean_shm() {
  # 只删 fastrtps_*, 绝不动 nvscibuf_*/nvmap_* (Jetson GPU IPC, 删了 GPU 挂)
  local n
  n=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    sudo rm -f /dev/shm/fastrtps_* && ok "清 $n 个 FastRTPS SHM segment (坑2)"
  else
    ok "无残留 FastRTPS SHM segment"
  fi
}

# ============================================================
# 健康检查 gate (失败返回非零, main 里 die)
# ============================================================
# Keep one DDS participant for the complete discovery and sampling window.
# Repeated short-lived ``ros2 topic hz`` processes can all expire before
# FastDDS discovery and falsely reject a healthy publisher.
wait_hz() {
  local topic=$1 min=$2 timeout_s=$3
  local type_name
  case "$topic" in
    /mid360/points_nav) type_name="sensor_msgs/msg/PointCloud2" ;;
    /wheel_odom|/Odometry|/odom|/localization_pose)
      type_name="nav_msgs/msg/Odometry" ;;
    *) die "wait_hz has no commissioned type for $topic" ;;
  esac
  log "等 $topic >= ${min}Hz (最长 ${timeout_s}s)..."
  ros_env
  timeout --kill-after=2 $((timeout_s + 3)) python3 \
    "$GO2W_WS/tools/topic_rate_gate.py" \
    --topic "$topic" --type "$type_name" --minimum-hz "$min" \
    --samples 20 --minimum-samples 10 --timeout "$timeout_s" \
    || die "$topic ${timeout_s}s 未达 ${min}Hz"
  ok "$topic >= ${min}Hz"
}

# Keep one DDS participant subscribed until a latched first message arrives.
# Repeated four-second `topic hz` probes can each expire just after FastDDS
# discovery and therefore miss a 1 Hz map forever.
wait_message() {
  local topic=$1 timeout_s=$2
  log "等 $topic 首帧 (最长 ${timeout_s}s)..."
  ros_env
  timeout "$timeout_s" ros2 topic echo --no-daemon --once "$topic" \
    nav_msgs/msg/OccupancyGrid --field info \
    >/dev/null 2>&1 || die "$topic ${timeout_s}s 未收到首帧"
  ok "$topic 首帧已收到"
}

# 等 TF parent→child 可查
wait_tf() {
  local parent=$1 child=$2 timeout_s=$3
  log "等 TF $parent → $child (最长 ${timeout_s}s)..."
  ros_env
  local output_file tf_pid
  local deadline=$((SECONDS + timeout_s))
  output_file=$(mktemp)
  # ``ros2 run`` owns a child process which can keep the pipe open after grep
  # succeeds or GNU timeout kills only the CLI parent.  Put the complete probe
  # in its own process group and reap that group explicitly on every exit.
  setsid stdbuf -oL ros2 run tf2_ros tf2_echo \
    "$parent" "$child" >"$output_file" 2>/dev/null &
  tf_pid=$!
  while (( SECONDS < deadline )); do
    if grep -q "At time" "$output_file"; then
      kill -TERM -- "-$tf_pid" 2>/dev/null || true
      sleep 0.1
      kill -KILL -- "-$tf_pid" 2>/dev/null || true
      wait "$tf_pid" 2>/dev/null || true
      rm -f "$output_file"
      ok "TF $parent → $child 可查"
      return 0
    fi
    kill -0 "$tf_pid" 2>/dev/null || break
    sleep 0.2
  done
  kill -TERM -- "-$tf_pid" 2>/dev/null || true
  sleep 0.1
  kill -KILL -- "-$tf_pid" 2>/dev/null || true
  wait "$tf_pid" 2>/dev/null || true
  rm -f "$output_file"
  die "TF $parent → $child ${timeout_s}s 不可查 (FastLIO 未发 TF?)"
}

# 等 lifecycle node active, 超时尝试手动 activate (不 die, 允许手动救坑4)
wait_active() {
  local node=$1 timeout_s=$2 outer_timeout
  log "waiting for /$node lifecycle state id=3 (read-only GetState)..."
  ros_env
  outer_timeout=$((timeout_s + 5))
  # rclpy installs SIGTERM handlers; on some Fast DDS shutdown paths the
  # process can remain stuck after GNU timeout sends TERM.  Escalate to KILL
  # so a read-only health probe can never wedge deployment until systemd's
  # 10-minute TimeoutStartSec.
  timeout --kill-after=2 "$outer_timeout" python3 \
    "$GO2W_WS/tools/wait_lifecycle_active.py" \
    --timeout "$timeout_s" "$node" \
    || die "/$node did not report lifecycle state active within ${timeout_s}s"
  ok "/$node active"
}

# Kept temporarily for rollback compatibility with already-staged releases;
# current bringup uses the read-only rclpy GetState probe above.
wait_active_ros2cli_unused() {
  local node=$1 timeout_s=$2
  log "等 /$node active (最长 ${timeout_s}s)..."
  ros_env
  local deadline=$((SECONDS + timeout_s))
  local poll_deadline=$((deadline - 6))
  local st="" st2="" remaining_s query_s spin_s sleep_s
  (( poll_deadline > SECONDS )) || poll_deadline=$deadline
  while :; do
    remaining_s=$((poll_deadline - SECONDS))
    (( remaining_s > 0 )) || break
    query_s=$remaining_s
    (( query_s > 6 )) && query_s=6
    spin_s=$((query_s - 2))
    (( spin_s > 0 )) || spin_s=1
    st=$(timeout "$query_s" ros2 lifecycle get --no-daemon \
      --spin-time "$spin_s" "/$node" 2>/dev/null)
    if echo "$st" | grep -Eq '^active([[:space:]]|$)'; then
      ok "/$node active"; return 0
    fi
    remaining_s=$((poll_deadline - SECONDS))
    (( remaining_s > 0 )) || break
    sleep 1
  done
  warn "/$node 尚未 active (坑4 lifecycle 时序), 在 ${timeout_s}s deadline 内尝试手动 activate"
  remaining_s=$((deadline - SECONDS))
  (( remaining_s > 0 )) || die "/$node ${timeout_s}s deadline 已到"
  query_s=$remaining_s
  (( query_s > 6 )) && query_s=6
  spin_s=$((query_s - 2))
  (( spin_s > 0 )) || spin_s=1
  timeout "$query_s" ros2 lifecycle set --no-daemon \
    --spin-time "$spin_s" "/$node" activate 2>/dev/null \
    || warn "手动 activate /$node 调用失败"
  remaining_s=$((deadline - SECONDS))
  (( remaining_s > 1 )) || die "/$node activate 后无剩余检查预算"
  sleep_s=2
  (( sleep_s < remaining_s )) || sleep_s=$((remaining_s - 1))
  (( sleep_s > 0 )) && sleep "$sleep_s"
  remaining_s=$((deadline - SECONDS))
  (( remaining_s > 0 )) || die "/$node ${timeout_s}s deadline 已到"
  query_s=$remaining_s
  (( query_s > 6 )) && query_s=6
  spin_s=$((query_s - 2))
  (( spin_s > 0 )) || spin_s=1
  st2=$(timeout "$query_s" ros2 lifecycle get --no-daemon \
    --spin-time "$spin_s" "/$node" 2>/dev/null)
  echo "$st2" | grep -Eq '^active([[:space:]]|$)' \
    || die "/$node activate 后仍非 active (状态: $st2). 查 TF map→base_link / costmap 日志 — critic C1 TF 拓扑?"
}

# 等 action server 出现在 graph。先完整捕获 ros2 输出再精确匹配，避免
# ``ros2 ... | grep -q`` 在 pipefail 下因 grep 提前退出让上游 SIGPIPE 假失败。
wait_action() {
  local action=$1 timeout_s=$2
  log "等 action $action (最长 ${timeout_s}s)..."
  ros_env
  local deadline=$((SECONDS + timeout_s))
  local actions="" remaining_s query_s
  while :; do
    remaining_s=$((deadline - SECONDS))
    (( remaining_s > 0 )) || break
    query_s=$remaining_s
    (( query_s > 3 )) && query_s=3
    actions=$(timeout "$query_s" ros2 action list 2>/dev/null) || actions=""
    if grep -Fxq -- "$action" <<<"$actions"; then
      ok "action $action 已暴露"
      return 0
    fi
    remaining_s=$((deadline - SECONDS))
    (( remaining_s > 0 )) || break
    sleep 1
  done
  die "action $action ${timeout_s}s 未暴露 (bt_navigator 未 active?)"
}

# motion 的 systemd active 只表示 Python 进程存在；SportClient lease、订阅和
# scan watchdog 可能尚未就绪。只有 STOPPED + SDK ready + fresh scan 才能暴露 Nav2。
wait_motion_ready() {
  local timeout_s=$1
  log "等 go2w-motion lease + scan gate ready (最长 ${timeout_s}s)..."
  ros_env
  local deadline=$((SECONDS + timeout_s))
  local sample="" compact="" api_compact="" remaining_s query_s battery_soc battery_ok parked_triggered=0
  while :; do
    remaining_s=$((deadline - SECONDS))
    (( remaining_s > 0 )) || break
    # The Web process already maintains the canonical motion snapshot from a
    # long-lived DDS participant. Prefer that local read over creating a new
    # ros2cli participant while FastLIO/SLAM are saturating discovery.
    sample=$(curl --fail --silent --max-time 2 \
      http://127.0.0.1:8000/api/status 2>/dev/null) || sample=""
    compact=$(tr -d '[:space:]' <<<"$sample")
    api_compact="$compact"
    battery_soc=$(sed -n 's/.*"battery_soc":\([0-9][0-9]*\).*/\1/p' <<<"$compact")
    battery_ok=$(awk -v b="${battery_soc:-0}" 'BEGIN {print (b >= 20) ? 1 : 0}')
    if [[ "$compact" == *'"sdk_ready":true'* \
        && "$compact" == *'"nav_scan_fresh":true'* \
        && "$compact" == *'"drive_session":"parked"'* \
        && "$compact" == *'"physical_mode":"joint_lock"'* \
        && "$compact" == *'"actual_motion":"stopped"'* \
        && "$compact" == *'"velocity_authorized":false'* \
        && "$compact" == *'"drive_fault":null'* \
        && "$battery_ok" = 1 ]]; then
      ok "go2w-motion SDK ready + canonical PARKED state + scan fresh (local status)"
      return 0
    fi

    # Fallback remains useful while Web is restarting, but it is no longer the
    # only gate and therefore cannot create a DDS-discovery false negative.
    query_s=$remaining_s
    # A fresh FastDDS CLI participant needs ~3.4s for discovery on the NX.
    (( query_s > 6 )) && query_s=6
    sample=$(timeout "$query_s" ros2 topic echo --no-daemon /dog_state --once --field data 2>/dev/null) || sample=""
    compact=$(tr -d '[:space:]' <<<"$sample")
    battery_soc=$(sed -n 's/.*"battery_soc":\([0-9][0-9]*\).*/\1/p' <<<"$compact")
    battery_ok=$(awk -v b="${battery_soc:-0}" 'BEGIN {print (b >= 20) ? 1 : 0}')
    if [[ "$compact" == *'"sdk_ready":true'* \
        && "$compact" == *'"nav_scan_fresh":true'* \
        && "$compact" == *'"session":"parked"'* \
        && "$compact" == *'"physical_mode":"joint_lock"'* \
        && "$compact" == *'"actual_motion":"stopped"'* \
        && "$compact" == *'"velocity_authorized":false'* \
        && "$compact" == *'"fault":null'* \
        && "$battery_ok" = 1 ]]; then
      ok "go2w-motion SDK ready + canonical PARKED state + scan fresh"
      return 0
    fi
    [ -n "$compact" ] || compact="$api_compact"
    # 2026-07-18 治本 v2: 读 supervisor snapshot 的 dog_state (长驻 rclpy 订阅,
    # discovery 已完成, 可靠). 原用 web /api/status + ros2 echo --no-daemon 的 compact,
    # 但 motion restart / 整机重启 web DDS 订阅断连 (motion 字段 None) + echo 新
    # participant discovery 6s 超时 → compact 拿不到 physical_mode → park 块不触发
    # → motion 卡 BOOT_HOLD → slam-nav 循环 restart (17:12 实测 supervisor snapshot
    # 有完整 session/physical_mode 但 park 块 grep 空). nav_health_supervisor.py 长驻
    # 订阅 /dog_state 存 latest_strings, snapshot 的 dog_state 字段是完整 JSON.
    # 幂等: parked_triggered 只发一次; park 失败由 wait_motion_ready 超时 die 兜底.
    local dog_json=""
    if [[ -n "$NAV_HEALTH_STATE" && -f "$NAV_HEALTH_STATE" ]]; then
      dog_json=$(python3 -c "import json,sys; d=json.load(open('$NAV_HEALTH_STATE')); sys.stdout.write(d.get('dog_state') or '')" 2>/dev/null || true)
    fi
    [[ -n "$dog_json" ]] || dog_json="$compact"
    if (( dbg_count++ < 5 )); then
      log "park_dbg#${dbg_count} trig=$parked_triggered djson_len=${#dog_json} navex=$([[ -f "$NAV_HEALTH_STATE" ]] && echo Y || echo N) wb=$(printf '%s' "$dog_json" | grep -c wheel_balance) bh=$(printf '%s' "$dog_json" | grep -c boot_hold) compact_len=${#compact}"
    fi
    if (( parked_triggered == 0 )) \
        && { [[ "$dog_json" == *'"physical_mode":"wheel_balance"'* \
              || "$dog_json" == *'"physical_mode":"wheel_locomotion"'* ]]; } \
        && { [[ "$dog_json" == *'"session":"boot_hold"'* ]] \
             || [[ "$dog_json" == *'"drive_session_phase":"boot_hold"'* ]] \
             || [[ "$dog_json" == *'"drive_session":"startup"'* ]]; }; then
      log "BOOT_HOLD + wheel mode 检测到 (supervisor snapshot), 显式发 PARK intent (/cmd_pose stand)"
      timeout 8 ros2 topic pub /cmd_pose std_msgs/String \
        "{data: stand}" --once >/dev/null 2>&1 || true
      parked_triggered=1
    fi
    remaining_s=$((deadline - SECONDS))
    (( remaining_s > 0 )) || break
    sleep 1
  done
  die "go2w-motion ${timeout_s}s 未就绪 (需 canonical PARKED + sdk/scan ready; last=${compact:-无数据})"
}

# ============================================================
# TF 拓扑门禁: base_link 必须只有 odom 一个 parent
# ============================================================
# 当前单链由 slam_toolbox 发布 map→odom、map_odom_fuser 发布
# odom→base_link。任何遗留 static base_link parent 都会让 tf2 查询结果随
# 时间戳变化，必须在暴露导航 action 前失败并由原子发布器回滚。
check_tf_topology() {
  ros_env
  log "诊断 TF 拓扑 (critic C1: base_link 双 parent?)..."
  local body_parent odom_parent
  body_parent=$(timeout 3 ros2 topic echo --no-daemon /tf_static --once 2>/dev/null | awk '/child_frame_id: base_link/{f=1} f && /frame_id:/{print $2; exit}')
  odom_parent=$(timeout 3 ros2 topic echo --no-daemon /tf --once 2>/dev/null | awk '/child_frame_id: base_link/{f=1} f && /frame_id:/{print $2; exit}')
  if [ -n "$body_parent" ] && [ -n "$odom_parent" ]; then
    die "base_link 双 parent: /tf_static=$body_parent + /tf=$odom_parent"
  fi
  ok "base_link 单 parent (TF 拓扑 OK)"
  return 0
}

# ============================================================
# systemd transient 启动 (坑6: 必须 -p User=nx 继承 RMW 环境)
# ============================================================
# 起 fastlio/nav2 transient unit. 参数: unit名 + source命令 + working_dir
start_transient() {
  local unit=$1 src_cmd=$2 workdir=$3
  local env_args=(
    -p "User=$RUN_USER"
    -p "WorkingDirectory=$workdir"
    -p "Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    -p "Environment=ROS_DOMAIN_ID=0"
    -p "Environment=LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$FASTLIO_WS/install/fast_lio/lib:$FASTLIO_WS/install/livox_ros_driver2/lib"
    -p "Environment=DOG_INTERFACE=$DOG_INTERFACE"
    -p "Environment=GO2W_RELEASE_ID=$GO2W_RELEASE_ID"
    -p "Restart=on-failure"
    -p "RestartSec=2"
  )
  if [ "$unit" = "fastlio" ] || [ "$unit" = "mid360-nav-bridge" ]; then
    env_args+=(-p "Nice=-5")
  fi
  # --no-shm 模式注入 FASTRTPS profile (全局禁 SHM)
  if [ "$MODE_NO_SHM" -eq 1 ] && [ -f "$PROFILE_XML" ]; then
    env_args+=(-p "Environment=FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE_XML")
  fi
  sudo systemctl stop "$unit.service" 2>/dev/null || true
  sudo systemctl reset-failed "$unit.service" 2>/dev/null || true
  sudo systemd-run --unit="$unit" --collect "${env_args[@]}" \
    bash -lc "$src_cmd" \
    || die "启动 $unit 失败"
}

# ============================================================
# IMU 断流监控 (坑5: livox driver 长跑 /livox/imu 断流, restart 恢复)
# ============================================================
watch_imu() {
  ros_env
  log "启动 IMU 监控 (每 10s 查一次, 断流 restart livox-mid360-driver)..."
  while true; do
    local hz
    hz=$(topic_hz /livox/imu)
    if [ -z "$hz" ]; then
      warn "/livox/imu 断流 (坑5), restart livox-mid360-driver"
      sudo systemctl restart livox-mid360-driver
      sleep 30  # restart 后等稳定
      # critic M2: restart livox 会让 fastlio 断 /livox/* 订阅, 重验 /Odometry 防 fastlio 僵尸
      local ohz; ohz=$(topic_hz /Odometry)
      [ -z "$ohz" ] && warn "/Odometry 在 livox restart 后断流, fastlio 可能僵尸 — 建议 systemctl restart fastlio + 重跑 bringup"
    else
      if ! awk -v h="$hz" -v m="$IMU_MIN_HZ" 'BEGIN{exit !(h>=m)}'; then
        warn "/livox/imu ${hz}Hz < ${IMU_MIN_HZ}Hz, 关注"
      fi
    fi
    sleep 10
  done
}

# ---- single-participant health gates ----
# A single long-lived rclpy process owns discovery, subscriptions, TF, service
# clients and the action client. Every startup gate below reads only its atomic
# local JSON snapshot, so bringup never creates a storm of short-lived DDS
# participants while the NX is under FastLIO/SLAM load.
health_gate() {
  local timeout_s=$1
  shift
  timeout --kill-after=1 $((timeout_s + 2)) python3 "$NAV_HEALTH_GATE" \
    --state-file "$NAV_HEALTH_STATE" --timeout "$timeout_s" "$@"
}

wait_hz() {
  local topic=$1 min=$2 timeout_s=$3
  log "等 $topic >= ${min}Hz (最长 ${timeout_s}s)..."
  health_gate "$timeout_s" --rate "$topic" "$min" \
    || die "$topic ${timeout_s}s 未达 ${min}Hz"
  ok "$topic >= ${min}Hz"
}

wait_message() {
  local topic=$1 timeout_s=$2
  log "等 $topic 首帧 (最长 ${timeout_s}s)..."
  health_gate "$timeout_s" --message "$topic" \
    || die "$topic ${timeout_s}s 未收到新鲜消息"
  ok "$topic 首帧已收到"
}

wait_tf() {
  local parent=$1 child=$2 timeout_s=$3
  log "等 TF $parent → $child (最长 ${timeout_s}s)..."
  health_gate "$timeout_s" --tf "$parent" "$child" \
    || die "TF $parent → $child ${timeout_s}s 不可查"
  ok "TF $parent → $child 可查"
}

wait_active() {
  local node=$1 timeout_s=$2
  log "等 /$node active (最长 ${timeout_s}s)..."
  health_gate "$timeout_s" --lifecycle "$node" \
    || die "/$node ${timeout_s}s 未报告 active"
  ok "/$node active"
}

wait_action() {
  local action=$1 timeout_s=$2
  log "等 action $action (最长 ${timeout_s}s)..."
  health_gate "$timeout_s" --action "$action" \
    || die "action $action ${timeout_s}s 未暴露"
  ok "action $action 已暴露"
}

wait_motion_ready() {
  local timeout_s=$1
  log "等 go2w-motion canonical PARKED + SDK/scan ready (最长 ${timeout_s}s)..."
  # 2026-07-18 治本: 狗在 wheel_balance 时 motion 卡 BOOT_HOLD (设计 observation-only
  # 不自动 park, test_explicit_park_from_boot_hold_is_the_only_startup_standup_path 守护).
  # health_gate --dog-ready 永远等不到 (dog_ready 要求 physical_mode=joint_lock).
  # 循环检测 wheel_balance + session=boot_hold → 显式 pub /cmd_pose stand (PARK intent)
  # 让 motion 走 STOPPING→PARKING→StandUp→joint_lock→PARKED. 数据源用 supervisor snapshot
  # 的 dog_state (nav_health_supervisor 长驻订阅可靠, 不依赖 web DDS 断连或 echo
  # --no-daemon discovery 6s 超时, 17:12/17:19 实测 supervisor 有完整 state 但 park 块
  # 没触发, 因原版 health_gate 无循环). 幂等: parked_triggered 只发一次.
  local parked_triggered=0 dbg_count=0 deadline=$((SECONDS + timeout_s)) dog_json
  while (( SECONDS < deadline )); do
    dog_json=""
    if [[ -n "$NAV_HEALTH_STATE" && -f "$NAV_HEALTH_STATE" ]]; then
      dog_json=$(python3 -c "import json,sys; d=json.load(open('$NAV_HEALTH_STATE')); sys.stdout.write(d.get('dog_state') or '')" 2>/dev/null || true)
    fi
    (( dbg_count++ < 5 )) && log "park_dbg#${dbg_count} trig=$parked_triggered djson_len=${#dog_json} wb=$(printf '%s' "$dog_json" | grep -c wheel_balance) bh=$(printf '%s' "$dog_json" | grep -c boot_hold)"
    if (( parked_triggered == 0 )) \
        && { [[ "$dog_json" == *'"physical_mode":"wheel_balance"'* \
              || "$dog_json" == *'"physical_mode":"wheel_locomotion"'* ]]; } \
        && { [[ "$dog_json" == *'"session":"boot_hold"'* ]] \
             || [[ "$dog_json" == *'"session":"parked"'* ]] \
             || [[ "$dog_json" == *'"drive_session_phase":"boot_hold"'* ]] \
             || [[ "$dog_json" == *'"drive_session":"startup"'* ]]; }; then
      log "BOOT_HOLD/PARKED + wheel mode 检测到 (supervisor snapshot), 显式发 PARK intent (/cmd_pose stand) 把狗 park 回 joint_lock"
      timeout 8 ros2 topic pub /cmd_pose std_msgs/String "{data: stand}" --once >/dev/null 2>&1 || true
      parked_triggered=1
    fi
    if health_gate 2 --dog-ready 2>/dev/null; then
      ok "go2w-motion SDK ready + canonical PARKED + scan fresh (park_triggered=$parked_triggered)"
      return 0
    fi
    sleep 1
  done
  die "go2w-motion ${timeout_s}s 未就绪 (需 canonical PARKED; park_triggered=$parked_triggered dog_json_len=${#dog_json})"
}

check_tf_topology() {
  log "检查 base_link 单 parent..."
  health_gate 5 --single-parent base_link \
    || die "base_link 存在多个 TF parent"
  ok "base_link 单 parent"
}

# ============================================================
# 主流程
# ============================================================
main() {
  log "===== Go2W FastLIO + Nav2 3D bringup ====="
  # critic M1: 必须以 nx 用户跑；不要依赖非登录 systemd 环境中的 $USER。
  [ "$RUN_USER" = "nx" ] || die "本脚本必须以 nx 用户跑 (当前 $RUN_USER); sudo -i 下变 root 缺 RMW"
  [ -n "${DOG_INTERFACE:-}" ] || die "DOG_INTERFACE is missing from /etc/go2w/hardware.env"
  [ -n "${GO2W_RELEASE_ID:-}" ] || die "GO2W_RELEASE_ID is missing from /etc/go2w/release.env"
  ros_env  # 首次 source (后续 ros_env no-op, 解决 M6 性能)
  [ -n "$GO2W_WS" ] || die "go2w_nav install WS 未找到 (设 GO2W_WS=)"
  ok "GO2W_WS=$GO2W_WS  FASTLIO_WS=$FASTLIO_WS  NO_SHM=$MODE_NO_SHM"

  # 0. SHM 治理
  # clean_shm  # 跳过: SHM 没损坏时清了反而让 livox DDS 订阅失效断流 (坑2反效果)

  # 1. systemd 永久服务健康 (只检查, 不重启 — 重启会断 lease 狗可能摔)
  log "检查 systemd 永久服务..."
  for svc in livox-mid360-driver go2w-motion go2w-web go2w-sensor; do
    systemctl is-active --quiet "$svc" || die "$svc 未运行, 先 systemctl start $svc"
  done
  ok "MID360 driver + motion + web + bounded sensor feedback ready"

  rm -f "$NAV_HEALTH_STATE"
  log "启动单一长驻导航健康监督器..."
  start_transient nav-health-supervisor \
    "source /opt/ros/humble/setup.bash && source $GO2W_WS/install/setup.bash && python3 -u $RUNTIME_ROOT/tools/nav_health_supervisor.py --state-file $NAV_HEALTH_STATE" \
    "$RUNTIME_ROOT"
  for _ in $(seq 1 100); do
    [ -s "$NAV_HEALTH_STATE" ] && break
    sleep 0.1
  done
  [ -s "$NAV_HEALTH_STATE" ] \
    || die "导航健康监督器未生成状态快照"

  # go2w-sensor.service is the sole lowstate/SportState reader. It preserves
  # /wheel_feedback and diagnostic /wheel_odom, but publishes neither /odom
  # nor odom->base_link. Wheel integration is never a navigation pose source.
  wait_hz /wheel_odom "$WHEEL_ODOM_MIN_HZ" 30

  # FAST_LIO is the only physical-pose source. Wheel integration cannot be a
  # fallback on Go2W because balancing motion produces false displacement.
  # Raw /livox/lidar independently remains the obstacle source.
  ros_env
  [ "$FASTLIO_ENABLE" = "1" ] \
    || die "FASTLIO_ENABLE=1 is required for physical-pose navigation"
  [ -r "$FASTLIO_CONFIG/mid360.yaml" ] \
    || die "release-owned FAST_LIO config is missing: $FASTLIO_CONFIG/mid360.yaml"
  log "commission FAST_LIO latest-frame QoS/build..."
  FASTLIO_WS="$FASTLIO_WS" \
    bash "$RUNTIME_ROOT/docker/prepare_fastlio_low_latency.sh" \
    || die "FAST_LIO low-latency preparation failed"
  log "start required FastLIO pose backbone (unit=fastlio)..."
  start_transient fastlio \
    "source $FASTLIO_WS/install/setup.bash && ros2 launch fast_lio mapping.launch.py rviz:=false config_path:=$FASTLIO_CONFIG" \
    "$FASTLIO_WS"
  wait_hz /Odometry "$ODOM_MIN_HZ" 60
  log "gate raw FAST_LIO pose latency before exposing TF to Nav2..."
  health_gate 25 --stamp-age /Odometry 0.35 \
    || die "FAST_LIO pose latency is unsafe; refusing Nav2 startup"

  # Keep /livox/lidar single-reader: FAST_LIO emits a compact 1000-point,
  # best-effort body cloud and the C++ bridge converts only its latest frame.
  if systemctl is-active --quiet mid360-nav-bridge 2>/dev/null; then
    ok "mid360-nav-bridge already running"
  else
    log "start MID360 navigation bridge (pitch=${BODY_TO_BASE_PITCH}rad)..."
    start_transient mid360-nav-bridge \
      "source /opt/ros/humble/setup.bash && source $FASTLIO_WS/install/setup.bash && source $GO2W_WS/install/setup.bash && ros2 run go2w_nav mid360_nav_bridge_cpp --ros-args -p body_to_base_pitch:=$BODY_TO_BASE_PITCH" \
      "$RUNTIME_ROOT"
  fi
  wait_hz /mid360/points_nav "$SCAN_MIN_HZ" 30
  log "re-gate FAST_LIO after obstacle bridge attachment..."
  health_gate 20 --stamp-age /Odometry 0.35 \
    || die "obstacle bridge is delaying FAST_LIO; refusing SLAM/Nav2 startup"

  # 4. FastLIO TF 检查 (camera_init→body 由 FastLIO 发)
  wait_tf camera_init body 30

  # 4.5 map→odom fuser (C1 根治, 独立脚本 — go2w_bridge 非 colcon 包, 用 python3 直接跑, 跟 nx_sensor/motion 同模式)
  # Clear any cached correction from a previous divergent estimator run.
  sudo systemctl stop map-odom-fuser.service 2>/dev/null || true
  if systemctl is-active --quiet map-odom-fuser 2>/dev/null; then
    ok "map-odom-fuser 已运行, 跳过启动"
  else
    log "启动 map_odom_fuser (C1 根治 + 倾斜共轭 pitch=${BODY_TO_BASE_PITCH}rad, transient unit=map-odom-fuser)..."
    start_transient map-odom-fuser \
      "source /opt/ros/humble/setup.bash && source $FASTLIO_WS/install/setup.bash && python3 $BRIDGE_RUNTIME/map_odom_fuser.py --ros-args -p body_to_base_pitch:=$BODY_TO_BASE_PITCH -p publish_map_to_odom:=true -p use_slam_pose:=false" \
      "$RUNTIME_ROOT"
  fi
  wait_hz /odom "$ODOM_MIN_HZ" 30
  wait_tf odom base_link 30

  # Keep SLAM's raw grid separate from Nav2. SLAM often starts with the robot
  # exactly on an OccupancyGrid edge; a small localization drift then makes
  # the planner reject every goal as "start position is off the costmap".
  # Padding remains unknown (-1), preserving frontier-exploration semantics.
  log "启动地图未知边界扩展 (/map_frontier_raw -> /map_frontier)..."
  start_transient map-padding \
    "source /opt/ros/humble/setup.bash && python3 $BRIDGE_RUNTIME/map_padding_bridge.py --ros-args -p padding_m:=2.0" \
    "$RUNTIME_ROOT"

  # Persistent SLAM is a raw frontier-map source only.  FAST_LIO/fuser owns
  # map->odom so false scan matches cannot move the physical navigation frame.
  log "启动 persistent online SLAM (/scan_mid360 -> /map_frontier_raw)..."
  start_transient slam-online \
    "source /opt/ros/humble/setup.bash && source $GO2W_WS/install/setup.bash && ros2 launch go2w_nav slam_online.launch.py" \
    "$GO2W_WS"
  wait_message /map_frontier 45
  wait_tf map odom 30
  wait_hz /localization_pose 0.5 30
  timeout 20 python3 "$BRIDGE_RUNTIME/map_padding_bridge.py" \
    --check-margin 0.5 --timeout 15 \
    || die "robot is outside /map_frontier or too close to its boundary"
  # Motion is ready only after its MID360 watchdog has accepted fresh scans.
  wait_motion_ready 200

  # 5. Nav2 3D (已起则跳过)
  if systemctl is-active --quiet nav2-3d 2>/dev/null; then
    ok "nav2-3d 已运行, 跳过启动"
  else
    log "启动 Nav2 3D (transient unit=nav2-3d)..."
    start_transient nav2-3d \
      "source /opt/ros/humble/setup.bash && source $GO2W_WS/install/setup.bash && ros2 launch go2w_nav nav2_3d.launch.py" \
      "$GO2W_WS"
  fi

  # 5.5 TF 单链门禁
  wait_tf map base_link 30
  check_tf_topology

  # 6. lifecycle 等 active (坑4: bt_navigator activate 需 TF 就绪, 时序失败自动救)
  sleep 3  # 给 nav2 节点 configure 时间
  wait_active controller_server 60
  wait_active smoother_server   60
  wait_active planner_server    60
  wait_active behavior_server    60
  wait_active bt_navigator      60
  wait_active velocity_smoother 60

  # 7. 最终验证
  ros_env
  log "验证 /navigate_to_pose action..."
  wait_action /navigate_to_pose 30

  # 8. 可选 IMU 监控 (坑5) + trap 清理 (critic M5: 防孤儿 watch_imu 进程持续轮询)
  if [ "$MODE_WATCH_IMU" -eq 1 ]; then
    trap '[ -n "$WATCH_IMU_PID" ] && kill "$WATCH_IMU_PID" 2>/dev/null' EXIT INT TERM
    watch_imu & WATCH_IMU_PID=$!
    warn "IMU 监控后台运行 (PID $WATCH_IMU_PID), 脚本退出 trap 自动 kill"
  fi

  echo ""
  ok "===== FastLIO + Nav2 3D 栈就绪 ====="
  echo ""
  echo "  建图验证: ros2 topic echo /Odometry --once   (FastLIO 位姿)"
  echo "           ros2 run tf2_tools view_frames      (生成 TF 树 PDF)"
  echo "  定点移动: ros2 action send_goal /navigate_to_pose \\"
  echo "            nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0, z: 0}}}}\""
  echo "  costmap 障碍: ros2 topic echo /local_costmap/costmap_updates --once"
  echo ""
  if [ "$MODE_NO_SHM" -eq 0 ]; then
    echo "  ${C_YEL}提示${C_OFF}: 若 FastDDS 报共享内存 transport 错误，可重跑加 --no-shm；TF 双 parent 不得绕过"
  fi
}

main "$@"
