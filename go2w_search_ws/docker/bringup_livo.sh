#!/bin/bash
# ============================================================
# Go2W Fast-LIVO2 + Nav2 3D 栈编排启动 (LiDAR-Inertial-Visual)
# ============================================================
# bringup_slam_nav2.sh 的 LIVO 兄弟版: 用 Fast-LIVO2 替换 FastLIO, 多一路 C13 视觉。
# 职责: 按依赖顺序起 c13_image + fastlivo2 + fuser + nav2, 每步 health gate, 失败即停。
#
# 编排 (每步 gate):
#   0. SHM 治理 + systemd 永久服务健康 (livox/sensor/motion/web)
#   1. /livox/lidar + /livox/imu 前置 (livox-mid360-driver)
#   2. ★ C13 Image 桥 (nx_c13_image_node) → /c13/image_raw [+ /c13/camera_info]
#   3. Fast-LIVO2 (替代 FastLIO) → /Odometry + camera_init→body TF
#   4. map_odom_fuser (带 20° 倾斜补偿 body_to_base_*) → map→odom TF
#   5. Nav2 3D (p2l + navigation_launch + lifecycle)
#
# 20° 倾斜处理 (memory + fastlivo2 yaml 贡献位):
#   - LiDAR↔IMU (extrinsic_R/T): 不改, MID360 模组内出厂值
#   - Camera↔LiDAR (T_lc): 在 fastlivo2_mid360_c13.yaml, 连通后标定
#   - body→base_link: map_odom_fuser 的 body_to_base_* 参数, 本脚本从 env 读注入
#     默认 BODY_TO_BASE_PITCH=-0.349 (-20° 弧度, 模组低头补偿)
#
# 用法 (NX 上, nx 用户):
#   bash bringup_livo.sh                     # 默认带 20° 倾斜补偿
#   BODY_TO_BASE_PITCH=0 bash bringup_livo.sh  # 模组水平装 (无倾斜)
#   bash bringup_livo.sh --no-shm            # 全局禁 SHM (SHM 反复损坏时)
# ============================================================
set -uo pipefail

# ---- 配置 (环境变量可覆盖) ----
WS_LIVOX="${WS_LIVOX:-$HOME/ws_livox}"
GO2W_WS="${GO2W_WS:-$HOME/go2w_ws}"
LIVO_CONFIG="${LIVO_CONFIG:-src/FAST_LIVO2/config/fastlivo2_mid360_c13.yaml}"
C13_INTRINSIC="${C13_INTRINSIC:-${GO2W_WS}/src/go2w_nav/config/c13_intrinsic.yaml}"
PROFILE_XML="${PROFILE_XML:-$HOME/go2w_ws/docker/fastdds_udp.xml}"
# ★ 20° 倾斜补偿 (rad). -0.349 = -20° (模组向下俯仰). 水平装设 0.
BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:- -0.349}"
BODY_TO_BASE_ROLL="${BODY_TO_BASE_ROLL:-0.0}"
BODY_TO_BASE_YAW="${BODY_TO_BASE_YAW:-0.0}"
BODY_TO_BASE_X="${BODY_TO_BASE_X:-0.0}"
BODY_TO_BASE_Y="${BODY_TO_BASE_Y:-0.0}"
BODY_TO_BASE_Z="${BODY_TO_BASE_Z:-0.15}"   # 模组装底盘上方 15cm (默认, 实测改)

IMU_MIN_HZ="${IMU_MIN_HZ:-50}"
LIDAR_MIN_HZ="${LIDAR_MIN_HZ:-5}"
IMG_MIN_HZ="${IMG_MIN_HZ:-10}"     # C13 可见光稳态 ~19-30fps (memory c13-fps-steady-19), 10 为健康下限
ODOM_MIN_HZ="${ODOM_MIN_HZ:-5}"

# ---- 参数解析 ----
MODE_NO_SHM=0
for _a in "$@"; do
  case "$_a" in
    --no-shm) MODE_NO_SHM=1 ;;
    *) echo "未知参数: $_a" >&2 ;;
  esac
done
unset _a

# ---- 颜色日志 ----
C_GREEN=$'\033[32m'; C_RED=$'\033[31m'; C_YEL=$'\033[33m'; C_OFF=$'\033[0m'
log()  { echo "[$(date +%H:%M:%S)] $*"; }
ok()   { echo "${C_GREEN}[OK]${C_OFF} $*"; }
warn() { echo "${C_YEL}[WARN]${C_OFF} $*"; }
die()  { echo "${C_RED}[FAIL]${C_OFF} $*"; exit 1; }

# ---- ROS 环境 (首次 source, 后续 no-op) ----
_ROS_ENV_DONE=0
ros_env() {
  [ "$_ROS_ENV_DONE" = "1" ] && return 0
  source /opt/ros/humble/setup.bash
  [ -f "$WS_LIVOX/install/setup.bash" ] && source "$WS_LIVOX/install/setup.bash"
  [ -n "$GO2W_WS" ] && [ -f "$GO2W_WS/install/setup.bash" ] && source "$GO2W_WS/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  _ROS_ENV_DONE=1
}

# ============================================================
# SHM 治理 (跟 bringup_slam_nav2.sh 同, 坑2)
# ============================================================
clean_shm() {
  local n
  n=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    sudo rm -f /dev/shm/fastrtps_* && ok "清 $n 个 FastRTPS SHM segment"
  else
    ok "无残留 FastRTPS SHM segment"
  fi
}

# ============================================================
# 健康检查 gate (跟 bringup_slam_nav2.sh 同套)
# ============================================================
topic_hz() {
  local topic=$1
  ros_env
  timeout 4 ros2 topic hz "$topic" 2>/dev/null | grep -oP 'average rate:\s*\K[0-9.]+' | head -1
}

wait_hz() {
  local topic=$1 min=$2 timeout=$3
  log "等 $topic >= ${min}Hz (最长 ${timeout}s)..."
  local i hz
  for ((i=1; i<=timeout; i++)); do
    hz=$(topic_hz "$topic")
    if [ -n "$hz" ]; then
      if awk -v h="$hz" -v m="$min" 'BEGIN{exit !(h>=m)}'; then
        ok "$topic ${hz}Hz >= ${min}Hz"; return 0
      fi
    fi
    sleep 1
  done
  die "$topic ${timeout}s 未达 ${min}Hz (last=${hz:-无数据})"
}

wait_tf() {
  local parent=$1 child=$2 timeout=$3
  log "等 TF $parent → $child (最长 ${timeout}s)..."
  local i
  ros_env
  for ((i=1; i<=timeout; i++)); do
    if timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null | grep -q "At time"; then
      ok "TF $parent → $child 可查"; return 0
    fi
    sleep 1
  done
  die "TF $parent → $child ${timeout}s 不可查"
}

# systemd transient 启动 (坑6: 必须 -p User=nx 继承 RMW)
start_transient() {
  local unit=$1 src_cmd=$2 workdir=$3
  local env_args=(
    -p "User=$USER"
    -p "WorkingDirectory=$workdir"
    -p "Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    -p "Environment=ROS_DOMAIN_ID=0"
    -p "Environment=LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$WS_LIVOX/install/fast_lio/lib:$WS_LIVOX/install/fast_livo/lib:$WS_LIVOX/install/livox_ros_driver2/lib"
  )
  if [ "$MODE_NO_SHM" -eq 1 ] && [ -f "$PROFILE_XML" ]; then
    env_args+=(-p "Environment=FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE_XML")
  fi
  sudo systemd-run --unit="$unit" --remain-after-exit "${env_args[@]}" \
    bash -lc "$src_cmd" \
    || die "启动 $unit 失败"
}

# ============================================================
# 主流程
# ============================================================
main() {
  log "===== Go2W Fast-LIVO2 + Nav2 3D bringup ====="
  [ "$(id -un)" = "nx" ] || die "本脚本必须以 nx 用户跑 (当前 $(id -un)); sudo -i 下变 root 缺 RMW"
  ros_env
  [ -n "$GO2W_WS" ] || die "GO2W_WS 未找到 (设 GO2W_WS=)"
  ok "GO2W_WS=$GO2W_WS  WS_LIVOX=$WS_LIVOX  NO_SHM=$MODE_NO_SHM"
  ok "20° 倾斜补偿: pitch=${BODY_TO_BASE_PITCH} roll=${BODY_TO_BASE_ROLL} yaw=${BODY_TO_BASE_YAW} xyz=(${BODY_TO_BASE_X},${BODY_TO_BASE_Y},${BODY_TO_BASE_Z})"

  # 0. SHM 治理
  clean_shm

  # 1. systemd 永久服务健康
  log "检查 systemd 永久服务..."
  for svc in livox-mid360-driver go2w-sensor go2w-motion go2w-web; do
    systemctl is-active --quiet "$svc" || die "$svc 未运行, 先 systemctl start $svc"
  done
  ok "4 个 systemd 永久服务就绪"

  # 2. 前置 topic 健康 (livox)
  wait_hz /livox/lidar "$LIDAR_MIN_HZ" 30
  wait_hz /livox/imu   "$IMU_MIN_HZ"    30

  # 3. ★ C13 Image 桥 (LIVO 新增, FastLIO 版无此步)
  ros_env
  if ros2 topic list 2>/dev/null | grep -q "^/c13/image_raw$"; then
    ok "/c13/image_raw 已存在, c13-image 跳过启动"
  else
    log "启动 C13 Image 桥 (transient unit=c13-image)..."
    [ -f "$GO2W_WS/web/nx_c13_image_node.py" ] \
      || die "$GO2W_WS/web/nx_c13_image_node.py 不存在, 先跑 deploy_fastlivo2.sh"
    start_transient c13-image \
      "source /opt/ros/humble/setup.bash && source $WS_LIVOX/install/setup.bash && \
       C13_INTRINSIC_YAML=$C13_INTRINSIC \
       python3 -u $GO2W_WS/web/nx_c13_image_node.py" \
      "$GO2W_WS"
  fi
  wait_hz /c13/image_raw "$IMG_MIN_HZ" 30

  # 4. Fast-LIVO2 (替代 FastLIO)
  ros_env
  if ros2 topic list 2>/dev/null | grep -q "^/Odometry$"; then
    ok "/Odometry 已存在, Fast-LIVO2 跳过启动"
  else
    log "启动 Fast-LIVO2 (transient unit=fastlivo2)..."
    [ -d "$WS_LIVOX/install/fast_livo" ] \
      || die "$WS_LIVOX/install/fast_livo 不存在, 先跑 deploy_fastlivo2.sh (编译 Fast-LIVO2 ROS2 移植)"
    # 注意: 不同 ROS2 移植的 launch 名/参数名略异。官方 FAST_LIVO2 用 mapping.launch.py +
    # config_path:=。若移植改名, 连通后在此 sed 对齐 (runbook 记)。
    start_transient fastlivo2 \
      "source $WS_LIVOX/install/setup.bash && ros2 launch fast_livo mapping.launch.py config_path:=$LIVO_CONFIG" \
      "$WS_LIVOX"
  fi
  wait_hz /Odometry "$ODOM_MIN_HZ" 60

  # 5. Fast-LIVO2 TF
  wait_tf camera_init body 30

  # 6. map→odom fuser (带 20° 倾斜补偿 — body_to_base_* 参数注入)
  if systemctl is-active --quiet map-odom-fuser 2>/dev/null; then
    ok "map-odom-fuser 已运行, 跳过启动 (注意: 若旧实例无倾斜参数, restart 它)"
  else
    log "启动 map_odom_fuser (带 20° 倾斜补偿, transient unit=map-odom-fuser)..."
    # 注意: go2w_bridge 若未 colcon install, 用 python3 直接跑 (跟 bringup_slam_nav2.sh 同范式)
    FUSER_PY="$GO2W_WS/src/go2w_bridge/go2w_bridge/map_odom_fuser.py"
    [ -f "$FUSER_PY" ] || FUSER_PY="$GO2W_WS/map_odom_fuser.py"
    [ -f "$FUSER_PY" ] || die "map_odom_fuser.py 未找到 (查 $GO2W_WS/src/go2w_bridge 或 $GO2W_WS)"
    start_transient map-odom-fuser \
      "source /opt/ros/humble/setup.bash && source $WS_LIVOX/install/setup.bash && \
       python3 $FUSER_PY --ros-args \
         -p body_to_base_pitch:=$BODY_TO_BASE_PITCH \
         -p body_to_base_roll:=$BODY_TO_BASE_ROLL \
         -p body_to_base_yaw:=$BODY_TO_BASE_YAW \
         -p body_to_base_x:=$BODY_TO_BASE_X \
         -p body_to_base_y:=$BODY_TO_BASE_Y \
         -p body_to_base_z:=$BODY_TO_BASE_Z" \
      "$GO2W_WS"
  fi
  wait_tf map odom 30 || die "map→odom 未发 (fuser 未就绪? 查 /livox/lidar + /Odometry)"

  # 7. Nav2 3D (跟 bringup_slam_nav2.sh 完全一致 — LIVO 出口同 FastLIO)
  if systemctl is-active --quiet nav2-3d 2>/dev/null; then
    ok "nav2-3d 已运行, 跳过启动"
  else
    log "启动 Nav2 3D (transient unit=nav2-3d)..."
    start_transient nav2-3d \
      "source /opt/ros/humble/setup.bash && source $GO2W_WS/install/setup.bash && ros2 launch go2w_nav nav2_3d.launch.py" \
      "$GO2W_WS"
  fi
  wait_tf map base_link 30

  # 8. 验证 LIVO 视觉约束生效判据 (视觉辅助后轨迹应比纯 LIO 光滑)
  echo ""
  ok "===== Fast-LIVO2 + Nav2 3D 栈就绪 ====="
  echo ""
  echo "  LIVO 位姿:      ros2 topic echo /Odometry --once"
  echo "  C13 视觉流:     ros2 topic hz /c13/image_raw   (应 ~19-30fps)"
  echo "  视觉约束判据:   对比 /Odometry 在静止 30s 的漂移"
  echo "                  - 纯 LIO (关 img): ros2 topic echo /Odometry (漂移大)"
  echo "                  - LIVO (开 img):   漂移应明显更小 (视觉约束拉住)"
  echo "  TF 树:          ros2 run tf2_tools view_frames"
  echo "  定点移动:       ros2 action send_goal /navigate_to_pose \\"
  echo "                  nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0, z: 0}}}}\""
  echo ""
  if [ "$MODE_NO_SHM" = "0" ]; then
    echo "  ${C_YEL}提示${C_OFF}: 若 costmap 报 TF two-trees, 重跑加 --no-shm 全局禁 SHM"
  fi
  echo ""
  echo "  ${C_YEL}首次连通必做${C_OFF}: 相机内参标定 + T_lc 外证实测 (见 docs/fastlivo2_runbook.md §3)"
}

main "$@"
