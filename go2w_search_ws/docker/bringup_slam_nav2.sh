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
#   bash bringup_slam_nav2.sh              # 默认: 清 SHM (保性能, SHM 损坏时手动重跑)
#   bash bringup_slam_nav2.sh --no-shm     # 全局禁 SHM (SHM 反复损坏时切, 点云走 UDP)
#   bash bringup_slam_nav2.sh --watch-imu  # 后台监控 /livox/imu, 断流自动 restart driver
# ============================================================
set -uo pipefail

# ---- 配置 (环境变量可覆盖) ----
FASTLIO_WS="${FASTLIO_WS:-$HOME/ws_livox}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-src/FAST_LIO_ROS2/config/mid360.yaml}"
PROFILE_XML="${PROFILE_XML:-$HOME/go2w_ws/docker/fastdds_udp.xml}"
IMU_MIN_HZ="${IMU_MIN_HZ:-50}"      # /livox/imu 断流阈值 (MID360 标称 200Hz)
LIDAR_MIN_HZ="${LIDAR_MIN_HZ:-5}"   # /livox/lidar 最低 Hz
ODOM_MIN_HZ="${ODOM_MIN_HZ:-5}"     # /Odometry (FastLIO) 最低 Hz

# go2w_nav colcon install WS 探测 (优先 env, 再常见路径)
GO2W_WS="${GO2W_WS:-}"
if [ -z "$GO2W_WS" ]; then
  for _w in "$HOME/go2w_ws" "$HOME/ros2_ws" "$HOME/dogs_ws" "$HOME/go2w_search_ws"; do
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
# 取 topic 平均 Hz (3s 采样), 失败返回空
topic_hz() {
  local topic=$1
  ros_env
  timeout 4 ros2 topic hz "$topic" 2>/dev/null | grep -oP 'average rate:\s*\K[0-9.]+' | head -1
}

# 等 topic 达 min_hz, 超时 die
wait_hz() {
  local topic=$1 min=$2 timeout=$3
  log "等 $topic >= ${min}Hz (最长 ${timeout}s)..."
  local i hz
  for ((i=1; i<=timeout; i++)); do
    hz=$(topic_hz "$topic")
    if [ -n "$hz" ]; then
      # bash 浮点比较: awk
      if awk -v h="$hz" -v m="$min" 'BEGIN{exit !(h>=m)}'; then
        ok "$topic ${hz}Hz >= ${min}Hz"; return 0
      fi
    fi
    sleep 1
  done
  die "$topic ${timeout}s 未达 ${min}Hz (last=${hz:-无数据})"
}

# 等 TF parent→child 可查
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
  die "TF $parent → $child ${timeout}s 不可查 (FastLIO 未发 TF?)"
}

# 等 lifecycle node active, 超时尝试手动 activate (不 die, 允许手动救坑4)
wait_active() {
  local node=$1 timeout=$2
  log "等 /$node active (最长 ${timeout}s)..."
  local i st
  ros_env
  for ((i=1; i<=timeout; i++)); do
    st=$(timeout 3 ros2 lifecycle get "/$node" 2>/dev/null)
    if echo "$st" | grep -q "active"; then
      ok "/$node active"; return 0
    fi
    sleep 1
  done
  warn "/$node ${timeout}s 未 active (坑4 lifecycle 时序), 尝试手动 activate"
  ros2 lifecycle set "/$node" activate 2>/dev/null || warn "手动 activate /$node 调用失败"
  sleep 2
  local st2
  st2=$(timeout 3 ros2 lifecycle get "/$node" 2>/dev/null)
  echo "$st2" | grep -q "active" \
    || die "/$node activate 后仍非 active (状态: $st2). 查 TF map→base_link / costmap 日志 — critic C1 TF 拓扑?"
}

# ============================================================
# TF 拓扑诊断 (critic C1: base_link 双 parent 检测)
# ============================================================
# nav2_3d.launch.py TF 桥发 body→base_link (static latched), nx_sensor 发 odom→base_link (动态 50Hz).
# base_link 双 parent → tf2 缓存选最新 (odom), 但 {map,camera_init,body} 与 {odom,base_link} 两棵树
# 无连接边 → costmap 查 map→base_link 报 two-trees. 根治需 map_odom_fuser 节点 (方案 Q, 下轮 GAN-Flow):
# 算 map→odom = T(camera_init→body) × inv(T(odom→base_link)) 发布, 删 TF 桥. 本脚本只诊断 warn (不 die,
# 因当前拓扑偶发能跑 — tf2 时间戳机制决定查询落 body 还是 odom parent).
check_tf_topology() {
  ros_env
  log "诊断 TF 拓扑 (critic C1: base_link 双 parent?)..."
  local body_parent odom_parent
  body_parent=$(timeout 3 ros2 topic echo /tf_static --once 2>/dev/null | awk '/child_frame_id: base_link/{f=1} f && /frame_id:/{print $2; exit}')
  odom_parent=$(timeout 3 ros2 topic echo /tf --once 2>/dev/null | awk '/child_frame_id: base_link/{f=1} f && /frame_id:/{print $2; exit}')
  if [ -n "$body_parent" ] && [ -n "$odom_parent" ]; then
    warn "base_link 双 parent: /tf_static=$body_parent + /tf=$odom_parent → costmap two-trees (critic C1)"
    warn "根治需 map_odom_fuser 节点 (方案 Q, 下轮). 当前定点移动可能失败"
    return 1
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
    -p "User=$USER"
    -p "WorkingDirectory=$workdir"
    -p "Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    -p "Environment=ROS_DOMAIN_ID=0"
    -p "Environment=LD_LIBRARY_PATH=$HOME/CycloneDDS/lib:$FASTLIO_WS/install/fast_lio/lib:$FASTLIO_WS/install/livox_ros_driver2/lib"
  )
  # --no-shm 模式注入 FASTRTPS profile (全局禁 SHM)
  if [ "$MODE_NO_SHM" -eq 1 ] && [ -f "$PROFILE_XML" ]; then
    env_args+=(-p "Environment=FASTRTPS_DEFAULT_PROFILES_FILE=$PROFILE_XML")
  fi
  sudo systemd-run --unit="$unit" --remain-after-exit "${env_args[@]}" \
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

# ============================================================
# 主流程
# ============================================================
main() {
  log "===== Go2W FastLIO + Nav2 3D bringup ====="
  # critic M1: 必须以 nx 用户跑, sudo -i/cron/ssh 非 login 下 $USER=root → -p User=$USER 缺 RMW → DDS 隐形 (坑6)
  [ "$(id -un)" = "nx" ] || die "本脚本必须以 nx 用户跑 (当前 $(id -un)); sudo -i 下变 root 缺 RMW"
  ros_env  # 首次 source (后续 ros_env no-op, 解决 M6 性能)
  [ -n "$GO2W_WS" ] || die "go2w_nav install WS 未找到 (设 GO2W_WS=)"
  ok "GO2W_WS=$GO2W_WS  FASTLIO_WS=$FASTLIO_WS  NO_SHM=$MODE_NO_SHM"

  # 0. SHM 治理
  clean_shm

  # 1. systemd 永久服务健康 (只检查, 不重启 — 重启会断 lease 狗可能摔)
  log "检查 systemd 永久服务..."
  for svc in livox-mid360-driver go2w-sensor go2w-motion go2w-web; do
    systemctl is-active --quiet "$svc" || die "$svc 未运行, 先 systemctl start $svc"
  done
  ok "4 个 systemd 永久服务就绪"

  # 2. 前置 topic 健康 (livox + sensor)
  wait_hz /livox/lidar "$LIDAR_MIN_HZ" 30
  wait_hz /livox/imu   "$IMU_MIN_HZ"    30

  # 3. FastLIO (已起则跳过, 否则 transient 起 + 等 /Odometry)
  ros_env
  if ros2 topic list 2>/dev/null | grep -q "^/Odometry$"; then
    ok "/Odometry 已存在, FastLIO 跳过启动"
  else
    log "启动 FastLIO (transient unit=fastlio)..."
    start_transient fastlio \
      "source $FASTLIO_WS/install/setup.bash && ros2 launch fast_lio mapping.launch.py config_path:=$FASTLIO_CONFIG" \
      "$FASTLIO_WS"
  fi
  wait_hz /Odometry "$ODOM_MIN_HZ" 60

  # 4. FastLIO TF 检查 (camera_init→body 由 FastLIO 发)
  wait_tf camera_init body 30

  # 4.5 map→odom fuser (C1 根治, 独立脚本 — go2w_bridge 非 colcon 包, 用 python3 直接跑, 跟 nx_sensor/motion 同模式)
  if systemctl is-active --quiet map-odom-fuser 2>/dev/null; then
    ok "map-odom-fuser 已运行, 跳过启动"
  else
    log "启动 map_odom_fuser (C1 根治, transient unit=map-odom-fuser)..."
    start_transient map-odom-fuser \
      "source /opt/ros/humble/setup.bash && source $FASTLIO_WS/install/setup.bash && python3 $HOME/go2w_ws/map_odom_fuser.py" \
      "$HOME/go2w_ws"
  fi
  wait_tf map odom 30 || die "map→odom 未发 (fuser 未就绪? 查 /livox/lidar + /Odometry 数据)"

  # 5. Nav2 3D (已起则跳过)
  if systemctl is-active --quiet nav2-3d 2>/dev/null; then
    ok "nav2-3d 已运行, 跳过启动"
  else
    log "启动 Nav2 3D (transient unit=nav2-3d)..."
    start_transient nav2-3d \
      "source /opt/ros/humble/setup.bash && source $GO2W_WS/install/setup.bash && ros2 launch go2w_nav nav2_3d.launch.py" \
      "$GO2W_WS"
  fi

  # 5.5 TF 桥就绪检查 + 拓扑诊断 (critic C1: 检测 base_link 双 parent, 不 die 但 warn)
  wait_tf map base_link 30
  check_tf_topology || warn "C1 TF 拓扑硬伤未根治, 栈起但定点移动可能 two-trees 失败"

  # 6. lifecycle 等 active (坑4: bt_navigator activate 需 TF 就绪, 时序失败自动救)
  sleep 3  # 给 nav2 节点 configure 时间
  wait_active controller_server 60
  wait_active planner_server    60
  wait_active bt_navigator      60

  # 7. 最终验证
  ros_env
  log "验证 /navigate_to_pose action..."
  ros2 action list 2>/dev/null | grep -q navigate_to_pose \
    || warn "/navigate_to_pose 未暴露 (bt_navigator 未 active?), 定点移动会超时"

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
    echo "  ${C_YEL}提示${C_OFF}: 若 costmap 报 TF two-trees (坑3), 重跑加 --no-shm 全局禁 SHM"
  fi
}

main "$@"
