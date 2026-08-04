#!/bin/bash
# Safely deploy the MID360-primary FastLIO/Nav2 stack to the NX.
set -euo pipefail

# Compatibility entrypoint only. This preserves the familiar command name
# while guaranteeing Nav2 deployment never overwrites or restarts motion.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "deploy_nav2_bprime.sh is retired; forwarding to the atomic nav release flow" >&2
artifact="$("$SCRIPT_DIR/build_release.sh" nav)"
exec "$SCRIPT_DIR/deploy_release.sh" "$artifact" "$@"

# LEGACY IMPLEMENTATION BELOW IS UNREACHABLE

NX_HOST="${NX_HOST:-192.168.1.104}"
NX_USER="${NX_USER:-nx}"
WIN_WS="${WIN_WS:-/c/Users/ROG/yangyuhui/DOGS/go2w_search_ws}"
NX_WS="/home/nx/go2w_ws"
SSH=(ssh -o BatchMode=yes "$NX_USER@$NX_HOST")

echo "[1/7] Quiesce Nav2 components only"
"${SSH[@]}" "if systemctl cat go2w-slam-nav.service >/dev/null 2>&1; then
    sudo -n systemctl stop go2w-slam-nav.service;
  fi
  for unit in nav2-3d slam-online map-padding mid360-nav-bridge map-odom-fuser fastlio; do
    if systemctl cat \"\$unit.service\" >/dev/null 2>&1; then
      sudo -n systemctl stop \"\$unit.service\" || exit 1
    fi
  done"

TS=$("${SSH[@]}" date +%s)
echo "[2/7] Backup current NX payloads (.bak.$TS)"
"${SSH[@]}" "cd $NX_WS
  for file in map_odom_fuser.py map_padding_bridge.py mid360_nav_bridge.py \
      bringup_slam_nav2.sh fastdds_udp.xml; do
    [ ! -e \"\$file\" ] || cp \"\$file\" \"\$file.bak.$TS\"
  done
  for file in src/go2w_nav/config/nav2_params_3d.yaml \
      install/go2w_nav/share/go2w_nav/config/nav2_params_3d.yaml \
      src/go2w_nav/config/slam_toolbox_online.yaml \
      install/go2w_nav/share/go2w_nav/config/slam_toolbox_online.yaml \
      src/go2w_nav/launch/nav2_3d.launch.py \
      install/go2w_nav/share/go2w_nav/launch/nav2_3d.launch.py \
      src/go2w_nav/launch/slam_online.launch.py \
      install/go2w_nav/share/go2w_nav/launch/slam_online.launch.py \
      src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml \
      install/go2w_nav/share/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml \
      web/costmap_bridge.py; do
    [ ! -e \"\$file\" ] || cp \"\$file\" \"\$file.bak.$TS\"
  done
  sudo -n sh -c '[ ! -e /etc/systemd/system/go2w-slam-nav.service ] || \
    cp /etc/systemd/system/go2w-slam-nav.service \
       /etc/systemd/system/go2w-slam-nav.service.bak.$TS'
  sudo -n sh -c '[ ! -e /etc/systemd/system/livox-mid360-watchdog.service ] || \
    cp /etc/systemd/system/livox-mid360-watchdog.service \
       /etc/systemd/system/livox-mid360-watchdog.service.bak.$TS'
  sudo -n sh -c '[ ! -e /usr/local/lib/go2w/livox_stream_watchdog.py ] || \
    cp /usr/local/lib/go2w/livox_stream_watchdog.py \
       /usr/local/lib/go2w/livox_stream_watchdog.py.bak.$TS'"

echo "[3/7] Upload exact payloads"
"${SSH[@]}" "rm -rf /tmp/bprime && mkdir -p /tmp/bprime"
scp -o BatchMode=yes -o StrictHostKeyChecking=no \
  "$WIN_WS/src/go2w_bridge/go2w_bridge/map_odom_fuser.py" \
  "$WIN_WS/src/go2w_bridge/go2w_bridge/map_padding_bridge.py" \
  "$WIN_WS/src/go2w_bridge/go2w_bridge/mid360_nav_bridge.py" \
  "$WIN_WS/src/go2w_nav/config/nav2_params_3d.yaml" \
  "$WIN_WS/src/go2w_nav/config/slam_toolbox_online.yaml" \
  "$WIN_WS/src/go2w_nav/launch/nav2_3d.launch.py" \
  "$WIN_WS/src/go2w_nav/launch/slam_online.launch.py" \
  "$WIN_WS/src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml" \
  "$WIN_WS/web/costmap_bridge.py" \
  "$WIN_WS/docker/bringup_slam_nav2.sh" \
  "$WIN_WS/docker/fastdds_udp.xml" \
  "$WIN_WS/docker/go2w-slam-nav.service" \
  "$WIN_WS/docker/livox-mid360-watchdog.service" \
  "$WIN_WS/tools/livox_stream_watchdog.py" \
  "$WIN_WS/tools/nav2_preflight.py" \
  "$WIN_WS/tools/probe_angular_response.py" \
  "$NX_USER@$NX_HOST:/tmp/bprime/"

echo "[4/7] Install root/src/install/systemd copies"
"${SSH[@]}" "cd $NX_WS
  mkdir -p tools
  cp /tmp/bprime/map_odom_fuser.py .
  cp /tmp/bprime/map_padding_bridge.py .
  cp /tmp/bprime/mid360_nav_bridge.py .
  cp /tmp/bprime/nav2_params_3d.yaml src/go2w_nav/config/
  cp /tmp/bprime/nav2_params_3d.yaml install/go2w_nav/share/go2w_nav/config/
  cp /tmp/bprime/slam_toolbox_online.yaml src/go2w_nav/config/
  cp /tmp/bprime/slam_toolbox_online.yaml install/go2w_nav/share/go2w_nav/config/
  cp /tmp/bprime/nav2_3d.launch.py src/go2w_nav/launch/
  cp /tmp/bprime/nav2_3d.launch.py install/go2w_nav/share/go2w_nav/launch/
  cp /tmp/bprime/slam_online.launch.py src/go2w_nav/launch/
  cp /tmp/bprime/slam_online.launch.py install/go2w_nav/share/go2w_nav/launch/
  mkdir -p src/go2w_nav/behavior_trees install/go2w_nav/share/go2w_nav/behavior_trees
  cp /tmp/bprime/navigate_to_pose_dynamic_safe.xml src/go2w_nav/behavior_trees/
  cp /tmp/bprime/navigate_to_pose_dynamic_safe.xml install/go2w_nav/share/go2w_nav/behavior_trees/
  cp /tmp/bprime/costmap_bridge.py web/
  cp /tmp/bprime/bringup_slam_nav2.sh /tmp/bprime/fastdds_udp.xml .
  cp /tmp/bprime/nav2_preflight.py tools/
  cp /tmp/bprime/probe_angular_response.py tools/
  chmod 755 tools/nav2_preflight.py tools/probe_angular_response.py
  chmod 775 bringup_slam_nav2.sh
  sudo -n cp /tmp/bprime/go2w-slam-nav.service /etc/systemd/system/
  sudo -n chmod 644 /etc/systemd/system/go2w-slam-nav.service
  sudo -n install -D -o root -g root -m 755 /tmp/bprime/livox_stream_watchdog.py /usr/local/lib/go2w/livox_stream_watchdog.py
  sudo -n install -o root -g root -m 644 /tmp/bprime/livox-mid360-watchdog.service /etc/systemd/system/livox-mid360-watchdog.service
  sudo -n systemctl daemon-reload
  sudo -n systemctl enable go2w-slam-nav.service
  sudo -n systemctl enable livox-mid360-watchdog.service"

echo "[5/7] Verify installed bytes and primary-mode markers"
"${SSH[@]}" "cd $NX_WS
  cmp -s /tmp/bprime/map_odom_fuser.py map_odom_fuser.py
  cmp -s /tmp/bprime/map_padding_bridge.py map_padding_bridge.py
  cmp -s /tmp/bprime/mid360_nav_bridge.py mid360_nav_bridge.py
  cmp -s /tmp/bprime/nav2_preflight.py tools/nav2_preflight.py
  cmp -s /tmp/bprime/probe_angular_response.py tools/probe_angular_response.py
  sudo -n cmp -s /tmp/bprime/livox_stream_watchdog.py /usr/local/lib/go2w/livox_stream_watchdog.py
  sudo -n cmp -s /tmp/bprime/livox-mid360-watchdog.service /etc/systemd/system/livox-mid360-watchdog.service
  cmp -s src/go2w_nav/config/nav2_params_3d.yaml \
    install/go2w_nav/share/go2w_nav/config/nav2_params_3d.yaml
  cmp -s src/go2w_nav/launch/nav2_3d.launch.py \
    install/go2w_nav/share/go2w_nav/launch/nav2_3d.launch.py
  cmp -s src/go2w_nav/config/slam_toolbox_online.yaml \
    install/go2w_nav/share/go2w_nav/config/slam_toolbox_online.yaml
  cmp -s src/go2w_nav/launch/slam_online.launch.py \
    install/go2w_nav/share/go2w_nav/launch/slam_online.launch.py
  cmp -s src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml \
    install/go2w_nav/share/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml
  cmp -s /tmp/bprime/costmap_bridge.py web/costmap_bridge.py
  grep -F -q '/cloud_registered_body' mid360_nav_bridge.py
  grep -F -q 'DurabilityPolicy.TRANSIENT_LOCAL' map_padding_bridge.py
  grep -F -q '("map", "/map_frontier_raw")' \
    install/go2w_nav/share/go2w_nav/launch/slam_online.launch.py
  grep -F -q 'map_topic: "/map_frontier"' \
    install/go2w_nav/share/go2w_nav/config/nav2_params_3d.yaml
  grep -F -q '<RateController hz="2.0">' \
    install/go2w_nav/share/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml
  grep -F -q '<RecoveryNode number_of_retries="1"' \
    install/go2w_nav/share/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml
  grep -F -q '<Wait wait_duration="1"/>' \
    install/go2w_nav/share/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml
  grep -F -q 'sudo systemctl stop go2w-sensor.service' bringup_slam_nav2.sh"

echo "[6/7] Restart and gate the persistent Nav2 stack"
"${SSH[@]}" "sudo -n systemctl stop go2w-sensor.service
  if ! sudo -n systemctl restart go2w-slam-nav.service; then
    sudo -n systemctl stop go2w-slam-nav.service || true
    exit 1
  fi
  sudo -n systemctl restart livox-mid360-watchdog.service
  systemctl is-active --quiet livox-mid360-watchdog.service"

echo "[7/7] Verify live MID360/TF/Nav2 chain"
"${SSH[@]}" "source /opt/ros/humble/setup.bash
  source /home/nx/ws_livox/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0
  check_hz() {
    local topic=\$1 minimum=\$2 output hz
    output=\$(timeout 8 ros2 topic hz \"\$topic\" 2>/dev/null || true)
    hz=\$(awk '/average rate:/{v=\$3} END{print v}' <<<\"\$output\")
    awk -v h=\"\$hz\" -v m=\"\$minimum\" 'BEGIN{exit !(h+0>=m)}'
  }
  check_hz /mid360/points_nav 5
  check_hz /odom 5
  timeout 15 ros2 topic echo --once /map_frontier --field info >/dev/null
  check_hz /localization_pose 0.5
  timeout 20 python3 /home/nx/go2w_ws/map_padding_bridge.py \
    --check-margin 0.5 --timeout 15
  DOG_STATE=\$(timeout 8 ros2 topic echo /dog_state --once --field data 2>/dev/null)
  tr -d '[:space:]' <<<\"\$DOG_STATE\" | grep -Fq '\"nav_scan_fresh\":true'
  timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation
  ros2 action list | grep -Fxq /navigate_to_pose
  systemctl is-active --quiet go2w-slam-nav.service
  systemctl is-active --quiet go2w-web.service
  systemctl is-active --quiet livox-mid360-watchdog.service
  ! systemctl is-active --quiet go2w-sensor.service || exit 1" || {
    "${SSH[@]}" "sudo -n systemctl stop go2w-slam-nav.service || true"
    echo "Live gate failed; autonomous stack stopped." >&2
    exit 1
  }

echo "Deployment passed. Backup timestamp: $TS"
