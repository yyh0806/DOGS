import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_deploy_web_copies_lidar_and_gimbal_components():
    script = read("docker/deploy_nx_web.sh")
    assert 'web/nx_lidar_node.py' in script
    assert 'web/nx_gimbal_node.py' in script
    assert 'web/nx_slam_map.py' in script


def test_deploy_web_copies_global_search_state_import_dependency():
    script = read("docker/deploy_nx_web.sh")

    assert 'web/nx_global_search_state.py' in script


def test_deploy_web_installs_livox_driver_services():
    script = read("docker/deploy_nx_web.sh")
    assert 'docker/livox-mid360-net.service' in script
    assert 'docker/livox-mid360-driver.service' in script
    assert 'systemctl enable livox-mid360-net.service livox-mid360-driver.service' in script


def test_livox_driver_service_launches_mid360_driver_after_network():
    service = read("docker/livox-mid360-driver.service")
    assert 'Requires=livox-mid360-net.service' in service
    assert 'After=livox-mid360-net.service' in service
    assert 'source /opt/ros/humble/setup.bash' in service
    assert 'source /home/nx/ws_livox/install/setup.bash' in service
    assert 'ros2 launch livox_ros_driver2 msg_MID360_launch.py' in service
    assert 'Restart=always' in service


def test_livox_network_service_uses_detected_interface_and_retries():
    service = read("docker/livox-mid360-net.service")

    assert "Environment=LIVOX_INTERFACE=enx207bd2edf780" in service
    assert "EnvironmentFile=-/etc/go2w/hardware.env" in service
    assert 'sys/class/net/${LIVOX_INTERFACE}' in service
    assert 'ip link set "$LIVOX_INTERFACE" up' in service
    assert 'ip addr replace 192.168.1.200/32 dev "$LIVOX_INTERFACE"' in service
    assert 'ip route replace 192.168.1.160/32 dev "$LIVOX_INTERFACE"' in service
    assert "Restart=on-failure" in service
    assert "done; exit 0" not in service


def test_livox_driver_waits_for_valid_wall_clock_before_rate_limiter_starts():
    service = read("docker/livox-mid360-driver.service")

    assert "timedatectl show -p NTPSynchronized --value" in service
    assert '" = "yes"' in service
    assert "NTP not synchronized in 300s" in service
    assert (
        "Environment=FASTRTPS_DEFAULT_PROFILES_FILE="
        "/home/nx/go2w/current/payload/docker/fastdds_udp.xml"
        in service
    )


def test_web_service_tolerates_missing_livox_workspace():
    service = read("docker/go2w-web.service")
    script = read("web/start_go2w_web.sh")

    assert '/home/nx/go2w/current/payload/web/start_go2w_web.sh' in service
    assert 'if [ -f /home/nx/ws_livox/install/setup.bash ]; then' in script
    assert 'source /home/nx/ws_livox/install/setup.bash' in script


def test_web_start_script_keeps_livox_msg_typesupport_after_ld_cleanup():
    script = read("web/start_go2w_web.sh")

    cleanup_index = script.index("grep -v 'ws_livox'")
    restore_index = script.index("for _pkg in livox_ros_driver2 fast_lio; do")
    assert cleanup_index < restore_index

    restore_loop = script[restore_index:script.index("unset _pkg _d", restore_index)]
    assert "_d=/home/nx/ws_livox/install/$_pkg/lib" in restore_loop
    assert '[ -d "$_d" ] && export LD_LIBRARY_PATH="$_d:$LD_LIBRARY_PATH"' in restore_loop


def test_slam_nav_bringup_wait_tf_uses_one_bounded_streaming_check():
    script = read("docker/bringup_slam_nav2.sh")
    wait_tf = script.split("wait_tf() {", 1)[1].split("\n}\n", 1)[0]

    assert "setsid stdbuf -oL ros2 run tf2_ros tf2_echo" in wait_tf
    assert 'grep -q "At time" "$output_file"' in wait_tf
    assert 'local deadline=$((SECONDS + timeout_s))' in wait_tf
    assert 'kill -TERM -- "-$tf_pid"' in wait_tf
    assert 'kill -KILL -- "-$tf_pid"' in wait_tf
    assert "for ((i=1; i<=timeout; i++))" not in wait_tf


def test_slam_nav_bringup_uses_root_profile_pitch_and_restartable_units():
    script = read("docker/bringup_slam_nav2.sh")
    start_transient = script.split("start_transient() {", 1)[1].split("\n}\n", 1)[0]

    assert 'PROFILE_XML="${PROFILE_XML:-$RUNTIME_ROOT/docker/fastdds_udp.xml}"' in script
    assert 'BODY_TO_BASE_PITCH="${BODY_TO_BASE_PITCH:--0.3490658504}"' in script
    assert '-p "Restart=on-failure"' in start_transient
    assert '-p "RestartSec=2"' in start_transient


def test_slam_nav_service_orders_prerequisites_and_persists_bringup():
    service = read("docker/go2w-slam-nav.service")

    prerequisites = "livox-mid360-driver.service go2w-motion.service"
    assert "Wants=network-online.target go2w-web.service" in service
    assert "Wants=go2w-motion.service" in service
    # A watchdog restart of a stalled Livox process must not tear down the
    # in-memory SLAM graph and reset the map.  Ordering is required, lifetime
    # coupling is not.
    assert "Wants=livox-mid360-driver.service" in service
    assert "Requires=livox-mid360-driver.service" not in service
    assert f"Requires={prerequisites}" not in service
    assert f"After={prerequisites}" in service
    assert "Type=oneshot" in service
    assert "User=nx" in service
    assert "Environment=HOME=/home/nx" in service
    assert "Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in service
    assert "Environment=ROS_DOMAIN_ID=0" in service
    assert "WorkingDirectory=/home/nx/go2w/current/payload" in service
    assert (
        "ExecStart=/bin/bash /home/nx/go2w/current/payload/docker/bringup_slam_nav2.sh --no-shm"
        in service
    )
    assert (
        "ExecStopPost=-+/usr/bin/systemctl stop nav2-3d.service "
        "slam-online.service mid360-nav-bridge.service "
        "map-odom-fuser.service fastlio.service"
    ) in service
    assert "RemainAfterExit=yes" in service
    assert "Restart=on-failure" in service
    assert "RestartSec=10" in service
    assert "TimeoutStartSec=600" in service
    assert "TimeoutStopSec=30" in service
    assert "WantedBy=multi-user.target" in service


def test_nav2_deploy_installs_persistent_bringup_assets():
    script = read("docker/deploy_nav2_bprime.sh")

    assert "docker/go2w-slam-nav.service" in script
    assert "docker/bringup_slam_nav2.sh" in script
    assert "docker/fastdds_udp.xml" in script
    assert "src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml" in script
    for target in ("map_odom_fuser.py", "map_padding_bridge.py", "mid360_nav_bridge.py",
                   "nav2_params_3d.yaml",
                   "nav2_3d.launch.py", "slam_online.launch.py",
                   "slam_toolbox_online.yaml", "costmap_bridge.py",
                   "nav2_preflight.py", "probe_angular_response.py"):
        assert target in script
    assert "nx_motion_node.py" not in script
    assert "nx_web_server.py" not in script
    assert ".bak.$TS" in script
    assert "cp /tmp/bprime/mid360_nav_bridge.py ." in script
    assert "cp /tmp/bprime/map_padding_bridge.py ." in script
    assert "cp /tmp/bprime/nav2_params_3d.yaml src/go2w_nav/config/" in script
    assert "cp /tmp/bprime/nav2_params_3d.yaml install/go2w_nav/share/go2w_nav/config/" in script
    assert "cp /tmp/bprime/nav2_3d.launch.py src/go2w_nav/launch/" in script
    assert "cp /tmp/bprime/nav2_3d.launch.py install/go2w_nav/share/go2w_nav/launch/" in script
    assert "cp /tmp/bprime/slam_online.launch.py install/go2w_nav/share/go2w_nav/launch/" in script
    assert "cp /tmp/bprime/slam_toolbox_online.yaml install/go2w_nav/share/go2w_nav/config/" in script
    assert (
        "cp /tmp/bprime/navigate_to_pose_dynamic_safe.xml "
        "install/go2w_nav/share/go2w_nav/behavior_trees/"
    ) in script
    assert "cp /tmp/bprime/costmap_bridge.py web/" in script
    assert "cp /tmp/bprime/bringup_slam_nav2.sh /tmp/bprime/fastdds_udp.xml ." in script
    assert "chmod 775 bringup_slam_nav2.sh" in script
    assert "sudo -n cp /tmp/bprime/go2w-slam-nav.service /etc/systemd/system/" in script
    assert "sudo -n systemctl daemon-reload" in script


def test_nav2_deploy_verifies_payload_and_documents_persistent_owner():
    script = read("docker/deploy_nav2_bprime.sh")

    assert "transform_tolerance: 2.0" not in script
    assert "cmp -s src/go2w_nav/config/nav2_params_3d.yaml" in script
    assert "cmp -s src/go2w_nav/launch/nav2_3d.launch.py" in script
    assert "cmp -s src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml" in script
    assert '<RecoveryNode number_of_retries="1"' in script
    assert '<Wait wait_duration="1"/>' in script
    assert "cmp -s /tmp/bprime/map_padding_bridge.py map_padding_bridge.py" in script
    assert "--check-margin 0.5" in script
    assert "grep -F -q '/cloud_registered_body' mid360_nav_bridge.py" in script
    assert "grep -F -q 'map_topic: \"/map_frontier\"'" in script
    assert "systemctl is-active --quiet go2w-slam-nav.service" in script

    assert "transient/session" not in script
    assert "pkill -9 -f" not in script
    assert "bash ~/go2w_ws/bringup_slam_nav2.sh" not in script
    assert "systemctl restart go2w-slam-nav.service" in script
    assert "go2w-slam-nav.service.bak.$TS" in script


def test_slam_nav_wait_active_uses_read_only_get_state_probe():
    script = read("docker/bringup_slam_nav2.sh")
    wait_active = script.split("wait_active() {", 1)[1].split("\n}\n", 1)[0]

    assert 'tools/wait_lifecycle_active.py' in wait_active
    assert '--timeout "$timeout_s" "$node"' in wait_active
    assert 'timeout --kill-after=2 "$outer_timeout" python3' in wait_active
    assert "ros2 lifecycle" not in wait_active
    assert "lifecycle set" not in wait_active


def test_nav2_deploy_restarts_persistent_owner_after_install():
    script = read("docker/deploy_nav2_bprime.sh")

    assert "sudo -n systemctl enable go2w-slam-nav.service" in script
    assert "sudo -n systemctl restart go2w-slam-nav.service" in script
    assert "sudo -n systemctl restart go2w-web.service" not in script
    assert "sudo -n systemctl restart go2w-motion.service" not in script
    assert "systemctl enable --now go2w-slam-nav.service" not in script


def test_slam_nav_transients_are_collectable_and_cleanup_after_failure():
    script = read("docker/bringup_slam_nav2.sh")
    service = read("docker/go2w-slam-nav.service")
    start_transient = script.split("start_transient() {", 1)[1].split("\n}\n", 1)[0]

    assert "--remain-after-exit" not in start_transient
    assert "--collect" in start_transient
    assert 'systemctl stop "$unit.service"' in start_transient
    assert 'systemctl reset-failed "$unit.service"' in start_transient
    assert (
        "ExecStopPost=-+/usr/bin/systemctl stop nav2-3d.service "
        "slam-online.service mid360-nav-bridge.service "
        "map-odom-fuser.service fastlio.service"
    ) in service
    assert "\nExecStop=" not in service


def test_slam_nav_gates_use_wall_clock_deadlines_with_bounded_samples():
    script = read("docker/bringup_slam_nav2.sh")
    rate_gate = read("tools/topic_rate_gate.py")
    service = read("docker/go2w-slam-nav.service")
    wait_hz = script.split("wait_hz() {", 1)[1].split("\n}\n", 1)[0]
    wait_active = script.split("wait_active() {", 1)[1].split("\n}\n", 1)[0]

    assert "for ((i=1; i<=timeout; i++))" not in wait_hz
    assert "for ((i=1; i<=timeout; i++))" not in wait_active
    assert 'timeout --kill-after=2 $((timeout_s + 3)) python3' in wait_hz
    assert '"$GO2W_WS/tools/topic_rate_gate.py"' in wait_hz
    assert "--samples 20 --minimum-samples 10" in wait_hz
    assert "deadline = time.monotonic()" in rate_gate
    assert "while len(receive_times) < int(samples)" in rate_gate
    assert "outer_timeout=$((timeout_s + 5))" in wait_active
    assert 'timeout --kill-after=2 "$outer_timeout" python3' in wait_active
    assert "TimeoutStartSec=600" in service


def test_slam_nav_requires_live_mid360_observations_before_starting_nav2():
    """Require the measured raw-bridge rate, not an unreachable nominal rate."""
    script = read("docker/bringup_slam_nav2.sh")
    main = script.split("main() {", 1)[1]

    assert 'SCAN_MIN_HZ="${SCAN_MIN_HZ:-3}"' in script
    assert 'wait_hz /mid360/points_nav "$SCAN_MIN_HZ" 30' in main
    assert 'wait_hz /scan_mid360 "$SCAN_MIN_HZ" 30' not in main
    assert main.index('wait_motion_ready 200') < main.index(
        "启动 Nav2 3D"
    )


def test_slam_nav_waits_for_motion_shaping_and_recovery_nodes():
    """Bringup must not report success before the safety-critical lifecycle nodes."""
    script = read("docker/bringup_slam_nav2.sh")
    main = script.split("main() {", 1)[1]

    assert "wait_active behavior_server    60" in main
    assert "wait_active velocity_smoother 60" in main
    assert main.index("wait_active velocity_smoother 60") < main.index(
        "验证 /navigate_to_pose action"
    )


def test_slam_nav_docs_match_shm_behavior_and_first_install_rollback():
    bringup = read("docker/bringup_slam_nav2.sh")
    deploy = read("docker/deploy_nav2_bprime.sh")

    assert "默认: 清 SHM" not in bringup
    assert "默认: 跳过 SHM 清理" in bringup
    assert "go2w-slam-nav.service.bak.$TS" in deploy
    assert "Live gate failed; autonomous stack stopped." in deploy
