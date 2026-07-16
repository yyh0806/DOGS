"""Contracts for persistent unknown-floor mapping and frontier navigation."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SLAM_LAUNCH = ROOT / "src/go2w_nav/launch/slam_online.launch.py"
SLAM_PARAMS = ROOT / "src/go2w_nav/config/slam_toolbox_online.yaml"
NAV_PARAMS = ROOT / "src/go2w_nav/config/nav2_params_3d.yaml"
FUSER = ROOT / "src/go2w_bridge/go2w_bridge/map_odom_fuser.py"
MAP_PADDING = ROOT / "src/go2w_bridge/go2w_bridge/map_padding_bridge.py"
BRINGUP = ROOT / "docker/bringup_slam_nav2.sh"
SERVICE = ROOT / "docker/go2w-slam-nav.service"
SAFE_REPLAN_BT = (
    ROOT / "src/go2w_nav/behavior_trees/navigate_to_pose_dynamic_safe.xml"
)


def _global_costmap_section():
    source = NAV_PARAMS.read_text(encoding="utf-8")
    return source.split("global_costmap:", 1)[1].split("planner_server:", 1)[0]


def test_online_slam_consumes_the_existing_mid360_scan_once():
    launch = SLAM_LAUNCH.read_text(encoding="utf-8")
    params = SLAM_PARAMS.read_text(encoding="utf-8")

    assert "pointcloud_to_laserscan" not in launch
    assert "p2l_node" not in launch
    # The YAML root key is `slam_toolbox`; a renamed node silently ignores it.
    assert "name='slam_toolbox'" in launch
    assert '("map", "/map_frontier_raw")' in launch
    assert '("pose", "/slam_pose")' in launch
    assert "scan_topic: /scan_mid360" in params
    assert "transform_publish_period: 0.05" in params


def test_slam_is_the_only_persistent_frontier_map_source():
    bridge = (ROOT / "web/costmap_bridge.py").read_text(encoding="utf-8")
    launch = SLAM_LAUNCH.read_text(encoding="utf-8")

    assert "'/map_frontier'" not in bridge
    assert "self._frontier_pub" not in bridge
    padding = MAP_PADDING.read_text(encoding="utf-8")
    assert '("map", "/map_frontier_raw")' in launch
    assert 'declare_parameter("input_topic", "/map_frontier_raw")' in padding
    assert 'declare_parameter("output_topic", "/map_frontier")' in padding


def test_fuser_can_yield_map_tf_ownership_and_publish_slam_pose():
    source = FUSER.read_text(encoding="utf-8")

    assert 'declare_parameter("publish_map_to_odom", True)' in source
    assert 'declare_parameter("use_slam_pose", False)' in source
    assert "self._publish_map_to_odom" in source
    assert "self._use_slam_pose" in source
    assert "PoseWithCovarianceStamped, '/slam_pose'" in source
    assert "def _on_slam_pose(" in source
    assert "if self._publish_map_to_odom:" in source
    assert "if not self._use_slam_pose:" in source
    # SLAM Toolbox's pose topic is event-driven and may publish only once at
    # standstill.  Continuous localization therefore consumes its canonical
    # map->odom TF edge as the primary correction source.
    assert "Buffer(cache_time=" in source
    assert "TransformListener(" in source
    assert "def _refresh_map_to_odom(" in source
    assert 'declare_parameter("max_slam_tf_future_skew_sec", 0.3)' in source
    refresh = source.split("def _refresh_map_to_odom", 1)[1].split(
        "def _on_slam_pose", 1
    )[0]
    assert "lookup_transform(self._world, self._odom" in refresh
    assert "self._slam_map_to_odom = _tf_to_mat" in refresh


def test_global_costmap_uses_persistent_slam_map_plus_live_obstacles():
    section = _global_costmap_section()

    assert "rolling_window: false" in section
    assert 'plugins: ["static_layer", "obstacle_layer", "inflation_layer"]' in section
    assert "static_layer:" in section
    assert 'map_topic: "/map_frontier"' in section
    assert "map_subscribe_transient_local: true" in section
    assert "track_unknown_space: true" in section
    assert "topic: /scan_mid360" in section


def test_nav2_periodically_replans_and_retries_without_motion_recovery():
    tree = ET.parse(SAFE_REPLAN_BT).getroot()
    tags = [node.tag for node in tree.iter()]
    rates = [node for node in tree.iter() if node.tag == "RateController"]
    recoveries = [node for node in tree.iter() if node.tag == "RecoveryNode"]
    clears = [node for node in tree.iter() if node.tag == "ClearEntireCostmap"]
    waits = [node for node in tree.iter() if node.tag == "Wait"]
    params = NAV_PARAMS.read_text(encoding="utf-8")
    launch = (ROOT / "src/go2w_nav/launch/nav2_3d.launch.py").read_text(
        encoding="utf-8"
    )

    assert len(rates) == 1
    assert float(rates[0].attrib["hz"]) >= 2.0
    assert tags.count("ComputePathToPose") == 1
    assert tags.count("FollowPath") == 1
    assert "PipelineSequence" in tags
    assert len(recoveries) == 1
    assert int(recoveries[0].attrib["number_of_retries"]) >= 3
    assert len(clears) == 1
    assert clears[0].attrib["service_name"] == (
        "global_costmap/clear_entirely_global_costmap"
    )
    assert len(waits) == 1
    assert float(waits[0].attrib["wait_duration"]) == 1.0
    for unsafe_motion_recovery in ("Spin", "BackUp", "DriveOnHeading"):
        assert unsafe_motion_recovery not in tags
    assert "navigate_w_replanning_only_if_goal_is_updated.xml" not in params
    assert "navigate_to_pose_dynamic_safe.xml" in launch
    assert "default_nav_to_pose_bt_xml" in launch
    assert "update_frequency: 2.0" in _global_costmap_section()


def test_bringup_starts_one_slam_owner_before_nav2_and_stops_it_cleanly():
    script = BRINGUP.read_text(encoding="utf-8")
    service = SERVICE.read_text(encoding="utf-8")

    assert "-p publish_map_to_odom:=false" in script
    assert "-p use_slam_pose:=true" in script
    assert "start_transient slam-online" in script
    assert "ros2 launch go2w_nav slam_online.launch.py" in script
    assert "start_transient map-padding" in script
    assert "map_padding_bridge.py" in script
    assert "wait_message /map_frontier 45" in script
    assert "--check-margin 0.5" in script
    assert 'ros2 topic echo --no-daemon --once "$topic"' in script
    assert 'nav_msgs/msg/OccupancyGrid --field info' in script
    assert "wait_hz /localization_pose" in script
    assert "Wants=livox-mid360-driver.service" in service
    assert "Requires=livox-mid360-driver.service" not in service
    assert script.index("start_transient map-padding") < script.index(
        "start_transient slam-online"
    )
    assert script.index("start_transient slam-online") < script.index(
        "start_transient nav2-3d"
    )
    assert "map-padding.service" in service
    assert "slam-online.service" in service


def test_slam_pose_correction_drives_continuous_map_localization():
    source = FUSER.read_text(encoding="utf-8")

    assert "def _propagate_map_pose(" in source
    assert "self._latest_odom_planar" in source
    assert "self._slam_map_to_odom" in source
    assert "_propagate_map_pose(" in source
    lio_callback = source.split("def _on_lio", 1)[1].split(
        "def _on_wheel", 1
    )[0]
    wheel_callback = source.split("def _on_wheel", 1)[1].split(
        "@staticmethod", 1
    )[0]
    assert "self._slam_map_to_odom @ planar" in lio_callback
    assert "self._publish_map_localization(" in lio_callback
    assert "self._pose_pub.publish(localization)" in lio_callback
    assert "self._pose_pub.publish" not in wheel_callback
    assert "self._odom_pub.publish" not in wheel_callback
