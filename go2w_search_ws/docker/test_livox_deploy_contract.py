from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_deploy_web_copies_lidar_and_gimbal_components():
    script = read("docker/deploy_nx_web.sh")
    assert 'web/nx_lidar_node.py' in script
    assert 'web/nx_gimbal_node.py' in script
    assert 'web/nx_slam_map.py' in script


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


def test_web_service_tolerates_missing_livox_workspace():
    service = read("docker/go2w-web.service")
    assert 'test -f /home/nx/ws_livox/install/setup.bash' in service
    assert 'source /home/nx/ws_livox/install/setup.bash' in service
