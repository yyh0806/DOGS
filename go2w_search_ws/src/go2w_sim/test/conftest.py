"""M1a pytest fixtures: bring up gzserver + spawn go2_sim for teleop test.

Session-scoped: starts Gazebo headless once, spawns the robot, yields a
rclpy node wired to /cmd_vel publisher + /odom_planar subscriber, tears
down on session end.
"""
import os
import time
import subprocess
import pytest
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope='session')
def sim_spawn_only_session():
    rclpy.init()
    node = Node('test_teleop_harness')
    pkg = get_package_share_directory('go2w_sim')
    world = os.path.join(pkg, 'worlds', 'indoor_empty.world')
    urdf = os.path.join(pkg, 'urdf', 'go2_sim.urdf')
    procs = []
    procs.append(subprocess.Popen([
        'gzserver', '--verbose',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(5.0)  # let world load
    procs.append(subprocess.Popen([
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'go2_sim', '-file', urdf,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(4.0)  # let robot spawn + planar_move attach
    yield node
    for p in procs:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope='session')
def sim_fastlio_session():
    """Real-fidelity Task2: 起 sim_fastlio_bringup (gzserver + spawn go2_sim_livox
    URDF: planar_move + Livox CustomMsg + IMU + fastlio_mapping). 等 /Odometry 发布.

    spec 2026-07-25 §9 step 6. 静止原点 test 消费此 fixture.
    """
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = Node('test_fastlio_harness')
    launch_proc = subprocess.Popen(
        ['bash', '-c',
         'source ~/go2w_ws/install/setup.bash && '
         'exec ros2 launch go2w_sim sim_fastlio_bringup.launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    odom_ready = {'ready': False}
    node.create_subscription(
        Odometry, '/Odometry',
        lambda m: odom_ready.update(ready=True), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90.0 and not odom_ready['ready']:
        rclpy.spin_once(node, timeout_sec=0.5)
    # FastLIO ESKF 静止收敛需 lidar 积累 (~60s); 首条 /Odometry 时 ESKF 仍在收敛,
    # 直接测会捕获收敛期漂移 (y/z 波动 0.02-0.04m)。等收敛后测稳定漂移 (<0.02m)。
    if odom_ready['ready']:
        time.sleep(60.0)
    yield node
    launch_proc.terminate()
    try:
        launch_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        launch_proc.kill()
    subprocess.run(['pkill', '-9', '-x', 'gzserver'])
    node.destroy_node()
    rclpy.shutdown()
