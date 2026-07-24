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
