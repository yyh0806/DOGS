import importlib
import sys
import threading
import types
from pathlib import Path


WEB_DIR = Path(__file__).resolve().parent
WEB_SOURCE = WEB_DIR / "nx_web_server.py"
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


def _install_ros_stubs(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None

    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        pass

    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")

    class Odometry:
        pass

    nav_msgs_msg.Odometry = Odometry
    nav_msgs.msg = nav_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class Imu:
        pass

    class LaserScan:
        pass

    sensor_msgs_msg.Imu = Imu
    sensor_msgs_msg.LaserScan = LaserScan
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        pass

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    nx_slam_map = types.ModuleType("nx_slam_map")

    class ObstacleGridAccumulator:
        def __init__(self, *args, **kwargs):
            self._points = []

        def update(self, points):
            self._points = list(points)
            return list(self._points)

        def points(self):
            return list(self._points)

    nx_slam_map.ObstacleGridAccumulator = ObstacleGridAccumulator

    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "nx_slam_map": nx_slam_map,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)


def _load_web_server(monkeypatch):
    _install_ros_stubs(monkeypatch)
    sys.modules.pop("nx_web_server", None)
    return importlib.import_module("nx_web_server")


def _new_scan_node(web):
    node = web.NxWebNode.__new__(web.NxWebNode)
    node._lock = threading.Lock()
    node._scan_count = 0
    node._scan_ranges = []
    node._scan_angle_min = 0.0
    node._scan_angle_increment = 0.0
    node._scan_range_min = 0.0
    node._scan_range_max = 0.0
    node._scan_timestamp = 0.0
    return node


def test_scan_snapshot_stores_laserscan_metadata_timestamp_and_copied_ranges(monkeypatch):
    web = _load_web_server(monkeypatch)
    node = _new_scan_node(web)
    ticks = iter([100.0, 100.25, 100.5])
    monkeypatch.setattr(web.time, "time", lambda: next(ticks))
    scan = types.SimpleNamespace(
        ranges=[0.12345, 1.23456, 9.87654],
        angle_min=-1.57,
        angle_increment=0.01,
        range_min=0.2,
        range_max=12.0,
    )

    node._on_scan(scan)
    snapshot = node.get_scan_snapshot()

    assert snapshot["angle_min"] == -1.57
    assert snapshot["angle_increment"] == 0.01
    assert snapshot["range_min"] == 0.2
    assert snapshot["range_max"] == 12.0
    assert snapshot["ranges"] == [0.123, 1.235, 9.877]
    assert snapshot["count"] == 1
    assert snapshot["timestamp"] == 100.0
    assert snapshot["age_sec"] == 0.25

    snapshot["ranges"][0] = 99

    assert node.get_scan_snapshot()["ranges"] == [0.123, 1.235, 9.877]


def test_web_server_does_not_start_locateanything_background_worker():
    text = WEB_SOURCE.read_text(encoding="utf-8")

    assert "LocateAnythingServer" not in text
    assert "GO2W_LOCATE_AUTORUN" not in text
    assert "locate_worker" not in text
