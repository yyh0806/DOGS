import importlib
import sys
import types
from pathlib import Path

import numpy as np


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


class _FakePng:
    def tobytes(self):
        return b"png"


class _FakeMsg:
    def __init__(self):
        self.is_bigendian = False
        self.point_step = 12
        self.fields = [
            types.SimpleNamespace(name="x", offset=0, datatype=7),
            types.SimpleNamespace(name="y", offset=4, datatype=7),
            types.SimpleNamespace(name="z", offset=8, datatype=7),
        ]
        self.data = np.asarray([
            [1.0, 0.1, 0.0], [2.0, -0.2, 0.0], [3.0, 0.3, 0.0]
        ], dtype="<f4").tobytes()


def _load_lidar_module(monkeypatch):
    fake_cv2 = types.SimpleNamespace(
        FONT_HERSHEY_SIMPLEX=0,
        IMWRITE_PNG_COMPRESSION=16,
        encode_count=0,
        circle=lambda *args, **kwargs: None,
        line=lambda *args, **kwargs: None,
        putText=lambda *args, **kwargs: None,
    )

    def _imencode(*args, **kwargs):
        fake_cv2.encode_count += 1
        return True, _FakePng()

    fake_cv2.imencode = _imencode
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    import nx_lidar_node

    return importlib.reload(nx_lidar_node), fake_cv2


def test_lidar_callback_throttles_png_rendering_to_configured_fps(monkeypatch):
    lidar, fake_cv2 = _load_lidar_module(monkeypatch)
    bridge = lidar.LidarBridge(lambda msg: None)
    times = iter([0.0, 0.01])
    monkeypatch.setattr(lidar.time, "monotonic", lambda: next(times, 0.01))

    bridge._cb(_FakeMsg())
    bridge._cb(_FakeMsg())

    assert fake_cv2.encode_count == 1


def test_lidar_uses_low_cpu_png_compression():
    text = (WEB_DIR / "nx_lidar_node.py").read_text(encoding="utf-8")

    assert "cv2.IMWRITE_PNG_COMPRESSION, 1" in text


def test_lidar_private_context_uses_a_private_executor():
    text = (WEB_DIR / "nx_lidar_node.py").read_text(encoding="utf-8")

    assert "SingleThreadedExecutor(context=self._ctx)" in text
    assert "self._executor.add_node(self._node)" in text
    assert "self._executor.spin_once(" in text
    assert "rclpy.spin_once(self._node" not in text


def test_web_lidar_consumes_filtered_pointcloud_not_raw_livox_custom_message():
    text = (WEB_DIR / "nx_lidar_node.py").read_text(encoding="utf-8")

    assert "from sensor_msgs.msg import PointCloud2" in text
    assert 'PointCloud2, "/mid360/points_nav"' in text
    assert "self._subscription = self._node.create_subscription" in text
    assert "qos_profile_sensor_data" in text
    assert "from livox_ros_driver2.msg import CustomMsg" not in text
    active = text.split("def start(self, node):", 1)[1]
    assert '"/livox/lidar"' not in active
