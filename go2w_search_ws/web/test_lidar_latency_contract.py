import importlib
import sys
import types
from pathlib import Path


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


class _FakePng:
    def tobytes(self):
        return b"png"


class _FakePoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _FakeMsg:
    def __init__(self):
        self.points = [_FakePoint(1.0, 0.1), _FakePoint(2.0, -0.2), _FakePoint(3.0, 0.3)]


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
