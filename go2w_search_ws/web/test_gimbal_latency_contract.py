import importlib
import sys
import types
from pathlib import Path


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


class _FakeJpeg:
    def tobytes(self):
        return b"jpeg"


class _FakeFrame:
    shape = (720, 1280, 3)


class _FakeCap:
    def __init__(self, stop_after, stop):
        self.reads = 0
        self.grabs = 0
        self._stop_after = stop_after
        self._stop = stop

    def isOpened(self):
        return True

    def grab(self):
        self.grabs += 1
        return True

    def read(self):
        self.reads += 1
        if self.reads >= self._stop_after:
            self._stop()
        return True, object()

    def release(self):
        pass


def _load_gimbal_module(monkeypatch):
    resized = []

    def _resize(frame, size):
        resized.append(size)
        return frame

    fake_cv2 = types.SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        CAP_FFMPEG=2,
        VideoCapture=lambda *args, **kwargs: None,
        imencode=lambda *args, **kwargs: (True, _FakeJpeg()),
        resize=_resize,
        _resized=resized,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)
    import nx_gimbal_node

    return importlib.reload(nx_gimbal_node)


def test_capture_waits_between_ticks_instead_of_busy_decoding(monkeypatch):
    gimbal = _load_gimbal_module(monkeypatch)
    bridge = gimbal.GimbalRtspBridge(lambda msg: None)
    sleeps = []
    times = iter([0.0, 0.01])
    monkeypatch.setattr(gimbal.time, "monotonic", lambda: next(times, 0.01))

    def _sleep(seconds):
        sleeps.append(seconds)
        bridge._running = False

    monkeypatch.setattr(gimbal.time, "sleep", _sleep)

    cap = _FakeCap(stop_after=5, stop=lambda: setattr(bridge, "_running", False))
    monkeypatch.setattr(bridge, "_open", lambda url, name, max_width=None, max_height=None: cap)

    bridge._running = True
    bridge._capture_loop("rtsp://example", "vis", "_vis_b64")

    assert cap.reads == 1
    assert sleeps
    assert 0 < sleeps[0] <= 0.1


def test_capture_grabs_stale_frames_before_read(monkeypatch):
    gimbal = _load_gimbal_module(monkeypatch)
    monkeypatch.setattr(gimbal, "_DROP_GRABS", 3)
    bridge = gimbal.GimbalRtspBridge(lambda msg: None)
    monkeypatch.setattr(gimbal.time, "monotonic", lambda: 1.0)

    cap = _FakeCap(stop_after=1, stop=lambda: setattr(bridge, "_running", False))
    monkeypatch.setattr(bridge, "_open", lambda url, name, max_width=None, max_height=None: cap)

    bridge._running = True
    bridge._capture_loop("rtsp://example", "vis", "_vis_b64")

    assert cap.grabs == 3
    assert cap.reads == 1


def test_large_gimbal_frames_are_downscaled_before_jpeg(monkeypatch):
    gimbal = _load_gimbal_module(monkeypatch)

    frame = gimbal._resize_for_ws(_FakeFrame(), max_width=gimbal._VIS_WIDTH)

    assert frame is not None
    assert sys.modules["cv2"]._resized == [(480, 270)]


def test_gimbal_defaults_are_low_latency(monkeypatch):
    gimbal = _load_gimbal_module(monkeypatch)

    assert gimbal._FPS == 4
    assert gimbal._JPEG_Q == 38
    assert gimbal._VIS_WIDTH == 480
    assert gimbal._IR_WIDTH == 256
    assert gimbal._VIS_HEIGHT == 270
    assert gimbal._IR_HEIGHT == 205


def test_gimbal_prefers_jetson_gstreamer_hardware_decode():
    text = (WEB_DIR / "nx_gimbal_node.py").read_text(encoding="utf-8")

    assert 'os.environ.get("C13_BACKEND", "auto")' in text
    assert "nvv4l2decoder enable-max-performance=1" in text
    assert "width={int(self._target_width)},height={int(self._target_height)}" in text
    assert "_GstRtspCapture(url, name, target_width, target_height)" in text
    assert "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true" in text
    assert "(backend=gst/nvv4l2decoder)" in text
