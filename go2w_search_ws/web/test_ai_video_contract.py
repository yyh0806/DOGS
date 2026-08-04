import sys
import types
from pathlib import Path

import numpy as np


WEB_DIR = Path(__file__).resolve().parent
WEB_SOURCE = WEB_DIR / "nx_ai_node.py"
SERVICE_SOURCE = Path(__file__).resolve().parents[1] / "docker" / "go2w-web.service"
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))


def test_dog_camera_video_loop_is_explicitly_gated():
    text = WEB_SOURCE.read_text(encoding="utf-8")

    assert 'os.environ.get("GO2W_AI_VIDEO_ENABLE", "0")' in text
    assert "if _AI_VIDEO_ENABLED:" in text
    assert 'threading.Thread(target=self._video_yolo_loop' in text


def test_nx_web_service_defaults_dog_camera_video_loop_off():
    text = SERVICE_SOURCE.read_text(encoding="utf-8")

    assert "Environment=GO2W_AI_VIDEO_ENABLE=0" in text


def test_dog_video_init_skips_channel_factory_when_interface_has_no_carrier(monkeypatch):
    import nx_ai_node
    from nx_ai_node import NxAiEngine

    calls = []

    class FakeChannelFactory:
        def Init(self, domain, iface):
            calls.append((domain, iface))

    class FakeVideoClient:
        def SetTimeout(self, timeout):
            pass

        def Init(self):
            pass

    monkeypatch.setitem(sys.modules, "unitree_sdk2py", types.ModuleType("unitree_sdk2py"))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core", types.ModuleType("unitree_sdk2py.core"))
    channel_mod = types.ModuleType("unitree_sdk2py.core.channel")
    channel_mod.ChannelFactory = FakeChannelFactory
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel_mod)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.go2", types.ModuleType("unitree_sdk2py.go2"))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.go2.video", types.ModuleType("unitree_sdk2py.go2.video"))
    video_mod = types.ModuleType("unitree_sdk2py.go2.video.video_client")
    video_mod.VideoClient = FakeVideoClient
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.go2.video.video_client", video_mod)

    monkeypatch.setenv("DOG_INTERFACE", "down0")
    monkeypatch.setattr(nx_ai_node, "_dog_interface_ready", lambda iface: False, raising=False)

    ai = NxAiEngine()
    ai._init_video()

    assert calls == []
    assert ai._mock_mode is True
    assert ai._video is None


def test_detection_scheduler_interleaves_dog_frames_with_external_sources(monkeypatch):
    import nx_ai_node
    from nx_ai_node import NxAiEngine

    monkeypatch.setattr(nx_ai_node, "_AI_VIDEO_ENABLED", True)

    ai = NxAiEngine()
    ai._mock_mode = False
    dog_frame = np.zeros((4, 4, 3), dtype=np.uint8)
    external_frame = np.ones((4, 4, 3), dtype=np.uint8)
    ai._get_frame = lambda: dog_frame
    ai._take_external_frame = lambda: (external_frame, "c13_vis")

    sources = [ai._get_next_detection_frame(0.0)[1] for _ in range(4)]

    assert "dog" in sources
    assert "c13_vis" in sources
