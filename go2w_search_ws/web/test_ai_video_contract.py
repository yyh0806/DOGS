from pathlib import Path


WEB_SOURCE = Path(__file__).resolve().parent / "nx_ai_node.py"
SERVICE_SOURCE = Path(__file__).resolve().parents[1] / "docker" / "go2w-web.service"


def test_dog_camera_video_loop_is_explicitly_gated():
    text = WEB_SOURCE.read_text(encoding="utf-8")

    assert 'os.environ.get("GO2W_AI_VIDEO_ENABLE", "0")' in text
    assert "if _AI_VIDEO_ENABLED:" in text
    assert 'threading.Thread(target=self._video_yolo_loop' in text


def test_nx_web_service_defaults_dog_camera_video_loop_off():
    text = SERVICE_SOURCE.read_text(encoding="utf-8")

    assert "Environment=GO2W_AI_VIDEO_ENABLE=0" in text
