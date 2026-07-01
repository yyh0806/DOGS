from pathlib import Path


SOURCE = Path(__file__).resolve().parent / "nx_web_server.py"


def test_ws_broadcast_has_backpressure_and_send_timeout():
    text = SOURCE.read_text(encoding="utf-8")

    assert "_WS_MAX_PENDING" in text
    assert "_WS_PENDING" in text
    assert "asyncio.wait_for(ws.send(msg)" in text
    assert "_WS_SEND_TIMEOUT" in text
    assert "await ws.send(msg)" not in text


def test_frame_broadcast_does_not_include_unused_jpeg_payload():
    text = SOURCE.read_text(encoding="utf-8")

    assert '"type": "frame", "data": b64' not in text
    assert '"type": "frame", "detections": int(det_count)' in text
    assert "ai_engine.get_frame_jpeg()" not in text
    assert "ai_engine.get_frame_detection_count()" in text


def test_realtime_bridges_are_stopped_on_process_exit():
    text = SOURCE.read_text(encoding="utf-8")

    assert "gimbal_bridge.stop()" in text
    assert "lidar_bridge.stop()" in text
