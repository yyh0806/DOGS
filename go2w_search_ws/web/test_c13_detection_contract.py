from pathlib import Path


WEB_DIR = Path(__file__).resolve().parent
SERVER_SOURCE = WEB_DIR / "nx_web_server.py"
GIMBAL_SOURCE = WEB_DIR / "nx_gimbal_node.py"
PANEL_SOURCE = WEB_DIR / "static" / "panel.html"


def test_broadcast_loop_feeds_c13_frames_and_broadcasts_detection_payload():
    text = SERVER_SOURCE.read_text(encoding="utf-8")

    assert "submit_external_frame" in text
    assert 'submit_external_frame(c13_vis_frame, source="c13_vis")' in text
    assert 'submit_external_frame(c13_ir_frame, source="c13_ir")' in text
    assert "get_detection_overlay" in text
    assert "get_detection_overlays" in text
    assert '"type": "detections"' in text
    assert "get_detections_world(x, y, yaw, ranges=ranges, lidar_points=lidar_points)" in text


def test_gimbal_bridge_exposes_visible_and_ir_frames_for_detection():
    text = GIMBAL_SOURCE.read_text(encoding="utf-8")

    assert "self._vis_frame" in text
    assert "self._ir_frame" in text
    assert "def get_vis_frame(self):" in text
    assert "def get_ir_frame(self):" in text


def test_http_serves_detection_snapshot_jpegs():
    text = SERVER_SOURCE.read_text(encoding="utf-8")

    assert "/api/detection_snapshot" in text
    assert "get_detection_snapshot_jpeg" in text
    assert "'Content-Type', 'image/jpeg'" in text
    assert "/api/video_frame" in text
    assert "get_video_frame_jpeg" in text
    assert "def _send_jpeg" in text
    assert "BrokenPipeError" in text
    assert "ConnectionResetError" in text


def test_http_serves_persisted_mission_photos_without_path_escape():
    text = SERVER_SOURCE.read_text(encoding="utf-8")

    assert "p.path.startswith('/missions/')" in text
    assert "def _serve_mission_artifact" in text
    assert "os.path.commonpath" in text
    assert "unquote(" in text
    assert "GO2W_MISSION_ROOT" in text


def test_frontend_draws_detection_boxes_and_clickable_snapshots():
    text = PANEL_SOURCE.read_text(encoding="utf-8")

    assert 'id="detectOverlay"' in text
    assert 'id="irDetectOverlay"' in text
    assert 'id="snapshotModal"' in text
    assert "renderDetectionOverlay" in text
    assert "renderDetectionStreams" in text
    assert "latestDetectionsBySource" in text
    assert "openDetectionSnapshot" in text
    assert "data.type === 'detections'" in text
    assert "c13_vis" in text
    assert "c13_ir" in text
    assert "crop_url" in text
    assert "frame_url" in text


def test_frontend_map_marker_click_opens_detection_snapshot():
    # Q1=b: 点地图 person 标记 → 弹检测截图 (map.js _hitTestMarker + panel.html onSelectMarker)
    map_text = (WEB_DIR / "static" / "map.js").read_text(encoding="utf-8")
    panel_text = PANEL_SOURCE.read_text(encoding="utf-8")

    assert "_hitTestMarker" in map_text
    assert "onSelectMarker" in map_text
    assert "map.onSelectMarker" in panel_text
