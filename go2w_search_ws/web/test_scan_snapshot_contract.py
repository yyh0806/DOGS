from pathlib import Path


WEB_SOURCE = Path(__file__).resolve().parent / "nx_web_server.py"


def test_scan_snapshot_stores_laserscan_angle_metadata():
    text = WEB_SOURCE.read_text(encoding="utf-8")

    assert "self._scan_angle_min" in text
    assert "self._scan_angle_increment" in text
    assert "self._scan_range_min" in text
    assert "self._scan_range_max" in text
    assert "msg.angle_min" in text
    assert "msg.angle_increment" in text
    assert "msg.range_min" in text
    assert "msg.range_max" in text
    assert "def get_scan_snapshot(self):" in text
