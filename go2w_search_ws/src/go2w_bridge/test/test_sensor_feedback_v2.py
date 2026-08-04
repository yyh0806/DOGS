import ast
from pathlib import Path


SENSOR = Path(__file__).resolve().parents[1] / "go2w_bridge" / "nx_sensor_node.py"


def test_sensor_publishes_each_lowstate_sample_once_with_v2_metadata():
    source = SENSOR.read_text(encoding="utf-8")
    tree = ast.parse(source)
    publish = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef)
        and item.name == "_publish_drive_feedback"
    )
    publish_source = ast.unparse(publish)

    assert "self._last_feedback_sample_id = 0" in source
    assert "sample_id = self._imu['count']" in publish_source
    assert "sample_id <= self._last_feedback_sample_id" in publish_source
    assert "build_wheel_feedback_payload(" in publish_source
    assert "self._last_feedback_sample_id = sample_id" in publish_source


def test_sensor_captures_wheel_motor_loss_and_attitude():
    source = SENSOR.read_text(encoding="utf-8")
    tree = ast.parse(source)
    callback = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "_on_imu"
    )
    callback_source = ast.unparse(callback)

    assert "self._imu['motor_lost']" in callback_source
    assert "getattr(ms[i], 'lost', 0)" in callback_source
    assert "self._imu['rpy']" in callback_source
