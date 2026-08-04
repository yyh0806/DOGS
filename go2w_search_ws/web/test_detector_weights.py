from __future__ import annotations

from types import SimpleNamespace


def test_ultralytics_clip_weights_use_persistent_model_directory(
        monkeypatch, tmp_path):
    from ai.detector import _configure_ultralytics_weights_dir

    weights_dir = tmp_path / "shared-models"
    monkeypatch.setenv("GO2W_ULTRALYTICS_WEIGHTS_DIR", str(weights_dir))
    ultralytics_utils = SimpleNamespace(WEIGHTS_DIR="weights")
    text_model = SimpleNamespace(WEIGHTS_DIR="weights")

    configured = _configure_ultralytics_weights_dir(
        ultralytics_utils, text_model)

    assert configured == weights_dir
    assert weights_dir.is_dir()
    assert ultralytics_utils.WEIGHTS_DIR == weights_dir
    assert text_model.WEIGHTS_DIR == weights_dir
