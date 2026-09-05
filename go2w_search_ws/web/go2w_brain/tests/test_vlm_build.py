"""test_vlm_build.py — VLM 客户端凭据选择测试 (2026-09-05 GLM 接入)。"""
from __future__ import annotations

from go2w_brain.vlm import (DEFAULT_VLM_MODEL, GLM_BASE_URL,
                            GLM_DEFAULT_MODEL, build_vlm)


def test_build_vlm_prefers_glm(monkeypatch):
    monkeypatch.setenv("GLM_API_KEY", "glm-key")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "ds-key")
    monkeypatch.delenv("GO2W_VLM_MODEL", raising=False)
    monkeypatch.delenv("GO2W_GLM_MODEL", raising=False)
    v = build_vlm()
    assert v._key == "glm-key"
    assert v._base == GLM_BASE_URL
    assert v._model == GLM_DEFAULT_MODEL


def test_build_vlm_explicit_model_uses_deepseek(monkeypatch):
    monkeypatch.setenv("GLM_API_KEY", "glm-key")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "ds-key")
    monkeypatch.setenv("GO2W_VLM_MODEL", "my-vision-model")
    v = build_vlm()
    assert v._key == "ds-key"
    assert v._model == "my-vision-model"
    assert v._base == "https://api.deepseek.com"


def test_build_vlm_falls_back_deepseek_default(monkeypatch):
    monkeypatch.setenv("GLM_API_KEY", "")
    monkeypatch.setenv("DEEPSEEK_API_KEY", "ds-key")
    monkeypatch.delenv("GO2W_VLM_MODEL", raising=False)
    # 屏蔽 yaml 凭据, 否则空 env 会回落到真实 GLM key
    import go2w_brain.config as cfg_mod
    monkeypatch.setattr(cfg_mod, "_read_credential", lambda kw: "")
    v = build_vlm()
    assert v._key == "ds-key"
    assert v._model == DEFAULT_VLM_MODEL
