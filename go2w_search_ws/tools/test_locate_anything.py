#!/usr/bin/env python3
"""LocateAnything wrapper contract tests.

These tests do not need the 6GB GGUF model. They verify the Python wrapper
that will call locate-anything.cpp on NX: prompt construction, CLI JSON parsing,
and command/error behavior.
"""

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_build_prompt_wraps_follow_target_for_open_vocabulary_grounding():
    from ai.locate_anything import build_locate_prompt

    prompt = build_locate_prompt("穿白色衣服的人")

    assert "Locate all the instances" in prompt
    assert "person" in prompt
    assert "white" in prompt
    assert prompt.endswith(".")


def test_build_prompt_maps_all_visible_objects_to_multi_class_query():
    from ai.locate_anything import build_locate_prompt

    prompt = build_locate_prompt("所有可见物体")

    assert "Locate all the instances" in prompt
    assert "</c>" in prompt  # M3: 多类别候选 (单 "object" 让模型返回全图框 [0,0,W,H])
    assert "person" in prompt
    assert "chair" in prompt


def test_parse_cli_output_normalizes_label_box_and_confidence():
    from ai.locate_anything import parse_cli_output

    out = json.dumps({
        "detections": [
            {"label": "person wearing white clothes", "box": [10, 20, 110, 220]},
            {"class": "chair", "bbox": [1.5, 2.2, 30.9, 40.1], "score": 0.42},
        ]
    })

    dets = parse_cli_output(out)

    assert dets == [
        {
            "class": "person wearing white clothes",
            "label_zh": "白衣人",
            "confidence": 1.0,
            "bbox": [10.0, 20.0, 110.0, 220.0],
            "source": "locate_anything",
        },
        {
            "class": "chair",
            "label_zh": "椅子",
            "confidence": 0.42,
            "bbox": [1.5, 2.2, 30.9, 40.1],
            "source": "locate_anything",
        },
    ]


def test_engine_locate_runs_cli_and_returns_found_bbox(tmp_path):
    from ai.locate_anything import LocateAnythingCli

    image_path = tmp_path / "frame.jpg"
    image_path.write_bytes(b"fake-jpeg")
    commands = []

    def runner(cmd, timeout):
        commands.append((cmd, timeout))
        output_path = Path(cmd[cmd.index("--output") + 1])
        output_path.write_text(
            json.dumps({
                "detections": [
                    {"label": "person wearing white clothes", "box": [5, 6, 50, 80]}
                ]
            }),
            encoding="utf-8",
        )
        return type("Completed", (), {"returncode": 0, "stdout": "", "stderr": ""})()

    fake_bin = tmp_path / "locate-anything-cli"
    fake_model = tmp_path / "locate-anything-q8_0.gguf"
    fake_bin.write_text("#!/bin/sh\n", encoding="utf-8")
    fake_model.write_bytes(b"gguf")

    engine = LocateAnythingCli(
        binary=str(fake_bin),
        model=str(fake_model),
        runner=runner,
    )

    result = engine.locate_path(image_path, "穿白色衣服的人")

    assert result["found"] is True
    assert result["bbox"] == [5.0, 6.0, 50.0, 80.0]
    assert result["label"] == "person wearing white clothes"
    cmd, timeout = commands[0]
    assert cmd[:2] == [str(fake_bin), "detect"]
    assert "--model" in cmd
    assert "--input" in cmd
    assert "--prompt" in cmd
    assert timeout > 0


def test_engine_reports_unavailable_without_binary_or_model(tmp_path):
    from ai.locate_anything import LocateAnythingCli

    engine = LocateAnythingCli(
        binary=str(tmp_path / "missing-cli"),
        model=str(tmp_path / "missing.gguf"),
    )

    result = engine.locate_path(tmp_path / "frame.jpg", "person")

    assert result["found"] is False
    assert "unavailable" in result["description"]


def test_nx_ai_vlm_proxy_exposes_track_target_for_follow_tasks():
    web_dir = ROOT / "web"
    if str(web_dir) not in sys.path:
        sys.path.insert(0, str(web_dir))
    import nx_ai_node

    class FakeAi:
        def track_target(self, frame, target, img_w=640, img_h=480):
            assert target == "穿白色衣服的人"
            return {
                "found": True,
                "bbox": [100, 120, 240, 400],
                "vx": 0.15,
                "vyaw": -0.2,
            }

    proxy = nx_ai_node.NxAiVlmProxy(FakeAi())
    result = proxy.track_target(object(), "穿白色衣服的人", img_w=640, img_h=480)

    assert result["found"] is True
    assert result["vx"] == 0.15
    assert result["vyaw"] == -0.2


def test_nx_ai_engine_locate_result_includes_frame_shape_and_broadcast_box():
    web_dir = ROOT / "web"
    if str(web_dir) not in sys.path:
        sys.path.insert(0, str(web_dir))
    import nx_ai_node

    class FakeFrame:
        shape = (480, 640, 3)

    class FakeLocate:
        available = True

        def locate(self, frame, target):
            return {
                "found": True,
                "bbox": [64, 48, 320, 400],
                "label": "person wearing white clothes",
                "confidence": 0.88,
                "detections": [
                    {
                        "class": "person wearing white clothes",
                        "confidence": 0.88,
                        "bbox": [64, 48, 320, 400],
                        "source": "locate_anything",
                    }
                ],
                "description": "found",
            }

    broadcasts = []
    engine = nx_ai_node.NxAiEngine()
    engine._locate_inited = True
    engine._locate = FakeLocate()
    engine._safe_broadcast = broadcasts.append

    result = engine.locate_target(FakeFrame(), "person wearing white clothes")

    assert result["frame_width"] == 640
    assert result["frame_height"] == 480
    assert result["detections"][0]["frame_width"] == 640
    assert result["detections"][0]["frame_height"] == 480
    assert broadcasts[0]["data"]["bbox"] == [64, 48, 320, 400]
    assert broadcasts[0]["data"]["frame_width"] == 640
    assert broadcasts[0]["data"]["frame_height"] == 480


def test_panel_has_locate_overlay_for_boxes_and_explanations():
    html = (ROOT / "web" / "static" / "panel.html").read_text(encoding="utf-8")

    assert 'id="locateOverlay"' in html
    assert "locate-box" in html
    assert "locate-note" in html
    assert 'id="visVideoWrap"' in html
    assert 'value="所有可见物体"' in html
    assert "function collectLocateBoxes" in html
    assert "function clampBox" in html
    assert "function displayLocateLabel" in html
    assert "label_zh" in html
    assert "overflow: hidden" in html
    assert "boxes.forEach" in html
    assert "renderLocateOverlay(result)" in html
    assert "scheduleLocateOverlay(data.data" in html
    assert "record: true" in html
    assert "expectedGeneration: streamFrameGeneration.c13_vis" in html
