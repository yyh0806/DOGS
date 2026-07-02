import json
import os
import sys

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_person_mission import PersonMissionStore


def test_nearby_lidar_observations_deduplicate_and_keep_highest_confidence():
    store = PersonMissionStore("m123")

    first = store.add_observation({
        "class": "person",
        "confidence": 0.7,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [10, 10, 50, 80],
    })
    second = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.4,
        "world_y": 2.1,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [12, 12, 52, 82],
    })

    markers = store.markers()
    assert first["id"] == "person_001"
    assert second["id"] == "person_001"
    assert len(markers) == 1
    assert markers[0]["confidence"] == 0.9

    markers[0]["confidence"] = 0.1
    assert store.markers()[0]["confidence"] == 0.9


def test_far_lidar_observations_create_separate_person_ids():
    store = PersonMissionStore("m123")

    store.add_observation({
        "class": "person",
        "confidence": 0.7,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })
    store.add_observation({
        "class": "person",
        "confidence": 0.8,
        "world_x": 4.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })

    assert [marker["id"] for marker in store.markers()] == ["person_001", "person_002"]


def test_saves_artifacts_for_confirmed_person_using_numpy_frame(tmp_path):
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    store = PersonMissionStore("m123", static_root=tmp_path)

    marker = store.add_observation({
        "class": "person",
        "confidence": 0.95,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [-10, 10, 120, 80],
    }, frame=frame)

    mission_dir = tmp_path / "missions" / "m123"
    raw_path = mission_dir / "person_001_raw.jpg"
    annotated_path = mission_dir / "person_001_annotated.jpg"
    crop_path = mission_dir / "person_001_crop.jpg"
    json_path = mission_dir / "person_001.json"

    assert marker["photo_url"].endswith("/missions/m123/person_001_annotated.jpg")
    assert marker["crop_url"].endswith("/missions/m123/person_001_crop.jpg")
    assert marker["raw_url"].endswith("/missions/m123/person_001_raw.jpg")
    assert raw_path.exists()
    assert annotated_path.exists()
    assert crop_path.exists()
    assert json_path.exists()

    with json_path.open("r", encoding="utf-8") as fh:
        metadata = json.load(fh)
    assert metadata["id"] == "person_001"
    assert metadata["bbox"] == [0, 10, 100, 80]
    assert metadata["photo_url"].endswith("/missions/m123/person_001_annotated.jpg")

    assert Image.open(crop_path).size == (100, 70)
    assert np.asarray(Image.open(annotated_path)).sum() > 0


def test_higher_confidence_merge_without_frame_preserves_existing_artifacts_and_bbox(tmp_path):
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    store = PersonMissionStore("m123", static_root=tmp_path)

    first = store.add_observation({
        "class": "person",
        "confidence": 0.6,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [10, 20, 60, 90],
    }, frame=frame)
    merged = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.1,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
    })

    assert merged["id"] == first["id"]
    assert merged["photo_url"] == first["photo_url"]
    assert merged["crop_url"] == first["crop_url"]
    assert merged["raw_url"] == first["raw_url"]
    assert merged["bbox"] == [10, 20, 60, 90]
    assert merged["frame_width"] == 100
    assert merged["frame_height"] == 100


def test_mission_id_is_sanitized_for_artifact_paths_and_urls(tmp_path):
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    store = PersonMissionStore("../../escape", static_root=tmp_path)

    marker = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [2, 3, 12, 13],
    }, frame=frame)

    mission_dir = tmp_path / "missions" / "escape"
    assert (mission_dir / "person_001_raw.jpg").exists()
    assert (mission_dir / "person_001_annotated.jpg").exists()
    assert (mission_dir / "person_001_crop.jpg").exists()
    assert not (tmp_path / "escape").exists()
    assert marker["raw_url"].endswith("/missions/escape/person_001_raw.jpg")
    assert marker["photo_url"].endswith("/missions/escape/person_001_annotated.jpg")
    assert marker["crop_url"].endswith("/missions/escape/person_001_crop.jpg")


def test_lidar_dedup_merges_with_nearest_marker_inside_radius():
    store = PersonMissionStore("m123", merge_distance_m=0.7)

    first = store.add_observation({
        "class": "person",
        "confidence": 0.5,
        "world_x": 0.0,
        "world_y": 0.0,
        "position_quality": "range_lidar",
    })
    second = store.add_observation({
        "class": "person",
        "confidence": 0.6,
        "world_x": 1.0,
        "world_y": 0.0,
        "position_quality": "range_lidar",
    })
    merged = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 0.65,
        "world_y": 0.0,
        "position_quality": "range_lidar",
    })

    assert first["id"] == "person_001"
    assert second["id"] == "person_002"
    assert merged["id"] == "person_002"
    assert [marker["id"] for marker in store.markers()] == ["person_001", "person_002"]


def test_inverted_bbox_coordinates_are_sorted_before_cropping(tmp_path):
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    store = PersonMissionStore("m123", static_root=tmp_path)

    marker = store.add_observation({
        "class": "person",
        "confidence": 0.95,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "bbox": [80, 10, 20, 70],
    }, frame=frame)

    crop_path = tmp_path / "missions" / "m123" / "person_001_crop.jpg"
    assert marker["bbox"] == [20, 10, 80, 70]
    assert Image.open(crop_path).size == (60, 60)
