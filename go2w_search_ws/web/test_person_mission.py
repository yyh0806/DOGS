import json
import os
import sys

import numpy as np
import pytest
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "web"))

from nx_person_mission import (
    PersonMissionStore,
    TargetMissionStore,
    load_latest_mission_report,
)


def test_persistent_mission_root_keeps_media_and_report_outside_release(tmp_path):
    release_static = tmp_path / "release" / "web" / "static"
    mission_root = tmp_path / "persistent" / "missions"
    store = TargetMissionStore(
        "mission-persistent",
        static_root=release_static,
        mission_root=mission_root,
    )

    marker = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.0,
        "world_y": 2.0,
        "bbox": [10, 10, 30, 50],
        "position_quality": "range_lidar",
    }, frame=np.zeros((60, 80, 3), dtype=np.uint8))
    report = {
        "mission_id": "mission-persistent",
        "status": "completed",
        "end_time": 123.0,
        "detections": [marker],
    }
    report_path = store.save_report(report)

    assert report_path == mission_root / "mission-persistent" / "report.json"
    assert report_path.is_file()
    assert not (release_static / "missions").exists()
    assert marker["photo_url"].startswith(
        "/missions/mission-persistent/")
    assert load_latest_mission_report(mission_root) == report


def test_latest_mission_report_ignores_corrupt_files(tmp_path):
    older = TargetMissionStore("older", mission_root=tmp_path)
    older.save_report({
        "mission_id": "older", "status": "completed",
        "end_time": 10.0, "detections": [],
    })
    newer = TargetMissionStore("newer", mission_root=tmp_path)
    expected = {
        "mission_id": "newer", "status": "completed",
        "end_time": 20.0, "detections": [{"id": "person_001"}],
    }
    newer.save_report(expected)
    corrupt = tmp_path / "corrupt"
    corrupt.mkdir()
    (corrupt / "report.json").write_text("not-json", encoding="utf-8")

    assert load_latest_mission_report(tmp_path) == expected


def test_generic_table_observations_deduplicate_with_target_specific_id():
    store = TargetMissionStore("m-table", default_class="dining table")

    first = store.add_observation({
        "class": "dining table",
        "confidence": 0.8,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })
    second = store.add_observation({
        "class": "dining table",
        "confidence": 0.9,
        "world_x": 1.3,
        "world_y": 2.1,
        "position_quality": "range_lidar",
    })

    assert first["id"] == "dining_table_001"
    assert second["id"] == "dining_table_001"
    assert len(store.markers()) == 1
    assert store.markers()[0]["class"] == "dining table"


def test_nearby_different_target_classes_never_merge():
    store = TargetMissionStore("m-mixed")

    table = store.add_observation({
        "class": "dining table",
        "confidence": 0.8,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })
    person = store.add_observation({
        "class": "person",
        "confidence": 0.9,
        "world_x": 1.1,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })

    assert table["id"] == "dining_table_001"
    assert person["id"] == "person_002"
    assert len(store.markers()) == 2


def test_observation_from_another_mission_is_rejected():
    store = TargetMissionStore("mission-a")

    with pytest.raises(ValueError, match="observation mission_id does not match"):
        store.add_observation({
            "mission_id": "mission-b",
            "class": "person",
            "confidence": 0.9,
            "world_x": 1.0,
            "world_y": 2.0,
            "position_quality": "range_lidar",
        })

    assert store.markers() == []


def test_merge_records_evidence_and_keeps_best_synchronized_range_position():
    store = TargetMissionStore("mission-a")

    first = store.add_observation({
        "observation_id": "obs-low-quality",
        "class": "person",
        "confidence": 0.95,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "localization_quality": "latest_pose",
        "scan_delta_s": 0.18,
    })
    merged = store.add_observation({
        "observation_id": "obs-synchronized",
        "class": "person",
        "confidence": 0.80,
        "world_x": 1.2,
        "world_y": 2.1,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "localization_quality": "timestamp_interpolated",
        "scan_delta_s": 0.02,
    })
    merged_again = store.add_observation({
        "observation_id": "obs-stale",
        "class": "person",
        "confidence": 0.99,
        "world_x": 1.4,
        "world_y": 2.2,
        "position_quality": "range_lidar",
        "range_source": "lidar",
        "localization_quality": "latest_pose",
        "scan_delta_s": 0.19,
    })

    assert first["mission_id"] == "mission-a"
    assert merged["world_x"] == pytest.approx(1.2)
    assert merged["world_y"] == pytest.approx(2.1)
    assert merged["canonical_observation_id"] == "obs-synchronized"
    assert merged_again["world_x"] == pytest.approx(1.2)
    assert merged_again["world_y"] == pytest.approx(2.1)
    assert merged_again["canonical_observation_id"] == "obs-synchronized"
    assert [item["observation_id"] for item in merged_again["merge_evidence"]] == [
        "obs-synchronized",
        "obs-stale",
    ]
    assert merged_again["merge_evidence"][0]["method"] == "spatial"


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


def test_nearby_lidar_observations_use_confidence_weighted_position():
    store = PersonMissionStore("m123")

    store.add_observation({
        "class": "person",
        "confidence": 0.8,
        "world_x": 1.0,
        "world_y": 2.0,
        "position_quality": "range_lidar",
    })
    merged = store.add_observation({
        "class": "person",
        "confidence": 0.4,
        "world_x": 1.6,
        "world_y": 2.3,
        "position_quality": "range_lidar",
    })

    assert merged["world_x"] == pytest.approx(1.2)
    assert merged["world_y"] == pytest.approx(2.1)
    assert merged["x"] == pytest.approx(1.2)
    assert merged["y"] == pytest.approx(2.1)
    assert merged["position_weight"] == pytest.approx(1.2)


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


def _solid_frame(color):
    frame = np.zeros((80, 60, 3), dtype=np.uint8)
    frame[:, :] = np.asarray(color, dtype=np.uint8)
    return frame


def test_same_appearance_merges_across_viewpoints_despite_position_jitter(tmp_path):
    store = PersonMissionStore(
        "m123", static_root=tmp_path, merge_distance_m=0.7,
        appearance_merge_distance_m=1.5)
    red = _solid_frame((0, 0, 255))

    first = store.add_observation({
        "class": "person", "confidence": 0.8,
        "world_x": 0.0, "world_y": 0.0,
        "position_quality": "range_lidar", "bbox": [5, 5, 55, 75],
    }, frame=red)
    merged = store.add_observation({
        "class": "person", "confidence": 0.9,
        "world_x": 1.0, "world_y": 0.0,
        "position_quality": "range_lidar", "bbox": [5, 5, 55, 75],
    }, frame=red)

    assert merged["id"] == first["id"]
    assert len(store.markers()) == 1
    assert merged["dedup_method"] == "appearance_spatial"
    assert merged["appearance_similarity"] == pytest.approx(1.0)
    assert merged["observation_count"] == 2


def test_different_appearance_prevents_nearby_people_from_collapsing(tmp_path):
    store = PersonMissionStore("m123", static_root=tmp_path)

    first = store.add_observation({
        "class": "person", "confidence": 0.8,
        "world_x": 0.0, "world_y": 0.0,
        "position_quality": "range_lidar", "bbox": [5, 5, 55, 75],
    }, frame=_solid_frame((0, 0, 255)))
    second = store.add_observation({
        "class": "person", "confidence": 0.9,
        "world_x": 0.3, "world_y": 0.0,
        "position_quality": "range_lidar", "bbox": [5, 5, 55, 75],
    }, frame=_solid_frame((255, 0, 0)))

    assert first["id"] == "person_001"
    assert second["id"] == "person_002"
    assert len(store.markers()) == 2


def test_3d_marker_fuses_world_z_and_preserves_dimension():
    store = PersonMissionStore("m123")
    store.add_observation({
        "class": "person", "confidence": 0.8,
        "world_x": 1.0, "world_y": 2.0, "world_z": 0.4,
        "position_dimension": 3,
        "position_quality": "range_lidar",
    })
    merged = store.add_observation({
        "class": "person", "confidence": 0.4,
        "world_x": 1.1, "world_y": 2.0, "world_z": 1.0,
        "position_dimension": 3,
        "position_quality": "range_lidar",
    })

    assert merged["world_z"] == pytest.approx(0.6)
    assert merged["z"] == pytest.approx(0.6)
    assert merged["position_dimension"] == 3


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


def test_artifact_failure_does_not_commit_marker(tmp_path):
    store = PersonMissionStore("m123", static_root=tmp_path)

    with pytest.raises(ValueError, match="frame must be a 2D or 3D image array"):
        store.add_observation({
            "class": "person",
            "confidence": 0.95,
            "world_x": 1.0,
            "world_y": 2.0,
            "position_quality": "range_lidar",
            "range_source": "lidar",
            "bbox": [10, 10, 50, 80],
        }, frame=np.array([1], dtype=np.uint8))

    assert store.markers() == []


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
