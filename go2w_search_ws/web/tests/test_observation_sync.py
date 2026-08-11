import math

import pytest

from nx_observation_sync import ObservationSynchronizer


def test_observation_uses_pose_at_camera_capture():
    sync = ObservationSynchronizer()
    sync.add_pose(stamp=10.0, x=0.0, y=0.0, yaw=0.0)
    sync.add_pose(stamp=11.0, x=1.0, y=0.0, yaw=0.0)
    scan = object()
    sync.add_scan(stamp=10.1, scan=scan)
    bundle = sync.bundle_for_detection(stamp=10.1, tolerance=0.15)
    assert bundle.pose.x == pytest.approx(0.1, abs=0.02)
    assert bundle.scan.value is scan
    assert bundle.capture_stamp == 10.1


def test_unsynchronized_scan_is_rejected():
    sync = ObservationSynchronizer()
    sync.add_pose(stamp=10.0, x=0.0, y=0.0, yaw=0.0)
    sync.add_scan(stamp=20.0, scan=object())
    assert sync.bundle_for_detection(stamp=10.0, tolerance=0.15) is None


def test_yaw_interpolation_wraps_across_pi():
    sync = ObservationSynchronizer()
    sync.add_pose(stamp=1.0, x=0, y=0, yaw=math.radians(179))
    sync.add_pose(stamp=3.0, x=0, y=0, yaw=math.radians(-179))
    sync.add_scan(stamp=2.0, scan=object())
    bundle = sync.bundle_for_detection(stamp=2.0, tolerance=0.1)
    assert abs(abs(bundle.pose.yaw) - math.pi) < math.radians(1)


def test_histories_are_bounded_and_nonfinite_samples_are_rejected():
    sync = ObservationSynchronizer(max_samples=3)
    with pytest.raises(ValueError):
        sync.add_pose(stamp=math.nan, x=0, y=0, yaw=0)
    for index in range(5):
        sync.add_scan(stamp=float(index + 1), scan=index)
    assert sync.snapshot()["scan_samples"] == 3
