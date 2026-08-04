#!/usr/bin/env python3
"""Sample the raw Unitree lidar and summarize persistent near-field returns.

This diagnostic is read-only.  It subscribes directly to ``rt/utlidar/cloud``
without publishing commands, then reports per-degree minima so a self-filter can
be based on measured chassis sectors instead of a blanket near-range cutoff.
"""

import math
import os
import statistics
import struct
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_._PointCloud2_ import PointCloud2_


SAMPLE_FRAMES = int(os.environ.get("SAMPLE_FRAMES", "40"))
IFACE = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")


class Sampler:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = []
        self.point_samples = []
        self.metadata = None

    def callback(self, msg):
        minima = [math.inf] * 360
        step = int(msg.point_step)
        data = bytes(msg.data)
        frame_points = []
        for index in range(int(msg.width)):
            offset = index * step
            if offset + 12 > len(data):
                break
            x, y, z = struct.unpack_from("fff", data, offset)
            distance = math.hypot(x, y)
            if not math.isfinite(distance) or not 0.03 <= distance <= 3.0:
                continue
            if math.isfinite(z):
                frame_points.append((distance, z))
            degree = int(round(math.degrees(math.atan2(y, x))))
            bin_index = (degree + 180) % 360
            minima[bin_index] = min(minima[bin_index], distance)
        with self.lock:
            if len(self.frames) < SAMPLE_FRAMES:
                self.frames.append(minima)
                self.point_samples.extend(frame_points[::20])
                if self.metadata is None:
                    fields = [
                        (getattr(field, "name", ""), getattr(field, "offset", None))
                        for field in getattr(msg, "fields", [])
                    ]
                    header = getattr(msg, "header", None)
                    self.metadata = {
                        "frame_id": getattr(header, "frame_id", ""),
                        "point_step": step,
                        "fields": fields,
                    }


def main():
    sampler = Sampler()
    factory = ChannelFactory()
    factory.Init(0, IFACE)
    subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
    subscriber.Init(sampler.callback, 1)

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        with sampler.lock:
            count = len(sampler.frames)
        if count >= SAMPLE_FRAMES:
            break
        time.sleep(0.05)

    with sampler.lock:
        frames = list(sampler.frames)
        point_samples = list(sampler.point_samples)
        metadata = sampler.metadata
    print(f"frames={len(frames)} interface={IFACE}")
    print(f"metadata={metadata}")
    if not frames:
        raise SystemExit("no rt/utlidar/cloud frames received")

    persistent = []
    for bin_index in range(360):
        values = [frame[bin_index] for frame in frames if math.isfinite(frame[bin_index])]
        near = [value for value in values if value < 1.0]
        ratio = len(near) / len(frames)
        if ratio >= 0.50:
            angle = bin_index - 180
            persistent.append((angle, ratio, statistics.median(near), min(near)))

    print("persistent bins (<1m in >=50% frames):")
    for angle, ratio, median, minimum in persistent:
        print(
            f"angle={angle:+4d}deg ratio={ratio:4.2f} "
            f"median={median:5.3f}m min={minimum:5.3f}m"
        )

    all_near = []
    for frame in frames:
        all_near.extend(value for value in frame if math.isfinite(value) and value < 1.0)
    if all_near:
        all_near.sort()
        print(
            "all near returns: "
            f"count={len(all_near)} min={all_near[0]:.3f}m "
            f"median={statistics.median(all_near):.3f}m "
            f"p90={all_near[int(0.9 * (len(all_near) - 1))]:.3f}m"
        )

    for low, high in ((0.03, 0.10), (0.10, 0.30), (0.30, 0.60), (0.60, 1.00), (1.00, 3.00)):
        zs = sorted(z for distance, z in point_samples if low <= distance < high)
        if not zs:
            continue
        print(
            f"z for xy-range [{low:.2f},{high:.2f})m: n={len(zs)} "
            f"min={zs[0]:.3f} p10={zs[int(0.1 * (len(zs) - 1))]:.3f} "
            f"median={statistics.median(zs):.3f} "
            f"p90={zs[int(0.9 * (len(zs) - 1))]:.3f} max={zs[-1]:.3f}"
        )


if __name__ == "__main__":
    main()
