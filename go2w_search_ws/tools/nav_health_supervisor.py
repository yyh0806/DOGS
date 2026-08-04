#!/usr/bin/env python3
"""One long-lived DDS participant for all read-only Nav2 bringup gates."""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
from collections import deque
from pathlib import Path
from statistics import median


TOPICS = {
    "/wheel_odom": "odom",
    "/Odometry": "odom",
    "/odom": "odom",
    "/localization_pose": "odom",
    "/mid360/points_nav": "cloud",
    "/map_frontier": "map",
    "/dog_state": "string",
}
LIFECYCLE_NODES = (
    "controller_server", "smoother_server", "planner_server",
    "behavior_server", "bt_navigator", "velocity_smoother",
)
LIFECYCLE_QUERY_TIMEOUT_SEC = 2.0

# TF parent tracking — see memory "slam-nav-restart-loop-tf-dual-parent".
# The old `parents_*` sets only grew (.add), so a boot-transient TF publisher
# that published once and died poisoned the topology gate for the rest of the
# supervisor's life (the gate reads these sets to enforce "base_link has a
# single parent"). Dynamic and static parents need different expiry strategies
# because /tf streams continuously while /tf_static is latched-once.
DYNAMIC_PARENT_TTL_SEC = 5.0        # /tf edge not refreshed in 5s → publisher dead
STATIC_REFRESH_INTERVAL_SEC = 3.0   # recreate /tf_static subscription this often
STATIC_LATCH_GRACE_SEC = 1.0        # let fresh subscription receive latch before swap


def topic_snapshot(samples: deque[float], now: float) -> dict:
    if not samples:
        return {"samples": 0, "rate_hz": None, "age_sec": None}
    age = max(0.0, now - samples[-1])
    rate = None
    if len(samples) >= 2 and samples[-1] > samples[0]:
        rate = (len(samples) - 1) / (samples[-1] - samples[0])
    return {"samples": len(samples), "rate_hz": rate, "age_sec": age}


def stamp_age_snapshot(samples: deque[float]) -> dict:
    """Summarize a bounded latency history without hiding intermittent stalls."""
    if not samples:
        return {
            "stamp_samples": 0,
            "stamp_median_sec": None,
            "stamp_p95_sec": None,
        }
    ordered = sorted(float(value) for value in samples)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "stamp_samples": len(ordered),
        "stamp_median_sec": median(ordered),
        "stamp_p95_sec": ordered[p95_index],
    }


def lifecycle_query_expired(
    started_at: float,
    now: float,
    *,
    timeout: float = LIFECYCLE_QUERY_TIMEOUT_SEC,
) -> bool:
    """Return whether a lifecycle request must be replaced after a restart."""
    return float(now) - float(started_at) >= float(timeout)


def atomic_write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(data, stream, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def run(state_file: Path) -> None:
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from lifecycle_msgs.srv import GetState
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    import tf2_ros

    rclpy.init()
    node = Node("go2w_nav_health_supervisor")
    samples = {topic: deque(maxlen=40) for topic in TOPICS}
    stamp_ages = {topic: deque(maxlen=100) for topic in TOPICS}
    stamp_totals = {topic: 0 for topic in TOPICS}
    latest_strings: dict[str, str] = {}
    # Dynamic parents: child -> {parent -> last_seen_monotonic}. /tf publishers
    # stream continuously, so an edge not refreshed within DYNAMIC_PARENT_TTL_SEC
    # belongs to a dead publisher and is pruned.
    parents_dynamic_seen: dict[str, dict[str, float]] = {}
    # Static parents: child -> {parent}. /tf_static is latched-once, so a static
    # publisher that dies still has its latch cached in any subscription that was
    # alive at the time. We periodically recreate the /tf_static subscription:
    # a fresh subscriber receives latch only from currently-LIVE publishers, so
    # dead boot-transients drop out after one refresh cycle.
    parents_static: dict[str, set[str]] = {}
    lifecycle: dict[str, dict] = {}
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    transient = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def record(topic: str, kind: str):
        def callback(message):
            samples[topic].append(time.monotonic())
            if hasattr(message, "header"):
                stamp = message.header.stamp
                value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                stamp_ages[topic].append(
                    node.get_clock().now().nanoseconds * 1e-9 - value)
                stamp_totals[topic] += 1
            if kind == "string":
                latest_strings[topic] = str(message.data)
        return callback

    subscriptions = []
    for topic, kind in TOPICS.items():
        msg_type = {"odom": Odometry, "cloud": PointCloud2, "map": OccupancyGrid, "string": String}[kind]
        qos = transient if kind == "map" else reliable
        subscriptions.append(node.create_subscription(msg_type, topic, record(topic, kind), qos))

    def record_tf_dynamic(message: TFMessage) -> None:
        now_mono = time.monotonic()
        for transform in message.transforms:
            parents_dynamic_seen.setdefault(
                transform.child_frame_id, {})[transform.header.frame_id] = now_mono

    def record_tf_static(target: dict[str, set[str]]):
        def callback(message: TFMessage):
            for transform in message.transforms:
                target.setdefault(transform.child_frame_id, set()).add(
                    transform.header.frame_id)
        return callback

    subscriptions.append(node.create_subscription(TFMessage, "/tf", record_tf_dynamic, 100))
    static_sub = node.create_subscription(
        TFMessage, "/tf_static", record_tf_static(parents_static), transient)
    # 2-phase static refresh state. `pending_*` hold a freshly-created
    # subscription collecting latch from live publishers; after the grace
    # window we destroy the old subscription and swap `parents_static` to the
    # fresh dict. No-gap: the old dict serves reads until the swap.
    pending_static_target = None
    pending_static_sub = None
    pending_static_since = 0.0
    last_static_refresh = time.monotonic()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
    clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in LIFECYCLE_NODES}
    pending = {}
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    last_poll = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_poll >= 0.5:
                last_poll = now
                for name, client in clients.items():
                    pending_request = pending.get(name)
                    if pending_request is not None:
                        future, started_at = pending_request
                        if future.done():
                            try:
                                current = future.result().current_state
                                lifecycle[name] = {
                                    "id": int(current.id),
                                    "label": str(current.label),
                                }
                            except Exception as exc:
                                lifecycle[name] = {
                                    "id": None,
                                    "label": "error",
                                    "error": str(exc),
                                }
                            pending.pop(name, None)
                        elif lifecycle_query_expired(started_at, now):
                            future.cancel()
                            pending.pop(name, None)
                    if name not in pending and client.service_is_ready():
                        pending[name] = (
                            client.call_async(GetState.Request()),
                            now,
                        )

                # ---- Expire dead dynamic (/tf) parents by last-seen ----
                for child in list(parents_dynamic_seen.keys()):
                    pmap = parents_dynamic_seen[child]
                    for parent in list(pmap.keys()):
                        if now - pmap[parent] > DYNAMIC_PARENT_TTL_SEC:
                            del pmap[parent]
                    if not pmap:
                        del parents_dynamic_seen[child]
                parents_dynamic_view = {
                    child: sorted(pmap)
                    for child, pmap in parents_dynamic_seen.items()
                    if pmap
                }

                # ---- Refresh /tf_static subscription to drop dead publishers ----
                # Phase 1: open a fresh subscription collecting into a temp dict.
                if (pending_static_target is None
                        and now - last_static_refresh >= STATIC_REFRESH_INTERVAL_SEC):
                    pending_static_target = {}
                    pending_static_sub = node.create_subscription(
                        TFMessage, "/tf_static",
                        record_tf_static(pending_static_target), transient)
                    pending_static_since = now
                    last_static_refresh = now
                # Phase 2: after grace, the fresh dict reflects only live
                # publishers' latches — swap it in and retire the old sub.
                if (pending_static_target is not None
                        and now - pending_static_since >= STATIC_LATCH_GRACE_SEC):
                    node.destroy_subscription(static_sub)
                    static_sub = pending_static_sub
                    parents_static = pending_static_target
                    pending_static_target = None
                    pending_static_sub = None

                tf_state = {}
                for parent, child in (("camera_init", "body"), ("odom", "base_link"), ("map", "odom"), ("map", "base_link")):
                    try:
                        tf_state[f"{parent}->{child}"] = tf_buffer.can_transform(
                            parent, child, rclpy.time.Time())
                    except Exception:
                        tf_state[f"{parent}->{child}"] = False
                data = {
                    "updated_wall": time.time(),
                    "topics": {
                        topic: {
                            **topic_snapshot(value, now),
                            **stamp_age_snapshot(stamp_ages[topic]),
                            "stamp_total": stamp_totals[topic],
                        }
                        for topic, value in samples.items()
                    },
                    "dog_state": latest_strings.get("/dog_state"),
                    "tf": tf_state,
                    "parents_dynamic": parents_dynamic_view,
                    "parents_static": {key: sorted(value) for key, value in parents_static.items()},
                    "lifecycle": lifecycle,
                    "actions": {"/navigate_to_pose": action.server_is_ready()},
                }
                atomic_write(state_file, data)
    finally:
        action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-file", required=True)
    args = parser.parse_args()
    run(Path(args.state_file))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
