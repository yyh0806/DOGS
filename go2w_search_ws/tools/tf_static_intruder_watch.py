#!/usr/bin/env python3
"""Catch the boot-time transient that publishes ``base_footprint -> base_link``.

Background (see memory "slam-nav-restart-loop-tf-dual-parent"):
``go2w-slam-nav``'s ``check_tf_topology`` gate failed 5x at boot because
``base_link`` had two parents — ``odom`` (from ``map_odom_fuser``, intended)
and ``base_footprint`` (from a transient ``/tf_static`` publisher that dies
shortly after boot). The supervisor sticky-set fix (``nav_health_supervisor.py``)
makes the gate recover from a *remembered* dead publisher, but the transient
itself is still a real TF-corruption source during its brief life. This script
runs once at boot to identify the publishing node so it can be removed.

Why per-message publisher attribution is hard:
ROS2 subscription callbacks do not receive the publisher's node identity, and
``/tf_static`` is latched (one shot). So this script samples the *current*
``/tf_static`` publisher set via ``get_publishers_info_by_topic`` every second
and logs it alongside the watch child's accumulated parents. When
``base_footprint`` appears as a parent of ``base_link``, the same log line shows
which publishers were live at that instant — any name other than the known-clean
``nx_sensor_node`` is the intruder.

Usage (on NX, at boot, as nx):
  python3 /home/nx/go2w/current/payload/tools/tf_static_intruder_watch.py \\
      --duration 180 --log /tmp/tf_intruder.log
  # then tail -f /tmp/tf_intruder.log and reboot / restart go2w-slam-nav
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0,
                        help="how long to watch (s); cover the full boot loop")
    parser.add_argument("--log", default="/tmp/tf_intruder.log")
    parser.add_argument("--watch-child", default="base_link")
    parser.add_argument("--known-clean", default="nx_sensor_node",
                        help="comma-separated node names known NOT to be the intruder")
    args = parser.parse_args()
    known_clean = {n.strip() for n in args.known_clean.split(",") if n.strip()}

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node("tf_static_intruder_watch")
    transient = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

    parents_static: dict[str, set[str]] = defaultdict(set)
    parents_dynamic: dict[str, set[str]] = defaultdict(set)
    first_seen: list = []

    def on_static(message: TFMessage) -> None:
        ts = time.time()
        for t in message.transforms:
            parents_static[t.child_frame_id].add(t.header.frame_id)
            first_seen.append((ts, "STATIC", t.header.frame_id, t.child_frame_id))

    def on_dynamic(message: TFMessage) -> None:
        ts = time.time()
        for t in message.transforms:
            parents_dynamic[t.child_frame_id].add(t.header.frame_id)
            if t.child_frame_id == args.watch_child:
                first_seen.append((ts, "DYNAMIC", t.header.frame_id, t.child_frame_id))

    node.create_subscription(TFMessage, "/tf_static", on_static, transient)
    node.create_subscription(TFMessage, "/tf", on_dynamic, reliable)

    log = open(args.log, "w", encoding="utf-8")
    def L(s: str) -> None:
        log.write(s + "\n")
        log.flush()

    L(f"=== tf_static_intruder_watch start duration={args.duration}s "
      f"watch_child={args.watch_child} known_clean={sorted(known_clean)} ===")
    start = time.monotonic()
    last_poll = 0.0
    flagged_once = False
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_poll >= 1.0:
                last_poll = now
                try:
                    static_pubs = node.get_publishers_info_by_topic("/tf_static")
                    dyn_pubs = node.get_publishers_info_by_topic("/tf")
                except Exception:
                    static_pubs = dyn_pubs = []
                sp_names = sorted({p.node_name for p in static_pubs})
                dp_names = sorted({p.node_name for p in dyn_pubs})
                wc_static = sorted(parents_static.get(args.watch_child, set()))
                wc_dyn = sorted(parents_dynamic.get(args.watch_child, set()))
                parents = sorted(set(wc_static) | set(wc_dyn))
                suspects = [n for n in sp_names if n not in known_clean]
                multi = len(parents) > 1
                flag = " <<< MULTI PARENT" if multi else ""
                L(f"[{time.strftime('%H:%M:%S')}] {args.watch_child} "
                  f"static={wc_static} dyn={wc_dyn}{flag}")
                L(f"    /tf_static pubs={sp_names}")
                L(f"    /tf        pubs={dp_names}")
                if multi:
                    L(f"    suspects (non known-clean /tf_static pubs)={suspects}")
                    flagged_once = True
    finally:
        L("=== static transforms seen (first-arrival order, filtered) ===")
        for entry in first_seen:
            ts, kind, parent, child = entry
            if child == args.watch_child or parent == "base_footprint":
                L(f"    {kind} {parent}->{child} @ {ts:.3f}")
        L(f"=== done (flagged={flagged_once}) ===")
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
