#!/usr/bin/env python3
"""Wait for ROS 2 lifecycle nodes to report the active state.

The probe is read-only: it calls each node's ``get_state`` service and never
requests a lifecycle transition.  A single rclpy participant is more reliable
on the NX during heavy Nav2 startup than repeatedly launching ros2cli discovery
processes.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Mapping, Sequence


ACTIVE_STATE_ID = 3


def normalize_nodes(values: Sequence[str]) -> tuple[str, ...]:
    nodes: list[str] = []
    for raw in values:
        name = str(raw).strip().strip("/")
        if not name:
            raise ValueError("lifecycle node name must be non-empty")
        if name not in nodes:
            nodes.append(name)
    if not nodes:
        raise ValueError("at least one lifecycle node is required")
    return tuple(nodes)


def states_are_active(
    nodes: Sequence[str], states: Mapping[str, Mapping[str, object]]
) -> bool:
    for name in nodes:
        state = states.get(name) or {}
        try:
            state_id = int(state.get("id", -1))
        except (TypeError, ValueError, OverflowError):
            return False
        if state_id != ACTIVE_STATE_ID:
            return False
        if str(state.get("label", "")).strip().lower() != "active":
            return False
    return True


def wait_lifecycle_active(
    nodes: Sequence[str], timeout: float, *, poll_interval: float = 0.1
) -> dict[str, object]:
    try:
        import rclpy
        from lifecycle_msgs.srv import GetState
        from rclpy.node import Node
    except ImportError as exc:
        return {"ok": False, "states": {}, "error": f"ROS unavailable: {exc}"}

    requested = normalize_nodes(nodes)
    deadline = time.monotonic() + float(timeout)
    states: dict[str, dict[str, object]] = {}
    rclpy.init()
    node = Node(f"go2w_lifecycle_readiness_probe_{os.getpid()}")
    clients = {
        name: node.create_client(GetState, f"/{name}/get_state")
        for name in requested
    }
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            for name, client in clients.items():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                if not client.wait_for_service(
                        timeout_sec=min(0.25, remaining)):
                    states[name] = {"id": None, "label": "service_unavailable"}
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    node, future, timeout_sec=min(1.0, remaining))
                if not future.done():
                    future.cancel()
                    states[name] = {"id": None, "label": "query_timeout"}
                    continue
                try:
                    response = future.result()
                    current = response.current_state
                    states[name] = {
                        "id": int(current.id),
                        "label": str(current.label),
                    }
                except Exception as exc:  # ROS service failures are diagnostic.
                    states[name] = {
                        "id": None,
                        "label": "query_error",
                        "error": str(exc),
                    }
            if states_are_active(requested, states):
                return {"ok": True, "states": states}
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                rclpy.spin_once(
                    node, timeout_sec=min(float(poll_interval), remaining))
        return {"ok": False, "states": states, "error": "timeout"}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only wait for ROS lifecycle active state")
    parser.add_argument("nodes", nargs="+")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0.2 <= args.timeout <= 300.0:
        parser.error("--timeout must be in [0.2, 300]")
    try:
        nodes = normalize_nodes(args.nodes)
        report = wait_lifecycle_active(nodes, args.timeout)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
