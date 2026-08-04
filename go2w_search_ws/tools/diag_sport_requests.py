#!/usr/bin/env python3
"""Summarize Unitree sport RPC requests without creating a control client."""

import collections
import json
import os
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
from unitree_sdk2py.idl.unitree_api.msg.dds_._Request_ import Request_


SAMPLE_SECONDS = float(os.environ.get("SAMPLE_SECONDS", "5"))
DOG_INTERFACE = os.environ.get("DOG_INTERFACE", "enxc8a362616c4c")

lock = threading.Lock()
requests = collections.Counter()


def on_request(message):
    try:
        key = (
            int(message.header.lease.id),
            int(message.header.identity.api_id),
            int(message.header.policy.priority),
            bool(message.header.policy.noreply),
            str(message.parameter),
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return
    with lock:
        requests[key] += 1


factory = ChannelFactory()
factory.Init(0, DOG_INTERFACE)
subscriber = ChannelSubscriber("rt/api/sport/request", Request_)
subscriber.Init(on_request, 100)

started = time.monotonic()
time.sleep(SAMPLE_SECONDS)
elapsed = time.monotonic() - started

with lock:
    snapshot = list(requests.items())

rows = []
for (lease_id, api_id, priority, noreply, parameter), count in snapshot:
    try:
        decoded_parameter = json.loads(parameter)
    except (json.JSONDecodeError, TypeError):
        decoded_parameter = parameter
    rows.append({
        "lease_id": lease_id,
        "api_id": api_id,
        "priority": priority,
        "noreply": noreply,
        "parameter": decoded_parameter,
        "count": count,
        "rate_hz": round(count / elapsed, 3),
    })

rows.sort(key=lambda row: (
    row["lease_id"], row["api_id"], json.dumps(row["parameter"], sort_keys=True)))
print(json.dumps({
    "interface": DOG_INTERFACE,
    "sample_seconds": round(elapsed, 3),
    "requests": rows,
}, separators=(",", ":")), flush=True)
