"""Bounded latest-value WebSocket delivery tests (ROS-free)."""

import asyncio
import json
import sys
import threading
from pathlib import Path

import pytest


WEB_DIR = Path(__file__).resolve().parent
if str(WEB_DIR) not in sys.path:
    sys.path.insert(0, str(WEB_DIR))

import nx_ws_latest  # noqa: E402
from nx_ws_latest import (  # noqa: E402
    LatestValueOutbox,
    ReliableQueueFull,
    classify_message,
    serialize_message,
)


def test_ten_gimbal_frames_retain_only_the_newest():
    outbox = LatestValueOutbox(reliable_capacity=8)

    for frame in range(10):
        outbox.enqueue({"type": "gimbal", "frame": frame})

    assert outbox.pending_count == 1
    assert outbox.stream_replaced == 9
    assert outbox.drain_nowait() == [{"type": "gimbal", "frame": 9}]


def test_reliable_events_keep_fifo_and_are_drained_before_streams():
    outbox = LatestValueOutbox(reliable_capacity=8)
    outbox.enqueue({"type": "gimbal", "frame": 1})
    events = [
        {"type": "mission_report", "seq": 1},
        {"type": "nav_goal", "seq": 2},
        {"type": "tasks", "seq": 3},
        {"type": "move_result", "seq": 4},
    ]
    for event in events:
        outbox.enqueue(event)
    outbox.enqueue({"type": "gimbal", "frame": 2})

    assert outbox.drain_nowait() == events + [
        {"type": "gimbal", "frame": 2},
    ]


def test_reliable_overflow_fails_closed_without_discarding_existing_events():
    outbox = LatestValueOutbox(reliable_capacity=2)
    first = {"type": "tasks", "seq": 1}
    second = {"type": "move_result", "seq": 2}
    outbox.enqueue(first)
    outbox.enqueue(second)

    with pytest.raises(ReliableQueueFull):
        outbox.enqueue({"type": "nav_goal", "seq": 3})

    assert outbox.drain_nowait() == [first, second]


def test_stream_classifier_includes_all_hot_paths_and_status_metadata():
    for event_type in (
        "gimbal", "lidar", "slam", "costmap", "costmap_global",
        "occupancy_map", "plan", "detections", "status", "frame",
    ):
        assert classify_message({"type": event_type}) == "stream"
    for event_type in ("mission_report", "nav_goal", "tasks", "move_result"):
        assert classify_message({"type": event_type}) == "reliable"


def test_slow_sender_does_not_block_another_client():
    class FakeSocket:
        def __init__(self, gate=None):
            self.gate = gate
            self.sent = []

        async def send(self, message):
            if self.gate is not None:
                await self.gate.wait()
            self.sent.append(json.loads(message))

    async def scenario():
        gate = asyncio.Event()
        slow_socket = FakeSocket(gate)
        fast_socket = FakeSocket()
        slow = LatestValueOutbox(reliable_capacity=8)
        fast = LatestValueOutbox(reliable_capacity=8)
        slow_task = asyncio.create_task(slow.send_forever(slow_socket, timeout=1.0))
        fast_task = asyncio.create_task(fast.send_forever(fast_socket, timeout=1.0))
        slow.enqueue({"type": "tasks", "seq": 1})
        fast.enqueue({"type": "tasks", "seq": 1})

        for _ in range(20):
            if fast_socket.sent:
                break
            await asyncio.sleep(0)
        assert fast_socket.sent == [{"type": "tasks", "seq": 1}]
        assert slow_socket.sent == []

        gate.set()
        slow.close()
        fast.close()
        await asyncio.gather(slow_task, fast_task)

    asyncio.run(scenario())


def test_close_clears_pending_state_and_wakes_sender():
    async def scenario():
        outbox = LatestValueOutbox(reliable_capacity=8)
        outbox.enqueue({"type": "tasks", "seq": 1})
        outbox.enqueue({"type": "gimbal", "frame": 1})
        outbox.close()

        assert outbox.closed is True
        assert outbox.pending_count == 0
        assert outbox.reliable_depth == 0
        assert await asyncio.wait_for(outbox.next_message(), 0.1) is None

    asyncio.run(scenario())


def test_producer_thread_safely_wakes_async_waiter():
    async def scenario():
        outbox = LatestValueOutbox(reliable_capacity=8)
        waiter = asyncio.create_task(outbox.next_message())
        await asyncio.sleep(0)  # bind the outbox to this running loop

        producer = threading.Thread(
            target=lambda: outbox.enqueue({"type": "tasks", "seq": 7}))
        producer.start()
        producer.join(timeout=1.0)

        assert producer.is_alive() is False
        assert await asyncio.wait_for(waiter, 0.2) == {
            "type": "tasks", "seq": 7,
        }

    asyncio.run(scenario())


def test_producer_thread_close_safely_wakes_async_waiter():
    async def scenario():
        outbox = LatestValueOutbox(reliable_capacity=8)
        waiter = asyncio.create_task(outbox.next_message())
        await asyncio.sleep(0)

        producer = threading.Thread(target=outbox.close)
        producer.start()
        producer.join(timeout=1.0)

        assert producer.is_alive() is False
        assert await asyncio.wait_for(waiter, 0.2) is None

    asyncio.run(scenario())


def test_serialized_payload_is_encoded_once_for_two_client_senders(monkeypatch):
    calls = []
    real_dumps = nx_ws_latest.json.dumps

    def counting_dumps(*args, **kwargs):
        calls.append(args[0])
        return real_dumps(*args, **kwargs)

    monkeypatch.setattr(nx_ws_latest.json, "dumps", counting_dumps)

    class FakeSocket:
        def __init__(self):
            self.sent = []

        async def send(self, payload):
            self.sent.append(payload)

    async def scenario():
        message = {"type": "tasks", "seq": 8}
        encoded = serialize_message(message)
        sockets = [FakeSocket(), FakeSocket()]
        outboxes = [LatestValueOutbox(), LatestValueOutbox()]
        tasks = [
            asyncio.create_task(outbox.send_forever(socket))
            for outbox, socket in zip(outboxes, sockets)
        ]
        for outbox in outboxes:
            outbox.enqueue_serialized(encoded)
        for _ in range(20):
            if all(socket.sent for socket in sockets):
                break
            await asyncio.sleep(0)
        for outbox in outboxes:
            outbox.close()
        await asyncio.gather(*tasks)

        assert sockets[0].sent == sockets[1].sent == [encoded.payload]

    asyncio.run(scenario())
    assert calls == [{"type": "tasks", "seq": 8}]


def test_server_uses_one_sender_per_client_and_force_is_compatibility_only():
    source = (WEB_DIR / "nx_web_server.py").read_text(encoding="utf-8")

    assert "_WS_OUTBOXES" in source
    assert "_register_ws" in source
    assert "_unregister_ws" in source
    assert "run_coroutine_threadsafe(_async_broadcast" not in source
    assert "del force" in source
    assert '"ws_stream_replaced"' in source
    assert '"ws_reliable_depth"' in source
    assert '"ws_connected_clients"' in source
