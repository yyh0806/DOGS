"""Bounded per-client WebSocket delivery with latest-value streams.

The producer-facing API is synchronous so ROS/background threads never wait on
network I/O.  A client owns one outbox and one async sender; stream updates
replace stale unsent values while control/task events retain FIFO ordering.
"""

from __future__ import annotations

import asyncio
import json
import threading
from collections import deque
from dataclasses import dataclass


CORE_STREAM_TYPES = frozenset({
    "gimbal",
    "lidar",
    "slam",
    "costmap",
    "costmap_global",
    "occupancy_map",
    "plan",
    "detections",
})

# These messages are snapshots too.  Treating them as reliable events would
# build a queue at their 6-30 Hz publish rates and deliver stale UI state.
STREAM_TYPES = CORE_STREAM_TYPES | frozenset({"status", "frame"})


def classify_message(message) -> str:
    """Return ``stream`` for replaceable snapshots, otherwise ``reliable``."""
    event_type = message.get("type") if isinstance(message, dict) else None
    return "stream" if event_type in STREAM_TYPES else "reliable"


class ReliableQueueFull(RuntimeError):
    """A reliable event could not be queued without losing an older event."""


@dataclass(frozen=True)
class SerializedMessage:
    """A message whose JSON payload was encoded once before client fanout."""

    message: object
    payload: str


def serialize_message(message) -> SerializedMessage:
    return SerializedMessage(
        message=message,
        payload=json.dumps(message, ensure_ascii=False),
    )


class LatestValueOutbox:
    """One bounded reliable FIFO plus one newest-unsent value per stream type."""

    def __init__(self, reliable_capacity=256, *, stream_types=STREAM_TYPES):
        capacity = int(reliable_capacity)
        if capacity < 1:
            raise ValueError("reliable_capacity must be positive")
        self._reliable_capacity = capacity
        self._stream_types = frozenset(stream_types)
        self._reliable = deque()
        self._streams = {}
        self._stream_replaced = 0
        self._closed = False
        self._lock = threading.Lock()
        self._ready = asyncio.Event()
        self._owner_loop = None

    @property
    def closed(self):
        with self._lock:
            return self._closed

    @property
    def reliable_depth(self):
        with self._lock:
            return len(self._reliable)

    @property
    def pending_count(self):
        with self._lock:
            return len(self._reliable) + len(self._streams)

    @property
    def stream_replaced(self):
        with self._lock:
            return self._stream_replaced

    def enqueue(self, message, *, notify=True):
        """Queue a message and return whether it replaced an unsent stream.

        ``notify=False`` is used by the server's thread-safe ingress buffer;
        that buffer schedules its own single event-loop drain callback.
        """
        return self._enqueue_item(message, message, notify=notify)

    def enqueue_serialized(self, serialized, *, notify=True):
        """Queue a centrally encoded payload without serializing per client."""
        if not isinstance(serialized, SerializedMessage):
            raise TypeError("serialized must be a SerializedMessage")
        return self._enqueue_item(
            serialized, serialized.message, notify=notify)

    def _enqueue_item(self, item, message, *, notify):
        event_type = message.get("type") if isinstance(message, dict) else None
        replaced = False
        with self._lock:
            if self._closed:
                return False
            if event_type in self._stream_types:
                replaced = event_type in self._streams
                if replaced:
                    self._stream_replaced += 1
                self._streams[event_type] = item
            else:
                if len(self._reliable) >= self._reliable_capacity:
                    raise ReliableQueueFull(
                        f"reliable WebSocket queue reached {self._reliable_capacity}")
                self._reliable.append(item)
        if notify:
            self._wake_owner_loop()
        return replaced

    def _bind_owner_loop(self):
        loop = asyncio.get_running_loop()
        with self._lock:
            if self._owner_loop is None:
                self._owner_loop = loop
            elif self._owner_loop is not loop:
                raise RuntimeError(
                    "LatestValueOutbox cannot be awaited from a second event loop")
        return loop

    def _wake_owner_loop(self):
        with self._lock:
            loop = self._owner_loop
        if loop is None or loop.is_closed():
            return
        try:
            current = asyncio.get_running_loop()
        except RuntimeError:
            current = None
        if current is loop:
            self._ready.set()
        else:
            try:
                loop.call_soon_threadsafe(self._ready.set)
            except RuntimeError:
                # The owning loop closed concurrently; there is no waiter left
                # that can be safely awakened.
                pass

    def _pop_nowait(self):
        with self._lock:
            if self._reliable:
                return self._reliable.popleft()
            if self._streams:
                stream_type = next(iter(self._streams))
                return self._streams.pop(stream_type)
            return None

    def drain_nowait(self):
        """Drain reliable FIFO first, followed by one latest value per stream."""
        drained = []
        while True:
            message = self._pop_nowait()
            if message is None:
                return drained
            drained.append(message)

    async def next_message(self):
        self._bind_owner_loop()
        while True:
            with self._lock:
                if self._reliable:
                    return self._reliable.popleft()
                if self._streams:
                    stream_type = next(iter(self._streams))
                    return self._streams.pop(stream_type)
                if self._closed:
                    return None
                # Clear while holding the queue lock so an enqueue cannot be
                # lost between checking the queues and beginning the wait.
                self._ready.clear()
            await self._ready.wait()

    async def send_forever(self, websocket, *, timeout=1.0):
        """Serialize and send this client's queue until closed or send failure."""
        while True:
            message = await self.next_message()
            if message is None:
                return
            payload = (
                message.payload
                if isinstance(message, SerializedMessage)
                else json.dumps(message, ensure_ascii=False)
            )
            send = websocket.send(payload)
            if timeout is None:
                await send
            else:
                await asyncio.wait_for(send, timeout=float(timeout))

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._reliable.clear()
            self._streams.clear()
        self._wake_owner_loop()
