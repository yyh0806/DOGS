from pathlib import Path
import re


SOURCE = Path(__file__).resolve().parent / "nx_web_server.py"
OUTBOX_SOURCE = Path(__file__).resolve().parent / "nx_ws_latest.py"


def test_ws_broadcast_uses_bounded_latest_value_ingress():
    text = SOURCE.read_text(encoding="utf-8")

    assert "LatestValueOutbox," in text
    assert "ReliableQueueFull," in text
    assert "_WS_INGRESS = LatestValueOutbox" in text
    assert "loop.call_soon_threadsafe(_drain_ws_ingress)" in text
    assert "run_coroutine_threadsafe(_async_broadcast" not in text
    assert "del force" in text


def test_each_client_has_one_sender_with_a_send_timeout():
    server = SOURCE.read_text(encoding="utf-8")
    outbox = OUTBOX_SOURCE.read_text(encoding="utf-8")

    assert "_WS_OUTBOXES[ws] = outbox" in server
    assert "_WS_SENDER_TASKS[ws] = sender" in server
    assert "outbox.send_forever(ws, timeout=_WS_SEND_TIMEOUT)" in server
    assert "await asyncio.wait_for(send, timeout=float(timeout))" in outbox


def test_ingress_drain_handles_one_snapshot_then_yields_to_event_loop():
    server = SOURCE.read_text(encoding="utf-8")
    drain = server.split("def _drain_ws_ingress():", 1)[1].split(
        "\ndef _schedule_ws_disconnect", 1)[0]

    assert "while True" not in drain
    assert "_WS_INGRESS_SCHEDULED = False" in drain
    assert "messages = _WS_INGRESS.drain_nowait()" in drain


def test_message_is_serialized_once_before_per_client_fanout():
    server = SOURCE.read_text(encoding="utf-8")
    drain = server.split("def _drain_ws_ingress():", 1)[1].split(
        "\ndef _schedule_ws_disconnect", 1)[0]

    assert drain.index("serialized = serialize_message(message)") < drain.index(
        "for ws, outbox in clients:")
    assert "outbox.enqueue_serialized(serialized)" in drain
    assert 'classify_message(message) == "reliable"' in drain
    assert '"reliable serialization failure"' in drain


def test_ws_server_has_modern_startup_and_deterministic_shutdown():
    server = SOURCE.read_text(encoding="utf-8")

    assert "async def _start_ws_server(websockets, host, port):" in server
    assert "return await websockets.serve(h, host, port)" in server
    assert "async def h(ws, path=None):" in server
    assert "async def _shutdown_ws_server(server):" in server
    assert "server.close()" in server
    assert "server.wait_closed()" in server
    assert "await asyncio.wait_for(" in server
    assert "sender.cancel()" in server
    assert "await asyncio.gather(*senders, return_exceptions=True)" in server
    assert "WS_LOOP = None" in server
    assert "loop.close()" in server


def test_frame_broadcast_does_not_include_unused_jpeg_payload():
    text = SOURCE.read_text(encoding="utf-8")

    assert '"type": "frame", "data": b64' not in text
    assert '"type": "frame", "detections": int(det_count)' in text
    assert "ai_engine.get_frame_jpeg()" not in text
    assert "ai_engine.get_frame_detection_count()" in text


def test_costmap_broadcast_is_latest_value_and_skips_unchanged_snapshots():
    text = SOURCE.read_text(encoding="utf-8")
    outbox = OUTBOX_SOURCE.read_text(encoding="utf-8")

    # Costmaps keep only their newest unsent snapshot.  The producer still
    # avoids enqueueing a snapshot when the IPC file has not changed.
    assert '"costmap",' in outbox
    assert "self._streams[event_type] = item" in outbox
    assert "def _broadcast_json_if_changed(path, event_type, *, force=True):" in text
    assert re.search(
        r'_broadcast_json_if_changed\(\s*[\'\"]?/tmp/costmap_lite\.json[\'\"]?,\s*[\'\"]costmap[\'\"],\s*force=True\)',
        text,
    )
    assert "modified <= float(ipc_mtimes.get(path, 0.0))" in text
    assert "if force:" not in text


def test_realtime_bridges_are_stopped_on_process_exit():
    text = SOURCE.read_text(encoding="utf-8")

    assert "gimbal_bridge.stop()" in text
    assert "lidar_bridge.stop()" in text
