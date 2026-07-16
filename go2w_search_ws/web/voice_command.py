"""PC voice/text entry point using the canonical search-mission contract."""

from __future__ import annotations

import json
import os
from urllib import error, request

from nx_mission_schema import SearchMissionRequest
from nx_product_command import parse_product_command


def parse_intent(text: str) -> dict:
    """Return a validated search intent or an explicit unknown result."""

    parsed = parse_product_command(text)
    if not parsed:
        return {"action": "unknown", "raw_text": str(text or "")}
    mission = SearchMissionRequest.from_dict(
        parsed["tasks"][0]["params"]["mission_request"])
    return {
        "action": "search_room",
        "raw_text": str(text),
        "mission_request": mission.to_dict(),
    }


def decompose_to_task(intent: dict) -> dict | None:
    """Serialize exactly one canonical mission; do not invent fallback moves."""

    if not isinstance(intent, dict) or intent.get("action") != "search_room":
        return None
    mission = SearchMissionRequest.from_dict(intent.get("mission_request"))
    return {"mission_request": mission.to_dict()}


def send_to_nx(task: dict, nx_url="http://localhost:8000", mock=True):
    """POST one authenticated canonical mission to the NX control API."""

    mission = SearchMissionRequest.from_dict(task.get("mission_request"))
    canonical_task = {"mission_request": mission.to_dict()}
    endpoint = nx_url.rstrip("/") + "/api/search_room"
    if mock:
        return {
            "status": "mock_sent",
            "endpoint": endpoint,
            "task": canonical_task,
        }
    token = os.environ.get("GO2W_CONTROL_TOKEN", "").strip()
    if not token:
        return {"status": "error", "reason": "control_token_not_configured"}
    body = json.dumps(
        canonical_task, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    http_request = request.Request(
        endpoint,
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json; charset=utf-8",
        },
    )
    try:
        with request.urlopen(http_request, timeout=10.0) as response:
            payload = json.loads(response.read().decode("utf-8"))
            return {"status": "sent", "http_status": response.status,
                    "response": payload}
    except (error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        return {"status": "error", "reason": str(exc)}


def parse_voice_command(text, nx_url="http://localhost:8000", mock=True):
    intent = parse_intent(text)
    task = decompose_to_task(intent)
    if task is None:
        return {
            "intent": intent,
            "task": None,
            "send_result": {"status": "no_task", "reason": "unknown_intent"},
        }
    return {
        "intent": intent,
        "task": task,
        "send_result": send_to_nx(task, nx_url=nx_url, mock=mock),
    }


def listen_microphone(timeout=5, phrase_limit=10):
    """Capture speech locally and transcribe with the existing Whisper path."""

    try:
        import io
        import speech_recognition as sr
        from faster_whisper import WhisperModel
    except ImportError as exc:
        return None, f"voice dependencies unavailable: {exc}"
    if not hasattr(listen_microphone, "_model"):
        listen_microphone._model = WhisperModel(
            "tiny", compute_type="int8", device="cpu")
    recognizer = sr.Recognizer()
    try:
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio = recognizer.listen(
                source, timeout=timeout, phrase_time_limit=phrase_limit)
        segments, _ = listen_microphone._model.transcribe(
            io.BytesIO(audio.get_wav_data()), language="zh")
        text = "".join(segment.text for segment in segments).strip()
        return (text, None) if text else (None, "empty_transcription")
    except Exception as exc:
        return None, str(exc)


def speak_and_command(nx_url="http://localhost:8000", mock=True):
    text, problem = listen_microphone()
    if problem:
        return {"error": problem}
    return parse_voice_command(text, nx_url=nx_url, mock=mock)


if __name__ == "__main__":
    import sys

    command = " ".join(sys.argv[1:]).strip()
    if command:
        print(json.dumps(parse_voice_command(command), ensure_ascii=False, indent=2))
    else:
        print(json.dumps(speak_and_command(mock=True), ensure_ascii=False, indent=2))

