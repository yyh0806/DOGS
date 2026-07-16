# PC Voice Room-Search Command Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the Chinese microphone transcript “去搜索这个房间，把所有人标注出来” into exactly one feedback-confirmed `search_room` task submission without allowing unrelated speech to move the robot.

**Architecture:** Keep speech-to-text on the PC with the installed offline Vosk model. Reuse the model-free `nx_product_command.parse_product_command` locally as an allow-list, post the original transcript to NX `/api/command`, and treat the command as sent only when NX returns `accepted: true`. A stateful dispatcher suppresses duplicate final transcripts for a bounded interval, and a text mode exercises the same validation and dispatch path without microphone hardware.

**Tech Stack:** Python 3.12, Vosk, sounddevice, requests, NX HTTP API, pytest.

---

### Task 1: Add a deterministic PC-side search-command safety gate

**Files:**
- Modify: `tools/test_voice_console.py`
- Modify: `tools/voice_console.py`

- [x] **Step 1: Write failing gate tests**

Add tests showing that spaced STT output for the exact product phrase returns the canonical `search_room` task, while “前进两米” and negated search commands return `unsupported_voice_command`.

- [x] **Step 2: Run the focused tests and verify RED**

Run: `python -m pytest tools/test_voice_console.py -q`

Expected: FAIL because `validate_search_command` does not exist.

- [x] **Step 3: Implement the minimal gate**

Import `parse_product_command` from `web/nx_product_command.py`, verify the returned task is `search_room` with `target_classes == ["person"]`, `require_photos`, and `mark_on_map`, and return a stable task fingerprint for duplicate detection.

- [x] **Step 4: Run the focused tests and verify GREEN**

Run: `python -m pytest tools/test_voice_console.py -q`

Expected: all gate tests pass.

### Task 2: Require confirmed NX admission and suppress duplicate STT finals

**Files:**
- Modify: `tools/test_voice_console.py`
- Modify: `tools/voice_console.py`

- [x] **Step 1: Write failing HTTP and duplicate tests**

Use a local HTTP server to assert the client posts `{"text": <transcript>}` and reports success only for an HTTP response whose JSON has both `ok: true` and `accepted: true`. Add dispatcher tests proving a second accepted transcript with the same task fingerprint is suppressed during the cooldown, while failed admission is retryable.

- [x] **Step 2: Run the focused tests and verify RED**

Run: `python -m pytest tools/test_voice_console.py -q`

Expected: FAIL because the existing sender treats every 2xx as success and has no dispatcher.

- [x] **Step 3: Implement the sender and dispatcher**

Preserve NX response fields, add `transport_ok`, map unconfirmed 2xx responses to `admission_unconfirmed`, and record a duplicate fingerprint only after confirmed admission. Print the NX rejection reason instead of a generic HTTP success.

- [x] **Step 4: Run the focused tests and verify GREEN**

Run: `python -m pytest tools/test_voice_console.py -q`

Expected: all sender and duplicate tests pass.

### Task 3: Wire microphone and text-mode execution to the guarded dispatcher

**Files:**
- Modify: `tools/test_voice_console.py`
- Modify: `tools/voice_console.py`
- Modify: `requirements-voice.txt`

- [x] **Step 1: Write failing CLI contract tests**

Require `--text`, `--dedupe-seconds`, and `--no-auto-send` in help. Run `--text "去 搜索 这个 房间，把 所有 人 标注 出来" --no-auto-send` and require a successful validation result without loading Vosk or opening the microphone.

- [x] **Step 2: Run the CLI tests and verify RED**

Run: `python -m pytest tools/test_voice_console.py -q`

Expected: FAIL because the arguments and text-only branch do not exist.

- [x] **Step 3: Implement one shared dispatch path**

Construct one `SearchCommandDispatcher` for microphone finals and text mode. Unsupported speech is ignored, confirmed submission is announced, and `--no-auto-send` validates without network mutation. Keep Vosk, TTS, and WebSocket imports optional until their runtime paths are selected.

- [x] **Step 4: Verify software and installed voice assets**

Run:

```powershell
python -m pytest tools/test_voice_console.py web/test_voice_search_contract.py web/test_panel_navigation_contract.py -q
python tools/voice_console.py --text "去 搜索 这个 房间，把 所有 人 标注 出来" --no-auto-send
python -c "from vosk import Model; Model('models/vosk-model-small-cn-0.22'); print('model-ok')"
```

Expected: tests pass, text validation prints the canonical `search_room` task without an HTTP request, and the offline model loads successfully.
