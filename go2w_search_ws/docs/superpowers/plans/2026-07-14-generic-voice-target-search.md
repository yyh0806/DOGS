# Generic Voice Target Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make “搜索这个房间，把所有桌子标记出来” produce a bounded room-search mission that detects `dining table`, localizes each table with MID360 range, de-duplicates observations, saves evidence, and renders class-labelled map markers while preserving person search.

**Architecture:** Natural-language parsing emits canonical detector classes (`桌子` → `dining table`) and the PC voice guard allows only deterministic `search_room` results. The AI engine exposes a class-filtered snapshot and a mission-scoped detection vocabulary so both YOLO and YOLO-World receive the requested classes. Existing bearing/range localization and mission storage become class-aware, while legacy person APIs/events remain compatibility wrappers.

**Tech Stack:** Python 3.12, ultralytics YOLO/YOLO-World, Vosk, MID360 LaserScan/PointCloud, pytest, browser JavaScript/Node contract tests.

---

### Task 1: Parse and safely dispatch table-search speech

**Files:**
- Modify: `web/test_product_command.py`
- Modify: `web/test_voice_search_contract.py`
- Modify: `tools/test_voice_console.py`
- Modify: `web/nx_product_command.py`
- Modify: `tools/voice_console.py`

- [ ] Add failing tests for “去搜索这个房间，把所有桌子标记出来” and spaced Vosk output; require `search_room`, `target_classes=["dining table"]`, photos, map marking, frontier exploration, bounded radius/time, and no acceptance of unrelated motion speech.
- [ ] Run the three focused test modules and confirm failure because the parser/guard only accepts `person`.
- [ ] Add deterministic Chinese target aliases and generate the canonical class without accepting arbitrary free-form speech; change the guard from `== ["person"]` to a non-empty supported target-class allow-list.
- [ ] Re-run the focused tests and preserve every existing person/negation/named-room contract.

### Task 2: Make the live detector snapshot mission-target aware

**Files:**
- Modify: `web/test_ai_snapshot_contract.py`
- Modify: `web/nx_ai_node.py`

- [ ] Add failing tests that `get_detection_snapshot(["dining table"])` returns table detections and excludes people, while `get_person_detection_snapshot()` remains a person-only compatibility wrapper.
- [ ] Add a failing test that a mission target vocabulary is passed into `Detector.detect(frame, target_classes=...)`, which is required for YOLO-World `set_classes`.
- [ ] Implement a locked, mission-scoped detection-target setter, pass its snapshot into the one video inference thread, and add the generic snapshot API with frame/bbox scaling identical to the person path.
- [ ] Re-run AI snapshot/video tests.

### Task 3: Generalize localization, de-duplication, artifacts, and mission orchestration

**Files:**
- Modify: `web/test_person_localizer.py`
- Modify: `web/test_person_mission.py`
- Modify: `web/test_frontier_explore.py`
- Modify: `web/test_product_room_orchestrator.py`
- Modify: `web/nx_person_localizer.py`
- Modify: `web/nx_person_mission.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: `web/nx_web_server.py`

- [ ] Add failing geometry tests proving table bboxes use the same camera-bearing plus LiDAR-range projection and retain class `dining table`.
- [ ] Add failing store tests proving two nearby table observations merge into `dining_table_001`, different classes never merge, and photo/JSON URLs use the target ID.
- [ ] Add failing frontier/orchestrator tests proving target classes are never coerced to person, the generic snapshot is consumed, detection vocabulary is restored at mission end, and the mission report retains table markers.
- [ ] Introduce `localize_target_detection` and `TargetMissionStore` while keeping `localize_person_detection` and `PersonMissionStore` as compatible person defaults.
- [ ] Replace person-only snapshot/filter/coercion in the product and frontier flows with requested-class filtering; accept both `use_lidar_target_range` and legacy `use_lidar_person_range`.
- [ ] Re-run localizer, mission, frontier, room-orchestrator, and HTTP contract tests.

### Task 4: Render generic target markers and verify the whole software chain

**Files:**
- Modify: `web/test_map_contract.js`
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`

- [ ] Add failing Node contracts for `target_markers` WebSocket handling and class-aware labels (`人1`, `dining table 1`) while preserving `person_markers` compatibility.
- [ ] Store/render generic target markers, keep clickable evidence URLs, and consume both live marker and mission-report detections.
- [ ] Run Node contracts, the focused Python suite, `py_compile`, and the full `pytest docker web tools -q` suite.
- [ ] Validate the exact table command with `tools/voice_console.py --text ... --no-auto-send`; do not deploy or send it while the robot is charging.

