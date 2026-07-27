# Live Search, UI, and Voice Hardening Implementation Plan

> **For agentic workers:** Execute this plan task-by-task with test-driven development and fresh verification evidence. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix every defect observed in the July 23 real-NX search run: unsafe terminal frontier goals, late global-completion evaluation, untyped synchronized scans, misleading frontend search/perception state, and a narrow local-LLM semantic gap.

**Architecture:** Keep the existing global occupancy/visibility planner and safety gate. Add a known-free terminal continuation check at each proposed endpoint so unknown space is approached incrementally instead of used as a one-way stopping pose; evaluate closed-boundary completion on fresh map revisions even while noisy candidates remain. Normalize scan samples at the observation boundary. Present mission results separately from instantaneous detections, and expose the backend's distinct visual/global/closure metrics. Keep LLM output behind deterministic command validation, adding only explicit grounded semantic aliases.

**Tech Stack:** Python 3, pytest, ROS 2 Humble/Nav2, vanilla JavaScript/HTML, Node.js contract tests, Vosk/Ollama local voice NLU, repository release/deployment scripts.

---

### Task 1: Lock the live failure modes with regression tests

**Files:**
- Modify: `web/test_visibility_coverage.py`
- Modify: `web/test_exploration_manager.py`
- Modify: `web/test_room_orchestrator.py`
- Modify: `web/test_nx_web_server_contract.py`

- [ ] Add a terminal-frontier case where the requested endpoint touches unknown space and has no known-free continuation; require a shortened, safe endpoint.
- [ ] Add a 0.8 m known-free doorway/corridor case; require the candidate to remain eligible when it has forward continuation despite insufficient turning radius.
- [ ] Add a global-closure case with explainable coverage above 95% and residual noise frontiers; require stable completion without entering another planning probe.
- [ ] Add a synchronized detection bundle containing a legacy scan dictionary; require conversion to `LaserScanSnapshot` before localization.
- [ ] Run the focused tests and record the expected RED failures before production changes.

### Task 2: Fix search endpoint safety and global completion

**Files:**
- Modify: `web/nx_visibility_coverage.py`
- Modify: `web/nx_exploration_manager.py`

- [ ] Compute terminal heading from the actual arrival segment, not the candidate's display yaw.
- [ ] Raycast from the terminal pose through known-free map cells and reserve at least one adaptive/minimum motion step when turning is blocked.
- [ ] Treat unknown cells as an unsafe terminal stopping boundary, not as a permanent wall; shorten the goal into known-free space so the next scan can reveal the doorway or room.
- [ ] Preserve 0.8 m passages when a known-free forward exit exists.
- [ ] Evaluate boundary closure plus explainable coverage on distinct map revisions before probing remaining candidates; retain the existing stability streak and explicit evidence.
- [ ] Run visibility, exploration-manager, frontier, and global-search tests.

### Task 3: Fix observation synchronization

**Files:**
- Modify: `web/nx_web_server.py`
- Modify: `web/nx_room_orchestrator.py`
- Modify: relevant tests from Task 1

- [ ] Store typed `LaserScanSnapshot` values in `ObservationSynchronizer`.
- [ ] Defensively normalize older dictionary samples at the orchestrator boundary.
- [ ] Reject malformed or stale scans explicitly instead of raising an attribute error inside target localization.
- [ ] Run observation, localization, orchestrator, and web-server tests.

### Task 4: Fix the frontend's representation of the real mission

**Files:**
- Modify: `web/static/map.js`
- Modify: `web/static/panel.html`
- Modify: `web/test_map_contract.js`
- Modify: `web/test_panel_nav_state.js`
- Modify: `web/test_panel_navigation_contract.py`

- [ ] Add RED contracts for distinct visual/global coverage and closure state.
- [ ] Prefer `motion_trapped` and its evidence over the generic `nav2_aborted` banner for a failed search.
- [ ] Keep mission target markers/results visible when a later live frame contains zero instantaneous detections.
- [ ] Derive the C13 health/FPS display from successful frame refreshes plus backend perception health instead of leaving `0 FPS`/`unknown`.
- [ ] Add an inline favicon to eliminate the only browser-console 404.
- [ ] Run Node/pytest frontend contracts, then inspect the real page in a browser.

### Task 5: Close the safe voice-semantic gap

**Files:**
- Modify: `tools/test_local_llm_nlu.py`
- Modify: `tools/local_llm_nlu.py`
- Modify: `tools/test_voice_console.py` if the integration contract requires it

- [ ] Add a RED case for “可以坐的东西/座位” and other explicitly grounded aliases.
- [ ] Canonicalize those aliases to supported detector classes before the deterministic command gate.
- [ ] Prove unsupported free-form model output is still rejected and all admitted movement/search commands use the autonomous navigation channel.
- [ ] Run local NLU and voice-console tests.

### Task 6: Verify, package, deploy, and inspect the real NX

**Files:**
- Modify only if a verification contract exposes a scoped defect.

- [ ] Run all focused suites, the repository's full offline gate, syntax checks, and `git diff --check`.
- [ ] Review the exact dirty-worktree diff without changing the existing `demo` tag or discarding prior user work.
- [ ] Build a content-addressed release through the repository scripts and deploy it to the NX host exposed as `192.168.1.200`.
- [ ] Confirm Web/Nav2/mapping/motion/perception health and release IDs while the dog remains stopped.
- [ ] Inspect the real frontend and APIs for correct mission failure/completion, target-marker persistence, C13 health, and search metrics.
- [ ] Do not start physical motion; hand the operator the exact voice command for a supervised end-to-end search retest.
