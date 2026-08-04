# Claude Code execution task: eliminate stale FAST_LIO navigation pose

## Role and limits

You are the implementation executor. Codex owns architecture, review, NX
deployment, and physical testing. Work only in the current dirty worktree.
Preserve all existing user changes. Do not stage, commit, deploy, SSH to the
NX, or send any robot command.

Use test-driven development: add the smallest failing contract tests first,
run them and record the expected failures, then implement, then rerun focused
tests. Do not claim the physical problem is fixed; only report code/test work.

## Proven incident

An authenticated Nav2 goal from map pose approximately `(0.011, 0.015)` to
`(0.310, 0.015)` was accepted. Runtime controller parameters were correct:
10 Hz, 0.10 m goal tolerance, 0.22 m/s minimum DWB speed, 0.30 m/s maximum.
Nav2 reported success, but final map pose was about `(0.62, -0.02)`. Gateway
logs show the SDK was commanded for about 3.1 s and the command integral is
about 0.55 m.

Read-only NX evidence:

- `/livox/lidar` header delay: typically 0.4--0.9 s.
- `/Odometry` header delay: typically 1.3--2.0 s, sometimes above 3 s.
- `FAST_LIO_ROS2/src/laserMapping.cpp` subscribes to Livox CustomMsg with
  reliable queue depth 20 and processes `lidar_buffer.front()`.
- The fuser currently relabels every old `/Odometry` pose with wall-clock now,
  hiding staleness from tf2/Nav2.
- The Python Livox watchdog subscribes to the full CustomMsg with default
  reliable QoS and consumes almost one CPU core just to note receipt.
- Nav2 logs confirm the requested goal was `(0.31, 0.01)`, not a frontend
  coordinate error.

## Required implementation

Implement a small, reproducible, fail-closed repair. Keep the scope narrow.

1. Add an idempotent FAST_LIO source patch under `docker/patches/` for the
   exact installed `FAST_LIO_ROS2/src/laserMapping.cpp` pattern. It must:
   - replace the Livox `CustomMsg` reliable depth-20 subscription with sensor
     data QoS using best-effort and keep-last 1;
   - prevent an internal `lidar_buffer` backlog by retaining the newest
     complete scan when the estimator has not yet claimed the current front;
   - keep `lidar_buffer` and `time_buffer` synchronized;
   - avoid invalidating a scan already claimed through `lidar_pushed`.

2. Add `docker/prepare_fastlio_low_latency.sh`, an idempotent helper that:
   - defaults to `/home/nx/ws_livox` but supports `FASTLIO_WS` override;
   - validates the expected source tree and patch preimage;
   - applies the patch once (recognizes already-patched state);
   - rebuilds only package `fast_lio` in Release mode when source is newly
     patched or the installed executable is older than the source;
   - fails with a clear error on an unknown source version;
   - never starts/stops any service and never controls the robot.

3. Add a release-owned optimized config at
   `src/go2w_nav/config/fastlio_low_latency/mid360.yaml`. Preserve the current
   MID360 topics, noise, calibration and mapping settings, but disable outputs
   not consumed by this product (`path`, registered/map/effect clouds, PCD
   accumulation). `/Odometry` and FAST_LIO TF must remain available.

4. Make `docker/bringup_slam_nav2.sh` default to the absolute release-owned
   low-latency config directory. Preserve a `FASTLIO_CONFIG` environment
   override. It must invoke the preparation helper before starting FAST_LIO,
   and must not silently fall back to the external stock config.

5. Make `tools/livox_stream_watchdog.py` monitor the small `/livox/imu`
   heartbeat with sensor-data QoS instead of deserializing `/livox/lidar`.
   Rename user-facing logs accordingly. Keep the restart policy behavior.

6. Make `map_odom_fuser.py` fail closed on stale raw LIO:
   - add a pure helper for finite message-age validation;
   - declare `max_lio_age_sec`, default `0.35` seconds;
   - compute age from ROS clock now minus the incoming `/Odometry` header;
   - reject negative beyond a small clock-skew allowance and reject age above
     the limit without updating TF, `/odom`, or `/localization_pose`;
   - keep the existing physical jump checks;
   - only re-stamp an accepted fresh pose at now.

7. Add a read-only latency gate tool under `tools/` that subscribes only to
   `/Odometry`, samples a bounded number of messages, and exits nonzero unless
   it receives enough finite, monotonic samples whose median age is <= 0.30 s
   and p95 age is <= 0.35 s. It must never create a publisher. Invoke it in
   bringup after `/Odometry` rate readiness and before map fuser/Nav2 startup.

8. Add focused tests/contracts covering every item above, including:
   - patch contains keep-last 1/best-effort and preserves buffer pairing;
   - helper idempotency/unknown-preimage behavior by exercising a temporary
     fake workspace without compiling;
   - optimized config disables expensive outputs;
   - bringup ordering and absolute release-owned config;
   - watchdog uses IMU + sensor QoS and not CustomMsg;
   - fuser age helper boundary cases and callback rejection contract;
   - latency gate statistics, insufficient samples, nonfinite and nonmonotonic
     timestamps, and source-level no-publisher contract.

Update artifact packaging/verifiers if new runtime files need to be present in
the release. Do not weaken existing safety, release, or architecture tests.

## Acceptance for your task

- Show the focused RED test command and its expected failures before editing
  production files.
- Show the focused GREEN commands and results.
- Run the existing relevant deployment/fuser/watchdog contract suites.
- Return a concise list of changed files, decisions, tests, and any unresolved
  concern. Do not deploy or claim real-world success.
