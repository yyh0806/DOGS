#!/usr/bin/env bash
# Build a content-addressed, subsystem-scoped NX release artifact.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SUBSYSTEM="${1:-all}"
OUTPUT="${2:-}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! "$PYTHON_BIN" --version >/dev/null 2>&1; then
  PYTHON_BIN=python
fi

case "$SUBSYSTEM" in
  motion) REQUIRED_SERVICES="go2w-sport-gateway.service,go2w-safety-observer.service,go2w-motion.service" ;;
  web) REQUIRED_SERVICES="go2w-web.service" ;;
  nav) REQUIRED_SERVICES="go2w-slam-nav.service" ;;
  sensor) REQUIRED_SERVICES="go2w-sensor.service" ;;
  all) REQUIRED_SERVICES="go2w-sport-gateway.service,go2w-safety-observer.service,go2w-motion.service,go2w-web.service,go2w-slam-nav.service,go2w-sensor.service" ;;
  *) echo "usage: $0 {motion|web|nav|sensor|all} [artifact.tar.gz]" >&2; exit 2 ;;
esac

release_id="${GO2W_RELEASE_ID:-$(git -C "$ROOT" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)}"
if [ -n "$(git -C "$ROOT" status --porcelain 2>/dev/null)" ]; then
  release_id="${release_id}-dirty"
fi
release_id="$(printf '%s' "$release_id" | tr -c 'A-Za-z0-9._-' '-')"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/payload"

copy_path() {
  local relative="$1"
  [ -e "$ROOT/$relative" ] || { echo "missing release input: $relative" >&2; exit 1; }
  mkdir -p "$TMP/payload/$(dirname "$relative")"
  cp -a "$ROOT/$relative" "$TMP/payload/$relative"
}

copy_motion_runtime() {
  local name
  for name in build_info.py motion_types.py motion_machine.py motion_protocol.py \
      motion_safety.py motion_controller.py unitree_sport_adapter.py \
      sport_gateway_protocol.py sport_gateway_server.py sport_gateway_client.py \
      safety_event_recorder.py nx_safety_observer.py nx_sport_gateway.py \
      nx_motion_node.py; do
    copy_path "src/go2w_bridge/go2w_bridge/$name"
  done
}

copy_sensor_runtime() {
  local name
  for name in build_info.py motion_types.py motion_machine.py motion_protocol.py nx_sensor_node.py; do
    copy_path "src/go2w_bridge/go2w_bridge/$name"
  done
}

copy_ai_runtime() {
  copy_path "ai/__init__.py"
  copy_path "ai/config.py"
  copy_path "ai/detector.py"
  copy_path "ai/locate_anything.py"
  copy_path "ai/tracker.py"
  copy_path "ai/vlm.py"
}

copy_web_runtime() {
  local file
  for file in "$ROOT"/web/nx_*.py "$ROOT/web/costmap_bridge.py" \
      "$ROOT/web/voice_command.py" "$ROOT/web/start_go2w_web.sh"; do
    copy_path "${file#$ROOT/}"
  done
  copy_path "web/static"
  copy_path "config/rooms.yaml"
  copy_sensor_runtime
}

copy_nav_runtime() {
  local file
  while IFS= read -r file; do
    copy_path "${file#$ROOT/}"
  done < <(find "$ROOT/src/go2w_nav" -type f \
    ! -path '*/__pycache__/*' ! -name '*.pyc' | sort)
  for name in map_odom_fuser.py map_padding_bridge.py mid360_nav_bridge.py; do
    copy_path "src/go2w_bridge/go2w_bridge/$name"
  done
  copy_path "docker/bringup_slam_nav2.sh"
  copy_path "docker/prepare_fastlio_low_latency.sh"
  copy_path "docker/patches/fast_lio_latest_frame.patch"
  copy_path "docker/patches/fast_lio_livox_reliable_qos.patch"
  copy_path "docker/patches/fast_lio_body_cloud.patch"
  copy_path "docker/patches/fast_lio_body_cloud_qos.patch"
  copy_path "docker/patches/fast_lio_bounded_body_cloud.patch"
  copy_path "docker/patches/fast_lio_rotating_body_sample.patch"
  copy_path "docker/patches/fast_lio_angular_body_cloud.patch"
  copy_path "docker/diagnose_nav2_goal.sh"
  copy_path "docker/fastdds_udp.xml"
  copy_path "tools/fastlio_latency_gate.py"
  copy_path "tools/topic_rate_gate.py"
  copy_path "tools/nav_health_supervisor.py"
  copy_path "tools/nav_health_gate.py"
}

copy_validation_tools() {
  copy_path "tools/diag_sport_requests.py"
  copy_path "tools/diag_sport_state.py"
  copy_path "tools/diag_wheel_dq.py"
  copy_path "tools/nav2_preflight.py"
  copy_path "tools/wait_lifecycle_active.py"
  copy_path "tools/nav2_benchmark.py"
  copy_path "tools/capture_map_pose.py"
  copy_path "tools/perception_preflight.py"
  copy_path "tools/sport_gateway_bootstrap_preflight.py"
}

# Every release is a complete immutable runtime.  ``subsystem`` controls only
# which service may be restarted, so switching ``current`` can never expose a
# partial tree to a later process restart.
copy_motion_runtime
copy_web_runtime
copy_nav_runtime
copy_ai_runtime
copy_validation_tools
copy_path "tools/verify_release_artifact.py"
copy_path "tools/nx_release_probe.py"
copy_path "docker/go2w-sport-gateway.service"
copy_path "docker/go2w-safety-observer.service"
copy_path "docker/go2w-motion.service"
copy_path "docker/go2w-web.service"
copy_path "docker/go2w-slam-nav.service"
copy_path "docker/go2w-sensor.service"
copy_path "docker/costmap-bridge.service"
copy_path "docker/livox-mid360-net.service"
copy_path "docker/livox-mid360-driver.service"
copy_path "docker/livox-mid360-watchdog.service"
copy_path "tools/livox_stream_watchdog.py"

RELEASE_ID="$release_id" SUBSYSTEM="$SUBSYSTEM" \
REQUIRED_SERVICES="$REQUIRED_SERVICES" RELEASE_ROOT="$TMP" "$PYTHON_BIN" - <<'PY'
import hashlib
import json
import os
from pathlib import Path

root = Path(os.environ["RELEASE_ROOT"])
payload = root / "payload"
hashes = {}
for path in sorted(item for item in payload.rglob("*") if item.is_file()):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    hashes[path.relative_to(payload).as_posix()] = digest
payload_digest = hashlib.sha256(
    json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode("utf-8")
).hexdigest()
subsystem = os.environ["SUBSYSTEM"]
verification = {
    "motion": "python3 -m py_compile src/go2w_bridge/go2w_bridge/nx_safety_observer.py src/go2w_bridge/go2w_bridge/nx_sport_gateway.py src/go2w_bridge/go2w_bridge/nx_motion_node.py",
    "web": "python3 -m compileall -q ai web",
    "nav": "python3 -m compileall -q src tools && test -f src/go2w_nav/config/nav2_params_3d.yaml",
    "sensor": "python3 -m py_compile src/go2w_bridge/go2w_bridge/nx_sensor_node.py",
    "all": "python3 -m compileall -q ai web src tools",
}[subsystem]
base_release_id = os.environ["RELEASE_ID"]
release_id = f"{base_release_id}-{payload_digest[:12]}"
manifest = {
    "schema_version": 1,
    "release_id": release_id,
    "base_release_id": base_release_id,
    "payload_digest": payload_digest,
    "subsystem": subsystem,
    "required_services": [
        value for value in os.environ["REQUIRED_SERVICES"].split(",") if value
    ],
    "verification_command": verification,
    "sha256": hashes,
}
(root / "manifest.json").write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

release_id="$("$PYTHON_BIN" -c 'import json,sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["release_id"])' "$TMP/manifest.json")"

if [ -z "$OUTPUT" ]; then
  mkdir -p "$ROOT/dist"
  OUTPUT="$ROOT/dist/go2w-${release_id}-${SUBSYSTEM}.tar.gz"
fi
mkdir -p "$(dirname "$OUTPUT")"
tar -C "$TMP" -czf "$OUTPUT" manifest.json payload
echo "$OUTPUT"
