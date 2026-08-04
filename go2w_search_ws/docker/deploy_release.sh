#!/usr/bin/env bash
# Verify, install, switch, health-check, and (on failure) roll back one NX release.
set -euo pipefail

ARTIFACT="${1:-}"
shift || true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! "$PYTHON_BIN" --version >/dev/null 2>&1; then
  PYTHON_BIN=python
fi
ALLOW_MOTION_RESTART=0
BOOTSTRAP_SPORT_GATEWAY=0
CONTROL_TOKEN_FILE=""
GENERATE_CONTROL_TOKEN_FILE=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --allow-motion-restart) ALLOW_MOTION_RESTART=1 ;;
    --bootstrap-sport-gateway) BOOTSTRAP_SPORT_GATEWAY=1 ;;
    --control-token-file)
      shift
      CONTROL_TOKEN_FILE="${1:-}"
      [ -n "$CONTROL_TOKEN_FILE" ] || {
        echo "--control-token-file requires a path" >&2
        exit 2
      }
      ;;
    --generate-control-token-file)
      shift
      GENERATE_CONTROL_TOKEN_FILE="${1:-}"
      [ -n "$GENERATE_CONTROL_TOKEN_FILE" ] || {
        echo "--generate-control-token-file requires a new path" >&2
        exit 2
      }
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -f "$ARTIFACT" ] || {
  echo "usage: $0 artifact.tar.gz [--allow-motion-restart] [--bootstrap-sport-gateway] [--control-token-file PATH | --generate-control-token-file NEW_PATH]" >&2
  exit 2
}
[ -z "$CONTROL_TOKEN_FILE" ] || [ -z "$GENERATE_CONTROL_TOKEN_FILE" ] || {
  echo "--control-token-file and --generate-control-token-file are mutually exclusive" >&2
  exit 2
}
"$PYTHON_BIN" "$SCRIPT_DIR/../tools/verify_release_artifact.py" \
  "$ARTIFACT" --quiet
if [ -n "$GENERATE_CONTROL_TOKEN_FILE" ]; then
  "$PYTHON_BIN" "$SCRIPT_DIR/../tools/generate_control_token.py" \
    "$GENERATE_CONTROL_TOKEN_FILE"
  CONTROL_TOKEN_FILE="$GENERATE_CONTROL_TOKEN_FILE"
fi
if [ -n "$CONTROL_TOKEN_FILE" ] && [ ! -f "$CONTROL_TOKEN_FILE" ]; then
  echo "--control-token-file requires a readable token file" >&2
  exit 2
fi

NX_HOST="${NX_HOST:-192.168.43.41}"
NX_USER="${NX_USER:-nx}"
NX_BIND_ADDRESS="${NX_BIND_ADDRESS:-}"
LIVOX_INTERFACE_OVERRIDE="${LIVOX_INTERFACE:-}"
case "$NX_HOST" in
  ""|*[!A-Za-z0-9.-]*)
    echo "NX_HOST must be an IPv4 address or DNS hostname" >&2
    exit 2
    ;;
esac
case "$NX_BIND_ADDRESS" in
  "") ;;
  *[!0-9.]*|.*|*.)
    echo "NX_BIND_ADDRESS must be a local IPv4 address" >&2
    exit 2
    ;;
esac
case "$LIVOX_INTERFACE_OVERRIDE" in
  "") ;;
  *[!A-Za-z0-9_.:-]*)
    echo "LIVOX_INTERFACE must be a Linux network interface name" >&2
    exit 2
    ;;
esac
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
SCP=(scp -q -o BatchMode=yes -o ConnectTimeout=8)
if [ -n "$NX_BIND_ADDRESS" ]; then
  SSH+=(-b "$NX_BIND_ADDRESS")
  SCP+=(-o "BindAddress=$NX_BIND_ADDRESS")
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
tar -xOf "$ARTIFACT" manifest.json > "$TMP/manifest.json"

read_manifest() {
  "$PYTHON_BIN" - "$TMP/manifest.json" "$1" <<'PY'
import json, sys
value = json.load(open(sys.argv[1], encoding="utf-8"))[sys.argv[2]]
print(",".join(value) if isinstance(value, list) else value)
PY
}

release_id="$(read_manifest release_id)"
subsystem="$(read_manifest subsystem)"
case "$subsystem" in
  motion) services="go2w-safety-observer.service go2w-motion.service" ;;
  web) services="go2w-web.service" ;;
  nav) services="go2w-sensor.service go2w-slam-nav.service" ;;
  sensor) services="go2w-sensor.service" ;;
  # The bounded sensor keeps wheel feedback alive; Nav remains the sole owner
  # of /odom and the odom->base_link transform.
  all) services="go2w-safety-observer.service go2w-motion.service go2w-sensor.service go2w-web.service go2w-slam-nav.service" ;;
  *) echo "invalid subsystem in manifest" >&2; exit 1 ;;
esac
if { [ "$subsystem" = "motion" ] || [ "$subsystem" = "all" ]; } \
    && [ "$ALLOW_MOTION_RESTART" -ne 1 ]; then
  echo "motion_restart_not_authorized: pass --allow-motion-restart only when the dog is powered, stable, and the area is safe" >&2
  exit 3
fi
if [ "$BOOTSTRAP_SPORT_GATEWAY" -eq 1 ] && {
    [ "$ALLOW_MOTION_RESTART" -ne 1 ] || {
      [ "$subsystem" != "motion" ] && [ "$subsystem" != "all" ];
    };
}; then
  echo "gateway_bootstrap_requires_motion_restart_authorization" >&2
  exit 3
fi

remote_preflight() {
  local artifact_bytes required_kb
  artifact_bytes="$(wc -c < "$ARTIFACT")"
  # Reserve 1 GiB for extraction/colcon plus several copies of the archive.
  required_kb=$((artifact_bytes / 1024 * 6 + 1048576))
  "${SSH[@]}" "$NX_USER@$NX_HOST" \
      bash -s -- "$subsystem" "$required_kb" "$control_token_supplied" \
      "$BOOTSTRAP_SPORT_GATEWAY" "$LIVOX_INTERFACE_OVERRIDE" <<'PREFLIGHT'
set -euo pipefail
subsystem="$1"
required_kb="$2"
control_token_supplied="$3"
bootstrap_sport_gateway="${4:-0}"
livox_interface_override="${5:-}"
[ "$(id -un)" = "nx" ] || {
  echo "NX preflight failed: deployment services require user nx" >&2
  exit 1
}
for command in python3 tar systemctl sudo df ping timeout ip; do
  command -v "$command" >/dev/null || {
    echo "NX preflight failed: missing $command" >&2
    exit 1
  }
done
sudo -n true || {
  echo "NX preflight failed: passwordless sudo is required" >&2
  exit 1
}
if { [ "$subsystem" = "web" ] || [ "$subsystem" = "all" ]; } \
    && [ "$control_token_supplied" -ne 1 ]; then
  existing_control_token="$(sudo -n sed -n \
    's/^GO2W_CONTROL_TOKEN=//p' /etc/go2w/control.env 2>/dev/null \
    | tail -n 1 || true)"
  case "$existing_control_token" in
    *[!A-Za-z0-9._~-]*) existing_control_token="" ;;
  esac
  [ "${#existing_control_token}" -ge 32 ] || {
    echo "NX preflight failed: existing control token is missing or weak; supply a new token file" >&2
    exit 1
  }
fi
[ -r /opt/ros/humble/setup.bash ] || {
  echo "NX preflight failed: ROS 2 Humble is missing" >&2
  exit 1
}
python3 -c 'import unitree_sdk2py' || {
  echo "NX preflight failed: unitree_sdk2py is not importable" >&2
  exit 1
}
python3 -c 'import numpy, cv2, torch, ultralytics' || {
  echo "NX preflight failed: numpy/OpenCV/PyTorch/ultralytics runtime is incomplete" >&2
  exit 1
}
yolo_model_path=""
for candidate in \
    /home/nx/models/yolov8x-worldv2.pt \
    /home/nx/go2w_ws/models/yolov8x-worldv2.pt; do
  if [ -r "$candidate" ]; then
    yolo_model_path="$candidate"
    break
  fi
done
[ -n "$yolo_model_path" ] || {
  echo "NX preflight failed: YOLO-World model yolov8x-worldv2.pt is missing" >&2
  exit 1
}
available_kb="$(df -Pk /home/nx | awk 'NR==2 {print $4}')"
[ -n "$available_kb" ] && [ "$available_kb" -ge "$required_kb" ] || {
  echo "NX preflight failed: insufficient disk space" >&2
  exit 1
}
route_line="$(ip route get 192.168.123.161 2>/dev/null | head -n 1)"
dog_interface="$(printf '%s\n' "$route_line" | awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}')"
dog_source_ip="$(printf '%s\n' "$route_line" | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')"
case "$dog_source_ip" in
  192.168.123.*) ;;
  *)
    echo "NX preflight failed: no dedicated 192.168.123.x route to the Go2W controller" >&2
    exit 1
    ;;
esac
[ -n "$dog_interface" ] && [ -e "/sys/class/net/$dog_interface" ] || {
  echo "NX preflight failed: could not resolve the Go2W network interface" >&2
  exit 1
}
livox_route_line="$(ip route get 192.168.1.160 2>/dev/null | head -n 1 || true)"
livox_route_interface="$(printf '%s\n' "$livox_route_line" | awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}')"
livox_source_ip="$(printf '%s\n' "$livox_route_line" | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')"
livox_interface=""
if [ "$livox_source_ip" = "192.168.1.200" ] \
    && [ -e "/sys/class/net/$livox_route_interface" ]; then
  livox_interface="$livox_route_interface"
elif [ -n "$livox_interface_override" ] \
    && [ -e "/sys/class/net/$livox_interface_override" ]; then
  livox_interface="$livox_interface_override"
else
  existing_livox="$(sudo -n sed -n 's/^LIVOX_INTERFACE=//p' \
    /etc/go2w/hardware.env 2>/dev/null | tail -n 1 || true)"
  if [ -n "$existing_livox" ] \
      && [ -e "/sys/class/net/$existing_livox" ]; then
    livox_interface="$existing_livox"
  elif [ -e /sys/class/net/enx207bd2edf780 ]; then
    livox_interface="enx207bd2edf780"
  fi
fi
if [ "$subsystem" = "nav" ] || [ "$subsystem" = "all" ]; then
  [ -n "$livox_interface" ] || {
    echo "NX preflight failed: MID360 interface not found; set LIVOX_INTERFACE" >&2
    exit 1
  }
fi
[ -n "$livox_interface" ] || livox_interface="enx207bd2edf780"
if [ "$subsystem" = "motion" ] || [ "$subsystem" = "all" ]; then
  ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1 || {
    echo "NX preflight failed: Go2W controller 192.168.123.161 is unreachable" >&2
    exit 1
  }
  gateway_was_active="$(sudo -n systemctl is-active \
    go2w-sport-gateway.service 2>/dev/null || true)"
  if [ "$gateway_was_active" = "active" ]; then
    test -S /run/go2w-sport-gateway/sport.sock || {
      echo "NX preflight failed: active Sport gateway has no socket" >&2
      exit 1
    }
  elif [ "$bootstrap_sport_gateway" -ne 1 ]; then
    echo "gateway_bootstrap_required: use a separate supported maintenance window" >&2
    exit 1
  fi
fi
if [ "$subsystem" = "nav" ] || [ "$subsystem" = "all" ]; then
  command -v colcon >/dev/null || {
    echo "NX preflight failed: colcon is missing" >&2
    exit 1
  }
  [ -r /home/nx/ws_livox/install/setup.bash ] || {
    echo "NX preflight failed: Livox workspace is missing" >&2
    exit 1
  }
fi
printf 'GO2W_DOG_INTERFACE=%s\n' "$dog_interface"
printf 'GO2W_YOLO_MODEL_PATH=%s\n' "$yolo_model_path"
printf 'GO2W_LIVOX_INTERFACE=%s\n' "$livox_interface"
echo "NX preflight passed: subsystem=$subsystem available_kb=$available_kb dog_interface=$dog_interface"
PREFLIGHT
}

remote_artifact="/tmp/go2w-release-${release_id}.tar.gz"
remote_control_env=""
remote_hardware_env="/tmp/go2w-hardware-${release_id}.env"
control_token_supplied=0
if [ -n "$CONTROL_TOKEN_FILE" ]; then
  token="$(sed 's/\r$//' "$CONTROL_TOKEN_FILE")"
  case "$token" in
    ""|*[!A-Za-z0-9._~-]*)
      echo "control token must be one non-empty URL-safe line" >&2
      exit 2
      ;;
  esac
  [ "${#token}" -ge 32 ] || {
    echo "control token must contain at least 32 URL-safe characters" >&2
    exit 2
  }
  printf 'GO2W_CONTROL_TOKEN=%s\n' "$token" > "$TMP/control.env"
  remote_control_env="/tmp/go2w-control-${release_id}.env"
  control_token_supplied=1
fi
preflight_output="$(remote_preflight)"
printf '%s\n' "$preflight_output"
dog_interface="$(printf '%s\n' "$preflight_output" | sed -n 's/^GO2W_DOG_INTERFACE=//p' | tail -n 1)"
yolo_model_path="$(printf '%s\n' "$preflight_output" | sed -n 's/^GO2W_YOLO_MODEL_PATH=//p' | tail -n 1)"
livox_interface="$(printf '%s\n' "$preflight_output" | sed -n 's/^GO2W_LIVOX_INTERFACE=//p' | tail -n 1)"
case "$dog_interface" in
  ""|*[!A-Za-z0-9_.:-]*)
    echo "NX preflight returned an invalid dog interface" >&2
    exit 1
    ;;
esac
case "$yolo_model_path" in
  /home/nx/*) ;;
  *)
    echo "NX preflight returned an invalid YOLO model path" >&2
    exit 1
    ;;
esac
case "$livox_interface" in
  ""|*[!A-Za-z0-9_.:-]*)
    echo "NX preflight returned an invalid MID360 interface" >&2
    exit 1
    ;;
esac
panel_origins="http://${NX_HOST}:8000,http://127.0.0.1:8000,http://localhost:8000"
printf 'DOG_INTERFACE=%s\nLIVOX_INTERFACE=%s\nGO2W_PUBLIC_IP=%s\nGO2W_PANEL_ORIGINS=%s\nGO2W_YOLO_MODEL=%s\n' \
  "$dog_interface" "$livox_interface" "$NX_HOST" "$panel_origins" \
  "$yolo_model_path" \
  > "$TMP/hardware.env"
"${SCP[@]}" "$ARTIFACT" "$NX_USER@$NX_HOST:$remote_artifact"
"${SCP[@]}" "$TMP/hardware.env" "$NX_USER@$NX_HOST:$remote_hardware_env"
"${SSH[@]}" "$NX_USER@$NX_HOST" "chmod 644 '$remote_hardware_env'"
if [ -n "$remote_control_env" ]; then
  "${SCP[@]}" "$TMP/control.env" "$NX_USER@$NX_HOST:$remote_control_env"
  "${SSH[@]}" "$NX_USER@$NX_HOST" "chmod 600 '$remote_control_env'"
fi

"${SSH[@]}" "$NX_USER@$NX_HOST" bash -s -- \
    "$remote_artifact" "$release_id" "$subsystem" \
    "$BOOTSTRAP_SPORT_GATEWAY" "$remote_hardware_env" \
    "$remote_control_env" <<'REMOTE'
set -euo pipefail
artifact="$1"
release_id="$2"
subsystem="$3"
bootstrap_sport_gateway="${4:-0}"
hardware_env="$5"
control_env="${6:-}"
case "$subsystem" in
  motion) services="go2w-safety-observer.service go2w-motion.service" ;;
  web) services="go2w-web.service" ;;
  nav) services="go2w-sensor.service go2w-slam-nav.service" ;;
  sensor) services="go2w-sensor.service" ;;
  all) services="go2w-safety-observer.service go2w-motion.service go2w-sensor.service go2w-web.service go2w-slam-nav.service" ;;
  *) echo "invalid subsystem in remote transaction" >&2; exit 1 ;;
esac
base="/home/nx/go2w"
releases="/home/nx/go2w/releases"
release_dir="$releases/$release_id"
staging="$releases/.staging-$release_id-$$"
current="$base/current"
gateway_service="go2w-sport-gateway.service"
gateway_socket="/run/go2w-sport-gateway/sport.sock"
previous_target=""
switched=0
created_release=0
system_backup=""
managed_units="go2w-sport-gateway.service go2w-safety-observer.service go2w-motion.service go2w-web.service go2w-slam-nav.service go2w-sensor.service costmap-bridge.service livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service"
active_state_units="go2w-safety-observer.service go2w-motion.service go2w-web.service go2w-slam-nav.service go2w-sensor.service costmap-bridge.service livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service"
restore_start_order="livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service go2w-safety-observer.service go2w-motion.service go2w-sensor.service go2w-web.service go2w-slam-nav.service costmap-bridge.service"
rollback_units="$services"
restart_units="$services"
enable_services="$services"
enable_state_units="$services"
gateway_bootstrapped=0

# ROS setup scripts intentionally inspect optional variables.  Keep the
# deploy transaction strict, but scope nounset off only while sourcing ROS.
source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  # Source only the Livox package overlay needed by the compiled bridge.
  # The workspace-wide setup records historical underlays (including the
  # mutable ``current`` release) and can fail while a web-only release has no
  # colcon install tree yet.
  if [ -r /home/nx/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash ]; then
    source /home/nx/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
  fi
  set -u
  export ROS2CLI_NO_DAEMON=1
}

if [ "$subsystem" = "nav" ] || [ "$subsystem" = "all" ]; then
  rollback_units="go2w-slam-nav.service costmap-bridge.service livox-mid360-watchdog.service livox-mid360-driver.service livox-mid360-net.service $services"
fi
if [ "$subsystem" = "nav" ]; then
  managed_units="go2w-sensor.service go2w-slam-nav.service costmap-bridge.service livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service"
  active_state_units="go2w-slam-nav.service costmap-bridge.service livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service go2w-sensor.service"
  restart_units="livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service go2w-sensor.service go2w-slam-nav.service"
  enable_state_units="$services"
fi
if [ "$subsystem" = "web" ]; then
  managed_units="go2w-web.service costmap-bridge.service"
  active_state_units="go2w-web.service costmap-bridge.service"
  restore_start_order="$active_state_units"
fi
if [ "$subsystem" = "sensor" ]; then
  managed_units="go2w-sensor.service"
  active_state_units="go2w-sensor.service"
  restore_start_order="$active_state_units"
fi
if [ "$subsystem" = "all" ]; then
  restart_units="go2w-safety-observer.service go2w-motion.service go2w-sensor.service go2w-web.service livox-mid360-net.service livox-mid360-driver.service livox-mid360-watchdog.service go2w-slam-nav.service"
  enable_services="go2w-sport-gateway.service go2w-safety-observer.service go2w-motion.service go2w-web.service go2w-sensor.service go2w-slam-nav.service"
  enable_state_units="go2w-sport-gateway.service $services"
fi
if [ "$subsystem" = "motion" ]; then
  managed_units="go2w-sport-gateway.service go2w-safety-observer.service go2w-motion.service"
  active_state_units="go2w-safety-observer.service go2w-motion.service"
  restore_start_order="$active_state_units"
  enable_services="go2w-sport-gateway.service go2w-safety-observer.service go2w-motion.service"
  enable_state_units="$enable_services"
fi
mkdir -p "$releases"
if [ -L "$current" ]; then
  previous_target="$(readlink -f "$current" || true)"
fi

backup_system_file() {
  local path="$1"
  local name="$2"
  if sudo -n test -e "$path"; then
    sudo -n cp -a -- "$path" "$system_backup/$name"
  else
    sudo -n touch "$system_backup/.missing-$name"
  fi
}

backup_enable_state() {
  local unit state
  for unit in $enable_state_units; do
    state="$(sudo -n systemctl is-enabled "$unit" 2>/dev/null || true)"
    [ -n "$state" ] || state="disabled"
    printf '%s\n' "$state" | sudo -n tee \
      "$system_backup/enabled-$unit" >/dev/null
  done
}

backup_active_state() {
  local unit state
  for unit in $active_state_units; do
    state="$(sudo -n systemctl is-active "$unit" 2>/dev/null || true)"
    [ -n "$state" ] || state="inactive"
    printf '%s\n' "$state" | sudo -n tee \
      "$system_backup/active-$unit" >/dev/null
  done
}

backup_system_state() {
  system_backup="$(sudo -n mktemp -d /var/tmp/go2w-deploy-backup.XXXXXX)"
  sudo -n chmod 0700 "$system_backup"
  backup_system_file /etc/go2w/release.env release.env
  backup_system_file /etc/go2w/hardware.env hardware.env
  backup_system_file /etc/go2w/control.env control.env
  for unit in $managed_units; do
    backup_system_file "/etc/systemd/system/$unit" "$unit"
  done
  backup_enable_state
  backup_active_state
}

restore_system_file() {
  local path="$1"
  local name="$2"
  if sudo -n test -e "$system_backup/$name"; then
    sudo -n rm -f -- "$path"
    sudo -n cp -a -- "$system_backup/$name" "$path"
  else
    sudo -n rm -f -- "$path"
  fi
}

restore_system_state() {
  [ -n "$system_backup" ] || return 1
  restore_system_file /etc/go2w/release.env release.env
  restore_system_file /etc/go2w/hardware.env hardware.env
  restore_system_file /etc/go2w/control.env control.env
  for unit in $managed_units; do
    restore_system_file "/etc/systemd/system/$unit" "$unit"
  done
}

restore_enable_state() {
  local unit state
  for unit in $enable_state_units; do
    state="$(sudo -n cat "$system_backup/enabled-$unit" 2>/dev/null \
      || printf 'disabled')"
    case "$state" in
      enabled|linked|alias|indirect)
        sudo -n systemctl enable "$unit" >/dev/null 2>&1 || true
        ;;
      enabled-runtime|linked-runtime)
        sudo -n systemctl enable --runtime "$unit" >/dev/null 2>&1 || true
        ;;
      masked)
        sudo -n systemctl mask "$unit" >/dev/null 2>&1 || true
        ;;
      masked-runtime)
        sudo -n systemctl mask --runtime "$unit" >/dev/null 2>&1 || true
        ;;
      *)
        sudo -n systemctl disable "$unit" >/dev/null 2>&1 || true
        ;;
    esac
  done
}

restore_active_state() {
  local unit state
  # First remove units that were not running before the transaction, then
  # restore prior active units in dependency-safe order.
  for unit in $active_state_units; do
    state="$(sudo -n cat "$system_backup/active-$unit" 2>/dev/null \
      || printf 'inactive')"
    case "$state" in
      active|activating|reloading) ;;
      *) sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true ;;
    esac
  done
  for unit in $restore_start_order; do
    state="$(sudo -n cat "$system_backup/active-$unit" 2>/dev/null \
      || printf 'inactive')"
    case "$state" in
      active|activating|reloading)
        sudo -n systemctl restart "$unit" || true
        ;;
    esac
  done
}

rollback() {
  local code="$?"
  if [ "$gateway_bootstrapped" -eq 1 ]; then
    for unit in $rollback_units; do
      sudo -n systemctl stop "$unit" || true
    done
    rm -rf "$staging"
    rm -f "$artifact" "$control_env" "$hardware_env"
    if [ -n "$system_backup" ]; then
      sudo -n rm -rf "$system_backup" || true
    fi
    echo "gateway_bootstrap_failed_zero_hold release=$release_id" >&2
    exit "$code"
  fi
  if [ "$switched" -eq 1 ]; then
    # Stop every process that may have loaded code or configuration from the
    # failed release before changing ``current`` or restoring its units.
    for unit in $rollback_units; do
      sudo -n systemctl stop "$unit" || true
    done
    if [ -n "$previous_target" ] && [ -d "$previous_target" ]; then
      ln -sfn "$previous_target" "$current.rollback"
      mv -Tf "$current.rollback" "$current"
    else
      # First atomic install has no known-good release to restore.
      rm -f "$current"
    fi
    restore_system_state || true
    sudo -n systemctl daemon-reload || true
    restore_enable_state || true
    restore_active_state || true
  fi
  if [ "$switched" -eq 0 ] && [ "$created_release" -eq 1 ]; then
    rm -rf "$release_dir"
  fi
  rm -rf "$staging"
  rm -f "$artifact" "$control_env" "$hardware_env"
  if [ -n "$system_backup" ]; then
    sudo -n rm -rf "$system_backup" || true
  fi
  echo "rollback release=$release_id subsystem=$subsystem" >&2
  exit "$code"
}
trap rollback ERR

rm -rf "$staging"
mkdir -p "$staging"
ARTIFACT="$artifact" STAGING="$staging" python3 - <<'PY'
import os
import tarfile
from pathlib import Path

destination = Path(os.environ["STAGING"]).resolve()
with tarfile.open(os.environ["ARTIFACT"], "r:gz") as archive:
    for member in archive.getmembers():
        candidate = (destination / member.name).resolve()
        if destination not in candidate.parents and candidate != destination:
            raise SystemExit("unsafe artifact path")
        if not (member.isfile() or member.isdir()):
            raise SystemExit(f"unsupported archive member: {member.name}")
    archive.extractall(destination)
PY

python3 "$staging/payload/tools/verify_release_artifact.py" \
  "$artifact" --quiet

# sha256 verification happens before the atomic current-link switch.
STAGING="$staging" python3 - <<'PY'
import hashlib
import json
import os
from pathlib import Path

root = Path(os.environ["STAGING"])
manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
payload = root / "payload"
for relative, expected in manifest["sha256"].items():
    path = payload / relative
    if not path.is_file():
        raise SystemExit(f"missing payload file: {relative}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f"sha256 mismatch: {relative}")
PY

verification_command="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["verification_command"])' "$staging/manifest.json")"
(cd "$staging/payload" && bash -c "$verification_command")
if [ -d "$release_dir" ]; then
  EXISTING="$release_dir/manifest.json" INCOMING="$staging/manifest.json" python3 - <<'PY'
import hashlib
import json
import os
from pathlib import Path

existing = json.load(open(os.environ["EXISTING"], encoding="utf-8"))
incoming = json.load(open(os.environ["INCOMING"], encoding="utf-8"))
if existing.get("payload_digest") != incoming.get("payload_digest"):
    raise SystemExit("release_id collision with different payload")
payload = Path(os.environ["EXISTING"]).parent / "payload"
for relative, expected in incoming["sha256"].items():
    path = payload / relative
    if not path.is_file():
        raise SystemExit(f"existing release payload mismatch: missing {relative}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f"existing release payload mismatch: {relative}")
PY
  rm -rf "$staging"
else
  mv "$staging" "$release_dir"
  created_release=1
fi
if [ "$subsystem" = "nav" ] || [ "$subsystem" = "all" ]; then
  if [ ! -f "$release_dir/.go2w-colcon-ready" ]; then
    rm -rf "$release_dir/payload/build" \
      "$release_dir/payload/install" "$release_dir/payload/log"
    (cd "$release_dir/payload" && \
      source_ros && \
      colcon build --packages-select go2w_nav --symlink-install)
    test -f "$release_dir/payload/install/setup.bash"
    touch "$release_dir/.go2w-colcon-ready"
  fi
fi
if { [ "$subsystem" = "motion" ] || [ "$subsystem" = "all" ]; } \
    && [ "$bootstrap_sport_gateway" -eq 1 ] \
    && ! sudo -n systemctl is-active --quiet "$gateway_service"; then
  set -a
  source "$hardware_env"
  set +a
  if ! timeout 8 python3 \
      "$release_dir/payload/tools/sport_gateway_bootstrap_preflight.py" \
      --timeout 5 --samples 3; then
    echo "gateway_bootstrap_state_not_safe" >&2
    false
  fi
fi
backup_system_state
ln -sfn "$release_dir" "$current.next"
mv -Tf "$current.next" "$current"
switched=1

sudo -n mkdir -p /etc/go2w
# A partial release switches the complete immutable tree but deliberately
# preserves the live process cohort id. Motion/Web keep reporting the id they
# loaded at process start, so rewriting it here would quarantine navigation or
# tempt the deployer to restart motion merely to regain consistency. The next
# explicit all-system deployment advances the cohort id atomically.
if [ "$subsystem" = "all" ] || ! sudo -n test -s /etc/go2w/release.env; then
  printf 'GO2W_RELEASE_ID=%s\n' "$release_id" | sudo -n tee /etc/go2w/release.env >/dev/null
  sudo -n chmod 600 /etc/go2w/release.env
fi
[ -n "$hardware_env" ] && [ -f "$hardware_env" ] || {
  echo "detected hardware environment is missing" >&2
  false
}
sudo -n install -o root -g root -m 0644 "$hardware_env" /etc/go2w/hardware.env
rm -f "$hardware_env"
if [ -n "$control_env" ]; then
  sudo -n install -o root -g root -m 0600 "$control_env" /etc/go2w/control.env
  rm -f "$control_env"
fi
for unit in $managed_units; do
  if [ -f "$current/payload/docker/$unit" ]; then
    sudo -n install -m 0644 "$current/payload/docker/$unit" "/etc/systemd/system/$unit"
  fi
done
sudo -n systemctl daemon-reload
gateway_was_active="$(sudo -n systemctl is-active \
  "$gateway_service" 2>/dev/null || true)"
if [ "$subsystem" = "motion" ] || [ "$subsystem" = "all" ]; then
  if [ "$gateway_was_active" != "active" ]; then
    [ "$bootstrap_sport_gateway" -eq 1 ] || {
      echo "gateway_bootstrap_required" >&2
      false
    }
    if ! timeout 8 python3 \
        "$current/payload/tools/sport_gateway_bootstrap_preflight.py" \
        --timeout 5 --samples 3; then
      echo "gateway_bootstrap_state_not_safe" >&2
      false
    fi
    sudo -n systemctl stop go2w-motion.service
    sudo -n systemctl enable "$gateway_service" >/dev/null
    sudo -n systemctl start "$gateway_service"
    for _ in $(seq 1 100); do
      [ -S "$gateway_socket" ] && break
      sleep 0.05
    done
    [ -S "$gateway_socket" ] || {
      echo "Sport gateway socket did not become ready" >&2
      false
    }
    gateway_bootstrapped=1
  fi
  sudo -n systemctl is-active --quiet "$gateway_service"
  test -S /run/go2w-sport-gateway/sport.sock
fi
for service in $restart_units; do
  sudo -n systemctl restart "$service"
  sudo -n systemctl is-active --quiet "$service"
done
if [ "$subsystem" = "all" ]; then
  mkdir -p "$base/validation"
  source_ros
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  export FASTRTPS_DEFAULT_PROFILES_FILE="$current/payload/docker/fastdds_udp.xml"
  timeout 45 python3 "$current/payload/tools/nx_release_probe.py" \
    --expected "$release_id" \
    --timeout 35 \
    --require-sdk-ready \
    --output "$base/validation/${release_id}-deploy.json"
  nav_preflight_ok=0
  for nav_attempt in 1 2 3; do
    if timeout 30 python3 "$current/payload/tools/nav2_preflight.py" \
        --timeout 6 \
        --json \
        --output "$base/validation/${release_id}-nav2-preflight.json"; then
      nav_preflight_ok=1
      break
    fi
    echo "Nav2 preflight attempt $nav_attempt failed; waiting for stack convergence" >&2
    sleep 3
  done
  [ "$nav_preflight_ok" -eq 1 ] || {
    echo "Nav2 preflight did not converge after three attempts" >&2
    false
  }
  timeout 150 python3 "$current/payload/tools/perception_preflight.py" \
    --expected "$release_id" \
    --wait 120 \
    --output "$base/validation/${release_id}-perception-preflight.json"
fi

sudo -n systemctl enable $enable_services >/dev/null
sudo -n rm -rf "$system_backup"
system_backup=""
trap - ERR
rm -f "$artifact" "$control_env" "$hardware_env"
echo "deployed release=$release_id subsystem=$subsystem services=$services restart_units=$restart_units"
REMOTE
