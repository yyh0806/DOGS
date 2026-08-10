#!/bin/bash
# Safely deploy the MID360-primary FastLIO/Nav2 stack to the NX.
set -euo pipefail

# Compatibility entrypoint only. This preserves the familiar command name
# while guaranteeing Nav2 deployment never overwrites or restarts motion.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "deploy_nav2_bprime.sh is retired; forwarding to the atomic nav release flow" >&2
artifact="$("$SCRIPT_DIR/build_release.sh" nav)"
exec "$SCRIPT_DIR/deploy_release.sh" "$artifact" "$@"
