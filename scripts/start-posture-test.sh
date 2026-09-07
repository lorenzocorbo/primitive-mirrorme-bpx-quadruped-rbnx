#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIRMATION="I_UNDERSTAND_POSTURE_RPC_CAN_MOVE_BPX"

[[ -n "${BPX_SDK_WHEEL:-}" ]] || {
  echo "BPX_SDK_WHEEL must point to the pinned BPX SDK v1.0.8 x86_64 wheel" >&2
  exit 1
}
[[ "${BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE:-}" == "$CONFIRMATION" ]] || {
  echo "refusing posture service: set BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE=$CONFIRMATION with a local operator present" >&2
  exit 2
}

export BPX_REQUIRED_BACKEND=sdk
export BPX_POSTURE_CAPABILITY=true
export BPX_REQUIRED_POSTURE_CAPABILITY=true
export RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-robonix.primitive.mirrorme.bpx.quadruped.posture_test}"
exec bash "$SCRIPT_DIR/start.sh"
