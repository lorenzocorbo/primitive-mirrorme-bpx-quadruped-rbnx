#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

[[ -n "${BPX_SDK_WHEEL:-}" ]] || {
  echo "BPX_SDK_WHEEL must point to the pinned BPX SDK v1.0.8 x86_64 wheel" >&2
  exit 1
}

# Constructing MotionLevelControl does not move the robot. The provider keeps
# velocity control disabled and exposes only the explicit stand/sit posture RPC.
export BPX_REQUIRED_BACKEND=sdk
export BPX_POSTURE_CAPABILITY=true
export BPX_REQUIRED_POSTURE_CAPABILITY=true
exec bash "$SCRIPT_DIR/start.sh"
