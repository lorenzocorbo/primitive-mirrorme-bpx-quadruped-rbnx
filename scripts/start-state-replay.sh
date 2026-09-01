#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export BPX_REQUIRED_BACKEND=replay
exec bash "$SCRIPT_DIR/start.sh"
