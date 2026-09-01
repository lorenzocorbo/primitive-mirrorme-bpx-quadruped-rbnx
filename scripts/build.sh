#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx is required for Robonix code generation; use scripts/build-container.sh for the offline image" >&2
  exit 1
}

CODEGEN_FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && CODEGEN_FLAGS+=(--clean)
rbnx codegen -p "$PKG" --ros2 "${CODEGEN_FLAGS[@]}"
bash scripts/build-container.sh
