#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
WHEEL="${1:-}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
shift || true

[[ -n "$WHEEL" && -f "$WHEEL" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl [probe options]" >&2
  exit 2
}

bash "$PKG/scripts/verify-sdk-wheel.sh" "$WHEEL"

wheel_name="$(basename "$WHEEL")"

docker run --rm --network host \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  -v "$PKG:/pkg:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && exec python3 -m bpx_quadruped.read_only_probe "$@"' \
  probe "$@"
