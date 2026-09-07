#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
WHEEL="${1:-}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
VERIFY_MODE="${2:-}"

[[ -n "$WHEEL" && -f "$WHEEL" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl [--posture|--yaw-circle]" >&2
  exit 2
}
[[ -z "$VERIFY_MODE" || "$VERIFY_MODE" == "--posture" || "$VERIFY_MODE" == "--yaw-circle" ]] || {
  echo "optional second argument must be --posture or --yaw-circle" >&2
  exit 2
}

expected="$(awk '!/^#/ && NF == 2 {print $1; exit}' "$PKG/sdk/SHA256SUMS")"
actual="$(sha256sum "$WHEEL" | awk '{print $1}')"
[[ "$actual" == "$expected" ]] || {
  echo "BPX SDK wheel checksum mismatch" >&2
  echo "expected: $expected" >&2
  echo "actual:   $actual" >&2
  exit 1
}

wheel_name="$(basename "$WHEEL")"

docker run --rm --network none \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -e BPX_SDK_VERIFY_MODE="$VERIFY_MODE" \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && python3 -m bpx_quadruped.verify_sdk_api $BPX_SDK_VERIFY_MODE'
