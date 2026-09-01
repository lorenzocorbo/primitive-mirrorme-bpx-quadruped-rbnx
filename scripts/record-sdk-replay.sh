#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
WHEEL="${1:-}"
OUTPUT="${2:-}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
[[ $# -ge 2 ]] && shift 2 || true

[[ -n "$WHEEL" && -f "$WHEEL" && -n "$OUTPUT" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-...x86_64.whl /path/to/output.jsonl [record options]" >&2
  exit 2
}
[[ ! -e "$OUTPUT" ]] || {
  echo "refusing to overwrite existing recording: $OUTPUT" >&2
  exit 1
}

bash "$PKG/scripts/verify-sdk-wheel.sh" "$WHEEL"

wheel_name="$(basename "$WHEEL")"
output_dir="$(realpath "$(dirname "$OUTPUT")")"
output_name="$(basename "$OUTPUT")"

docker run --rm --network host \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  -v "$PKG:/pkg:ro" \
  -v "$output_dir:/capture" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && exec python3 -m bpx_quadruped.recording "/capture/$1" "${@:2}"' \
  record "$output_name" "$@"

PYTHONPATH="$PKG${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m bpx_quadruped.replay_audit "$OUTPUT"
