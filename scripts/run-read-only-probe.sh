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
robot_state_port=9873
probe_args=("$@")
for ((index = 0; index < ${#probe_args[@]}; index++)); do
  case "${probe_args[$index]}" in
    --robot-state-port)
      ((index + 1 < ${#probe_args[@]})) || {
        echo "--robot-state-port requires a value" >&2
        exit 2
      }
      robot_state_port="${probe_args[$((index + 1))]}"
      ;;
    --robot-state-port=*)
      robot_state_port="${probe_args[$index]#*=}"
      ;;
  esac
done

docker_network_args=()
case "${BPX_DOCKER_NETWORK_MODE:-host}" in
  host)
    docker_network_args=(--network host)
    ;;
  bridge)
    docker_network_args=(
      --network bridge
      --publish "$robot_state_port:$robot_state_port/udp"
    )
    ;;
  *)
    echo "BPX_DOCKER_NETWORK_MODE must be host or bridge" >&2
    exit 2
    ;;
esac

docker run --rm \
  "${docker_network_args[@]}" \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -e PYTHONPATH=/pkg \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  -v "$PKG:/pkg:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && cd /pkg && exec python3 -m bpx_quadruped.read_only_probe "$@"' \
  probe "$@"
