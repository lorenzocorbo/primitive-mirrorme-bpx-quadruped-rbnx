#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
CONTAINER="${BPX_RBNX_CONTAINER:-rbnx_mirrorme_bpx_quadruped}"
LABEL_KEY="ai.robonix.package"
LABEL_VALUE="robonix.primitive.mirrorme.bpx.quadruped"
RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-robonix.primitive.mirrorme.bpx.quadruped}"
SDK_WHEEL="${BPX_SDK_WHEEL:-}"
export RBNX_INSTANCE_NAME
cd "$PKG"

command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 1
}
command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx is required to locate robonix-api" >&2
  exit 1
}

DOCKER_SDK_ARGS=()
if [[ -n "$SDK_WHEEL" ]]; then
  [[ -f "$SDK_WHEEL" ]] || {
    echo "BPX_SDK_WHEEL is not a file: $SDK_WHEEL" >&2
    exit 1
  }
  wheel_name="$(basename "$SDK_WHEEL")"
  expected_sha="$(awk -v name="$wheel_name" '$2 == name {print $1}' "$PKG/sdk/SHA256SUMS")"
  [[ -n "$expected_sha" ]] || {
    echo "BPX_SDK_WHEEL is not a pinned artifact: $wheel_name" >&2
    exit 1
  }
  actual_sha="$(sha256sum "$SDK_WHEEL" | awk '{print $1}')"
  [[ "$actual_sha" == "$expected_sha" ]] || {
    echo "BPX_SDK_WHEEL checksum mismatch: $wheel_name" >&2
    exit 1
  }
  sdk_wheel_path="$(realpath "$SDK_WHEEL")"
  BPX_SDK_WHEEL="/vendor-sdk/$wheel_name"
  export BPX_SDK_WHEEL
  DOCKER_SDK_ARGS+=(
    -v "$sdk_wheel_path:/vendor-sdk/$wheel_name:ro"
    -e BPX_SDK_WHEEL
  )
fi

DOCKER_NETWORK_ARGS=()
case "${BPX_DOCKER_NETWORK_MODE:-host}" in
  host)
    DOCKER_NETWORK_ARGS=(--network host)
    ;;
  bridge)
    state_udp_port="${BPX_DOCKER_STATE_UDP_PORT:-9873}"
    docker_host_address="${BPX_DOCKER_HOST_ADDRESS:-host.docker.internal}"
    [[ "$state_udp_port" =~ ^[0-9]+$ ]] &&
      ((state_udp_port >= 1024 && state_udp_port <= 65535)) || {
      echo "BPX_DOCKER_STATE_UDP_PORT must be in 1024..65535" >&2
      exit 2
    }
    DOCKER_NETWORK_ARGS=(
      --network bridge
      --publish "$state_udp_port:$state_udp_port/udp"
      --add-host host.docker.internal:host-gateway
    )
    case "${ROBONIX_ATLAS:-127.0.0.1:50051}" in
      127.0.0.1:*)
        ROBONIX_ATLAS="${docker_host_address}:${ROBONIX_ATLAS##*:}"
        export ROBONIX_ATLAS
        ;;
    esac
    ;;
  *)
    echo "BPX_DOCKER_NETWORK_MODE must be host or bridge" >&2
    exit 2
    ;;
esac

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
  existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
  [[ "$existing_label" == "$LABEL_VALUE" ]] || {
    echo "refusing to replace container $CONTAINER with unexpected label $existing_label" >&2
    exit 1
  }
  docker rm -f "$CONTAINER" >/dev/null
fi

cleanup() {
  bash "$PKG/scripts/stop.sh" || true
}
trap cleanup EXIT INT TERM

docker run --rm \
  --name "$CONTAINER" \
  --label "$LABEL_KEY=$LABEL_VALUE" \
  "${DOCKER_NETWORK_ARGS[@]}" \
  --ipc host \
  -e ROBONIX_ATLAS \
  -e ROBONIX_PROVIDER_BIND_HOST \
  -e ROBONIX_ADVERTISE_HOST \
  -e BPX_DOCKER_NETWORK_MODE \
  -e RBNX_INSTANCE_NAME \
  -e RBNX_DEPLOY_MANAGED \
  -e BPX_REQUIRED_BACKEND \
  -e BPX_POSTURE_CAPABILITY \
  -e BPX_REQUIRED_POSTURE_CAPABILITY \
  -e ROBONIX_DRIVER_CONTRACT_ID \
  -e ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK \
  -e ROS_DOMAIN_ID \
  -e RMW_IMPLEMENTATION \
  -e ROBONIX_ZENOH_ROUTER \
  -e ROBONIX_ZENOH_MODE \
  -v "$PKG:/pkg" \
  -v "$(rbnx path robonix-api):/robonix-api:ro" \
  "${DOCKER_SDK_ARGS[@]}" \
  "$IMAGE" &
runner_pid=$!
wait "$runner_pid"
