#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
TEST_IMAGE="${BPX_RBNX_TEST_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:test}"
cd "$PKG"

command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 1
}

DOCKER_FLAGS=(--network=host)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && DOCKER_FLAGS+=(--no-cache)
DOCKER_FLAGS+=(--build-arg "USE_TUNA_MIRROR=${BPX_USE_TUNA_MIRROR:-1}")

docker build "${DOCKER_FLAGS[@]}" --target test -t "$TEST_IMAGE" -f docker/Dockerfile .
docker build "${DOCKER_FLAGS[@]}" --target runtime -t "$IMAGE" -f docker/Dockerfile .

echo "built $IMAGE after containerized tests passed"
