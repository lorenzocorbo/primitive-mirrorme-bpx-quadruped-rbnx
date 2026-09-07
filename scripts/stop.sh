#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${BPX_RBNX_CONTAINER:-rbnx_mirrorme_bpx_quadruped}"
LABEL_KEY="ai.robonix.package"
LABEL_VALUE="robonix.primitive.mirrorme.bpx.quadruped"

docker inspect "$CONTAINER" >/dev/null 2>&1 || exit 0
existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
[[ "$existing_label" == "$LABEL_VALUE" ]] || {
  echo "refusing to stop container $CONTAINER with unexpected label $existing_label" >&2
  exit 1
}

docker stop --time "${BPX_RBNX_STOP_TIMEOUT_S:-30}" "$CONTAINER" >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
