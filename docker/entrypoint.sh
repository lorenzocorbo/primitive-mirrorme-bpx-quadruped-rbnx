#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

if [[ -d /pkg ]]; then
  cd /pkg
  export PYTHONPATH="/pkg:${PYTHONPATH:-}"
fi
if [[ -d /robonix-api ]]; then
  export PYTHONPATH="/robonix-api:${PYTHONPATH:-}"
fi

if [[ -n "${BPX_SDK_WHEEL:-}" ]]; then
  [[ -f "$BPX_SDK_WHEEL" ]] || {
    echo "mounted BPX_SDK_WHEEL is missing: $BPX_SDK_WHEEL" >&2
    exit 1
  }
  python3 -m pip install --no-index --no-deps "$BPX_SDK_WHEEL"
  python3 -c 'import bpx_sdk; assert bpx_sdk.__version__ == "1.0.8"'
fi

exec "$@"
