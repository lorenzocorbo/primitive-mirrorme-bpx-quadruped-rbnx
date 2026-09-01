"""Offline compatibility check for the externally supplied BPX SDK wheel."""

from __future__ import annotations

import json
from types import ModuleType
from typing import Any, Dict


REQUIRED_READ_METHODS = (
    "connect",
    "disconnect",
    "isConnected",
    "setRobotIp",
    "setRobotStateUploadPort",
    "setRobotStateUploadRate",
    "setTcpLocalPort",
    "getLegOdom",
    "getOdometryTimestamp",
    "getImuQuat",
    "getImuAcc",
    "getImuOmega",
    "getJointPosition",
    "getJointVelocity",
    "getJointTorque",
)

MOTION_METHODS = (
    "setZeroPositionsFlag",
    "setVelocity",
    "setStandUp",
    "setSitDown",
    "setDamping",
)


def validate_sdk_module(sdk: ModuleType) -> Dict[str, Any]:
    version = getattr(sdk, "__version__", None)
    if version != "1.0.8":
        raise RuntimeError("expected BPX SDK 1.0.8, got {!r}".format(version))
    request_type = getattr(sdk, "RequestRobotState", None)
    if request_type is None:
        raise RuntimeError("BPX SDK has no RequestRobotState")
    missing = [name for name in REQUIRED_READ_METHODS if not hasattr(request_type, name)]
    if missing:
        raise RuntimeError("RequestRobotState is missing: {}".format(", ".join(missing)))
    leaked_motion = [name for name in MOTION_METHODS if hasattr(request_type, name)]
    if leaked_motion:
        raise RuntimeError(
            "RequestRobotState unexpectedly exposes motion methods: {}".format(
                ", ".join(leaked_motion)
            )
        )
    return {
        "sdk_version": version,
        "state_api_methods_checked": len(REQUIRED_READ_METHODS),
        "request_robot_state_has_motion_methods": False,
    }


def main() -> int:
    import bpx_sdk

    print(json.dumps(validate_sdk_module(bpx_sdk), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
