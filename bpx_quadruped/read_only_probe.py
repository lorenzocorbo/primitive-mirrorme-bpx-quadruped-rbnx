"""Bounded BPX state capture using RequestRobotState only.

This module intentionally does not import or construct MotionLevelControl or
JointLevelControl. It never sends posture, velocity, zero-position or joint
commands.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import sys
import time
from typing import Any, Dict, Iterable, Optional


STATE_FIELDS = (
    ("joint_position", "getJointPosition"),
    ("joint_velocity", "getJointVelocity"),
    ("joint_torque", "getJointTorque"),
    ("imu_rpy", "getImuRpy"),
    ("imu_quaternion", "getImuQuat"),
    ("imu_acceleration", "getImuAcc"),
    ("imu_angular_velocity", "getImuOmega"),
    ("leg_odometry", "getLegOdom"),
    ("motor_temperature", "getMotorTemperature"),
    ("driver_temperature", "getDriverTemperature"),
    ("motion_state", "getCurrentMotionState"),
    ("motion_gait", "getCurrentGait"),
    ("sub_gait", "getSubGait"),
    ("max_velocity", "getMaxVelocity"),
    ("battery_level", "getBatteryLevel"),
    ("battery_current", "getBatteryCurrent"),
    ("joint_timestamp", "getJointStateTimestamp"),
    ("imu_timestamp", "getImuTimestamp"),
    ("odometry_timestamp", "getOdometryTimestamp"),
    ("motion_state_timestamp", "getMotionStateTimestamp"),
    ("battery_timestamp", "getBatteryTimestamp"),
)


def collect_sample(robot_state: Any) -> Dict[str, Any]:
    sample: Dict[str, Any] = {
        "host_wall_time": datetime.now(timezone.utc).isoformat(),
        "host_monotonic_ns": time.monotonic_ns(),
        "connected": bool(robot_state.isConnected()),
    }
    errors: Dict[str, str] = {}
    for output_name, method_name in STATE_FIELDS:
        method = getattr(robot_state, method_name, None)
        if method is None:
            errors[output_name] = "method unavailable"
            continue
        try:
            sample[output_name] = _json_safe(method())
        except Exception as exc:  # SDK calls must not terminate cleanup
            errors[output_name] = "{}: {}".format(type(exc).__name__, exc)
    if errors:
        sample["errors"] = errors
    return sample


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture BPX state with RequestRobotState; sends no motion commands."
    )
    parser.add_argument("--robot-ip", default="10.21.20.1")
    parser.add_argument("--robot-state-port", type=int, default=9873)
    parser.add_argument("--tcp-local-port", type=int, default=0)
    parser.add_argument("--state-rate-hz", type=int, default=50)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--sample-period-s", type=float, default=0.1)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not 1024 <= args.robot_state_port <= 65535:
        raise ValueError("robot-state-port must be in 1024..65535")
    if not 0 <= args.tcp_local_port <= 65535:
        raise ValueError("tcp-local-port must be in 0..65535")
    if not 1 <= args.state_rate_hz <= 200:
        raise ValueError("state-rate-hz must be in 1..200")
    for name in ("connect_timeout_s", "duration_s", "sample_period_s"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and > 0".format(name.replace("_", "-")))


def run(args: argparse.Namespace) -> int:
    validate_args(args)
    import bpx_sdk

    robot_state = bpx_sdk.RequestRobotState()
    robot_state.setRobotIp(args.robot_ip)
    robot_state.setRobotStateUploadPort(args.robot_state_port)
    robot_state.setTcpLocalPort(args.tcp_local_port)
    robot_state.setRobotStateUploadRate(args.state_rate_hz)

    print(
        json.dumps(
            {
                "event": "probe_start",
                "sdk_version": getattr(bpx_sdk, "__version__", "unknown"),
                "robot_ip": args.robot_ip,
                "robot_state_port": args.robot_state_port,
                "state_rate_hz": args.state_rate_hz,
            },
            sort_keys=True,
        ),
        flush=True,
    )

    if not robot_state.connect():
        raise RuntimeError("RequestRobotState.connect() returned false")

    try:
        connect_deadline = time.monotonic() + args.connect_timeout_s
        while not robot_state.isConnected():
            if time.monotonic() >= connect_deadline:
                raise TimeoutError("BPX state connection did not become ready")
            time.sleep(0.05)

        end_at = time.monotonic() + args.duration_s
        while time.monotonic() < end_at:
            print(json.dumps(collect_sample(robot_state), sort_keys=True), flush=True)
            time.sleep(args.sample_period_s)
    finally:
        robot_state.disconnect()
        print(json.dumps({"event": "probe_stop"}, sort_keys=True), flush=True)
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print("read-only probe failed: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
