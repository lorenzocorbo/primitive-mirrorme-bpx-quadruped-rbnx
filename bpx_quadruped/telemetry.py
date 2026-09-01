"""Validation boundary for normalized BPX telemetry.

Every state source must return the same transport-independent ``RobotState``
shape.  Keeping the checks here prevents SDK, replay, and future recording
formats from leaking special cases into the ROS/Robonix publishing loop.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Optional, Sequence, Tuple

from .model import RobotState, VendorTimestamps


def normalize_robot_state(state: RobotState) -> RobotState:
    """Validate a state sample and return one with unit quaternions.

    Raises ``ValueError`` for malformed telemetry.  Empty optional vectors are
    preserved, while present vectors must have the exact BPX dimensions.
    """

    if not isinstance(state.connected, bool):
        raise ValueError("connected must be a boolean")
    _finite_scalar(state.received_at_s, "received_at_s")

    return replace(
        state,
        position_m=_vector(state.position_m, 3, "position_m"),
        orientation_xyzw=_unit_quaternion(
            state.orientation_xyzw, "orientation_xyzw"
        ),
        linear_velocity_body_mps=_vector(
            state.linear_velocity_body_mps, 3, "linear_velocity_body_mps"
        ),
        angular_velocity_body_rps=_vector(
            state.angular_velocity_body_rps, 3, "angular_velocity_body_rps"
        ),
        imu_orientation_xyzw=_optional_quaternion(
            state.imu_orientation_xyzw, "imu_orientation_xyzw"
        ),
        imu_linear_acceleration_mps2=_optional_vector(
            state.imu_linear_acceleration_mps2,
            3,
            "imu_linear_acceleration_mps2",
        ),
        imu_angular_velocity_rps=_optional_vector(
            state.imu_angular_velocity_rps, 3, "imu_angular_velocity_rps"
        ),
        joint_position_rad=_optional_vector(
            state.joint_position_rad, 12, "joint_position_rad"
        ),
        joint_velocity_rad_s=_optional_vector(
            state.joint_velocity_rad_s, 12, "joint_velocity_rad_s"
        ),
        joint_torque_nm=_optional_vector(
            state.joint_torque_nm, 12, "joint_torque_nm"
        ),
        motor_temperature_c=_optional_vector(
            state.motor_temperature_c, 12, "motor_temperature_c"
        ),
        driver_temperature_c=_optional_vector(
            state.driver_temperature_c, 12, "driver_temperature_c"
        ),
        motion_state=_optional_int(state.motion_state, "motion_state"),
        motion_gait=_optional_int(state.motion_gait, "motion_gait"),
        sub_gait=_optional_int(state.sub_gait, "sub_gait"),
        battery_level_percent=_optional_int(
            state.battery_level_percent, "battery_level_percent"
        ),
        battery_current_a=_optional_float(
            state.battery_current_a, "battery_current_a"
        ),
        vendor_timestamps=_timestamps(state.vendor_timestamps),
    )


def state_for_publication(
    state: RobotState, *, now_s: float, timeout_s: float
) -> Optional[RobotState]:
    """Return a normalized fresh sample, or ``None`` when it must be dropped."""

    if not math.isfinite(now_s) or not math.isfinite(timeout_s) or timeout_s <= 0.0:
        return None
    try:
        normalized = normalize_robot_state(state)
    except (TypeError, ValueError):
        return None
    age_s = now_s - normalized.received_at_s
    if not normalized.connected or not math.isfinite(age_s):
        return None
    if age_s < 0.0 or age_s > timeout_s:
        return None
    return normalized


def _timestamps(value: VendorTimestamps) -> VendorTimestamps:
    if not isinstance(value, VendorTimestamps):
        raise ValueError("vendor_timestamps must be VendorTimestamps")
    return VendorTimestamps(
        joint_ms=_optional_timestamp(value.joint_ms, "joint_ms"),
        imu_ms=_optional_timestamp(value.imu_ms, "imu_ms"),
        odometry_ms=_optional_timestamp(value.odometry_ms, "odometry_ms"),
        motion_state_ms=_optional_timestamp(
            value.motion_state_ms, "motion_state_ms"
        ),
        battery_ms=_optional_timestamp(value.battery_ms, "battery_ms"),
    )


def _unit_quaternion(value: Sequence[float], name: str) -> Tuple[float, ...]:
    result = _vector(value, 4, name)
    norm = math.sqrt(sum(component * component for component in result))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("{} has zero or invalid norm".format(name))
    return tuple(component / norm for component in result)


def _optional_quaternion(value: Sequence[float], name: str) -> Tuple[float, ...]:
    if not value:
        return ()
    return _unit_quaternion(value, name)


def _optional_vector(
    value: Sequence[float], length: int, name: str
) -> Tuple[float, ...]:
    if not value:
        return ()
    return _vector(value, length, name)


def _vector(value: Sequence[float], length: int, name: str) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)) or len(value) != length:
        raise ValueError("{} must contain {} values".format(name, length))
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError("{} contains a non-finite value".format(name))
    return result


def _finite_scalar(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be a finite number".format(name))
    return result


def _optional_float(value: Optional[float], name: str) -> Optional[float]:
    if value is None:
        return None
    return _finite_scalar(value, name)


def _optional_int(value: Optional[int], name: str) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer".format(name))
    return value


def _optional_timestamp(value: Optional[int], name: str) -> Optional[int]:
    result = _optional_int(value, name)
    if result is not None and result < 0:
        raise ValueError("{} must be non-negative".format(name))
    return result
