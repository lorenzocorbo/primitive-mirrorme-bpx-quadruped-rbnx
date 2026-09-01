"""Transport-independent types used at the backend seam."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping, Optional, Tuple


class LifecycleState(Enum):
    INACTIVE = "inactive"
    ACTIVE = "active"
    SHUTDOWN = "shutdown"


class SafetyState(Enum):
    DISARMED = "disarmed"
    ARMED = "armed"
    FAULT = "fault"


class Posture(Enum):
    STAND = "stand"
    SIT = "sit"
    DAMPING = "damping"


@dataclass(frozen=True)
class PlanarTwist:
    linear_x_mps: float = 0.0
    linear_y_mps: float = 0.0
    angular_z_rps: float = 0.0

    def is_finite(self) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                self.linear_x_mps,
                self.linear_y_mps,
                self.angular_z_rps,
            )
        )

    def is_zero(self, tolerance: float = 0.0) -> bool:
        return all(
            abs(value) <= tolerance
            for value in (
                self.linear_x_mps,
                self.linear_y_mps,
                self.angular_z_rps,
            )
        )


ZERO_TWIST = PlanarTwist()


@dataclass(frozen=True)
class VelocityCommand:
    twist: PlanarTwist


@dataclass(frozen=True)
class StopCommand:
    reason: str


@dataclass(frozen=True)
class PostureCommand:
    posture: Posture


@dataclass(frozen=True)
class VendorTimestamps:
    """Raw millisecond counters reported by the BPX motion host.

    Their epoch, reset, and wrap behavior still require hardware validation;
    callers must not treat them as ROS wall-clock timestamps.
    """

    joint_ms: Optional[int] = None
    imu_ms: Optional[int] = None
    odometry_ms: Optional[int] = None
    motion_state_ms: Optional[int] = None
    battery_ms: Optional[int] = None


@dataclass(frozen=True)
class RobotState:
    connected: bool
    received_at_s: float
    position_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    linear_velocity_body_mps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity_body_rps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_orientation_xyzw: Tuple[float, ...] = ()
    imu_linear_acceleration_mps2: Tuple[float, ...] = ()
    imu_angular_velocity_rps: Tuple[float, ...] = ()
    joint_position_rad: Tuple[float, ...] = ()
    joint_velocity_rad_s: Tuple[float, ...] = ()
    joint_torque_nm: Tuple[float, ...] = ()
    motor_temperature_c: Tuple[float, ...] = ()
    driver_temperature_c: Tuple[float, ...] = ()
    motion_state: Optional[int] = None
    motion_gait: Optional[int] = None
    sub_gait: Optional[int] = None
    battery_level_percent: Optional[int] = None
    battery_current_a: Optional[float] = None
    vendor_timestamps: VendorTimestamps = VendorTimestamps()


@dataclass(frozen=True)
class CommandDecision:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class ControllerSnapshot:
    lifecycle: LifecycleState
    safety: SafetyState
    fault_reason: str
    last_output: PlanarTwist


@dataclass(frozen=True)
class ControllerConfig:
    allow_motion: bool = False
    command_timeout_s: float = 0.25
    state_timeout_s: float = 0.50
    max_linear_x_mps: float = 0.30
    max_linear_y_mps: float = 0.20
    max_angular_z_rps: float = 0.50
    max_linear_accel_mps2: float = 0.50
    max_angular_accel_rps2: float = 1.00
    zero_preamble_count: int = 3

    def validate(self) -> None:
        if not isinstance(self.allow_motion, bool):
            raise ValueError("allow_motion must be a boolean")
        positive = {
            "command_timeout_s": self.command_timeout_s,
            "state_timeout_s": self.state_timeout_s,
            "max_linear_accel_mps2": self.max_linear_accel_mps2,
            "max_angular_accel_rps2": self.max_angular_accel_rps2,
        }
        non_negative = {
            "max_linear_x_mps": self.max_linear_x_mps,
            "max_linear_y_mps": self.max_linear_y_mps,
            "max_angular_z_rps": self.max_angular_z_rps,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and > 0".format(name))
        for name, value in non_negative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and >= 0".format(name))
        if (
            isinstance(self.zero_preamble_count, bool)
            or not isinstance(self.zero_preamble_count, int)
            or self.zero_preamble_count < 1
        ):
            raise ValueError("zero_preamble_count must be an integer >= 1")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "ControllerConfig":
        known = {
            "allow_motion",
            "command_timeout_s",
            "state_timeout_s",
            "max_linear_x_mps",
            "max_linear_y_mps",
            "max_angular_z_rps",
            "max_linear_accel_mps2",
            "max_angular_accel_rps2",
            "zero_preamble_count",
        }
        unknown = sorted(set(values).difference(known))
        if unknown:
            raise ValueError("unknown controller config fields: {}".format(", ".join(unknown)))
        cfg = cls(**dict(values))
        cfg.validate()
        return cfg
