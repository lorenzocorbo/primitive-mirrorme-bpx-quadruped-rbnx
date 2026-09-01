"""Project BPX telemetry into the ROS ``sensor_msgs/JointState`` shape."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .model import RobotState


# BPX SDK v1.0.8, include/motion_types.h, JointIndex and kJointNames.
# Keep this explicit: an array reordering here would animate the wrong leg.
JOINT_NAMES: Tuple[str, ...] = (
    "fl_hip_roll_joint",
    "fl_hip_pitch_joint",
    "fl_knee_joint",
    "fr_hip_roll_joint",
    "fr_hip_pitch_joint",
    "fr_knee_joint",
    "hl_hip_roll_joint",
    "hl_hip_pitch_joint",
    "hl_knee_joint",
    "hr_hip_roll_joint",
    "hr_hip_pitch_joint",
    "hr_knee_joint",
)


@dataclass(frozen=True)
class JointStatePayload:
    names: Tuple[str, ...]
    position: Tuple[float, ...]
    velocity: Tuple[float, ...]
    effort: Tuple[float, ...]


class JointStateProjector:
    """Validate and de-duplicate SDK joint samples before ROS publication.

    The SDK timestamp is used only as an opaque change counter. Its epoch and
    wrap behavior are not assumed. ROS messages receive the local ROS clock at
    the moment a new sample is observed.
    """

    def __init__(self) -> None:
        self._last_vendor_timestamp_ms: Optional[int] = None
        self._has_timestamped_sample = False

    def reset(self) -> None:
        self._last_vendor_timestamp_ms = None
        self._has_timestamped_sample = False

    def project(self, state: RobotState) -> Optional[JointStatePayload]:
        if not state.connected or not state.joint_position_rad:
            return None

        position = _vector(state.joint_position_rad, "joint position")
        velocity = _optional_vector(state.joint_velocity_rad_s, "joint velocity")

        timestamp_ms = state.vendor_timestamps.joint_ms
        if (
            timestamp_ms is not None
            and self._has_timestamped_sample
            and timestamp_ms == self._last_vendor_timestamp_ms
        ):
            return None
        if timestamp_ms is not None:
            self._last_vendor_timestamp_ms = timestamp_ms
            self._has_timestamped_sample = True

        # ROS JointState effort for revolute joints is torque in N*m. The BPX
        # public API names this field "joint torque" but does not state its
        # unit, so leave effort absent until hardware/vendor confirmation.
        return JointStatePayload(
            names=JOINT_NAMES,
            position=position,
            velocity=velocity,
            effort=(),
        )


def _optional_vector(values: Sequence[float], name: str) -> Tuple[float, ...]:
    if not values:
        return ()
    return _vector(values, name)


def _vector(values: Sequence[float], name: str) -> Tuple[float, ...]:
    if isinstance(values, (str, bytes)) or len(values) != len(JOINT_NAMES):
        raise ValueError("{} must contain {} values".format(name, len(JOINT_NAMES)))
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("{} contains a non-finite value".format(name))
    return result
