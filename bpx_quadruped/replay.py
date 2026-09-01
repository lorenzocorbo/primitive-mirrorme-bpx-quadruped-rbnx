"""Versioned deterministic replay source for normalized BPX telemetry."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from .model import RobotState, VendorTimestamps
from .telemetry import normalize_robot_state


REPLAY_SCHEMA = "mirrorme-bpx-robot-state"
REPLAY_VERSION = 1


@dataclass(frozen=True)
class ReplayFrame:
    """A sample event and its original freshness time, both relative seconds."""

    offset_s: float
    state: RobotState


class ReplayStateSource:
    """Serve recorded samples using an injected monotonic clock.

    Once the final event is reached it remains cached with its original
    freshness time.  The normal watchdog can therefore detect an exhausted or
    frozen replay exactly as it detects a frozen SDK stream.
    """

    def __init__(
        self,
        frames: Sequence[ReplayFrame],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._frames = _validated_frames(frames)
        self._clock = clock
        self._lock = threading.RLock()
        self._started_at_s: Optional[float] = None
        self._index = 0

    @classmethod
    def from_path(
        cls,
        path: Union[str, Path],
        clock: Callable[[], float] = time.monotonic,
    ) -> "ReplayStateSource":
        return cls(load_replay(path), clock=clock)

    @property
    def name(self) -> str:
        return "bpx-normalized-state-replay"

    def start(self) -> None:
        with self._lock:
            if self._started_at_s is not None:
                return
            self._started_at_s = self._clock()
            self._index = 0

    def stop(self) -> None:
        with self._lock:
            self._started_at_s = None
            self._index = 0

    def read_state(self) -> RobotState:
        with self._lock:
            if self._started_at_s is None:
                raise RuntimeError("BPX replay state source is not started")
            elapsed_s = max(0.0, self._clock() - self._started_at_s)
            while (
                self._index + 1 < len(self._frames)
                and self._frames[self._index + 1].offset_s <= elapsed_s
            ):
                self._index += 1
            state = self._frames[self._index].state
            return replace(
                state,
                received_at_s=self._started_at_s + state.received_at_s,
            )


def load_replay(path: Union[str, Path]) -> Tuple[ReplayFrame, ...]:
    """Load and strictly validate a version-1 JSON Lines replay."""

    replay_path = Path(path)
    try:
        lines = [
            line
            for line in replay_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    except OSError as exc:
        raise ValueError("cannot read replay {}: {}".format(replay_path, exc)) from exc
    if not lines:
        raise ValueError("replay is empty")
    header = _json_object(lines[0], 1)
    if header != {"schema": REPLAY_SCHEMA, "version": REPLAY_VERSION}:
        raise ValueError("unsupported replay schema or version")
    frames: List[ReplayFrame] = []
    for line_number, line in enumerate(lines[1:], start=2):
        value = _json_object(line, line_number)
        if set(value) != {"offset_s", "state"}:
            raise ValueError(
                "replay line {} must contain only offset_s and state".format(
                    line_number
                )
            )
        frames.append(
            ReplayFrame(
                offset_s=_number(value["offset_s"], "offset_s"),
                state=_state_from_mapping(value["state"]),
            )
        )
    return _validated_frames(frames)


def dump_replay(frames: Iterable[ReplayFrame], path: Union[str, Path]) -> None:
    """Write a deterministic version-1 JSON Lines replay."""

    validated = _validated_frames(tuple(frames))
    records = [
        json.dumps(
            {"schema": REPLAY_SCHEMA, "version": REPLAY_VERSION},
            separators=(",", ":"),
            sort_keys=True,
        )
    ]
    for frame in validated:
        records.append(
            json.dumps(
                {"offset_s": frame.offset_s, "state": asdict(frame.state)},
                separators=(",", ":"),
                sort_keys=True,
            )
        )
    try:
        Path(path).write_text("\n".join(records) + "\n", encoding="utf-8")
    except OSError as exc:
        raise ValueError("cannot write replay {}: {}".format(path, exc)) from exc


def _validated_frames(frames: Sequence[ReplayFrame]) -> Tuple[ReplayFrame, ...]:
    if not frames:
        raise ValueError("replay must contain at least one frame")
    result: List[ReplayFrame] = []
    previous_offset = -1.0
    for index, frame in enumerate(frames):
        if not isinstance(frame, ReplayFrame):
            raise ValueError("replay frames must be ReplayFrame values")
        offset_s = _number(frame.offset_s, "offset_s")
        if index == 0 and offset_s != 0.0:
            raise ValueError("first replay offset_s must be 0")
        if offset_s <= previous_offset:
            raise ValueError("replay offset_s values must be strictly increasing")
        state = normalize_robot_state(frame.state)
        if state.received_at_s < 0.0 or state.received_at_s > offset_s:
            raise ValueError(
                "state received_at_s must be between 0 and its frame offset_s"
            )
        result.append(ReplayFrame(offset_s=offset_s, state=state))
        previous_offset = offset_s
    return tuple(result)


def _state_from_mapping(value: Any) -> RobotState:
    if not isinstance(value, Mapping):
        raise ValueError("replay state must be an object")
    allowed = {
        field.name for field in RobotState.__dataclass_fields__.values()
    }
    unknown = sorted(set(value).difference(allowed))
    if unknown:
        raise ValueError("unknown replay state fields: {}".format(", ".join(unknown)))
    required = {"connected", "received_at_s"}
    if not required.issubset(value):
        raise ValueError("replay state requires connected and received_at_s")
    kwargs = dict(value)
    timestamps = kwargs.get("vendor_timestamps")
    if timestamps is not None:
        if not isinstance(timestamps, Mapping):
            raise ValueError("vendor_timestamps must be an object")
        timestamp_fields = {
            field.name for field in VendorTimestamps.__dataclass_fields__.values()
        }
        unknown_timestamps = sorted(set(timestamps).difference(timestamp_fields))
        if unknown_timestamps:
            raise ValueError(
                "unknown vendor timestamp fields: {}".format(
                    ", ".join(unknown_timestamps)
                )
            )
        kwargs["vendor_timestamps"] = VendorTimestamps(**dict(timestamps))
    for name in (
        "position_m",
        "orientation_xyzw",
        "linear_velocity_body_mps",
        "angular_velocity_body_rps",
        "imu_orientation_xyzw",
        "imu_linear_acceleration_mps2",
        "imu_angular_velocity_rps",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "joint_torque_nm",
        "motor_temperature_c",
        "driver_temperature_c",
    ):
        if name in kwargs:
            raw = kwargs[name]
            if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence):
                raise ValueError("{} must be an array".format(name))
            kwargs[name] = tuple(raw)
    try:
        return normalize_robot_state(RobotState(**kwargs))
    except TypeError as exc:
        raise ValueError("invalid replay state: {}".format(exc)) from exc


def _json_object(line: str, line_number: int) -> Mapping[str, Any]:
    try:
        value = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ValueError("invalid JSON on replay line {}".format(line_number)) from exc
    if not isinstance(value, Mapping):
        raise ValueError("replay line {} must be an object".format(line_number))
    return value


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite non-negative number".format(name))
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("{} must be a finite non-negative number".format(name))
    return result
