"""The small interface implemented by fake and BPX SDK adapters."""

from __future__ import annotations

from typing import Protocol, Union

from .model import PostureCommand, RobotState, StopCommand, VelocityCommand


BackendCommand = Union[VelocityCommand, StopCommand, PostureCommand]


class BpxStateSource(Protocol):
    """Read-only BPX state ownership seam.

    Keeping this narrower than :class:`BpxBackend` lets hardware bring-up use
    ``RequestRobotState`` without ever constructing a motion-control object.
    """

    @property
    def name(self) -> str:
        ...

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def read_state(self) -> RobotState:
        ...


class BpxBackend(BpxStateSource, Protocol):
    """Own one BPX connection and hide all vendor-SDK behavior.

    Implementations must be thread-safe for one controller process. ``stop``
    and a ``StopCommand`` must be idempotent and best-effort safe.
    """

    def apply(self, command: BackendCommand) -> None:
        ...
