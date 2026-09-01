"""Provider runtime seam for controlled and strictly read-only adapters."""

from __future__ import annotations

import time
from typing import Callable, Optional, Protocol

from .backend import BpxStateSource
from .config import ProviderConfig
from .controller import QuadrupedController
from .fake_backend import FakeBpxBackend
from .model import CommandDecision, PlanarTwist, RobotState
from .replay import ReplayStateSource
from .sdk_state_source import SdkStateConfig, SdkStateSource
from .telemetry import state_for_publication


class ProviderRuntime(Protocol):
    """Small interface consumed by the Robonix lifecycle provider."""

    @property
    def supports_twist(self) -> bool:
        ...

    def activate(self) -> None:
        ...

    def deactivate(self) -> None:
        ...

    def shutdown(self) -> None:
        ...

    def arm(self) -> CommandDecision:
        ...

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        ...

    def tick(self) -> RobotState:
        ...


class ReadOnlyRuntime:
    """Adapt a ``BpxStateSource`` without inventing a command interface."""

    def __init__(self, source: BpxStateSource) -> None:
        self._source = source
        self._active = False
        self._shutdown = False

    @property
    def supports_twist(self) -> bool:
        return False

    def activate(self) -> None:
        if self._shutdown:
            raise RuntimeError("read-only runtime is shut down")
        if self._active:
            return
        self._source.start()
        self._active = True

    def deactivate(self) -> None:
        if not self._active:
            return
        try:
            self._source.stop()
        finally:
            self._active = False

    def shutdown(self) -> None:
        if self._shutdown:
            return
        try:
            self.deactivate()
        finally:
            self._shutdown = True

    def arm(self) -> CommandDecision:
        return CommandDecision(False, "read-only runtime cannot arm")

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        del command
        return CommandDecision(False, "read-only runtime has no command interface")

    def tick(self) -> RobotState:
        if not self._active:
            raise RuntimeError("read-only runtime is not active")
        return self._source.read_state()


def build_runtime(
    config: ProviderConfig,
    *,
    sdk_request_factory: Optional[Callable[[], object]] = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ProviderRuntime:
    """Build the single runtime adapter selected by validated configuration."""

    if config.backend == "fake":
        return QuadrupedController(
            FakeBpxBackend(clock=clock),
            config.controller,
            clock=clock,
        )

    if config.backend == "replay":
        assert config.replay_path is not None
        return ReadOnlyRuntime(
            ReplayStateSource.from_path(config.replay_path, clock=clock)
        )

    source = SdkStateSource(
        SdkStateConfig(
            robot_ip=config.robot_ip,
            robot_state_port=config.robot_state_port,
            tcp_local_port=config.sdk_tcp_local_port,
            state_rate_hz=config.state_rate_hz,
            connect_timeout_s=config.sdk_connect_timeout_s,
            poll_period_s=config.sdk_poll_period_s,
        ),
        request_factory=sdk_request_factory,
        clock=clock,
        sleep=sleep,
    )
    return ReadOnlyRuntime(source)


def state_is_publishable(state: RobotState, *, now_s: float, timeout_s: float) -> bool:
    """Reject disconnected, stale, future-dated, or invalid host samples."""

    return state_for_publication(state, now_s=now_s, timeout_s=timeout_s) is not None
