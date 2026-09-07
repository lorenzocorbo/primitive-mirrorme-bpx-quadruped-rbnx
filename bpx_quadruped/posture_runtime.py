"""Lifecycle-safe Robonix posture runtime for BPX hardware.

The runtime deliberately exposes only the named ``stand`` and ``sit``
operations.  It owns one :class:`SdkPostureSession`, serializes RPC calls, and
uses the inherited robot-state stream to close every command on a reported
motion state.  Velocity and joint-level control are not part of this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable

from .model import CommandDecision, PlanarTwist, RobotState
from .sdk_posture import (
    MOTION_STATE_LYING_DOWN,
    MOTION_STATE_MOTION,
    PostureCycleConfig,
    PostureSession,
    command_until_state,
    flush_sit_down,
    require_healthy_state,
)


@dataclass(frozen=True)
class PostureRuntimeConfig:
    stand_timeout_s: float = 15.0
    sit_timeout_s: float = 15.0
    poll_period_s: float = 0.2
    state_timeout_s: float = 0.5
    cleanup_sit_flush_s: float = 1.0

    def validate(self) -> None:
        bounded = {
            "stand_timeout_s": (self.stand_timeout_s, 0.1, 60.0),
            "sit_timeout_s": (self.sit_timeout_s, 0.1, 60.0),
            "poll_period_s": (self.poll_period_s, 0.01, 1.0),
            "state_timeout_s": (self.state_timeout_s, 0.1, 5.0),
            "cleanup_sit_flush_s": (self.cleanup_sit_flush_s, 0.1, 3.0),
        }
        for name, (value, lower, upper) in bounded.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("{} must be a number".format(name))
            if not math.isfinite(float(value)) or not lower <= value <= upper:
                raise ValueError(
                    "{} must be finite and in {}..{}".format(name, lower, upper)
                )

    def command_config(self) -> PostureCycleConfig:
        """Reuse the probe's state/timeout checks without its hold phase."""

        return PostureCycleConfig(
            stand_timeout_s=self.stand_timeout_s,
            hold_duration_s=0.1,
            sit_timeout_s=self.sit_timeout_s,
            poll_period_s=self.poll_period_s,
            state_timeout_s=self.state_timeout_s,
            cleanup_sit_flush_s=self.cleanup_sit_flush_s,
        )


class PostureRuntime:
    """One-owner SDK runtime exposing only state and guarded stand/sit."""

    def __init__(
        self,
        session: PostureSession,
        config: PostureRuntimeConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        config.validate()
        self._session = session
        self._config = config
        self._command_config = config.command_config()
        self._clock = clock
        self._sleep = sleep
        self._operation_lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._cancel = threading.Event()
        self._active = False
        self._shutdown = False

    @property
    def supports_twist(self) -> bool:
        return False

    @property
    def supports_posture(self) -> bool:
        return True

    def activate(self) -> None:
        with self._operation_lock:
            with self._lifecycle_lock:
                if self._shutdown:
                    raise RuntimeError("posture runtime is shut down")
                if self._active:
                    return
                self._cancel.clear()
                try:
                    self._session.start()
                    state = self._healthy_state()
                    if state.motion_state not in {
                        MOTION_STATE_LYING_DOWN,
                        MOTION_STATE_MOTION,
                    }:
                        raise RuntimeError(
                            "posture activation requires LyingDown(0) or Motion(6), "
                            "got {}".format(
                                state.motion_state
                            )
                        )
                    self._active = True
                except BaseException:
                    self._session.stop()
                    raise

    def deactivate(self) -> None:
        self._cancel.set()
        with self._operation_lock:
            with self._lifecycle_lock:
                if not self._active:
                    return
                try:
                    # Lifecycle transitions must not create an unexpected body
                    # motion. SdkPostureSession.stop() disables velocity mode
                    # before disconnecting the one MotionLevelControl owner.
                    self._session.stop()
                finally:
                    self._active = False

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            if self._shutdown:
                return
        try:
            self.deactivate()
        finally:
            with self._lifecycle_lock:
                self._shutdown = True

    def arm(self) -> CommandDecision:
        return CommandDecision(False, "posture runtime has no twist arm")

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        del command
        return CommandDecision(False, "posture runtime has no twist interface")

    def tick(self) -> RobotState:
        with self._lifecycle_lock:
            if not self._active:
                raise RuntimeError("posture runtime is not active")
        return self._session.read_state()

    def set_posture(self, posture_name: str) -> CommandDecision:
        if posture_name not in {"stand", "sit"}:
            return CommandDecision(False, "supported posture names are: stand, sit")
        if not self._operation_lock.acquire(blocking=False):
            return CommandDecision(False, "another posture operation is in progress")
        try:
            with self._lifecycle_lock:
                if not self._active:
                    return CommandDecision(False, "posture runtime is not active")
                if self._cancel.is_set():
                    return CommandDecision(False, "posture runtime is stopping")
            return self._execute_posture(posture_name)
        except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
            return CommandDecision(False, str(exc))
        finally:
            self._operation_lock.release()

    def _execute_posture(self, posture_name: str) -> CommandDecision:
        state = self._healthy_state()
        if posture_name == "stand":
            if state.motion_state == MOTION_STATE_MOTION:
                return CommandDecision(True, "already standing: Motion(6)")
            if state.motion_state != MOTION_STATE_LYING_DOWN:
                return CommandDecision(
                    False,
                    "stand requires LyingDown(0), got {}".format(state.motion_state),
                )
            try:
                self._command_until(
                    target_state=MOTION_STATE_MOTION,
                    target_name="Motion(6)",
                    command=self._session.request_stand_up,
                    command_name="setStandUp",
                    timeout_s=self._config.stand_timeout_s,
                )
            except BaseException:
                # A command failure may use a bounded sit request as part of
                # that explicit posture operation. A lifecycle cancellation,
                # however, must not manufacture a new body-motion command.
                if not self._cancel.is_set():
                    flush_sit_down(
                        self._session,
                        self._command_config,
                        clock=self._clock,
                        sleep=self._sleep,
                    )
                raise
            return CommandDecision(True, "standing confirmed: Motion(6)")

        if state.motion_state == MOTION_STATE_LYING_DOWN:
            return CommandDecision(True, "already sitting: LyingDown(0)")
        if state.motion_state != MOTION_STATE_MOTION:
            return CommandDecision(
                False,
                "sit requires Motion(6), got {}".format(state.motion_state),
            )
        try:
            self._command_until(
                target_state=MOTION_STATE_LYING_DOWN,
                target_name="LyingDown(0)",
                command=self._session.request_sit_down,
                command_name="setSitDown",
                timeout_s=self._config.sit_timeout_s,
            )
        except BaseException:
            if not self._cancel.is_set():
                flush_sit_down(
                    self._session,
                    self._command_config,
                    clock=self._clock,
                    sleep=self._sleep,
                )
            raise
        return CommandDecision(True, "sitting confirmed: LyingDown(0)")

    def _command_until(
        self,
        *,
        target_state: int,
        target_name: str,
        command: Callable[[], bool],
        command_name: str,
        timeout_s: float,
    ) -> None:
        command_until_state(
            self._session,
            self._command_config,
            target_state=target_state,
            target_name=target_name,
            command=command,
            command_name=command_name,
            timeout_s=timeout_s,
            clock=self._clock,
            sleep=self._sleep,
            stop_requested=self._cancel.is_set,
            report=lambda _message: None,
        )

    def _healthy_state(self) -> RobotState:
        return require_healthy_state(
            self._session, self._command_config, self._clock
        )
