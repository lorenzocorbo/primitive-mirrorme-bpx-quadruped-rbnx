"""Bounded manual BPX stand/yaw-circle/stop/sit hardware test.

This is intentionally not a Robonix ``twist_in`` adapter. It exposes only a
zero-linear-velocity yaw request for one manually confirmed test cycle and
never imports or constructs ``JointLevelControl``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol

from .model import RobotState
from .sdk_posture import (
    MOTION_STATE_LYING_DOWN,
    MOTION_STATE_MOTION,
    PostureCycleConfig,
    PostureSession,
    SdkPostureSession,
    command_until_state,
    flush_sit_down,
    raise_if_stopped,
    require_healthy_state,
)


@dataclass(frozen=True)
class YawCircleConfig:
    yaw_rate_rps: float = 0.5
    turn_angle_rad: float = 2.0 * math.pi
    turn_timeout_s: float = 30.0
    max_translation_m: float = 0.15
    opposite_direction_tolerance_rad: float = 0.2
    zero_flush_s: float = 2.0
    stand_timeout_s: float = 15.0
    sit_timeout_s: float = 15.0
    poll_period_s: float = 0.1
    state_timeout_s: float = 0.5
    cleanup_sit_flush_s: float = 1.0

    def validate(self) -> None:
        numeric = {
            "yaw_rate_rps": self.yaw_rate_rps,
            "turn_angle_rad": self.turn_angle_rad,
            "turn_timeout_s": self.turn_timeout_s,
            "max_translation_m": self.max_translation_m,
            "opposite_direction_tolerance_rad": self.opposite_direction_tolerance_rad,
            "zero_flush_s": self.zero_flush_s,
        }
        for name, value in numeric.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("{} must be a number".format(name))
            if not math.isfinite(float(value)):
                raise ValueError("{} must be finite".format(name))
        if not 0.1 <= abs(self.yaw_rate_rps) <= 1.0:
            raise ValueError("abs(yaw_rate_rps) must be in 0.1..1.0")
        if not 0.1 <= self.turn_angle_rad <= 2.0 * math.pi:
            raise ValueError("turn_angle_rad must be in 0.1..2*pi")
        if not 1.0 <= self.turn_timeout_s <= 120.0:
            raise ValueError("turn_timeout_s must be in 1..120")
        theoretical_s = self.turn_angle_rad / abs(self.yaw_rate_rps)
        if self.turn_timeout_s < theoretical_s:
            raise ValueError(
                "turn_timeout_s must be at least the commanded-angle duration "
                "({:.3f}s)".format(theoretical_s)
            )
        if not 0.02 <= self.max_translation_m <= 1.0:
            raise ValueError("max_translation_m must be in 0.02..1.0")
        if not 0.05 <= self.opposite_direction_tolerance_rad <= 1.0:
            raise ValueError(
                "opposite_direction_tolerance_rad must be in 0.05..1.0"
            )
        if not 0.2 <= self.zero_flush_s <= 5.0:
            raise ValueError("zero_flush_s must be in 0.2..5.0")
        self.posture_config().validate()

    def posture_config(self) -> PostureCycleConfig:
        return PostureCycleConfig(
            stand_timeout_s=self.stand_timeout_s,
            hold_duration_s=0.1,
            sit_timeout_s=self.sit_timeout_s,
            poll_period_s=self.poll_period_s,
            state_timeout_s=self.state_timeout_s,
            cleanup_sit_flush_s=self.cleanup_sit_flush_s,
        )


class YawCircleSession(PostureSession, Protocol):
    def set_velocity_enabled(self, enabled: bool) -> None:
        ...

    def request_yaw_rate(self, yaw_rate_rps: float) -> bool:
        ...


class SdkYawCircleSession(SdkPostureSession):
    """Narrow manual-test facade over the single MotionLevelControl owner."""

    @staticmethod
    def _require_posture_api(control) -> None:
        SdkPostureSession._require_posture_api(control)
        if not callable(getattr(control, "setVelocity", None)):
            raise RuntimeError("MotionLevelControl is missing: setVelocity")

    def set_velocity_enabled(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        self._require_started_control().setVelocityControlFlag(enabled)

    def request_yaw_rate(self, yaw_rate_rps: float) -> bool:
        if (
            isinstance(yaw_rate_rps, bool)
            or not isinstance(yaw_rate_rps, (int, float))
            or not math.isfinite(float(yaw_rate_rps))
            or abs(float(yaw_rate_rps)) > 1.0
        ):
            raise ValueError("yaw_rate_rps must be finite and in -1.0..1.0")
        return bool(
            self._require_started_control().setVelocity(
                0.0, 0.0, float(yaw_rate_rps)
            )
        )

    def stop(self) -> None:
        if self._control is not None and self._source is not None:
            try:
                self._control.setVelocity(0.0, 0.0, 0.0)
            except Exception:
                pass
        super().stop()


def run_yaw_circle(
    session: YawCircleSession,
    config: YawCircleConfig,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    stop_requested: Callable[[], bool] = lambda: False,
    report: Callable[[str], None] = print,
) -> None:
    """Stand, rotate about body yaw by one bounded angle, stop, and sit."""

    config.validate()
    posture = config.posture_config()
    started = False
    stand_phase_started = False
    completed = False
    try:
        session.start()
        started = True
        initial = require_healthy_state(session, posture, clock)
        if initial.motion_state != MOTION_STATE_LYING_DOWN:
            raise RuntimeError(
                "initial motion state must be LyingDown(0), got {}".format(
                    initial.motion_state
                )
            )
        report("initial state confirmed: LyingDown(0)")

        stand_phase_started = True
        command_until_state(
            session,
            posture,
            target_state=MOTION_STATE_MOTION,
            target_name="Motion(6)",
            command=session.request_stand_up,
            command_name="setStandUp",
            timeout_s=config.stand_timeout_s,
            clock=clock,
            sleep=sleep,
            stop_requested=stop_requested,
            report=report,
        )

        standing = require_healthy_state(session, posture, clock)
        previous_yaw = _yaw_from_state(standing)
        origin_x, origin_y = standing.position_m[:2]
        accumulated_yaw = 0.0
        turn_direction = math.copysign(1.0, config.yaw_rate_rps)
        maximum_translation = 0.0
        deadline = clock() + config.turn_timeout_s

        session.set_velocity_enabled(True)
        report(
            "yaw control enabled: rate={:.3f}rad/s target={:.6f}rad".format(
                config.yaw_rate_rps, config.turn_angle_rad
            )
        )
        while turn_direction * accumulated_yaw < config.turn_angle_rad:
            raise_if_stopped(stop_requested)
            state = require_healthy_state(session, posture, clock)
            if state.motion_state != MOTION_STATE_MOTION:
                raise RuntimeError(
                    "robot left Motion(6) during yaw circle: {}".format(
                        state.motion_state
                    )
                )

            current_yaw = _yaw_from_state(state)
            accumulated_yaw += math.atan2(
                math.sin(current_yaw - previous_yaw),
                math.cos(current_yaw - previous_yaw),
            )
            previous_yaw = current_yaw
            directed_progress = turn_direction * accumulated_yaw
            if directed_progress < -config.opposite_direction_tolerance_rad:
                raise RuntimeError(
                    "yaw moved {:.6f}rad in the opposite direction; tolerance "
                    "is {:.6f}rad".format(
                        -directed_progress,
                        config.opposite_direction_tolerance_rad,
                    )
                )
            translation = math.hypot(
                state.position_m[0] - origin_x,
                state.position_m[1] - origin_y,
            )
            maximum_translation = max(maximum_translation, translation)
            if translation > config.max_translation_m:
                raise RuntimeError(
                    "in-place translation {:.3f}m exceeded {:.3f}m".format(
                        translation, config.max_translation_m
                    )
                )
            if turn_direction * accumulated_yaw >= config.turn_angle_rad:
                break
            if clock() >= deadline:
                raise TimeoutError(
                    "yaw circle reached only {:.6f}rad within {:.3f}s".format(
                        turn_direction * accumulated_yaw, config.turn_timeout_s
                    )
                )
            if not session.request_yaw_rate(config.yaw_rate_rps):
                raise RuntimeError("setVelocity(0, 0, yaw) returned false")
            sleep(config.poll_period_s)

        report(
            "yaw target reached: accumulated={:.6f}rad max_translation={:.3f}m".format(
                accumulated_yaw, maximum_translation
            )
        )
        _stop_yaw_strict(session, config, clock=clock, sleep=sleep, report=report)

        command_until_state(
            session,
            posture,
            target_state=MOTION_STATE_LYING_DOWN,
            target_name="LyingDown(0)",
            command=session.request_sit_down,
            command_name="setSitDown",
            timeout_s=config.sit_timeout_s,
            clock=clock,
            sleep=sleep,
            stop_requested=stop_requested,
            report=report,
        )
        completed = True
        report("yaw-circle cycle complete: LyingDown(0)")
    finally:
        if started and stand_phase_started and not completed:
            report(
                "cycle interrupted or failed; zeroing yaw, disabling velocity, "
                "and sending bounded sit-down cleanup"
            )
            _stop_yaw_best_effort(session, config, clock=clock, sleep=sleep)
            flush_sit_down(session, posture, clock=clock, sleep=sleep)
        if started:
            session.stop()


def _yaw_from_state(state: RobotState) -> float:
    x, y, z, w = state.orientation_xyzw
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _stop_yaw_strict(
    session: YawCircleSession,
    config: YawCircleConfig,
    *,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    report: Callable[[str], None],
) -> None:
    accepted = False
    try:
        accepted = session.request_yaw_rate(0.0)
    finally:
        session.set_velocity_enabled(False)
    if not accepted:
        raise RuntimeError("zero setVelocity returned false")
    report(
        "zero yaw accepted; velocity control disabled; flushing zero for {:.3f}s".format(
            config.zero_flush_s
        )
    )
    deadline = clock() + config.zero_flush_s
    while clock() < deadline:
        if not session.request_yaw_rate(0.0):
            raise RuntimeError("zero setVelocity returned false during flush")
        sleep(min(config.poll_period_s, max(0.0, deadline - clock())))


def _stop_yaw_best_effort(
    session: YawCircleSession,
    config: YawCircleConfig,
    *,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    try:
        session.request_yaw_rate(0.0)
    except Exception:
        pass
    try:
        session.set_velocity_enabled(False)
    except Exception:
        pass
    deadline = clock() + config.zero_flush_s
    while clock() < deadline:
        try:
            session.request_yaw_rate(0.0)
        except Exception:
            pass
        sleep(min(config.poll_period_s, max(0.0, deadline - clock())))
