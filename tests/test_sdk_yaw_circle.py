from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import RobotState
from bpx_quadruped.sdk_posture import (
    MOTION_STATE_LYING_DOWN,
    MOTION_STATE_MOTION,
    SdkPostureConfig,
)
from bpx_quadruped.sdk_state_source import SdkStateConfig
from bpx_quadruped.sdk_yaw_circle import (
    SdkYawCircleSession,
    YawCircleConfig,
    run_yaw_circle,
)
from bpx_quadruped.yaw_circle_probe import build_parser, run as run_probe


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now


class FakeYawCircleSession:
    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.started = False
        self.stopped = False
        self.motion_state = MOTION_STATE_LYING_DOWN
        self.yaw = 0.0
        self.position_x = 0.0
        self.velocity_enabled = False
        self.current_yaw_rate = 0.0
        self.stand_calls = 0
        self.sit_calls = 0
        self.velocity_flags = []
        self.velocity_requests = []
        self.yaw_response_scale = 1.0
        self.translation_after_motion_m = 0.0
        self.events = []

    def start(self) -> None:
        self.started = True
        self.events.append((self.clock.now, "start", None))

    def stop(self) -> None:
        self.stopped = True
        self.events.append((self.clock.now, "stop", None))

    def read_state(self) -> RobotState:
        return RobotState(
            connected=True,
            received_at_s=self.clock.now,
            position_m=(self.position_x, 0.0, 0.0),
            orientation_xyzw=(
                0.0,
                0.0,
                math.sin(self.yaw / 2.0),
                math.cos(self.yaw / 2.0),
            ),
            motion_state=self.motion_state,
        )

    def request_stand_up(self) -> bool:
        self.stand_calls += 1
        self.events.append((self.clock.now, "stand", None))
        if self.stand_calls >= 2:
            self.motion_state = MOTION_STATE_MOTION
        return True

    def request_sit_down(self) -> bool:
        self.sit_calls += 1
        self.events.append((self.clock.now, "sit", None))
        if self.sit_calls >= 2:
            self.motion_state = MOTION_STATE_LYING_DOWN
        return True

    def set_velocity_enabled(self, enabled: bool) -> None:
        self.velocity_enabled = enabled
        self.velocity_flags.append(enabled)
        self.events.append((self.clock.now, "velocity_enabled", enabled))

    def request_yaw_rate(self, yaw_rate_rps: float) -> bool:
        self.current_yaw_rate = yaw_rate_rps
        self.velocity_requests.append(yaw_rate_rps)
        self.events.append((self.clock.now, "yaw", yaw_rate_rps))
        return True

    def sleep(self, duration_s: float) -> None:
        if self.velocity_enabled:
            self.yaw += self.current_yaw_rate * duration_s * self.yaw_response_scale
            if self.current_yaw_rate != 0.0:
                self.position_x = self.translation_after_motion_m
        self.clock.now += duration_s


class FakeControl:
    def __init__(self) -> None:
        self.velocity_flags = []
        self.velocity_commands = []

    def setVelocityControlFlag(self, enabled):
        self.velocity_flags.append(enabled)

    def setVelocity(self, x, y, yaw):
        self.velocity_commands.append((x, y, yaw))
        return True


class SdkYawCircleTest(unittest.TestCase):
    def test_successful_circle_uses_yaw_only_then_zero_and_sits(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)
        reports = []

        run_yaw_circle(
            session,
            YawCircleConfig(
                yaw_rate_rps=0.5,
                turn_angle_rad=2.0 * math.pi,
                turn_timeout_s=30.0,
            ),
            clock=clock,
            sleep=session.sleep,
            report=reports.append,
        )

        self.assertTrue(session.started)
        self.assertTrue(session.stopped)
        self.assertGreaterEqual(session.stand_calls, 2)
        self.assertGreaterEqual(session.sit_calls, 2)
        self.assertEqual([True, False], session.velocity_flags)
        self.assertTrue(any(value > 0.0 for value in session.velocity_requests))
        last_nonzero = max(
            index
            for index, value in enumerate(session.velocity_requests)
            if value != 0.0
        )
        self.assertTrue(
            all(value == 0.0 for value in session.velocity_requests[last_nonzero + 1 :])
        )
        self.assertTrue(any("yaw target reached" in line for line in reports))
        self.assertIn("yaw-circle cycle complete: LyingDown(0)", reports)

    def test_vendor_example_core_order_keeps_safe_feedback_closure(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)

        run_yaw_circle(
            session,
            YawCircleConfig(
                yaw_rate_rps=1.0,
                turn_angle_rad=3.0,
                turn_timeout_s=3.0,
                zero_flush_s=2.0,
                poll_period_s=0.2,
            ),
            clock=clock,
            sleep=session.sleep,
            report=lambda _message: None,
        )

        events = session.events
        stand_index = next(i for i, event in enumerate(events) if event[1] == "stand")
        enable_index = next(
            i
            for i, event in enumerate(events)
            if event[1:] == ("velocity_enabled", True)
        )
        nonzero_index = next(
            i for i, event in enumerate(events) if event[1] == "yaw" and event[2]
        )
        zero_index = next(
            i
            for i, event in enumerate(events[nonzero_index + 1 :], nonzero_index + 1)
            if event[1:] == ("yaw", 0.0)
        )
        disable_index = next(
            i
            for i, event in enumerate(events[zero_index + 1 :], zero_index + 1)
            if event[1:] == ("velocity_enabled", False)
        )
        sit_index = next(i for i, event in enumerate(events) if event[1] == "sit")
        stop_index = next(i for i, event in enumerate(events) if event[1] == "stop")

        self.assertLess(stand_index, enable_index)
        self.assertLess(enable_index, nonzero_index)
        self.assertLess(nonzero_index, zero_index)
        self.assertLess(zero_index, disable_index)
        self.assertLess(disable_index, sit_index)
        self.assertLess(sit_index, stop_index)
        self.assertGreaterEqual(events[sit_index][0] - events[zero_index][0], 2.0)
        self.assertTrue(
            all(
                event[2] == 0.0
                for event in events[zero_index:]
                if event[1] == "yaw"
            )
        )
        self.assertEqual(MOTION_STATE_LYING_DOWN, session.motion_state)

    def test_translation_limit_zeroes_and_sits(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)
        session.translation_after_motion_m = 0.2

        with self.assertRaisesRegex(RuntimeError, "translation"):
            run_yaw_circle(
                session,
                YawCircleConfig(max_translation_m=0.1),
                clock=clock,
                sleep=session.sleep,
                report=lambda _message: None,
            )

        self.assertIn(False, session.velocity_flags)
        self.assertEqual(0.0, session.velocity_requests[-1])
        self.assertGreater(session.sit_calls, 0)
        self.assertTrue(session.stopped)

    def test_turn_timeout_zeroes_and_sits(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)
        session.yaw_response_scale = 0.0

        with self.assertRaisesRegex(TimeoutError, "reached only"):
            run_yaw_circle(
                session,
                YawCircleConfig(
                    yaw_rate_rps=1.0,
                    turn_angle_rad=1.0,
                    turn_timeout_s=1.0,
                ),
                clock=clock,
                sleep=session.sleep,
                report=lambda _message: None,
            )

        self.assertIn(False, session.velocity_flags)
        self.assertEqual(0.0, session.velocity_requests[-1])
        self.assertGreater(session.sit_calls, 0)

    def test_opposite_direction_feedback_aborts_early_and_cleans_up(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)
        session.yaw_response_scale = -1.0

        with self.assertRaisesRegex(RuntimeError, "opposite direction"):
            run_yaw_circle(
                session,
                YawCircleConfig(
                    yaw_rate_rps=0.5,
                    turn_angle_rad=0.2,
                    turn_timeout_s=1.0,
                    opposite_direction_tolerance_rad=0.1,
                    zero_flush_s=0.2,
                    cleanup_sit_flush_s=0.1,
                    poll_period_s=0.1,
                ),
                clock=clock,
                sleep=session.sleep,
                report=lambda _message: None,
            )

        self.assertIn(False, session.velocity_flags)
        self.assertEqual(0.0, session.velocity_requests[-1])
        self.assertGreater(session.sit_calls, 0)
        self.assertTrue(session.stopped)
        self.assertLess(clock.now, 11.0)

    def test_negative_yaw_command_accepts_negative_feedback(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)

        run_yaw_circle(
            session,
            YawCircleConfig(
                yaw_rate_rps=-0.5,
                turn_angle_rad=0.2,
                turn_timeout_s=1.0,
                zero_flush_s=0.2,
                cleanup_sit_flush_s=0.1,
                poll_period_s=0.1,
            ),
            clock=clock,
            sleep=session.sleep,
            report=lambda _message: None,
        )

        self.assertTrue(any(value < 0.0 for value in session.velocity_requests))
        self.assertEqual(0.0, session.velocity_requests[-1])
        self.assertEqual(MOTION_STATE_LYING_DOWN, session.motion_state)
        self.assertTrue(session.stopped)

    def test_interrupt_after_yaw_starts_zeroes_disables_and_sits(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)

        with self.assertRaisesRegex(InterruptedError, "interrupted"):
            run_yaw_circle(
                session,
                YawCircleConfig(
                    yaw_rate_rps=0.5,
                    turn_angle_rad=1.0,
                    turn_timeout_s=2.0,
                    zero_flush_s=0.2,
                    cleanup_sit_flush_s=0.1,
                    poll_period_s=0.1,
                ),
                clock=clock,
                sleep=session.sleep,
                stop_requested=lambda: any(
                    value != 0.0 for value in session.velocity_requests
                ),
                report=lambda _message: None,
            )

        self.assertEqual([True, False], session.velocity_flags)
        self.assertEqual(0.0, session.velocity_requests[-1])
        self.assertGreater(session.sit_calls, 0)
        self.assertTrue(session.stopped)

    def test_opposite_direction_tolerance_is_bounded(self) -> None:
        with self.assertRaisesRegex(ValueError, "opposite_direction_tolerance_rad"):
            YawCircleConfig(opposite_direction_tolerance_rad=0.01).validate()

    def test_initial_non_lying_state_sends_no_command(self) -> None:
        clock = FakeClock()
        session = FakeYawCircleSession(clock)
        session.motion_state = MOTION_STATE_MOTION

        with self.assertRaisesRegex(RuntimeError, "initial motion state"):
            run_yaw_circle(
                session,
                YawCircleConfig(),
                clock=clock,
                sleep=session.sleep,
                report=lambda _message: None,
            )

        self.assertEqual(0, session.stand_calls)
        self.assertEqual([], session.velocity_requests)
        self.assertEqual(0, session.sit_calls)

    def test_sdk_facade_forces_zero_linear_velocity_and_caps_yaw(self) -> None:
        session = SdkYawCircleSession(
            SdkPostureConfig(state=SdkStateConfig(robot_ip="192.168.1.237"))
        )
        control = FakeControl()
        session._control = control
        session._source = object()

        session.set_velocity_enabled(True)
        self.assertTrue(session.request_yaw_rate(-0.5))
        self.assertEqual((0.0, 0.0, -0.5), control.velocity_commands[-1])
        with self.assertRaises(ValueError):
            session.request_yaw_rate(1.01)

    def test_config_requires_enough_bounded_time(self) -> None:
        with self.assertRaisesRegex(ValueError, "commanded-angle duration"):
            YawCircleConfig(
                yaw_rate_rps=0.1,
                turn_angle_rad=2.0 * math.pi,
                turn_timeout_s=30.0,
            ).validate()

    def test_probe_requires_explicit_physical_confirmation(self) -> None:
        args = argparse.Namespace(confirm_physical_yaw_circle=False)
        with self.assertRaisesRegex(ValueError, "confirm-physical-yaw-circle"):
            run_probe(args, stop_event=threading.Event())

    def test_probe_exposes_bounded_opposite_direction_tolerance(self) -> None:
        defaults = build_parser().parse_args(["--robot-ip", "192.168.1.237"])
        self.assertEqual(0.2, defaults.opposite_direction_tolerance_rad)

        overridden = build_parser().parse_args(
            [
                "--robot-ip",
                "192.168.1.237",
                "--opposite-direction-tolerance-rad",
                "0.3",
            ]
        )
        self.assertEqual(0.3, overridden.opposite_direction_tolerance_rad)


if __name__ == "__main__":
    unittest.main()
