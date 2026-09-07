from __future__ import annotations

import argparse
import os
import sys
import threading
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import RobotState
from bpx_quadruped.posture_cycle_probe import run as run_probe
from bpx_quadruped.sdk_posture import (
    MOTION_STATE_LYING_DOWN,
    MOTION_STATE_MOTION,
    PostureCycleConfig,
    SdkPostureConfig,
    SdkPostureSession,
    run_posture_cycle,
)
from bpx_quadruped.sdk_state_source import SdkStateConfig


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeMotionLevelControl:
    def __init__(self) -> None:
        self.connected = False
        self.disconnect_calls = 0
        self.robot_ip = None
        self.robot_state_port = None
        self.tcp_local_port = None
        self.state_rate_hz = None
        self.motion_command_rate_hz = None
        self.velocity_control_flags = []
        self.stand_calls = 0
        self.sit_calls = 0
        self.odometry_timestamp = 100
        self.motion_state = MOTION_STATE_LYING_DOWN

    def setRobotIp(self, value):
        self.robot_ip = value

    def setRobotStateUploadPort(self, value):
        self.robot_state_port = value

    def setTcpLocalPort(self, value):
        self.tcp_local_port = value

    def setRobotStateUploadRate(self, value):
        self.state_rate_hz = value

    def setMotionCommandRate(self, value):
        self.motion_command_rate_hz = value

    def setVelocityControlFlag(self, value):
        self.velocity_control_flags.append(value)

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False
        self.disconnect_calls += 1

    def isConnected(self):
        return self.connected

    def setStandUp(self):
        self.stand_calls += 1
        return True

    def setSitDown(self):
        self.sit_calls += 1
        return True

    def getLegOdom(self):
        self.odometry_timestamp += 1
        return {
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "velocity_body": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
        }

    def getOdometryTimestamp(self):
        return self.odometry_timestamp

    def getImuQuat(self):
        return None

    def getImuAcc(self):
        return None

    def getImuOmega(self):
        return None

    def getJointPosition(self):
        return None

    def getJointVelocity(self):
        return None

    def getJointTorque(self):
        return None

    def getMotorTemperature(self):
        return None

    def getDriverTemperature(self):
        return None

    def getCurrentMotionState(self):
        return self.motion_state

    def getCurrentGait(self):
        return 0

    def getSubGait(self):
        return 0

    def getBatteryLevel(self):
        return None

    def getBatteryCurrent(self):
        return None

    def getJointStateTimestamp(self):
        return None

    def getImuTimestamp(self):
        return None

    def getMotionStateTimestamp(self):
        return self.odometry_timestamp

    def getBatteryTimestamp(self):
        return None


class FakePostureSession:
    def __init__(self, clock: FakeClock, initial_state: int = 0) -> None:
        self.clock = clock
        self.motion_state = initial_state
        self.started = False
        self.stopped = False
        self.stand_calls = 0
        self.sit_calls = 0
        self.fail_stand = False
        self.stale = False
        self.read_calls = 0
        self.state_on_read = {}

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def read_state(self) -> RobotState:
        self.read_calls += 1
        if self.read_calls in self.state_on_read:
            self.motion_state = self.state_on_read[self.read_calls]
        timestamp = self.clock.now - 10.0 if self.stale else self.clock.now
        return RobotState(
            connected=True,
            received_at_s=timestamp,
            motion_state=self.motion_state,
        )

    def request_stand_up(self) -> bool:
        self.stand_calls += 1
        if self.fail_stand:
            return False
        if self.stand_calls >= 2:
            self.motion_state = MOTION_STATE_MOTION
        return True

    def request_sit_down(self) -> bool:
        self.sit_calls += 1
        if self.sit_calls >= 2:
            self.motion_state = MOTION_STATE_LYING_DOWN
        return True


class SdkPostureSessionTest(unittest.TestCase):
    def test_single_motion_level_object_owns_state_and_posture(self) -> None:
        clock = FakeClock()
        control = FakeMotionLevelControl()
        session = SdkPostureSession(
            SdkPostureConfig(
                state=SdkStateConfig(
                    robot_ip="192.168.1.237",
                    connect_timeout_s=1.0,
                ),
                motion_command_rate_hz=40,
            ),
            control_factory=lambda: control,
            clock=clock,
            sleep=clock.sleep,
        )

        session.start()
        self.assertEqual("192.168.1.237", control.robot_ip)
        self.assertEqual(40, control.motion_command_rate_hz)
        self.assertEqual([False], control.velocity_control_flags)
        self.assertFalse(session.supports_twist)
        self.assertFalse(hasattr(session, "set_velocity"))
        self.assertTrue(session.request_stand_up())
        self.assertTrue(session.request_sit_down())

        session.stop()
        session.stop()
        self.assertEqual([False, False], control.velocity_control_flags)
        self.assertEqual(1, control.disconnect_calls)

    def test_cycle_stands_holds_three_seconds_and_sits(self) -> None:
        clock = FakeClock()
        session = FakePostureSession(clock)
        reports = []

        run_posture_cycle(
            session,
            PostureCycleConfig(hold_duration_s=3.0),
            clock=clock,
            sleep=clock.sleep,
            report=reports.append,
        )

        self.assertTrue(session.started)
        self.assertTrue(session.stopped)
        self.assertGreaterEqual(session.stand_calls, 2)
        self.assertGreaterEqual(session.sit_calls, 2)
        self.assertGreaterEqual(clock.now, 13.4)
        self.assertTrue(any("holding for 3.000s" in line for line in reports))
        self.assertIn("posture cycle complete: LyingDown(0)", reports)

    def test_non_lying_initial_state_refuses_to_send_commands(self) -> None:
        clock = FakeClock()
        session = FakePostureSession(clock, initial_state=MOTION_STATE_MOTION)

        with self.assertRaisesRegex(RuntimeError, "initial motion state"):
            run_posture_cycle(
                session,
                PostureCycleConfig(),
                clock=clock,
                sleep=clock.sleep,
                report=lambda _message: None,
            )

        self.assertEqual(0, session.stand_calls)
        self.assertEqual(0, session.sit_calls)
        self.assertTrue(session.stopped)

    def test_failed_stand_sends_bounded_sit_cleanup(self) -> None:
        clock = FakeClock()
        session = FakePostureSession(clock)
        session.fail_stand = True

        with self.assertRaisesRegex(RuntimeError, "setStandUp returned false"):
            run_posture_cycle(
                session,
                PostureCycleConfig(cleanup_sit_flush_s=0.5),
                clock=clock,
                sleep=clock.sleep,
                report=lambda _message: None,
            )

        self.assertGreater(session.sit_calls, 0)
        self.assertTrue(session.stopped)

    def test_leaving_motion_state_during_hold_triggers_sit_cleanup(self) -> None:
        clock = FakeClock()
        session = FakePostureSession(clock)
        session.state_on_read[5] = 2

        with self.assertRaisesRegex(RuntimeError, "left Motion"):
            run_posture_cycle(
                session,
                PostureCycleConfig(),
                clock=clock,
                sleep=clock.sleep,
                report=lambda _message: None,
            )

        self.assertGreater(session.sit_calls, 0)
        self.assertTrue(session.stopped)

    def test_stale_state_refuses_cycle(self) -> None:
        clock = FakeClock()
        session = FakePostureSession(clock)
        session.stale = True

        with self.assertRaisesRegex(RuntimeError, "state is stale"):
            run_posture_cycle(
                session,
                PostureCycleConfig(),
                clock=clock,
                sleep=clock.sleep,
                report=lambda _message: None,
            )

        self.assertEqual(0, session.stand_calls)
        self.assertTrue(session.stopped)

    def test_probe_requires_explicit_physical_motion_confirmation(self) -> None:
        args = argparse.Namespace(confirm_physical_motion=False)
        with self.assertRaisesRegex(ValueError, "confirm-physical-motion"):
            run_probe(args, stop_event=threading.Event())


if __name__ == "__main__":
    unittest.main()
