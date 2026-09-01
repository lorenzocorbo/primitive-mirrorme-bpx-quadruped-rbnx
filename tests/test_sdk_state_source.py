from __future__ import annotations

import os
import sys
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.sdk_state_source import SdkStateConfig, SdkStateSource


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeRequestRobotState:
    def __init__(self, odometry_after_reads: int = 0) -> None:
        self.odometry_after_reads = odometry_after_reads
        self.odom_reads = 0
        self.connected = False
        self.disconnect_calls = 0
        self.robot_ip = None
        self.robot_state_port = None
        self.tcp_local_port = None
        self.state_rate_hz = None
        self.odom_timestamp = 100

    def setRobotIp(self, value):
        self.robot_ip = value

    def setRobotStateUploadPort(self, value):
        self.robot_state_port = value

    def setTcpLocalPort(self, value):
        self.tcp_local_port = value

    def setRobotStateUploadRate(self, value):
        self.state_rate_hz = value

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False
        self.disconnect_calls += 1

    def isConnected(self):
        return self.connected

    def getLegOdom(self):
        self.odom_reads += 1
        if self.odom_reads <= self.odometry_after_reads:
            return None
        return {
            "position": [1.0, 2.0, 3.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "velocity_body": [0.1, 0.2, 0.3],
            "angular_velocity": [0.4, 0.5, 0.6],
        }

    def getOdometryTimestamp(self):
        return None if self.odom_reads <= self.odometry_after_reads else self.odom_timestamp

    def getImuQuat(self):
        return [0.0, 0.0, 0.0, 1.0]

    def getImuAcc(self):
        return [0.0, 0.0, 9.8]

    def getImuOmega(self):
        return [0.01, 0.02, 0.03]

    def getJointPosition(self):
        return [0.1] * 12

    def getJointVelocity(self):
        return [0.2] * 12

    def getJointTorque(self):
        return [0.3] * 12

    def getMotorTemperature(self):
        return [40.0] * 12

    def getDriverTemperature(self):
        return [41.0] * 12

    def getCurrentMotionState(self):
        return 4

    def getCurrentGait(self):
        return 0

    def getSubGait(self):
        return 2

    def getBatteryLevel(self):
        return 88

    def getBatteryCurrent(self):
        return -1.25

    def getJointStateTimestamp(self):
        return 98

    def getImuTimestamp(self):
        return 99

    def getMotionStateTimestamp(self):
        return 97

    def getBatteryTimestamp(self):
        return 96


class SdkStateSourceTest(unittest.TestCase):
    def test_maps_sdk_state_without_a_command_interface(self) -> None:
        clock = FakeClock()
        request = FakeRequestRobotState(odometry_after_reads=1)
        source = SdkStateSource(
            SdkStateConfig(connect_timeout_s=1.0, poll_period_s=0.1),
            request_factory=lambda: request,
            clock=clock,
            sleep=clock.sleep,
        )

        source.start()
        state = source.read_state()

        self.assertEqual("10.21.20.1", request.robot_ip)
        self.assertEqual((1.0, 2.0, 3.0), state.position_m)
        self.assertEqual(12, len(state.joint_position_rad))
        self.assertEqual(88, state.battery_level_percent)
        self.assertEqual(100, state.vendor_timestamps.odometry_ms)
        self.assertFalse(hasattr(source, "apply"))

        source.stop()
        source.stop()
        self.assertEqual(1, request.disconnect_calls)

    def test_cached_vendor_frame_does_not_look_fresh(self) -> None:
        clock = FakeClock()
        request = FakeRequestRobotState()
        source = SdkStateSource(
            SdkStateConfig(),
            request_factory=lambda: request,
            clock=clock,
            sleep=clock.sleep,
        )
        source.start()
        first = source.read_state()
        clock.now += 5.0
        cached = source.read_state()
        self.assertEqual(first.received_at_s, cached.received_at_s)

        request.odom_timestamp += 1
        fresh = source.read_state()
        self.assertEqual(clock.now, fresh.received_at_s)

    def test_first_frame_timeout_disconnects(self) -> None:
        clock = FakeClock()
        request = FakeRequestRobotState(odometry_after_reads=1000)
        source = SdkStateSource(
            SdkStateConfig(connect_timeout_s=0.2, poll_period_s=0.05),
            request_factory=lambda: request,
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaisesRegex(TimeoutError, "odometry frame"):
            source.start()
        self.assertEqual(1, request.disconnect_calls)

    def test_invalid_telemetry_is_rejected(self) -> None:
        clock = FakeClock()
        request = FakeRequestRobotState()
        request.getImuQuat = lambda: [float("nan"), 0.0, 0.0, 1.0]
        source = SdkStateSource(
            SdkStateConfig(),
            request_factory=lambda: request,
            clock=clock,
            sleep=clock.sleep,
        )

        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            source.start()
        self.assertEqual(1, request.disconnect_calls)


if __name__ == "__main__":
    unittest.main()
