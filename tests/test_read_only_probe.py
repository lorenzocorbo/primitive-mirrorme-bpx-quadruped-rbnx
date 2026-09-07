from __future__ import annotations

import argparse
import os
import sys
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.read_only_probe import collect_sample, validate_args


class FakeRobotState:
    def isConnected(self):
        return True

    def getJointPosition(self):
        return [0.0] * 12

    def getLegOdom(self):
        return {
            "velocity_body": [0.0, 0.0, 0.0],
            "position": [1.0, 2.0, 3.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "angular_velocity": [0.0, 0.0, 0.0],
        }

    def getOdometryTimestamp(self):
        return 1234

    def getRobotVersion(self):
        return (1, 0, 15, 90561076, 20260828, 152313)


class ReadOnlyProbeTest(unittest.TestCase):
    def test_collect_sample_tolerates_unavailable_sdk_methods(self) -> None:
        sample = collect_sample(FakeRobotState())
        self.assertTrue(sample["connected"])
        self.assertEqual(12, len(sample["joint_position"]))
        self.assertEqual([1.0, 2.0, 3.0], sample["leg_odometry"]["position"])
        self.assertIn("imu_rpy", sample["errors"])

    def test_invalid_probe_args_are_rejected(self) -> None:
        args = argparse.Namespace(
            robot_state_port=9873,
            tcp_local_port=0,
            state_rate_hz=201,
            connect_timeout_s=10.0,
            duration_s=10.0,
            sample_period_s=0.1,
        )
        with self.assertRaisesRegex(ValueError, "state-rate-hz"):
            validate_args(args)


if __name__ == "__main__":
    unittest.main()
