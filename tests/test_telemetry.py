from __future__ import annotations

import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import RobotState, VendorTimestamps
from bpx_quadruped.telemetry import normalize_robot_state, state_for_publication


class TelemetryValidationTest(unittest.TestCase):
    def test_normalizes_odom_and_optional_imu_quaternions(self) -> None:
        state = RobotState(
            connected=True,
            received_at_s=10.0,
            orientation_xyzw=(0.0, 0.0, 0.0, 2.0),
            imu_orientation_xyzw=(0.0, 0.0, 3.0, 0.0),
        )
        normalized = normalize_robot_state(state)
        self.assertEqual((0.0, 0.0, 0.0, 1.0), normalized.orientation_xyzw)
        self.assertEqual((0.0, 0.0, 1.0, 0.0), normalized.imu_orientation_xyzw)

    def test_rejects_zero_quaternion_bad_dimensions_and_non_finite_values(self) -> None:
        invalid = (
            RobotState(
                connected=True,
                received_at_s=10.0,
                orientation_xyzw=(0.0, 0.0, 0.0, 0.0),
            ),
            RobotState(
                connected=True,
                received_at_s=10.0,
                joint_position_rad=(0.0,) * 11,
            ),
            RobotState(
                connected=True,
                received_at_s=10.0,
                linear_velocity_body_mps=(math.nan, 0.0, 0.0),
            ),
            RobotState(
                connected=True,
                received_at_s=10.0,
                vendor_timestamps=VendorTimestamps(odometry_ms=-1),
            ),
        )
        for state in invalid:
            with self.subTest(state=state):
                with self.assertRaises(ValueError):
                    normalize_robot_state(state)

    def test_publication_gate_returns_normalized_state_only_when_fresh(self) -> None:
        state = RobotState(
            connected=True,
            received_at_s=9.8,
            orientation_xyzw=(0.0, 0.0, 0.0, 2.0),
        )
        ready = state_for_publication(state, now_s=10.0, timeout_s=0.5)
        self.assertIsNotNone(ready)
        assert ready is not None
        self.assertEqual(1.0, ready.orientation_xyzw[3])

        self.assertIsNone(
            state_for_publication(state, now_s=10.5, timeout_s=0.5)
        )
        self.assertIsNone(
            state_for_publication(
                RobotState(connected=False, received_at_s=9.9),
                now_s=10.0,
                timeout_s=0.5,
            )
        )


if __name__ == "__main__":
    unittest.main()
