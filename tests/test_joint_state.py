from __future__ import annotations

import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.joint_state import JOINT_NAMES, JointStateProjector
from bpx_quadruped.model import RobotState, VendorTimestamps


EXPECTED_JOINT_NAMES = (
    "fl_hip_roll_joint",
    "fl_hip_pitch_joint",
    "fl_knee_joint",
    "fr_hip_roll_joint",
    "fr_hip_pitch_joint",
    "fr_knee_joint",
    "hl_hip_roll_joint",
    "hl_hip_pitch_joint",
    "hl_knee_joint",
    "hr_hip_roll_joint",
    "hr_hip_pitch_joint",
    "hr_knee_joint",
)


def make_state(
    *,
    position=tuple(float(index) / 10.0 for index in range(12)),
    velocity=tuple(float(index) / 20.0 for index in range(12)),
    torque=tuple(float(index) / 30.0 for index in range(12)),
    timestamp=100,
):
    return RobotState(
        connected=True,
        received_at_s=10.0,
        joint_position_rad=position,
        joint_velocity_rad_s=velocity,
        joint_torque_nm=torque,
        vendor_timestamps=VendorTimestamps(joint_ms=timestamp),
    )


class JointStateProjectorTest(unittest.TestCase):
    def test_names_match_the_pinned_sdk_joint_index_contract(self) -> None:
        self.assertEqual(EXPECTED_JOINT_NAMES, JOINT_NAMES)

    def test_projects_position_and_velocity_but_not_unverified_effort(self) -> None:
        payload = JointStateProjector().project(make_state())
        self.assertIsNotNone(payload)
        assert payload is not None
        self.assertEqual(EXPECTED_JOINT_NAMES, payload.names)
        self.assertEqual(tuple(float(index) / 10.0 for index in range(12)), payload.position)
        self.assertEqual(tuple(float(index) / 20.0 for index in range(12)), payload.velocity)
        self.assertEqual((), payload.effort)

    def test_missing_position_suppresses_the_message(self) -> None:
        self.assertIsNone(JointStateProjector().project(make_state(position=())))

    def test_disconnected_state_is_not_published(self) -> None:
        state = make_state()
        state = RobotState(
            connected=False,
            received_at_s=state.received_at_s,
            joint_position_rad=state.joint_position_rad,
            joint_velocity_rad_s=state.joint_velocity_rad_s,
            vendor_timestamps=state.vendor_timestamps,
        )
        self.assertIsNone(JointStateProjector().project(state))

    def test_optional_velocity_may_be_absent(self) -> None:
        payload = JointStateProjector().project(make_state(velocity=()))
        self.assertIsNotNone(payload)
        assert payload is not None
        self.assertEqual((), payload.velocity)

    def test_wrong_length_or_non_finite_telemetry_is_rejected(self) -> None:
        invalid_states = (
            make_state(position=(0.0,) * 11),
            make_state(velocity=(0.0,) * 11),
            make_state(position=(math.nan,) + (0.0,) * 11),
        )
        for state in invalid_states:
            with self.subTest(state=state):
                with self.assertRaises(ValueError):
                    JointStateProjector().project(state)

    def test_duplicate_vendor_timestamp_is_not_republished(self) -> None:
        projector = JointStateProjector()
        self.assertIsNotNone(projector.project(make_state(timestamp=100)))
        self.assertIsNone(projector.project(make_state(timestamp=100)))
        self.assertIsNotNone(projector.project(make_state(timestamp=101)))

    def test_timestamp_wrap_or_reset_is_treated_as_a_new_sample(self) -> None:
        projector = JointStateProjector()
        self.assertIsNotNone(projector.project(make_state(timestamp=4294967295)))
        self.assertIsNotNone(projector.project(make_state(timestamp=0)))

    def test_timestamp_free_sources_publish_each_sample(self) -> None:
        projector = JointStateProjector()
        state = make_state(timestamp=None)
        self.assertIsNotNone(projector.project(state))
        self.assertIsNotNone(projector.project(state))

    def test_reset_allows_same_timestamp_after_reactivation(self) -> None:
        projector = JointStateProjector()
        state = make_state(timestamp=100)
        self.assertIsNotNone(projector.project(state))
        self.assertIsNone(projector.project(state))
        projector.reset()
        self.assertIsNotNone(projector.project(state))


if __name__ == "__main__":
    unittest.main()
