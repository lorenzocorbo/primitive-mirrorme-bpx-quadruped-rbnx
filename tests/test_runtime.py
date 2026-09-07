from __future__ import annotations

import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import PlanarTwist, RobotState
from bpx_quadruped.config import ProviderConfig
from bpx_quadruped.posture_runtime import PostureRuntime
from bpx_quadruped.runtime import ReadOnlyRuntime, build_runtime, state_is_publishable


class FakeStateSource:
    def __init__(self, state: RobotState) -> None:
        self.state = state
        self.start_count = 0
        self.stop_count = 0

    @property
    def name(self) -> str:
        return "fake-read-only-source"

    def start(self) -> None:
        self.start_count += 1

    def stop(self) -> None:
        self.stop_count += 1

    def read_state(self) -> RobotState:
        return self.state


class ReadOnlyRuntimeTest(unittest.TestCase):
    def test_lifecycle_and_sampling_delegate_to_the_state_source(self) -> None:
        state = RobotState(connected=True, received_at_s=10.0)
        source = FakeStateSource(state)
        runtime = ReadOnlyRuntime(source)

        self.assertFalse(runtime.supports_twist)
        self.assertFalse(runtime.supports_posture)
        runtime.activate()
        runtime.activate()
        self.assertEqual(1, source.start_count)
        self.assertIs(state, runtime.tick())

        runtime.deactivate()
        runtime.shutdown()
        self.assertEqual(1, source.stop_count)

    def test_read_only_runtime_rejects_arm_and_twist_without_an_apply_seam(self) -> None:
        source = FakeStateSource(RobotState(connected=True, received_at_s=10.0))
        runtime = ReadOnlyRuntime(source)

        self.assertFalse(runtime.arm().accepted)
        decision = runtime.submit_twist(PlanarTwist(0.1, 0.0, 0.0))
        self.assertFalse(decision.accepted)
        self.assertIn("read-only", decision.reason)
        self.assertFalse(hasattr(source, "apply"))
        self.assertFalse(runtime.set_posture("stand").accepted)

    def test_tick_requires_an_active_runtime(self) -> None:
        runtime = ReadOnlyRuntime(
            FakeStateSource(RobotState(connected=True, received_at_s=10.0))
        )
        with self.assertRaisesRegex(RuntimeError, "not active"):
            runtime.tick()


class StatePublicationGateTest(unittest.TestCase):
    def test_only_connected_fresh_monotonic_samples_are_publishable(self) -> None:
        valid = RobotState(connected=True, received_at_s=9.8)
        self.assertTrue(state_is_publishable(valid, now_s=10.0, timeout_s=0.5))

        invalid = (
            RobotState(connected=False, received_at_s=9.8),
            RobotState(connected=True, received_at_s=9.0),
            RobotState(connected=True, received_at_s=10.1),
            RobotState(connected=True, received_at_s=math.nan),
        )
        for state in invalid:
            with self.subTest(state=state):
                self.assertFalse(
                    state_is_publishable(state, now_s=10.0, timeout_s=0.5)
                )


class RuntimeFactoryTest(unittest.TestCase):
    def test_fake_selects_the_controlled_adapter(self) -> None:
        runtime = build_runtime(ProviderConfig.from_mapping({}))
        self.assertTrue(runtime.supports_twist)

    def test_sdk_selects_the_read_only_adapter_without_importing_the_wheel(self) -> None:
        runtime = build_runtime(
            ProviderConfig.from_mapping({"backend": "sdk"}),
            sdk_request_factory=lambda: object(),
        )
        self.assertIsInstance(runtime, ReadOnlyRuntime)
        self.assertFalse(runtime.supports_twist)

    def test_sdk_posture_profile_selects_single_owner_posture_runtime(self) -> None:
        runtime = build_runtime(
            ProviderConfig.from_mapping(
                {
                    "backend": "sdk",
                    "allow_motion": True,
                    "enable_posture_service": True,
                }
            ),
            sdk_control_factory=lambda: object(),
        )
        self.assertIsInstance(runtime, PostureRuntime)
        self.assertTrue(runtime.supports_posture)
        self.assertFalse(runtime.supports_twist)

    def test_replay_selects_the_read_only_adapter(self) -> None:
        fixture = os.path.join(PACKAGE_ROOT, "tests", "fixtures", "replay_nominal.jsonl")
        runtime = build_runtime(
            ProviderConfig.from_mapping(
                {"backend": "replay", "replay_path": fixture}
            )
        )
        self.assertIsInstance(runtime, ReadOnlyRuntime)
        self.assertFalse(runtime.supports_twist)


if __name__ == "__main__":
    unittest.main()
