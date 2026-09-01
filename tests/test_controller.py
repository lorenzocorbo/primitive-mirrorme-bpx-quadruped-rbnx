from __future__ import annotations

import math
import os
import sys
import unittest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.controller import QuadrupedController
from bpx_quadruped.fake_backend import FakeBpxBackend
from bpx_quadruped.model import (
    ControllerConfig,
    LifecycleState,
    PlanarTwist,
    SafetyState,
    StopCommand,
    VelocityCommand,
)


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 100.0

    def __call__(self) -> float:
        return self.now_s

    def advance(self, seconds: float) -> None:
        self.now_s += seconds


def make_controller(allow_motion: bool = True):
    clock = FakeClock()
    backend = FakeBpxBackend(clock)
    config = ControllerConfig(
        allow_motion=allow_motion,
        command_timeout_s=0.25,
        state_timeout_s=0.50,
        max_linear_x_mps=0.30,
        max_linear_y_mps=0.20,
        max_angular_z_rps=0.50,
        max_linear_accel_mps2=1.0,
        max_angular_accel_rps2=2.0,
        zero_preamble_count=3,
    )
    return clock, backend, QuadrupedController(backend, config, clock)


class QuadrupedControllerTest(unittest.TestCase):
    def test_activation_is_disarmed_and_sends_zero_preamble(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        snapshot = controller.snapshot()
        self.assertEqual(LifecycleState.ACTIVE, snapshot.lifecycle)
        self.assertEqual(SafetyState.DISARMED, snapshot.safety)
        self.assertEqual(3, len(backend.commands))
        self.assertTrue(all(isinstance(item, StopCommand) for item in backend.commands))

    def test_motion_disabled_cannot_arm(self) -> None:
        _, _, controller = make_controller(allow_motion=False)
        controller.activate()
        decision = controller.arm()
        self.assertFalse(decision.accepted)
        self.assertEqual(SafetyState.DISARMED, controller.snapshot().safety)

    def test_disarmed_twist_is_rejected_and_zeroed(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        decision = controller.submit_twist(PlanarTwist(0.1, 0.0, 0.0))
        self.assertFalse(decision.accepted)
        self.assertIsInstance(backend.commands[-1], StopCommand)

    def test_speed_and_acceleration_are_bounded(self) -> None:
        clock, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        self.assertTrue(controller.submit_twist(PlanarTwist(9.0, -9.0, 9.0)).accepted)
        clock.advance(0.1)
        controller.tick()
        command = backend.commands[-1]
        self.assertIsInstance(command, VelocityCommand)
        assert isinstance(command, VelocityCommand)
        self.assertAlmostEqual(0.1, command.twist.linear_x_mps)
        self.assertAlmostEqual(-0.1, command.twist.linear_y_mps)
        self.assertAlmostEqual(0.2, command.twist.angular_z_rps)
        clock.advance(0.3)
        controller.submit_twist(PlanarTwist(9.0, -9.0, 9.0))
        controller.tick()
        command = backend.commands[-1]
        assert isinstance(command, VelocityCommand)
        self.assertAlmostEqual(0.3, command.twist.linear_x_mps)
        self.assertAlmostEqual(-0.2, command.twist.linear_y_mps)
        self.assertAlmostEqual(0.5, command.twist.angular_z_rps)

    def test_command_watchdog_returns_to_zero(self) -> None:
        clock, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        controller.submit_twist(PlanarTwist(0.2, 0.0, 0.0))
        clock.advance(0.2)
        controller.tick()
        clock.advance(0.3)
        controller.tick()
        command = backend.commands[-1]
        self.assertIsInstance(command, StopCommand)
        self.assertTrue(controller.snapshot().last_output.is_zero())

    def test_stale_state_latches_fault_and_stops(self) -> None:
        clock, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        backend.set_state_frozen(True)
        clock.advance(0.6)
        controller.tick()
        snapshot = controller.snapshot()
        self.assertEqual(SafetyState.FAULT, snapshot.safety)
        self.assertIn("state watchdog", snapshot.fault_reason)
        self.assertIsInstance(backend.commands[-1], StopCommand)

    def test_disconnect_latches_fault_and_stops(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        backend.set_connected(False)
        controller.tick()
        self.assertEqual(SafetyState.FAULT, controller.snapshot().safety)
        self.assertIsInstance(backend.commands[-1], StopCommand)

    def test_reconnect_requires_clear_and_rearm_without_replaying_old_twist(self) -> None:
        clock, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        self.assertTrue(controller.submit_twist(PlanarTwist(0.2, 0.0, 0.0)).accepted)
        clock.advance(0.1)
        controller.tick()
        self.assertFalse(controller.snapshot().last_output.is_zero())

        backend.set_connected(False)
        controller.tick()
        self.assertEqual(SafetyState.FAULT, controller.snapshot().safety)
        backend.set_connected(True)

        self.assertTrue(controller.clear_fault().accepted)
        self.assertEqual(SafetyState.DISARMED, controller.snapshot().safety)
        controller.tick()
        self.assertIsInstance(backend.commands[-1], StopCommand)

        self.assertTrue(controller.arm().accepted)
        controller.tick()
        command = backend.commands[-1]
        self.assertIsInstance(command, VelocityCommand)
        assert isinstance(command, VelocityCommand)
        self.assertTrue(command.twist.is_zero())

    def test_repeated_deactivate_and_reactivate_starts_disarmed(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        controller.deactivate()
        controller.deactivate()
        controller.activate()
        self.assertEqual(2, backend.start_count)
        self.assertEqual(1, backend.stop_count)
        self.assertEqual(SafetyState.DISARMED, controller.snapshot().safety)

    def test_non_finite_twist_latches_fault(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        self.assertTrue(controller.arm().accepted)
        decision = controller.submit_twist(PlanarTwist(math.nan, 0.0, 0.0))
        self.assertFalse(decision.accepted)
        self.assertEqual(SafetyState.FAULT, controller.snapshot().safety)
        self.assertIsInstance(backend.commands[-1], StopCommand)

    def test_shutdown_is_idempotent_and_stops_backend(self) -> None:
        _, backend, controller = make_controller()
        controller.activate()
        controller.shutdown()
        first_stop_count = backend.stop_count
        controller.shutdown()
        self.assertEqual(first_stop_count, backend.stop_count)
        self.assertEqual(LifecycleState.SHUTDOWN, controller.snapshot().lifecycle)
        self.assertIsInstance(backend.commands[-1], StopCommand)


if __name__ == "__main__":
    unittest.main()
