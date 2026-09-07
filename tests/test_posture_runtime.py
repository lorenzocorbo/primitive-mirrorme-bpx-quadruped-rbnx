from __future__ import annotations

import os
import sys
import threading
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import PlanarTwist, RobotState
from bpx_quadruped.posture_runtime import PostureRuntime, PostureRuntimeConfig
from bpx_quadruped.sdk_posture import (
    MOTION_STATE_LYING_DOWN,
    MOTION_STATE_MOTION,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeSession:
    def __init__(self, clock: FakeClock, motion_state: int = 0) -> None:
        self.clock = clock
        self.motion_state = motion_state
        self.started = False
        self.stopped = False
        self.stand_calls = 0
        self.sit_calls = 0
        self.fail_stand = False

    def start(self) -> None:
        self.started = True
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True

    def read_state(self) -> RobotState:
        return RobotState(
            connected=True,
            received_at_s=self.clock.now,
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


class BlockingStandSession(FakeSession):
    def __init__(self, clock: FakeClock) -> None:
        super().__init__(clock)
        self.command_entered = threading.Event()
        self.command_release = threading.Event()

    def request_stand_up(self) -> bool:
        self.stand_calls += 1
        self.command_entered.set()
        if not self.command_release.wait(timeout=1.0):
            raise RuntimeError("test did not release stand request")
        return True


class PostureRuntimeTest(unittest.TestCase):
    def make_runtime(self, session: FakeSession, clock: FakeClock) -> PostureRuntime:
        return PostureRuntime(
            session,
            PostureRuntimeConfig(
                stand_timeout_s=2.0,
                sit_timeout_s=2.0,
                poll_period_s=0.1,
                state_timeout_s=0.5,
                cleanup_sit_flush_s=0.2,
            ),
            clock=clock,
            sleep=clock.sleep,
        )

    def test_stand_and_sit_are_state_closed_and_idempotent(self) -> None:
        clock = FakeClock()
        session = FakeSession(clock)
        runtime = self.make_runtime(session, clock)

        runtime.activate()
        self.assertTrue(runtime.supports_posture)
        self.assertFalse(runtime.supports_twist)
        self.assertTrue(runtime.set_posture("sit").accepted)
        self.assertEqual(0, session.sit_calls)

        stand = runtime.set_posture("stand")
        self.assertTrue(stand.accepted, stand.reason)
        self.assertEqual(MOTION_STATE_MOTION, runtime.tick().motion_state)
        self.assertTrue(runtime.set_posture("stand").accepted)

        sit = runtime.set_posture("sit")
        self.assertTrue(sit.accepted, sit.reason)
        self.assertEqual(MOTION_STATE_LYING_DOWN, runtime.tick().motion_state)
        runtime.shutdown()
        self.assertTrue(session.stopped)

    def test_rejects_unknown_posture_twist_arm_and_concurrent_operation(self) -> None:
        clock = FakeClock()
        runtime = self.make_runtime(FakeSession(clock), clock)
        runtime.activate()

        self.assertFalse(runtime.set_posture("Stand").accepted)
        self.assertFalse(runtime.set_posture("damping").accepted)
        self.assertFalse(runtime.arm().accepted)
        self.assertFalse(runtime.submit_twist(PlanarTwist(0.1, 0.0, 0.0)).accepted)
        runtime._operation_lock.acquire()
        try:
            busy = runtime.set_posture("stand")
        finally:
            runtime._operation_lock.release()
        self.assertFalse(busy.accepted)
        self.assertIn("in progress", busy.reason)

    def test_activation_accepts_fresh_supported_state_without_moving(self) -> None:
        clock = FakeClock()
        session = FakeSession(clock, motion_state=MOTION_STATE_MOTION)
        runtime = self.make_runtime(session, clock)

        runtime.activate()
        self.assertTrue(session.started)
        self.assertEqual(0, session.stand_calls)
        self.assertEqual(0, session.sit_calls)

    def test_failed_stand_attempts_bounded_sit_cleanup(self) -> None:
        clock = FakeClock()
        session = FakeSession(clock)
        session.fail_stand = True
        runtime = self.make_runtime(session, clock)
        runtime.activate()

        decision = runtime.set_posture("stand")
        self.assertFalse(decision.accepted)
        self.assertIn("setStandUp returned false", decision.reason)
        self.assertGreater(session.sit_calls, 0)

    def test_deactivate_does_not_change_posture_before_disconnect(self) -> None:
        clock = FakeClock()
        session = FakeSession(clock)
        runtime = self.make_runtime(session, clock)
        runtime.activate()
        self.assertTrue(runtime.set_posture("stand").accepted)

        runtime.deactivate()

        self.assertEqual(MOTION_STATE_MOTION, session.motion_state)
        self.assertEqual(0, session.sit_calls)
        self.assertTrue(session.stopped)
        with self.assertRaisesRegex(RuntimeError, "not active"):
            runtime.tick()

    def test_deactivate_cancels_inflight_rpc_without_sending_sit(self) -> None:
        clock = FakeClock()
        session = BlockingStandSession(clock)
        runtime = self.make_runtime(session, clock)
        runtime.activate()
        result = {}

        command_thread = threading.Thread(
            target=lambda: result.setdefault("decision", runtime.set_posture("stand"))
        )
        command_thread.start()
        self.assertTrue(session.command_entered.wait(timeout=1.0))

        deactivate_thread = threading.Thread(target=runtime.deactivate)
        deactivate_thread.start()
        self.assertTrue(runtime._cancel.wait(timeout=1.0))
        session.command_release.set()
        command_thread.join(timeout=1.0)
        deactivate_thread.join(timeout=1.0)

        self.assertFalse(command_thread.is_alive())
        self.assertFalse(deactivate_thread.is_alive())
        self.assertFalse(result["decision"].accepted)
        self.assertIn("interrupted", result["decision"].reason)
        self.assertEqual(0, session.sit_calls)
        self.assertTrue(session.stopped)


if __name__ == "__main__":
    unittest.main()
