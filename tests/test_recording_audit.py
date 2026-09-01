from __future__ import annotations

import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import RobotState, VendorTimestamps
from bpx_quadruped.recording import record_source
from bpx_quadruped.replay_audit import audit_frames


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 10.0

    def __call__(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += seconds


class ChangingSource:
    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.started = False
        self.stopped = False

    @property
    def name(self) -> str:
        return "changing"

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def read_state(self) -> RobotState:
        tick = round((self.clock.now_s - 10.0) * 10)
        return RobotState(
            connected=True,
            received_at_s=self.clock.now_s,
            position_m=(tick / 10.0, 0.0, 0.0),
            joint_position_rad=(0.0,) * 12,
            vendor_timestamps=VendorTimestamps(
                odometry_ms=1000 + tick * 100,
                joint_ms=1000 + tick * 100,
            ),
        )


class RecordingAuditTest(unittest.TestCase):
    def test_records_changed_states_and_always_stops_source(self) -> None:
        clock = FakeClock()
        source = ChangingSource(clock)
        frames = record_source(
            source,
            duration_s=0.3,
            sample_period_s=0.1,
            clock=clock,
            sleep=clock.sleep,
        )
        self.assertTrue(source.started)
        self.assertTrue(source.stopped)
        self.assertEqual(4, len(frames))
        self.assertEqual(0.0, frames[0].offset_s)
        self.assertAlmostEqual(0.3, frames[-1].offset_s)

    def test_audit_reports_timestamp_regressions_disconnects_and_missing_frames(self) -> None:
        clock = FakeClock()
        frames = list(
            record_source(
                ChangingSource(clock),
                duration_s=0.3,
                sample_period_s=0.1,
                clock=clock,
                sleep=clock.sleep,
            )
        )
        second = frames[1].state
        frames[1] = type(frames[1])(
            frames[1].offset_s,
            RobotState(
                connected=False,
                received_at_s=second.received_at_s,
                vendor_timestamps=VendorTimestamps(odometry_ms=900),
            ),
        )
        report = audit_frames(frames)
        self.assertEqual(1, report["disconnect_event_count"])
        self.assertEqual(1, report["odometry_timestamp_regression_count"])
        self.assertGreaterEqual(report["estimated_missing_odometry_frames"], 0)


if __name__ == "__main__":
    unittest.main()
