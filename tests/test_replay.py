from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from bpx_quadruped.model import RobotState, VendorTimestamps
from bpx_quadruped.replay import (
    ReplayFrame,
    ReplayStateSource,
    dump_replay,
    load_replay,
)


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 100.0

    def __call__(self) -> float:
        return self.now_s

    def advance(self, seconds: float) -> None:
        self.now_s += seconds


class ReplayStateSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.frames = (
            ReplayFrame(
                0.0,
                RobotState(
                    connected=True,
                    received_at_s=0.0,
                    position_m=(0.0, 0.0, 0.0),
                    orientation_xyzw=(0.0, 0.0, 0.0, 2.0),
                    vendor_timestamps=VendorTimestamps(odometry_ms=1000),
                ),
            ),
            ReplayFrame(
                0.1,
                RobotState(
                    connected=True,
                    received_at_s=0.1,
                    position_m=(0.1, 0.0, 0.0),
                    vendor_timestamps=VendorTimestamps(odometry_ms=1100),
                ),
            ),
            ReplayFrame(
                0.2,
                RobotState(
                    connected=False,
                    received_at_s=0.1,
                    position_m=(0.1, 0.0, 0.0),
                    vendor_timestamps=VendorTimestamps(odometry_ms=1100),
                ),
            ),
        )

    def test_uses_recorded_event_and_freshness_timelines(self) -> None:
        clock = FakeClock()
        source = ReplayStateSource(self.frames, clock=clock)
        with self.assertRaisesRegex(RuntimeError, "not started"):
            source.read_state()

        source.start()
        first = source.read_state()
        self.assertEqual((0.0, 0.0, 0.0), first.position_m)
        self.assertEqual(100.0, first.received_at_s)
        self.assertEqual(1.0, first.orientation_xyzw[3])

        clock.advance(0.15)
        second = source.read_state()
        self.assertEqual((0.1, 0.0, 0.0), second.position_m)
        self.assertAlmostEqual(100.1, second.received_at_s)

        clock.advance(0.10)
        disconnected = source.read_state()
        self.assertFalse(disconnected.connected)
        self.assertAlmostEqual(100.1, disconnected.received_at_s)

        source.stop()
        with self.assertRaisesRegex(RuntimeError, "not started"):
            source.read_state()

    def test_round_trip_is_deterministic_and_strict(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            dump_replay(self.frames, path)
            first_bytes = path.read_bytes()
            loaded = load_replay(path)
            dump_replay(loaded, path)
            self.assertEqual(first_bytes, path.read_bytes())
            self.assertEqual(self.frames[1].state.position_m, loaded[1].state.position_m)

    def test_rejects_unknown_schema_fields_and_invalid_timeline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.jsonl"
            path.write_text(
                '{"schema":"wrong","version":1}\n', encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "schema"):
                load_replay(path)

        with self.assertRaisesRegex(ValueError, "first replay"):
            ReplayStateSource((ReplayFrame(0.1, self.frames[0].state),))

        bad_state = RobotState(connected=True, received_at_s=0.2)
        with self.assertRaisesRegex(ValueError, "between 0"):
            ReplayStateSource((ReplayFrame(0.0, bad_state),))


if __name__ == "__main__":
    unittest.main()
