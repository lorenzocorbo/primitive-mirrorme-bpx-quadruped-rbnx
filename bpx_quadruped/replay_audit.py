"""Offline telemetry quality report for normalized BPX replay files."""

from __future__ import annotations

import argparse
import json
from statistics import median
from typing import Any, Dict, Iterable, Optional, Sequence

from .replay import ReplayFrame, load_replay


def audit_frames(frames: Sequence[ReplayFrame]) -> Dict[str, Any]:
    if not frames:
        raise ValueError("at least one replay frame is required")
    offsets = [frame.offset_s for frame in frames]
    host_gaps = [right - left for left, right in zip(offsets, offsets[1:])]
    timestamps = [
        frame.state.vendor_timestamps.odometry_ms
        for frame in frames
        if frame.state.vendor_timestamps.odometry_ms is not None
    ]
    timestamp_deltas = [
        right - left for left, right in zip(timestamps, timestamps[1:])
    ]
    positive_deltas = [delta for delta in timestamp_deltas if delta > 0]
    nominal_delta_ms = median(positive_deltas) if positive_deltas else None
    estimated_missing = 0
    if nominal_delta_ms:
        estimated_missing = sum(
            max(0, round(delta / nominal_delta_ms) - 1)
            for delta in positive_deltas
        )
    duration_s = offsets[-1] - offsets[0]
    return {
        "schema": "mirrorme-bpx-replay-audit",
        "frame_count": len(frames),
        "duration_s": duration_s,
        "event_rate_hz": (
            (len(frames) - 1) / duration_s if duration_s > 0.0 else None
        ),
        "connected_frame_count": sum(frame.state.connected for frame in frames),
        "disconnect_event_count": sum(
            not frame.state.connected for frame in frames
        ),
        "max_host_event_gap_s": max(host_gaps) if host_gaps else None,
        "odometry_timestamp_count": len(timestamps),
        "odometry_timestamp_duplicate_count": sum(
            delta == 0 for delta in timestamp_deltas
        ),
        "odometry_timestamp_regression_count": sum(
            delta < 0 for delta in timestamp_deltas
        ),
        "nominal_odometry_delta_ms": nominal_delta_ms,
        "estimated_missing_odometry_frames": estimated_missing,
        "coverage": {
            "imu": sum(bool(frame.state.imu_orientation_xyzw) for frame in frames),
            "joint_position": sum(
                bool(frame.state.joint_position_rad) for frame in frames
            ),
            "battery": sum(
                frame.state.battery_level_percent is not None for frame in frames
            ),
        },
    }


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("replay")
    args = parser.parse_args(argv)
    report = audit_frames(load_replay(args.replay))
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
