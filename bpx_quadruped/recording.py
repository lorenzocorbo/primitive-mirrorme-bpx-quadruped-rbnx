"""Bounded read-only recording from a ``BpxStateSource`` to replay JSONL."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import sys
import time
from typing import Callable, Iterable, Optional, Tuple

from .backend import BpxStateSource
from .replay import ReplayFrame, dump_replay
from .sdk_state_source import SdkStateConfig, SdkStateSource
from .telemetry import normalize_robot_state


def record_source(
    source: BpxStateSource,
    *,
    duration_s: float,
    sample_period_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> Tuple[ReplayFrame, ...]:
    """Record changed normalized states for a bounded duration."""

    _positive(duration_s, "duration_s")
    _positive(sample_period_s, "sample_period_s")
    frames = []
    source.start()
    try:
        first = normalize_robot_state(source.read_state())
        origin_s = first.received_at_s
        first = replace(first, received_at_s=0.0)
        frames.append(ReplayFrame(0.0, first))
        previous_key = _event_key(first)
        deadline_s = clock() + duration_s
        while clock() < deadline_s:
            remaining_s = deadline_s - clock()
            sleep(min(sample_period_s, max(0.0, remaining_s)))
            now_s = clock()
            state = normalize_robot_state(source.read_state())
            received_at_s = max(0.0, state.received_at_s - origin_s)
            relative = replace(state, received_at_s=received_at_s)
            key = _event_key(relative)
            if key == previous_key:
                continue
            offset_s = max(now_s - origin_s, received_at_s)
            if offset_s <= frames[-1].offset_s:
                offset_s = frames[-1].offset_s + 1e-9
            frames.append(ReplayFrame(offset_s, relative))
            previous_key = key
    finally:
        source.stop()
    return tuple(frames)


def _event_key(state):
    odometry_ms = state.vendor_timestamps.odometry_ms
    freshness_key = odometry_ms if odometry_ms is not None else state.received_at_s
    return (
        state.connected,
        freshness_key,
        state.vendor_timestamps.joint_ms,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record normalized BPX telemetry; never opens a motion session."
    )
    parser.add_argument("output")
    parser.add_argument("--robot-ip", default="10.21.20.1")
    parser.add_argument("--robot-state-port", type=int, default=9873)
    parser.add_argument("--tcp-local-port", type=int, default=0)
    parser.add_argument("--state-rate-hz", type=int, default=50)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--sample-period-s", type=float, default=0.01)
    return parser


def run(args: argparse.Namespace) -> int:
    output = Path(args.output)
    if output.exists():
        raise ValueError("refusing to overwrite existing recording: {}".format(output))
    source = SdkStateSource(
        SdkStateConfig(
            robot_ip=args.robot_ip,
            robot_state_port=args.robot_state_port,
            tcp_local_port=args.tcp_local_port,
            state_rate_hz=args.state_rate_hz,
            connect_timeout_s=args.connect_timeout_s,
            poll_period_s=min(0.05, args.sample_period_s),
        )
    )
    frames = record_source(
        source,
        duration_s=args.duration_s,
        sample_period_s=args.sample_period_s,
    )
    dump_replay(frames, output)
    print("recorded {} state events to {}".format(len(frames), output))
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except (ImportError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print("read-only recording failed: {}".format(exc), file=sys.stderr)
        return 2


def _positive(value: float, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be finite and > 0".format(name))
    if not math.isfinite(float(value)) or float(value) <= 0.0:
        raise ValueError("{} must be finite and > 0".format(name))


if __name__ == "__main__":
    raise SystemExit(main())
