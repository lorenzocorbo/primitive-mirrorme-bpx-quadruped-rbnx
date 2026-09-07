"""Explicit physical-motion probe for one stand/hold/sit cycle."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from typing import Iterable, Optional

from .sdk_posture import (
    PostureCycleConfig,
    SdkPostureConfig,
    SdkPostureSession,
    run_posture_cycle,
)
from .sdk_state_source import SdkStateConfig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Execute one physical BPX stand/hold/sit cycle through "
            "MotionLevelControl. No velocity, zero-position, gait, damping, "
            "or joint-level command is available."
        )
    )
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--robot-state-port", type=int, default=9873)
    parser.add_argument("--tcp-local-port", type=int, default=0)
    parser.add_argument("--state-rate-hz", type=int, default=50)
    parser.add_argument("--motion-command-rate-hz", type=int, default=50)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--stand-timeout-s", type=float, default=15.0)
    parser.add_argument("--hold-duration-s", type=float, default=3.0)
    parser.add_argument("--sit-timeout-s", type=float, default=15.0)
    parser.add_argument("--poll-period-s", type=float, default=0.2)
    parser.add_argument(
        "--confirm-physical-motion",
        action="store_true",
        help="required acknowledgement that the site and physical E-stop are ready",
    )
    return parser


def run(args: argparse.Namespace, *, stop_event: threading.Event) -> int:
    if not args.confirm_physical_motion:
        raise ValueError("--confirm-physical-motion is required")

    posture_config = SdkPostureConfig(
        state=SdkStateConfig(
            robot_ip=args.robot_ip,
            robot_state_port=args.robot_state_port,
            tcp_local_port=args.tcp_local_port,
            state_rate_hz=args.state_rate_hz,
            connect_timeout_s=args.connect_timeout_s,
            poll_period_s=min(args.poll_period_s, 0.2),
        ),
        motion_command_rate_hz=args.motion_command_rate_hz,
    )
    cycle_config = PostureCycleConfig(
        stand_timeout_s=args.stand_timeout_s,
        hold_duration_s=args.hold_duration_s,
        sit_timeout_s=args.sit_timeout_s,
        poll_period_s=args.poll_period_s,
    )
    posture_config.validate()
    cycle_config.validate()

    print(
        "BPX PHYSICAL POSTURE TEST: robot={} state_port={} hold={:.3f}s".format(
            args.robot_ip, args.robot_state_port, args.hold_duration_s
        ),
        flush=True,
    )
    print(
        "command boundary: MotionLevelControl setStandUp/setSitDown only; "
        "no setVelocity, zero-position, gait, damping, or JointLevelControl",
        flush=True,
    )
    session = SdkPostureSession(posture_config)
    run_posture_cycle(
        session,
        cycle_config,
        stop_requested=stop_event.is_set,
        report=lambda message: print(message, flush=True),
    )
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    stop_event = threading.Event()
    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    try:
        return run(args, stop_event=stop_event)
    except InterruptedError as exc:
        print("posture probe interrupted: {}".format(exc), file=sys.stderr)
        return 130
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print("posture probe failed: {}".format(exc), file=sys.stderr)
        return 2
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
