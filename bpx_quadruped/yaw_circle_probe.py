"""CLI for one explicitly confirmed physical BPX yaw-circle test."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
from typing import Iterable, Optional

from .sdk_posture import SdkPostureConfig
from .sdk_state_source import SdkStateConfig
from .sdk_yaw_circle import SdkYawCircleSession, YawCircleConfig, run_yaw_circle


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Execute one physical BPX stand/yaw-circle/stop/sit cycle. This is "
            "a manual test, not a Robonix twist_in provider."
        )
    )
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--robot-state-port", type=int, default=9873)
    parser.add_argument("--tcp-local-port", type=int, default=0)
    parser.add_argument("--state-rate-hz", type=int, default=50)
    parser.add_argument("--motion-command-rate-hz", type=int, default=50)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--stand-timeout-s", type=float, default=15.0)
    parser.add_argument("--sit-timeout-s", type=float, default=15.0)
    parser.add_argument("--yaw-rate-rps", type=float, default=0.5)
    parser.add_argument("--turn-angle-rad", type=float, default=2.0 * math.pi)
    parser.add_argument("--turn-timeout-s", type=float, default=30.0)
    parser.add_argument("--max-translation-m", type=float, default=0.15)
    parser.add_argument(
        "--opposite-direction-tolerance-rad", type=float, default=0.2
    )
    parser.add_argument("--zero-flush-s", type=float, default=2.0)
    parser.add_argument("--poll-period-s", type=float, default=0.1)
    parser.add_argument(
        "--confirm-physical-yaw-circle",
        action="store_true",
        help="required acknowledgement that the site and physical E-stop are ready",
    )
    return parser


def run(args: argparse.Namespace, *, stop_event: threading.Event) -> int:
    if not args.confirm_physical_yaw_circle:
        raise ValueError("--confirm-physical-yaw-circle is required")

    sdk_config = SdkPostureConfig(
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
    cycle_config = YawCircleConfig(
        yaw_rate_rps=args.yaw_rate_rps,
        turn_angle_rad=args.turn_angle_rad,
        turn_timeout_s=args.turn_timeout_s,
        max_translation_m=args.max_translation_m,
        opposite_direction_tolerance_rad=args.opposite_direction_tolerance_rad,
        zero_flush_s=args.zero_flush_s,
        stand_timeout_s=args.stand_timeout_s,
        sit_timeout_s=args.sit_timeout_s,
        poll_period_s=args.poll_period_s,
    )
    sdk_config.validate()
    cycle_config.validate()

    print(
        "BPX PHYSICAL YAW-CIRCLE TEST: robot={} rate={:.3f}rad/s "
        "angle={:.6f}rad max_translation={:.3f}m "
        "opposite_tolerance={:.3f}rad".format(
            args.robot_ip,
            cycle_config.yaw_rate_rps,
            cycle_config.turn_angle_rad,
            cycle_config.max_translation_m,
            cycle_config.opposite_direction_tolerance_rad,
        ),
        flush=True,
    )
    print(
        "command boundary: stand/sit plus setVelocity(0, 0, yaw) only; no "
        "linear velocity, zero-position, gait, damping, or JointLevelControl",
        flush=True,
    )
    session = SdkYawCircleSession(sdk_config)
    run_yaw_circle(
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
        print("yaw-circle probe interrupted: {}".format(exc), file=sys.stderr)
        return 130
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print("yaw-circle probe failed: {}".format(exc), file=sys.stderr)
        return 2
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
