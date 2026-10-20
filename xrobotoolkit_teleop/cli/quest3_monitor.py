"""Continuously print raw Quest 3 tracking and controller input state."""

from __future__ import annotations

import argparse
import sys
import time

from xrobotoolkit_teleop.common.quest3_reader import (
    POSE_ORDER,
    POSITION_UNIT,
    QUATERNION_ORDER,
    XR_COORDINATE_SYSTEM,
    Quest3ReadError,
    Quest3Reader,
    format_snapshot,
)
from xrobotoolkit_teleop.common.xr_client import (
    XrClient,
    XrSdkInitializationError,
    XrSdkUnavailableError,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only Quest 3 state monitor using the XRoboToolkit Python SDK."
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Console sampling rate (default: 20 Hz).",
    )
    parser.add_argument(
        "--stale-timeout-s",
        type=float,
        default=0.5,
        help="Mark tracking stale after this many seconds without a timestamp update (default: 0.5).",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Stop after this duration; omit to run until Ctrl-C.",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.stale_timeout_s <= 0:
        parser.error("--stale-timeout-s must be positive")
    if args.duration_s is not None and args.duration_s <= 0:
        parser.error("--duration-s must be positive")


def _print_conventions() -> None:
    print("Quest 3 raw data conventions")
    print(f"  pose order: {POSE_ORDER}")
    print(f"  quaternion order: {QUATERNION_ORDER}")
    print(f"  position unit: {POSITION_UNIT}")
    print(f"  coordinate system: {XR_COORDINATE_SYSTEM}")
    print("  tracking validity: inferred from SDK timestamp freshness and numeric pose validation")
    print()


def run(args: argparse.Namespace) -> int:
    period_s = 1.0 / args.rate_hz
    _print_conventions()

    try:
        with XrClient() as xr_client:
            reader = Quest3Reader(xr_client, stale_timeout_s=args.stale_timeout_s)
            started_at = time.monotonic()
            while args.duration_s is None or time.monotonic() - started_at < args.duration_s:
                cycle_started_at = time.monotonic()
                try:
                    snapshot = reader.read()
                    print(format_snapshot(snapshot), flush=True)
                except Quest3ReadError as exc:
                    print(f"XR read error: {exc}", file=sys.stderr, flush=True)

                sleep_s = period_s - (time.monotonic() - cycle_started_at)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except (XrSdkUnavailableError, XrSdkInitializationError) as exc:
        print(f"Cannot start Quest 3 monitor: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nQuest 3 monitor stopped by user.")

    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
