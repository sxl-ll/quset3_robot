"""Run the Quest 3 to Piper IK pipeline with optional guarded ROS 2 output."""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from xrobotoolkit_teleop.common.quest3_reader import Quest3ReadError, Quest3Reader
from xrobotoolkit_teleop.common.ros2_joint_command import (
    JointCommandRejected,
    Ros2JointCommandError,
    Ros2JointCommandPublisher,
)
from xrobotoolkit_teleop.common.ros2_joint_state import (
    Ros2JointStateError,
    read_piper_joint_state_once,
)
from xrobotoolkit_teleop.common.teleop_pipeline import (
    Quest3PiperPipeline,
    format_pipeline_result,
)
from xrobotoolkit_teleop.common.xr_client import (
    XrClient,
    XrSdkInitializationError,
    XrSdkUnavailableError,
)
from xrobotoolkit_teleop.robot.piper_model import PiperModelError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read Quest 3 right-controller motion and solve Piper IK. By default results "
            "are printed only; --publish-joint-topic explicitly enables ROS 2 commands."
        )
    )
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--stale-timeout-s", type=float, default=0.5)
    parser.add_argument(
        "--position-scale",
        type=float,
        default=1.0,
        help=(
            "Multiplier from measured XR position deltas to robot metres (default: 1.0; "
            "verify the SDK position unit on the target setup first)."
        ),
    )
    parser.add_argument("--smoothing-alpha", type=float, default=0.25)
    parser.add_argument("--grip-threshold", type=float, default=0.9)
    parser.add_argument(
        "--initial-joint-topic",
        default=None,
        help=(
            "Optional ROS 2 sensor_msgs/JointState topic read once at startup. "
            "For /puppet/joint_right, joint0..joint5 initialize the six Piper arm joints; "
            "joint6 is retained as the gripper position."
        ),
    )
    parser.add_argument("--initial-joint-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--initial-joint-limit-tolerance-rad",
        type=float,
        default=0.01,
        help=(
            "Maximum measured startup limit deviation accepted from the ROS topic "
            "(default: 0.01 rad). IK results still use strict URDF limits."
        ),
    )
    parser.add_argument(
        "--publish-joint-topic",
        default=None,
        help=(
            "Explicitly enable real joint-command publishing to a ROS 2 "
            "sensor_msgs/JointState topic, normally /joint_right_states."
        ),
    )
    parser.add_argument("--command-subscriber-timeout-s", type=float, default=5.0)
    parser.add_argument("--max-command-joint-step-rad", type=float, default=0.05)
    parser.add_argument("--max-command-joint-speed-rad-s", type=float, default=0.5)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.duration_s is not None and args.duration_s <= 0:
        parser.error("--duration-s must be positive")
    if args.stale_timeout_s <= 0:
        parser.error("--stale-timeout-s must be positive")
    if args.position_scale <= 0:
        parser.error("--position-scale must be positive")
    if not 0 < args.smoothing_alpha <= 1:
        parser.error("--smoothing-alpha must be in (0, 1]")
    if not 0 <= args.grip_threshold <= 1:
        parser.error("--grip-threshold must be in [0, 1]")
    if not np.isfinite(args.initial_joint_timeout_s) or args.initial_joint_timeout_s <= 0:
        parser.error("--initial-joint-timeout-s must be positive")
    if (
        not np.isfinite(args.initial_joint_limit_tolerance_rad)
        or args.initial_joint_limit_tolerance_rad < 0
    ):
        parser.error("--initial-joint-limit-tolerance-rad must be non-negative")
    if args.publish_joint_topic and not args.initial_joint_topic:
        parser.error("--publish-joint-topic requires --initial-joint-topic")
    if args.publish_joint_topic == args.initial_joint_topic:
        parser.error("initial-state and command topics must be different")
    if (
        not np.isfinite(args.command_subscriber_timeout_s)
        or args.command_subscriber_timeout_s <= 0
    ):
        parser.error("--command-subscriber-timeout-s must be positive")
    if (
        not np.isfinite(args.max_command_joint_step_rad)
        or args.max_command_joint_step_rad <= 0
    ):
        parser.error("--max-command-joint-step-rad must be positive")
    if (
        not np.isfinite(args.max_command_joint_speed_rad_s)
        or args.max_command_joint_speed_rad_s <= 0
    ):
        parser.error("--max-command-joint-speed-rad-s must be positive")


def run(args: argparse.Namespace) -> int:
    initial_joint_sample = None
    if args.initial_joint_topic:
        print(f"Reading initial Piper joints from {args.initial_joint_topic} ...", flush=True)
        try:
            initial_joint_sample = read_piper_joint_state_once(
                args.initial_joint_topic,
                timeout_s=args.initial_joint_timeout_s,
            )
        except (Ros2JointStateError, ValueError) as exc:
            print(f"Cannot read initial Piper joint state: {exc}", file=sys.stderr)
            return 2

    initial_joint_angles_rad = (
        None
        if initial_joint_sample is None
        else np.asarray(initial_joint_sample.joint_angles_rad, dtype=np.float64)
    )
    try:
        pipeline = Quest3PiperPipeline.create(
            position_scale=args.position_scale,
            smoothing_alpha=args.smoothing_alpha,
            grip_threshold=args.grip_threshold,
            initial_joint_angles_rad=initial_joint_angles_rad,
            initial_joint_limit_tolerance_rad=(
                args.initial_joint_limit_tolerance_rad
                if initial_joint_sample is not None
                else 0.0
            ),
        )
    except (PiperModelError, ValueError, RuntimeError) as exc:
        print(f"Cannot initialize Piper IK: {exc}", file=sys.stderr)
        return 2

    command_publisher = None
    if args.publish_joint_topic:
        assert initial_joint_sample is not None
        if initial_joint_sample.gripper_position_m is None:
            print(
                "Cannot publish joint commands: initial JointState has no joint6 gripper value",
                file=sys.stderr,
            )
            return 2
        try:
            command_publisher = Ros2JointCommandPublisher(
                topic=args.publish_joint_topic,
                initial_joint_angles_rad=initial_joint_sample.joint_angles_rad,
                gripper_position_m=initial_joint_sample.gripper_position_m,
                max_joint_step_rad=args.max_command_joint_step_rad,
                max_joint_speed_rad_s=args.max_command_joint_speed_rad_s,
                subscriber_timeout_s=args.command_subscriber_timeout_s,
            )
        except (Ros2JointCommandError, JointCommandRejected, ValueError) as exc:
            print(f"Cannot initialize joint command publisher: {exc}", file=sys.stderr)
            return 2

    print("Quest 3 -> relative pose mapping -> Piper Placo IK -> joint angles")
    if initial_joint_sample is not None:
        joint_values = ", ".join(
            f"{name}={value:+.6f}"
            for name, value in zip(
                initial_joint_sample.source_joint_names,
                initial_joint_sample.joint_angles_rad,
            )
        )
        initial_pose = pipeline.mapper.target_pose
        position = ", ".join(f"{value:+.6f}" for value in initial_pose.position)
        orientation = ", ".join(
            f"{value:+.6f}" for value in initial_pose.orientation_xyzw
        )
        violations = pipeline.ik_solver.initial_joint_limit_violations_rad
        print(
            f"Initial joints read from {initial_joint_sample.topic} at "
            f"timestamp_ns={initial_joint_sample.timestamp_ns}."
        )
        print(f"  source_joint_angles_rad: {joint_values}")
        print(f"  initial_target_position=({position})")
        print(f"  initial_target_orientation_xyzw=({orientation})")
        if np.any(violations > 0.0):
            print(
                "  WARNING: measured initial joints exceed strict URDF limits by "
                f"{violations.tolist()} rad; their FK pose was preserved while the IK "
                "state was reconciled to strict limits."
            )
            reconciled_values = ", ".join(
                f"{value:+.6f}" for value in pipeline.ik_solver.joint_angles_rad
            )
            print(f"  reconciled_ik_joint_angles_rad=({reconciled_values})")
    if command_publisher is None:
        print("Safety: calculation-only mode; no physical robot commands are published.")
        print("Joint command publishing is disabled.")
    else:
        print(
            f"WARNING: REAL ROBOT joint commands are enabled on {command_publisher.topic}; "
            "the program does not enable the arm automatically."
        )
        print(
            f"  max_joint_step={args.max_command_joint_step_rad:g} rad "
            f"max_joint_speed={args.max_command_joint_speed_rad_s:g} rad/s "
            f"held_gripper={command_publisher.gripper_position_m:g} m"
        )
    print("Right Grip activates the clutch; release Grip to hold the target.")
    print(
        f"position_scale={args.position_scale:g}; verify the XR position unit and axis "
        "directions on the target Quest 3 setup before interpreting motion."
    )
    print()

    period_s = 1.0 / args.rate_hz
    try:
        with XrClient() as xr_client:
            reader = Quest3Reader(xr_client, stale_timeout_s=args.stale_timeout_s)
            started_at = time.monotonic()
            while args.duration_s is None or time.monotonic() - started_at < args.duration_s:
                cycle_started_at = time.monotonic()
                try:
                    snapshot = reader.read()
                    result = pipeline.process(snapshot)
                    print(format_pipeline_result(result), flush=True)
                    if (
                        command_publisher is not None
                        and result.ik is not None
                        and result.ik.success
                        and result.ik.joint_limits_valid
                        and result.ik.joint_angles_rad is not None
                    ):
                        command_publisher.publish(result.ik.joint_angles_rad)
                        print(
                            f"  joint_command=PUBLISHED topic={command_publisher.topic}",
                            flush=True,
                        )
                except Quest3ReadError as exc:
                    print(f"XR read error: {exc}", file=sys.stderr, flush=True)

                sleep_s = period_s - (time.monotonic() - cycle_started_at)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except JointCommandRejected as exc:
        print(
            f"Joint command rejected; stopping before another command is sent: {exc}",
            file=sys.stderr,
        )
        return 2
    except (XrSdkUnavailableError, XrSdkInitializationError, Ros2JointCommandError) as exc:
        print(f"Cannot start Quest 3 Piper IK pipeline: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nQuest 3 Piper IK pipeline stopped by user.")
    finally:
        if command_publisher is not None:
            command_publisher.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())

