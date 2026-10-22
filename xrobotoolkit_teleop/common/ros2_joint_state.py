"""Read one Piper arm joint sample from an optional ROS 2 JointState topic."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence


PIPER_TOPIC_ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(6))


class Ros2JointStateError(RuntimeError):
    """Raised when a usable Piper joint sample cannot be obtained."""


@dataclass(frozen=True)
class PiperJointStateSample:
    """One name-ordered arm sample extracted from sensor_msgs/JointState."""

    topic: str
    source_joint_names: tuple[str, ...]
    joint_angles_rad: tuple[float, ...]
    gripper_position_m: float | None
    timestamp_ns: int


def parse_piper_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    *,
    topic: str,
    timestamp_ns: int = 0,
) -> PiperJointStateSample:
    """Extract arm joints by name and preserve an optional joint6 gripper position."""
    source_names = tuple(str(name) for name in names)
    try:
        source_positions = tuple(float(value) for value in positions)
    except (TypeError, ValueError) as exc:
        raise Ros2JointStateError(f"{topic}: JointState.position is not numeric: {exc}") from exc

    if len(source_names) != len(source_positions):
        raise Ros2JointStateError(
            f"{topic}: JointState name/position lengths differ "
            f"({len(source_names)} != {len(source_positions)})"
        )
    if len(set(source_names)) != len(source_names):
        raise Ros2JointStateError(f"{topic}: JointState contains duplicate joint names")

    positions_by_name = dict(zip(source_names, source_positions))
    missing = [name for name in PIPER_TOPIC_ARM_JOINT_NAMES if name not in positions_by_name]
    if missing:
        raise Ros2JointStateError(f"{topic}: missing required arm joints: {missing}")

    joint_angles = tuple(positions_by_name[name] for name in PIPER_TOPIC_ARM_JOINT_NAMES)
    if not all(math.isfinite(value) for value in joint_angles):
        raise Ros2JointStateError(f"{topic}: arm joint positions contain NaN or Inf")
    gripper_position_m = positions_by_name.get("joint6")
    if gripper_position_m is not None and not math.isfinite(gripper_position_m):
        raise Ros2JointStateError(f"{topic}: gripper joint6 position contains NaN or Inf")

    return PiperJointStateSample(
        topic=topic,
        source_joint_names=PIPER_TOPIC_ARM_JOINT_NAMES,
        joint_angles_rad=joint_angles,
        gripper_position_m=gripper_position_m,
        timestamp_ns=int(timestamp_ns),
    )


def read_piper_joint_state_once(
    topic: str,
    timeout_s: float = 5.0,
) -> PiperJointStateSample:
    """Subscribe to a ROS 2 JointState topic and return the first valid arm sample."""
    if not topic.startswith("/"):
        raise ValueError("ROS 2 joint topic must be an absolute name beginning with '/'")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("ROS 2 joint-state timeout must be positive")

    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise Ros2JointStateError(
            "ROS 2 Python packages are unavailable. Source the ROS 2 environment before "
            "using --initial-joint-topic."
        ) from exc

    context = rclpy.Context()
    node = None
    executor = None
    samples: list[PiperJointStateSample] = []
    errors: list[Ros2JointStateError] = []
    try:
        rclpy.init(args=None, context=context)
        node = rclpy.create_node("quest3_piper_initial_joint_reader", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        def callback(message: JointState) -> None:
            try:
                timestamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                    message.header.stamp.nanosec
                )
                samples.append(
                    parse_piper_joint_state(
                        message.name,
                        message.position,
                        topic=topic,
                        timestamp_ns=timestamp_ns,
                    )
                )
            except Ros2JointStateError as exc:
                errors.append(exc)

        subscription = node.create_subscription(JointState, topic, callback, qos)
        deadline = time.monotonic() + timeout_s
        while not samples and not errors:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                raise Ros2JointStateError(
                    f"Timed out after {timeout_s:g}s waiting for a JointState on {topic}"
                )
            executor.spin_once(timeout_sec=min(remaining_s, 0.1))

        # Keep the subscription alive until after the spin loop.
        _ = subscription
        if errors:
            raise errors[0]
        return samples[0]
    except Ros2JointStateError:
        raise
    except Exception as exc:
        raise Ros2JointStateError(f"Failed to read ROS 2 topic {topic}: {exc}") from exc
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
