"""Guarded ROS 2 JointState publisher for the Piper right-arm command topic."""

from __future__ import annotations

import math
import time
from typing import Callable, Sequence

import numpy as np


PIPER_COMMAND_JOINT_NAMES = tuple(f"joint{index}" for index in range(7))


class Ros2JointCommandError(RuntimeError):
    """Raised when the ROS 2 command publisher cannot be created or used."""


class JointCommandRejected(RuntimeError):
    """Raised when a joint command fails a local safety check."""


class JointCommandLimiter:
    """Reject non-finite commands, large steps, and excessive command velocity."""

    def __init__(
        self,
        initial_joint_angles_rad: Sequence[float],
        max_joint_step_rad: float,
        max_joint_speed_rad_s: float,
        monotonic: Callable[[], float] = time.monotonic,
    ):
        if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
            raise ValueError("max_joint_step_rad must be positive")
        if not math.isfinite(max_joint_speed_rad_s) or max_joint_speed_rad_s <= 0:
            raise ValueError("max_joint_speed_rad_s must be positive")

        self.max_joint_step_rad = float(max_joint_step_rad)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self._monotonic = monotonic
        self._last_joint_angles_rad = self._parse_angles(initial_joint_angles_rad)
        self._last_command_monotonic_s = self._monotonic()

    @staticmethod
    def _parse_angles(joint_angles_rad: Sequence[float]) -> np.ndarray:
        try:
            angles = np.asarray(joint_angles_rad, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise JointCommandRejected(f"joint command is not numeric: {exc}") from exc
        if angles.shape != (6,) or not np.all(np.isfinite(angles)):
            raise JointCommandRejected("joint command must contain six finite radians values")
        return angles

    def check(self, joint_angles_rad: Sequence[float]) -> tuple[np.ndarray, float]:
        angles = self._parse_angles(joint_angles_rad)
        now_s = self._monotonic()
        elapsed_s = max(0.0, now_s - self._last_command_monotonic_s)
        delta = np.abs(angles - self._last_joint_angles_rad)
        largest_step = float(np.max(delta))
        if largest_step > self.max_joint_step_rad:
            raise JointCommandRejected(
                f"largest joint step {largest_step:.6f} rad exceeds "
                f"limit {self.max_joint_step_rad:.6f} rad"
            )

        speed_limit_step = self.max_joint_speed_rad_s * elapsed_s
        if largest_step > speed_limit_step:
            raise JointCommandRejected(
                f"largest joint step {largest_step:.6f} rad over {elapsed_s:.6f}s exceeds "
                f"speed limit {self.max_joint_speed_rad_s:.6f} rad/s"
            )
        return angles, now_s

    def accept(self, joint_angles_rad: np.ndarray, command_monotonic_s: float) -> None:
        self._last_joint_angles_rad = np.asarray(joint_angles_rad, dtype=np.float64).copy()
        self._last_command_monotonic_s = float(command_monotonic_s)


class Ros2JointCommandPublisher:
    """Publish validated six-axis IK results while preserving the current gripper position."""

    def __init__(
        self,
        topic: str,
        initial_joint_angles_rad: Sequence[float],
        gripper_position_m: float,
        max_joint_step_rad: float,
        max_joint_speed_rad_s: float,
        subscriber_timeout_s: float = 5.0,
    ):
        if not topic.startswith("/"):
            raise ValueError("ROS 2 command topic must be an absolute name beginning with '/'")
        if not math.isfinite(gripper_position_m):
            raise ValueError("gripper_position_m must be finite")
        if not math.isfinite(subscriber_timeout_s) or subscriber_timeout_s <= 0:
            raise ValueError("ROS 2 command subscriber timeout must be positive")

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise Ros2JointCommandError(
                "ROS 2 Python packages are unavailable. Source the ROS 2 environment before "
                "enabling joint command publishing."
            ) from exc

        self.topic = topic
        self.gripper_position_m = float(gripper_position_m)
        self._rclpy = rclpy
        self._joint_state_type = JointState
        self._context = rclpy.Context()
        self._node = None
        self._executor = None
        self._publisher = None
        self._closed = False
        self._limiter = JointCommandLimiter(
            initial_joint_angles_rad=initial_joint_angles_rad,
            max_joint_step_rad=max_joint_step_rad,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
        )

        try:
            rclpy.init(args=None, context=self._context)
            self._node = rclpy.create_node(
                "quest3_piper_joint_command_publisher",
                context=self._context,
            )
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._publisher = self._node.create_publisher(JointState, topic, qos)
            deadline = time.monotonic() + subscriber_timeout_s
            while self._publisher.get_subscription_count() < 1:
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    raise Ros2JointCommandError(
                        f"No subscriber discovered on {topic} within {subscriber_timeout_s:g}s"
                    )
                self._executor.spin_once(timeout_sec=min(remaining_s, 0.1))
        except Exception as exc:
            self.close()
            if isinstance(exc, Ros2JointCommandError):
                raise
            raise Ros2JointCommandError(
                f"Failed to initialize ROS 2 publisher for {topic}: {exc}"
            ) from exc

    def publish(self, joint_angles_rad: Sequence[float]) -> None:
        if self._closed or self._node is None or self._publisher is None:
            raise Ros2JointCommandError("ROS 2 joint command publisher is closed")
        if self._publisher.get_subscription_count() < 1:
            raise Ros2JointCommandError(f"Command subscriber disappeared from {self.topic}")

        angles, command_time_s = self._limiter.check(joint_angles_rad)
        message = self._joint_state_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = list(PIPER_COMMAND_JOINT_NAMES)
        message.position = [float(value) for value in angles] + [self.gripper_position_m]
        message.velocity = [0.0] * len(PIPER_COMMAND_JOINT_NAMES)
        message.effort = [0.0] * len(PIPER_COMMAND_JOINT_NAMES)
        try:
            self._publisher.publish(message)
        except Exception as exc:
            raise Ros2JointCommandError(
                f"Failed to publish joint command on {self.topic}: {exc}"
            ) from exc
        self._limiter.accept(angles, command_time_s)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)

    def __enter__(self) -> "Ros2JointCommandPublisher":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False
