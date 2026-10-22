"""Guarded ROS 2 JointState publisher for the Piper right-arm command topic."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


PIPER_COMMAND_JOINT_NAMES = tuple(f"joint{index}" for index in range(7))


class Ros2JointCommandError(RuntimeError):
    """Raised when the ROS 2 command publisher cannot be created or used."""


class JointCommandRejected(RuntimeError):
    """Raised when a joint command fails a local safety check."""


@dataclass(frozen=True)
class JointCommandPlan:
    """One acceleration- and velocity-limited command toward an IK target."""

    desired_joint_angles_rad: tuple[float, ...]
    command_joint_angles_rad: tuple[float, ...]
    command_velocity_rad_s: tuple[float, ...]
    elapsed_s: float
    rate_limited: bool
    largest_tracking_error_rad: float
    command_monotonic_s: float


class JointCommandLimiter:
    """Reject IK jumps and smoothly approach accepted targets."""

    def __init__(
        self,
        initial_joint_angles_rad: Sequence[float],
        max_joint_step_rad: float,
        max_joint_speed_rad_s: float,
        max_joint_acceleration_rad_s2: float,
        monotonic: Callable[[], float] = time.monotonic,
        max_timestep_s: float = 0.1,
    ):
        if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
            raise ValueError("max_joint_step_rad must be positive")
        if not math.isfinite(max_joint_speed_rad_s) or max_joint_speed_rad_s <= 0:
            raise ValueError("max_joint_speed_rad_s must be positive")
        if (
            not math.isfinite(max_joint_acceleration_rad_s2)
            or max_joint_acceleration_rad_s2 <= 0
        ):
            raise ValueError("max_joint_acceleration_rad_s2 must be positive")
        if not math.isfinite(max_timestep_s) or max_timestep_s <= 0:
            raise ValueError("max_timestep_s must be positive")

        self.max_joint_step_rad = float(max_joint_step_rad)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self.max_joint_acceleration_rad_s2 = float(max_joint_acceleration_rad_s2)
        self.max_timestep_s = float(max_timestep_s)
        self._monotonic = monotonic
        initial_angles = self._parse_angles(initial_joint_angles_rad)
        self._last_desired_joint_angles_rad = initial_angles.copy()
        self._last_command_joint_angles_rad = initial_angles.copy()
        self._last_command_velocity_rad_s = np.zeros(6, dtype=np.float64)
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

    def check(self, joint_angles_rad: Sequence[float]) -> JointCommandPlan:
        desired_angles = self._parse_angles(joint_angles_rad)
        now_s = self._monotonic()
        elapsed_s = min(
            max(0.0, now_s - self._last_command_monotonic_s),
            self.max_timestep_s,
        )

        desired_delta = np.abs(desired_angles - self._last_desired_joint_angles_rad)
        largest_step = float(np.max(desired_delta))
        if largest_step > self.max_joint_step_rad:
            raise JointCommandRejected(
                f"largest IK target step {largest_step:.6f} rad exceeds "
                f"limit {self.max_joint_step_rad:.6f} rad"
            )

        tracking_error = desired_angles - self._last_command_joint_angles_rad
        if elapsed_s <= 0.0:
            command_velocity = np.zeros(6, dtype=np.float64)
            command_angles = self._last_command_joint_angles_rad.copy()
        else:
            braking_speed = np.sqrt(
                2.0 * self.max_joint_acceleration_rad_s2 * np.abs(tracking_error)
            )
            target_speed = np.minimum(self.max_joint_speed_rad_s, braking_speed)
            target_velocity = np.sign(tracking_error) * target_speed
            maximum_velocity_change = self.max_joint_acceleration_rad_s2 * elapsed_s
            command_velocity = np.clip(
                target_velocity,
                self._last_command_velocity_rad_s - maximum_velocity_change,
                self._last_command_velocity_rad_s + maximum_velocity_change,
            )

            # A target reversal must not make the arm continue away from the new target.
            command_velocity[command_velocity * tracking_error <= 0.0] = 0.0
            command_delta = command_velocity * elapsed_s
            reached_target = np.abs(command_delta) >= np.abs(tracking_error)
            command_delta[reached_target] = tracking_error[reached_target]
            command_velocity[reached_target] = 0.0
            command_angles = self._last_command_joint_angles_rad + command_delta

        rate_limited = not np.allclose(command_angles, desired_angles, rtol=0.0, atol=1e-12)
        return JointCommandPlan(
            desired_joint_angles_rad=tuple(float(value) for value in desired_angles),
            command_joint_angles_rad=tuple(float(value) for value in command_angles),
            command_velocity_rad_s=tuple(float(value) for value in command_velocity),
            elapsed_s=elapsed_s,
            rate_limited=rate_limited,
            largest_tracking_error_rad=float(np.max(np.abs(tracking_error))),
            command_monotonic_s=now_s,
        )

    def accept(self, plan: JointCommandPlan) -> None:
        self._last_desired_joint_angles_rad = np.asarray(
            plan.desired_joint_angles_rad,
            dtype=np.float64,
        )
        self._last_command_joint_angles_rad = np.asarray(
            plan.command_joint_angles_rad,
            dtype=np.float64,
        )
        self._last_command_velocity_rad_s = np.asarray(
            plan.command_velocity_rad_s,
            dtype=np.float64,
        )
        self._last_command_monotonic_s = float(plan.command_monotonic_s)

    def hold(self) -> None:
        """Stop trajectory progress and make the last command the next target origin."""
        self._last_desired_joint_angles_rad = self._last_command_joint_angles_rad.copy()
        self._last_command_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._last_command_monotonic_s = self._monotonic()

    @property
    def last_command_joint_angles_rad(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self._last_command_joint_angles_rad)


class Ros2JointCommandPublisher:
    """Publish validated six-axis IK results while preserving the current gripper position."""

    def __init__(
        self,
        topic: str,
        initial_joint_angles_rad: Sequence[float],
        gripper_position_m: float,
        max_joint_step_rad: float,
        max_joint_speed_rad_s: float,
        max_joint_acceleration_rad_s2: float,
        driver_speed_percent: float = 20.0,
        subscriber_timeout_s: float = 5.0,
    ):
        if not topic.startswith("/"):
            raise ValueError("ROS 2 command topic must be an absolute name beginning with '/'")
        if not math.isfinite(gripper_position_m):
            raise ValueError("gripper_position_m must be finite")
        if not math.isfinite(subscriber_timeout_s) or subscriber_timeout_s <= 0:
            raise ValueError("ROS 2 command subscriber timeout must be positive")
        if (
            not math.isfinite(driver_speed_percent)
            or not 1 <= driver_speed_percent <= 100
        ):
            raise ValueError("driver_speed_percent must be within [1, 100]")

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
        self.driver_speed_percent = float(driver_speed_percent)
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
            max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
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

    def publish(self, joint_angles_rad: Sequence[float]) -> JointCommandPlan:
        if self._closed or self._node is None or self._publisher is None:
            raise Ros2JointCommandError("ROS 2 joint command publisher is closed")
        if self._publisher.get_subscription_count() < 1:
            raise Ros2JointCommandError(f"Command subscriber disappeared from {self.topic}")

        plan = self._limiter.check(joint_angles_rad)
        message = self._joint_state_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = list(PIPER_COMMAND_JOINT_NAMES)
        message.position = list(plan.command_joint_angles_rad) + [self.gripper_position_m]
        message.velocity = [0.0] * 6 + [self.driver_speed_percent]
        message.effort = [0.0] * len(PIPER_COMMAND_JOINT_NAMES)
        try:
            self._publisher.publish(message)
        except Exception as exc:
            raise Ros2JointCommandError(
                f"Failed to publish joint command on {self.topic}: {exc}"
            ) from exc
        self._limiter.accept(plan)
        return plan

    @property
    def last_command_joint_angles_rad(self) -> tuple[float, ...]:
        return self._limiter.last_command_joint_angles_rad

    def hold(self) -> None:
        self._limiter.hold()

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
