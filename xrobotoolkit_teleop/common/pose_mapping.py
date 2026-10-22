"""Hardware-independent relative pose mapping with a Grip clutch."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)


DEFAULT_XR_TO_ROBOT_ROTATION = np.asarray(R_HEADSET_TO_WORLD, dtype=np.float64).copy()


@dataclass(frozen=True)
class CartesianPose:
    """A Cartesian pose using robot-frame metres and an xyzw quaternion."""

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseMappingResult:
    """One mapping update and the reason that it did or did not move the target."""

    target_pose: CartesianPose
    clutch_active: bool
    target_updated: bool
    reason: str
    controller_delta_raw_units: tuple[float, float, float] | None = None
    mapped_position_delta_m: tuple[float, float, float] | None = None


class RelativePoseMapper:
    """Map relative XR-controller motion into the robot base frame.

    The mapping follows the reusable XRoboToolkit teleoperation logic: XR pose
    is rotated into the robot frame, position is scaled, orientation is applied
    as a relative quaternion, and a low-pass filter smooths controller input.
    Releasing Grip holds the target. Re-engaging Grip creates a new controller
    anchor at the held target, preventing a jump.
    """

    def __init__(
        self,
        initial_target_pose: CartesianPose,
        xr_to_robot_rotation: np.ndarray = DEFAULT_XR_TO_ROBOT_ROTATION,
        position_scale: float = 1.0,
        orientation_scale: float = 1.0,
        smoothing_alpha: float = 0.25,
        grip_threshold: float = 0.9,
        quaternion_norm_tolerance: float = 1e-2,
    ):
        rotation = np.asarray(xr_to_robot_rotation, dtype=np.float64)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("xr_to_robot_rotation must be a finite 3x3 matrix")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
            raise ValueError("xr_to_robot_rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7):
            raise ValueError("xr_to_robot_rotation must be a proper rotation with determinant +1")
        if not np.isfinite(position_scale) or position_scale <= 0:
            raise ValueError("position_scale must be a finite positive number")
        if not np.isfinite(orientation_scale) or not 0 <= orientation_scale <= 1:
            raise ValueError("orientation_scale must be in [0, 1]")
        if not np.isfinite(smoothing_alpha) or not 0 < smoothing_alpha <= 1:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if not np.isfinite(grip_threshold) or not 0 <= grip_threshold <= 1:
            raise ValueError("grip_threshold must be in [0, 1]")
        if not np.isfinite(quaternion_norm_tolerance) or quaternion_norm_tolerance <= 0:
            raise ValueError("quaternion_norm_tolerance must be positive")

        self.xr_to_robot_rotation = rotation.copy()
        self.position_scale = float(position_scale)
        self.orientation_scale = float(orientation_scale)
        self.smoothing_alpha = float(smoothing_alpha)
        self.grip_threshold = float(grip_threshold)
        self.quaternion_norm_tolerance = float(quaternion_norm_tolerance)
        target_position, target_quaternion_wxyz = self._validate_target(initial_target_pose)

        self._target_position = target_position
        self._target_quaternion_wxyz = target_quaternion_wxyz
        self._reference_target_position: np.ndarray | None = None
        self._reference_target_quaternion_wxyz: np.ndarray | None = None
        self._reference_controller_position: np.ndarray | None = None
        self._reference_controller_quaternion_wxyz: np.ndarray | None = None
        self._filtered_controller_position: np.ndarray | None = None
        self._filtered_controller_quaternion_wxyz: np.ndarray | None = None

        rotation_transform = np.eye(4)
        rotation_transform[:3, :3] = self.xr_to_robot_rotation
        self._rotation_quaternion_wxyz = tf.quaternion_from_matrix(rotation_transform)

    def _validate_target(self, pose: CartesianPose) -> tuple[np.ndarray, np.ndarray]:
        position = np.asarray(pose.position, dtype=np.float64)
        quaternion_xyzw = np.asarray(pose.orientation_xyzw, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Target position must contain three finite values")
        if quaternion_xyzw.shape != (4,) or not np.all(np.isfinite(quaternion_xyzw)):
            raise ValueError("Target orientation must contain four finite xyzw values")
        norm = float(np.linalg.norm(quaternion_xyzw))
        if norm <= 0 or not np.isclose(
            norm,
            1.0,
            atol=self.quaternion_norm_tolerance,
        ):
            raise ValueError(f"Target quaternion must have unit norm, got {norm:.6f}")
        quaternion_xyzw = quaternion_xyzw / norm
        quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]]
        return position.copy(), quaternion_wxyz

    def _parse_controller_pose(
        self, controller_pose_xyzw: Sequence[float]
    ) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            pose = np.asarray(controller_pose_xyzw, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            return None
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            return None
        quaternion_xyzw = pose[3:]
        norm = float(np.linalg.norm(quaternion_xyzw))
        if norm <= 0 or not np.isclose(norm, 1.0, atol=self.quaternion_norm_tolerance):
            return None

        controller_position = self.xr_to_robot_rotation @ pose[:3]
        controller_quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]] / norm
        controller_quaternion_wxyz = tf.quaternion_multiply(
            tf.quaternion_multiply(
                self._rotation_quaternion_wxyz,
                controller_quaternion_wxyz,
            ),
            tf.quaternion_conjugate(self._rotation_quaternion_wxyz),
        )
        controller_quaternion_wxyz /= np.linalg.norm(controller_quaternion_wxyz)
        return controller_position, controller_quaternion_wxyz

    @property
    def target_pose(self) -> CartesianPose:
        quaternion_xyzw = self._target_quaternion_wxyz[[1, 2, 3, 0]]
        return CartesianPose(
            position=tuple(float(value) for value in self._target_position),
            orientation_xyzw=tuple(float(value) for value in quaternion_xyzw),
        )

    def _clear_controller_reference(self) -> None:
        self._reference_target_position = None
        self._reference_target_quaternion_wxyz = None
        self._reference_controller_position = None
        self._reference_controller_quaternion_wxyz = None
        self._filtered_controller_position = None
        self._filtered_controller_quaternion_wxyz = None

    def hold_target(self, target_pose: CartesianPose) -> None:
        """Replace the held target and require a fresh clutch anchor.

        The integration pipeline uses this after a rejected IK target so the
        next valid controller sample cannot jump from an unreachable request.
        """
        target_position, target_quaternion_wxyz = self._validate_target(target_pose)
        self._target_position = target_position
        self._target_quaternion_wxyz = target_quaternion_wxyz
        self._clear_controller_reference()

    def update(
        self,
        controller_pose_xyzw: Sequence[float],
        grip_value: float,
        tracking_valid: bool,
    ) -> PoseMappingResult:
        """Update and return the held target pose without invoking IK."""
        if not tracking_valid:
            self._clear_controller_reference()
            return PoseMappingResult(self.target_pose, False, False, "tracking_invalid")

        try:
            grip_value = float(grip_value)
        except (TypeError, ValueError):
            grip_value = float("nan")
        if not np.isfinite(grip_value):
            self._clear_controller_reference()
            return PoseMappingResult(self.target_pose, False, False, "grip_invalid")

        clutch_active = grip_value > self.grip_threshold
        if not clutch_active:
            self._clear_controller_reference()
            return PoseMappingResult(self.target_pose, False, False, "clutch_inactive")

        parsed_pose = self._parse_controller_pose(controller_pose_xyzw)
        if parsed_pose is None:
            self._clear_controller_reference()
            return PoseMappingResult(self.target_pose, True, False, "controller_pose_invalid")
        controller_position, controller_quaternion_wxyz = parsed_pose

        if self._reference_controller_position is None:
            self._reference_target_position = self._target_position.copy()
            self._reference_target_quaternion_wxyz = self._target_quaternion_wxyz.copy()
            self._reference_controller_position = controller_position.copy()
            self._reference_controller_quaternion_wxyz = controller_quaternion_wxyz.copy()
            self._filtered_controller_position = controller_position.copy()
            self._filtered_controller_quaternion_wxyz = controller_quaternion_wxyz.copy()
            zero_delta = (0.0, 0.0, 0.0)
            return PoseMappingResult(
                self.target_pose,
                True,
                False,
                "clutch_anchored",
                controller_delta_raw_units=zero_delta,
                mapped_position_delta_m=zero_delta,
            )

        assert self._reference_target_position is not None
        assert self._reference_target_quaternion_wxyz is not None
        assert self._reference_controller_quaternion_wxyz is not None
        assert self._filtered_controller_position is not None
        assert self._filtered_controller_quaternion_wxyz is not None

        alpha = self.smoothing_alpha
        self._filtered_controller_position = (
            alpha * controller_position + (1.0 - alpha) * self._filtered_controller_position
        )
        self._filtered_controller_quaternion_wxyz = tf.quaternion_slerp(
            self._filtered_controller_quaternion_wxyz,
            controller_quaternion_wxyz,
            alpha,
        )

        controller_delta = controller_position - self._reference_controller_position
        filtered_controller_delta = (
            self._filtered_controller_position - self._reference_controller_position
        )
        delta_position = filtered_controller_delta * self.position_scale
        delta_rotation = quat_diff_as_angle_axis(
            self._reference_controller_quaternion_wxyz,
            self._filtered_controller_quaternion_wxyz,
        )
        delta_rotation *= self.orientation_scale
        target_position, target_quaternion_wxyz = apply_delta_pose(
            self._reference_target_position,
            self._reference_target_quaternion_wxyz,
            delta_position,
            delta_rotation,
        )
        target_quaternion_wxyz /= np.linalg.norm(target_quaternion_wxyz)
        self._target_position = target_position
        self._target_quaternion_wxyz = target_quaternion_wxyz
        return PoseMappingResult(
            self.target_pose,
            True,
            True,
            "target_updated",
            controller_delta_raw_units=tuple(float(value) for value in controller_delta),
            mapped_position_delta_m=tuple(float(value) for value in delta_position),
        )

