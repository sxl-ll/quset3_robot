"""Validated Placo inverse kinematics for the repository's Piper model."""

from __future__ import annotations

from dataclasses import dataclass

import meshcat.transformations as tf
import numpy as np
import placo

from xrobotoolkit_teleop.common.pose_mapping import CartesianPose
from xrobotoolkit_teleop.robot.piper_model import (
    PIPER_END_EFFECTOR_FRAME,
    PIPER_JOINT_NAMES,
    load_piper_robot,
    piper_joint_limits,
    piper_joint_positions,
    set_piper_joint_positions,
)


@dataclass(frozen=True)
class IKResult:
    """A solved and independently validated Piper IK result."""

    success: bool
    joint_names: tuple[str, ...]
    joint_angles_rad: tuple[float, ...] | None
    joint_limits_valid: bool
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    message: str


def _rotation_error_rad(target_rotation: np.ndarray, actual_rotation: np.ndarray) -> float:
    relative_rotation = target_rotation.T @ actual_rotation
    cosine = (float(np.trace(relative_rotation)) - 1.0) / 2.0
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def _pose_to_transform(pose: CartesianPose) -> np.ndarray:
    position = np.asarray(pose.position, dtype=np.float64)
    quaternion_xyzw = np.asarray(pose.orientation_xyzw, dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("IK target position must contain three finite values")
    if quaternion_xyzw.shape != (4,) or not np.all(np.isfinite(quaternion_xyzw)):
        raise ValueError("IK target quaternion must contain four finite xyzw values")
    quaternion_norm = float(np.linalg.norm(quaternion_xyzw))
    if quaternion_norm <= 0 or not np.isclose(quaternion_norm, 1.0, atol=1e-2):
        raise ValueError(f"IK target quaternion must have unit norm, got {quaternion_norm:.6f}")

    quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]] / quaternion_norm
    transform = tf.quaternion_matrix(quaternion_wxyz)
    transform[:3, 3] = position
    return transform


def _transform_to_pose(transform: np.ndarray) -> CartesianPose:
    transform = np.asarray(transform, dtype=np.float64)
    quaternion_wxyz = tf.quaternion_from_matrix(transform)
    quaternion_xyzw = quaternion_wxyz[[1, 2, 3, 0]]
    return CartesianPose(
        position=tuple(float(value) for value in transform[:3, 3]),
        orientation_xyzw=tuple(float(value) for value in quaternion_xyzw),
    )


class PiperIKSolver:
    """Continuous, fixed-base Piper IK using the original Placo approach."""

    def __init__(
        self,
        initial_joint_angles_rad: np.ndarray | None = None,
        position_tolerance_m: float = 2e-3,
        orientation_tolerance_rad: float = 1e-3,
        joint_limit_tolerance_rad: float = 1e-8,
        initial_joint_limit_tolerance_rad: float = 0.0,
        max_iterations: int = 100,
        dt: float = 0.01,
    ):
        if not np.isfinite(position_tolerance_m) or position_tolerance_m <= 0:
            raise ValueError("position_tolerance_m must be positive")
        if not np.isfinite(orientation_tolerance_rad) or orientation_tolerance_rad <= 0:
            raise ValueError("orientation_tolerance_rad must be positive")
        if not np.isfinite(joint_limit_tolerance_rad) or joint_limit_tolerance_rad < 0:
            raise ValueError("joint_limit_tolerance_rad must be non-negative")
        if (
            not np.isfinite(initial_joint_limit_tolerance_rad)
            or initial_joint_limit_tolerance_rad < 0
        ):
            raise ValueError("initial_joint_limit_tolerance_rad must be non-negative")
        if not isinstance(max_iterations, int) or max_iterations <= 0:
            raise ValueError("max_iterations must be a positive integer")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")

        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)
        self.joint_limit_tolerance_rad = float(joint_limit_tolerance_rad)
        self.initial_joint_limit_tolerance_rad = float(initial_joint_limit_tolerance_rad)
        self.max_iterations = max_iterations

        self.robot = load_piper_robot()
        self.lower_joint_limits_rad, self.upper_joint_limits_rad = piper_joint_limits(self.robot)
        if initial_joint_angles_rad is None:
            initial_joint_angles_rad = (
                self.lower_joint_limits_rad + self.upper_joint_limits_rad
            ) / 2.0
        initial_joint_angles_rad = np.asarray(initial_joint_angles_rad, dtype=np.float64)
        if initial_joint_angles_rad.shape != (len(PIPER_JOINT_NAMES),) or not np.all(
            np.isfinite(initial_joint_angles_rad)
        ):
            raise ValueError(
                "Initial Piper joint angles must be finite and have shape (6,)"
            )
        self.initial_joint_limit_violations_rad = np.maximum.reduce(
            (
                self.lower_joint_limits_rad - initial_joint_angles_rad,
                initial_joint_angles_rad - self.upper_joint_limits_rad,
                np.zeros_like(initial_joint_angles_rad),
            )
        )
        if np.any(
            self.initial_joint_limit_violations_rad
            > self.initial_joint_limit_tolerance_rad
        ):
            raise ValueError(
                "Initial Piper joint angles exceed URDF limits beyond the allowed startup "
                f"tolerance {self.initial_joint_limit_tolerance_rad:g} rad; violations by "
                f"{PIPER_JOINT_NAMES}: {self.initial_joint_limit_violations_rad.tolist()}"
            )
        set_piper_joint_positions(self.robot, initial_joint_angles_rad)
        self.initial_target_pose = self.end_effector_pose

        # A measured joint can sit a few milliradians outside the URDF boundary
        # because of encoder/model zero offsets. Preserve its exact FK pose as the
        # mapping origin, but start the numerical solver from an in-limit seed.
        if np.any(self.initial_joint_limit_violations_rad > 0.0):
            solver_seed = np.clip(
                initial_joint_angles_rad,
                self.lower_joint_limits_rad,
                self.upper_joint_limits_rad,
            )
            set_piper_joint_positions(self.robot, solver_seed)

        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = float(dt)
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)

        initial_transform = np.asarray(
            self.robot.get_T_world_frame(PIPER_END_EFFECTOR_FRAME),
            dtype=np.float64,
        )
        self.effector_task = self.solver.add_frame_task(
            PIPER_END_EFFECTOR_FRAME,
            initial_transform,
        )
        self.effector_task.configure("piper_end_effector", "soft", 1.0, 1.0)
        manipulability_task = self.solver.add_manipulability_task(
            PIPER_END_EFFECTOR_FRAME,
            "both",
            1.0,
        )
        manipulability_task.configure("manipulability", "soft", 1e-2)

        self.initialization_result: IKResult | None = None
        if np.any(self.initial_joint_limit_violations_rad > 0.0):
            self.initialization_result = self.solve(self.initial_target_pose)
            if not self.initialization_result.success:
                raise ValueError(
                    "Could not reconcile the measured startup joints with strict URDF limits: "
                    f"{self.initialization_result.message}; position_error_m="
                    f"{self.initialization_result.position_error_m:.6g}; orientation_error_rad="
                    f"{self.initialization_result.orientation_error_rad:.6g}"
                )

    @property
    def joint_angles_rad(self) -> np.ndarray:
        return piper_joint_positions(self.robot)

    @property
    def end_effector_pose(self) -> CartesianPose:
        transform = np.asarray(
            self.robot.get_T_world_frame(PIPER_END_EFFECTOR_FRAME),
            dtype=np.float64,
        )
        return _transform_to_pose(transform)

    def forward_kinematics(self, joint_angles_rad: np.ndarray | None = None) -> CartesianPose:
        """Return link6 pose; restore the continuous solver state after probes."""
        if joint_angles_rad is None:
            return self.end_effector_pose
        joint_angles_rad = np.asarray(joint_angles_rad, dtype=np.float64)
        if not self.joint_limits_valid(joint_angles_rad):
            raise ValueError("FK joint angles must have shape (6,), be finite, and satisfy limits")

        saved_state = self.robot.state.q.copy()
        try:
            set_piper_joint_positions(self.robot, joint_angles_rad)
            return self.end_effector_pose
        finally:
            self.robot.state.q = saved_state
            self.robot.update_kinematics()

    def joint_limits_valid(self, joint_angles_rad: np.ndarray) -> bool:
        joint_angles_rad = np.asarray(joint_angles_rad, dtype=np.float64)
        if joint_angles_rad.shape != (len(PIPER_JOINT_NAMES),):
            return False
        if not np.all(np.isfinite(joint_angles_rad)):
            return False
        tolerance = self.joint_limit_tolerance_rad
        return bool(
            np.all(joint_angles_rad >= self.lower_joint_limits_rad - tolerance)
            and np.all(joint_angles_rad <= self.upper_joint_limits_rad + tolerance)
        )

    def _errors(self, target_transform: np.ndarray) -> tuple[float, float]:
        actual_transform = np.asarray(
            self.robot.get_T_world_frame(PIPER_END_EFFECTOR_FRAME),
            dtype=np.float64,
        )
        position_error = float(
            np.linalg.norm(actual_transform[:3, 3] - target_transform[:3, 3])
        )
        orientation_error = _rotation_error_rad(
            target_transform[:3, :3],
            actual_transform[:3, :3],
        )
        return position_error, orientation_error

    def solve(self, target_pose: CartesianPose) -> IKResult:
        """Solve a target and roll back the robot state if validation fails."""
        try:
            target_transform = _pose_to_transform(target_pose)
        except ValueError as exc:
            return IKResult(
                success=False,
                joint_names=PIPER_JOINT_NAMES,
                joint_angles_rad=None,
                joint_limits_valid=False,
                position_error_m=float("inf"),
                orientation_error_rad=float("inf"),
                iterations=0,
                message=f"invalid target: {exc}",
            )

        saved_state = self.robot.state.q.copy()
        self.effector_task.T_world_frame = target_transform
        position_error, orientation_error = self._errors(target_transform)
        if (
            position_error <= self.position_tolerance_m
            and orientation_error <= self.orientation_tolerance_rad
        ):
            angles = self.joint_angles_rad
            limits_valid = self.joint_limits_valid(angles)
            return IKResult(
                success=limits_valid,
                joint_names=PIPER_JOINT_NAMES,
                joint_angles_rad=tuple(float(value) for value in angles) if limits_valid else None,
                joint_limits_valid=limits_valid,
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                iterations=0,
                message="target already satisfied" if limits_valid else "joint limits invalid",
            )

        failure_message = "target tolerance not reached"
        iterations = 0
        try:
            for iterations in range(1, self.max_iterations + 1):
                delta_q = np.asarray(self.solver.solve(True), dtype=np.float64)
                self.robot.update_kinematics()
                if not np.all(np.isfinite(delta_q)):
                    failure_message = "Placo returned a non-finite joint update"
                    break

                angles = self.joint_angles_rad
                limits_valid = self.joint_limits_valid(angles)
                position_error, orientation_error = self._errors(target_transform)
                if (
                    limits_valid
                    and position_error <= self.position_tolerance_m
                    and orientation_error <= self.orientation_tolerance_rad
                ):
                    return IKResult(
                        success=True,
                        joint_names=PIPER_JOINT_NAMES,
                        joint_angles_rad=tuple(float(value) for value in angles),
                        joint_limits_valid=True,
                        position_error_m=position_error,
                        orientation_error_rad=orientation_error,
                        iterations=iterations,
                        message="IK solution validated",
                    )
                if not limits_valid:
                    failure_message = "joint limits invalid"
                    break
        except Exception as exc:
            failure_message = f"Placo solve failed: {exc}"

        candidate_angles = self.joint_angles_rad
        limits_valid = self.joint_limits_valid(candidate_angles)
        position_error, orientation_error = self._errors(target_transform)
        self.robot.state.q = saved_state
        self.robot.update_kinematics()
        return IKResult(
            success=False,
            joint_names=PIPER_JOINT_NAMES,
            joint_angles_rad=None,
            joint_limits_valid=limits_valid,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            iterations=iterations,
            message=failure_message,
        )

