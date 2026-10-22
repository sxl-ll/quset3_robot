"""Quest 3 to Piper IK integration without robot actuation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from xrobotoolkit_teleop.common.pose_mapping import (
    CartesianPose,
    PoseMappingResult,
    RelativePoseMapper,
)
from xrobotoolkit_teleop.common.quest3_reader import Quest3Snapshot
from xrobotoolkit_teleop.robot.piper_ik import IKResult, PiperIKSolver


class IKSolver(Protocol):
    @property
    def end_effector_pose(self) -> CartesianPose: ...

    def solve(self, target_pose: CartesianPose) -> IKResult: ...


@dataclass(frozen=True)
class PipelineResult:
    """Result of processing one Quest 3 snapshot."""

    timestamp_ns: int
    tracking_valid: bool
    right_controller_raw: tuple[float, ...]
    grip_value: float
    mapping: PoseMappingResult
    ik: IKResult | None


class Quest3PiperPipeline:
    """Feed validated right-controller samples into mapping and Piper IK."""

    def __init__(self, mapper: RelativePoseMapper, ik_solver: IKSolver):
        self.mapper = mapper
        self.ik_solver = ik_solver
        self._last_valid_target = mapper.target_pose

    @classmethod
    def create(
        cls,
        position_scale: float = 1.0,
        smoothing_alpha: float = 0.25,
        grip_threshold: float = 0.9,
        initial_joint_angles_rad: np.ndarray | None = None,
        initial_joint_limit_tolerance_rad: float = 0.0,
    ) -> "Quest3PiperPipeline":
        ik_solver = PiperIKSolver(
            initial_joint_angles_rad=initial_joint_angles_rad,
            initial_joint_limit_tolerance_rad=initial_joint_limit_tolerance_rad,
        )
        mapper = RelativePoseMapper(
            initial_target_pose=ik_solver.initial_target_pose,
            position_scale=position_scale,
            smoothing_alpha=smoothing_alpha,
            grip_threshold=grip_threshold,
        )
        return cls(mapper=mapper, ik_solver=ik_solver)

    def process(self, snapshot: Quest3Snapshot) -> PipelineResult:
        right_controller = snapshot.poses["right_controller"]
        grip_value = snapshot.analog_inputs["right_grip"]
        right_tracking_valid = snapshot.timestamp_valid and right_controller.valid
        mapping = self.mapper.update(
            controller_pose_xyzw=right_controller.raw,
            grip_value=grip_value,
            tracking_valid=right_tracking_valid,
        )

        ik_result = None
        if mapping.target_updated:
            ik_result = self.ik_solver.solve(mapping.target_pose)
            if ik_result.success:
                self._last_valid_target = mapping.target_pose
            else:
                self.mapper.hold_target(self._last_valid_target)

        return PipelineResult(
            timestamp_ns=snapshot.timestamp_ns,
            tracking_valid=right_tracking_valid,
            right_controller_raw=right_controller.raw,
            grip_value=grip_value,
            mapping=mapping,
            ik=ik_result,
        )


def format_pipeline_result(result: PipelineResult) -> str:
    """Format one pipeline result with target, IK validity, and joint angles."""
    target = result.mapping.target_pose
    right_controller_raw = ", ".join(
        f"{value:+.6f}" for value in result.right_controller_raw
    )
    position = ", ".join(f"{value:+.6f}" for value in target.position)
    orientation = ", ".join(f"{value:+.6f}" for value in target.orientation_xyzw)
    lines = [
        (
            f"timestamp_ns={result.timestamp_ns} right_tracking_valid={result.tracking_valid} "
            f"right_grip={result.grip_value:.3f} mapping={result.mapping.reason}"
        ),
        f"  right_controller.raw=({right_controller_raw})",
        f"  target_position=({position})",
        f"  target_orientation_xyzw=({orientation})",
    ]
    if result.ik is None:
        lines.append("  ik_status=NOT_RUN")
        return "\n".join(lines)

    ik_result = result.ik
    lines.append(
        (
            f"  ik_status={'SUCCESS' if ik_result.success else 'FAILURE'} "
            f"joint_limits_valid={ik_result.joint_limits_valid} "
            f"position_error_m={ik_result.position_error_m:.6g} "
            f"orientation_error_rad={ik_result.orientation_error_rad:.6g} "
            f"iterations={ik_result.iterations} message={ik_result.message}"
        )
    )
    if ik_result.joint_angles_rad is not None:
        joint_values = ", ".join(
            f"{name}={angle:+.6f}"
            for name, angle in zip(ik_result.joint_names, ik_result.joint_angles_rad)
        )
        lines.append(f"  joint_angles_rad: {joint_values}")
    else:
        lines.append("  joint_angles_rad: unavailable")
    return "\n".join(lines)

