import unittest

import numpy as np

from xrobotoolkit_teleop.common.pose_mapping import CartesianPose, RelativePoseMapper
from xrobotoolkit_teleop.common.quest3_reader import PoseSample, Quest3Snapshot
from xrobotoolkit_teleop.common.teleop_pipeline import (
    Quest3PiperPipeline,
    format_pipeline_result,
)
from xrobotoolkit_teleop.robot.piper_ik import IKResult
from xrobotoolkit_teleop.robot.piper_model import PIPER_JOINT_NAMES


def make_snapshot(
    right_position=(0.0, 0.0, 0.0),
    right_grip=0.0,
    tracking_valid=True,
    timestamp_valid=None,
    left_pose_valid=True,
    right_pose_valid=True,
    timestamp_ns=1,
):
    if timestamp_valid is None:
        timestamp_valid = tracking_valid
    identity_orientation = (0.0, 0.0, 0.0, 1.0)
    right_raw = (*right_position, *identity_orientation)
    poses = {
        "headset": PoseSample(
            (0.0, 0.0, 0.0, *identity_orientation),
            (0.0, 0.0, 0.0),
            identity_orientation,
            left_pose_valid,
        ),
        "left_controller": PoseSample(
            (0.0, 0.0, 0.0, *identity_orientation),
            (0.0, 0.0, 0.0),
            identity_orientation,
            True,
        ),
        "right_controller": PoseSample(
            right_raw,
            tuple(right_position),
            identity_orientation,
            right_pose_valid,
        ),
    }
    return Quest3Snapshot(
        timestamp_ns=timestamp_ns,
        host_monotonic_ns=timestamp_ns,
        timestamp_advanced=True,
        timestamp_age_s=0.0,
        stale=not timestamp_valid,
        timestamp_valid=timestamp_valid,
        tracking_valid=tracking_valid,
        poses=poses,
        analog_inputs={
            "left_grip": 0.0,
            "right_grip": right_grip,
            "left_trigger": 0.0,
            "right_trigger": 0.0,
        },
        buttons={},
        joysticks={},
        issues=(),
    )


class FakeIKSolver:
    def __init__(self):
        self._pose = CartesianPose((0.4, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))
        self.calls = []
        self.should_succeed = True
        self.commanded_joint_angles = np.zeros(6)

    @property
    def end_effector_pose(self):
        return self._pose

    def solve(self, target_pose):
        self.calls.append(target_pose)
        if self.should_succeed:
            self._pose = target_pose
            angles = (0.0,) * 6
        else:
            angles = None
        return IKResult(
            success=self.should_succeed,
            joint_names=PIPER_JOINT_NAMES,
            joint_angles_rad=angles,
            joint_limits_valid=True,
            position_error_m=0.0 if self.should_succeed else 1.0,
            orientation_error_rad=0.0,
            iterations=1,
            message="test result",
        )

    def set_joint_angles(self, joint_angles_rad):
        self.commanded_joint_angles = np.asarray(joint_angles_rad, dtype=np.float64)
        self._pose = CartesianPose(
            (0.4 + float(self.commanded_joint_angles[0]), 0.0, 0.3),
            (0.0, 0.0, 0.0, 1.0),
        )


class Quest3PiperPipelineTest(unittest.TestCase):
    def setUp(self):
        self.ik = FakeIKSolver()
        self.mapper = RelativePoseMapper(
            self.ik.end_effector_pose,
            xr_to_robot_rotation=np.eye(3),
            smoothing_alpha=1.0,
        )
        self.pipeline = Quest3PiperPipeline(self.mapper, self.ik)

    def test_inactive_and_anchor_samples_do_not_run_ik(self):
        inactive = self.pipeline.process(make_snapshot(right_grip=0.0))
        anchored = self.pipeline.process(make_snapshot(right_grip=1.0))

        self.assertEqual(inactive.mapping.reason, "clutch_inactive")
        self.assertEqual(anchored.mapping.reason, "clutch_anchored")
        self.assertEqual(self.ik.calls, [])

    def test_valid_motion_runs_ik_and_formats_joint_angles(self):
        self.pipeline.process(make_snapshot(right_grip=1.0))

        result = self.pipeline.process(
            make_snapshot(right_position=(0.01, 0.0, 0.0), right_grip=1.0)
        )

        self.assertEqual(len(self.ik.calls), 1)
        self.assertTrue(result.ik.success)
        formatted = format_pipeline_result(result)
        self.assertIn(
            "right_controller.raw=(+0.010000, +0.000000, +0.000000, "
            "+0.000000, +0.000000, +0.000000, +1.000000)",
            formatted,
        )
        self.assertIn("ik_status=SUCCESS", formatted)
        self.assertIn("joint_angles_rad", formatted)

    def test_invalid_tracking_never_runs_ik(self):
        result = self.pipeline.process(
            make_snapshot(right_position=(1.0, 0.0, 0.0), right_grip=1.0, tracking_valid=False)
        )

        self.assertEqual(result.mapping.reason, "tracking_invalid")
        self.assertIsNone(result.ik)
        self.assertEqual(self.ik.calls, [])

    def test_command_synchronization_updates_solver_and_held_mapping_target(self):
        commanded = (0.02, 0.0, 0.0, 0.0, 0.0, 0.0)

        commanded_pose = self.pipeline.synchronize_commanded_joint_angles(
            commanded,
            hold_mapping_target=True,
        )

        np.testing.assert_allclose(self.ik.commanded_joint_angles, commanded)
        np.testing.assert_allclose(commanded_pose.position, (0.42, 0.0, 0.3))
        self.assertEqual(self.mapper.target_pose, commanded_pose)

    def test_invalid_left_controller_does_not_block_right_controller_teleop(self):
        self.pipeline.process(
            make_snapshot(
                right_grip=1.0,
                tracking_valid=False,
                timestamp_valid=True,
                left_pose_valid=False,
            )
        )
        result = self.pipeline.process(
            make_snapshot(
                right_position=(0.01, 0.0, 0.0),
                right_grip=1.0,
                tracking_valid=False,
                timestamp_valid=True,
                left_pose_valid=False,
            )
        )

        self.assertTrue(result.tracking_valid)
        self.assertEqual(result.mapping.reason, "target_updated")
        self.assertEqual(len(self.ik.calls), 1)

    def test_failed_ik_restores_last_valid_target_and_requires_reanchor(self):
        self.pipeline.process(make_snapshot(right_grip=1.0))
        successful = self.pipeline.process(
            make_snapshot(right_position=(0.01, 0.0, 0.0), right_grip=1.0)
        )
        last_valid_target = successful.mapping.target_pose
        self.ik.should_succeed = False

        failed = self.pipeline.process(
            make_snapshot(right_position=(1.0, 0.0, 0.0), right_grip=1.0)
        )
        reanchored = self.pipeline.process(
            make_snapshot(right_position=(1.0, 0.0, 0.0), right_grip=1.0)
        )

        self.assertFalse(failed.ik.success)
        self.assertEqual(self.mapper.target_pose, last_valid_target)
        self.assertEqual(reanchored.mapping.reason, "clutch_anchored")
        self.assertEqual(len(self.ik.calls), 2)

    def test_real_placo_pipeline_solves_small_synthetic_controller_motion(self):
        pipeline = Quest3PiperPipeline.create(position_scale=1.0, smoothing_alpha=1.0)
        pipeline.process(make_snapshot(right_grip=1.0))

        result = pipeline.process(
            make_snapshot(right_position=(0.0, 0.0, 0.005), right_grip=1.0)
        )

        self.assertIsNotNone(result.ik)
        self.assertTrue(result.ik.success, result.ik)
        self.assertTrue(result.ik.joint_limits_valid)

    def test_measured_joint_state_initializes_target_and_reconciles_limits(self):
        measured = np.array(
            [-0.07923795, -0.00434587, -0.15837118, -0.09841912, 0.28754299, 0.10171779]
        )
        pipeline = Quest3PiperPipeline.create(
            initial_joint_angles_rad=measured,
            initial_joint_limit_tolerance_rad=0.01,
            smoothing_alpha=1.0,
        )

        np.testing.assert_allclose(
            pipeline.mapper.target_pose.position,
            (0.04550313, -0.00616530, 0.24030219),
            atol=1e-7,
        )
        self.assertTrue(pipeline.ik_solver.joint_limits_valid(pipeline.ik_solver.joint_angles_rad))
        pipeline.process(make_snapshot(right_grip=1.0))
        result = pipeline.process(
            make_snapshot(right_position=(0.0, 0.005, 0.0), right_grip=1.0)
        )
        self.assertIsNotNone(result.ik)
        self.assertTrue(result.ik.success, result.ik)


if __name__ == "__main__":
    unittest.main()
