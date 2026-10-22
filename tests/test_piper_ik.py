import unittest

import numpy as np

from xrobotoolkit_teleop.common.pose_mapping import CartesianPose
from xrobotoolkit_teleop.robot.piper_ik import PiperIKSolver
from xrobotoolkit_teleop.robot.piper_model import PIPER_JOINT_NAMES


class PiperIKSolverTest(unittest.TestCase):
    def setUp(self):
        self.ik = PiperIKSolver()

    def test_initial_state_uses_finite_in_limit_joint_vector(self):
        angles = self.ik.joint_angles_rad

        self.assertEqual(angles.shape, (6,))
        self.assertTrue(np.all(np.isfinite(angles)))
        self.assertTrue(self.ik.joint_limits_valid(angles))

    def test_measured_initial_state_allows_only_configured_small_limit_deviation(self):
        measured = np.array([-0.08, -0.004, -0.16, -0.10, 0.29, 0.10])

        with self.assertRaisesRegex(ValueError, "exceed URDF limits"):
            PiperIKSolver(initial_joint_angles_rad=measured)

        measured_ik = PiperIKSolver(
            initial_joint_angles_rad=measured,
            initial_joint_limit_tolerance_rad=0.01,
        )

        self.assertTrue(measured_ik.joint_limits_valid(measured_ik.joint_angles_rad))
        self.assertAlmostEqual(measured_ik.initial_joint_limit_violations_rad[1], 0.004)
        self.assertIsNotNone(measured_ik.initialization_result)
        self.assertTrue(measured_ik.initialization_result.success)

    def test_solves_pose_generated_by_forward_kinematics(self):
        known_angles = np.array([0.2, 1.4, -1.4, 0.1, 0.2, -0.1])
        target_pose = self.ik.forward_kinematics(known_angles)

        result = self.ik.solve(target_pose)

        self.assertTrue(result.success, result)
        self.assertEqual(result.joint_names, PIPER_JOINT_NAMES)
        self.assertIsNotNone(result.joint_angles_rad)
        self.assertEqual(len(result.joint_angles_rad), 6)
        self.assertTrue(result.joint_limits_valid)
        self.assertLessEqual(result.position_error_m, self.ik.position_tolerance_m)
        self.assertLessEqual(result.orientation_error_rad, self.ik.orientation_tolerance_rad)

        reconstructed = self.ik.forward_kinematics(np.asarray(result.joint_angles_rad))
        np.testing.assert_allclose(
            reconstructed.position,
            target_pose.position,
            atol=self.ik.position_tolerance_m,
        )

    def test_unreachable_target_fails_and_restores_last_valid_state(self):
        saved_angles = self.ik.joint_angles_rad.copy()
        current = self.ik.end_effector_pose
        unreachable = CartesianPose(
            (current.position[0] + 2.0, current.position[1], current.position[2]),
            current.orientation_xyzw,
        )

        result = self.ik.solve(unreachable)

        self.assertFalse(result.success)
        self.assertIsNone(result.joint_angles_rad)
        self.assertGreater(result.position_error_m, self.ik.position_tolerance_m)
        np.testing.assert_allclose(self.ik.joint_angles_rad, saved_angles)

    def test_invalid_target_quaternion_fails_without_changing_state(self):
        saved_angles = self.ik.joint_angles_rad.copy()
        invalid = CartesianPose((0.2, 0.0, 0.3), (0.0, 0.0, 0.0, 0.0))

        result = self.ik.solve(invalid)

        self.assertFalse(result.success)
        self.assertIn("invalid target", result.message)
        np.testing.assert_allclose(self.ik.joint_angles_rad, saved_angles)

    def test_repeated_nearby_targets_remain_continuous(self):
        first_pose = self.ik.end_effector_pose
        first_target = CartesianPose(
            (first_pose.position[0] + 0.01, first_pose.position[1], first_pose.position[2]),
            first_pose.orientation_xyzw,
        )
        first_result = self.ik.solve(first_target)
        self.assertTrue(first_result.success, first_result)
        first_angles = np.asarray(first_result.joint_angles_rad)

        second_target = CartesianPose(
            (first_target.position[0] + 0.005, first_target.position[1], first_target.position[2]),
            first_target.orientation_xyzw,
        )
        second_result = self.ik.solve(second_target)

        self.assertTrue(second_result.success, second_result)
        second_angles = np.asarray(second_result.joint_angles_rad)
        self.assertLess(float(np.linalg.norm(second_angles - first_angles)), 0.5)

    def test_near_singular_seed_returns_validated_result_or_safe_failure(self):
        singular_ik = PiperIKSolver(initial_joint_angles_rad=np.zeros(6))
        saved_angles = singular_ik.joint_angles_rad.copy()
        current = singular_ik.end_effector_pose
        target = CartesianPose(
            (current.position[0] + 0.005, current.position[1], current.position[2]),
            current.orientation_xyzw,
        )

        result = singular_ik.solve(target)

        self.assertTrue(np.isfinite(result.position_error_m))
        self.assertTrue(np.isfinite(result.orientation_error_rad))
        if result.success:
            self.assertTrue(result.joint_limits_valid)
            self.assertIsNotNone(result.joint_angles_rad)
            self.assertLessEqual(result.position_error_m, singular_ik.position_tolerance_m)
            self.assertLessEqual(
                result.orientation_error_rad,
                singular_ik.orientation_tolerance_rad,
            )
        else:
            self.assertIsNone(result.joint_angles_rad)
            np.testing.assert_allclose(singular_ik.joint_angles_rad, saved_angles)


if __name__ == "__main__":
    unittest.main()
