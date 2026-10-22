import unittest

import numpy as np

from xrobotoolkit_teleop.robot.piper_model import (
    PIPER_END_EFFECTOR_FRAME,
    PIPER_JOINT_NAMES,
    PIPER_PACKAGE_URI_PREFIX,
    load_piper_robot,
    piper_joint_limits,
    piper_joint_positions,
    resolved_piper_urdf_content,
    set_piper_joint_positions,
)
from xrobotoolkit_teleop.utils.path_utils import PIPER_ASSET_DIR


class PiperModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_piper_robot()

    def setUp(self):
        set_piper_joint_positions(self.robot, np.zeros(len(PIPER_JOINT_NAMES)))

    def test_resolves_ros_package_mesh_uris_without_modifying_source(self):
        resolved = resolved_piper_urdf_content()

        self.assertNotIn(PIPER_PACKAGE_URI_PREFIX, resolved)
        self.assertNotIn("package://", resolved)
        self.assertIn(PIPER_ASSET_DIR.resolve().as_uri(), resolved)

    def test_loads_expected_joint_order_and_end_effector_frame(self):
        self.assertEqual(tuple(self.robot.joint_names()), PIPER_JOINT_NAMES)
        self.assertIn(PIPER_END_EFFECTOR_FRAME, tuple(self.robot.frame_names()))

    def test_reads_finite_ordered_joint_limits(self):
        lower, upper = piper_joint_limits(self.robot)

        self.assertEqual(lower.shape, (6,))
        self.assertEqual(upper.shape, (6,))
        self.assertTrue(np.all(np.isfinite(lower)))
        self.assertTrue(np.all(np.isfinite(upper)))
        self.assertTrue(np.all(lower <= upper))
        np.testing.assert_allclose(
            lower,
            [-2.6179938, 0.0, -2.9670597, -1.7453292, -1.2217304, -2.9670597],
        )
        np.testing.assert_allclose(
            upper,
            [2.6179938, 3.1415926, 0.0, 1.7453292, 1.2217304, 2.9670597],
        )

    def test_joint_position_helpers_preserve_verified_order(self):
        lower, upper = piper_joint_limits(self.robot)
        expected = (lower + upper) / 2.0

        set_piper_joint_positions(self.robot, expected)

        np.testing.assert_allclose(piper_joint_positions(self.robot), expected)

    def test_forward_kinematics_returns_finite_transform(self):
        transform = np.asarray(self.robot.get_T_world_frame(PIPER_END_EFFECTOR_FRAME))

        self.assertEqual(transform.shape, (4, 4))
        self.assertTrue(np.all(np.isfinite(transform)))
        np.testing.assert_allclose(transform[3], [0.0, 0.0, 0.0, 1.0])
        np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
