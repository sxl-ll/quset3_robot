import math
import unittest

from xrobotoolkit_teleop.common.ros2_joint_state import (
    Ros2JointStateError,
    parse_piper_joint_state,
)


class Ros2JointStateTest(unittest.TestCase):
    def test_extracts_arm_joints_by_name_and_preserves_gripper(self):
        sample = parse_piper_joint_state(
            ["joint6", "joint3", "joint0", "joint5", "joint2", "joint1", "joint4"],
            [0.0002, 0.3, 0.0, 0.5, 0.2, 0.1, 0.4],
            topic="/puppet/joint_right",
            timestamp_ns=123,
        )

        self.assertEqual(sample.source_joint_names, tuple(f"joint{i}" for i in range(6)))
        self.assertEqual(sample.joint_angles_rad, (0.0, 0.1, 0.2, 0.3, 0.4, 0.5))
        self.assertEqual(sample.gripper_position_m, 0.0002)
        self.assertEqual(sample.timestamp_ns, 123)

    def test_rejects_missing_arm_joint(self):
        with self.assertRaisesRegex(Ros2JointStateError, "missing required arm joints"):
            parse_piper_joint_state(
                ["joint0", "joint1"],
                [0.0, 0.1],
                topic="/puppet/joint_right",
            )

    def test_rejects_duplicate_names(self):
        with self.assertRaisesRegex(Ros2JointStateError, "duplicate"):
            parse_piper_joint_state(
                ["joint0", "joint0"],
                [0.0, 0.1],
                topic="/puppet/joint_right",
            )

    def test_rejects_non_finite_arm_position(self):
        with self.assertRaisesRegex(Ros2JointStateError, "NaN or Inf"):
            parse_piper_joint_state(
                [f"joint{i}" for i in range(6)],
                [0.0, 0.1, 0.2, math.nan, 0.4, 0.5],
                topic="/puppet/joint_right",
            )

    def test_rejects_non_finite_gripper_position(self):
        with self.assertRaisesRegex(Ros2JointStateError, "gripper"):
            parse_piper_joint_state(
                [f"joint{i}" for i in range(7)],
                [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, math.inf],
                topic="/puppet/joint_right",
            )


if __name__ == "__main__":
    unittest.main()
