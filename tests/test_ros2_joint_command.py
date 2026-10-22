import unittest

import numpy as np

from xrobotoolkit_teleop.common.ros2_joint_command import (
    JointCommandLimiter,
    JointCommandRejected,
)


class ManualClock:
    def __init__(self):
        self.now_s = 0.0

    def __call__(self):
        return self.now_s


class JointCommandLimiterTest(unittest.TestCase):
    def setUp(self):
        self.clock = ManualClock()
        self.limiter = JointCommandLimiter(
            initial_joint_angles_rad=np.zeros(6),
            max_joint_step_rad=0.05,
            max_joint_speed_rad_s=0.5,
            max_joint_acceleration_rad_s2=2.0,
            monotonic=self.clock,
        )

    def test_accepts_small_continuous_command(self):
        self.clock.now_s = 0.1
        plan = self.limiter.check([0.01] * 6)
        self.limiter.accept(plan)

        self.clock.now_s = 0.2
        next_plan = self.limiter.check([0.02] * 6)
        np.testing.assert_allclose(next_plan.command_joint_angles_rad, [0.02] * 6)
        self.assertFalse(next_plan.rate_limited)

    def test_rejects_large_ik_target_step(self):
        self.clock.now_s = 1.0
        with self.assertRaisesRegex(JointCommandRejected, "largest IK target step"):
            self.limiter.check([0.051, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_rate_limits_instead_of_rejecting_excessive_joint_speed(self):
        self.clock.now_s = 0.01
        plan = self.limiter.check([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.assertTrue(plan.rate_limited)
        self.assertAlmostEqual(plan.command_velocity_rad_s[0], 0.02)
        self.assertAlmostEqual(plan.command_joint_angles_rad[0], 0.0002)

    def test_command_velocity_and_acceleration_remain_bounded(self):
        self.clock.now_s = 0.1
        first = self.limiter.check([0.04, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.limiter.accept(first)
        self.clock.now_s = 0.2
        second = self.limiter.check([0.04, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.assertLessEqual(abs(first.command_velocity_rad_s[0]), 0.5)
        self.assertLessEqual(abs(second.command_velocity_rad_s[0]), 0.5)
        self.assertLessEqual(
            abs(second.command_velocity_rad_s[0] - first.command_velocity_rad_s[0]),
            2.0 * 0.1 + 1e-12,
        )

    def test_hard_step_compares_consecutive_ik_targets_not_lagging_command(self):
        self.clock.now_s = 0.01
        first = self.limiter.check([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.limiter.accept(first)

        self.clock.now_s = 0.02
        second = self.limiter.check([0.055, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.assertTrue(second.rate_limited)
        self.assertGreater(
            second.largest_tracking_error_rad,
            self.limiter.max_joint_step_rad,
        )

    def test_logged_wrist_jump_is_smoothed_instead_of_rejected(self):
        initial = [-0.427106, 0.080081, -0.166450, 0.357908, 0.298834, 0.058223]
        desired = [-0.435534, 0.081376, -0.167985, 0.373037, 0.303177, 0.040420]
        limiter = JointCommandLimiter(
            initial_joint_angles_rad=initial,
            max_joint_step_rad=0.05,
            max_joint_speed_rad_s=0.5,
            max_joint_acceleration_rad_s2=2.0,
            monotonic=self.clock,
        )
        self.clock.now_s = 0.020225

        plan = limiter.check(desired)

        self.assertTrue(plan.rate_limited)
        self.assertLessEqual(max(abs(value) for value in plan.command_velocity_rad_s), 0.5)
        self.assertLess(
            max(
                abs(command - start)
                for command, start in zip(plan.command_joint_angles_rad, initial)
            ),
            0.001,
        )

    def test_hold_resets_velocity_and_desired_origin(self):
        self.clock.now_s = 0.1
        moving = self.limiter.check([0.04, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.limiter.accept(moving)
        self.clock.now_s = 0.2
        self.limiter.hold()
        self.clock.now_s = 0.3

        held = self.limiter.check(self.limiter.last_command_joint_angles_rad)

        self.assertFalse(held.rate_limited)
        np.testing.assert_allclose(held.command_velocity_rad_s, np.zeros(6))

    def test_rejects_non_finite_or_wrong_dimension(self):
        with self.assertRaisesRegex(JointCommandRejected, "six finite"):
            self.limiter.check([0.0] * 5)
        with self.assertRaisesRegex(JointCommandRejected, "six finite"):
            self.limiter.check([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")])


if __name__ == "__main__":
    unittest.main()
