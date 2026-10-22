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
            monotonic=self.clock,
        )

    def test_accepts_small_continuous_command(self):
        self.clock.now_s = 0.1
        angles, command_time_s = self.limiter.check([0.01] * 6)
        self.limiter.accept(angles, command_time_s)

        self.clock.now_s = 0.2
        next_angles, _ = self.limiter.check([0.02] * 6)
        np.testing.assert_allclose(next_angles, [0.02] * 6)

    def test_rejects_large_joint_step(self):
        self.clock.now_s = 1.0
        with self.assertRaisesRegex(JointCommandRejected, "largest joint step"):
            self.limiter.check([0.051, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_rejects_excessive_joint_speed(self):
        self.clock.now_s = 0.01
        with self.assertRaisesRegex(JointCommandRejected, "speed limit"):
            self.limiter.check([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_rejects_non_finite_or_wrong_dimension(self):
        with self.assertRaisesRegex(JointCommandRejected, "six finite"):
            self.limiter.check([0.0] * 5)
        with self.assertRaisesRegex(JointCommandRejected, "six finite"):
            self.limiter.check([0.0, 0.0, 0.0, 0.0, 0.0, float("nan")])


if __name__ == "__main__":
    unittest.main()
