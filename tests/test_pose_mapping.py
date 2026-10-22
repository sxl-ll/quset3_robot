import math
import unittest

import numpy as np

from xrobotoolkit_teleop.common.pose_mapping import CartesianPose, RelativePoseMapper


IDENTITY_ROTATION = np.eye(3)
IDENTITY_TARGET = CartesianPose((0.4, -0.1, 0.3), (0.0, 0.0, 0.0, 1.0))


def xr_pose(position=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0)):
    return np.asarray((*position, *quaternion_xyzw), dtype=np.float64)


class RelativePoseMapperTest(unittest.TestCase):
    def make_mapper(self, **kwargs):
        return RelativePoseMapper(
            IDENTITY_TARGET,
            xr_to_robot_rotation=IDENTITY_ROTATION,
            smoothing_alpha=1.0,
            **kwargs,
        )

    def test_first_clutched_sample_anchors_without_moving_target(self):
        mapper = self.make_mapper()

        result = mapper.update(xr_pose((1.0, 2.0, 3.0)), grip_value=1.0, tracking_valid=True)

        self.assertFalse(result.target_updated)
        self.assertEqual(result.reason, "clutch_anchored")
        np.testing.assert_allclose(result.target_pose.position, IDENTITY_TARGET.position)
        np.testing.assert_allclose(result.target_pose.orientation_xyzw, IDENTITY_TARGET.orientation_xyzw)

    def test_translation_direction_and_scale(self):
        mapper = self.make_mapper(position_scale=2.5)
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)

        result = mapper.update(xr_pose((0.1, -0.2, 0.3)), grip_value=1.0, tracking_valid=True)

        np.testing.assert_allclose(result.target_pose.position, [0.65, -0.6, 1.05])
        np.testing.assert_allclose(result.controller_delta_raw_units, [0.1, -0.2, 0.3])
        np.testing.assert_allclose(result.mapped_position_delta_m, [0.25, -0.5, 0.75])

    def test_configured_axis_rotation_maps_translation(self):
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        mapper = RelativePoseMapper(
            CartesianPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            xr_to_robot_rotation=rotation,
            smoothing_alpha=1.0,
        )
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)

        result = mapper.update(xr_pose((0.2, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)

        np.testing.assert_allclose(result.target_pose.position, [0.0, 0.2, 0.0], atol=1e-12)

    def test_relative_orientation_is_applied(self):
        mapper = self.make_mapper()
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)
        half_angle = math.pi / 4.0
        controller_quaternion = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

        result = mapper.update(
            xr_pose(quaternion_xyzw=controller_quaternion),
            grip_value=1.0,
            tracking_valid=True,
        )

        np.testing.assert_allclose(
            result.target_pose.orientation_xyzw,
            controller_quaternion,
            atol=1e-7,
        )

    def test_orientation_scale_reduces_controller_rotation(self):
        mapper = self.make_mapper(orientation_scale=0.25)
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)
        half_angle = math.pi / 4.0

        result = mapper.update(
            xr_pose(
                quaternion_xyzw=(
                    0.0,
                    0.0,
                    math.sin(half_angle),
                    math.cos(half_angle),
                )
            ),
            grip_value=1.0,
            tracking_valid=True,
        )

        expected_half_angle = math.pi / 16.0
        np.testing.assert_allclose(
            result.target_pose.orientation_xyzw,
            (0.0, 0.0, math.sin(expected_half_angle), math.cos(expected_half_angle)),
            atol=1e-7,
        )

    def test_clutch_reactivation_does_not_jump(self):
        mapper = self.make_mapper()
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)
        moved = mapper.update(xr_pose((0.1, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)
        released = mapper.update(xr_pose((5.0, 0.0, 0.0)), grip_value=0.0, tracking_valid=True)
        reanchored = mapper.update(xr_pose((5.0, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)
        moved_again = mapper.update(xr_pose((5.1, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)

        np.testing.assert_allclose(released.target_pose.position, moved.target_pose.position)
        np.testing.assert_allclose(reanchored.target_pose.position, moved.target_pose.position)
        np.testing.assert_allclose(moved_again.target_pose.position, [0.6, -0.1, 0.3])

    def test_invalid_tracking_holds_target_and_forces_reanchor(self):
        mapper = self.make_mapper()
        mapper.update(xr_pose(), grip_value=1.0, tracking_valid=True)
        moved = mapper.update(xr_pose((0.1, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)

        invalid = mapper.update(xr_pose((10.0, 0.0, 0.0)), grip_value=1.0, tracking_valid=False)
        recovered = mapper.update(xr_pose((10.0, 0.0, 0.0)), grip_value=1.0, tracking_valid=True)

        self.assertEqual(invalid.reason, "tracking_invalid")
        self.assertEqual(recovered.reason, "clutch_anchored")
        np.testing.assert_allclose(invalid.target_pose.position, moved.target_pose.position)
        np.testing.assert_allclose(recovered.target_pose.position, moved.target_pose.position)

    def test_invalid_controller_quaternion_cannot_update_target(self):
        mapper = self.make_mapper()

        result = mapper.update(
            xr_pose(quaternion_xyzw=(0.0, 0.0, 0.0, 0.0)),
            grip_value=1.0,
            tracking_valid=True,
        )

        self.assertEqual(result.reason, "controller_pose_invalid")
        self.assertFalse(result.target_updated)
        np.testing.assert_allclose(result.target_pose.position, IDENTITY_TARGET.position)


if __name__ == "__main__":
    unittest.main()
