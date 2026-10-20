import unittest

import numpy as np

from xrobotoolkit_teleop.common.quest3_reader import Quest3ReadError, Quest3Reader
from xrobotoolkit_teleop.common.xr_client import XrClient


class ManualClock:
    def __init__(self):
        self.now_ns = 0

    def __call__(self):
        return self.now_ns


class FakeXrSdk:
    """SDK-compatible mock containing only the API used by Quest3Reader."""

    def __init__(self):
        self.initialized = False
        self.close_calls = 0
        self.timestamp_ns = 1_000_000_000
        self.fail_pose_reads = False
        self.poses = {
            "headset": np.array([0.0, 1.60, 0.0, 0.0, 0.0, 0.0, 1.0]),
            "left": np.array([-0.25, 1.20, -0.30, 0.0, 0.0, 0.0, 1.0]),
            "right": np.array([0.25, 1.20, -0.30, 0.0, 0.0, 0.0, 1.0]),
        }
        self.axes = {"left": [-0.2, 0.4], "right": [0.5, -0.1]}
        self.analogs = {
            "left_trigger": 0.1,
            "right_trigger": 0.2,
            "left_grip": 0.3,
            "right_grip": 0.4,
        }
        self.buttons = {
            "A": True,
            "B": False,
            "X": False,
            "Y": True,
            "left_menu_button": False,
            "right_menu_button": True,
            "left_axis_click": True,
            "right_axis_click": False,
        }

    def init(self):
        self.initialized = True

    def close(self):
        self.close_calls += 1

    def get_time_stamp_ns(self):
        return self.timestamp_ns

    def get_headset_pose(self):
        if self.fail_pose_reads:
            raise RuntimeError("mock device disconnected")
        return self.poses["headset"]

    def get_left_controller_pose(self):
        return self.poses["left"]

    def get_right_controller_pose(self):
        return self.poses["right"]

    def get_left_trigger(self):
        return self.analogs["left_trigger"]

    def get_right_trigger(self):
        return self.analogs["right_trigger"]

    def get_left_grip(self):
        return self.analogs["left_grip"]

    def get_right_grip(self):
        return self.analogs["right_grip"]

    def get_A_button(self):
        return self.buttons["A"]

    def get_B_button(self):
        return self.buttons["B"]

    def get_X_button(self):
        return self.buttons["X"]

    def get_Y_button(self):
        return self.buttons["Y"]

    def get_left_menu_button(self):
        return self.buttons["left_menu_button"]

    def get_right_menu_button(self):
        return self.buttons["right_menu_button"]

    def get_left_axis_click(self):
        return self.buttons["left_axis_click"]

    def get_right_axis_click(self):
        return self.buttons["right_axis_click"]

    def get_left_axis(self):
        return self.axes["left"]

    def get_right_axis(self):
        return self.axes["right"]


class Quest3ReaderTest(unittest.TestCase):
    def setUp(self):
        self.sdk = FakeXrSdk()
        self.clock = ManualClock()
        self.client = XrClient(sdk_module=self.sdk)
        self.reader = Quest3Reader(
            self.client,
            stale_timeout_s=0.5,
            monotonic_ns=self.clock,
        )

    def tearDown(self):
        self.client.close()

    def test_reads_complete_snapshot(self):
        snapshot = self.reader.read()

        self.assertTrue(snapshot.tracking_valid)
        self.assertTrue(snapshot.timestamp_advanced)
        self.assertEqual(snapshot.timestamp_ns, 1_000_000_000)
        self.assertEqual(snapshot.poses["headset"].position, (0.0, 1.6, 0.0))
        self.assertEqual(snapshot.poses["right_controller"].orientation_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(snapshot.analog_inputs["right_grip"], 0.4)
        self.assertTrue(snapshot.buttons["A"])
        self.assertTrue(snapshot.buttons["Y"])
        self.assertEqual(snapshot.joysticks["left"], (-0.2, 0.4))

    def test_timestamp_update_and_stale_detection(self):
        self.reader.read()
        self.clock.now_ns = 200_000_000
        self.sdk.timestamp_ns += 10_000_000
        updated = self.reader.read()
        self.assertTrue(updated.tracking_valid)
        self.assertTrue(updated.timestamp_advanced)

        self.clock.now_ns = 800_000_001
        stale = self.reader.read()
        self.assertFalse(stale.tracking_valid)
        self.assertTrue(stale.stale)
        self.assertIn("has not advanced", " ".join(stale.issues))

    def test_zero_timestamp_and_zero_pose_represent_no_device_data(self):
        self.sdk.timestamp_ns = 0
        zero_pose = np.zeros(7)
        self.sdk.poses = {"headset": zero_pose, "left": zero_pose, "right": zero_pose}

        snapshot = self.reader.read()

        self.assertFalse(snapshot.tracking_valid)
        self.assertTrue(snapshot.stale)
        self.assertTrue(all(not pose.valid for pose in snapshot.poses.values()))
        self.assertIn("no active tracking data", " ".join(snapshot.issues))

    def test_invalid_pose_is_reported(self):
        self.sdk.poses["right"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0])

        snapshot = self.reader.read()

        self.assertFalse(snapshot.tracking_valid)
        self.assertFalse(snapshot.poses["right_controller"].valid)
        self.assertIn("invalid quaternion norm", " ".join(snapshot.issues))

    def test_sdk_disconnect_error_is_wrapped(self):
        self.sdk.fail_pose_reads = True

        with self.assertRaisesRegex(Quest3ReadError, "mock device disconnected"):
            self.reader.read()

    def test_close_is_idempotent(self):
        self.client.close()
        self.client.close()
        self.assertEqual(self.sdk.close_calls, 1)


if __name__ == "__main__":
    unittest.main()
