import contextlib
import io
import unittest

from xrobotoolkit_teleop.cli.quest3_piper_ik import _validate_args, build_parser


class Quest3PiperIKCliTest(unittest.TestCase):
    def _validate(self, arguments):
        parser = build_parser()
        args = parser.parse_args(arguments)
        with contextlib.redirect_stderr(io.StringIO()):
            _validate_args(parser, args)

    def test_publish_requires_initial_joint_topic(self):
        with self.assertRaises(SystemExit):
            self._validate(["--publish-joint-topic", "/joint_right_states"])

    def test_state_and_command_topics_must_differ(self):
        with self.assertRaises(SystemExit):
            self._validate(
                [
                    "--initial-joint-topic",
                    "/joint_right_states",
                    "--publish-joint-topic",
                    "/joint_right_states",
                ]
            )

    def test_explicit_state_and_command_topics_are_accepted(self):
        self._validate(
            [
                "--initial-joint-topic",
                "/puppet/joint_right",
                "--publish-joint-topic",
                "/joint_right_states",
            ]
        )

    def test_rejects_invalid_orientation_scale_and_command_acceleration(self):
        with self.assertRaises(SystemExit):
            self._validate(["--orientation-scale", "1.1"])
        with self.assertRaises(SystemExit):
            self._validate(["--max-command-joint-acceleration-rad-s2", "0"])
        with self.assertRaises(SystemExit):
            self._validate(["--driver-speed-percent", "101"])


if __name__ == "__main__":
    unittest.main()
