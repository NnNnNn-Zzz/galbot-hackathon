import math
import unittest
from types import SimpleNamespace

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.head import (
    HEAD_JOINT_NAMES,
    HEAD_JOINT_NAME,
    HEAD_MEASUREMENT_FRAME,
    MAX_HEAD_PITCH_JOINT_RAD,
    MAX_STEP_RAD,
    HeadLookTool,
)
from scripts.keyboard_head_control import format_step_success


class FakeHeadRobot:
    def __init__(self):
        self.positions = [-0.01, -0.02]
        self.head_pitch = -0.02
        self.commands = []
        self.timestamp_ns = 100

    def get_joint_states(self, groups, names):
        if groups or tuple(names) != HEAD_JOINT_NAMES:
            raise AssertionError((groups, names))
        self.timestamp_ns += 1
        return [
            SimpleNamespace(position=value, timestamp_ns=self.timestamp_ns)
            for value in self.positions
        ]

    def get_transform(self, target, source, timestamp_ns=0, timeout_ms=500):
        if (target, source) != ("base_link", HEAD_MEASUREMENT_FRAME):
            raise AssertionError((target, source))
        return [
            0.11,
            0.0,
            1.42,
            0.0,
            math.sin(self.head_pitch / 2),
            0.0,
            math.cos(self.head_pitch / 2),
        ], self.timestamp_ns

    def set_joint_positions(self, **kwargs):
        self.commands.append(kwargs)
        target = float(kwargs["joint_positions"][0])
        old = self.positions[1]
        self.positions[1] = target
        self.head_pitch += target - old
        return "SUCCESS"


class HeadLookToolTest(unittest.TestCase):
    def test_requires_authorization_and_confirmation(self):
        denied = HeadLookTool().lower_head_step(operator_confirmed=True)
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

        robot = FakeHeadRobot()
        unconfirmed = HeadLookTool(robot=robot).lower_head_step()
        self.assertEqual(unconfirmed.status, TaskStatus.REJECTED)
        self.assertEqual(robot.commands, [])

    def test_lower_head_commands_only_head_joint2(self):
        robot = FakeHeadRobot()
        result = HeadLookTool(robot=robot).lower_head_step(
            operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(len(robot.commands), 1)
        command = robot.commands[0]
        self.assertEqual(command["joint_names"], [HEAD_JOINT_NAME])
        self.assertEqual(command["joint_groups"], [])
        self.assertEqual(command["joint_positions"], [0.0])
        self.assertTrue(command["is_blocking"])
        self.assertEqual(command["speed_rad_s"], 0.05)

    def test_raise_head_moves_in_opposite_direction(self):
        robot = FakeHeadRobot()
        result = HeadLookTool(robot=robot).raise_head_step(
            operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(robot.commands[0]["joint_positions"], [-0.04])

    def test_rejects_step_above_cap(self):
        robot = FakeHeadRobot()
        result = HeadLookTool(robot=robot).lower_head_step(
            MAX_STEP_RAD + 0.001, operator_confirmed=True
        )
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(robot.commands, [])

    def test_clamps_at_software_limit(self):
        robot = FakeHeadRobot()
        robot.positions[1] = MAX_HEAD_PITCH_JOINT_RAD - 0.005
        robot.head_pitch = robot.positions[1]
        result = HeadLookTool(robot=robot).lower_head_step(
            operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(
            robot.commands[0]["joint_positions"], [MAX_HEAD_PITCH_JOINT_RAD]
        )

    def test_wrong_pitch_direction_returns_unknown(self):
        robot = FakeHeadRobot()

        def wrong_direction(**kwargs):
            old = robot.positions[1]
            robot.positions[1] = float(kwargs["joint_positions"][0])
            robot.head_pitch -= robot.positions[1] - old
            return "SUCCESS"

        robot.set_joint_positions = wrong_direction
        result = HeadLookTool(robot=robot).lower_head_step(
            operator_confirmed=True
        )
        self.assertEqual(result.status, TaskStatus.UNKNOWN)
        self.assertEqual(result.error_code, ErrorCode.STATE_UNCERTAIN)

    def test_keyboard_success_formatter(self):
        result = SimpleNamespace(
            data={
                "actual_joint_rad": 0.03,
                "head_pitch_delta_rad": math.radians(0.5),
                "head_joint1_drift_rad": 0.00002,
            }
        )
        text = format_step_success(result)
        self.assertIn("head_joint2=0.0300 rad", text)
        self.assertIn("pitch_delta=+0.500°", text)
        self.assertIn("other_head_drift=0.00002 rad", text)


if __name__ == "__main__":
    unittest.main()
