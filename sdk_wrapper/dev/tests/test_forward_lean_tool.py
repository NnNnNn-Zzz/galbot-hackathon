import math
import unittest
from types import SimpleNamespace

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.forward_lean import (
    LEAN_JOINT_NAME,
    LEG_JOINT_NAMES,
    MAX_STEP_RAD,
    ForwardLeanTool,
)


class FakeForwardLeanRobot:
    def __init__(self):
        self.positions = [0.125, 0.375, 0.25, 0.0, 0.0]
        self.head_x = 0.1
        self.head_pitch = 0.0
        self.commands = []
        self.timestamp_ns = 100

    def get_joint_states(self, groups, names):
        if groups or tuple(names) != LEG_JOINT_NAMES:
            raise AssertionError((groups, names))
        self.timestamp_ns += 1
        return [
            SimpleNamespace(position=value, timestamp_ns=self.timestamp_ns)
            for value in self.positions
        ]

    def get_transform(self, target, source, timestamp_ns=0, timeout_ms=500):
        if (target, source) != ("base_link", "head_base_link"):
            raise AssertionError((target, source))
        return [
            self.head_x,
            0.0,
            1.42,
            0.0,
            math.sin(self.head_pitch / 2),
            0.0,
            math.cos(self.head_pitch / 2),
        ], self.timestamp_ns

    def set_joint_positions(self, **kwargs):
        self.commands.append(kwargs)
        old = self.positions[2]
        target = float(kwargs["joint_positions"][0])
        self.positions[2] = target
        delta = target - old
        self.head_x += delta * 0.6
        self.head_pitch += delta
        return "SUCCESS"


class ForwardLeanToolTest(unittest.TestCase):
    def test_requires_authorization_and_confirmation(self):
        denied = ForwardLeanTool().lean_forward_step(operator_confirmed=True)
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

        robot = FakeForwardLeanRobot()
        unconfirmed = ForwardLeanTool(robot=robot).lean_forward_step()
        self.assertEqual(unconfirmed.status, TaskStatus.REJECTED)
        self.assertEqual(robot.commands, [])

    def test_positive_step_commands_only_leg_joint3(self):
        robot = FakeForwardLeanRobot()
        result = ForwardLeanTool(robot=robot).lean_forward_step(
            operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(len(robot.commands), 1)
        command = robot.commands[0]
        self.assertEqual(command["joint_names"], [LEAN_JOINT_NAME])
        self.assertEqual(command["joint_positions"], [0.26])
        self.assertEqual(command["joint_groups"], [])
        self.assertGreater(result.data["actual_delta_rad"], 0)
        self.assertGreater(result.data["head_x_delta_m"], 0)

    def test_negative_step_is_return_direction(self):
        robot = FakeForwardLeanRobot()
        result = ForwardLeanTool(robot=robot).lean_backward_step(
            operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(robot.commands[0]["joint_positions"], [0.24])

    def test_rejects_step_above_cap(self):
        robot = FakeForwardLeanRobot()
        result = ForwardLeanTool(robot=robot).lean_forward_step(
            MAX_STEP_RAD + 0.001, operator_confirmed=True
        )
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(robot.commands, [])

    def test_wrong_direction_returns_unknown(self):
        robot = FakeForwardLeanRobot()

        def wrong_direction(**kwargs):
            old = robot.positions[2]
            robot.positions[2] = float(kwargs["joint_positions"][0])
            delta = robot.positions[2] - old
            robot.head_x -= delta
            robot.head_pitch -= delta
            return "SUCCESS"

        robot.set_joint_positions = wrong_direction
        result = ForwardLeanTool(robot=robot).lean_forward_step(
            operator_confirmed=True
        )
        self.assertEqual(result.status, TaskStatus.UNKNOWN)
        self.assertEqual(result.error_code, ErrorCode.STATE_UNCERTAIN)

    def test_restores_only_leg_joint3(self):
        robot = FakeForwardLeanRobot()
        robot.positions[2] = 0.29
        result = ForwardLeanTool(robot=robot).restore_lean_joint(
            0.25, operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(robot.commands[0]["joint_names"], [LEAN_JOINT_NAME])
        self.assertAlmostEqual(robot.positions[2], 0.25)


if __name__ == "__main__":
    unittest.main()
