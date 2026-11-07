import math
import unittest
from types import SimpleNamespace

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.forward_lean import (
    DEFAULT_SPEED_DEG_S,
    LEAN_JOINT_NAME,
    LEG_JOINT_NAMES,
    MAX_SPEED_DEG_S,
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
    def test_requires_authorization(self):
        denied = ForwardLeanTool().set_forward_lean(10)
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

    def test_positive_degrees_command_one_absolute_leg_joint3_target(self):
        robot = FakeForwardLeanRobot()
        result = ForwardLeanTool(robot=robot).set_forward_lean(10)
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(len(robot.commands), 1)
        command = robot.commands[0]
        self.assertEqual(command["joint_names"], [LEAN_JOINT_NAME])
        self.assertAlmostEqual(
            command["joint_positions"][0], round(0.25 + math.radians(10), 6)
        )
        self.assertEqual(command["joint_groups"], [])
        self.assertAlmostEqual(
            command["speed_rad_s"], math.radians(DEFAULT_SPEED_DEG_S)
        )
        self.assertAlmostEqual(result.data["actual_lean_deg"], 10, places=5)

    def test_negative_degrees_return_in_one_command(self):
        robot = FakeForwardLeanRobot()
        result = ForwardLeanTool(robot=robot).set_forward_lean(-8)
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertAlmostEqual(
            robot.commands[0]["joint_positions"][0],
            round(0.25 + math.radians(-8), 6),
        )

    def test_rejects_zero_angle_and_too_fast_speed(self):
        robot = FakeForwardLeanRobot()
        tool = ForwardLeanTool(robot=robot)
        zero = tool.set_forward_lean(0)
        too_fast = tool.set_forward_lean(
            10, speed_deg_s=MAX_SPEED_DEG_S + 0.1
        )
        self.assertEqual(zero.error_code, ErrorCode.INVALID_ARGUMENT)
        self.assertEqual(too_fast.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(robot.commands, [])

    def test_wrong_direction_returns_unknown(self):
        robot = FakeForwardLeanRobot()

        def wrong_direction(**kwargs):
            old = robot.positions[2]
            requested_delta = float(kwargs["joint_positions"][0]) - old
            robot.positions[2] = old - requested_delta
            return "SUCCESS"

        robot.set_joint_positions = wrong_direction
        result = ForwardLeanTool(robot=robot).set_forward_lean(4)
        self.assertEqual(result.status, TaskStatus.UNKNOWN)
        self.assertEqual(result.error_code, ErrorCode.STATE_UNCERTAIN)

    def test_restores_only_leg_joint3(self):
        robot = FakeForwardLeanRobot()
        robot.positions[2] = 0.29
        result = ForwardLeanTool(robot=robot).restore_lean_joint(0.25)
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(robot.commands[0]["joint_names"], [LEAN_JOINT_NAME])
        self.assertAlmostEqual(robot.positions[2], 0.25)


if __name__ == "__main__":
    unittest.main()
