import math
import unittest
from types import SimpleNamespace

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.leg_joint_control import (
    CONTROL_JOINT_NAMES,
    LEG_STATE_JOINT_NAMES,
    MAX_STEP_RAD,
    LegJointControlTool,
)


class FakeLegJointRobot:
    def __init__(self):
        self.positions = [0.125, 0.375, 0.25, 0.0, 0.0]
        self.height = 1.0516
        self.torso_pitch = 0.0
        self.commands = []
        self.timestamp_ns = 100

    def get_joint_states(self, groups, names):
        if groups or tuple(names) != LEG_STATE_JOINT_NAMES:
            raise AssertionError((groups, names))
        self.timestamp_ns += 1
        return [
            SimpleNamespace(position=value, timestamp_ns=self.timestamp_ns)
            for value in self.positions
        ]

    def get_transform(self, target, source, timestamp_ns=0, timeout_ms=500):
        if target != "base_link":
            raise AssertionError(target)
        if source == "head_base_link":
            return [
                0.1,
                0.0,
                self.height,
                0.0,
                0.0,
                0.0,
                1.0,
            ], self.timestamp_ns
        if source == "torso_base_link":
            return [
                0.1,
                0.0,
                0.7,
                0.0,
                math.sin(self.torso_pitch / 2),
                0.0,
                math.cos(self.torso_pitch / 2),
            ], self.timestamp_ns
        raise AssertionError(source)

    def set_joint_positions(self, **kwargs):
        self.commands.append(kwargs)
        names = list(kwargs["joint_names"])
        values = list(kwargs["joint_positions"])
        for name, value in zip(names, values):
            index = LEG_STATE_JOINT_NAMES.index(name)
            delta = float(value) - self.positions[index]
            self.positions[index] = float(value)
            if name in CONTROL_JOINT_NAMES:
                self.torso_pitch += delta * 0.5
                self.height -= delta * 0.05
        return "SUCCESS"


class LegJointControlToolTest(unittest.TestCase):
    def test_requires_authorization_and_confirmation(self):
        denied = LegJointControlTool().move_leg_joint_step(
            "leg_joint1", 0.005, operator_confirmed=True
        )
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

        robot = FakeLegJointRobot()
        unconfirmed = LegJointControlTool(robot=robot).move_leg_joint_step(
            "leg_joint1", 0.005
        )
        self.assertEqual(unconfirmed.status, TaskStatus.REJECTED)
        self.assertEqual(robot.commands, [])

    def test_commands_only_selected_joint(self):
        robot = FakeLegJointRobot()
        result = LegJointControlTool(robot=robot).move_leg_joint_step(
            "leg_joint2", 0.005, operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(len(robot.commands), 1)
        command = robot.commands[0]
        self.assertEqual(command["joint_names"], ["leg_joint2"])
        self.assertEqual(command["joint_positions"], [0.38])
        self.assertEqual(command["joint_groups"], [])
        self.assertTrue(command["is_blocking"])

    def test_rejects_unsupported_and_oversized_joints(self):
        robot = FakeLegJointRobot()
        tool = LegJointControlTool(robot=robot)
        unsupported = tool.move_leg_joint_step(
            "leg_joint4", 0.005, operator_confirmed=True
        )
        oversized = tool.move_leg_joint_step(
            "leg_joint1", MAX_STEP_RAD + 0.001, operator_confirmed=True
        )
        self.assertEqual(unsupported.error_code, ErrorCode.INVALID_ARGUMENT)
        self.assertEqual(oversized.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(robot.commands, [])

    def test_uncommanded_drift_returns_unknown(self):
        robot = FakeLegJointRobot()
        original = robot.set_joint_positions

        def drifting_command(**kwargs):
            status = original(**kwargs)
            robot.positions[4] += 0.02
            return status

        robot.set_joint_positions = drifting_command
        result = LegJointControlTool(robot=robot).move_leg_joint_step(
            "leg_joint1", 0.005, operator_confirmed=True
        )
        self.assertEqual(result.status, TaskStatus.UNKNOWN)
        self.assertEqual(result.error_code, ErrorCode.STATE_UNCERTAIN)

    def test_restores_all_three_to_baseline(self):
        robot = FakeLegJointRobot()
        targets = {name: value for name, value in zip(CONTROL_JOINT_NAMES, [0.1, 0.4, 0.2])}
        result = LegJointControlTool(robot=robot).restore_leg_joints(
            targets, operator_confirmed=True
        )
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(
            robot.commands[0]["joint_names"], list(CONTROL_JOINT_NAMES)
        )
        for name, target in targets.items():
            self.assertAlmostEqual(
                robot.positions[LEG_STATE_JOINT_NAMES.index(name)], target
            )


if __name__ == "__main__":
    unittest.main()
