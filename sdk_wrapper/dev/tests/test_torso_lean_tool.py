import math
import unittest
from types import SimpleNamespace

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.torso_lean import (
    LEG_JOINT_NAMES,
    TorsoLeanTool,
)


class FakeTorsoRobot:
    def __init__(self):
        self.positions = [0.125, 0.375, 0.25, 0.0, 0.0]
        self.head_x = 0.11
        self.head_pitch = 0.0
        self.head_yaw = 0.0
        self.commands = []
        self.timestamp_ns = 1000

    def get_joint_states(self, groups, names):
        if groups or tuple(names) != LEG_JOINT_NAMES:
            raise AssertionError((groups, names))
        self.timestamp_ns += 1
        return [
            SimpleNamespace(position=position, timestamp_ns=self.timestamp_ns)
            for position in self.positions
        ]

    def get_transform(self, target, source, timestamp_ns=0, timeout_ms=500):
        if (target, source) != ("base_link", "head_base_link"):
            raise AssertionError((target, source))
        return [
            self.head_x,
            0.0,
            1.05,
            0.0,
            math.sin(self.head_pitch / 2),
            math.sin(self.head_yaw / 2),
            math.cos(self.head_pitch / 2) * math.cos(self.head_yaw / 2),
        ], self.timestamp_ns

    def set_joint_positions(self, **kwargs):
        self.commands.append(kwargs)
        target = float(kwargs["joint_positions"][0])
        old = self.positions[LEG_JOINT_NAMES.index(TARGET_JOINT_NAME)]
        self.positions[LEG_JOINT_NAMES.index(TARGET_JOINT_NAME)] = target
        self.head_x += (target - old) * 0.35
        self.head_pitch += target - old
        return "SUCCESS"


class TorsoLeanToolTest(unittest.TestCase):
    def test_requires_authorization_and_confirmation(self):
        denied = TorsoLeanTool().lean_forward_step(operator_confirmed=True)
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

        robot = FakeTorsoRobot()
        unconfirmed = TorsoLeanTool(robot=robot).lean_forward_step()
        self.assertEqual(unconfirmed.status, TaskStatus.REJECTED)
        self.assertEqual(robot.commands, [])

    def test_confirmed_step_is_disabled_after_axis_test(self):
        robot = FakeTorsoRobot()
        result = TorsoLeanTool(robot=robot).lean_forward_step(
            operator_confirmed=True
        )
        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.NOT_SUPPORTED)
        self.assertEqual(robot.commands, [])

    def test_read_only_state_is_still_available(self):
        robot = FakeTorsoRobot()
        result = TorsoLeanTool(robot=robot).get_torso_lean_state()
        self.assertTrue(result.succeeded, result.to_dict())
        self.assertEqual(robot.positions, [0.125, 0.375, 0.25, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
