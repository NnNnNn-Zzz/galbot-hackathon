import unittest

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.squat import MIN_LEG_HEIGHT_M, SquatTool


class FakeLegRobot:
    def __init__(self, height):
        self.height = height
        self.commands = []
        self.timestamp_ns = 456

    def get_transform(self, target, source, timestamp_ns=0, timeout_ms=100):
        self.assert_frames(target, source)
        self.timestamp_ns += 1
        return [0, 0, self.height, 0, 0, 0, 1], self.timestamp_ns

    def assert_frames(self, target, source):
        if (target, source) != ("base_link", "head_base_link"):
            raise AssertionError((target, source))

    def set_leg_height(self, target, duration, blocking, timeout):
        self.commands.append((target, duration, blocking, timeout))
        self.height = target
        return "SUCCESS"


class SquatToolTest(unittest.TestCase):
    def test_real_motion_requires_explicit_authorization_and_confirmation(self):
        denied = SquatTool().lower_leg_height_step(operator_confirmed=True)
        self.assertEqual(denied.status, TaskStatus.REJECTED)
        self.assertEqual(denied.error_code, ErrorCode.PERMISSION_DENIED)

        robot = FakeLegRobot(1.20)
        tool = SquatTool(robot=robot)
        unconfirmed = tool.lower_leg_height_step()
        self.assertEqual(unconfirmed.status, TaskStatus.REJECTED)
        self.assertEqual(robot.commands, [])

    def test_step_is_capped_and_reads_back_actual_height(self):
        robot = FakeLegRobot(1.20)
        tool = SquatTool(robot=robot)
        over_limit = tool.lower_leg_height_step(0.03, operator_confirmed=True)
        self.assertEqual(over_limit.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(robot.commands, [])

        result = tool.lower_leg_height_step(operator_confirmed=True)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["target_height_m"], 1.18)
        self.assertEqual(result.data["actual_height_m"], 1.18)
        self.assertEqual(robot.commands, [(1.18, 4.0, True, 12.0)])

    def test_lower_step_clamps_at_documented_sdk_floor(self):
        robot = FakeLegRobot(MIN_LEG_HEIGHT_M + 0.01)
        result = SquatTool(robot=robot).lower_leg_height_step(operator_confirmed=True)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(robot.commands[0][0], MIN_LEG_HEIGHT_M)
        self.assertEqual(robot.height, MIN_LEG_HEIGHT_M)


if __name__ == "__main__":
    unittest.main()
