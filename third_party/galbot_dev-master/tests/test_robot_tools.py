import unittest
from unittest.mock import patch

from adapters import MockAdapter
from adapters.galbot_adapter import GalbotAdapter
from contracts import ErrorCode, RunMode, TaskStatus, ok, running
from robot_tools.motion import MotionTool
from robot_tools.navigation import NavigationTool
from robot_tools.safety import RobotSafety


class RobotToolTests(unittest.TestCase):
    def test_motion_tool_starts_allowlisted_gesture(self):
        adapter = MockAdapter(task_polls_until_succeeded=0)

        result = MotionTool(adapter).execute_gesture("wave_right")

        self.assertEqual(result.status, TaskStatus.RUNNING)
        self.assertEqual(result.mode, RunMode.MOCK)
        self.assertEqual(result.data["target"], "wave_right")

    def test_motion_tool_rejects_unregistered_gesture(self):
        adapter = MockAdapter()

        result = MotionTool(adapter).execute_gesture("wave_left")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.INVALID_ARGUMENT)

    def test_navigation_tools_share_task_tracking(self):
        adapter = MockAdapter(task_polls_until_succeeded=0)
        safety = RobotSafety(adapter)
        motion = MotionTool(adapter, safety=safety)
        navigation = NavigationTool(adapter, safety=safety)

        started = motion.execute_gesture("wave_right")
        conflicting = navigation.navigate_to_place("demo_table")
        completed = navigation.get_task_status(started.task_id)
        destination = navigation.navigate_to_place("demo_table")

        self.assertEqual(conflicting.status, TaskStatus.REJECTED)
        self.assertEqual(conflicting.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(completed.status, TaskStatus.SUCCEEDED)
        self.assertEqual(destination.status, TaskStatus.RUNNING)

    def test_navigation_tool_rejects_unregistered_place(self):
        result = NavigationTool(MockAdapter()).navigate_to_place("unknown")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.INVALID_ARGUMENT)

    def test_real_robot_is_rejected_without_authorization(self):
        adapter = GalbotAdapter()

        through_tool = MotionTool(adapter).execute_gesture("wave_right")
        through_adapter = adapter.execute_gesture("wave_right")

        self.assertEqual(through_tool.status, TaskStatus.REJECTED)
        self.assertEqual(through_tool.error_code, ErrorCode.PERMISSION_DENIED)
        self.assertEqual(through_adapter.status, TaskStatus.REJECTED)
        self.assertEqual(through_adapter.error_code, ErrorCode.PERMISSION_DENIED)

    def test_expected_mode_mismatch_is_rejected_before_motion(self):
        adapter = MockAdapter()
        safety = RobotSafety(adapter, expected_mode=RunMode.REAL)

        result = MotionTool(adapter, safety=safety).execute_gesture("wave_right")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.PERMISSION_DENIED)

    def test_preflight_rejects_robot_that_is_already_moving(self):
        adapter = MockAdapter()
        adapter.get_robot_state = lambda: ok(
            "Simulated external motion.",
            mode=RunMode.MOCK,
            data={"motion_state": "moving"},
        )

        result = MotionTool(adapter).execute_gesture("wave_right")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)

    def test_timeout_returns_cancelled_only_after_confirmation(self):
        adapter = MockAdapter(task_polls_until_succeeded=10)

        with patch("robot_tools.safety.time.monotonic", side_effect=[10.0, 12.0]):
            safety = RobotSafety(adapter, task_timeout_seconds=1)
            navigation = NavigationTool(adapter, safety=safety)
            started = navigation.navigate_to_place("demo_table")
            result = navigation.get_task_status(started.task_id)

        self.assertEqual(result.status, TaskStatus.CANCELLED)
        self.assertEqual(result.task_id, started.task_id)

    def test_explicit_real_authorization_does_not_invent_sdk_calls(self):
        """真机授权只放开"允许调用 SDK"，不会凭空造出一个动作。

        SDK 没有高层"挥手"接口（只有 execute_joint_trajectory），
        所以这里必须是 rejected + not_supported，而不是伪造 running/succeeded。
        """
        adapter = GalbotAdapter(enable_real_control=True, real_authorized=True)

        result = adapter.execute_gesture("wave_right")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.NOT_SUPPORTED)
        self.assertEqual(result.mode, RunMode.REAL)
        self.assertIsNone(result.task_id)

    def test_real_mode_without_authorization_is_denied(self):
        """未授权时连 SDK 都不该 import，直接 permission_denied。"""
        adapter = GalbotAdapter()

        for call in (
            adapter.get_robot_state,
            lambda: adapter.execute_gesture("wave_right"),
            lambda: adapter.navigate_to_place("demo_table"),
            lambda: adapter.get_task_status("whatever"),
            lambda: adapter.cancel_task("whatever"),
        ):
            result = call()
            self.assertEqual(result.status, TaskStatus.REJECTED)
            self.assertEqual(result.error_code, ErrorCode.PERMISSION_DENIED)

    def test_timeout_without_confirmed_cancellation_returns_unknown(self):
        adapter = MockAdapter(task_polls_until_succeeded=10)

        with patch("robot_tools.safety.time.monotonic", side_effect=[10.0, 12.0]):
            safety = RobotSafety(adapter, task_timeout_seconds=1)
            navigation = NavigationTool(adapter, safety=safety)
            started = navigation.navigate_to_place("demo_table")
            with patch.object(
                adapter,
                "cancel_task",
                return_value=running(
                    "Cancellation not confirmed.",
                    mode=RunMode.MOCK,
                    task_id=started.task_id,
                ),
            ):
                result = navigation.get_task_status(started.task_id)

        self.assertEqual(result.status, TaskStatus.UNKNOWN)
        self.assertEqual(result.error_code, ErrorCode.TIMEOUT)
        self.assertEqual(result.task_id, started.task_id)

    def test_unknown_task_blocks_new_motion_across_tools(self):
        adapter = MockAdapter()
        safety = RobotSafety(adapter)
        navigation = NavigationTool(adapter, safety=safety)
        motion = MotionTool(adapter, safety=safety)
        started = navigation.navigate_to_place("demo_table")
        adapter.simulate_unknown_task_state(started.task_id)

        status = navigation.get_task_status(started.task_id)
        new_motion = motion.execute_gesture("wave_right")

        self.assertEqual(status.status, TaskStatus.UNKNOWN)
        self.assertEqual(new_motion.status, TaskStatus.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
