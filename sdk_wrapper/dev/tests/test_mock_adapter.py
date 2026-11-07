import unittest

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.contracts import ErrorCode, RunMode, TaskStatus


class MockAdapterTests(unittest.TestCase):
    def test_reads_mock_robot_state(self):
        result = MockAdapter().get_robot_state()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.mode, RunMode.MOCK)
        self.assertEqual(result.data["motion_state"], "idle")

    def test_starts_registered_gesture_and_completes_task(self):
        adapter = MockAdapter(task_polls_until_succeeded=0)

        started = adapter.execute_gesture("wave_right")
        completed = adapter.get_task_status(started.task_id)

        self.assertEqual(started.status, TaskStatus.RUNNING)
        self.assertTrue(started.task_id)
        self.assertEqual(started.mode, RunMode.MOCK)
        self.assertEqual(completed.status, TaskStatus.SUCCEEDED)
        self.assertEqual(completed.task_id, started.task_id)

    def test_rejects_invalid_gesture_parameters(self):
        adapter = MockAdapter()

        for gesture_id in ("", "wave_left", None):
            with self.subTest(gesture_id=gesture_id):
                result = adapter.execute_gesture(gesture_id)
                self.assertEqual(result.status, TaskStatus.REJECTED)
                self.assertEqual(result.error_code, ErrorCode.INVALID_ARGUMENT)
                self.assertEqual(result.mode, RunMode.MOCK)

    def test_starts_navigation_and_transitions_from_running_to_succeeded(self):
        adapter = MockAdapter(task_polls_until_succeeded=1)

        started = adapter.navigate_to_place("demo_table")
        still_running = adapter.get_task_status(started.task_id)
        completed = adapter.get_task_status(started.task_id)

        self.assertEqual(started.status, TaskStatus.RUNNING)
        self.assertEqual(still_running.status, TaskStatus.RUNNING)
        self.assertEqual(completed.status, TaskStatus.SUCCEEDED)
        self.assertEqual(completed.mode, RunMode.MOCK)

    def test_rejects_unknown_navigation_place(self):
        result = MockAdapter().navigate_to_place("unregistered_place")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.INVALID_ARGUMENT)
        self.assertEqual(result.mode, RunMode.MOCK)

    def test_cancels_running_task_and_reports_confirmed_state(self):
        adapter = MockAdapter()
        started = adapter.navigate_to_place("demo_table")

        cancelled = adapter.cancel_task(started.task_id)
        queried = adapter.get_task_status(started.task_id)

        self.assertEqual(cancelled.status, TaskStatus.CANCELLED)
        self.assertEqual(queried.status, TaskStatus.CANCELLED)
        self.assertEqual(cancelled.task_id, started.task_id)

    def test_reports_simulated_execution_failure(self):
        adapter = MockAdapter(fail_next_task=True)
        started = adapter.execute_gesture("wave_right")

        result = adapter.get_task_status(started.task_id)

        self.assertEqual(started.status, TaskStatus.RUNNING)
        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.SDK_ERROR)

    def test_unknown_task_state_is_not_reported_as_stopped(self):
        adapter = MockAdapter()
        started = adapter.navigate_to_place("demo_table")
        adapter.simulate_unknown_task_state(started.task_id)

        status = adapter.get_task_status(started.task_id)
        state = adapter.get_robot_state()
        cancellation = adapter.cancel_task(started.task_id)

        self.assertEqual(status.status, TaskStatus.UNKNOWN)
        self.assertEqual(status.error_code, ErrorCode.STATE_UNCERTAIN)
        self.assertEqual(status.task_id, started.task_id)
        self.assertEqual(state.status, TaskStatus.UNKNOWN)
        self.assertEqual(state.data["motion_state"], "unknown")
        self.assertEqual(cancellation.status, TaskStatus.UNKNOWN)
        self.assertEqual(cancellation.task_id, started.task_id)

    def test_rejects_invalid_task_identifier(self):
        result = MockAdapter().get_task_status("")

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.INVALID_ARGUMENT)


if __name__ == "__main__":
    unittest.main()
