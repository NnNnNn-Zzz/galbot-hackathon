import tempfile
import unittest
from pathlib import Path

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.algorithms.charger_search import plan_head_search_action
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus, failed, ok
from sdk_wrapper.dev.robot_tools.charger_search import ChargerSearchTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


def strip_data(width=40, angle=0.0, touches=False):
    return {
        "usable_for_direction": True,
        "horizontal_angle_deg": angle,
        "bbox_width_px": width,
        "bbox_xyxy": [200, 150, 200 + width, 200],
        "bbox_center_uv": [200 + width / 2, 175],
        "confidence": 0.8,
        "touches_image_edge": touches,
        "image_width": 640,
        "image_height": 512,
    }


def charger_data(ready=False, touches=False, angle=0.0):
    return {
        "confidence": 0.85,
        "bbox_xyxy": [250, 180, 390, 260],
        "image_space": {
            "touches_image_edge": touches,
            "fully_visible": not touches,
        },
        "head_handoff": {
            "usable_for_wrist_handoff": ready,
            "reasons": [] if ready else ["head_short_edge_below_minimum"],
            "horizontal_angle_deg": angle,
            "grasp_coordinate": None,
        },
    }


def metric_strip_data():
    data = strip_data(width=40)
    data["target_track_id"] = 7
    data["stereo"] = {
        "measurement_stable": True,
        "metric_distance_available": True,
        "metric_distance_m": 1.0,
        "target_3d": {
            "available": True,
            "center_m": [1.0, 0.0, 0.3],
            "plane_rms_m": 0.005,
            "surface_normal_base": [0.0, 0.0, 1.0],
        },
        "target_safety": {
            "safe": True,
            "reasons": [],
        },
    }
    return data


class FakeStripTool:
    def __init__(self, data):
        self.data = data
        self.calls = 0
        self.track_ok = True

    def detect_power_strip_head(self):
        self.calls += 1
        return ok("strip", "mock", data=self.data)

    def track_power_strip_head(self):
        self.calls += 1
        if not self.track_ok:
            return failed(
                ErrorCode.NOT_READY,
                "power_strip visual track lost; base motion must stop",
                "mock",
            )
        return ok("tracked strip", "mock", data=self.data)


class FakeChargerTool:
    def __init__(self, data):
        self.data = data
        self.calls = 0

    def detect_charger_head(self):
        self.calls += 1
        return ok("charger", "mock", data=self.data)


class UnavailableChargerTool:
    def detect_charger_head(self):
        return failed(
            ErrorCode.NOT_READY,
            "charger YOLO backend unavailable: runtime missing",
            "mock",
        )


class ChargerSearchPlannerTest(unittest.TestCase):
    def test_positive_camera_bearing_turns_robot_right(self):
        plan = plan_head_search_action(
            "find_power_strip",
            strip=strip_data(width=40, angle=8.0),
            charger=None,
            config={"center_tolerance_deg": 4, "max_turn_command_deg": 15},
            step_count=0,
        )
        self.assertEqual(plan["action"], "turn_toward_power_strip")
        self.assertLess(plan["yaw_deg"], 0)

    def test_direction_without_metric_stereo_holds_instead_of_forward_servo(self):
        plan = plan_head_search_action(
            "find_power_strip",
            strip=strip_data(width=20),
            charger=None,
            config={},
            step_count=0,
        )
        self.assertEqual(plan["action"], "hold_for_stable_metric_stereo")
        self.assertEqual(plan["forward_m"], 0.0)

    def test_ready_charger_locks_base_without_grasp_coordinate(self):
        plan = plan_head_search_action(
            "find_charger",
            strip=strip_data(width=100),
            charger={"usable_for_wrist_handoff": True},
            config={},
            step_count=2,
        )
        self.assertEqual(plan["action"], "lock_base_for_wrist")
        self.assertIsNone(plan["grasp_coordinate"])
        self.assertFalse(plan["metric_distance_used"])

    def test_stable_stereo_without_slam_pose_holds(self):
        strip = strip_data(width=20)
        strip["stereo"] = {
            "measurement_stable": True,
            "metric_distance_available": True,
            "metric_distance_m": 0.8,
        }
        plan = plan_head_search_action(
            "approach_power_strip",
            strip=strip,
            charger=None,
            config={
                "observation_distance_m": 0.8,
                "observation_distance_tolerance_m": 0.1,
            },
            step_count=1,
        )
        self.assertEqual(plan["action"], "hold_for_mapped_observation_pose")
        self.assertTrue(plan["metric_distance_used"])


class ChargerSearchToolTest(unittest.TestCase):
    def make_tool(self, directory, strip, charger):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        tool = ChargerSearchTool(
            perception,
            strip_tool=FakeStripTool(strip),
            charger_tool=FakeChargerTool(charger),
        )
        return tool, adapter

    def test_requires_confirmation_before_visual_servo_step(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(directory, metric_strip_data(), charger_data())
            result = tool.advance_charger_search(operator_confirmed=False)

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.PERMISSION_DENIED)
        self.assertEqual(adapter.base_moves, [])

    def test_starts_one_shot_slam_navigation(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(directory, metric_strip_data(), charger_data())
            result = tool.advance_charger_search(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.RUNNING)
        self.assertIsNotNone(result.task_id)
        self.assertEqual(tool.phase, "navigating_to_observation")
        self.assertEqual(adapter.base_moves, [])

    def test_monitor_tracks_target_until_slam_arrival(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = MockAdapter(task_polls_until_succeeded=1)
            perception = PerceptionTool(adapter)
            tool = ChargerSearchTool(
                perception,
                strip_tool=FakeStripTool(metric_strip_data()),
                charger_tool=FakeChargerTool(charger_data()),
            )
            started = tool.advance_charger_search(operator_confirmed=True)
            first = tool.monitor_observation_navigation(started.task_id)
            result = tool.monitor_observation_navigation(started.task_id)

        self.assertEqual(first.status, TaskStatus.RUNNING)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["phase"], "find_charger")
        self.assertTrue(result.data["navigation_arrived"])
        self.assertEqual(adapter.base_moves, [])
        self.assertGreaterEqual(adapter.base_stops, 1)

    def test_repeated_tracking_loss_cancels_slam_navigation(self):
        adapter = MockAdapter(task_polls_until_succeeded=10)
        perception = PerceptionTool(adapter)
        strip_tool = FakeStripTool(metric_strip_data())
        tool = ChargerSearchTool(
            perception,
            strip_tool=strip_tool,
            charger_tool=FakeChargerTool(charger_data()),
        )
        started = tool.advance_charger_search(operator_confirmed=True)
        strip_tool.track_ok = False
        first = tool.monitor_observation_navigation(started.task_id)
        second = tool.monitor_observation_navigation(started.task_id)
        result = tool.monitor_observation_navigation(started.task_id)

        self.assertEqual(first.status, TaskStatus.RUNNING)
        self.assertEqual(second.status, TaskStatus.RUNNING)
        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(tool.track_miss_count, 3)
        self.assertGreaterEqual(adapter.base_stops, 1)

    def test_head_charger_ready_locks_base_for_wrist_handoff(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(directory, strip_data(width=100), charger_data(ready=True))
            tool.phase = "find_charger"
            first = tool.advance_charger_search(operator_confirmed=True)
            second = tool.advance_charger_search(operator_confirmed=True)
            result = tool.advance_charger_search(operator_confirmed=True)

        self.assertEqual(first.data["plan"]["action"], "wait_for_stable_charger_handoff")
        self.assertEqual(second.data["plan"]["action"], "wait_for_stable_charger_handoff")
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["charger_ready_streak"], 3)
        self.assertTrue(result.data["base_locked"])
        self.assertTrue(result.data["wrist_handoff_requested"])
        self.assertEqual(adapter.base_moves, [])
        self.assertGreaterEqual(adapter.base_stops, 1)

    def test_head_metric_target_moves_wrist_camera_above_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(
                directory,
                strip_data(width=100),
                charger_data(ready=True),
            )
            handoff = {
                "usable_for_wrist_handoff": True,
                "target_3d": {
                    "available": True,
                    "center_m": [1.0, 0.0, 0.3],
                    "surface_normal_base": [0.0, 0.0, 1.0],
                    "long_axis_base": [1.0, 0.0, 0.0],
                    "short_axis_base": [0.0, 1.0, 0.0],
                },
                "target_safety": {"safe": True, "reasons": []},
            }
            tool.last_head_charger = {
                "head_handoff": handoff,
            }
            tool.phase = "ready_for_wrist"
            result = tool.align_wrist_from_head(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["pose_matched"])
        self.assertEqual(tool.phase, "wrist_view_aligned")
        self.assertEqual(len(adapter.ee_moves), 1)
        self.assertAlmostEqual(
            result.data["measured_camera_pose"][2],
            0.6,
            places=4,
        )

    def test_charger_backend_failure_does_not_keep_driving_forward(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        tool = ChargerSearchTool(
            perception,
            strip_tool=FakeStripTool(strip_data(width=100)),
            charger_tool=UnavailableChargerTool(),
        )
        tool.phase = "find_charger"

        result = tool.advance_charger_search(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(adapter.base_moves, [])


if __name__ == "__main__":
    unittest.main()
