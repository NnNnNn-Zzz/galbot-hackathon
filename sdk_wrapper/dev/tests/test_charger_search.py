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


class FakeStripTool:
    def __init__(self, data):
        self.data = data
        self.calls = 0

    def detect_power_strip_head(self):
        self.calls += 1
        return ok("strip", "mock", data=self.data)


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

    def test_small_strip_approaches_and_large_strip_stops_for_charger_scan(self):
        small = plan_head_search_action(
            "find_power_strip",
            strip=strip_data(width=20),
            charger=None,
            config={"strip_handoff_min_width_ratio": 0.05},
            step_count=0,
        )
        large = plan_head_search_action(
            "find_power_strip",
            strip=strip_data(width=100),
            charger=None,
            config={"strip_handoff_min_width_ratio": 0.05},
            step_count=0,
        )
        self.assertEqual(small["action"], "approach_power_strip")
        self.assertEqual(large["action"], "stop_and_scan_charger")

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

    def test_stable_stereo_range_stops_at_observation_distance(self):
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
        self.assertEqual(plan["action"], "stop_and_scan_charger")
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
            tool, adapter = self.make_tool(directory, strip_data(width=20), charger_data())
            result = tool.advance_charger_search(operator_confirmed=False)

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.PERMISSION_DENIED)
        self.assertEqual(adapter.base_moves, [])

    def test_executes_one_segment_and_redetects(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(directory, strip_data(width=20), charger_data())
            result = tool.advance_charger_search(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["executed"])
        self.assertEqual(len(adapter.base_moves), 1)
        self.assertAlmostEqual(adapter.base_moves[0]["distance_m"], 0.15)
        self.assertEqual(tool.step_count, 1)
        self.assertEqual(result.data["phase"], "approach_power_strip")

    def test_strip_observation_band_stops_and_scans_charger(self):
        with tempfile.TemporaryDirectory() as directory:
            tool, adapter = self.make_tool(directory, strip_data(width=100), charger_data())
            result = tool.advance_charger_search(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["phase"], "find_charger")
        self.assertTrue(result.data["observation_point_reached"])
        self.assertEqual(adapter.base_moves, [])
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
