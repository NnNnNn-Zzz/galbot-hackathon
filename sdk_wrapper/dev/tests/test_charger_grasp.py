import unittest

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus, ok, rejected
from sdk_wrapper.dev.robot_tools.charger_grasp import ChargerGraspTool
from sdk_wrapper.dev.robot_tools.motion import MotionTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


def observation_data(*, eligible=True):
    surface = {
        "available": True,
        "frame": "base_link",
        "surface_center_m": [0.45, -0.2, 0.70],
        "surface_normal_base": [0.0, 0.0, 1.0],
        "long_edge_axis_base": [1.0, 0.0, 0.0],
        "short_edge_axis_base": [0.0, 1.0, 0.0],
        "gripper_long_axis_base": [1.0, 0.0, 0.0],
        "jaw_closing_axis_base": [0.0, 1.0, 0.0],
        "long_dimension_m": 0.14,
        "short_dimension_m": 0.08,
        "pregrasp_position_m": [0.45, -0.2, 0.82],
        "safe_standoff_m": 0.12,
    }
    return {
        "camera_id": "right_arm",
        "confidence": 0.91,
        "image_space": {"fully_visible": True, "short_edge_px": 80.0},
        "depth": {"available": True, "valid_ratio": 0.98},
        "surface": surface,
        "eligible_for_precision": bool(eligible),
        "rejection_reasons": [] if eligible else ["target_touches_image_edge"],
        "depth_alignment_verified": True,
        "overlay_ref": "/tmp/overlay.png",
        "observation_ref": "/tmp/observation.json",
    }


class FakeChargerTool:
    def __init__(self, data):
        self.data = data
        self.calls = 0

    def segment_charger_top_wrist(self):
        self.calls += 1
        return ok("wrist observation", "mock", data=self.data)


class ChargerGraspToolTest(unittest.TestCase):
    def make_tool(self, *, eligible=True):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        charger_tool = FakeChargerTool(observation_data(eligible=eligible))
        tool = ChargerGraspTool(
            perception,
            charger_tool=charger_tool,
            motion_tool=MotionTool(adapter),
        )
        return tool, adapter, charger_tool

    def test_plan_generates_pregrasp_pose_without_motion(self):
        tool, adapter, charger_tool = self.make_tool()

        result = tool.plan_wrist_charger_pregrasp()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(charger_tool.calls, 1)
        self.assertFalse(result.data["motion_dispatched"])
        self.assertEqual(
            result.data["end_effector_goal"][:3],
            [0.45, -0.2, 0.82],
        )
        self.assertEqual(adapter.ee_moves, [])

    def test_left_arm_override_switches_chain_and_gripper(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        tool = ChargerGraspTool(
            perception,
            charger_tool=FakeChargerTool(observation_data()),
            motion_tool=MotionTool(adapter),
            arm="left",
        )

        self.assertEqual(tool.config["chain"], "left_arm")
        self.assertEqual(tool.config["end_effector"], "left_gripper")

    def test_ineligible_wrist_observation_never_plans_motion(self):
        tool, adapter, charger_tool = self.make_tool(eligible=False)

        result = tool.plan_wrist_charger_pregrasp()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(charger_tool.calls, 1)
        self.assertEqual(adapter.ee_moves, [])

    def test_move_requires_explicit_confirmation_before_detection(self):
        tool, adapter, charger_tool = self.make_tool()

        result = tool.move_to_wrist_charger_pregrasp(operator_confirmed=False)

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.PERMISSION_DENIED)
        self.assertEqual(charger_tool.calls, 0)
        self.assertEqual(adapter.ee_moves, [])

    def test_confirmed_move_reobserves_and_commands_pregrasp_pose(self):
        tool, adapter, charger_tool = self.make_tool()

        result = tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(charger_tool.calls, 1)
        self.assertEqual(len(adapter.ee_moves), 1)
        self.assertTrue(result.data["motion_dispatched"])
        self.assertTrue(result.data["pose_matched"])
        self.assertFalse(result.data["grasp_completed"])
        self.assertEqual(
            adapter.ee_moves[0]["target"][:3],
            [0.45, -0.2, 0.82],
        )

    def test_unreachable_ik_blocks_plan_before_motion(self):
        tool, adapter, charger_tool = self.make_tool()
        tool.motion_tool.check_ee_pose_reachable = lambda *args, **kwargs: rejected(
            ErrorCode.SAFETY_LIMIT,
            "target unreachable",
            "mock",
            data={"reachable": False},
        )

        result = tool.plan_wrist_charger_pregrasp()

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertFalse(result.data["ik_preflight"]["data"]["reachable"])
        self.assertEqual(charger_tool.calls, 1)
        self.assertEqual(adapter.ee_moves, [])


if __name__ == "__main__":
    unittest.main()
