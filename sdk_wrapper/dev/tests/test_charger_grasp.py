import math
import unittest

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus, failed, ok, rejected
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


class FakeHeadChargerTool(FakeChargerTool):
    def __init__(self):
        target_3d = {
            "available": True,
            "center_m": [1.0, 0.0, 0.3],
            "surface_normal_base": [0.0, 0.0, 1.0],
            "long_axis_base": [1.0, 0.0, 0.0],
            "short_axis_base": [0.0, 1.0, 0.0],
        }
        data = {
            "head_handoff": {
                "usable_for_wrist_handoff": True,
                "reasons": [],
                "target_3d": target_3d,
                "target_safety": {"safe": True, "reasons": []},
            }
        }
        super().__init__(data)

    def detect_charger_head(self):
        self.calls += 1
        return ok("head observation", "mock", data=self.data)


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
        tool.config["decomposed_motion_settle_s"] = 0.0
        return tool, adapter, charger_tool

    def test_plan_generates_pregrasp_pose_without_motion(self):
        tool, adapter, charger_tool = self.make_tool()

        result = tool.plan_wrist_charger_pregrasp()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(charger_tool.calls, 1)
        self.assertFalse(result.data["motion_dispatched"])
        self.assertEqual(
            result.data["grasp_plan"]["target_kind"],
            "wrist_charger_diagonal_observation",
        )
        offset = [
            float(tcp) - float(center)
            for tcp, center in zip(
                result.data["grasp_plan"]["tcp_observation_position_m"],
                result.data["grasp_plan"]["surface_center_m"],
            )
        ]
        self.assertAlmostEqual(
            sum(value * value for value in offset) ** 0.5,
            0.05,
            places=6,
        )
        self.assertEqual(adapter.ee_moves, [])

    def test_head_camera_plans_initial_wrist_observation_without_motion(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        charger_tool = FakeHeadChargerTool()
        tool = ChargerGraspTool(
            perception,
            charger_tool=charger_tool,
            motion_tool=MotionTool(adapter),
        )
        tool.config["decomposed_motion_settle_s"] = 0.0

        result = tool.plan_wrist_observation_from_head()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(charger_tool.calls, 1)
        self.assertFalse(result.data["motion_dispatched"])
        self.assertEqual(
            result.data["camera_plan"]["camera_position_base"],
            [1.0, 0.0, 0.6],
        )
        self.assertIn("ik_preflight", result.data)
        self.assertEqual(adapter.ee_moves, [])

    def test_head_guided_move_reaches_camera_observation_pose(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        charger_tool = FakeHeadChargerTool()
        tool = ChargerGraspTool(
            perception,
            charger_tool=charger_tool,
            motion_tool=MotionTool(adapter),
        )

        result = tool.move_wrist_to_observation_from_head(
            operator_confirmed=True
        )

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["motion_dispatched"])
        self.assertTrue(result.data["pose_matched"])
        self.assertGreater(len(adapter.ee_moves), 1)
        self.assertTrue(
            all(
                move["actuate"] == "with_chain_only"
                for move in adapter.ee_moves
            )
        )
        self.assertEqual(result.data["next_action"], "wrist_plan")

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
        self.assertEqual(charger_tool.calls, 2)
        self.assertGreater(len(adapter.ee_moves), 1)
        self.assertTrue(result.data["motion_dispatched"])
        self.assertTrue(result.data["pose_matched"])
        self.assertFalse(result.data["grasp_completed"])
        self.assertTrue(result.data["plan"]["grasp_plan"]["safe_to_plan"])
        self.assertTrue(result.data["descent_plan"]["safe_to_plan"])
        self.assertAlmostEqual(
            adapter.gripper_widths["right_gripper"],
            0.11,
        )
        self.assertFalse(result.data["grasp_started"])
        transition = result.data["motion"]["data"]["pose_transition"]
        self.assertEqual(transition["strategy"], "translate_then_rotate")
        self.assertEqual(transition["actuate"], "with_chain_only")
        self.assertTrue(transition["collision_check"])
        self.assertGreater(transition["translation_steps"], 0)
        self.assertGreater(transition["rotation_steps"], 0)
        kinds = [
            step["requested"]["kind"]
            for step in transition["steps"]
        ]
        self.assertEqual(
            kinds,
            sorted(kinds, key=lambda kind: kind != "translate"),
        )

    def test_confirmed_move_reuses_recent_preflight_plan(self):
        tool, adapter, charger_tool = self.make_tool()

        preflight = tool.plan_wrist_charger_pregrasp()
        result = tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)

        self.assertEqual(preflight.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["plan_source"], "recent_preflight")
        self.assertEqual(charger_tool.calls, 2)
        self.assertGreater(len(adapter.ee_moves), 1)

    def test_execute_grasp_descends_closes_and_lifts(self):
        tool, adapter, charger_tool = self.make_tool()
        move = tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)

        result = tool.execute_wrist_charger_grasp(operator_confirmed=True)

        self.assertEqual(move.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(charger_tool.calls, 3)
        self.assertTrue(result.data["motion_dispatched"])
        self.assertTrue(result.data["grasp_started"])
        self.assertEqual(
            result.data["grasp_state"],
            "closed_and_lifted_unverified",
        )
        self.assertFalse(result.data["grasp_completed"])
        self.assertGreaterEqual(len(adapter.ee_moves), 3)
        self.assertAlmostEqual(
            adapter.gripper_widths["right_gripper"],
            0.025,
        )

    def test_contact_limited_close_timeout_is_recorded_and_lift_still_runs(self):
        tool, adapter, _charger_tool = self.make_tool()
        tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)
        calls = {"count": 0}
        original = tool.motion_tool.set_gripper

        def set_gripper(*args, **kwargs):
            calls["count"] += 1
            if calls["count"] == 2:
                return failed(
                    ErrorCode.SDK_ERROR,
                    "close timeout",
                    "mock",
                    data={"control_status": "TIMEOUT"},
                )
            return original(*args, **kwargs)

        tool.motion_tool.set_gripper = set_gripper
        result = tool.execute_wrist_charger_grasp(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["close_contact_limited"])
        self.assertEqual(
            result.data["grasp_state"],
            "closed_and_lifted_unverified",
        )

    def test_residual_base_motion_is_stopped_and_allowed_to_settle(self):
        tool, adapter, _charger_tool = self.make_tool()
        tool.config["idle_wait_timeout_s"] = 0.5
        tool.config["idle_poll_interval_s"] = 0.0
        states = [
            ok(
                "moving",
                "mock",
                data={
                    "motion_state": "moving",
                    "base_speed": 0.08,
                    "active_task_ids": [],
                },
            ),
            ok(
                "idle",
                "mock",
                data={
                    "motion_state": "idle",
                    "base_speed": 0.01,
                    "active_task_ids": [],
                },
            ),
        ]
        stop_calls = []
        tool.motion_tool.get_robot_state = lambda: states.pop(0)
        tool.motion_tool.stop_base = lambda: (
            stop_calls.append(True) or ok("stopped", "mock", data={})
        )

        result = tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(stop_calls, [True])
        self.assertEqual(result.data["idle_stabilization"]["attempts"], 2)
        self.assertGreater(len(adapter.ee_moves), 1)

    def test_persistent_base_motion_still_blocks_arm_command(self):
        tool, adapter, _charger_tool = self.make_tool()
        tool.config["idle_wait_timeout_s"] = 0.0
        tool.motion_tool.get_robot_state = lambda: ok(
            "moving",
            "mock",
            data={
                "motion_state": "moving",
                "base_speed": 0.20,
                "active_task_ids": [],
            },
        )
        tool.motion_tool.stop_base = lambda: ok("stopped", "mock", data={})

        result = tool.move_to_wrist_charger_pregrasp(operator_confirmed=True)

        self.assertEqual(result.status, TaskStatus.REJECTED)
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(adapter.ee_moves, [])

    def test_decomposed_motion_stops_when_success_has_no_progress(self):
        tool, _adapter, _charger_tool = self.make_tool()
        pose = [0.45, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0]
        tool.motion_tool.get_end_effector_pose = lambda chain: ok(
            "pose",
            "mock",
            data={"chain": chain, "pose": list(pose)},
        )
        tool.motion_tool.move_ee_to = lambda *args, **kwargs: ok(
            "commanded",
            "mock",
            data={"motion_status": "SUCCESS"},
        )

        result = tool._move_pose_decomposed(
            [0.50, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0],
            chain="right_arm",
        )

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(
            result.data["pose_transition"]["reason"],
            "pose_step_no_progress",
        )

    def test_decomposed_motion_rejects_an_actual_pose_branch_overshoot(self):
        tool, _adapter, _charger_tool = self.make_tool()
        start = [0.45, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0]
        overshot = [0.60, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0]
        poses = [start, overshot]
        tool.motion_tool.get_end_effector_pose = lambda chain: ok(
            "pose",
            "mock",
            data={"chain": chain, "pose": poses.pop(0)},
        )
        tool.motion_tool.move_ee_to = lambda *args, **kwargs: ok(
            "commanded",
            "mock",
            data={"motion_status": "SUCCESS"},
        )

        result = tool._move_pose_decomposed(
            [0.50, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0],
            chain="right_arm",
        )

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(
            result.data["pose_transition"]["reason"],
            "pose_step_overshoot",
        )

    def test_decomposed_motion_rejects_cumulative_translation_rotation_drift(self):
        tool, _adapter, _charger_tool = self.make_tool()
        start = [0.45, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0]

        def drift_pose(translation_m, angle_deg):
            half = math.radians(angle_deg) / 2.0
            return [
                0.45 + translation_m,
                0.20,
                0.85,
                0.0,
                0.0,
                math.sin(half),
                math.cos(half),
            ]

        poses = [
            start,
            drift_pose(0.02, 1.5),
            drift_pose(0.04, 3.0),
        ]
        tool.motion_tool.get_end_effector_pose = lambda chain: ok(
            "pose",
            "mock",
            data={"chain": chain, "pose": poses.pop(0)},
        )
        tool.motion_tool.move_ee_to = lambda *args, **kwargs: ok(
            "commanded",
            "mock",
            data={"motion_status": "SUCCESS"},
        )

        result = tool._move_pose_decomposed(
            [0.55, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0],
            chain="right_arm",
        )

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.SAFETY_LIMIT)
        self.assertEqual(
            result.data["pose_transition"]["reason"],
            "translation_phase_orientation_drift",
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
