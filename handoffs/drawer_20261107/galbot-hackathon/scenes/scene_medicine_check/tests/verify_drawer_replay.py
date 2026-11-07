"""Offline contract checks for the recorded drawer command sequence."""

import copy
import time
import unittest

from scenes.scene_medicine_check import drawer_replay


class FakeDrawerIO:
    def __init__(self, recipe, *, sag_at_step=None, sag_at_steps=None):
        self.state = {
            "tcp_base_link": list(recipe["pre_grasp_tcp_base_link"]),
            "base_map": dict(recipe["alignment_base_map"]),
            "right_arm_joints_rad": dict(recipe["alignment_right_arm_joints_rad"]),
            "right_gripper": {"width": recipe["pre_grasp_right_gripper_width_m"],
                              "effort": 2.9, "is_moving": False},
        }
        self.sag_at_step = sag_at_step
        self.sag_at_steps = sag_at_steps or {}
        self.moves = []
        self.stops = 0
        self.grips = 0

    def get_drawer_state(self):
        value = copy.deepcopy(self.state)
        value["localized"] = True
        value["sample_ns"] = time.time_ns()
        return value

    def set_gripper_command(self, *, side, width, speed, effort):
        assert side == "right"
        self.grips += 1
        self.state["right_gripper"].update(width=0.0243, effort=8.2)
        return {"accepted": True, "sdk_result": "ControlStatus.SUCCESS"}

    def move_end_effector_offset(self, *, arm, dx, dy, dz, roll, pitch, yaw):
        assert arm == "right_arm"
        assert roll == pitch == yaw == 0.0
        self.moves.append((dx, dy, dz))
        for axis, delta in enumerate((dx, dy, dz)):
            self.state["tcp_base_link"][axis] += delta
        if len(self.moves) == self.sag_at_step:
            self.state["tcp_base_link"][2] -= 0.008
        self.state["tcp_base_link"][2] -= self.sag_at_steps.get(len(self.moves), 0.0)
        return {"accepted": True, "sdk_result": "MotionStatus.SUCCESS"}

    def stop_controller(self, group):
        assert group == "right_arm"
        self.stops += 1
        return {"accepted": True, "sdk_result": "ControlStatus.SUCCESS"}


class DrawerReplayTests(unittest.TestCase):
    def setUp(self):
        self.recipe = drawer_replay.load_recipe()
        self.confirmed = {"operator_alignment_confirmed": True,
                          "operator_support_confirmed": True,
                          "operator_travel_confirmed": True,
                          "operator_extension_confirmed": True}

    def test_plan_and_missing_backend_never_move(self):
        self.assertFalse(drawer_replay.plan()["motion_executed"])
        result = drawer_replay.run(self.confirmed)
        self.assertFalse(result["success"])
        self.assertFalse(result["motion_executed"])

    def test_replay_uses_right_arm_eight_steps_and_no_base(self):
        io = FakeDrawerIO(self.recipe)
        result = drawer_replay.run(self.confirmed, io=io)
        self.assertTrue(result["success"], result)
        self.assertEqual(len(io.moves), 8)
        self.assertEqual(io.grips, 1)
        self.assertGreaterEqual(io.stops, 1)
        self.assertAlmostEqual(sum(move[0] for move in io.moves), -0.30, places=6)
        self.assertAlmostEqual(result["final_tcp_base_link"][1],
                               self.recipe["pre_grasp_tcp_base_link"][1], places=6)

    def test_axis_drift_stops_before_extension(self):
        io = FakeDrawerIO(self.recipe, sag_at_step=6)
        result = drawer_replay.run(self.confirmed, io=io)
        self.assertFalse(result["success"])
        self.assertEqual(len(io.moves), 6)
        self.assertEqual(io.stops, 1)
        self.assertIn("超差", result["feedback"])

    def test_supervised_rebase_reaches_30cm_but_reports_drift(self):
        io = FakeDrawerIO(self.recipe, sag_at_steps={6: 0.008, 8: 0.009})
        params = dict(self.confirmed, allow_recorded_axis_drift=True)
        result = drawer_replay.run(params, io=io)
        self.assertFalse(result["success"])
        self.assertTrue(result["command_sequence_completed"])
        self.assertFalse(result["other_axes_within_tolerance"])
        self.assertEqual(len(io.moves), 8)
        self.assertGreaterEqual(io.stops, 2)

    def test_bad_alignment_rejected_before_closing_gripper(self):
        io = FakeDrawerIO(self.recipe)
        io.state["tcp_base_link"][0] += 0.05
        result = drawer_replay.run(self.confirmed, io=io)
        self.assertFalse(result["motion_executed"])
        self.assertEqual(io.grips, 0)
        self.assertEqual(io.moves, [])


if __name__ == "__main__":
    unittest.main()
