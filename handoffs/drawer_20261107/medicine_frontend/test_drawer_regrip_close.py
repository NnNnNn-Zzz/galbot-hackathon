"""Offline checks for the re-grip / close geometry and gates; no robot needed."""

import json
import math
import os
import tempfile
import unittest
from pathlib import Path

from medicine_frontend import drawer_regrip_close as rclose


class RecipeTests(unittest.TestCase):
    def setUp(self):
        self.recipe = rclose.load_recipe()

    def test_release_and_closed_poses_come_from_the_pull_record(self):
        release = rclose.release_pose(self.recipe)
        closed = rclose.closed_pose(self.recipe)
        self.assertAlmostEqual(release[0], 0.450579, places=5)
        self.assertAlmostEqual(closed[0], 0.751716, places=5)
        self.assertAlmostEqual(rclose.travel_to_closed(self.recipe), 0.301137, places=5)

    def test_push_path_is_the_reversed_pull_and_monotonic(self):
        points = self.recipe["push_path_tcp_base_link"]
        self.assertGreaterEqual(len(points), 5)
        self.assertTrue(all(points[i + 1][0] > points[i][0] for i in range(len(points) - 1)))
        self.assertLess(math.dist(points[0][:3], rclose.release_pose(self.recipe)[:3]), 1e-6)
        self.assertLess(math.dist(points[-1][:3], rclose.closed_pose(self.recipe)[:3]), 1e-6)
        # The pull sagged in Z; a straight line would load the handle sideways.
        self.assertGreater(points[-1][2] - points[0][2], 0.010)

    def test_segments_cover_the_travel_and_shrink_at_the_end(self):
        segments = rclose.push_segments(self.recipe)
        self.assertEqual(len(segments), len(self.recipe["push_path_tcp_base_link"]) - 1)
        self.assertAlmostEqual(sum(segment["dx"] for segment in segments),
                               rclose.travel_to_closed(self.recipe), places=9)
        self.assertTrue(segments[-1]["near_closed"])
        self.assertLess(segments[-1]["dx"], segments[0]["dx"] + 0.02)

    def test_changed_source_state_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "recipe.json"
            path.write_text(json.dumps(dict(self.recipe, source_state_sha256="0" * 64)),
                            encoding="utf-8")
            with self.assertRaises(ValueError):
                rclose.load_recipe(path)


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.recipe = rclose.load_recipe()
        self.release = rclose.release_pose(self.recipe)

    def _pose(self, dx=0.0, dy=0.0, dz=0.0, quat=None):
        quat = self.release[3:] if quat is None else quat
        return [self.release[0] + dx, self.release[1] + dy, self.release[2] + dz, *quat]

    def test_far_pose_aligns_at_the_standoff_not_the_handle(self):
        plan = rclose.approach_command(self._pose(dx=-0.30), self.recipe)
        self.assertEqual(plan["mode"], "align")
        self.assertAlmostEqual(plan["goal_pose"][0], self.release[0] - 0.09, places=6)
        self.assertLessEqual(plan["position_step_m"], 0.060 + 1e-9)

    def test_near_pose_engages_in_small_steps(self):
        plan = rclose.approach_command(self._pose(dx=-0.03), self.recipe)
        self.assertEqual(plan["mode"], "engage")
        self.assertLessEqual(plan["position_step_m"], 0.020 + 1e-9)
        self.assertGreater(plan["position_step_m"], 0.0)
        self.assertGreater(plan["command"]["dx"], 0.0)

    def test_pose_beyond_the_handle_plane_lifts_and_backs_off_first(self):
        plan = rclose.approach_command(self._pose(dx=0.05), self.recipe)
        self.assertEqual(plan["mode"], "clear")
        self.assertLess(plan["command"]["dx"], 0.0)
        self.assertGreater(plan["command"]["dz"], 0.0)

    def test_long_travel_is_clamped_to_one_bounded_step(self):
        plan = rclose.approach_command(self._pose(dx=-0.8, dy=-0.2), self.recipe)
        self.assertTrue(plan["clamped"])
        self.assertLessEqual(max(abs(plan["command"][key]) for key in ("dx", "dy", "dz")), 0.060)

    def test_orientation_gap_requires_explicit_rotation_consent(self):
        tilted = rclose._qmul(self.release[3:], [0.0, 0.0, 0.176, 0.984])  # ~20 deg
        with self.assertRaises(ValueError):
            rclose.approach_command(self._pose(dx=-0.1, quat=tilted), self.recipe)
        plan = rclose.approach_command(self._pose(dx=-0.1, quat=tilted), self.recipe,
                                       allow_rotation=True)
        self.assertGreater(max(abs(v) for v in plan["rotation_rad"]), 0.0)

    def test_gap_over_45_deg_always_blocks(self):
        flipped = [0.0, 0.7071, 0.0, 0.7071]
        with self.assertRaises(ValueError):
            rclose.approach_command(self._pose(dx=-0.1, quat=flipped), self.recipe,
                                    allow_rotation=True)

    def _flip(self):
        return [0.0, 0.7071, 0.0, 0.7071]

    def test_small_orientation_gap_rotates_with_a_clamp(self):
        small = rclose._qmul(self.release[3:], [0.0, 0.0, 0.05, 0.99875])
        plan = rclose.approach_command(self._pose(dx=-0.1, quat=small), self.recipe,
                                       allow_rotation=True, max_rotation_rad=0.10)
        self.assertGreater(max(abs(v) for v in plan["rotation_rad"]), 0.0)
        self.assertLessEqual(max(abs(v) for v in plan["rotation_rad"]), 0.10 + 1e-9)

    def test_at_release_pose_sends_no_offset(self):
        plan = rclose.approach_command(self._pose(), self.recipe)
        self.assertEqual(plan["mode"], "engage")
        self.assertAlmostEqual(plan["distance_to_release_m"], 0.0, places=9)


class RebaseTests(unittest.TestCase):
    """Re-expressing the recipe in a moved base frame must be exact geometry."""

    @staticmethod
    def _base(x=0.0, y=0.0, yaw=0.0):
        half = math.radians(yaw) / 2
        return {"x": x, "y": y, "z": 0.0, "qx": 0.0, "qy": 0.0,
                "qz": math.sin(half), "qw": math.cos(half), "frame_id": "map"}

    def setUp(self):
        self.recipe = rclose.load_recipe()
        self.record = self.recipe["release_base_map"]

    def test_same_base_keeps_the_original_recipe(self):
        active, info, issues = rclose.resolve_recipe_for_base(self.recipe, self.record)
        self.assertIs(active, self.recipe)
        self.assertFalse(info["rebased"])
        self.assertEqual(issues, [])

    def test_pure_translation_moves_the_point_back_towards_the_body(self):
        pose = [0.5, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]
        moved = rclose.rebase_pose(pose, self._base(), self._base(x=0.10))
        self.assertAlmostEqual(moved[0], 0.4, places=9)
        self.assertAlmostEqual(moved[1], 0.0, places=9)
        self.assertAlmostEqual(moved[2], 0.8, places=9)
        self.assertEqual(moved[3:], pose[3:])

    def test_pure_yaw_rotates_position_and_orientation_together(self):
        moved = rclose.rebase_pose([0.5, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0],
                                   self._base(), self._base(yaw=90.0))
        self.assertAlmostEqual(moved[0], 0.0, places=9)
        self.assertAlmostEqual(moved[1], -0.5, places=9)
        expected = [0.0, 0.0, math.sin(math.radians(-45)), math.cos(math.radians(-45))]
        for got, want in zip(moved[3:], expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_rebase_round_trip_recovers_the_recorded_poses(self):
        moved = self._base(self.record["x"] + 0.12, self.record["y"] - 0.05,
                           rclose.yaw_deg(self.record) + 7.0)
        forward = rclose.rebase_recipe(self.recipe, moved)
        back = rclose.rebase_recipe(forward, self.record)
        for key in ("release_tcp_base_link", "closed_tcp_base_link"):
            for got, want in zip(back[key], self.recipe[key]):
                self.assertAlmostEqual(got, want, places=9)

    def test_push_path_endpoints_follow_release_and_closed(self):
        moved = self._base(self.record["x"] + 0.08, self.record["y"],
                           rclose.yaw_deg(self.record) - 4.0)
        active = rclose.rebase_recipe(self.recipe, moved)
        self.assertEqual(active["push_path_tcp_base_link"][0], active["release_tcp_base_link"])
        self.assertEqual(active["push_path_tcp_base_link"][-1], active["closed_tcp_base_link"])
        # Stroke is a 3D distance: rotating the base rotates the push direction,
        # so the base-frame X component legitimately changes.
        self.assertAlmostEqual(math.dist(active["release_tcp_base_link"][:3],
                                         active["closed_tcp_base_link"][:3]),
                               math.dist(self.recipe["release_tcp_base_link"][:3],
                                         self.recipe["closed_tcp_base_link"][:3]), places=9)

    def test_large_shift_is_refused_instead_of_guessing(self):
        moved = self._base(self.record["x"] + 0.40, self.record["y"],
                           rclose.yaw_deg(self.record))
        _, info, issues = rclose.resolve_recipe_for_base(self.recipe, moved)
        self.assertTrue(info["rebased"])
        self.assertTrue(any("re-localising only supports" in issue for issue in issues))

    def test_large_yaw_is_refused(self):
        moved = self._base(self.record["x"], self.record["y"],
                           rclose.yaw_deg(self.record) + 20.0)
        _, _, issues = rclose.resolve_recipe_for_base(self.recipe, moved)
        self.assertTrue(any("yaw differs" in issue for issue in issues))


class GateTests(unittest.TestCase):
    def setUp(self):
        self.recipe = rclose.load_recipe()
        self.state = {
            "tcp": rclose.release_pose(self.recipe),
            "base_map": dict(self.recipe["release_base_map"]),
            "chassis_joints_rad": dict(self.recipe["release_chassis_joints_rad"]),
            "right_arm_joints_rad": dict(self.recipe["release_right_arm_joints_rad"]),
            "right_gripper": {"width": 0.050, "effort": 2.9, "is_moving": False},
        }
        self.controller = {"group": "right_arm", "active": "right_arm_pvt_ctrl",
                           "dry_run": False}
        self.confirmations = {name: True for name in rclose.APPROACH_CONFIRMATIONS}

    def test_recorded_state_passes_all_start_gates(self):
        self.assertEqual(rclose.prepare_issues(self.state, self.controller, self.recipe,
                                               self.confirmations), [])
        self.assertEqual(rclose.base_return_issues(self.state, self.recipe), [])

    def test_teleop_controller_blocks(self):
        teleop = dict(self.controller, active="right_arm_pvt_ctrl_teleop")
        issues = rclose.prepare_issues(self.state, teleop, self.recipe, self.confirmations)
        self.assertTrue(any("right_arm_pvt_ctrl is not active" in issue for issue in issues))

    def test_fully_open_gripper_is_a_valid_start_state(self):
        state = dict(self.state, right_gripper={"width": 0.085, "effort": 2.9,
                                                "is_moving": False})
        self.assertEqual(rclose.prepare_issues(state, self.controller, self.recipe,
                                              self.confirmations), [])

    def test_implausible_gripper_width_blocks(self):
        state = dict(self.state, right_gripper={"width": 0.180, "effort": 2.9,
                                                "is_moving": False})
        issues = rclose.prepare_issues(state, self.controller, self.recipe, self.confirmations)
        self.assertTrue(any("not open enough" in issue for issue in issues))

    def test_closed_gripper_blocks(self):
        state = dict(self.state, right_gripper={"width": 0.025, "effort": 8.0,
                                                "is_moving": False})
        issues = rclose.prepare_issues(state, self.controller, self.recipe, self.confirmations)
        self.assertTrue(any("is not open" in issue for issue in issues))

    def test_base_that_did_not_come_back_blocks(self):
        base = dict(self.recipe["release_base_map"])
        base["x"] += 0.045
        issues = rclose.base_return_issues(dict(self.state, base_map=base), self.recipe)
        self.assertTrue(any("from the recorded release position" in issue for issue in issues))

    def test_base_yaw_drift_blocks(self):
        base = dict(self.recipe["release_base_map"])
        yaw = math.radians(rclose.yaw_deg(base) + 4.0) / 2
        base.update(qz=math.sin(yaw), qw=math.cos(yaw))
        issues = rclose.base_return_issues(dict(self.state, base_map=base), self.recipe)
        self.assertTrue(any("yaw" in issue for issue in issues))

    def test_missing_confirmations_block(self):
        issues = rclose.prepare_issues(self.state, self.controller, self.recipe, {})
        self.assertEqual(len([i for i in issues if "confirmation missing" in i]),
                         len(rclose.APPROACH_CONFIRMATIONS))

    def test_grip_needs_handle_width_and_effort(self):
        good = dict(self.state, right_gripper={"width": 0.0245, "effort": 8.2,
                                               "is_moving": False})
        self.assertEqual(rclose.grip_issues(self.state, good, self.recipe), [])
        empty = dict(self.state, right_gripper={"width": 0.004, "effort": 0.5,
                                                "is_moving": False})
        issues = rclose.grip_issues(self.state, empty, self.recipe)
        self.assertGreaterEqual(len(issues), 2)

    def test_push_detects_no_advance_and_slipping(self):
        segments = rclose.push_segments(self.recipe)
        segment = segments[0]
        baseline = rclose.release_pose(self.recipe)
        before = dict(self.state, tcp=list(baseline),
                      right_gripper={"width": 0.0245, "effort": 8.2, "is_moving": False})
        after_ok = {**before, "tcp": [baseline[0] + segment["dx"], segment["to_pose"][1],
                                      segment["to_pose"][2], *baseline[3:]]}
        self.assertEqual(rclose.push_issues(before, after_ok, segment, baseline,
                                            grip_width_baseline_m=0.0245), [])
        stuck = {**before, "tcp": list(baseline)}
        issues = rclose.push_issues(before, stuck, segment, baseline,
                                    grip_width_baseline_m=0.0245)
        self.assertTrue(any("did not advance" in issue for issue in issues))
        slipped = {**after_ok, "right_gripper": {"width": 0.008, "effort": 1.0,
                                                 "is_moving": False}}
        issues = rclose.push_issues(before, slipped, segment, baseline,
                                    grip_width_baseline_m=0.0245)
        self.assertTrue(any("handle lost" in issue for issue in issues))

    def test_push_tolerates_a_start_that_was_already_off_the_path(self):
        """The start gate allows 8 mm; judging the result against the record alone
        would fail a run where the arm did exactly what it was told."""
        segments = rclose.push_segments(self.recipe)
        segment = segments[1]
        start = [segment["from_pose"][0], segment["from_pose"][1] + 0.007,
                 segment["from_pose"][2] - 0.007, *segment["from_pose"][3:]]
        before = dict(self.state, tcp=list(start),
                      right_gripper={"width": 0.0245, "effort": 8.2, "is_moving": False})
        after = {**before, "tcp": [start[0] + segment["dx"], start[1] + segment["dy"],
                                   start[2] + segment["dz"], *start[3:]]}
        self.assertEqual(rclose.push_issues(before, after, segment, start,
                                            grip_width_baseline_m=0.0245), [])

    def test_push_reports_lateral_drift_and_path_error_separately(self):
        segments = rclose.push_segments(self.recipe)
        segment = segments[1]
        start = list(segment["from_pose"])
        before = dict(self.state, tcp=start,
                      right_gripper={"width": 0.0245, "effort": 8.2, "is_moving": False})
        drifted = {**before, "tcp": [start[0] + segment["dx"],
                                     start[1] + segment["dy"] + 0.015,
                                     start[2] + segment["dz"], *start[3:]]}
        issues = rclose.push_issues(before, drifted, segment, start,
                                    grip_width_baseline_m=0.0245)
        self.assertTrue(any("drifted" in issue for issue in issues))
        self.assertTrue(any("off the recorded push path" in issue for issue in issues))

    def test_stroke_mismatch_blocks_before_any_motion(self):
        recorded = rclose.recorded_travel_mm(self.recipe)
        self.assertGreater(recorded, 0)
        self.assertEqual(rclose.stroke_issues(recorded, self.recipe), [])
        self.assertEqual(rclose.stroke_issues(recorded + 10, self.recipe), [])
        issues = rclose.stroke_issues(250.0, self.recipe)
        self.assertTrue(issues and "disagrees" in issues[0])
        self.assertEqual(rclose.stroke_issues(None, self.recipe), [])
        self.assertTrue(rclose.stroke_issues(0, self.recipe))

    def test_closed_check_uses_the_closed_reference(self):
        closed = rclose.closed_pose(self.recipe)
        state = dict(self.state, tcp=list(closed),
                     right_gripper={"width": 0.0245, "effort": 8.2, "is_moving": False})
        self.assertEqual(rclose.closed_issues(state, self.recipe), [])
        early = {**state, "tcp": [closed[0] - 0.05, *closed[1:]]}
        self.assertTrue(any("closed reference" in issue
                            for issue in rclose.closed_issues(early, self.recipe)))


class HandleGateTests(unittest.TestCase):
    def setUp(self):
        self.recipe = rclose.load_recipe()
        self.standoff = rclose.standoff_pose(self.recipe)
        self.mismatch = {"status": "mismatch", "failures": ["handle shifted 240 px"]}

    def _tcp(self, dx):
        return [self.standoff[0] + dx, self.standoff[1], self.standoff[2],
                *self.standoff[3:]]

    def test_far_arm_is_not_compared_at_all(self):
        gate = rclose.handle_gate(self.mismatch, self._tcp(-0.35), self.recipe)
        self.assertEqual(gate["gate"], "skipped")
        self.assertEqual(gate["reason"], "arm_far_from_release_pose")

    def test_midtrip_is_advisory_only(self):
        gate = rclose.handle_gate(self.mismatch, self._tcp(-0.06), self.recipe)
        self.assertEqual(gate["gate"], "advisory")

    def test_at_the_handle_a_mismatch_blocks(self):
        gate = rclose.handle_gate(self.mismatch, self._tcp(0.0), self.recipe)
        self.assertEqual(gate["gate"], "blocking")
        self.assertTrue(gate["failures"])

    def test_blocking_can_be_downgraded_on_site(self):
        gate = rclose.handle_gate(self.mismatch, self._tcp(0.0), self.recipe, advisory=True)
        self.assertEqual(gate["gate"], "advisory")

    def test_missing_frame_is_never_a_block(self):
        gate = rclose.handle_gate({"status": "no_reference_image"},
                                  self._tcp(0.0), self.recipe)
        self.assertEqual(gate["gate"], "skipped")


class HandleViewTests(unittest.TestCase):
    @staticmethod
    def _image(bar_x=300, bar_w=40):
        """Vertical red bar: this is what the 90 deg rotated wrist camera outputs
        for a horizontal handle."""
        import numpy as np
        image = np.zeros((360, 640, 3), dtype="uint8")
        image[150:270, bar_x:bar_x + bar_w] = (30, 30, 200)  # BGR red
        return image

    def test_identical_views_pass(self):
        image = self._image()
        result = rclose.handle_view(image, image)
        self.assertEqual(result["status"], "ok")
        self.assertAlmostEqual(result["centroid_offset_px"], 0.0)

    def test_rotation_normalisation_turns_the_bar_horizontal(self):
        result = rclose.handle_view(self._image(), self._image())
        self.assertEqual(result["quarter_turns"], 1)
        self.assertGreater(result["canonical_aspect"], 2.0)
        raw = rclose.handle_view(self._image(), self._image(), quarter_turns=0)
        self.assertLess(raw["canonical_aspect"], 1.0)

    def test_rotation_is_recorded_in_the_recipe(self):
        recipe = rclose.load_recipe()
        self.assertEqual(recipe["wrist_image_rotation_quarter_turns"], 1)
        self.assertIn("90 deg", recipe["wrist_camera_note"])

    def test_shifted_handle_is_reported(self):
        result = rclose.handle_view(self._image(), self._image(bar_x=520))
        self.assertEqual(result["status"], "mismatch")
        self.assertTrue(any("shifted" in failure for failure in result["failures"]))

    def test_missing_handle_is_not_silently_ignored(self):
        import numpy as np
        result = rclose.handle_view(self._image(), np.zeros((360, 640, 3), dtype="uint8"))
        self.assertEqual(result["status"], "handle_not_detected")

    def test_non_bar_red_region_is_rejected_as_the_handle(self):
        import numpy as np
        square = np.zeros((360, 640, 3), dtype="uint8")
        square[150:270, 300:420] = (30, 30, 200)  # 120x120 square: a red box, not a bar
        result = rclose.handle_view(self._image(), square)
        self.assertEqual(result["status"], "mismatch")
        self.assertTrue(any("aspect" in failure for failure in result["failures"]))


class PlanTests(unittest.TestCase):
    def test_plan_is_read_only_and_states_the_flow(self):
        value = rclose.plan()
        self.assertTrue(value["dry_run"])
        self.assertEqual(value["motion_commands_sent"], [])
        self.assertEqual(value["flow"][0], "prepare (read-only)")
        self.assertEqual(len(value["push_segments_mm"]),
                         len(rclose.push_segments(rclose.load_recipe())))
        self.assertFalse(value["verified_on_robot"])


if __name__ == "__main__":
    unittest.main()
