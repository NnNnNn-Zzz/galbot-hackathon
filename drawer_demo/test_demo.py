"""Offline fault tests; none import the robot SDK or communicate with hardware."""
import math
import unittest
from copy import deepcopy

from .contracts import Context, Receipt
from .feasibility import assess
from .geometry import localize, yaw_arc_offset_m, yaw_segments
from .mock import MockIO, example_recipe, site_template
from .runtime import EVIDENCE, FixedBoxDemo, PLAN, plan_for, validate_recipe
from .selftest import run as selftest_run


class DemoTests(unittest.TestCase):
    def run_demo(self, fault="", recipe=None, io_type=MockIO):
        recipe = recipe or example_recipe()
        io = io_type(recipe, fault)
        ctx = Context("test", recipe["setup_id"])
        return FixedBoxDemo(recipe).run(ctx, io), ctx, io

    def names(self, io):
        return [a.name for a in io.actions]

    # ---------------------------------------------------------------- happy path
    def test_success_runs_whole_sequence(self):
        res, ctx, io = self.run_demo()
        self.assertEqual(res.status, "ok")
        self.assertTrue(ctx.flags["drawer_demo.box_in_bin"])
        self.assertTrue(ctx.flags["drawer_demo.drawer_closed"])
        self.assertFalse(ctx.picked)
        self.assertEqual(self.names(io),
                         [s for s in PLAN if s not in ("preflight", "localize_high", "verify_above_box")])
        self.assertEqual(self.names(io)[-2:], ["release_handle", "return_home"])
        self.assertFalse(io.stops)

    def test_yaw_command_is_split_and_negative(self):
        _, _, io = self.run_demo()
        [turn] = [a for a in io.actions if a.name == "rotate_to_drop"]
        self.assertAlmostEqual(turn.params["yaw_rad"], -math.pi/6)
        self.assertEqual(len(turn.params["yaw_segments_rad"]), 2)
        for step in turn.params["yaw_segments_rad"]:
            self.assertAlmostEqual(step, -math.pi/12)
        self.assertLessEqual(abs(turn.params["yaw_segments_rad"][0]), math.pi/12 + 1e-12)
        self.assertGreater(turn.params["max_speed_rad_s"], 0)
        [back] = [a for a in io.actions if a.name == "rotate_back"]
        self.assertAlmostEqual(back.params["yaw_rad"], math.pi/6)

    # ------------------------------------------------------------- perception
    def test_perception_faults_never_descend(self):
        for fault in ("stale", "multiple", "no_target", "shift"):
            with self.subTest(fault=fault):
                res, ctx, io = self.run_demo(fault)
                self.assertEqual(res.status, "unknown")
                self.assertNotIn("descend", self.names(io))
                self.assertTrue(io.stops)
                self.assertIsNone(ctx.picked)

    # ------------------------------------------------------- motion / base yaw
    def test_no_recovery_after_motion_fault(self):
        for fault in ("timeout", "empty_grasp", "blocked_retreat"):
            with self.subTest(fault=fault):
                res, ctx, io = self.run_demo(fault)
                self.assertEqual(res.status, "unknown")
                self.assertNotIn("release_box", self.names(io))
                self.assertNotIn("return_home", self.names(io))
                self.assertFalse(ctx.flags["drawer_demo.payload_clear_of_drawer"])

    def test_short_yaw_blocks_release(self):
        res, ctx, io = self.run_demo("yaw_short")
        self.assertEqual(res.status, "unknown")
        self.assertIn("rotate_to_drop", self.names(io))
        self.assertNotIn("release_box", self.names(io))
        self.assertNotIn("verify_drop", self.names(io))
        self.assertFalse(ctx.flags.get("drawer_demo.drop_yaw_verified", False))
        self.assertIn("Drop yaw residual", ctx.stopped_reason)

    def test_base_drift_blocks_regrip(self):
        _, ctx, io = self.run_demo("base_drift")
        self.assertIn("rotate_back", self.names(io))
        self.assertNotIn("regrip_handle", self.names(io))
        self.assertIn("drifted", ctx.stopped_reason)

    def test_stuck_return_turn_latches(self):
        res, _, io = self.run_demo("rotate_stuck")
        self.assertEqual(res.status, "unknown")
        self.assertNotIn("regrip_handle", self.names(io))
        self.assertTrue(io.stops)

    def test_missing_bin_blocks_push(self):
        _, ctx, io = self.run_demo("bin_miss")
        self.assertNotIn("rotate_back", self.names(io))
        self.assertFalse(ctx.flags.get("drawer_demo.box_in_bin", False))
        self.assertIn("box_in_bin", ctx.stopped_reason)

    # ------------------------------------------------------------ drawer push
    def test_unclosed_drawer_is_not_reported_ok(self):
        res, ctx, io = self.run_demo("close_blocked")
        self.assertEqual(res.status, "unknown")
        self.assertIn("push_drawer_closed", self.names(io))
        self.assertNotIn("release_handle", self.names(io))
        self.assertNotIn("return_home", self.names(io))
        self.assertFalse(ctx.flags.get("drawer_demo.drawer_closed", False))

    def test_empty_regrip_blocks_push(self):
        _, _, io = self.run_demo("regrip_empty")
        self.assertIn("regrip_handle", self.names(io))
        self.assertNotIn("push_drawer_closed", self.names(io))

    def test_short_stroke_is_rejected(self):
        r = example_recipe()

        class ShortStroke(MockIO):
            def execute(self, action, timeout_s):
                receipt = super().execute(action, timeout_s)
                if action.name == "push_drawer_closed":
                    evidence = dict(receipt.evidence, stroke_m=0.20)
                    return Receipt(receipt.status, receipt.action_id,
                                   receipt.timestamp_ns, evidence)
                return receipt
        res, ctx, _ = self.run_demo(recipe=r, io_type=ShortStroke)
        self.assertNotEqual(res.status, "ok")
        self.assertIn("Push stroke", ctx.stopped_reason)

    # ------------------------------------------------------------- guard rails
    def test_site_template_cannot_execute(self):
        res, _, io = self.run_demo(recipe=site_template())
        self.assertNotEqual(res.status, "ok")
        self.assertEqual(io.actions, [])

    def test_missing_calibration_rejected(self):
        r = example_recipe()
        r["calibration"]["verified"] = False
        res, _, io = self.run_demo(recipe=r)
        self.assertEqual(res.status, "failed")
        self.assertEqual(io.actions, [])

    def test_old_recipe_schema_rejected(self):
        r = deepcopy(example_recipe())
        r["schema"] = "drawer_demo_recipe/1"
        with self.assertRaises(ValueError):
            validate_recipe(r, "mock")

    def test_yaw_budget_violation_rejected(self):
        r = example_recipe()
        r["base_motion"]["yaw_tolerance_rad"] = 0.2
        with self.assertRaises(ValueError):
            validate_recipe(r, "mock")
        res, _, io = self.run_demo(recipe=r)
        self.assertEqual(res.status, "failed")
        self.assertEqual(io.actions, [])

    def test_corrupt_receipt_rejected(self):
        class WrongReceipt(MockIO):
            def execute(self, action, timeout_s):
                r = super().execute(action, timeout_s)
                return Receipt(r.status, "another_request", r.timestamp_ns, r.evidence)
        res, _, io = self.run_demo(io_type=WrongReceipt)
        self.assertEqual(res.status, "unknown")
        self.assertEqual(len(io.actions), 1)

    def test_stop_failure_reported(self):
        class BrokenStop(MockIO):
            def stop(self, reason):
                raise RuntimeError("stop unavailable")
        res, _, _ = self.run_demo("timeout", io_type=BrokenStop)
        self.assertEqual(res.status, "unknown")
        self.assertFalse(res.data["stop"]["confirmed"])

    def test_latched_context_cannot_resume(self):
        r = example_recipe()
        io, ctx = MockIO(r), Context("test", r["setup_id"], stopped_reason="prior fault")
        self.assertNotEqual(FixedBoxDemo(r).run(ctx, io).status, "ok")
        self.assertEqual(io.actions, [])

    # ----------------------------------------------------------- geometry math
    def test_yaw_segments_cover_the_turn(self):
        for total, step, count in ((-math.pi/6, math.pi/12, 2),
                                   (-math.pi/6, 0.26, 3),
                                   (-0.1, 0.5, 1)):
            with self.subTest(total=total, step=step):
                segments = yaw_segments(total, step)
                self.assertEqual(len(segments), count)
                self.assertAlmostEqual(sum(segments), total)
                self.assertTrue(all(abs(s) <= step + 1e-12 for s in segments))

    def test_yaw_arc_offset(self):
        self.assertAlmostEqual(yaw_arc_offset_m(0.62, math.radians(2.0)), 0.0216, places=4)

    def test_localization_and_mask_rejections(self):
        r, io = example_recipe(), MockIO(example_recipe())
        frame = io.capture("OBSERVE_HIGH", 1)
        payload = io.segment(frame, 1)
        target = localize(payload, frame, r, io.now_ns())
        self.assertAlmostEqual(target["position_m"][0], 0.5)
        self.assertAlmostEqual(target["position_m"][1], 0)
        for mutation in ("nan", "wrong_camera", "cropped", "out_of_roi"):
            with self.subTest(mutation=mutation):
                p = deepcopy(payload)
                poly = p["detections"][0]["mask_polygon_xy_px"]
                if mutation == "nan":
                    poly[0][0] = float("nan")
                elif mutation == "wrong_camera":
                    p["camera_id"] = "wrong"
                elif mutation == "cropped":
                    poly[1][0] -= 20
                    poly[2][0] -= 20
                else:
                    for point in poly:
                        point[0] += 200
                with self.assertRaises(ValueError):
                    localize(p, frame, r, io.now_ns())


class SelftestTests(unittest.TestCase):
    def test_full_matrix_passes(self):
        report = selftest_run()
        self.assertTrue(report["ok"], report["problems"])
        self.assertEqual(len(report["cases"]), 17)
        self.assertTrue(all(case["ok"] for case in report["cases"]))
        self.assertTrue(report["structure"]["ok"])
        self.assertEqual(report["structure"]["steps"], len(PLAN))
        steps = set(PLAN) | set(plan_for(example_recipe(fixed=True)))
        self.assertEqual(set(EVIDENCE), steps-{"preflight", "localize_high", "verify_above_box"})


class FeasibilityTests(unittest.TestCase):
    def test_synthetic_recipe_is_computable(self):
        report = assess(example_recipe())
        self.assertEqual(report["overall"], "offline_verifiable")
        self.assertTrue(report["recipe_accepted_by_runtime"])
        self.assertEqual(report["blocked_stages"], [])
        stages = {s["stage"]: s for s in report["stages"]}
        yaw = stages["S5_yaw_right_30"]["computed"]
        self.assertEqual(yaw["segments"], 2)
        self.assertAlmostEqual(yaw["commanded_yaw_deg"], -30.0, places=6)
        self.assertGreater(yaw["margin_mm"], 0)
        self.assertEqual(stages["S6_yaw_return"]["computed"]["strict_reuse"],
                         "recorded push path used as-is")

    def test_site_template_is_blocked(self):
        report = assess(site_template())
        self.assertEqual(report["overall"], "blocked")
        self.assertFalse(report["recipe_accepted_by_runtime"])
        for stage in ("S1_pull_drawer", "S3_yolo_localize", "S5_fixed_drop", "home"):
            self.assertIn(stage, report["blocked_stages"])
        self.assertNotIn("rotate_to_drop", report["plan"])

    def test_loose_return_tolerance_needs_rebase(self):
        r = example_recipe()
        r["base_motion"]["return_yaw_tolerance_rad"] = math.radians(5.0)
        r["base_motion"]["return_shift_tolerance_m"] = 0.08
        stages = {s["stage"]: s for s in assess(r)["stages"]}
        self.assertIsNone(stages["S6_yaw_return"]["computed"]["strict_reuse"])
        self.assertIsNotNone(stages["S6_yaw_return"]["computed"]["rebase_reuse"])

    def test_beyond_rebase_limits_is_blocked(self):
        r = example_recipe()
        r["base_motion"]["return_yaw_tolerance_rad"] = math.radians(15.0)
        stages = {s["stage"]: s for s in assess(r)["stages"]}
        self.assertEqual(stages["S6_yaw_return"]["verdict"], "blocked")
        self.assertEqual(assess(r)["overall"], "blocked")


if __name__ == "__main__":
    unittest.main()
