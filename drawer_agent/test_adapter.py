"""适配器自测：在线路层注入故障，验证"证据来自读数"而不是"回执为真"。

跑法（仓库根目录）：
    python -m unittest drawer_agent.test_adapter -v
"""

from __future__ import annotations

import json
import math
import threading
import unittest
from pathlib import Path

from drawer_agent.framework import load as load_framework
from drawer_agent.io import SharedAgentIO
from drawer_agent.routes import RouteStore
from drawer_agent.runner import (DEFAULT_RECIPE, DEFAULT_ROUTES, offline_recipe, read_json,
                                 run_flow, validate_site)
from drawer_agent.sim_link import OfflinePerception, OfflineRig
from drawer_agent.link import NEEDED_HINT
from drawer_agent.shared_link import SharedAgentLink


def _framework():
    return load_framework()


def _run(link_factory=None, recipe_path=DEFAULT_RECIPE, routes_path=DEFAULT_ROUTES):
    fw = _framework()
    recipe = read_json(recipe_path)
    routes = RouteStore.load(routes_path)
    link = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                      tcp_frame=recipe["tcp_frame"]) if link_factory is None \
        else link_factory(recipe, routes)
    io = SharedAgentIO(link, OfflinePerception(link), routes, framework=fw)
    ctx = fw.Context("test-run", recipe["setup_id"])
    result = fw.FixedBoxDemo(recipe).run(ctx, io)
    return result, ctx, link, io, recipe, routes


class NominalFlowTests(unittest.TestCase):
    def test_full_flow_ok(self):
        result, ctx, link, io, recipe, _ = _run()
        self.assertEqual(result.status, "ok", result.message)
        self.assertEqual(result.data, {"box_in_bin": True, "drawer_closed": True})
        self.assertIsNone(ctx.stopped_reason)

    def test_every_planned_step_ran(self):
        result, ctx, *_ = _run()
        names = {entry["step"] for entry in ctx.trace}
        for step in _framework().PLAN:
            self.assertIn(step, names, f"计划步骤 {step} 未出现在轨迹里")

    def test_evidence_facts_present_for_each_action(self):
        _, ctx, *rest = _run()
        fw = _framework()
        seen = set()
        for entry in ctx.trace:
            step = entry["step"]
            if step not in fw.EVIDENCE or entry["status"] != "ok":
                continue
            evidence = entry.get("evidence") or {}
            for fact in fw.EVIDENCE[step]:
                self.assertIs(evidence.get(fact), True, f"{step} 缺物证 {fact}")
            seen.add(step)
        self.assertEqual(seen, set(fw.PLAN) & fw.EVIDENCE.keys(), "有动作没留下证据记录")

    def test_base_turned_then_returned(self):
        _, ctx, link, *_ = _run()
        turned = [e for e in ctx.trace
                  if e["step"] == "rotate_to_drop" and e["status"] == "ok"][0]
        self.assertAlmostEqual(turned["evidence"]["yaw_measured_rad"],
                               math.radians(-30), places=9)
        self.assertAlmostEqual(link.base["yaw"], 0.0, places=9)
        self.assertLess(link.drawer_opening, 0.005)
        self.assertEqual(link.box["where"], "world")
        dist = math.hypot(link.box["x"] - link.bin_world[0],
                          link.box["y"] - link.bin_world[1])
        self.assertLess(dist, 0.02)

    def test_success_path_never_calls_stop(self):
        _, _, link, *_ = _run()
        self.assertEqual(link.stops, [], "成功路径不应触发停止通道")


class PreflightTests(unittest.TestCase):
    def test_missing_p0_capability_refuses_before_motion(self):
        def factory(recipe, routes):
            rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                             tcp_frame=recipe["tcp_frame"])
            original = rig.caps

            def caps():
                data = original()
                data["drawer_state"] = False
                return data
            rig.caps = caps
            return rig

        result, ctx, link, *_ = _run(factory)
        # 非 GateError 的失败一律按 unknown 处理（运行时更保守），关键是没动硬件。
        self.assertNotEqual(result.status, "ok")
        self.assertIn("P0", result.message)
        motions = [c for c in link.calls
                   if c in ("move_tcp", "set_gripper", "set_base_pose_rel", "move_joints")]
        self.assertEqual(motions, [], "预检失败后不应下发任何运动指令")

    def test_missing_p1_capability_stops_at_drop_check(self):
        def factory(recipe, routes):
            rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                             tcp_frame=recipe["tcp_frame"])
            original = rig.caps

            def caps():
                data = original()
                data["bin_state"] = False
                return data
            rig.caps = caps
            return rig

        result, _, _, _, _, _ = _run(factory)
        self.assertNotEqual(result.status, "ok")
        self.assertEqual(result.data["failed_step"], "verify_drop")
        self.assertIn("bin_state", result.message)

    def test_route_gap_blocks_preflight(self):
        fw = _framework()
        recipe = read_json(DEFAULT_RECIPE)
        routes = RouteStore.load(DEFAULT_ROUTES)
        routes.data["verified"]["base_yaw_free_path"] = False
        io = SharedAgentIO(OfflineRig(recipe, routes), OfflinePerception(None), routes,
                           framework=fw)
        facts = io.preflight(recipe)
        self.assertFalse(facts["routes_verified"])
        self.assertIn("base_yaw_free_path", " ".join(facts["route_gaps"]))


class FaultTests(unittest.TestCase):
    def test_each_fault_stops_before_declaring_success(self):
        expected = {
            "short_yaw": "rotate_to_drop", "yaw_stuck": "rotate_to_drop",
            "drawer_jam": "open_drawer_release", "stale_frame": "localize_high",
            "no_target": "localize_high", "regrip_miss": "regrip_handle",
            "bin_shift": "verify_drop", "stroke_short": "push_drawer_closed",
            "jam_step": "open_drawer_release",
        }
        for fault, step in expected.items():
            with self.subTest(fault=fault):
                result = run_flow(backend="offline", faults=[fault], extended=True)
                self.assertNotEqual(result["status"], "ok")
                self.assertEqual(result["data"]["failed_step"], step)
                self.assertTrue(result["data"]["requires_operator_review"])

    def test_short_yaw_is_caught_by_odometry_not_by_receipt(self):
        result = run_flow(backend="offline", faults=["short_yaw"], extended=True)
        self.assertEqual(result["data"]["failed_step"], "rotate_to_drop")
        self.assertIn("残差", result["message"])

    def test_empty_grasp_caught_by_effort(self):
        result = run_flow(backend="offline", faults=["empty_grasp"])
        self.assertIn("夹持电流", result["message"])

    def test_jam_step_reports_unknown_and_stops(self):
        result = run_flow(backend="offline", faults=["jam_step"])
        self.assertEqual(result["status"], "unknown")
        self.assertTrue(result["data"]["stop"]["confirmed"])


class ContractTests(unittest.TestCase):
    def test_latched_stop_refuses_further_actions(self):
        fw = _framework()
        recipe = read_json(DEFAULT_RECIPE)
        routes = RouteStore.load(DEFAULT_ROUTES)
        rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                         tcp_frame=recipe["tcp_frame"])
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw)
        io.preflight(recipe)
        io.stop("operator")
        calls = len(rig.calls)
        action = fw.Action("open_gripper", {"width_m": 0.095}, ("gap_verified",), "req-1")
        receipt = io.execute(action, 5.0)
        self.assertEqual(receipt.status, "failed")
        self.assertEqual(len(rig.calls), calls, "锁存后仍下发了动作")

    def test_staged_run_stops_after_named_action(self):
        result = run_flow(backend="offline", stop_after="open_drawer_release")
        self.assertNotEqual(result["status"], "ok")
        self.assertEqual(result["data"]["failed_step"], "observe_high")
        opened = [e for e in result["trace"]
                  if e["step"] == "open_drawer_release" and e["status"] == "ok"]
        self.assertTrue(opened and opened[0]["evidence"]["drawer_open_verified"])

    def test_segment_echo_mismatch_is_rejected(self):
        fw = _framework()
        recipe = read_json(DEFAULT_RECIPE)
        routes = RouteStore.load(DEFAULT_ROUTES)
        rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                         tcp_frame=recipe["tcp_frame"])

        class CrossTalk(OfflinePerception):
            def segment(self, frame):
                payload = super().segment(frame)
                payload["frame_id"] = "someone-elses-frame"
                return payload

        io = SharedAgentIO(rig, CrossTalk(rig), routes, framework=fw)
        io.preflight(recipe)
        frame = io.capture("OBSERVE_HIGH", 3.0)
        with self.assertRaises(Exception) as ctx:
            io.segment(frame, 3.0)
        self.assertIn("frame_id", str(ctx.exception))

    def test_stale_frame_is_rejected_at_capture(self):
        fw = _framework()
        recipe = read_json(DEFAULT_RECIPE)
        routes = RouteStore.load(DEFAULT_ROUTES)
        rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                         tcp_frame=recipe["tcp_frame"])
        rig.inject("stale_frame")
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw)
        io.preflight(recipe)
        with self.assertRaises(Exception) as ctx:
            io.capture("OBSERVE_HIGH", 3.0)
        self.assertIn("时间戳", str(ctx.exception))


class IntegrationRegressionTests(unittest.TestCase):
    def test_capabilities_use_adapter_names(self):
        class Wrapper:
            pass
        for name in NEEDED_HINT.values():
            setattr(Wrapper, name, lambda self: {})
        self.assertEqual(SharedAgentLink({}, wrapper=Wrapper()).caps(),
                         dict.fromkeys(NEEDED_HINT, True))

    def test_offline_recipe_refused_by_live_gate(self):
        with self.assertRaisesRegex(ValueError, "现场配方"):
            validate_site(offline_recipe(), RouteStore.load(DEFAULT_ROUTES), _framework())

    def test_fixed_flow_and_wrist_plan_reuse_approach(self):
        fw = _framework()
        recipe = offline_recipe()
        routes = RouteStore.load(DEFAULT_ROUTES)
        steps = routes.route("drawer_open_release")
        routes.data["waypoints"]["HANDLE_CLEAR"] = steps[-1]["pose"]
        recipe["drawer_open"] = {
            "mode": "wrist_vision", "camera_id": "fixture_wrist",
            "calibration_id": recipe["calibration"]["id"],
        }

        class Planner:
            request = None
            def plan(self, request):
                self.request = request
                return steps

        class TracedRig(OfflineRig):
            def __init__(self, recipe, routes):
                super().__init__(recipe, routes)
                self.motion = []
            def move_tcp(self, pose, timeout_s, **kwargs):
                self.motion.append((self.payload, list(pose)))
                return super().move_tcp(pose, timeout_s, **kwargs)

        rig, planner = TracedRig(recipe, routes), Planner()
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw, opener=planner)
        ctx = fw.Context("wrist-test", recipe["setup_id"])
        result = fw.FixedBoxDemo(recipe).run(ctx, io)
        self.assertEqual(result.status, "ok", result.message)
        self.assertEqual(result.data, {"box_in_bin": True, "drawer_closed": False})
        self.assertNotIn("set_base_pose_rel", rig.calls)
        self.assertAlmostEqual(rig.drawer_opening, 0.27)
        self.assertEqual(planner.request.frame.camera_id, "fixture_wrist")
        self.assertEqual(planner.request.target_opening_m, 0.27)
        held_poses = [pose for payload, pose in rig.motion if payload == "box"]
        expected = [p["tcp"] for p in reversed(io._approach)]
        self.assertEqual(held_poses[1:1+len(expected)], expected)

    def test_wrist_plan_with_wrong_pull_sign_does_not_move(self):
        fw = _framework()
        recipe, routes = offline_recipe(), RouteStore.load(DEFAULT_ROUTES)
        recipe["drawer_open"] = {"mode": "wrist_vision", "camera_id": "wrist",
                                 "calibration_id": recipe["calibration"]["id"]}
        steps = routes.route("drawer_open_release")
        next(step for step in steps if step["op"] == "move_rel")["dx"] *= -1

        class Planner:
            def plan(self, request):
                return steps

        rig = OfflineRig(recipe, routes)
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw, opener=Planner())
        result = fw.FixedBoxDemo(recipe).run(fw.Context("bad", recipe["setup_id"]), io)
        self.assertEqual(result.data["failed_step"], "open_drawer_release")
        self.assertNotIn("move_tcp", rig.calls)
        self.assertNotIn("set_gripper", rig.calls)

    def test_watchdog_requests_stop_during_blocking_call(self):
        fw = _framework()
        recipe, routes = offline_recipe(), RouteStore.load(DEFAULT_ROUTES)
        stopped = threading.Event()

        class BlockingRig(OfflineRig):
            def move_tcp(self, pose, timeout_s, **kwargs):
                if not stopped.wait(1):
                    raise AssertionError("watchdog did not stop the blocking call")
                return super().move_tcp(pose, timeout_s, **kwargs)
            def stop(self, chain):
                response = super().stop(chain)
                if chain == "base":
                    stopped.set()
                return response

        rig = BlockingRig(recipe, routes)
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw)
        io.preflight(recipe)
        action = fw.Action("open_drawer_release", {"route_ref": "drawer_open_release",
                                                  "target_opening_m": 0.27}, (), "blocked")
        reply = io.execute(action, 0.05)
        self.assertTrue(stopped.is_set())
        self.assertEqual(reply.status, "unknown")
        self.assertNotIn("set_gripper", rig.calls)


class RouteTests(unittest.TestCase):
    def _flipped(self, routes: RouteStore) -> RouteStore:
        data = json.loads(json.dumps(routes.data))
        for step in data["routes"]["drawer_open_release"]["steps"]:
            if step["op"] == "move_rel":
                step["dx"] = -step["dx"]
        return RouteStore(data)

    def test_reversed_pull_direction_is_reported(self):
        recipe = read_json(DEFAULT_RECIPE)
        gaps = self._flipped(RouteStore.load(DEFAULT_ROUTES)).check(recipe)
        self.assertTrue(any("方向反了" in gap for gap in gaps), gaps)

    def test_stroke_mismatch_is_reported(self):
        recipe = read_json(DEFAULT_RECIPE)
        store = RouteStore.load(DEFAULT_ROUTES)
        recipe["drawer_opening_m"] = 0.20
        gaps = store.check(recipe)
        self.assertTrue(any("不符" in gap for gap in gaps), gaps)

    def test_missing_waypoint_is_reported(self):
        data = json.loads(Path(DEFAULT_ROUTES).read_text(encoding="utf-8"))
        del data["waypoints"]["DROP"]
        gaps = RouteStore(data).check(read_json(DEFAULT_RECIPE))
        self.assertTrue(any("DROP" in gap for gap in gaps))


class GeometryRoundTripTests(unittest.TestCase):
    def test_synthetic_mask_maps_back_to_box_center(self):
        recipe = read_json(DEFAULT_RECIPE)
        routes = RouteStore.load(DEFAULT_ROUTES)
        rig = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                         tcp_frame=recipe["tcp_frame"])
        rig.drawer_opening = 0.27
        fw = _framework()
        io = SharedAgentIO(rig, OfflinePerception(rig), routes, framework=fw)
        io.preflight(recipe)
        frame = io.capture("OBSERVE_HIGH", 3.0)
        target = fw.module.geometry.localize(io.segment(frame, 3.0), frame, recipe,
                                            io.now_ns())
        truth = rig.box_center()
        self.assertLess(math.dist(target["position_m"][:2], truth), 0.001)
        self.assertAlmostEqual(target["position_m"][2], recipe["calibration"]["box_top_z_m"])


if __name__ == "__main__":
    unittest.main(verbosity=2)


class TeachConversionTests(unittest.TestCase):
    """示教点 → 路线文件：只映射，不插值。"""

    def _session(self, tmp: Path) -> Path:
        q = [0.0, 0.0, 0.0, 1.0]
        points = {
            "HOME": (0.25, 0.0, 1.30, None),
            "HANDLE_PRE": (0.90, 0.0, 1.06, None),
            "HANDLE_GRIP": (0.90, 0.0, 0.98, None),
            "PULL_START": (0.90, 0.0, 0.98, 0.0),
            "PULL_05CM": (0.85, 0.0, 0.98, 50.0),
            "PULL_10CM": (0.80, 0.0, 0.98, 100.0),
            "PULL_15CM": (0.75, 0.0, 0.98, 150.0),
            "PULL_20CM": (0.70, 0.0, 0.98, 200.0),
            "PULL_25CM": (0.65, 0.0, 0.98, 250.0),
            "PULL_END": (0.63, 0.0, 0.98, 270.0),
            "HANDLE_CLEAR": (0.63, 0.0, 1.06, None),
            "RISE_MID": (0.50, 0.0, 1.12, None),
            "BOX_PRE": (0.38, 0.0, 1.22, None),
            "BOX_HELD": (0.52, 0.0, 0.85, None),
            "CARRY": (0.30, 0.0, 1.15, None),
            "DROP": (0.45, -0.19, 0.95, None),
            "HOME_END": (0.25, 0.0, 1.30, None),
        }
        for name, (x, y, z, opening) in points.items():
            folder = tmp / name
            folder.mkdir(parents=True)
            (folder / "point.json").write_text(json.dumps({
                "point": name, "recorded_ns": 1,
                "state": {"tcp": [x, y, z, *q]},
                "operator_measured_opening_mm": opening,
            }, ensure_ascii=False), encoding="utf-8")
        return tmp

    def test_builds_valid_routes_from_taught_points(self):
        import tempfile
        from drawer_agent import teach
        recipe = read_json(DEFAULT_RECIPE)
        with tempfile.TemporaryDirectory() as tmp:
            session = self._session(Path(tmp) / "drawer_demo_test")
            data, gaps = teach.build(session, setup_id="SITE_A",
                                     gripper_open_m=0.095, drawer_opening_m=0.27)
        self.assertEqual(gaps, [])
        pull = [s["dx"] for s in data["routes"]["drawer_open_release"]["steps"]
                if s["op"] == "move_rel"]
        push = [s["dx"] for s in data["routes"]["push_drawer_closed"]["steps"]
                if s["op"] == "move_rel"]
        self.assertAlmostEqual(sum(pull), -0.27, places=6)
        self.assertAlmostEqual(sum(push), 0.27, places=6)
        self.assertEqual(data["waypoints"]["OBSERVE_HIGH"],
                         data["waypoints"]["OBSERVE_HIGH"])
        self.assertAlmostEqual(data["waypoints"]["OBSERVE_HIGH"][2], 1.22)

        store = RouteStore({**data, "verified": {k: True for k in
                                                 ("observe_high_max_z", "bin_fixed",
                                                  "base_yaw_free_path")}})
        recipe["setup_id"] = "SITE_A"
        self.assertEqual(store.check(recipe), [])

    def test_missing_pull_samples_refuses(self):
        import tempfile
        from drawer_agent import teach
        with tempfile.TemporaryDirectory() as tmp:
            session = self._session(Path(tmp) / "drawer_demo_test")
            for extra in ("PULL_05CM", "PULL_10CM", "PULL_15CM", "PULL_20CM",
                          "PULL_25CM"):
                for f in (session / extra).iterdir():
                    f.unlink()
                (session / extra).rmdir()
            with self.assertRaises(teach.TeachError):
                teach.build(session, setup_id="SITE_A", gripper_open_m=0.095,
                            drawer_opening_m=0.27)
