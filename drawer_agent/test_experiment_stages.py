import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from .experiments import _experiment_3_metadata, mock
from .link import LinkError
from .perception import YoloHttpClient
from .routes import RouteStore
from .runner import DEFAULT_ROUTES, offline_recipe


class ExperimentStagesTest(unittest.TestCase):
    def test_hover_stops_after_two_valid_frames(self):
        result = mock(3, "hover")
        self.assertEqual(result["status"], "ok")
        self.assertFalse(result["data"]["full_cycle"])
        names = [row["step"] for row in result["trace"]]
        self.assertIn("localize_high", names)
        self.assertIn("verify_above_box", names)
        self.assertTrue({"descend", "close_gripper", "release_box"}.isdisjoint(names))

    def test_full_requires_hover_for_same_model_and_routes(self):
        routes = RouteStore.load(DEFAULT_ROUTES)
        recipe = offline_recipe()
        recipe["drawer_open"] = {"mode": "wrist_vision"}
        recipe["waypoint_refs"]["OBSERVE_HIGH"] = "P0"
        recipe.update(model_id="bench_top_mask_v1", perception_backend="bench_hsv")
        approval = recipe["experiment_approval"] = {
            "experiment": 3, "stages": ["hover", "full"],
            "experiment_1_evidence_ref": "E1", "experiment_2_evidence_ref": "E2",
            "wrist_open_evidence_ref": "wrist opening",
            "vision_validation_ref": "bench images",
        }
        with tempfile.TemporaryDirectory() as tmp:
            hover_path = Path(tmp) / "hover.json"
            bench_path = Path(tmp) / "bench.json"
            approval.update(hover_report=str(hover_path), bench_full_report=str(bench_path))
            bench = _experiment_3_metadata("hover", recipe, routes)
            hover_facts = {"checkpoint": "verify_above_box", "terminal_pose": "ABOVE_BOX",
                           "full_cycle": False}
            hover_path.write_text(json.dumps({
                **bench, "status": "ok", "backend": "robot",
                "experiment": 3, "stage": "hover", "data": hover_facts,
            }), encoding="utf-8")
            _experiment_3_metadata("full", recipe, routes)
            bench_path.write_text(json.dumps({
                **bench, "status": "ok", "backend": "robot",
                "experiment": 3, "stage": "full", "data": {"box_in_bin": True},
            }), encoding="utf-8")

            recipe.update(model_id="trained_box_v1", perception_backend="yolo", weights_sha256="a"*64)
            yolo = _experiment_3_metadata("hover", recipe, routes)
            with self.assertRaisesRegex(ValueError, "当前配置不符"):
                _experiment_3_metadata("full", recipe, routes)
            hover_path.write_text(json.dumps({
                **yolo, "status": "ok", "backend": "robot",
                "experiment": 3, "stage": "hover", "data": hover_facts,
            }), encoding="utf-8")
            _experiment_3_metadata("full", recipe, routes)
            recipe["weights_sha256"] = "b"*64
            with self.assertRaisesRegex(ValueError, "当前配置不符"):
                _experiment_3_metadata("full", recipe, routes)

    def test_client_rejects_unexpected_weights_at_preflight_and_inference(self):
        client = YoloHttpClient("http://127.0.0.1:8010", model_id="box_v1",
                                perception_backend="yolo", weights_sha256="a"*64)
        response = {"schema": "drawer_box_seg/1", "status": "ok", "model_id": "box_v1",
                    "perception_backend": "yolo", "weights_sha256": "b"*64}
        with patch.object(client, "_json", return_value=response):
            self.assertFalse(client.health())
            from types import SimpleNamespace
            frame = SimpleNamespace(image_ref="capture.jpg", frame_id="one",
                                    camera_id="head", capture_timestamp_ns=1, image_size_wh=(64, 48))
            with self.assertRaises(LinkError):
                client.segment(frame, 1)

    def test_p0_binding_rejects_old_rise_endpoint(self):
        routes = RouteStore.load(DEFAULT_ROUTES)
        recipe = offline_recipe()
        recipe["waypoint_refs"]["OBSERVE_HIGH"] = "P0"
        pose = routes.waypoint("OBSERVE_HIGH")
        pose[2] += 0.02
        routes.data["waypoints"]["P0"] = pose
        with self.assertRaisesRegex(ValueError, "终点与观察点 P0 不一致"):
            routes.check_observation_endpoint(recipe)
        routes.data["waypoint_joints_rad"] = {"P0": {f"q{i}": 0 for i in range(7)}}
        routes.route("rise_to_observe")[-1]["pose"] = pose
        routes.check_observation_endpoint(recipe)
