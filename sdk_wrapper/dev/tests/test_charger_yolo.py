import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.algorithms.charger_cascade import (
    WholeRoiTopSurfaceBackend,
)
from sdk_wrapper.dev.algorithms.charger_yolo import (
    YoloInstance,
    analyze_charger_top,
    analyze_charger_whole_fallback,
)
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus, ok
from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


def rectangle_instance(
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    *,
    confidence: float = 0.9,
    class_name: str = "charger",
    stage: str = "refined",
) -> YoloInstance:
    shape = (512, 640)
    mask = np.zeros(shape, dtype=bool)
    mask[y0:y1, x0:x1] = True
    return YoloInstance(
        class_id=0,
        class_name=class_name,
        confidence=confidence,
        bbox_xyxy=(float(x0), float(y0), float(x1), float(y1)),
        mask=mask,
        stage=stage,
    )


class ConstantBackend:
    def __init__(self, instances):
        self.instances = list(instances)
        self.calls = 0

    def describe(self):
        return {
            "weights": "/fake/charger/best.pt",
            "task": "segment",
            "classes": {0: "charger"},
            "target_class_id": 0,
        }

    def predict(self, bgr):
        self.calls += 1
        return list(self.instances)


class FirstMissBackend(ConstantBackend):
    def predict(self, bgr):
        self.calls += 1
        if self.calls == 1:
            return []
        return list(self.instances)


class FakeStereoTool:
    def __init__(self, depth, directory):
        self.depth = depth
        self.directory = Path(directory)

    def capture_stereo_depth(self, timeout_s=8.0):
        path = self.directory / "fake_stereo_depth.npy"
        np.save(path, self.depth, allow_pickle=False)
        return ok(
            "stereo",
            "mock",
            data={
                "capture_id": "fake_stereo",
                "depth_ref": str(path),
                "timestamp_ns": 0,
                "sensor_name": "head_left_camera_color_optical_frame",
            },
        )


class ChargerYoloGeometryTest(unittest.TestCase):
    def test_complete_top_surface_produces_grasp_frame(self):
        instance = rectangle_instance(180, 180, 420, 270)
        depth = np.full((512, 640), 0.45, dtype=np.float32)
        intrinsics = type(
            "Intrinsics",
            (),
            {"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
        )()

        result, mask = analyze_charger_top(
            instance,
            (512, 640),
            depth_m=depth,
            intrinsics=intrinsics,
            extrinsic=[0.35, 0.12, 0.55, 0.0, 0.0, 0.0, 1.0],
            min_short_edge_px=24,
            safe_standoff_m=0.12,
        )

        self.assertTrue(result["eligible_for_precision"])
        self.assertTrue(result["image_space"]["fully_visible"])
        self.assertGreater(result["image_space"]["long_edge_px"], 200)
        self.assertGreater(result["depth"]["valid_ratio"], 0.99)
        self.assertTrue(result["surface"]["available"])
        self.assertAlmostEqual(result["surface"]["short_dimension_m"], 0.09, delta=0.03)
        self.assertGreater(result["surface"]["pregrasp_position_m"][2], result["surface"]["surface_center_m"][2])
        self.assertEqual(mask.shape, (512, 640))

    def test_large_but_clipped_target_is_rejected(self):
        instance = rectangle_instance(0, 0, 520, 330)

        result, _ = analyze_charger_top(
            instance,
            (512, 640),
            min_area_px=600,
            min_short_edge_px=24,
        )

        self.assertFalse(result["eligible_for_precision"])
        self.assertIn("target_touches_image_edge", result["rejection_reasons"])
        self.assertGreater(result["image_space"]["mask_area_px"], 100000)

    def test_large_but_thin_target_is_rejected_by_short_edge(self):
        instance = rectangle_instance(40, 200, 600, 215)

        result, _ = analyze_charger_top(
            instance,
            (512, 640),
            min_area_px=600,
            min_short_edge_px=24,
        )

        self.assertFalse(result["eligible_for_precision"])
        self.assertIn("short_edge_below_minimum", result["rejection_reasons"])

    def test_large_area_with_invalid_depth_is_rejected(self):
        instance = rectangle_instance(180, 180, 420, 270)
        depth = np.full((512, 640), np.nan, dtype=np.float32)

        result, _ = analyze_charger_top(
            instance,
            (512, 640),
            depth_m=depth,
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
            extrinsic=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )

        self.assertFalse(result["eligible_for_precision"])
        self.assertIn("depth_valid_ratio_below_minimum", result["rejection_reasons"])
        self.assertIn("surface_3d_unavailable", result["rejection_reasons"])

    def test_side_surface_with_small_dimensions_and_large_tilt_is_rejected(self):
        instance = rectangle_instance(180, 180, 420, 270)
        depth = np.full((512, 640), 0.45, dtype=np.float32)
        angle = math.radians(60.0)
        tilted_extrinsic = [
            0.35,
            0.12,
            0.55,
            0.0,
            math.sin(angle / 2.0),
            0.0,
            math.cos(angle / 2.0),
        ]

        result, _ = analyze_charger_top(
            instance,
            (512, 640),
            depth_m=depth,
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
            extrinsic=tilted_extrinsic,
            min_short_dimension_m=0.04,
            max_surface_tilt_deg=30.0,
        )

        self.assertFalse(result["eligible_for_precision"])
        self.assertIn(
            "surface_normal_tilt_above_maximum",
            result["rejection_reasons"],
        )

    def test_compact_charger_top_above_calibrated_minimums_is_accepted(self):
        instance = rectangle_instance(250, 200, 285, 230)
        depth = np.full((512, 640), 0.45, dtype=np.float32)

        result, _ = analyze_charger_top(
            instance,
            (512, 640),
            depth_m=depth,
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
            extrinsic=[0.35, 0.12, 0.55, 0.0, 0.0, 0.0, 1.0],
            min_long_dimension_m=0.02,
            min_short_dimension_m=0.015,
            max_surface_tilt_deg=30.0,
        )

        self.assertTrue(result["eligible_for_precision"])
        self.assertGreaterEqual(result["surface"]["long_dimension_m"], 0.02)
        self.assertGreaterEqual(result["surface"]["short_dimension_m"], 0.015)

    def test_whole_mask_fallback_randomly_samples_a_valid_local_plane(self):
        instance = rectangle_instance(
            180,
            180,
            420,
            270,
            class_name="charger_whole",
            stage="whole_fallback",
        )
        result, mask = analyze_charger_whole_fallback(
            instance,
            (512, 640),
            depth_m=np.full((512, 640), 0.45, dtype=np.float32),
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
            extrinsic=[0.35, 0.12, 0.55, 0.0, 0.0, 0.0, 1.0],
            sample_attempts=16,
            sample_radius_px=8,
        )

        self.assertTrue(result["eligible_for_precision"])
        self.assertEqual(result["target_mode"], "whole_mask_random_sample")
        self.assertTrue(result["surface"]["available"])
        self.assertTrue(result["surface"]["sampled_target"])
        sampled_uv = result["whole_fallback_sampling"]["sampled_uv"]
        self.assertIsNotNone(sampled_uv)
        self.assertTrue(mask[int(sampled_uv[1]), int(sampled_uv[0])])

    def test_whole_mask_fallback_without_depth_is_rejected(self):
        instance = rectangle_instance(
            180,
            180,
            420,
            270,
            class_name="charger_whole",
            stage="whole_fallback",
        )
        result, _ = analyze_charger_whole_fallback(
            instance,
            (512, 640),
        )

        self.assertFalse(result["eligible_for_precision"])
        self.assertIn("depth_unavailable", result["rejection_reasons"])


class ChargerYoloToolTest(unittest.TestCase):
    def test_default_config_uses_charger_whole_backend(self):
        tool = ChargerYoloTool(PerceptionTool(MockAdapter()))

        self.assertIsInstance(tool.backend, WholeRoiTopSurfaceBackend)
        self.assertEqual(tool.config["target_class"], "charger_whole")
        self.assertEqual(
            tool.config["weights"],
            "~/yoloseg/runs/charger_seg_whole_final/weights/best.pt",
        )
        self.assertEqual(
            tool.config["cascade"]["top_surface_weights"],
            "~/yoloseg/runs/charger_seg_clean_v2/weights/best.pt",
        )

    def test_left_arm_override_switches_wrist_rgb_and_depth(self):
        tool = ChargerYoloTool(
            PerceptionTool(MockAdapter()),
            arm="left",
        )

        self.assertEqual(tool.config["wrist_camera"], "left_arm")
        self.assertEqual(tool.config["wrist_depth_camera"], "left_arm_depth")

    def test_wrist_tool_returns_structured_observation_and_artifacts(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([rectangle_instance(120, 360, 520, 430)])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

            self.assertEqual(result.status, TaskStatus.SUCCEEDED)
            self.assertEqual(result.mode.value, "mock")
            self.assertEqual(set(result.to_dict()), {
                "status", "mode", "message", "task_id", "data", "error_code"
            })
            self.assertTrue(result.data["eligible_for_precision"])
            self.assertTrue(Path(result.data["mask_ref"]).is_file())
            self.assertTrue(Path(result.data["overlay_ref"]).is_file())
            self.assertTrue(Path(result.data["observation_ref"]).is_file())
            self.assertEqual(result.data["depth_rgb_alignment"], "calibration_match")
            self.assertTrue(result.data["depth_alignment_verified"])
            self.assertEqual(backend.calls, 1)

    def test_valid_top_surface_wins_over_higher_confidence_whole_fallback(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([
            rectangle_instance(
                160,
                170,
                440,
                300,
                confidence=0.99,
                class_name="charger_whole",
                stage="whole_fallback",
            ),
            rectangle_instance(
                190,
                190,
                410,
                270,
                confidence=0.55,
                class_name="charger",
                stage="top_surface",
            ),
        ])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["detection_stage"], "top_surface")
        self.assertTrue(result.data["eligible_for_precision"])

    def test_whole_fallback_is_selected_when_top_surface_is_invalid(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([
            rectangle_instance(
                0,
                0,
                520,
                330,
                confidence=0.95,
                class_name="charger",
                stage="top_surface",
            ),
            rectangle_instance(
                160,
                170,
                440,
                300,
                confidence=0.80,
                class_name="charger_whole",
                stage="whole_fallback",
            ),
        ])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["detection_stage"], "whole_fallback")
        self.assertEqual(
            result.data["target_mode"],
            "whole_mask_random_sample",
        )
        self.assertTrue(result.data["eligible_for_precision"])

    def test_head_detection_can_handoff_to_wrist_without_grasp_coordinate(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([rectangle_instance(180, 180, 420, 270)])

        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(
                (Path(__file__).resolve().parents[1] / "config" / "charger_yolo.json").read_text(
                    encoding="utf-8"
                )
            )
            config["stereo"]["raw_to_m"] = 0.001
            config["target_safety"]["min_range_m"] = 0.30
            config_path = Path(directory) / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                stereo_tool=FakeStereoTool(
                    np.full((512, 640), 900, dtype=np.uint16),
                    directory,
                ),
                config_path=config_path,
                output_dir=Path(directory),
            )
            result = tool.detect_charger_head()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["head_handoff"]["usable_for_wrist_handoff"])
        self.assertTrue(result.data["head_handoff"]["target_safety"]["safe"])
        self.assertTrue(
            result.data["head_handoff"]["target_3d"]["available"]
        )
        self.assertIsNone(result.data["head_handoff"]["grasp_coordinate"])
        self.assertFalse(result.data["eligible_for_precision"])

    def test_missing_depth_keeps_detection_but_blocks_precision(self):
        adapter = MockAdapter(fail_depth=True)
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([rectangle_instance(180, 180, 420, 270)])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertFalse(result.data["eligible_for_precision"])
        self.assertIn("depth_unavailable", result.data["rejection_reasons"])

    def test_default_calibration_match_allows_precision(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([rectangle_instance(120, 360, 520, 430)])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["depth_alignment_verified"])
        self.assertTrue(result.data["eligible_for_precision"])

    def test_wrist_tool_uses_whole_mask_random_sample_fallback(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([
            rectangle_instance(
                120,
                360,
                520,
                430,
                class_name="charger_whole",
                stage="whole_fallback",
            )
        ])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(
            result.data["target_mode"],
            "whole_mask_random_sample",
        )
        self.assertTrue(result.data["eligible_for_precision"])
        self.assertIsNotNone(
            result.data["whole_fallback_sampling"]["sampled_uv"]
        )

    def test_precision_capture_retries_a_transient_empty_frame(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = FirstMissBackend([
            rectangle_instance(
                120,
                360,
                520,
                430,
                class_name="charger_whole",
                stage="whole_fallback",
            )
        ])
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(
                (
                    Path(__file__).resolve().parents[1]
                    / "config"
                    / "charger_yolo.json"
                ).read_text(encoding="utf-8")
            )
            config["capture_retry_delay_s"] = 0.0
            config_path = Path(directory) / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                config_path=config_path,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(backend.calls, 2)
        self.assertEqual(result.data["capture_attempt"], 2)

    def test_precision_capture_retries_an_empty_adapter_payload(self):
        adapter = MockAdapter()
        original = adapter.get_rgb_data
        state = {"failed": False}

        def flaky_rgb(sensor):
            if sensor == "RIGHT_ARM_CAMERA" and not state["failed"]:
                state["failed"] = True
                return None
            return original(sensor)

        adapter.get_rgb_data = flaky_rgb
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([
            rectangle_instance(
                120,
                360,
                520,
                430,
                class_name="charger_whole",
                stage="whole_fallback",
            )
        ])
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(
                (
                    Path(__file__).resolve().parents[1]
                    / "config"
                    / "charger_yolo.json"
                ).read_text(encoding="utf-8")
            )
            config["capture_retry_delay_s"] = 0.0
            config_path = Path(directory) / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                config_path=config_path,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(result.data["capture_attempt"], 2)
        self.assertEqual(
            result.data["capture_attempt_reports"][0]["reason"],
            "capture_error",
        )

    def test_valid_frames_with_no_target_stop_at_detection_limit(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([])
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(
                (
                    Path(__file__).resolve().parents[1]
                    / "config"
                    / "charger_yolo.json"
                ).read_text(encoding="utf-8")
            )
            config["capture_attempts"] = 20
            config["detection_attempts"] = 3
            config["capture_retry_delay_s"] = 0.0
            config_path = Path(directory) / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                config_path=config_path,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(backend.calls, 3)
        self.assertEqual(result.data["capture_attempts"], 3)

    def test_rgbd_calibration_mismatch_blocks_precision(self):
        adapter = MockAdapter()
        original = adapter.get_sensor_extrinsic

        def mismatched(sensor, reference_frame="base_link"):
            transform, timestamp = original(sensor, reference_frame)
            if sensor == "RIGHT_ARM_DEPTH_CAMERA":
                transform = list(transform)
                transform[0] += 0.01
            return transform, timestamp

        adapter.get_sensor_extrinsic = mismatched
        perception = PerceptionTool(adapter)
        backend = ConstantBackend([rectangle_instance(120, 360, 520, 430)])

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertFalse(result.data["depth_alignment_verified"])
        self.assertFalse(result.data["eligible_for_precision"])
        self.assertIn(
            "depth_rgb_calibration_mismatch",
            result.data["rejection_reasons"],
        )

    def test_no_target_returns_not_ready(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)

        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=ConstantBackend([]),
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)

    def test_unavailable_model_returns_not_ready_before_camera_capture(self):
        class UnavailableBackend:
            def describe(self):
                raise RuntimeError("model runtime missing")

        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=UnavailableBackend(),
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(
            perception.frames.list_recent(limit=10),
            [],
        )


if __name__ == "__main__":
    unittest.main()
