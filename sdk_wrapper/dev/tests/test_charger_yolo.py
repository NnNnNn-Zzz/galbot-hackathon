import tempfile
import unittest
from pathlib import Path

import numpy as np

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.algorithms.charger_yolo import YoloInstance, analyze_charger_top
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


def rectangle_instance(
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    *,
    confidence: float = 0.9,
) -> YoloInstance:
    shape = (512, 640)
    mask = np.zeros(shape, dtype=bool)
    mask[y0:y1, x0:x1] = True
    return YoloInstance(
        class_id=0,
        class_name="charger",
        confidence=confidence,
        bbox_xyxy=(float(x0), float(y0), float(x1), float(y1)),
        mask=mask,
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


class ChargerYoloToolTest(unittest.TestCase):
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
