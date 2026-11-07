import tempfile
import unittest
from pathlib import Path

import numpy as np

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.algorithms.charger_yolo import YoloInstance
from sdk_wrapper.dev.algorithms.power_strip_yolo import (
    analyze_power_strip_box,
    analyze_power_strip_stereo,
)
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool
from sdk_wrapper.dev.robot_tools.power_strip_yolo import PowerStripYoloTool


def strip_instance(x1, y1, x2, y2, confidence=0.85):
    return YoloInstance(
        class_id=0,
        class_name="power_strip",
        confidence=confidence,
        bbox_xyxy=(float(x1), float(y1), float(x2), float(y2)),
        mask=None,
    )


class FakeStripBackend:
    def __init__(self, instances):
        self.instances = list(instances)

    def describe(self):
        return {
            "weights": "/fake/strip/best.pt",
            "task": "detect",
            "classes": {0: "power_strip"},
            "target_class_id": 0,
        }

    def predict(self, bgr):
        return list(self.instances)


class PowerStripGeometryTest(unittest.TestCase):
    def test_visible_box_exports_bearing_but_no_grasp_coordinate(self):
        instance = strip_instance(360, 180, 480, 220)
        result = analyze_power_strip_box(
            instance,
            (512, 640),
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
        )

        self.assertTrue(result["usable_for_direction"])
        self.assertFalse(result["usable_for_metric_distance"])
        self.assertIsNone(result["metric_target_base_m"])
        self.assertIsNone(result["grasp_coordinate"])
        self.assertGreater(result["horizontal_angle_deg"], 0)
        self.assertLess(result["vertical_angle_deg"], 0)

    def test_clipped_box_is_rejected_for_direction(self):
        instance = strip_instance(0, 180, 480, 220)
        result = analyze_power_strip_box(instance, (512, 640))

        self.assertFalse(result["usable_for_direction"])
        self.assertIn("bbox_touches_image_edge", result["rejection_reasons"])
        self.assertIsNone(result["grasp_coordinate"])

    def test_stereo_measurement_requires_scale_and_stable_values(self):
        depth_raw = np.full((256, 320), 800, dtype=np.uint16)
        result = analyze_power_strip_stereo(
            depth_raw,
            [80, 60, 240, 160],
            (256, 320),
            raw_to_m=0.001,
            rgb_timestamp_ns=1_000_000_000,
            depth_timestamp_ns=1_010_000_000,
        )

        self.assertTrue(result["available"])
        self.assertTrue(result["measurement_stable"])
        self.assertTrue(result["metric_distance_available"])
        self.assertAlmostEqual(result["metric_distance_m"], 0.8)

        unscaled = analyze_power_strip_stereo(
            depth_raw,
            [80, 60, 240, 160],
            (256, 320),
            raw_to_m=None,
        )
        self.assertFalse(unscaled["metric_distance_available"])
        self.assertEqual(unscaled["unit"], "sdk_raw_unverified")


class PowerStripToolTest(unittest.TestCase):
    def test_tool_returns_direction_only_observation(self):
        adapter = MockAdapter()
        perception = PerceptionTool(adapter)
        backend = FakeStripBackend([strip_instance(300, 180, 430, 230)])

        with tempfile.TemporaryDirectory() as directory:
            tool = PowerStripYoloTool(
                perception,
                backend=backend,
                output_dir=Path(directory),
            )
            result = tool.detect_power_strip_head()

            self.assertEqual(result.status, TaskStatus.SUCCEEDED)
            self.assertEqual(result.data["approach_mode"], "bearing_only")
            self.assertIsNone(result.data["grasp_coordinate"])
            self.assertTrue(Path(result.data["overlay_ref"]).is_file())
            self.assertTrue(Path(result.data["observation_ref"]).is_file())

    def test_missing_detection_returns_not_ready(self):
        perception = PerceptionTool(MockAdapter())
        with tempfile.TemporaryDirectory() as directory:
            tool = PowerStripYoloTool(
                perception,
                backend=FakeStripBackend([]),
                output_dir=Path(directory),
            )
            result = tool.detect_power_strip_head()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)


if __name__ == "__main__":
    unittest.main()
