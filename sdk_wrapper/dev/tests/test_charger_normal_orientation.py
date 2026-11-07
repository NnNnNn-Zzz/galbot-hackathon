import math
import unittest

import numpy as np

from sdk_wrapper.dev.algorithms.charger_yolo import YoloInstance, analyze_charger_top


def _rectangle_instance() -> YoloInstance:
    mask = np.zeros((512, 640), dtype=bool)
    mask[180:270, 180:420] = True
    return YoloInstance(
        class_id=0,
        class_name="charger",
        confidence=0.9,
        bbox_xyxy=(180.0, 180.0, 420.0, 270.0),
        mask=mask,
    )


class ChargerNormalOrientationTest(unittest.TestCase):
    def test_small_negative_x_tilt_keeps_normal_pointing_up(self):
        angle = math.radians(-10.0)
        extrinsic = [
            0.35,
            0.12,
            0.55,
            0.0,
            math.sin(angle / 2.0),
            0.0,
            math.cos(angle / 2.0),
        ]

        result, _ = analyze_charger_top(
            _rectangle_instance(),
            (512, 640),
            depth_m=np.full((512, 640), 0.45, dtype=np.float32),
            intrinsics={"fx": 520.0, "fy": 520.0, "cx": 320.0, "cy": 256.0},
            extrinsic=extrinsic,
            max_surface_tilt_deg=30.0,
        )

        self.assertTrue(result["surface"]["available"])
        self.assertGreater(result["surface"]["surface_normal_base"][2], 0.0)
        self.assertAlmostEqual(
            result["surface"]["tilt_from_base_up_deg"],
            10.0,
            delta=0.5,
        )


if __name__ == "__main__":
    unittest.main()
