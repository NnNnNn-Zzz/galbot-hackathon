import unittest
import tempfile
from pathlib import Path

import numpy as np

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.algorithms.charger_cascade import (
    CascadeChargerSegmentationBackend,
)
from sdk_wrapper.dev.algorithms.charger_yolo import YoloInstance
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


def instance(bbox, confidence, shape, *, mask=True):
    x1, y1, x2, y2 = bbox
    pixels = None
    if mask:
        pixels = np.zeros(shape, dtype=bool)
        pixels[y1:y2, x1:x2] = True
    return YoloInstance(
        class_id=0,
        class_name="charger",
        confidence=confidence,
        bbox_xyxy=tuple(float(value) for value in bbox),
        mask=pixels,
    )


class StaticBackend:
    def __init__(self, values):
        self.values = list(values)
        self.calls = 0

    def describe(self):
        return {
            "weights": "/fake/model.pt",
            "task": "segment",
            "classes": {0: "charger"},
        }

    def predict(self, bgr):
        self.calls += 1
        return list(self.values)


class RefineAfterFullMiss:
    def __init__(self):
        self.calls = 0

    def describe(self):
        return {
            "weights": "/fake/refine.pt",
            "task": "segment",
            "classes": {0: "charger"},
        }

    def predict(self, bgr):
        self.calls += 1
        if self.calls == 1:
            return []
        return [
            instance(
                (15, 15, min(55, bgr.shape[1]), min(55, bgr.shape[0])),
                0.8,
                bgr.shape[:2],
            )
        ]


def make_cascade(refine, proposal):
    return CascadeChargerSegmentationBackend(
        [],
        weights="/fake/refine.pt",
        proposal_weights="/fake/proposal.pt",
        refine_backend=refine,
        proposal_backend=proposal,
    )


class CascadeChargerBackendTest(unittest.TestCase):
    def test_stable_clean_full_result_skips_proposal_model(self):
        clean = StaticBackend([instance((20, 20, 70, 70), 0.8, (100, 100))])
        proposal = StaticBackend([])
        backend = make_cascade(clean, proposal)

        results = backend.predict_cascade(
            np.zeros((100, 100, 3), dtype=np.uint8),
            purpose="precision",
        )

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].stage, "refined")
        self.assertEqual(proposal.calls, 0)
        self.assertEqual(backend.last_report["strategy"], "clean_full")

    def test_clean_miss_promotes_overlapping_roi_result_to_full_image(self):
        refine = RefineAfterFullMiss()
        proposal = StaticBackend([instance((30, 30, 90, 90), 0.12, (140, 160))])
        backend = make_cascade(refine, proposal)

        results = backend.predict_cascade(
            np.zeros((140, 160, 3), dtype=np.uint8),
            purpose="precision",
        )

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].stage, "refined")
        self.assertEqual(results[0].mask.shape, (140, 160))
        self.assertGreaterEqual(results[0].bbox_xyxy[0], 30)
        self.assertEqual(backend.last_report["refined_count"], 1)

    def test_unconfirmed_proposal_has_no_mask_and_is_not_precision(self):
        proposal = StaticBackend([instance((30, 30, 90, 90), 0.12, (140, 160))])
        backend = make_cascade(StaticBackend([]), proposal)

        results = backend.predict_cascade(
            np.zeros((140, 160, 3), dtype=np.uint8),
            purpose="precision",
        )

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].stage, "proposal_only")
        self.assertIsNone(results[0].mask)
        self.assertEqual(backend.last_report["proposal_only_count"], 1)


class ProposalOnlyToolBoundaryTest(unittest.TestCase):
    def test_wrist_tool_rejects_proposal_without_clean_mask(self):
        class ProposalOnlyCascade:
            def __init__(self):
                self.last_report = {
                    "strategy": "proposal_roi",
                    "refined_count": 0,
                    "proposal_only_count": 1,
                }

            def describe(self):
                return {
                    "strategy": "clean_full_then_proposal_roi",
                    "weights": {"weights": "/fake/refine.pt"},
                    "proposal_weights": {"weights": "/fake/proposal.pt"},
                }

            def predict_cascade(self, bgr, *, purpose):
                return [
                    YoloInstance(
                        class_id=0,
                        class_name="charger",
                        confidence=0.12,
                        bbox_xyxy=(100.0, 100.0, 260.0, 220.0),
                        mask=None,
                        stage="proposal_only",
                        proposal_confidence=0.12,
                    )
                ]

        perception = PerceptionTool(MockAdapter())
        with tempfile.TemporaryDirectory() as directory:
            tool = ChargerYoloTool(
                perception,
                backend=ProposalOnlyCascade(),
                output_dir=Path(directory),
            )
            result = tool.segment_charger_top_wrist()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertEqual(result.data["cascade"]["proposal_only_count"], 1)


if __name__ == "__main__":
    unittest.main()
