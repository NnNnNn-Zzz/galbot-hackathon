"""Two-stage charger YOLO proposal and ROI refinement.

The clean segmentation model is the precision authority. If it has no stable
full-frame detection, a high-recall proposal model may identify candidate ROIs,
but those proposals remain explicitly marked and cannot be promoted to grasp
coordinates without a clean mask from the refinement stage.
"""

from __future__ import annotations

import os
import typing
from pathlib import Path

import numpy as np

from sdk_wrapper.dev.algorithms.charger_yolo import (
    UltralyticsYoloBackend,
    YoloInstance,
)

__all__ = ["CascadeChargerSegmentationBackend"]


class CascadeChargerSegmentationBackend:
    """Run charger proposals first only when the precise model cannot decide."""

    def __init__(
        self,
        roots: typing.Sequence[str | os.PathLike[str]],
        *,
        weights: str | os.PathLike[str],
        proposal_weights: str | os.PathLike[str],
        target_class: str = "charger",
        imgsz: int = 640,
        refine_confidence: float = 0.01,
        full_accept_confidence: float = 0.20,
        proposal_confidence: float = 0.02,
        iou: float = 0.45,
        device: str = "cpu",
        roi_margin: float = 0.25,
        proposal_limit: int = 3,
        refine_limit: int = 1,
        min_bbox_iou: float = 0.15,
        refine_backend: typing.Any = None,
        proposal_backend: typing.Any = None,
    ) -> None:
        self.roots = [os.fspath(root) for root in roots]
        self.weights = os.fspath(weights)
        self.proposal_weights = os.fspath(proposal_weights)
        self.target_class = str(target_class)
        self.imgsz = int(imgsz)
        self.refine_confidence = float(refine_confidence)
        self.full_accept_confidence = float(full_accept_confidence)
        self.proposal_confidence = float(proposal_confidence)
        self.iou = float(iou)
        self.device = str(device)
        self.roi_margin = float(roi_margin)
        self.proposal_limit = max(1, int(proposal_limit))
        self.refine_limit = max(1, int(refine_limit))
        self.min_bbox_iou = float(min_bbox_iou)
        self.last_report: dict[str, typing.Any] = {}

        self._refine_backend = refine_backend or UltralyticsYoloBackend(
            self.roots,
            weights=self.weights,
            target_class=self.target_class,
            required_task="segment",
            imgsz=self.imgsz,
            confidence=self.refine_confidence,
            iou=self.iou,
            device=self.device,
        )
        self._proposal_backend = proposal_backend or UltralyticsYoloBackend(
            self.roots,
            weights=self.proposal_weights,
            target_class=self.target_class,
            required_task="segment",
            imgsz=self.imgsz,
            confidence=self.proposal_confidence,
            iou=self.iou,
            device=self.device,
        )

    def describe(self) -> dict[str, typing.Any]:
        refine = dict(self._refine_backend.describe())
        proposal = dict(self._proposal_backend.describe())
        return {
            "strategy": "clean_full_then_proposal_roi",
            "weights": refine,
            "proposal_weights": proposal,
            "full_accept_confidence": self.full_accept_confidence,
            "proposal_confidence": self.proposal_confidence,
            "refine_confidence": self.refine_confidence,
            "imgsz": self.imgsz,
            "device": self.device,
        }

    @staticmethod
    def _bbox_iou(first, second) -> float:
        x1 = max(float(first[0]), float(second[0]))
        y1 = max(float(first[1]), float(second[1]))
        x2 = min(float(first[2]), float(second[2]))
        y2 = min(float(first[3]), float(second[3]))
        intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if intersection <= 0.0:
            return 0.0
        area_first = max(0.0, float(first[2]) - float(first[0])) * max(
            0.0, float(first[3]) - float(first[1])
        )
        area_second = max(0.0, float(second[2]) - float(second[0])) * max(
            0.0, float(second[3]) - float(second[1])
        )
        return intersection / max(1e-9, area_first + area_second - intersection)

    @staticmethod
    def _with_stage(
        instance: YoloInstance,
        stage: str,
        proposal_confidence: float | None = None,
        *,
        bbox_xyxy=None,
        mask: np.ndarray | None = None,
        drop_mask: bool = False,
    ) -> YoloInstance:
        output_mask = None if drop_mask else (instance.mask if mask is None else mask)
        return YoloInstance(
            class_id=int(instance.class_id),
            class_name=str(instance.class_name),
            confidence=float(instance.confidence),
            bbox_xyxy=tuple(
                float(value)
                for value in (
                    instance.bbox_xyxy if bbox_xyxy is None else bbox_xyxy
                )
            ),
            mask=output_mask,
            track_id=instance.track_id,
            stage=stage,
            proposal_confidence=proposal_confidence,
        )

    def _dedupe(self, instances: list[YoloInstance]) -> list[YoloInstance]:
        selected: list[YoloInstance] = []
        for instance in sorted(
            instances, key=lambda item: float(item.confidence), reverse=True
        ):
            if any(
                self._bbox_iou(instance.bbox_xyxy, previous.bbox_xyxy)
                >= 0.60
                for previous in selected
            ):
                continue
            selected.append(instance)
            if len(selected) >= self.proposal_limit:
                break
        return selected

    @staticmethod
    def _promote_to_full(
        instance: YoloInstance,
        origin: tuple[int, int],
        image_shape: tuple[int, ...],
        proposal_confidence: float,
    ) -> YoloInstance:
        x_offset, y_offset = origin
        x1, y1, x2, y2 = [float(value) for value in instance.bbox_xyxy]
        mask = None
        if instance.mask is not None:
            source_mask = np.asarray(instance.mask, dtype=bool)
            mask = np.zeros(image_shape[:2], dtype=bool)
            height, width = source_mask.shape[:2]
            target_x2 = min(image_shape[1], x_offset + width)
            target_y2 = min(image_shape[0], y_offset + height)
            mask[y_offset:target_y2, x_offset:target_x2] = source_mask[
                : target_y2 - y_offset,
                : target_x2 - x_offset,
            ]
        return CascadeChargerSegmentationBackend._with_stage(
            instance,
            "refined",
            proposal_confidence,
            bbox_xyxy=(
                x1 + x_offset,
                y1 + y_offset,
                x2 + x_offset,
                y2 + y_offset,
            ),
            mask=mask,
        )

    def _proposal_roi_refinement(
        self, image: np.ndarray, proposals: list[YoloInstance]
    ) -> list[YoloInstance]:
        results: list[YoloInstance] = []
        for index, proposal in enumerate(self._dedupe(proposals)):
            if index >= self.refine_limit:
                results.append(
                    self._with_stage(
                        proposal,
                        "proposal_only",
                        float(proposal.confidence),
                        drop_mask=True,
                    )
                )
                continue
            x1, y1, x2, y2 = [
                int(round(float(value))) for value in proposal.bbox_xyxy
            ]
            width = max(1, x2 - x1)
            height = max(1, y2 - y1)
            pad_x = max(15, int(width * self.roi_margin))
            pad_y = max(15, int(height * self.roi_margin))
            crop_x1 = max(0, x1 - pad_x)
            crop_y1 = max(0, y1 - pad_y)
            crop_x2 = min(image.shape[1], x2 + pad_x)
            crop_y2 = min(image.shape[0], y2 + pad_y)
            if crop_x2 <= crop_x1 or crop_y2 <= crop_y1:
                results.append(
                    self._with_stage(
                        proposal,
                        "proposal_only",
                        float(proposal.confidence),
                        drop_mask=True,
                    )
                )
                continue
            crop = image[crop_y1:crop_y2, crop_x1:crop_x2]
            promoted = []
            for instance in self._refine_backend.predict(crop):
                full_instance = self._promote_to_full(
                    instance,
                    (crop_x1, crop_y1),
                    image.shape,
                    float(proposal.confidence),
                )
                if (
                    self._bbox_iou(
                        proposal.bbox_xyxy,
                        full_instance.bbox_xyxy,
                    )
                    >= self.min_bbox_iou
                ):
                    promoted.append(full_instance)
            if promoted:
                promoted.sort(
                    key=lambda item: float(item.confidence), reverse=True
                )
                results.append(promoted[0])
            else:
                results.append(
                    self._with_stage(
                        proposal,
                        "proposal_only",
                        float(proposal.confidence),
                        drop_mask=True,
                    )
                )
        return results

    def predict_cascade(
        self,
        bgr: np.ndarray,
        *,
        purpose: str,
    ) -> list[YoloInstance]:
        image = np.asarray(bgr)
        if image.ndim != 3 or image.dtype != np.uint8:
            raise ValueError("bgr must be HxWx3 uint8")

        clean_full = self._refine_backend.predict(image)
        stable = [
            instance
            for instance in clean_full
            if float(instance.confidence) >= self.full_accept_confidence
        ]
        if purpose == "precision" and stable:
            stable = [
                self._with_stage(instance, "refined")
                for instance in sorted(
                    stable,
                    key=lambda item: float(item.confidence),
                    reverse=True,
                )
            ]
            self.last_report = {
                "strategy": "clean_full",
                "clean_full_count": len(clean_full),
                "stable_count": len(stable),
                "proposal_count": 0,
                "refined_count": len(stable),
                "proposal_only_count": 0,
            }
            return stable

        proposals = self._proposal_backend.predict(image)
        if purpose == "precision":
            results = self._proposal_roi_refinement(image, proposals)
        else:
            # Head handoff needs a proposal even when ROI refinement is absent.
            clean_candidates = [
                self._with_stage(instance, "refined")
                for instance in stable
            ]
            results = self._proposal_roi_refinement(image, proposals)
            results.extend(clean_candidates)
        self.last_report = {
            "strategy": "proposal_roi",
            "clean_full_count": len(clean_full),
            "stable_count": len(stable),
            "proposal_count": len(proposals),
            "refined_count": sum(item.stage == "refined" for item in results),
            "proposal_only_count": sum(
                item.stage == "proposal_only" for item in results
            ),
        }
        return results

    def predict(self, bgr: np.ndarray) -> list[YoloInstance]:
        """Compatibility entry point: wrist precision by default."""
        return self.predict_cascade(bgr, purpose="precision")
