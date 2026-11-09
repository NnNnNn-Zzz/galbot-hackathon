"""Tool-layer charger recognition on head and right-wrist cameras.

The language model sees compact detection/quality records and frame
references, never arbitrary file paths or raw tensor outputs. Model discovery,
camera acquisition and artifact storage stay below the tool boundary.
"""

from __future__ import annotations

import functools
import json
import math
import os
import time
import typing
import uuid
from pathlib import Path

import cv2
import numpy as np

from sdk_wrapper.dev.algorithms.charger_yolo import (
    UltralyticsYoloBackend,
    YoloInstance,
    analyze_charger_top,
    analyze_charger_whole_fallback,
    annotation_overlay,
)
from sdk_wrapper.dev.algorithms.charger_cascade import (
    CascadeChargerSegmentationBackend,
    WholeRoiTopSurfaceBackend,
)
from sdk_wrapper.dev.algorithms.calibration import compare_rgbd_calibration
from sdk_wrapper.dev.algorithms.power_strip_yolo import (
    analyze_power_strip_stereo,
    validate_head_target_3d,
)
from sdk_wrapper.dev.contracts import ErrorCode, RunMode, ToolError, ToolResult, failed, ok
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool

__all__ = ["ChargerYoloTool"]

_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "charger_yolo.json"
_DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "runs" / "charger_yolo"


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    @functools.wraps(method)
    def wrapper(self: "ChargerYoloTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
        try:
            return method(self, *args, **kwargs)
        except ToolError as exc:
            return exc.to_result(self.mode)
        except Exception as exc:  # noqa: BLE001 - tool boundary must not leak SDK exceptions
            return failed(
                ErrorCode.SDK_ERROR,
                f"{type(exc).__name__}: {exc}",
                self.mode,
                data={"tool": method.__name__},
            )

    return wrapper


class ChargerYoloTool:
    """Recognize a charger top surface and validate whether it is actionable."""

    TOOL_NAMES = ("detect_charger_head", "segment_charger_top_wrist")

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        backend: typing.Any = None,
        stereo_tool: typing.Any = None,
        arm: str | None = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
        output_dir: str | os.PathLike[str] = _DEFAULT_OUTPUT,
    ) -> None:
        self.perception = perception
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        if arm is not None:
            arm_name = str(arm)
            if arm_name not in ("left", "right"):
                raise ValueError(f"arm must be 'left' or 'right', got {arm_name!r}")
            self.config["wrist_camera"] = f"{arm_name}_arm"
            self.config["wrist_depth_camera"] = f"{arm_name}_arm_depth"
        cascade = dict(self.config.get("cascade") or {})
        if backend is not None:
            self.backend = backend
        elif cascade.get("enabled", False) and str(
            cascade.get("strategy", "")
        ) == "whole_roi_top_surface":
            self.backend = WholeRoiTopSurfaceBackend(
                roots=self.config["model_search_roots"],
                whole_weights=self.config["weights"],
                top_surface_weights=cascade["top_surface_weights"],
                whole_target_class=self.config.get("target_class", "charger_whole"),
                top_surface_target_class=cascade.get(
                    "top_surface_target_class", "charger"
                ),
                whole_confidence=float(self.config.get("confidence", 0.30)),
                top_surface_confidence=float(
                    cascade.get("top_surface_confidence", 0.05)
                ),
                imgsz=int(self.config.get("imgsz", 640)),
                iou=float(cascade.get("iou", 0.45)),
                device=str(self.config.get("device", "cpu")),
                roi_margin=float(cascade.get("roi_margin", 0.20)),
                proposal_limit=int(cascade.get("proposal_limit", 3)),
                min_bbox_iou=float(cascade.get("min_bbox_iou", 0.15)),
                min_mask_containment=float(
                    cascade.get("min_mask_containment", 0.50)
                ),
            )
        elif cascade.get("enabled", False):
            self.backend = CascadeChargerSegmentationBackend(
                roots=self.config["model_search_roots"],
                weights=self.config["weights"],
                proposal_weights=cascade["proposal_weights"],
                target_class=self.config.get("target_class", "charger"),
                imgsz=int(self.config.get("imgsz", 640)),
                refine_confidence=float(cascade.get("refine_confidence", 0.01)),
                full_accept_confidence=float(
                    cascade.get("full_accept_confidence", 0.20)
                ),
                proposal_confidence=float(
                    cascade.get("proposal_confidence", 0.02)
                ),
                iou=float(cascade.get("iou", 0.45)),
                device=str(self.config.get("device", "cpu")),
                roi_margin=float(cascade.get("roi_margin", 0.25)),
                proposal_limit=int(cascade.get("proposal_limit", 3)),
                refine_limit=int(cascade.get("refine_limit", 1)),
                min_bbox_iou=float(cascade.get("min_bbox_iou", 0.15)),
            )
        else:
            self.backend = UltralyticsYoloBackend(
                roots=self.config["model_search_roots"],
                weights=self.config.get("weights"),
                target_class=self.config.get("target_class", "charger"),
                required_task=self.config.get("required_task", "segment"),
                imgsz=int(self.config.get("imgsz", 640)),
                confidence=float(self.config.get("confidence", 0.30)),
                iou=float(self.config.get("iou", 0.70)),
                device=str(self.config.get("device", "cpu")),
            )
        if stereo_tool is None and self.mode is RunMode.REAL:
            from sdk_wrapper.dev.robot_tools.stereo_depth import StereoDepthTool

            stereo_tool = StereoDepthTool(warmup_s=10.0)
        self.stereo_tool = stereo_tool
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def mode(self) -> RunMode:
        return self.perception.mode

    def _analyze_instance(
        self,
        instance: YoloInstance,
        bgr: np.ndarray,
        depth: np.ndarray | None,
        intrinsics: typing.Any,
        extrinsic: dict[str, typing.Any] | None,
        purpose: str,
    ) -> tuple[dict[str, typing.Any], np.ndarray]:
        quality = (
            self.config.get("head_quality") or {}
            if purpose == "head_handoff"
            else self.config.get("quality") or {}
        )
        default_area = 300 if purpose == "head_handoff" else 600
        default_area_ratio = 0.001 if purpose == "head_handoff" else 0.002
        default_short_edge = 18.0 if purpose == "head_handoff" else 24.0
        default_fill_ratio = 0.25 if purpose == "head_handoff" else 0.30
        transform = None if extrinsic is None else extrinsic.get("transform")
        if instance.stage == "whole_fallback":
            sampling = self.config.get("whole_fallback_sampling") or {}
            analysis, mask = analyze_charger_whole_fallback(
                instance,
                bgr.shape[:2],
                depth_m=depth,
                intrinsics=intrinsics,
                extrinsic=transform,
                min_area_px=int(quality.get("min_area_px", default_area)),
                min_area_ratio=float(
                    quality.get("min_area_ratio", default_area_ratio)
                ),
                min_mask_fill_ratio=float(
                    sampling.get(
                        "min_mask_fill_ratio",
                        quality.get("min_mask_fill_ratio", 0.20),
                    )
                ),
                edge_margin_px=int(quality.get("edge_margin_px", 3)),
                min_depth_valid_ratio=float(
                    sampling.get(
                        "min_depth_valid_ratio",
                        quality.get("min_depth_valid_ratio", 0.50),
                    )
                ),
                max_depth_m=float(
                    sampling.get(
                        "max_depth_m",
                        quality.get("max_depth_m", 1.5),
                    )
                ),
                max_surface_tilt_deg=float(
                    sampling.get(
                        "max_surface_tilt_deg",
                        quality.get("max_surface_tilt_deg", 50.0),
                    )
                ),
                max_local_plane_rms_m=float(
                    sampling.get("max_local_plane_rms_m", 0.02)
                ),
                safe_standoff_m=float(
                    quality.get("safe_standoff_m", 0.12)
                ),
                depth_sample_stride=int(
                    quality.get("depth_sample_stride", 2)
                ),
                sample_attempts=int(sampling.get("sample_attempts", 64)),
                sample_radius_px=int(sampling.get("sample_radius_px", 12)),
                sample_seed=int(sampling.get("sample_seed", 20261108)),
            )
        else:
            analysis, mask = analyze_charger_top(
                instance,
                bgr.shape[:2],
                depth_m=depth,
                intrinsics=intrinsics,
                extrinsic=transform,
                min_area_px=int(quality.get("min_area_px", default_area)),
                min_area_ratio=float(
                    quality.get("min_area_ratio", default_area_ratio)
                ),
                min_short_edge_px=float(
                    quality.get("min_short_edge_px", default_short_edge)
                ),
                min_mask_fill_ratio=float(
                    quality.get("min_mask_fill_ratio", default_fill_ratio)
                ),
                edge_margin_px=int(quality.get("edge_margin_px", 3)),
                min_depth_valid_ratio=float(
                    quality.get("min_depth_valid_ratio", 0.80)
                ),
                max_depth_m=float(quality.get("max_depth_m", 1.5)),
                min_long_dimension_m=float(
                    quality.get("min_long_dimension_m", 0.0)
                ),
                min_short_dimension_m=float(
                    quality.get("min_short_dimension_m", 0.0)
                ),
                max_surface_tilt_deg=float(
                    quality.get("max_surface_tilt_deg", 180.0)
                ),
                safe_standoff_m=float(quality.get("safe_standoff_m", 0.12)),
                depth_sample_stride=int(quality.get("depth_sample_stride", 2)),
            )
        if extrinsic is not None:
            analysis["extrinsic_timestamp_ns"] = int(extrinsic.get("timestamp_ns") or 0)
        return analysis, mask

    def _recognize(
        self,
        camera_id: str,
        *,
        include_depth: bool,
        require_mask: bool,
        purpose: str,
        note: str,
    ) -> ToolResult:
        try:
            model_info = self.backend.describe()
        except RuntimeError as exc:
            raise ToolError(ErrorCode.NOT_READY, f"charger YOLO backend unavailable: {exc}") from exc
        cascade_predict = getattr(self.backend, "predict_cascade", None)
        capture_attempts = max(
            1,
            int(self.config.get("capture_attempts", 1))
            if purpose == "precision"
            else 1,
        )
        detection_attempts = max(
            1,
            min(
                capture_attempts,
                int(self.config.get("detection_attempts", 3)),
            )
            if purpose == "precision"
            else 1,
        )
        retry_delay_s = max(
            0.0,
            float(self.config.get("capture_retry_delay_s", 0.35)),
        )
        instances: list[YoloInstance] = []
        cascade_report: dict[str, typing.Any] = {}
        attempt_reports: list[dict[str, typing.Any]] = []
        capture_attempt = 0
        detection_failures = 0
        snap = None
        last_capture_error: str | None = None
        for capture_attempt in range(1, capture_attempts + 1):
            try:
                snap = self.perception._grab(
                    camera_id,
                    include_depth=include_depth,
                    note=note,
                )
            except ToolError as exc:
                last_capture_error = exc.message
                attempt_reports.append(
                    {
                        "attempt": capture_attempt,
                        "reason": "capture_error",
                        "error_code": exc.code.value,
                        "message": exc.message,
                    }
                )
                snap = None
                if capture_attempt < capture_attempts:
                    time.sleep(retry_delay_s)
                continue
            if snap.bgr is None:
                detection_failures += 1
                attempt_reports.append(
                    {
                        "attempt": capture_attempt,
                        "capture_id": snap.data.get("capture_id"),
                        "reason": "rgb_unavailable",
                    }
                )
            else:
                if callable(cascade_predict):
                    instances = cascade_predict(snap.bgr, purpose=purpose)
                else:
                    instances = self.backend.predict(snap.bgr)
                cascade_report = dict(
                    getattr(self.backend, "last_report", {}) or {}
                )
                if purpose == "head_handoff":
                    for instance in instances:
                        if (
                            instance.stage == "proposal_only"
                            and instance.mask is None
                        ):
                            x1, y1, x2, y2 = [
                                max(0, int(round(float(value))))
                                for value in instance.bbox_xyxy
                            ]
                            x2 = min(snap.bgr.shape[1], x2)
                            y2 = min(snap.bgr.shape[0], y2)
                            mask = np.zeros(snap.bgr.shape[:2], dtype=bool)
                            mask[y1:y2, x1:x2] = True
                            instance.mask = mask
                if purpose == "precision":
                    instances = [
                        item
                        for item in instances
                        if item.stage in {
                            "refined",
                            "top_surface",
                            "whole_fallback",
                        }
                        and item.mask is not None
                    ]
                elif require_mask:
                    instances = [
                        item for item in instances if item.mask is not None
                    ]
                attempt_reports.append(
                    {
                        "attempt": capture_attempt,
                        "capture_id": snap.data.get("capture_id"),
                        "candidate_count": len(instances),
                        "cascade": cascade_report,
                    }
                )
                if not instances:
                    detection_failures += 1
            if instances:
                break
            if detection_failures >= detection_attempts:
                break
            if capture_attempt < capture_attempts:
                time.sleep(retry_delay_s)
        if snap is None:
            return failed(
                ErrorCode.NOT_READY,
                f"{camera_id} 连续 {capture_attempt} 次没有可用 RGB"
                + (f"：{last_capture_error}" if last_capture_error else ""),
                self.mode,
                data={
                    "camera_id": camera_id,
                    "capture_attempts": capture_attempt,
                    "detection_attempts": detection_attempts,
                    "capture_attempt_reports": attempt_reports,
                    "model": model_info,
                },
            )
        if not instances:
            return failed(
                ErrorCode.NOT_READY,
                f"{camera_id} 中没有通过对应阶段的 charger 候选",
                self.mode,
                data={
                    "camera_id": camera_id,
                    "capture_id": snap.data.get("capture_id"),
                    "model": model_info,
                    "cascade": cascade_report,
                    "capture_attempts": capture_attempt,
                    "detection_attempts": detection_attempts,
                    "capture_attempt_reports": attempt_reports,
                },
            )

        depth_intrinsics = snap.intr
        depth_extrinsic: dict[str, typing.Any] | None = None
        depth_calibration: dict[str, typing.Any] | None = None
        if snap.depth is not None:
            depth_camera_id = (
                str(self.config.get("wrist_depth_camera", "right_arm_depth"))
                if camera_id == str(self.config.get("wrist_camera", "right_arm"))
                else camera_id
            )
            try:
                depth_intrinsics = self.perception.intrinsics(depth_camera_id)
                depth_extrinsic = self.perception.extrinsic(depth_camera_id)
                rgb_extrinsic = self.perception.extrinsic(camera_id)
            except ToolError as exc:
                raise ToolError(
                    ErrorCode.NOT_READY,
                    f"{depth_camera_id} depth calibration unavailable: {exc.message}",
                ) from exc
            if depth_extrinsic is None:
                raise ToolError(
                    ErrorCode.NOT_READY,
                    f"{depth_camera_id} extrinsic unavailable; refusing 2-D-to-base coordinate promotion",
                )
            if rgb_extrinsic is None:
                raise ToolError(
                    ErrorCode.NOT_READY,
                    f"{camera_id} extrinsic unavailable; cannot verify RGB-D calibration",
                )
            depth_calibration = compare_rgbd_calibration(
                snap.intr,
                depth_intrinsics,
                rgb_extrinsic["transform"],
                depth_extrinsic["transform"],
                max_translation_delta_m=float(
                    self.config.get("max_rgb_depth_translation_delta_m", 0.002)
                ),
                max_rotation_delta_deg=float(
                    self.config.get("max_rgb_depth_rotation_delta_deg", 0.5)
                ),
                max_focal_delta_px=float(
                    self.config.get("max_rgb_depth_focal_delta_px", 1.0)
                ),
                max_principal_delta_px=float(
                    self.config.get("max_rgb_depth_principal_delta_px", 1.0)
                ),
            )
        analyzed: list[tuple[dict[str, typing.Any], YoloInstance, np.ndarray]] = []
        for instance in instances:
            if instance.mask is None:
                continue
            analysis, mask = self._analyze_instance(
                instance,
                snap.bgr,
                snap.depth,
                depth_intrinsics,
                depth_extrinsic,
                purpose,
            )
            analyzed.append((analysis, instance, mask))
        if not analyzed:
            return failed(
                ErrorCode.NOT_READY,
                f"{camera_id} 的 YOLO 结果没有可用的 charger 分割掩码",
                self.mode,
                data={
                    "camera_id": camera_id,
                    "capture_id": snap.data.get("capture_id"),
                    "model": model_info,
                },
            )

        # Preference is lexicographic: quality eligibility first, then
        # confidence, then short-edge pixels. Area alone is intentionally not
        # the ranking key because a clipped target can have a huge mask.
        analyzed.sort(
            key=lambda item: (
                bool(item[0].get("eligible_for_precision")),
                {
                    "top_surface": 3,
                    "refined": 2,
                    "whole_fallback": 1,
                }.get(item[1].stage, 0),
                float(item[1].confidence),
                float(item[0]["image_space"]["short_edge_px"]),
            ),
            reverse=True,
        )
        selected, instance, mask = analyzed[0]
        overlay = annotation_overlay(snap.bgr, instance, selected)

        folder = self.output_dir / (
            time.strftime("%Y%m%dT%H%M%S", time.gmtime())
            + "_"
            + uuid.uuid4().hex[:8]
        )
        folder.mkdir(parents=True, exist_ok=False)
        mask_path = folder / "charger_mask_bool.npy"
        overlay_path = folder / "charger_overlay.png"
        report_path = folder / "charger_observation.json"
        np.save(mask_path, mask.astype(bool), allow_pickle=False)
        if not cv2.imwrite(str(overlay_path), overlay):
            raise OSError(f"failed to save {overlay_path}")

        selected = dict(selected)
        selected["detection_stage"] = instance.stage
        selected["proposal_confidence"] = instance.proposal_confidence
        selected["cascade"] = cascade_report
        head_stereo: dict[str, typing.Any] | None = None
        if purpose == "head_handoff":
            if self.stereo_tool is None:
                head_stereo = {
                    "available": False,
                    "measurement_stable": False,
                    "target_3d": {"available": False},
                    "target_safety": {
                        "safe": False,
                        "reasons": ["head_stereo_not_initialized"],
                    },
                }
            else:
                stereo_result = self.stereo_tool.capture_stereo_depth(
                    timeout_s=float(self.config.get("stereo_timeout_s", 8.0))
                )
                if stereo_result.succeeded and stereo_result.data.get("depth_ref"):
                    depth_raw = np.load(
                        stereo_result.data["depth_ref"], allow_pickle=False
                    )
                    stereo_config = self.config.get("stereo") or {}
                    head_extrinsic = self.perception.extrinsic(camera_id)
                    head_stereo = analyze_power_strip_stereo(
                        depth_raw,
                        selected["bbox_xyxy"],
                        snap.bgr.shape[:2],
                        rgb_timestamp_ns=int(snap.data.get("rgb_timestamp_ns") or 0),
                        depth_timestamp_ns=int(stereo_result.data.get("timestamp_ns") or 0),
                        raw_to_m=stereo_config.get("raw_to_m"),
                        rgb_intrinsics=snap.intr,
                        base_from_camera=(
                            None if head_extrinsic is None
                            else head_extrinsic.get("transform")
                        ),
                        min_valid_ratio=float(stereo_config.get("min_valid_ratio", 0.60)),
                        max_relative_spread=float(
                            stereo_config.get("max_relative_spread", 0.35)
                        ),
                        max_timestamp_skew_ms=float(
                            stereo_config.get("max_timestamp_skew_ms", 300.0)
                        ),
                    )
                    safety_config = self.config.get("target_safety") or {}
                    head_stereo["target_safety"] = validate_head_target_3d(
                        head_stereo.get("target_3d"),
                        min_range_m=float(safety_config.get("min_range_m", 0.45)),
                        max_range_m=float(safety_config.get("max_range_m", 4.0)),
                        min_z_m=float(safety_config.get("min_z_m", -0.05)),
                        max_z_m=float(safety_config.get("max_z_m", 1.6)),
                        max_plane_rms_m=float(
                            safety_config.get("max_plane_rms_m", 0.04)
                        ),
                        max_normal_tilt_deg=float(
                            safety_config.get("max_normal_tilt_deg", 35.0)
                        ),
                    )
                    head_stereo["capture_id"] = stereo_result.data.get("capture_id")
                    head_stereo["depth_ref"] = stereo_result.data.get("depth_ref")
                    head_stereo["sensor_name"] = stereo_result.data.get("sensor_name")
                else:
                    head_stereo = {
                        "available": False,
                        "measurement_stable": False,
                        "target_3d": {"available": False},
                        "target_safety": {
                            "safe": False,
                            "reasons": ["head_stereo_capture_failed"],
                        },
                        "reason": stereo_result.message,
                    }
            handoff = dict(selected.get("head_handoff") or {})
            metric_required = bool(
                (self.config.get("head_quality") or {}).get(
                    "require_metric_stereo", True
                )
            )
            if metric_required and not (head_stereo.get("target_safety") or {}).get("safe"):
                handoff["usable_for_wrist_handoff"] = False
                reasons = list(handoff.get("reasons") or [])
                reasons.extend(
                    (head_stereo.get("target_safety") or {}).get("reasons")
                    or ["head_metric_target_unavailable"]
                )
                handoff["reasons"] = sorted(set(reasons))
            handoff["target_3d"] = head_stereo.get("target_3d")
            handoff["target_safety"] = head_stereo.get("target_safety")
            handoff["stereo_measurement"] = {
                key: head_stereo.get(key)
                for key in (
                    "available", "measurement_stable", "reasons",
                    "metric_distance_available", "metric_distance_m",
                    "unit", "relative_spread", "valid_ratio",
                    "timestamp_skew_ms", "capture_id", "depth_ref", "sensor_name",
                )
            }
            selected["head_handoff"] = handoff
            selected["head_stereo"] = head_stereo
        rgb_timestamp_ns = int(snap.data.get("rgb_timestamp_ns") or 0)
        depth_timestamp_ns = int(snap.data.get("depth_timestamp_ns") or 0)
        skew_ms: float | None = None
        if rgb_timestamp_ns and depth_timestamp_ns:
            skew_ms = abs(depth_timestamp_ns - rgb_timestamp_ns) / 1_000_000.0
        alignment_verified = bool(
            depth_calibration is not None
            and depth_calibration.get("calibration_match")
        )
        if snap.depth is not None and not alignment_verified:
            selected["rejection_reasons"].append("depth_rgb_calibration_mismatch")
        if (
            skew_ms is not None
            and skew_ms > float(self.config.get("max_rgb_depth_skew_ms", 150.0))
        ):
            selected["rejection_reasons"].append("rgb_depth_timestamp_skew_too_large")
        selected["eligible_for_precision"] = not selected["rejection_reasons"]
        head_handoff: dict[str, typing.Any] | None = None
        if purpose == "head_handoff":
            stereo_handoff = dict(selected.get("head_handoff") or {})
            head_quality = self.config.get("head_quality") or {}
            head_reasons = list(selected["rejection_reasons"])
            head_reasons = [
                item for item in head_reasons
                if item not in {
                    "depth_unavailable",
                    "surface_3d_intrinsics_or_extrinsics_unavailable",
                    "surface_3d_unavailable",
                }
            ]
            if selected["image_space"]["mask_area_px"] < int(
                head_quality.get("min_area_px", 300)
            ):
                head_reasons.append("head_mask_area_below_minimum")
            if selected["image_space"]["short_edge_px"] < float(
                head_quality.get("min_short_edge_px", 18.0)
            ):
                head_reasons.append("head_short_edge_below_minimum")
            if selected["image_space"]["mask_fill_ratio"] < float(
                head_quality.get("min_mask_fill_ratio", 0.25)
            ):
                head_reasons.append("head_mask_fill_ratio_below_minimum")
            bbox = selected["bbox_xyxy"]
            bbox_center_x = (float(bbox[0]) + float(bbox[2])) / 2.0
            fx = float(snap.intr.fx)
            cx0 = float(snap.intr.cx)
            horizontal_angle_deg = math.degrees(math.atan2(bbox_center_x - cx0, fx))
            head_handoff = {
                "usable_for_wrist_handoff": not head_reasons,
                "reasons": head_reasons,
                "horizontal_angle_deg": round(horizontal_angle_deg, 4),
                "purpose": "select wrist observation; not a precision grasp point",
                "grasp_coordinate": None,
                "detection_stage": instance.stage,
                "proposal_confidence": instance.proposal_confidence,
                "cascade": cascade_report,
            }
            for key in (
                "target_3d",
                "target_safety",
                "stereo_measurement",
            ):
                if key in stereo_handoff:
                    head_handoff[key] = stereo_handoff[key]
            if bool(head_quality.get("require_metric_stereo", True)) and not (
                head_handoff.get("target_safety") or {}
            ).get("safe"):
                head_handoff["usable_for_wrist_handoff"] = False
                head_handoff["reasons"] = sorted(set(
                    list(head_handoff.get("reasons") or [])
                    + list(
                        (head_handoff.get("target_safety") or {}).get("reasons")
                        or ["head_metric_target_unavailable"]
                    )
                ))
            selected["head_handoff"] = head_handoff
            # Head observations can hand off to wrist planning, but absence of
            # D405 depth and calibration always prevents direct precision use.
            selected["eligible_for_precision"] = False
        selected.update(
            {
                "camera_id": camera_id,
                "capture_id": snap.data.get("capture_id"),
                "rgb_ref": snap.data.get("rgb_ref"),
                "source_depth_ref": snap.data.get("depth_ref"),
                "mask_ref": str(mask_path),
                "overlay_ref": str(overlay_path),
                "coordinate_frame": snap.data.get("coordinate_frame"),
                "camera_timestamp_ns": rgb_timestamp_ns,
                "depth_timestamp_ns": depth_timestamp_ns,
                "rgb_depth_skew_ms": None if skew_ms is None else round(skew_ms, 3),
                "depth_alignment_verified": alignment_verified,
                "warnings": list(snap.warnings),
                "depth_rgb_alignment": (
                    "calibration_match"
                    if alignment_verified
                    else snap.data.get("depth_alignment", "unverified")
                ),
                "depth_calibration_check": depth_calibration,
                "model": model_info,
                "capture_attempt": capture_attempt,
                "capture_attempt_reports": attempt_reports,
            }
        )
        report_path.write_text(
            json.dumps(selected, indent=2, ensure_ascii=False, default=str),
            encoding="utf-8",
        )
        selected["observation_ref"] = str(report_path)

        if purpose == "head_handoff":
            if head_handoff and head_handoff["usable_for_wrist_handoff"]:
                message = (
                    "头部检测到完整 charger 上表面，可开始腕部观察位规划；"
                    "该结果不生成抓取坐标"
                )
            else:
                message = (
                    "头部已识别 charger，但还不能切换腕部："
                    + ", ".join((head_handoff or {}).get("reasons", []))
                )
        elif (
            selected["eligible_for_precision"]
            and selected.get("target_mode") == "whole_mask_random_sample"
        ):
            sampled_uv = (
                selected.get("whole_fallback_sampling") or {}
            ).get("sampled_uv")
            message = (
                f"上表面模型未给出可用目标，已从 charger 整体掩码随机采样："
                f"conf={instance.confidence:.3f}, uv={sampled_uv}, "
                f"深度有效={selected['depth']['valid_ratio']:.1%}"
            )
        elif selected["eligible_for_precision"]:
            message = (
                f"charger 上表面可精定位：conf={instance.confidence:.3f}, "
                f"短边={selected['image_space']['short_edge_px']:.1f}px, "
                f"深度有效={selected['depth']['valid_ratio']:.1%}"
            )
        else:
            message = (
                "charger 已识别，但不可进入精抓取："
                + ", ".join(selected["rejection_reasons"])
            )
        return ok(message, self.mode, data=selected)

    @_tool
    def detect_charger_head(self) -> ToolResult:
        """Head-camera charger-top detection for wrist observation handoff."""
        return self._recognize(
            str(self.config.get("head_camera", "head_left")),
            include_depth=False,
            require_mask=True,
            purpose="head_handoff",
            note="charger YOLO head mid-range wrist handoff",
        )

    @_tool
    def segment_charger_top_wrist(self) -> ToolResult:
        """Right-wrist charger top segmentation plus D405 depth validation."""
        return self._recognize(
            str(self.config.get("wrist_camera", "right_arm")),
            include_depth=True,
            require_mask=True,
            purpose="precision",
            note="charger YOLO right-wrist top-surface validation",
        )
