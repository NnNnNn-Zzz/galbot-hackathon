"""Tool-layer charger recognition on head and right-wrist cameras.

The language model sees compact detection/quality records and frame
references, never arbitrary file paths or raw tensor outputs. Model discovery,
camera acquisition and artifact storage stay below the tool boundary.
"""

from __future__ import annotations

import functools
import json
import os
import time
import typing
import uuid
from pathlib import Path

import cv2
import numpy as np

from sdk_wrapper.dev.algorithms.charger_yolo import (
    UltralyticsSegmentationBackend,
    YoloInstance,
    analyze_charger_top,
    annotation_overlay,
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
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
        output_dir: str | os.PathLike[str] = _DEFAULT_OUTPUT,
    ) -> None:
        self.perception = perception
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        self.backend = backend or UltralyticsSegmentationBackend(
            roots=self.config["model_search_roots"],
            weights=self.config.get("weights"),
            target_class=self.config.get("target_class", "charger"),
            required_task=self.config.get("required_task", "segment"),
            imgsz=int(self.config.get("imgsz", 640)),
            confidence=float(self.config.get("confidence", 0.30)),
            iou=float(self.config.get("iou", 0.70)),
            device=str(self.config.get("device", "cpu")),
        )
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
    ) -> tuple[dict[str, typing.Any], np.ndarray]:
        quality = self.config.get("quality") or {}
        analysis, mask = analyze_charger_top(
            instance,
            bgr.shape[:2],
            depth_m=depth,
            intrinsics=intrinsics,
            extrinsic=None if extrinsic is None else extrinsic.get("transform"),
            min_area_px=int(quality.get("min_area_px", 600)),
            min_area_ratio=float(quality.get("min_area_ratio", 0.002)),
            min_short_edge_px=float(quality.get("min_short_edge_px", 24.0)),
            min_mask_fill_ratio=float(quality.get("min_mask_fill_ratio", 0.30)),
            edge_margin_px=int(quality.get("edge_margin_px", 3)),
            min_depth_valid_ratio=float(quality.get("min_depth_valid_ratio", 0.80)),
            max_depth_m=float(quality.get("max_depth_m", 1.5)),
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
        note: str,
    ) -> ToolResult:
        try:
            model_info = self.backend.describe()
        except RuntimeError as exc:
            raise ToolError(ErrorCode.NOT_READY, f"charger YOLO backend unavailable: {exc}") from exc
        snap = self.perception._grab(
            camera_id,
            include_depth=include_depth,
            note=note,
        )
        if snap.bgr is None:
            raise ToolError(
                ErrorCode.NOT_READY,
                f"{camera_id} did not provide RGB data",
            )

        instances = self.backend.predict(snap.bgr)
        if require_mask:
            instances = [item for item in instances if item.mask is not None]
        if not instances:
            return failed(
                ErrorCode.NOT_READY,
                f"{camera_id} 中未检测到 charger 上表面",
                self.mode,
                data={
                    "camera_id": camera_id,
                    "capture_id": snap.data.get("capture_id"),
                    "model": model_info,
                },
            )

        depth_intrinsics = snap.intr
        depth_extrinsic: dict[str, typing.Any] | None = None
        if snap.depth is not None:
            depth_camera_id = (
                str(self.config.get("wrist_depth_camera", "right_arm_depth"))
                if camera_id == str(self.config.get("wrist_camera", "right_arm"))
                else camera_id
            )
            try:
                depth_intrinsics = self.perception.intrinsics(depth_camera_id)
                depth_extrinsic = self.perception.extrinsic(depth_camera_id)
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
        rgb_timestamp_ns = int(snap.data.get("rgb_timestamp_ns") or 0)
        depth_timestamp_ns = int(snap.data.get("depth_timestamp_ns") or 0)
        skew_ms: float | None = None
        if rgb_timestamp_ns and depth_timestamp_ns:
            skew_ms = abs(depth_timestamp_ns - rgb_timestamp_ns) / 1_000_000.0
        alignment_verified = bool(self.config.get("depth_alignment_verified", False))
        if snap.depth is not None and not alignment_verified:
            selected["rejection_reasons"].append("depth_rgb_alignment_unverified")
        if (
            skew_ms is not None
            and skew_ms > float(self.config.get("max_rgb_depth_skew_ms", 150.0))
        ):
            selected["rejection_reasons"].append("rgb_depth_timestamp_skew_too_large")
        selected["eligible_for_precision"] = not selected["rejection_reasons"]
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
                "depth_rgb_alignment": snap.data.get("depth_alignment", "unverified"),
                "model": model_info,
            }
        )
        report_path.write_text(
            json.dumps(selected, indent=2, ensure_ascii=False, default=str),
            encoding="utf-8",
        )
        selected["observation_ref"] = str(report_path)

        if selected["eligible_for_precision"]:
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
        """Head-camera charger box/segmentation for coarse approach only."""
        return self._recognize(
            str(self.config.get("head_camera", "head_left")),
            include_depth=False,
            require_mask=False,
            note="charger YOLO head coarse detection",
        )

    @_tool
    def segment_charger_top_wrist(self) -> ToolResult:
        """Right-wrist charger top segmentation plus D405 depth validation."""
        return self._recognize(
            str(self.config.get("wrist_camera", "right_arm")),
            include_depth=True,
            require_mask=True,
            note="charger YOLO right-wrist top-surface validation",
        )
