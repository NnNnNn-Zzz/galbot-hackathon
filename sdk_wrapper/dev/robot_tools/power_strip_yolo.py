"""Head-camera power-strip detector for coarse approach planning."""

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

from sdk_wrapper.dev.algorithms.charger_yolo import UltralyticsYoloBackend
from sdk_wrapper.dev.algorithms.power_strip_yolo import (
    analyze_power_strip_box,
    analyze_power_strip_stereo,
    draw_power_strip_overlay,
)
from sdk_wrapper.dev.contracts import ErrorCode, RunMode, ToolError, ToolResult, failed, ok
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool

__all__ = ["PowerStripYoloTool"]

_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "power_strip_yolo.json"
_DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "runs" / "power_strip_yolo"


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    @functools.wraps(method)
    def wrapper(self: "PowerStripYoloTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
        try:
            return method(self, *args, **kwargs)
        except ToolError as exc:
            return exc.to_result(self.mode)
        except Exception as exc:  # noqa: BLE001 - tool boundary
            return failed(
                ErrorCode.SDK_ERROR,
                f"{type(exc).__name__}: {exc}",
                self.mode,
                data={"tool": method.__name__},
            )

    return wrapper


class PowerStripYoloTool:
    """Detect a power strip in the head image for direction-only approach."""

    TOOL_NAMES = ("detect_power_strip_head",)

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        backend: typing.Any = None,
        stereo_tool: typing.Any = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
        output_dir: str | os.PathLike[str] = _DEFAULT_OUTPUT,
    ) -> None:
        self.perception = perception
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        self.backend = backend or UltralyticsYoloBackend(
            roots=self.config["model_search_roots"],
            weights=self.config.get("weights"),
            target_class=self.config.get("target_class", "power_strip"),
            required_task=self.config.get("required_task", "detect"),
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

    @_tool
    def detect_power_strip_head(self) -> ToolResult:
        """Return apparent bearing only; never a metric/grasp coordinate."""
        try:
            model_info = self.backend.describe()
        except RuntimeError as exc:
            raise ToolError(ErrorCode.NOT_READY, f"power-strip YOLO backend unavailable: {exc}") from exc

        camera_id = str(self.config.get("camera", "head_left"))
        snap = self.perception._grab(
            camera_id,
            include_depth=False,
            note="power_strip YOLO head coarse detection",
        )
        if snap.bgr is None:
            raise ToolError(ErrorCode.NOT_READY, f"{camera_id} did not provide RGB data")
        instances = self.backend.predict(snap.bgr)
        if not instances:
            return failed(
                ErrorCode.NOT_READY,
                f"{camera_id} 中未检测到 power_strip",
                self.mode,
                data={
                    "camera_id": camera_id,
                    "capture_id": snap.data.get("capture_id"),
                    "model": model_info,
                },
            )

        quality = self.config.get("quality") or {}
        analyzed = []
        for instance in instances:
            analysis = analyze_power_strip_box(
                instance,
                snap.bgr.shape[:2],
                intrinsics=snap.intr,
                min_confidence=float(quality.get("min_confidence", 0.30)),
                min_width_px=float(quality.get("min_width_px", 12.0)),
                min_height_px=float(quality.get("min_height_px", 8.0)),
                edge_margin_px=int(quality.get("edge_margin_px", 3)),
            )
            analyzed.append((analysis, instance))
        analyzed.sort(
            key=lambda item: (
                bool(item[0]["usable_for_direction"]),
                float(item[1].confidence),
                float(item[0]["bbox_area_px"]),
            ),
            reverse=True,
        )
        selected, instance = analyzed[0]
        stereo = None
        if self.stereo_tool is not None:
            stereo_result = self.stereo_tool.capture_stereo_depth(
                timeout_s=float(self.config.get("stereo_timeout_s", 8.0))
            )
            if stereo_result.succeeded and stereo_result.data.get("depth_ref"):
                depth_raw = np.load(stereo_result.data["depth_ref"], allow_pickle=False)
                stereo_config = self.config.get("stereo") or {}
                stereo = analyze_power_strip_stereo(
                    depth_raw,
                    selected["bbox_xyxy"],
                    snap.bgr.shape[:2],
                    rgb_timestamp_ns=int(snap.data.get("rgb_timestamp_ns") or 0),
                    depth_timestamp_ns=int(stereo_result.data.get("timestamp_ns") or 0),
                    raw_to_m=stereo_config.get("raw_to_m"),
                    min_valid_ratio=float(stereo_config.get("min_valid_ratio", 0.60)),
                    max_relative_spread=float(
                        stereo_config.get("max_relative_spread", 0.35)
                    ),
                    max_timestamp_skew_ms=float(
                        stereo_config.get("max_timestamp_skew_ms", 300.0)
                    ),
                )
                stereo["capture_id"] = stereo_result.data.get("capture_id")
                stereo["depth_ref"] = stereo_result.data.get("depth_ref")
                stereo["sensor_name"] = stereo_result.data.get("sensor_name")
            else:
                stereo = {
                    "available": False,
                    "measurement_stable": False,
                    "reason": stereo_result.message,
                }
        overlay = draw_power_strip_overlay(snap.bgr, instance, selected)

        folder = self.output_dir / (
            time.strftime("%Y%m%dT%H%M%S", time.gmtime())
            + "_"
            + uuid.uuid4().hex[:8]
        )
        folder.mkdir(parents=True, exist_ok=False)
        overlay_path = folder / "power_strip_overlay.png"
        if not cv2.imwrite(str(overlay_path), overlay):
            raise OSError(f"failed to save {overlay_path}")

        selected = dict(selected)
        selected.update(
            {
                "camera_id": camera_id,
                "capture_id": snap.data.get("capture_id"),
                "rgb_ref": snap.data.get("rgb_ref"),
                "overlay_ref": str(overlay_path),
                "camera_timestamp_ns": int(snap.data.get("rgb_timestamp_ns") or 0),
                "warnings": list(snap.warnings),
                "stereo": stereo,
                "approach_mode": "bearing_only",
                "requires_fresh_detection_after_each_segment": True,
                "model": model_info,
            }
        )
        report_path = folder / "power_strip_observation.json"
        report_path.write_text(
            json.dumps(selected, indent=2, ensure_ascii=False, default=str),
            encoding="utf-8",
        )
        selected["observation_ref"] = str(report_path)

        if selected["usable_for_direction"]:
            message = (
                f"power_strip 方向可用：水平 {selected['horizontal_angle_deg']:.2f}°，"
                "仅用于分段靠近，不生成米制抓取点"
            )
        else:
            message = (
                "power_strip 已识别但不可用于靠近："
                + ", ".join(selected["rejection_reasons"])
            )
        return ok(message, self.mode, data=selected)
