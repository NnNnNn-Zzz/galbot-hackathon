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
    validate_head_target_3d,
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

    TOOL_NAMES = ("detect_power_strip_head", "track_power_strip_head")

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
        self.last_track_id: int | None = None

    @property
    def mode(self) -> RunMode:
        return self.perception.mode

    def _observe(self, *, track: bool) -> ToolResult:
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
        if track:
            tracker = getattr(self.backend, "track", None)
            if not callable(tracker):
                raise ToolError(
                    ErrorCode.NOT_SUPPORTED,
                    "power-strip backend does not implement visual tracking",
                )
            instances = tracker(
                snap.bgr,
                persist=self.last_track_id is not None,
            )
        else:
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

        if track and self.last_track_id is not None:
            instances = [
                item for item in instances
                if item.track_id == self.last_track_id
            ]
            if not instances:
                return failed(
                    ErrorCode.NOT_READY,
                    "power_strip visual track lost; base motion must stop",
                    self.mode,
                    data={
                        "camera_id": camera_id,
                        "capture_id": snap.data.get("capture_id"),
                        "lost_track_id": self.last_track_id,
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
        if track:
            if instance.track_id is None:
                return failed(
                    ErrorCode.NOT_READY,
                    "power_strip detection has no tracker ID",
                    self.mode,
                    data={"camera_id": camera_id, "model": model_info},
                )
            self.last_track_id = int(instance.track_id)
        stereo = None
        if self.stereo_tool is not None:
            stereo_result = self.stereo_tool.capture_stereo_depth(
                timeout_s=float(self.config.get("stereo_timeout_s", 8.0))
            )
            if stereo_result.succeeded and stereo_result.data.get("depth_ref"):
                depth_raw = np.load(stereo_result.data["depth_ref"], allow_pickle=False)
                stereo_config = self.config.get("stereo") or {}
                head_extrinsic = self.perception.extrinsic(camera_id)
                stereo = analyze_power_strip_stereo(
                    depth_raw,
                    selected["bbox_xyxy"],
                    snap.bgr.shape[:2],
                    rgb_timestamp_ns=int(snap.data.get("rgb_timestamp_ns") or 0),
                    depth_timestamp_ns=int(stereo_result.data.get("timestamp_ns") or 0),
                    raw_to_m=stereo_config.get("raw_to_m"),
                    rgb_intrinsics=snap.intr,
                    base_from_camera=(
                        None
                        if head_extrinsic is None
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
                stereo["capture_id"] = stereo_result.data.get("capture_id")
                stereo["depth_ref"] = stereo_result.data.get("depth_ref")
                stereo["sensor_name"] = stereo_result.data.get("sensor_name")
                safety_config = self.config.get("target_safety") or {}
                stereo["target_safety"] = validate_head_target_3d(
                    stereo.get("target_3d"),
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
                "tracking_mode": "yolo_track" if track else "single_frame_detect",
                "target_track_id": self.last_track_id if track else None,
                "approach_mode": (
                    "metric_and_track" if stereo and stereo.get("metric_distance_available")
                    else "track_and_direction"
                    if track
                    else "bearing_only"
                ),
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
                f"模式={selected['approach_mode']}，不生成抓取点"
            )
        else:
            message = (
                "power_strip 已识别但不可用于靠近："
                + ", ".join(selected["rejection_reasons"])
            )
        return ok(message, self.mode, data=selected)

    @_tool
    def detect_power_strip_head(self) -> ToolResult:
        """Return one-frame strip bearing and optional stereo stability."""
        return self._observe(track=False)

    @_tool
    def track_power_strip_head(self) -> ToolResult:
        """Track the same strip ID across sequential navigation camera frames."""
        return self._observe(track=True)
