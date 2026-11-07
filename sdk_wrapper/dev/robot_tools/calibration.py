"""Read-only calibration Tool wrappers around official SDK TF data."""

from __future__ import annotations

import functools
import typing

import numpy as np

from sdk_wrapper.dev.algorithms.calibration import compare_rgbd_calibration, rotation_error_deg
from sdk_wrapper.dev.contracts import ErrorCode, RunMode, ToolError, ToolResult, failed, ok
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool

__all__ = ["CameraCalibrationTool"]


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    @functools.wraps(method)
    def wrapper(self: "CameraCalibrationTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
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


class CameraCalibrationTool:
    """Expose official camera calibration without duplicating hand-eye math."""

    TOOL_NAMES = (
        "check_right_wrist_rgbd_calibration",
        "check_right_arm_eye_in_hand",
    )

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        rgb_camera: str = "right_arm",
        depth_camera: str = "right_arm_depth",
    ) -> None:
        self.perception = perception
        self.rgb_camera = rgb_camera
        self.depth_camera = depth_camera

    @property
    def mode(self) -> RunMode:
        return self.perception.mode

    @_tool
    def check_right_wrist_rgbd_calibration(self) -> ToolResult:
        """Compare official RGB/depth intrinsics and base_link extrinsics."""
        try:
            rgb_intr = self.perception.intrinsics(self.rgb_camera)
            depth_intr = self.perception.intrinsics(self.depth_camera)
            rgb_ext = self.perception.extrinsic(self.rgb_camera)
            depth_ext = self.perception.extrinsic(self.depth_camera)
        except ToolError as exc:
            raise ToolError(
                ErrorCode.NOT_READY,
                f"RGB-D calibration query failed: {exc.message}",
                data=exc.data,
            ) from exc
        if rgb_ext is None or depth_ext is None:
            raise ToolError(
                ErrorCode.NOT_READY,
                "Official RGB or depth extrinsic is unavailable",
            )

        comparison = compare_rgbd_calibration(
            rgb_intr,
            depth_intr,
            rgb_ext["transform"],
            depth_ext["transform"],
        )
        data = {
            **comparison,
            "rgb_camera": self.rgb_camera,
            "depth_camera": self.depth_camera,
            "rgb_frame_id": rgb_intr.frame_id,
            "depth_frame_id": depth_intr.frame_id,
            "rgb_intrinsics": rgb_intr.to_dict(),
            "depth_intrinsics": depth_intr.to_dict(),
            "base_from_rgb": {
                "transform": rgb_ext["transform"],
                "timestamp_ns": rgb_ext["timestamp_ns"],
            },
            "base_from_depth": {
                "transform": depth_ext["transform"],
                "timestamp_ns": depth_ext["timestamp_ns"],
            },
            "live_alignment_acceptance_required": True,
        }
        if comparison["calibration_match"]:
            return ok(
                "右腕 RGB-D 官方标定参数一致；仍需实时边缘对齐验收",
                self.mode,
                data=data,
            )
        return failed(
            ErrorCode.NOT_READY,
            "右腕 RGB-D 官方标定参数不一致，禁止跨模态套用像素",
            self.mode,
            data=data,
        )

    @_tool
    def check_right_arm_eye_in_hand(self) -> ToolResult:
        """Cross-check motion Camera frame against the camera TF/extrinsic."""
        motion_result = self.perception.adapter.get_chain_frame_pose(
            "right_arm", "Camera", "base_link"
        )
        if not motion_result.succeeded:
            return failed(
                motion_result.error_code or ErrorCode.NOT_READY,
                f"运动学 Camera 帧不可用：{motion_result.message}",
                self.mode,
                data={"motion_result": motion_result.to_dict()},
            )
        extrinsic = self.perception.extrinsic("right_arm")
        if extrinsic is None:
            raise ToolError(ErrorCode.NOT_READY, "right_arm camera extrinsic unavailable")

        pose = [float(v) for v in motion_result.data["pose"]]
        tf = extrinsic["transform"]
        translation_delta = float(
            np.linalg.norm(np.asarray(pose[:3], np.float64) - np.asarray(tf[:3], np.float64))
        )
        rotation_delta = rotation_error_deg(pose, tf)
        matched = translation_delta <= 0.003 and rotation_delta <= 1.0
        data = {
            "matched": matched,
            "motion_camera_pose": pose,
            "base_from_camera_extrinsic": tf,
            "translation_delta_m": round(translation_delta, 8),
            "rotation_delta_deg": round(rotation_delta, 8),
            "max_translation_delta_m": 0.003,
            "max_rotation_delta_deg": 1.0,
            "motion_timestamp_ns": int(motion_result.data.get("timestamp_ns") or 0),
            "extrinsic_timestamp_ns": int(extrinsic.get("timestamp_ns") or 0),
        }
        if matched:
            return ok(
                "右臂 Camera 帧与官方手眼外参一致",
                self.mode,
                data=data,
            )
        return failed(
            ErrorCode.NOT_READY,
            "右臂 Camera 帧与官方手眼外参不一致，禁止用于抓取",
            self.mode,
            data=data,
        )
