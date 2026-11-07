"""Supervised, incremental G1 leg-height control using the official SDK API."""

from __future__ import annotations

import math
import threading
from typing import Any

from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    ToolResult,
    failed,
    ok,
    rejected,
    unknown,
)

__all__ = ["SquatTool"]

MIN_LEG_HEIGHT_M = 0.96
MAX_LEG_HEIGHT_M = 1.54
MAX_STEP_M = 0.02
MOTION_DURATION_S = 4.0
MOTION_TIMEOUT_S = 12.0
_ROBOT_LOCK = threading.RLock()


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


class SquatTool:
    """Move the G1 leg-height mechanism in small, explicitly confirmed steps.

    The SDK defines a supported link-height interval of 0.96–1.54 m. A downward
    step is capped at 2 cm and is never sent without a separate operator
    confirmation. The value is head_base_link Z in base_link, not a measured
    floor clearance or a certified mechanical limit.
    """

    TOOL_NAMES = ("get_leg_height", "lower_leg_height_step", "raise_leg_height_step")

    def __init__(self, *, real_authorized: bool = False, robot: Any = None) -> None:
        if not isinstance(real_authorized, bool):
            raise TypeError("real_authorized must be bool")
        self.mode = RunMode.MOCK if robot is not None else RunMode.REAL
        self.real_authorized = real_authorized
        self._robot_instance = robot
        self._sdk: Any = None
        self._lock = threading.RLock()

    def _robot(self) -> tuple[Any, Any]:
        if self._robot_instance is not None:
            return self._robot_instance, self._sdk
        if not self.real_authorized:
            raise PermissionError("real robot control is not explicitly authorized")
        from sdk_wrapper.wrapper import _mock_mode, _sdk_bootstrap, get_robot

        if _mock_mode:
            raise RuntimeError("GALBOT_MOCK=1; refusing to label the mock robot as real")

        _sdk_bootstrap()
        if self._sdk is None:
            import galbot_sdk.g1 as gm

            self._sdk = gm
        return get_robot(), self._sdk

    def _read_height(self, robot: Any) -> tuple[float, int]:
        pose, timestamp_ns = robot.get_transform(
            "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500
        )
        if pose is None or len(pose) < 3:
            raise RuntimeError("get_transform returned no head_base_link pose")
        height = float(pose[2])
        if not math.isfinite(height) or not MIN_LEG_HEIGHT_M <= height <= MAX_LEG_HEIGHT_M:
            raise RuntimeError(
                f"current SDK leg height {height!r} m is outside supported range "
                f"[{MIN_LEG_HEIGHT_M:.2f}, {MAX_LEG_HEIGHT_M:.2f}] m"
            )
        return height, int(timestamp_ns or 0)

    def get_leg_height(self) -> ToolResult:
        """Read `head_base_link` Z from the latest TF sample."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机读取未授权；请通过现场监督流程显式授权",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                height, timestamp_ns = self._read_height(robot)
            return ok("已读取 G1 腿部高度", self.mode, {
                "height_m": height,
                "frame": "base_link <- head_base_link",
                "timestamp_ns": timestamp_ns,
                "sdk_supported_range_m": [MIN_LEG_HEIGHT_M, MAX_LEG_HEIGHT_M],
            })
        except Exception as exc:
            return failed(ErrorCode.NOT_READY, f"无法读取腿部高度：{type(exc).__name__}: {exc}", self.mode)

    def lower_leg_height_step(
        self, step_m: float = MAX_STEP_M, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Lower by at most 2 cm; requires an explicit human confirmation per step."""
        return self._step(step_m, direction=-1, operator_confirmed=operator_confirmed)

    def raise_leg_height_step(
        self, step_m: float = MAX_STEP_M, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Raise by at most 2 cm; requires an explicit human confirmation per step."""
        return self._step(step_m, direction=1, operator_confirmed=operator_confirmed)

    def _step(self, step_m: float, *, direction: int, operator_confirmed: bool) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机下蹲未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "每一步都必须由现场监督者确认后才能下发",
                self.mode,
            )
        if (
            isinstance(step_m, bool)
            or not isinstance(step_m, (int, float))
            or not math.isfinite(step_m)
            or not 0 < step_m <= MAX_STEP_M
        ):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"step_m 必须大于 0 且不超过 {MAX_STEP_M:.2f} m",
                self.mode,
                {"max_step_m": MAX_STEP_M},
            )

        try:
            with _ROBOT_LOCK:
                robot, sdk = self._robot()
                current, before_timestamp_ns = self._read_height(robot)
                floor = MIN_LEG_HEIGHT_M if direction < 0 else MIN_LEG_HEIGHT_M
                ceiling = MAX_LEG_HEIGHT_M if direction > 0 else MAX_LEG_HEIGHT_M
                target = round(current + direction * float(step_m), 6)
                target = max(floor, min(ceiling, target))
                actual_request = abs(target - current)
                if actual_request <= 1e-6:
                    return rejected(
                        ErrorCode.SAFETY_LIMIT,
                        "已到 SDK 支持高度边界，停止继续移动",
                        self.mode,
                        {"height_m": current, "supported_range_m": [MIN_LEG_HEIGHT_M, MAX_LEG_HEIGHT_M]},
                    )

                status = robot.set_leg_height(
                    target,
                    MOTION_DURATION_S,
                    True,
                    MOTION_TIMEOUT_S,
                )
                status_name = _enum_name(status)
                if status_name != "SUCCESS":
                    if status_name == "TIMEOUT":
                        return unknown(
                            ErrorCode.STATE_UNCERTAIN,
                            "腿部高度命令超时；当前位置可能仍在变化，请观察机器人并确认稳定后再继续",
                            self.mode,
                            {"from_height_m": current, "target_height_m": target, "sdk_status": status_name},
                        )
                    return failed(
                        ErrorCode.SDK_ERROR,
                        f"SDK set_leg_height 返回 {status_name}",
                        self.mode,
                        {"from_height_m": current, "target_height_m": target, "sdk_status": status_name},
                    )

                try:
                    actual, timestamp_ns = self._read_height(robot)
                except Exception as exc:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"动作已返回成功，但无法复读高度：{type(exc).__name__}: {exc}",
                        self.mode,
                        {"from_height_m": current, "target_height_m": target, "sdk_status": status_name},
                    )
                actual_delta = actual - current
                minimum_progress_m = min(0.002, actual_request * 0.5)
                if (
                    (before_timestamp_ns > 0 and timestamp_ns > 0 and timestamp_ns <= before_timestamp_ns)
                    or direction * actual_delta < minimum_progress_m
                ):
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "SDK 报告完成，但未读到更新且方向正确的高度变化；停止后续步进，请现场确认",
                        self.mode,
                        {
                            "from_height_m": current,
                            "target_height_m": target,
                            "actual_height_m": actual,
                            "sdk_status": status_name,
                            "before_timestamp_ns": before_timestamp_ns,
                            "timestamp_ns": timestamp_ns,
                        },
                    )

            return ok(
                "已完成一个现场确认的下蹲步进" if direction < 0 else "已完成一个现场确认的抬升步进",
                self.mode,
                {
                    "from_height_m": current,
                    "target_height_m": target,
                    "actual_height_m": actual,
                    "actual_delta_m": actual_delta,
                    "target_error_m": actual - target,
                    "sdk_status": status_name,
                    "timestamp_ns": timestamp_ns,
                    "sdk_supported_range_m": [MIN_LEG_HEIGHT_M, MAX_LEG_HEIGHT_M],
                },
            )
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(ErrorCode.SDK_ERROR, f"腿部高度步进失败：{type(exc).__name__}: {exc}", self.mode)

    def restore_leg_position_controller(self, *, operator_confirmed: bool = False) -> ToolResult:
        """Return the leg group to the SDK example's PVT controller after recovery."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机控制器切换未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(ErrorCode.PERMISSION_DENIED, "恢复控制器前需要现场监督者确认", self.mode)
        try:
            with _ROBOT_LOCK:
                robot, sdk = self._robot()
                if sdk is None:
                    return rejected(ErrorCode.NOT_SUPPORTED, "注入的 Mock SDK 不支持控制器恢复", self.mode)
                status = robot.switch_controller(sdk.G1ControllerName.LEG_PVT_CTRL)
                status_name = _enum_name(status)
            if status_name != "SUCCESS":
                return failed(
                    ErrorCode.SDK_ERROR,
                    f"switch_controller(LEG_PVT_CTRL) 返回 {status_name}",
                    self.mode,
                    {"sdk_status": status_name},
                )
            return ok("已按官方示例切回 leg_pvt_ctrl", self.mode, {"controller": "leg_pvt_ctrl"})
        except Exception as exc:
            return failed(ErrorCode.SDK_ERROR, f"控制器恢复失败：{type(exc).__name__}: {exc}", self.mode)
