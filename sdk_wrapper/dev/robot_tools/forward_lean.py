"""Dedicated G1 forward-lean control using only leg_joint3."""

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

__all__ = ["ForwardLeanTool"]

LEAN_JOINT_NAME = "leg_joint3"
LEG_JOINT_NAMES = (
    "leg_joint1",
    "leg_joint2",
    "leg_joint3",
    "leg_joint4",
    "leg_joint5",
)
MAX_STEP_RAD = 0.01
MAX_SPEED_RAD_S = 0.03
MOTION_TIMEOUT_S = 8.0
MAX_UNCOMMANDED_DRIFT_RAD = 0.01
MIN_FORWARD_X_M = 0.0005
MIN_FORWARD_PITCH_RAD = 0.0005
RESTORE_TOLERANCE_RAD = 0.002
_ROBOT_LOCK = threading.RLock()


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


def _rpy(pose: Any) -> tuple[float, float, float]:
    qx, qy, qz, qw = (float(value) for value in pose[3:7])
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise RuntimeError("head_base_link quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise RuntimeError("head_base_link orientation is not finite")
    return roll, pitch, yaw


class ForwardLeanTool:
    """Command only ``leg_joint3``; positive delta means forward lean.

    ``leg_joint3`` is part of the height-coordinated leg mechanism. Moving it
    therefore also changes support geometry and body height. Each command is
    capped at 0.01 rad and verified through joint feedback, head X motion, and
    head pitch before being reported as succeeded.
    """

    TOOL_NAMES = (
        "get_forward_lean_state",
        "lean_forward_step",
        "lean_backward_step",
        "restore_lean_joint",
    )

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
        from sdk_wrapper.dev import toolhub
        from sdk_wrapper.wrapper import get_robot

        if toolhub._is_mock():
            raise RuntimeError("GALBOT_MOCK=1; refusing to label the mock robot as real")
        if self._sdk is None:
            import galbot_sdk.g1 as gm

            self._sdk = gm
        return get_robot(), self._sdk

    def _read_snapshot(self, robot: Any) -> dict[str, Any]:
        states = list(robot.get_joint_states([], list(LEG_JOINT_NAMES)))
        if len(states) != len(LEG_JOINT_NAMES):
            raise RuntimeError(
                f"expected {len(LEG_JOINT_NAMES)} leg joint states, got {len(states)}"
            )
        positions = {name: float(state.position) for name, state in zip(LEG_JOINT_NAMES, states)}
        if not all(math.isfinite(value) for value in positions.values()):
            raise RuntimeError("leg joint state contains a non-finite position")
        pose, timestamp_ns = robot.get_transform(
            "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500
        )
        if pose is None or len(pose) < 7:
            raise RuntimeError("get_transform returned no head_base_link pose")
        roll, pitch, yaw = _rpy(pose)
        return {
            "joint_positions_rad": positions,
            "joint_rad": positions[LEAN_JOINT_NAME],
            "head_x_m": float(pose[0]),
            "head_rpy_rad": (roll, pitch, yaw),
            "timestamp_ns": int(timestamp_ns or 0),
        }

    def get_forward_lean_state(self) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机前倾读取未授权；请通过现场监督流程显式授权",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                snapshot = self._read_snapshot(robot)
            return ok(
                "已读取前倾关节状态",
                self.mode,
                {
                    "joint_name": LEAN_JOINT_NAME,
                    "joint_rad": snapshot["joint_rad"],
                    "head_x_m": snapshot["head_x_m"],
                    "head_rpy_rad": list(snapshot["head_rpy_rad"]),
                    "leg_joint_positions_rad": snapshot["joint_positions_rad"],
                    "positive_direction": "forward",
                    "frame": "base_link <- head_base_link",
                    "timestamp_ns": snapshot["timestamp_ns"],
                },
            )
        except Exception as exc:
            return failed(
                ErrorCode.NOT_READY,
                f"无法读取前倾关节状态：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def lean_forward_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Increase leg_joint3 by a bounded step to lean forward."""
        return self._step(step_rad, direction=1, operator_confirmed=operator_confirmed)

    def lean_backward_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Decrease leg_joint3 by a bounded step to return/lean backward."""
        return self._step(step_rad, direction=-1, operator_confirmed=operator_confirmed)

    def _step(self, step_rad: float, *, direction: int, operator_confirmed: bool) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机前倾动作未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "每一步都必须由现场监督者确认后才能下发",
                self.mode,
            )
        if (
            isinstance(step_rad, bool)
            or not isinstance(step_rad, (int, float))
            or not math.isfinite(step_rad)
            or not 0 < step_rad <= MAX_STEP_RAD
        ):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"step_rad 必须大于 0 且不超过 {MAX_STEP_RAD:.3f} rad",
                self.mode,
                {"max_step_rad": MAX_STEP_RAD},
            )

        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                before = self._read_snapshot(robot)
                target = round(before["joint_rad"] + direction * float(step_rad), 6)
                status = robot.set_joint_positions(
                    joint_positions=[target],
                    joint_groups=[],
                    joint_names=[LEAN_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=MOTION_TIMEOUT_S,
                )
                status_name = _enum_name(status)
                after = self._read_snapshot(robot)
                actual_delta = after["joint_rad"] - before["joint_rad"]
                head_x_delta = after["head_x_m"] - before["head_x_m"]
                pitch_delta = after["head_rpy_rad"][1] - before["head_rpy_rad"][1]
                other_drift = max(
                    abs(after["joint_positions_rad"][name] - before["joint_positions_rad"][name])
                    for name in LEG_JOINT_NAMES
                    if name != LEAN_JOINT_NAME
                )
                data = {
                    "joint_name": LEAN_JOINT_NAME,
                    "from_joint_rad": before["joint_rad"],
                    "target_joint_rad": target,
                    "actual_joint_rad": after["joint_rad"],
                    "actual_delta_rad": actual_delta,
                    "target_error_rad": after["joint_rad"] - target,
                    "head_x_delta_m": head_x_delta,
                    "head_pitch_delta_rad": pitch_delta,
                    "other_leg_joint_max_drift_rad": other_drift,
                    "sdk_status": status_name,
                    "timestamp_ns": after["timestamp_ns"],
                }
                if status_name != "SUCCESS":
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"SDK set_joint_positions({LEAN_JOINT_NAME}) 返回 {status_name}；"
                        "位置可能已变化，请停步确认",
                        self.mode,
                        data,
                    )
                minimum_joint_progress = min(0.001, abs(float(step_rad)) * 0.4)
                if direction * actual_delta < minimum_joint_progress:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "目标关节未产生预期方向的变化；停止后续步进",
                        self.mode,
                        data,
                    )
                if direction * head_x_delta < MIN_FORWARD_X_M:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "关节已移动，但头部未按预期前后移动；停止后续步进",
                        self.mode,
                        data,
                    )
                if direction * pitch_delta < MIN_FORWARD_PITCH_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "关节已移动，但头部俯仰方向与预期不符；停止后续步进",
                        self.mode,
                        data,
                    )
                if other_drift > MAX_UNCOMMANDED_DRIFT_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "检测到未命令腿关节明显漂移；停止后续步进",
                        self.mode,
                        data,
                    )
            action = "前倾" if direction > 0 else "回正"
            return ok(f"已完成{action}步进", self.mode, data)
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"前倾步进失败：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def restore_lean_joint(
        self, target_rad: float, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Restore only leg_joint3 to a session baseline."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机前倾恢复未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(ErrorCode.PERMISSION_DENIED, "恢复前需要现场确认", self.mode)
        if (
            isinstance(target_rad, bool)
            or not isinstance(target_rad, (int, float))
            or not math.isfinite(target_rad)
        ):
            return rejected(ErrorCode.INVALID_ARGUMENT, "恢复目标必须是有限数值", self.mode)
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                status = robot.set_joint_positions(
                    joint_positions=[float(target_rad)],
                    joint_groups=[],
                    joint_names=[LEAN_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=12.0,
                )
                status_name = _enum_name(status)
                after = self._read_snapshot(robot)
                error = after["joint_rad"] - float(target_rad)
                data = {
                    "joint_name": LEAN_JOINT_NAME,
                    "target_rad": float(target_rad),
                    "actual_rad": after["joint_rad"],
                    "error_rad": error,
                    "sdk_status": status_name,
                }
                if status_name != "SUCCESS" or abs(error) > RESTORE_TOLERANCE_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "前倾关节恢复未得到明确成功状态，请现场确认",
                        self.mode,
                        data,
                    )
            return ok("leg_joint3 已恢复到会话起始角度", self.mode, data)
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"前倾关节恢复失败：{type(exc).__name__}: {exc}",
                self.mode,
            )
