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
MAX_SPEED_DEG_S = 5.0
DEFAULT_SPEED_DEG_S = 3.0
TARGET_TOLERANCE_RAD = 0.02
RESTORE_TOLERANCE_RAD = 0.02
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
    therefore also changes support geometry and body height. This is a
    skill-facing API: the caller chooses the desired lean amount in degrees in
    a single command. There is no artificial per-call angle or cumulative
    session limit; motion speed is capped at 5 degrees/second.
    """

    TOOL_NAMES = (
        "get_forward_lean_state",
        "set_forward_lean",
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

    def set_forward_lean(
        self,
        angle_deg: float,
        *,
        speed_deg_s: float = DEFAULT_SPEED_DEG_S,
    ) -> ToolResult:
        """Move leg_joint3 by a caller-selected relative lean in one command.

        Positive ``angle_deg`` means forward lean; negative means return or
        lean backward. The complete angle change is sent as one target rather
        than split into hidden micro-steps.
        """
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机前倾动作未授权", self.mode)
        if (
            isinstance(angle_deg, bool)
            or not isinstance(angle_deg, (int, float))
            or not math.isfinite(angle_deg)
            or angle_deg == 0
        ):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "angle_deg 必须是非零有限角度",
                self.mode,
            )
        if (
            isinstance(speed_deg_s, bool)
            or not isinstance(speed_deg_s, (int, float))
            or not math.isfinite(speed_deg_s)
            or not 0 < speed_deg_s <= MAX_SPEED_DEG_S
        ):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"speed_deg_s 必须大于 0 且不超过 {MAX_SPEED_DEG_S:.1f} °/s",
                self.mode,
                {"max_speed_deg_s": MAX_SPEED_DEG_S},
            )

        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                before = self._read_snapshot(robot)
                requested_delta_rad = math.radians(float(angle_deg))
                target = round(before["joint_rad"] + requested_delta_rad, 6)
                timeout_s = max(8.0, abs(float(angle_deg)) / float(speed_deg_s) + 8.0)
                status = robot.set_joint_positions(
                    joint_positions=[target],
                    joint_groups=[],
                    joint_names=[LEAN_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=math.radians(float(speed_deg_s)),
                    timeout_s=timeout_s,
                )
                status_name = _enum_name(status)
                after = self._read_snapshot(robot)
                actual_delta = after["joint_rad"] - before["joint_rad"]
                data = {
                    "joint_name": LEAN_JOINT_NAME,
                    "requested_lean_deg": float(angle_deg),
                    "speed_deg_s": float(speed_deg_s),
                    "from_joint_rad": before["joint_rad"],
                    "target_joint_rad": target,
                    "actual_joint_rad": after["joint_rad"],
                    "actual_delta_rad": actual_delta,
                    "actual_lean_deg": math.degrees(actual_delta),
                    "target_error_rad": after["joint_rad"] - target,
                    "head_x_delta_m": after["head_x_m"] - before["head_x_m"],
                    "head_pitch_delta_rad": (
                        after["head_rpy_rad"][1] - before["head_rpy_rad"][1]
                    ),
                    "other_leg_joint_max_drift_rad": max(
                        abs(
                            after["joint_positions_rad"][name]
                            - before["joint_positions_rad"][name]
                        )
                        for name in LEG_JOINT_NAMES
                        if name != LEAN_JOINT_NAME
                    ),
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
                if math.copysign(1.0, actual_delta) != math.copysign(
                    1.0, requested_delta_rad
                ):
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "目标关节变化方向与请求不一致；请现场确认",
                        self.mode,
                        data,
                    )
                if abs(data["target_error_rad"]) > TARGET_TOLERANCE_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "SDK 报告成功，但目标关节误差超过验收阈值；请现场确认",
                        self.mode,
                        data,
                    )
            return ok("已前倾到请求角度" if angle_deg > 0 else "已回正到请求角度", self.mode, data)
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"前倾步进失败：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def restore_lean_joint(
        self,
        target_rad: float,
        *,
        speed_deg_s: float = DEFAULT_SPEED_DEG_S,
    ) -> ToolResult:
        """Restore only leg_joint3 to a session baseline."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机前倾恢复未授权", self.mode)
        if (
            isinstance(target_rad, bool)
            or not isinstance(target_rad, (int, float))
            or not math.isfinite(target_rad)
        ):
            return rejected(ErrorCode.INVALID_ARGUMENT, "恢复目标必须是有限数值", self.mode)
        if (
            isinstance(speed_deg_s, bool)
            or not isinstance(speed_deg_s, (int, float))
            or not math.isfinite(speed_deg_s)
            or not 0 < speed_deg_s <= MAX_SPEED_DEG_S
        ):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"speed_deg_s 必须大于 0 且不超过 {MAX_SPEED_DEG_S:.1f} °/s",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                before = self._read_snapshot(robot)
                status = robot.set_joint_positions(
                    joint_positions=[float(target_rad)],
                    joint_groups=[],
                    joint_names=[LEAN_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=math.radians(float(speed_deg_s)),
                    timeout_s=max(
                        8.0,
                        math.degrees(abs(before["joint_rad"] - float(target_rad)))
                        / float(speed_deg_s)
                        + 8.0,
                    )
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
