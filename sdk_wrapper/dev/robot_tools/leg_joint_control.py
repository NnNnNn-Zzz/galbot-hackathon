"""Supervised, one-joint-at-a-time control for G1 leg joints 1-3."""

from __future__ import annotations

import math
import threading
from typing import Any, Mapping

from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    ToolResult,
    failed,
    ok,
    rejected,
    unknown,
)

__all__ = ["LegJointControlTool"]

CONTROL_JOINT_NAMES = ("leg_joint1", "leg_joint2", "leg_joint3")
LEG_STATE_JOINT_NAMES = (
    "leg_joint1",
    "leg_joint2",
    "leg_joint3",
    "leg_joint4",
    "leg_joint5",
)
MAX_STEP_RAD = 0.01
MAX_SPEED_RAD_S = 0.03
MOTION_TIMEOUT_S = 8.0
MAX_OTHER_LEG_DRIFT_RAD = 0.01
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
        raise RuntimeError("torso_base_link quaternion has zero norm")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise RuntimeError("torso_base_link orientation is not finite")
    return roll, pitch, yaw


class LegJointControlTool:
    """Move one of G1 leg joints 1-3 per confirmed command.

    Direct movement is coupled to body height and support geometry. The
    keyboard script limits each joint to a small offset from its session
    starting angle; this class enforces only the per-command step cap.
    """

    TOOL_NAMES = ("get_leg_joint_state", "move_leg_joint_step", "restore_leg_joints")

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
        states = list(robot.get_joint_states([], list(LEG_STATE_JOINT_NAMES)))
        if len(states) != len(LEG_STATE_JOINT_NAMES):
            raise RuntimeError(
                f"expected {len(LEG_STATE_JOINT_NAMES)} leg joint states, got {len(states)}"
            )
        positions = {name: float(state.position) for name, state in zip(LEG_STATE_JOINT_NAMES, states)}
        if not all(math.isfinite(value) for value in positions.values()):
            raise RuntimeError("leg joint state contains a non-finite position")
        head_pose, head_ts = robot.get_transform(
            "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500
        )
        torso_pose, torso_ts = robot.get_transform(
            "base_link", "torso_base_link", timestamp_ns=0, timeout_ms=500
        )
        if head_pose is None or len(head_pose) < 3:
            raise RuntimeError("get_transform returned no head_base_link pose")
        if torso_pose is None or len(torso_pose) < 7:
            raise RuntimeError("get_transform returned no torso_base_link pose")
        roll, pitch, yaw = _rpy(torso_pose)
        return {
            "joint_positions_rad": positions,
            "height_m": float(head_pose[2]),
            "torso_rpy_rad": (roll, pitch, yaw),
            "timestamps_ns": [int(getattr(state, "timestamp_ns", 0) or 0) for state in states],
            "head_timestamp_ns": int(head_ts or 0),
            "torso_timestamp_ns": int(torso_ts or 0),
        }

    def get_leg_joint_state(self) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机腿部读取未授权；请通过现场监督流程显式授权",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                snapshot = self._read_snapshot(robot)
            return ok(
                "已读取腿部关节与躯干姿态",
                self.mode,
                {
                    "control_joint_names": list(CONTROL_JOINT_NAMES),
                    "leg_joint_positions_rad": snapshot["joint_positions_rad"],
                    "height_m": snapshot["height_m"],
                    "torso_rpy_rad": list(snapshot["torso_rpy_rad"]),
                    "frame": "base_link <- torso_base_link",
                    "timestamp_ns": snapshot["torso_timestamp_ns"],
                },
            )
        except Exception as exc:
            return failed(
                ErrorCode.NOT_READY,
                f"无法读取腿部关节状态：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def move_leg_joint_step(
        self,
        joint_name: str,
        delta_rad: float,
        *,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Move exactly one requested joint by a signed delta."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机腿部动作未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "每一步都必须由现场监督者确认后才能下发",
                self.mode,
            )
        if joint_name not in CONTROL_JOINT_NAMES:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "只允许单独控制 leg_joint1/2/3",
                self.mode,
                {"allowed_joint_names": list(CONTROL_JOINT_NAMES)},
            )
        if (
            isinstance(delta_rad, bool)
            or not isinstance(delta_rad, (int, float))
            or not math.isfinite(delta_rad)
            or delta_rad == 0
            or abs(delta_rad) > MAX_STEP_RAD
        ):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"delta_rad 必须非零且绝对值不超过 {MAX_STEP_RAD:.3f} rad",
                self.mode,
                {"max_step_rad": MAX_STEP_RAD},
            )

        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                before = self._read_snapshot(robot)
                target = round(before["joint_positions_rad"][joint_name] + float(delta_rad), 6)
                status = robot.set_joint_positions(
                    joint_positions=[target],
                    joint_groups=[],
                    joint_names=[joint_name],
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=MOTION_TIMEOUT_S,
                )
                status_name = _enum_name(status)
                after = self._read_snapshot(robot)
                actual_delta = (
                    after["joint_positions_rad"][joint_name]
                    - before["joint_positions_rad"][joint_name]
                )
                other_drift = max(
                    abs(
                        after["joint_positions_rad"][name]
                        - before["joint_positions_rad"][name]
                    )
                    for name in LEG_STATE_JOINT_NAMES
                    if name != joint_name
                )
                torso_delta = tuple(
                    new - old
                    for new, old in zip(after["torso_rpy_rad"], before["torso_rpy_rad"])
                )
                data = {
                    "joint_name": joint_name,
                    "from_joint_rad": before["joint_positions_rad"][joint_name],
                    "target_joint_rad": target,
                    "actual_joint_rad": after["joint_positions_rad"][joint_name],
                    "actual_delta_rad": actual_delta,
                    "target_error_rad": after["joint_positions_rad"][joint_name] - target,
                    "height_delta_m": after["height_m"] - before["height_m"],
                    "torso_rpy_delta_rad": list(torso_delta),
                    "other_leg_joint_max_drift_rad": other_drift,
                    "sdk_status": status_name,
                    "timestamp_ns": after["torso_timestamp_ns"],
                }
                if status_name != "SUCCESS":
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"SDK set_joint_positions({joint_name}) 返回 {status_name}；"
                        "位置可能已变化，请停步确认",
                        self.mode,
                        data,
                    )
                minimum_progress = min(0.0005, abs(float(delta_rad)) * 0.5)
                if (
                    math.copysign(1.0, actual_delta) != math.copysign(1.0, float(delta_rad))
                    or abs(actual_delta) < minimum_progress
                ):
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "SDK 报告完成，但目标关节未按指定方向产生足够变化；停止后续步进",
                        self.mode,
                        data,
                    )
                if other_drift > MAX_OTHER_LEG_DRIFT_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "检测到其它腿关节明显漂移；停止后续步进，请现场确认",
                        self.mode,
                        data,
                    )
            return ok(f"已单独移动 {joint_name} 一步", self.mode, data)
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"腿部单关节步进失败：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def restore_leg_joints(
        self,
        targets_rad: Mapping[str, float],
        *,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Restore joints 1-3 together to the supplied session baseline."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机腿部恢复未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(ErrorCode.PERMISSION_DENIED, "恢复腿部位置前需要现场确认", self.mode)
        try:
            targets = [float(targets_rad[name]) for name in CONTROL_JOINT_NAMES]
        except (KeyError, TypeError, ValueError) as exc:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"恢复目标必须包含 leg_joint1/2/3：{exc}",
                self.mode,
            )
        if not all(math.isfinite(value) for value in targets):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "恢复目标包含非有限值",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                status = robot.set_joint_positions(
                    joint_positions=targets,
                    joint_groups=[],
                    joint_names=list(CONTROL_JOINT_NAMES),
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=12.0,
                )
                status_name = _enum_name(status)
                after = self._read_snapshot(robot)
                errors = {
                    name: after["joint_positions_rad"][name] - target
                    for name, target in zip(CONTROL_JOINT_NAMES, targets)
                }
                data = {
                    "targets_rad": dict(zip(CONTROL_JOINT_NAMES, targets)),
                    "actual_rad": {
                        name: after["joint_positions_rad"][name]
                        for name in CONTROL_JOINT_NAMES
                    },
                    "errors_rad": errors,
                    "sdk_status": status_name,
                }
                if status_name != "SUCCESS" or max(abs(value) for value in errors.values()) > RESTORE_TOLERANCE_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "腿部恢复未得到明确成功状态，请现场确认",
                        self.mode,
                        data,
                    )
            return ok("腿部三关节已恢复会话起始位置", self.mode, data)
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"腿部关节恢复失败：{type(exc).__name__}: {exc}",
                self.mode,
            )
