"""Supervised single-joint G1 torso lean using the official SDK API."""

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

__all__ = ["TorsoLeanTool"]

TARGET_JOINT_NAME = "leg_joint5"
LEG_JOINT_NAMES = (
    "leg_joint1",
    "leg_joint2",
    "leg_joint3",
    "leg_joint4",
    "leg_joint5",
)
MAX_STEP_RAD = 0.02
MAX_FORWARD_JOINT_RAD = 0.20
MAX_BACKWARD_JOINT_RAD = -0.20
MAX_SPEED_RAD_S = 0.05
MOTION_TIMEOUT_S = 8.0
MAX_UNCOMMANDED_DRIFT_RAD = 0.01
MIN_HEAD_FORWARD_M = 0.0005
MIN_HEAD_PITCH_RAD = 0.002
SINGLE_JOINT_PITCH_SUPPORTED = False
_ROBOT_LOCK = threading.RLock()


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


class TorsoLeanTool:
    """Move only ``leg_joint4`` in small, explicitly confirmed steps.

    The SDK groups ``leg_joint4`` and ``leg_joint5`` into the torso chain.
    Supervised tests on this G1 found that ``leg_joint5`` also does not produce
    forward pitch: the robot visibly moved laterally, and telemetry showed a
    head roll change while pitch and yaw stayed nearly constant. The two torso
    joints therefore have no single-joint pitch axis. Motion commands are
    disabled; a forward lean requires a coordinated multi-joint trajectory.

    The ±0.20 rad envelope is a conservative software limit for supervised
    experiments, not a certified hardware or balance limit.
    """

    TOOL_NAMES = ("get_torso_lean_state", "lean_forward_step", "lean_backward_step")

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
        positions = [float(state.position) for state in states]
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("leg joint state contains a non-finite position")

        pose, timestamp_ns = robot.get_transform(
            "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500
        )
        if pose is None or len(pose) < 3:
            raise RuntimeError("get_transform returned no head_base_link pose")
        head_x = float(pose[0])
        if not math.isfinite(head_x):
            raise RuntimeError("head_base_link X is not finite")
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

        target_index = LEG_JOINT_NAMES.index(TARGET_JOINT_NAME)
        target_timestamp_ns = int(getattr(states[target_index], "timestamp_ns", 0) or 0)
        return {
            "positions": positions,
            "target_rad": positions[target_index],
            "target_timestamp_ns": target_timestamp_ns,
            "head_x_m": head_x,
            "head_roll_rad": roll,
            "head_pitch_rad": pitch,
            "head_yaw_rad": yaw,
            "head_timestamp_ns": int(timestamp_ns or 0),
        }

    def get_torso_lean_state(self) -> ToolResult:
        """Read the controlled joint and the resulting head position."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机躯干读取未授权；请通过现场监督流程显式授权",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                snapshot = self._read_snapshot(robot)
            return ok(
                "已读取躯干单关节状态",
                self.mode,
                {
                    "joint_name": TARGET_JOINT_NAME,
                    "joint_rad": snapshot["target_rad"],
                    "head_x_m": snapshot["head_x_m"],
                    "head_rpy_rad": [
                        snapshot["head_roll_rad"],
                        snapshot["head_pitch_rad"],
                        snapshot["head_yaw_rad"],
                    ],
                    "leg_joint_positions_rad": dict(
                        zip(LEG_JOINT_NAMES, snapshot["positions"])
                    ),
                    "software_range_rad": [
                        MAX_BACKWARD_JOINT_RAD,
                        MAX_FORWARD_JOINT_RAD,
                    ],
                    "frame": "base_link <- head_base_link",
                    "timestamp_ns": snapshot["target_timestamp_ns"],
                },
            )
        except Exception as exc:
            return failed(
                ErrorCode.NOT_READY,
                f"无法读取躯干单关节状态：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def lean_forward_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Increase only ``leg_joint5`` by at most 0.02 rad."""
        return self._step(step_rad, direction=1, operator_confirmed=operator_confirmed)

    def lean_backward_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Decrease only ``leg_joint5`` by at most 0.02 rad."""
        return self._step(step_rad, direction=-1, operator_confirmed=operator_confirmed)

    def _step(self, step_rad: float, *, direction: int, operator_confirmed: bool) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机躯干动作未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "每一步都必须由现场监督者确认后才能下发",
                self.mode,
            )
        if not SINGLE_JOINT_PITCH_SUPPORTED:
            return rejected(
                ErrorCode.NOT_SUPPORTED,
                "当前 G1 的躯干链没有单关节前倾自由度；"
                "leg_joint4/leg_joint5 均不能安全实现俯仰，单关节动作已禁用",
                self.mode,
                {
                    "observed_axes": {
                        "leg_joint4": "yaw",
                        "leg_joint5": "roll",
                    },
                    "required_redesign": "coordinated multi-joint body pitch",
                },
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
                target = round(before["target_rad"] + direction * float(step_rad), 6)
                target = max(
                    MAX_BACKWARD_JOINT_RAD,
                    min(MAX_FORWARD_JOINT_RAD, target),
                )
                actual_request = abs(target - before["target_rad"])
                if actual_request <= 1e-6:
                    return rejected(
                        ErrorCode.SAFETY_LIMIT,
                        "已到躯干单关节软件边界，停止继续移动",
                        self.mode,
                        {
                            "joint_name": TARGET_JOINT_NAME,
                            "joint_rad": before["target_rad"],
                            "software_range_rad": [
                                MAX_BACKWARD_JOINT_RAD,
                                MAX_FORWARD_JOINT_RAD,
                            ],
                        },
                    )

                status = robot.set_joint_positions(
                    joint_positions=[target],
                    joint_groups=[],
                    joint_names=[TARGET_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=MOTION_TIMEOUT_S,
                )
                status_name = _enum_name(status)
                base_data = {
                    "joint_name": TARGET_JOINT_NAME,
                    "from_joint_rad": before["target_rad"],
                    "target_joint_rad": target,
                    "sdk_status": status_name,
                }
                if status_name != "SUCCESS":
                    if status_name == "TIMEOUT":
                        return unknown(
                            ErrorCode.STATE_UNCERTAIN,
                            "躯干单关节命令超时；姿态可能仍在变化，请观察机器人并确认稳定",
                            self.mode,
                            base_data,
                        )
                    return failed(
                        ErrorCode.SDK_ERROR,
                        f"SDK set_joint_positions({TARGET_JOINT_NAME}) 返回 {status_name}",
                        self.mode,
                        base_data,
                    )

                try:
                    after = self._read_snapshot(robot)
                except Exception as exc:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"动作已返回成功，但无法复读躯干状态：{type(exc).__name__}: {exc}",
                        self.mode,
                        base_data,
                    )

                actual_delta = after["target_rad"] - before["target_rad"]
                minimum_joint_progress = min(0.005, actual_request * 0.5)
                other_drift = max(
                    abs(new - old)
                    for index, (new, old) in enumerate(
                        zip(after["positions"], before["positions"])
                    )
                    if LEG_JOINT_NAMES[index] != TARGET_JOINT_NAME
                )
                head_x_delta = after["head_x_m"] - before["head_x_m"]
                pitch_delta = after["head_pitch_rad"] - before["head_pitch_rad"]
                roll_delta = after["head_roll_rad"] - before["head_roll_rad"]
                yaw_delta = after["head_yaw_rad"] - before["head_yaw_rad"]
                stale_target_timestamp = (
                    before["target_timestamp_ns"] > 0
                    and after["target_timestamp_ns"] > 0
                    and after["target_timestamp_ns"] <= before["target_timestamp_ns"]
                )
                data = {
                    **base_data,
                    "actual_joint_rad": after["target_rad"],
                    "actual_delta_rad": actual_delta,
                    "target_error_rad": after["target_rad"] - target,
                    "other_leg_joint_max_drift_rad": other_drift,
                    "head_x_before_m": before["head_x_m"],
                    "head_x_after_m": after["head_x_m"],
                    "head_x_delta_m": head_x_delta,
                    "head_roll_delta_rad": roll_delta,
                    "head_pitch_delta_rad": pitch_delta,
                    "head_yaw_delta_rad": yaw_delta,
                    "before_timestamp_ns": before["target_timestamp_ns"],
                    "after_timestamp_ns": after["target_timestamp_ns"],
                    "software_range_rad": [
                        MAX_BACKWARD_JOINT_RAD,
                        MAX_FORWARD_JOINT_RAD,
                    ],
                }
                if stale_target_timestamp or direction * actual_delta < minimum_joint_progress:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "SDK 报告完成，但未读到更新的、方向正确的单关节变化；停止后续步进",
                        self.mode,
                        data,
                    )
                if other_drift > MAX_UNCOMMANDED_DRIFT_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "检测到未经命令的腿关节漂移；停止后续步进，请现场确认",
                        self.mode,
                        data,
                    )
                if direction * head_x_delta < MIN_HEAD_FORWARD_M:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "关节已移动，但头部未按预期前后移动；正方向可能不是前倾，请现场确认",
                        self.mode,
                        data,
                    )
                if (
                    abs(pitch_delta) < MIN_HEAD_PITCH_RAD
                    or abs(pitch_delta) < 2 * abs(roll_delta)
                    or abs(pitch_delta) < 2 * abs(yaw_delta)
                ):
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "关节已移动，但头部主要表现为偏航或滚转而非俯仰；停止后续步进",
                        self.mode,
                        data,
                    )

            direction_text = "前倾" if direction > 0 else "回正/后仰"
            return ok(
                f"已完成一个现场确认的躯干{direction_text}步进",
                self.mode,
                data,
            )
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"躯干单关节步进失败：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def restore_leg_position_controller(self, *, operator_confirmed: bool = False) -> ToolResult:
        """Return the leg group to the normal PVT controller after the test."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机控制器切换未授权", self.mode)
        if operator_confirmed is not True:
            return rejected(ErrorCode.PERMISSION_DENIED, "恢复控制器前需要现场监督者确认", self.mode)
        try:
            with _ROBOT_LOCK:
                robot, sdk = self._robot()
                if sdk is None:
                    return rejected(
                        ErrorCode.NOT_SUPPORTED,
                        "注入的 Mock SDK 不支持控制器恢复",
                        self.mode,
                    )
                status = robot.switch_controller(sdk.G1ControllerName.LEG_PVT_CTRL)
                status_name = _enum_name(status)
            if status_name != "SUCCESS":
                return failed(
                    ErrorCode.SDK_ERROR,
                    f"switch_controller(LEG_PVT_CTRL) 返回 {status_name}",
                    self.mode,
                    {"sdk_status": status_name},
                )
            return ok(
                "已按官方控制器名称切回 leg_pvt_ctrl",
                self.mode,
                {"controller": "leg_pvt_ctrl"},
            )
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"控制器恢复失败：{type(exc).__name__}: {exc}",
                self.mode,
            )
