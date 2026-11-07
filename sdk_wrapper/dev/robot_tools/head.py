"""Supervised G1 head pitch control using the official SDK API."""

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

__all__ = ["HeadLookTool"]

HEAD_JOINT_NAME = "head_joint2"
HEAD_JOINT_NAMES = ("head_joint1", "head_joint2")
HEAD_MEASUREMENT_FRAME = "head_end_effector_mount_link"
MIN_HEAD_PITCH_JOINT_RAD = -0.25
MAX_HEAD_PITCH_JOINT_RAD = 0.40
MAX_STEP_RAD = 0.02
MAX_SPEED_RAD_S = 0.05
MOTION_TIMEOUT_S = 5.0
MAX_YAW_JOINT_DRIFT_RAD = 0.005
MIN_VISIBLE_PITCH_RAD = 0.002
_ROBOT_LOCK = threading.RLock()


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


def _head_rpy(pose: Any) -> tuple[float, float, float]:
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


class HeadLookTool:
    """Move only ``head_joint2`` in small, explicitly confirmed pitch steps.

    The positive direction is intended as looking down. Each command checks
    the measured head pitch before reporting success. The software envelope
    ``[-0.25, 0.40] rad`` is deliberately conservative and is not a certified
    mechanical limit.
    """

    TOOL_NAMES = ("get_head_pose", "lower_head_step", "raise_head_step")

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
        states = list(robot.get_joint_states([], list(HEAD_JOINT_NAMES)))
        if len(states) != len(HEAD_JOINT_NAMES):
            raise RuntimeError(
                f"expected {len(HEAD_JOINT_NAMES)} head joint states, got {len(states)}"
            )
        positions = [float(state.position) for state in states]
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("head joint state contains a non-finite position")
        pose, timestamp_ns = robot.get_transform(
            "base_link", HEAD_MEASUREMENT_FRAME, timestamp_ns=0, timeout_ms=500
        )
        if pose is None or len(pose) < 7:
            raise RuntimeError(f"get_transform returned no {HEAD_MEASUREMENT_FRAME} pose")
        roll, pitch, yaw = _head_rpy(pose)
        return {
            "joint_positions_rad": dict(zip(HEAD_JOINT_NAMES, positions)),
            "target_rad": positions[1],
            "target_timestamp_ns": int(getattr(states[1], "timestamp_ns", 0) or 0),
            "head_roll_rad": roll,
            "head_pitch_rad": pitch,
            "head_yaw_rad": yaw,
            "transform_timestamp_ns": int(timestamp_ns or 0),
        }

    def get_head_pose(self) -> ToolResult:
        """Read head joint positions and the transformed head orientation."""
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机头部读取未授权；请通过监督流程显式授权",
                self.mode,
            )
        try:
            with _ROBOT_LOCK:
                robot, _ = self._robot()
                snapshot = self._read_snapshot(robot)
            return ok(
                "已读取 G1 头部姿态",
                self.mode,
                {
                    "controlled_joint": HEAD_JOINT_NAME,
                    "head_joint1_rad": snapshot["joint_positions_rad"]["head_joint1"],
                    "head_joint2_rad": snapshot["target_rad"],
                    "head_rpy_rad": [
                        snapshot["head_roll_rad"],
                        snapshot["head_pitch_rad"],
                        snapshot["head_yaw_rad"],
                    ],
                    "frame": f"base_link <- {HEAD_MEASUREMENT_FRAME}",
                    "software_range_rad": [
                        MIN_HEAD_PITCH_JOINT_RAD,
                        MAX_HEAD_PITCH_JOINT_RAD,
                    ],
                    "timestamp_ns": snapshot["target_timestamp_ns"],
                },
            )
        except Exception as exc:
            return failed(
                ErrorCode.NOT_READY,
                f"无法读取头部姿态：{type(exc).__name__}: {exc}",
                self.mode,
            )

    def lower_head_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Increase only ``head_joint2`` by at most 0.02 rad."""
        return self._step(step_rad, direction=1, operator_confirmed=operator_confirmed)

    def raise_head_step(
        self, step_rad: float = MAX_STEP_RAD, *, operator_confirmed: bool = False
    ) -> ToolResult:
        """Decrease only ``head_joint2`` by at most 0.02 rad."""
        return self._step(step_rad, direction=-1, operator_confirmed=operator_confirmed)

    def _step(self, step_rad: float, *, direction: int, operator_confirmed: bool) -> ToolResult:
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(ErrorCode.PERMISSION_DENIED, "真机头部动作未授权", self.mode)
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
                target = round(before["target_rad"] + direction * float(step_rad), 6)
                target = max(
                    MIN_HEAD_PITCH_JOINT_RAD,
                    min(MAX_HEAD_PITCH_JOINT_RAD, target),
                )
                actual_request = abs(target - before["target_rad"])
                if actual_request <= 1e-6:
                    return rejected(
                        ErrorCode.SAFETY_LIMIT,
                        "已到头部俯仰软件边界，停止继续移动",
                        self.mode,
                        {
                            "joint_name": HEAD_JOINT_NAME,
                            "joint_rad": before["target_rad"],
                            "software_range_rad": [
                                MIN_HEAD_PITCH_JOINT_RAD,
                                MAX_HEAD_PITCH_JOINT_RAD,
                            ],
                        },
                    )

                status = robot.set_joint_positions(
                    joint_positions=[target],
                    joint_groups=[],
                    joint_names=[HEAD_JOINT_NAME],
                    is_blocking=True,
                    speed_rad_s=MAX_SPEED_RAD_S,
                    timeout_s=MOTION_TIMEOUT_S,
                )
                status_name = _enum_name(status)
                base_data = {
                    "joint_name": HEAD_JOINT_NAME,
                    "from_joint_rad": before["target_rad"],
                    "target_joint_rad": target,
                    "sdk_status": status_name,
                }
                if status_name != "SUCCESS":
                    if status_name == "TIMEOUT":
                        return unknown(
                            ErrorCode.STATE_UNCERTAIN,
                            "头部命令超时；姿态可能仍在变化，请观察并确认稳定",
                            self.mode,
                            base_data,
                        )
                    return failed(
                        ErrorCode.SDK_ERROR,
                        f"SDK set_joint_positions({HEAD_JOINT_NAME}) 返回 {status_name}",
                        self.mode,
                        base_data,
                    )

                try:
                    after = self._read_snapshot(robot)
                except Exception as exc:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"动作已返回成功，但无法复读头部姿态：{type(exc).__name__}: {exc}",
                        self.mode,
                        base_data,
                    )

                actual_delta = after["target_rad"] - before["target_rad"]
                yaw_joint_drift = abs(
                    after["joint_positions_rad"]["head_joint1"]
                    - before["joint_positions_rad"]["head_joint1"]
                )
                pitch_delta = after["head_pitch_rad"] - before["head_pitch_rad"]
                stale_timestamp = (
                    before["target_timestamp_ns"] > 0
                    and after["target_timestamp_ns"] > 0
                    and after["target_timestamp_ns"] <= before["target_timestamp_ns"]
                )
                data = {
                    **base_data,
                    "actual_joint_rad": after["target_rad"],
                    "actual_delta_rad": actual_delta,
                    "target_error_rad": after["target_rad"] - target,
                    "head_joint1_drift_rad": yaw_joint_drift,
                    "head_pitch_delta_rad": pitch_delta,
                    "before_timestamp_ns": before["target_timestamp_ns"],
                    "after_timestamp_ns": after["target_timestamp_ns"],
                    "software_range_rad": [
                        MIN_HEAD_PITCH_JOINT_RAD,
                        MAX_HEAD_PITCH_JOINT_RAD,
                    ],
                }
                minimum_progress = min(0.005, actual_request * 0.5)
                if stale_timestamp or direction * actual_delta < minimum_progress:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "SDK 报告完成，但未读到更新的、方向正确的头部关节变化；停止后续步进",
                        self.mode,
                        data,
                    )
                if yaw_joint_drift > MAX_YAW_JOINT_DRIFT_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "检测到 head_joint1 未经命令的漂移；停止后续步进",
                        self.mode,
                        data,
                    )
                if direction * pitch_delta < MIN_VISIBLE_PITCH_RAD:
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        "头部关节已移动，但实际俯仰方向与预期不符；停止后续步进",
                        self.mode,
                        data,
                    )

            direction_text = "低头" if direction > 0 else "抬头"
            return ok(
                f"已完成一个{direction_text}步进",
                self.mode,
                data,
            )
        except PermissionError as exc:
            return rejected(ErrorCode.PERMISSION_DENIED, str(exc), self.mode)
        except Exception as exc:
            return failed(
                ErrorCode.SDK_ERROR,
                f"头部俯仰步进失败：{type(exc).__name__}: {exc}",
                self.mode,
            )
