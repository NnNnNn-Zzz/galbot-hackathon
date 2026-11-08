"""robot_tools/motion.py —— 受控动作原语（预设动作 / 夹爪 / 推）。

设计约束（协议 §3、§5）：
  * 只暴露**末端笛卡尔**级别的动作，不接受关节角、自由速度、力矩；
  * 单次末端平移有硬上限（默认 5 cm），要移动更远必须多次调用 + 每步复测；
  * 所有动作先过 ``RobotSafety.authorize()``，真机模式下不显式授权一律拒绝。

为什么推要"小步 + 复测"而不是一次推 10 cm：
    开环推 10 cm 的落点取决于摩擦、杯重、桌面材质，不可控；
    2.5 cm 一小步、每步用视觉复测 d，才是可控的。
"""

from __future__ import annotations

import logging
import typing

from sdk_wrapper.dev.adapters.base import RobotAdapter
from sdk_wrapper.dev.contracts import ErrorCode, RunMode, ToolResult, failed, rejected
from sdk_wrapper.dev.robot_tools.safety import RobotSafety

LOG = logging.getLogger(__name__)

#: 单次末端平移上限（米）。超过就要求分多次，避免一次大位移失控
MAX_STEP_M = 0.05

#: 方向名 → base_link 单位向量
DIRECTION_VECTORS: dict[str, tuple[float, float, float]] = {
    "x+": (1.0, 0.0, 0.0),   # 远离机器人（前方）
    "x-": (-1.0, 0.0, 0.0),  # 朝机器人
    "y+": (0.0, 1.0, 0.0),   # 机器人左侧
    "y-": (0.0, -1.0, 0.0),  # 机器人右侧
    "z+": (0.0, 0.0, 1.0),   # 向上
    "z-": (0.0, 0.0, -1.0),  # 向下
}


class MotionTool:
    """Expose registered gestures and safe Cartesian primitives."""

    TOOL_NAMES = ("execute_gesture", "get_end_effector_pose", "set_gripper", "push_once")

    def __init__(
        self,
        adapter: RobotAdapter,
        *,
        safety: RobotSafety | None = None,
        real_authorized: bool = False,
        max_step_m: float = MAX_STEP_M,
    ) -> None:
        self.adapter = adapter
        self.safety = safety or RobotSafety(adapter, real_authorized=real_authorized)
        if self.safety.adapter is not adapter:
            raise ValueError("safety and motion tool must share the same adapter")
        self.max_step_m = float(max_step_m)
        if self.max_step_m <= 0:
            raise ValueError("max_step_m must be positive")

    @property
    def mode(self) -> RunMode:
        return self.safety.mode

    # ------------------------------------------------------------------ #
    # 预设动作
    # ------------------------------------------------------------------ #
    def execute_gesture(self, gesture_id: str) -> ToolResult:
        """Start a registered gesture such as ``wave_right``."""
        return self.safety.start_gesture(gesture_id)

    # ------------------------------------------------------------------ #
    # 末端位姿 / 夹爪 / 推
    # ------------------------------------------------------------------ #
    def get_end_effector_pose(self, chain: str = "left_arm") -> ToolResult:
        """读末端在 base_link 下的位姿（米、四元数）。"""
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        return self.adapter.get_end_effector_pose(chain)

    def get_chain_frame_pose(
        self,
        chain: str = "right_arm",
        frame_id: str = "Camera",
        reference_frame: str = "base_link",
    ) -> ToolResult:
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        return self.adapter.get_chain_frame_pose(
            chain, frame_id, reference_frame
        )

    def get_robot_state(self) -> ToolResult:
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        return self.adapter.get_robot_state()

    def check_ee_pose_reachable(
        self,
        pose: typing.Sequence[float],
        *,
        chain: str = "right_arm",
        enable_collision_check: bool = True,
        actuate: str = "with_torso",
    ) -> ToolResult:
        """Read-only IK/collision preflight before an absolute arm command."""
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        return self.adapter.check_end_effector_reachable(
            chain,
            pose,
            enable_collision_check=enable_collision_check,
            actuate=actuate,
        )

    def stop_base(self) -> ToolResult:
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        result = self.adapter.stop_base()
        if result.status.value == "unknown":
            self.safety.trip_global_fuse(f"stop_base returned unknown: {result.message}")
        return result

    def move_base(
        self,
        distance_m: float,
        yaw_rad: float = 0.0,
        *,
        timeout_s: float = 30.0,
    ) -> ToolResult:
        """Move base only for supervised visual-servo fallback paths."""
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        try:
            distance = float(distance_m)
            yaw = float(yaw_rad)
        except (TypeError, ValueError):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "distance_m / yaw_rad 必须是数值",
                mode=self.mode,
            )
        if max(abs(distance), abs(yaw)) > 0.25:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "视觉/底盘单步超过 MotionTool 上限",
                mode=self.mode,
                data={"distance_m": distance, "yaw_rad": yaw},
            )
        result = self.adapter.move_base(distance, yaw, timeout_s=timeout_s)
        if result.status.value == "unknown":
            self.safety.trip_global_fuse(f"move_base returned unknown: {result.message}")
        return result

    def set_gripper(
        self,
        end_effector: str = "left_gripper",
        width_m: float = 0.0,
        effort: float = 10.0,
    ) -> ToolResult:
        """设定夹爪开度（米）。推杯子时闭到 0 用夹爪侧面推。"""
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        if isinstance(width_m, bool) or not isinstance(width_m, (int, float)):
            return rejected(ErrorCode.INVALID_ARGUMENT, "width_m 必须是数值", mode=self.mode)
        return self.adapter.set_gripper_width(
            end_effector, float(width_m), effort=float(effort)
        )

    def move_ee_to(
        self,
        pose: typing.Sequence[float],
        *,
        chain: str = "left_arm",
        straight_line: bool = True,
        enable_collision_check: bool = True,
        actuate: str = "with_torso",
    ) -> ToolResult:
        """移动到 base_link 下的绝对末端位姿。

        **不暴露给大模型**（绝对位姿风险高），只给场景编排用来"回位再复测"。
        """
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection
        try:
            target = [float(v) for v in list(pose)]
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "pose 必须是 7 个数值", mode=self.mode)
        if len(target) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"pose 必须是 [x,y,z,qx,qy,qz,qw]，收到 {len(target)} 个",
                mode=self.mode,
            )
        return self.adapter.move_ee_to(
            chain,
            target,
            straight_line=straight_line,
            enable_collision_check=enable_collision_check,
            actuate=actuate,
        )

    def push_once(
        self,
        distance_m: float,
        *,
        direction: str = "x+",
        direction_base: typing.Sequence[float] | None = None,
        chain: str = "left_arm",
        close_gripper: bool = True,
        gripper: str = "left_gripper",
        effort: float = 10.0,
    ) -> ToolResult:
        """沿指定方向**直线**推一小段（默认上限 5 cm）。

        ``direction_base`` 优先（来自 ``inspect_table`` 的 push_direction_base），
        否则用 ``direction`` 名字。推之前先把夹爪闭合，避免张开的手指刮到杯子。
        """
        rejection = self.safety.authorize()
        if rejection is not None:
            return rejection

        try:
            distance = float(distance_m)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "distance_m 必须是数值", mode=self.mode)

        if distance <= 0:
            return rejected(
                ErrorCode.INVALID_ARGUMENT, f"distance_m 必须 > 0，收到 {distance}", mode=self.mode
            )
        if distance > self.max_step_m + 1e-9:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次推 {distance:.3f} m 超过上限 {self.max_step_m:.3f} m；"
                f"请拆成多步、每步复测距离后再推",
                mode=self.mode,
                data={"distance_m": distance, "max_step_m": self.max_step_m},
            )

        if direction_base is not None:
            try:
                vec = tuple(float(v) for v in list(direction_base)[:3])
            except (TypeError, ValueError):
                return rejected(
                    ErrorCode.INVALID_ARGUMENT, "direction_base 必须是 3 个数值", mode=self.mode
                )
            if len(vec) != 3:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT, "direction_base 必须是 3 个数值", mode=self.mode
                )
        else:
            if direction not in DIRECTION_VECTORS:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"未知方向 {direction!r}",
                    mode=self.mode,
                    data={"legal_directions": sorted(DIRECTION_VECTORS)},
                )
            vec = DIRECTION_VECTORS[direction]

        norm = sum(v * v for v in vec) ** 0.5
        if norm < 1e-9:
            return rejected(ErrorCode.INVALID_ARGUMENT, "方向向量长度为 0", mode=self.mode)
        unit = tuple(v / norm for v in vec)
        delta = tuple(distance * v for v in unit)

        # 1) 先闭合夹爪（用侧面推）
        if close_gripper:
            grip = self.adapter.set_gripper_width(gripper, 0.0, effort=float(effort))
            if grip.status.value != "succeeded":
                return failed(
                    grip.error_code or ErrorCode.SDK_ERROR,
                    f"推之前闭合夹爪失败：{grip.message}",
                    self.mode,
                    data={"gripper": gripper},
                )

        # 2) 直线推
        LOG.info("push_once: chain=%s delta=%s straight_line=True", chain, delta)
        result = self.adapter.move_ee_delta(
            chain, delta[0], delta[1], delta[2], straight_line=True
        )
        if result.status.value == "unknown":
            # 运动状态不可知 → 打全局熔断，避免后续动作在未知状态下继续
            self.safety.trip_global_fuse(f"push_once 返回 unknown：{result.message}")
        return result


def push_once(
    distance_m: float,
    *,
    adapter: RobotAdapter,
    safety: RobotSafety | None = None,
    direction: str = "x+",
    direction_base: typing.Sequence[float] | None = None,
    chain: str = "left_arm",
) -> ToolResult:
    """Functional entry point for callers that inject their adapter explicitly."""
    return MotionTool(adapter, safety=safety).push_once(
        distance_m, direction=direction, direction_base=direction_base, chain=chain
    )


def execute_gesture(
    gesture_id: str,
    *,
    adapter: RobotAdapter,
    safety: RobotSafety | None = None,
) -> ToolResult:
    """Functional entry point for callers that inject their adapter explicitly."""
    return MotionTool(adapter, safety=safety).execute_gesture(gesture_id)


__all__ = ["MotionTool", "DIRECTION_VECTORS", "MAX_STEP_M", "push_once", "execute_gesture"]
