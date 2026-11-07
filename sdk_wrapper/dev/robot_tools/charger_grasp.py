"""Isolated wrist charger pre-grasp pipeline.

This tool composes existing building blocks without duplicating their logic:

``ChargerYoloTool.segment_charger_top_wrist``
    YOLO-seg mask + D405 depth + base_link surface geometry;
``plan_end_effector_grasp_pose``
    charger-top center/axes + standoff -> absolute EndEffector pre-grasp pose;
``MotionTool.move_ee_to``
    optional supervised pose command.

The final grasp (descent/closing/lift) is deliberately outside this tool.
"""

from __future__ import annotations

import functools
import json
import math
import os
import typing
from pathlib import Path

from sdk_wrapper.dev.algorithms.calibration import rotation_error_deg
from sdk_wrapper.dev.algorithms.observation_pose import plan_end_effector_grasp_pose
from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    ToolError,
    ToolResult,
    failed,
    ok,
    rejected,
    unknown,
)
from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool
from sdk_wrapper.dev.robot_tools.motion import MotionTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool

__all__ = ["ChargerGraspTool"]

_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "charger_grasp.json"


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    @functools.wraps(method)
    def wrapper(self: "ChargerGraspTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
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


class ChargerGraspTool:
    """Plan and optionally command a pose above a wrist-observed charger."""

    TOOL_NAMES = ("plan_wrist_charger_pregrasp", "move_to_wrist_charger_pregrasp")

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        charger_tool: ChargerYoloTool | None = None,
        motion_tool: MotionTool | None = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
    ) -> None:
        self.perception = perception
        self.adapter = perception.adapter
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        self.charger_tool = charger_tool or ChargerYoloTool(perception)
        self.motion_tool = motion_tool or MotionTool(
            self.adapter,
            real_authorized=self.mode is RunMode.REAL,
        )

    @property
    def mode(self) -> RunMode:
        return self.perception.mode

    @staticmethod
    def _observation_summary(data: dict[str, typing.Any]) -> dict[str, typing.Any]:
        return {
            key: data.get(key)
            for key in (
                "camera_id",
                "capture_id",
                "confidence",
                "bbox_xyxy",
                "mask_center_uv",
                "image_space",
                "depth",
                "surface",
                "eligible_for_precision",
                "rejection_reasons",
                "depth_alignment_verified",
                "rgb_depth_skew_ms",
                "detection_stage",
                "proposal_confidence",
                "cascade",
                "overlay_ref",
                "observation_ref",
                "model",
            )
        }

    def _call_plan(self) -> ToolResult:
        """Use the public planning method so internal composition keeps one contract."""
        return self.plan_wrist_charger_pregrasp()

    @_tool
    def plan_wrist_charger_pregrasp(self) -> ToolResult:
        """Observe with the wrist camera and return a pre-grasp pose without motion."""
        observation_result = self.charger_tool.segment_charger_top_wrist()
        observation = (
            dict(observation_result.data)
            if observation_result.status.value == "succeeded"
            else None
        )
        if observation is None:
            result = observation_result.to_dict()
            return failed(
                observation_result.error_code or ErrorCode.NOT_READY,
                f"腕部 charger 观测失败：{observation_result.message}",
                self.mode,
                data={"observation": result},
            )

        observation_summary = self._observation_summary(observation)
        if not observation.get("eligible_for_precision"):
            return failed(
                ErrorCode.NOT_READY,
                "腕部已识别 charger，但该观测不满足精定位门槛",
                self.mode,
                data={
                    "observation": observation_summary,
                    "rejection_reasons": list(
                        observation.get("rejection_reasons") or []
                    ),
                    "motion_dispatched": False,
                },
            )

        surface = observation.get("surface") or {}
        if not surface.get("available"):
            return failed(
                ErrorCode.NOT_READY,
                "腕部观测缺少可用的 charger 上表面三维几何",
                self.mode,
                data={
                    "observation": observation_summary,
                    "motion_dispatched": False,
                },
            )

        chain = str(self.config.get("chain", "right_arm"))
        current_result = self.motion_tool.get_end_effector_pose(chain)
        if not current_result.succeeded:
            return failed(
                current_result.error_code or ErrorCode.NOT_READY,
                f"无法读取当前 {chain} 末端位姿：{current_result.message}",
                self.mode,
                data={
                    "observation": observation_summary,
                    "current_end_effector": current_result.to_dict(),
                    "motion_dispatched": False,
                },
            )
        current_pose = list(current_result.data["pose"])
        grasp_plan = plan_end_effector_grasp_pose(
            surface["surface_center_m"],
            surface["surface_normal_base"],
            surface["long_edge_axis_base"],
            standoff_m=float(surface["safe_standoff_m"]),
            current_end_effector_pose_base=current_pose,
            min_standoff_m=float(self.config.get("min_plan_standoff_m", 0.02)),
            max_standoff_m=float(self.config.get("max_plan_standoff_m", 0.30)),
        )
        delta_m = math.sqrt(
            sum(
                (float(target) - float(current)) ** 2
                for target, current in zip(
                    grasp_plan["pre_grasp_pose"][:3],
                    current_pose[:3],
                )
            )
        )
        rotation_delta_deg = rotation_error_deg(
            current_pose,
            grasp_plan["pre_grasp_pose"],
        )
        data = {
            "chain": chain,
            "end_effector": str(
                self.config.get("end_effector", "right_gripper")
            ),
            "observation": observation_summary,
            "grasp_plan": grasp_plan,
            "current_end_effector_pose": current_pose,
            "end_effector_goal": grasp_plan["pre_grasp_pose"],
            "translation_to_goal_m": round(delta_m, 7),
            "rotation_to_goal_deg": round(rotation_delta_deg, 6),
            "motion_dispatched": False,
            "next_action": "move_to_wrist_charger_pregrasp",
        }
        if not grasp_plan.get("safe_to_plan"):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "charger pre-grasp 位姿不在安全规划范围内",
                self.mode,
                data=data,
            )
        if bool(self.config.get("require_ik_reachability", True)):
            reachable = self.motion_tool.check_ee_pose_reachable(
                grasp_plan["pre_grasp_pose"],
                chain=chain,
                enable_collision_check=True,
            )
            data["ik_preflight"] = reachable.to_dict()
            if not reachable.succeeded:
                return rejected(
                    reachable.error_code or ErrorCode.SAFETY_LIMIT,
                    f"腕部视觉位姿未通过只读 IK 预检：{reachable.message}",
                    self.mode,
                    data=data,
                )
        return ok(
            "腕部 charger 坐标已生成 pre-grasp 末端位姿；尚未下发运动",
            self.mode,
            data=data,
        )

    @_tool
    def move_to_wrist_charger_pregrasp(
        self,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Re-observe, then command the right arm to the charger pre-grasp pose."""
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "charger pre-grasp 末端位姿下发必须由现场监督者确认",
                self.mode,
                data={"motion_dispatched": False},
            )
        if bool(self.config.get("require_robot_idle", True)):
            state = self.motion_tool.get_robot_state()
            if not state.succeeded:
                return unknown(
                    state.error_code or ErrorCode.STATE_UNCERTAIN,
                    f"下发前机器人状态不可确认：{state.message}",
                    self.mode,
                    data={"robot_state": state.to_dict(), "motion_dispatched": False},
                )
            if state.data.get("motion_state") != "idle":
                return rejected(
                    ErrorCode.SAFETY_LIMIT,
                    f"下发前机器人不是 idle：{state.data.get('motion_state')}",
                    self.mode,
                    data={"robot_state": state.to_dict(), "motion_dispatched": False},
                )

        plan_result = self._call_plan()
        if not plan_result.succeeded:
            if plan_result.error_code == ErrorCode.SAFETY_LIMIT:
                return plan_result
            return failed(
                plan_result.error_code or ErrorCode.NOT_READY,
                f"运动前重新规划失败：{plan_result.message}",
                self.mode,
                data={"plan_result": plan_result.to_dict(), "motion_dispatched": False},
            )

        plan_data = dict(plan_result.data)
        target = [float(v) for v in plan_data["end_effector_goal"]]
        motion = self.motion_tool.move_ee_to(
            target,
            chain=str(plan_data["chain"]),
            straight_line=bool(self.config.get("straight_line", False)),
        )
        if not motion.succeeded:
            motion.data.update(
                {
                    "plan": plan_data,
                    "motion_dispatched": motion.status.value != "rejected",
                }
            )
            return motion

        measured = self.motion_tool.get_end_effector_pose(str(plan_data["chain"]))
        if not measured.succeeded:
            return unknown(
                measured.error_code or ErrorCode.STATE_UNCERTAIN,
                "末端已移动，但最终位姿无法确认",
                self.mode,
                data={
                    "plan": plan_data,
                    "motion": motion.to_dict(),
                    "measured_end_effector": measured.to_dict(),
                    "motion_dispatched": True,
                },
            )

        actual = list(measured.data["pose"])
        position_error = math.sqrt(
            sum(
                (float(target_value) - float(actual_value)) ** 2
                for target_value, actual_value in zip(target[:3], actual[:3])
            )
        )
        orientation_error = rotation_error_deg(target, actual)
        matched = (
            position_error
            <= float(self.config.get("position_tolerance_m", 0.01))
            and orientation_error
            <= float(self.config.get("rotation_tolerance_deg", 3.0))
        )
        data = {
            "chain": plan_data["chain"],
            "plan": plan_data,
            "motion": motion.to_dict(),
            "target_end_effector_pose": target,
            "measured_end_effector_pose": actual,
            "position_error_m": round(position_error, 7),
            "orientation_error_deg": round(orientation_error, 6),
            "pose_matched": matched,
            "motion_dispatched": True,
            "grasp_completed": False,
        }
        if not matched:
            return failed(
                ErrorCode.NOT_READY,
                "右臂已移动，但未达到腕部视觉生成的 pre-grasp 位姿",
                self.mode,
                data=data,
            )
        return ok(
            "右臂已到达 charger 上方 pre-grasp 位姿；未下降、未闭爪",
            self.mode,
            data=data,
        )


__all__ = ["ChargerGraspTool"]
