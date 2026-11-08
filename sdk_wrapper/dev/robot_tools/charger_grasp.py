"""Isolated wrist charger pre-grasp pipeline.

This tool composes existing building blocks without duplicating their logic:

``ChargerYoloTool.segment_charger_top_wrist``
    YOLO-seg mask + D405 depth + base_link surface geometry;
``plan_end_effector_grasp_pose``
    charger-top center/axes + standoff -> absolute EndEffector pre-grasp pose;
``MotionTool.move_ee_to``
    optional supervised pose command.

The final sequence is split into two supervised stages: move to the diagonal
observation pose and open/re-observe, then execute bounded descent, close and
lift. A close ``TIMEOUT`` is retained as possible contact evidence, never as a
standalone declaration that the grasp succeeded.
"""

from __future__ import annotations

import functools
import json
import math
import os
import time
import typing
from pathlib import Path

from sdk_wrapper.dev.algorithms.calibration import rotation_error_deg
from sdk_wrapper.dev.algorithms.observation_pose import (
    end_effector_pose_for_camera,
    plan_diagonal_observation_pose,
    plan_grasp_descent_pose,
    plan_wrist_camera_pose,
)
from sdk_wrapper.dev.algorithms.pose_transition import (
    PoseTransitionController,
    rotation_error_deg as pose_rotation_error_deg,
)
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

    TOOL_NAMES = (
        "plan_wrist_observation_from_head",
        "move_wrist_to_observation_from_head",
        "plan_wrist_charger_pregrasp",
        "move_to_wrist_charger_pregrasp",
        "execute_wrist_charger_grasp",
    )

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        charger_tool: ChargerYoloTool | None = None,
        motion_tool: MotionTool | None = None,
        arm: str | None = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
    ) -> None:
        self.perception = perception
        self.adapter = perception.adapter
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        if arm is not None:
            arm_name = str(arm)
            if arm_name not in ("left", "right"):
                raise ValueError(f"arm must be 'left' or 'right', got {arm_name!r}")
            self.config["chain"] = f"{arm_name}_arm"
            self.config["end_effector"] = f"{arm_name}_gripper"
        self.charger_tool = charger_tool or ChargerYoloTool(
            perception,
            arm=arm,
        )
        self.motion_tool = motion_tool or MotionTool(
            self.adapter,
            real_authorized=self.mode is RunMode.REAL,
        )
        self._pending_plan: dict[str, typing.Any] | None = None
        self._pending_plan_at: float | None = None

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
                "target_mode",
                "whole_fallback_sampling",
                "eligible_for_precision",
                "rejection_reasons",
                "depth_alignment_verified",
                "rgb_depth_skew_ms",
                "detection_stage",
                "proposal_confidence",
                "cascade",
                "capture_attempt",
                "capture_attempt_reports",
                "overlay_ref",
                "observation_ref",
                "model",
            )
        }

    @staticmethod
    def _result_with_data(
        result: ToolResult,
        data: dict[str, typing.Any],
    ) -> ToolResult:
        return ToolResult(
            status=result.status,
            mode=result.mode,
            message=result.message,
            task_id=result.task_id,
            data={**dict(result.data), **data},
            error_code=result.error_code,
        )

    def _move_pose_decomposed(
        self,
        target_pose: typing.Sequence[float],
        *,
        chain: str,
    ) -> ToolResult:
        """Translate on the current IK branch, then rotate in place.

        Every command is generated from the latest measured pose. Translation
        preserves orientation, rotation preserves position, and both phases
        have hard per-step limits. Collision monitoring remains enabled.
        """
        target = [float(v) for v in target_pose]
        max_position_step = max(
            0.001,
            float(
                self.config.get(
                    "decomposed_motion_max_translation_m",
                    0.02,
                )
            ),
        )
        max_rotation_step = max(
            0.1,
            float(
                self.config.get(
                    "decomposed_motion_max_rotation_deg",
                    7.5,
                )
            ),
        )
        max_steps = max(
            1,
            int(self.config.get("decomposed_motion_max_steps", 120)),
        )
        settle_s = max(
            0.0,
            float(self.config.get("decomposed_motion_settle_s", 0.1)),
        )
        position_tolerance = max(
            0.0,
            float(self.config.get("position_tolerance_m", 0.01)),
        )
        rotation_tolerance = max(
            0.0,
            float(self.config.get("rotation_tolerance_deg", 3.0)),
        )
        handoff_tolerance = max(
            0.0,
            float(
                self.config.get(
                    "decomposed_motion_position_handoff_m",
                    0.002,
                )
            ),
        )

        steps: list[dict[str, typing.Any]] = []
        translation_steps = 0
        rotation_steps = 0
        current_result = self.motion_tool.get_end_effector_pose(chain)
        if not current_result.succeeded:
            return failed(
                current_result.error_code or ErrorCode.NOT_READY,
                f"分解运动前无法读取 {chain} 位姿：{current_result.message}",
                self.mode,
                data={
                    "current_end_effector": current_result.to_dict(),
                    "motion_dispatched": False,
                },
            )
        current = [float(v) for v in current_result.data["pose"]]
        start = list(current)
        controller = PoseTransitionController(
            target,
            position_tolerance_m=position_tolerance,
            position_handoff_tolerance_m=handoff_tolerance,
            rotation_tolerance_deg=rotation_tolerance,
            max_position_step_m=max_position_step,
            max_rotation_step_deg=max_rotation_step,
        )

        for step_index in range(1, max_steps + 1):
            step = controller.next(current)
            if step.kind == "complete":
                return ok(
                    "末端已按“平移后旋转”安全到达目标位姿",
                    self.mode,
                    data={
                        "motion_dispatched": bool(steps),
                        "pose_transition": {
                            "strategy": "translate_then_rotate",
                            "actuate": "with_chain_only",
                            "collision_check": True,
                            "target_pose": target,
                            "start_pose": start,
                            "final_pose": current,
                            "steps_used": len(steps),
                            "translation_steps": translation_steps,
                            "rotation_steps": rotation_steps,
                            "max_translation_step_m": max_position_step,
                            "max_rotation_step_deg": max_rotation_step,
                            "position_tolerance_m": position_tolerance,
                            "rotation_tolerance_deg": rotation_tolerance,
                            "steps": steps,
                        },
                    },
                )

            before = list(current)
            motion = self.motion_tool.move_ee_to(
                step.target_pose,
                chain=chain,
                straight_line=False,
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            measured = self.motion_tool.get_end_effector_pose(chain)
            record = {
                "index": step_index,
                "requested": step.to_dict(),
                "before_pose": before,
                "motion": motion.to_dict(),
                "measured_end_effector": measured.to_dict(),
            }
            steps.append(record)
            if not motion.succeeded:
                return self._result_with_data(
                    motion,
                    {
                        "motion_dispatched": (
                            motion.status.value != "rejected"
                        ),
                        "pose_transition": {
                            "strategy": "translate_then_rotate",
                            "target_pose": target,
                            "start_pose": start,
                            "failed_step": step_index,
                            "steps": steps,
                        },
                    },
                )
            if not measured.succeeded:
                return unknown(
                    measured.error_code or ErrorCode.STATE_UNCERTAIN,
                    "分解运动已发送，但无法回读实际位姿",
                    self.mode,
                    data={
                        "motion_dispatched": True,
                        "pose_transition": {
                            "target_pose": target,
                            "failed_step": step_index,
                            "steps": steps,
                        },
                    },
                )

            actual = [float(v) for v in measured.data["pose"]]
            actual_translation = math.sqrt(
                sum(
                    (float(after_value) - float(before_value)) ** 2
                    for after_value, before_value in zip(
                        actual[:3],
                        before[:3],
                    )
                )
            )
            actual_rotation = pose_rotation_error_deg(before, actual)
            record["actual_translation_m"] = round(actual_translation, 7)
            record["actual_rotation_deg"] = round(actual_rotation, 6)

            allowed_translation = (
                step.step_translation_m
                + min(
                    max(position_tolerance, handoff_tolerance),
                    0.005,
                )
            )
            allowed_rotation = (
                step.step_rotation_deg
                + min(rotation_tolerance, 2.0)
            )
            if (
                actual_translation > allowed_translation
                or actual_rotation > allowed_rotation
            ):
                return failed(
                    ErrorCode.SAFETY_LIMIT,
                    "单步实际位姿变化超过分支安全上限，已停止继续运动",
                    self.mode,
                    data={
                        "motion_dispatched": True,
                        "pose_transition": {
                            "strategy": "translate_then_rotate",
                            "target_pose": target,
                            "start_pose": start,
                            "failed_step": step_index,
                            "reason": "pose_step_overshoot",
                            "allowed_translation_m": round(
                                allowed_translation, 7
                            ),
                            "allowed_rotation_deg": round(
                                allowed_rotation, 6
                            ),
                            "steps": steps,
                        },
                    },
                )

            if (
                controller.phase == "translate"
                and controller.translation_reference_quaternion is not None
            ):
                translation_reference_pose = [
                    *actual[:3],
                    *controller.translation_reference_quaternion,
                ]
                translation_phase_drift = pose_rotation_error_deg(
                    translation_reference_pose,
                    actual,
                )
                record[
                    "translation_phase_orientation_drift_deg"
                ] = round(translation_phase_drift, 6)
                if translation_phase_drift > min(rotation_tolerance, 2.0):
                    return failed(
                        ErrorCode.SAFETY_LIMIT,
                        "平移阶段累计姿态漂移超限，已停止继续运动",
                        self.mode,
                        data={
                            "motion_dispatched": True,
                            "pose_transition": {
                                "strategy": "translate_then_rotate",
                                "target_pose": target,
                                "start_pose": start,
                                "failed_step": step_index,
                                "reason": (
                                    "translation_phase_orientation_drift"
                                ),
                                "orientation_drift_deg": round(
                                    translation_phase_drift, 6
                                ),
                                "steps": steps,
                            },
                        },
                    )
            if (
                controller.phase == "rotate"
                and controller.rotation_reference_position is not None
            ):
                reference = controller.rotation_reference_position
                rotation_phase_drift = math.sqrt(
                    sum(
                        (float(after) - float(reference_value)) ** 2
                        for after, reference_value in zip(
                            actual[:3],
                            reference,
                        )
                    )
                )
                record[
                    "rotation_phase_position_drift_m"
                ] = round(rotation_phase_drift, 7)
                if rotation_phase_drift > max(position_tolerance, 0.005):
                    return failed(
                        ErrorCode.SAFETY_LIMIT,
                        "旋转阶段位置漂移超限，已停止继续运动",
                        self.mode,
                        data={
                            "motion_dispatched": True,
                            "pose_transition": {
                                "strategy": "translate_then_rotate",
                                "target_pose": target,
                                "start_pose": start,
                                "failed_step": step_index,
                                "reason": "rotation_phase_position_drift",
                                "position_drift_m": round(
                                    rotation_phase_drift, 7
                                ),
                                "steps": steps,
                            },
                        },
                    )
            if actual_translation < 0.0005 and actual_rotation < 0.05:
                return failed(
                    ErrorCode.NOT_READY,
                    "运动返回成功但实际位姿没有前进，已停止避免重复下发",
                    self.mode,
                    data={
                        "motion_dispatched": True,
                        "pose_transition": {
                            "strategy": "translate_then_rotate",
                            "target_pose": target,
                            "start_pose": start,
                            "failed_step": step_index,
                            "reason": "pose_step_no_progress",
                            "steps": steps,
                        },
                    },
                )

            if step.kind == "translate":
                translation_steps += 1
            else:
                rotation_steps += 1
            current = actual
            if settle_s:
                time.sleep(settle_s)

        return failed(
            ErrorCode.NOT_READY,
            f"分解运动超过最大步数 {max_steps}，已停止",
            self.mode,
            data={
                "motion_dispatched": bool(steps),
                "pose_transition": {
                    "strategy": "translate_then_rotate",
                    "target_pose": target,
                    "start_pose": start,
                    "last_pose": current,
                    "steps_used": len(steps),
                    "steps": steps,
                },
            },
        )

    def _call_plan(self) -> ToolResult:
        """Use the public planning method so internal composition keeps one contract."""
        return self.plan_wrist_charger_pregrasp()

    @_tool
    def plan_wrist_observation_from_head(self) -> ToolResult:
        """Use head RGB-D to plan the initial wrist camera observation pose."""
        head_result = self.charger_tool.detect_charger_head()
        head_data = (
            dict(head_result.data)
            if head_result.succeeded
            else None
        )
        if head_data is None:
            return failed(
                head_result.error_code or ErrorCode.NOT_READY,
                f"头部 charger 观测失败：{head_result.message}",
                self.mode,
                data={"head_result": head_result.to_dict(), "motion_dispatched": False},
            )
        handoff = dict(head_data.get("head_handoff") or {})
        target_3d = dict(handoff.get("target_3d") or {})
        target_safety = dict(handoff.get("target_safety") or {})
        if not handoff.get("usable_for_wrist_handoff"):
            return failed(
                ErrorCode.NOT_READY,
                "头部 charger 尚未满足腕部交接条件",
                self.mode,
                data={
                    "head_observation": self._observation_summary(head_data),
                    "head_handoff": handoff,
                    "motion_dispatched": False,
                },
            )
        if not target_3d.get("available") or not target_safety.get("safe"):
            return failed(
                ErrorCode.NOT_READY,
                "头部 charger 的米制 3D 目标或安全检查不可用",
                self.mode,
                data={
                    "head_observation": self._observation_summary(head_data),
                    "target_3d": target_3d,
                    "target_safety": target_safety,
                    "motion_dispatched": False,
                },
            )

        standoff = float(
            self.config.get("head_wrist_camera_standoff_m", 0.30)
        )
        camera_plan = plan_wrist_camera_pose(
            target_3d["center_m"],
            target_3d["surface_normal_base"],
            target_3d["long_axis_base"],
            camera_standoff_m=standoff,
            min_camera_standoff_m=float(
                self.config.get("min_head_wrist_camera_standoff_m", 0.20)
            ),
            max_camera_standoff_m=float(
                self.config.get("max_head_wrist_camera_standoff_m", 0.55)
            ),
        )
        if not camera_plan.get("safe_to_plan"):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "头部规划的腕部相机观察位超出安全范围",
                self.mode,
                data={
                    "camera_plan": camera_plan,
                    "motion_dispatched": False,
                },
            )

        chain = str(self.config.get("chain", "right_arm"))
        current_ee = self.motion_tool.get_chain_frame_pose(
            chain,
            "EndEffector",
            "base_link",
        )
        current_camera = self.motion_tool.get_chain_frame_pose(
            chain,
            "Camera",
            "base_link",
        )
        if not current_ee.succeeded or not current_camera.succeeded:
            return failed(
                ErrorCode.NOT_READY,
                "无法读取当前腕部相机手眼位姿",
                self.mode,
                data={
                    "current_end_effector": current_ee.to_dict(),
                    "current_camera": current_camera.to_dict(),
                    "motion_dispatched": False,
                },
            )
        transferred = end_effector_pose_for_camera(
            camera_plan["camera_pose_base"],
            current_ee.data["pose"],
            current_camera.data["pose"],
        )
        end_effector_goal = transferred["end_effector_pose_base"]
        data = {
            "chain": chain,
            "head_observation": self._observation_summary(head_data),
            "head_handoff": handoff,
            "target_3d": target_3d,
            "target_safety": target_safety,
            "camera_plan": camera_plan,
            "end_effector_goal": end_effector_goal,
            "ee_from_camera": transferred["ee_from_camera"],
            "motion_dispatched": False,
        }
        if bool(self.config.get("require_ik_reachability", True)):
            reachable = self.motion_tool.check_ee_pose_reachable(
                end_effector_goal,
                chain=chain,
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            data["ik_preflight"] = reachable.to_dict()
            if not reachable.succeeded:
                return rejected(
                    reachable.error_code or ErrorCode.SAFETY_LIMIT,
                    "头部生成的腕部观察位未通过只读 IK 预检",
                    self.mode,
                    data=data,
                )
        return ok(
            "头部相机已生成腕部初始观察位；尚未下发运动",
            self.mode,
            data=data,
        )

    @_tool
    def move_wrist_to_observation_from_head(
        self,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Move the wrist to the head-generated observation pose, then verify it."""
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "腕部头部观察位移动必须由现场监督者确认",
                self.mode,
                data={"motion_dispatched": False},
            )
        idle_result = self._settle_robot_idle()
        if not idle_result.succeeded:
            return idle_result
        plan_result = self.plan_wrist_observation_from_head()
        if not plan_result.succeeded:
            return plan_result
        plan_data = dict(plan_result.data)
        chain = str(plan_data["chain"])
        goal = [float(v) for v in plan_data["end_effector_goal"]]
        motion = self._move_pose_decomposed(goal, chain=chain)
        if not motion.succeeded:
            return self._result_with_data(
                motion,
                {
                    "plan": plan_data,
                    "idle_stabilization": idle_result.data,
                    "motion_dispatched": motion.status.value != "rejected",
                },
            )
        measured = self.motion_tool.get_chain_frame_pose(
            chain,
            "Camera",
            "base_link",
        )
        if not measured.succeeded:
            return unknown(
                measured.error_code or ErrorCode.STATE_UNCERTAIN,
                "头部引导移动完成，但腕部相机位姿无法确认",
                self.mode,
                data={
                    "plan": plan_data,
                    "motion": motion.to_dict(),
                    "measured_camera": measured.to_dict(),
                    "motion_dispatched": True,
                },
            )
        desired = [float(v) for v in plan_data["camera_plan"]["camera_pose_base"]]
        actual = [float(v) for v in measured.data["pose"]]
        position_error = math.sqrt(
            sum(
                (desired_value - actual_value) ** 2
                for desired_value, actual_value in zip(desired[:3], actual[:3])
            )
        )
        orientation_error = rotation_error_deg(desired, actual)
        matched = (
            position_error
            <= float(
                self.config.get("head_wrist_camera_position_tolerance_m", 0.01)
            )
            and orientation_error
            <= float(
                self.config.get("head_wrist_camera_rotation_tolerance_deg", 3.0)
            )
        )
        data = {
            "plan": plan_data,
            "idle_stabilization": idle_result.data,
            "motion": motion.to_dict(),
            "measured_camera_pose": actual,
            "position_error_m": round(position_error, 7),
            "orientation_error_deg": round(orientation_error, 6),
            "pose_matched": matched,
            "motion_dispatched": True,
            "next_action": "wrist_plan",
        }
        if not matched:
            return failed(
                ErrorCode.NOT_READY,
                "腕部相机未达到头部生成的观察位",
                self.mode,
                data=data,
            )
        return ok(
            "腕部已到达头部生成的初始观察位；可用腕部相机继续 5 cm 斜向规划",
            self.mode,
            data=data,
        )

    def _settle_robot_idle(self) -> ToolResult:
        """Allow residual chassis motion to decay without bypassing the idle gate."""
        timeout_s = max(0.0, float(self.config.get("idle_wait_timeout_s", 3.0)))
        poll_s = max(0.0, float(self.config.get("idle_poll_interval_s", 0.20)))
        speed_tolerance = max(
            0.0,
            float(self.config.get("idle_speed_tolerance_mps", 0.05)),
        )
        deadline = time.monotonic() + timeout_s
        attempts: list[dict[str, typing.Any]] = []
        stop_requested = False

        while True:
            state = self.motion_tool.get_robot_state()
            if not state.succeeded:
                return unknown(
                    state.error_code or ErrorCode.STATE_UNCERTAIN,
                    f"下发前机器人状态不可确认：{state.message}",
                    self.mode,
                    data={
                        "robot_state": state.to_dict(),
                        "idle_stabilization": attempts,
                        "motion_dispatched": False,
                    },
                )
            data = dict(state.data)
            active_tasks = list(data.get("active_task_ids") or [])
            speed_value = data.get("base_speed")
            speed = (
                float(speed_value)
                if isinstance(speed_value, (int, float))
                else None
            )
            moving_by_speed = speed is not None and speed > speed_tolerance
            attempt = {
                "motion_state": data.get("motion_state"),
                "base_speed": speed,
                "active_task_ids": active_tasks,
            }
            attempts.append(attempt)
            if not active_tasks and not moving_by_speed:
                return ok(
                    "下发前机器人已稳定",
                    self.mode,
                    data={
                        "attempts": len(attempts),
                        "speed_tolerance_mps": speed_tolerance,
                        "reports": attempts,
                    },
                )

            if (
                not stop_requested
                and not active_tasks
                and bool(self.config.get("stop_base_before_arm", True))
            ):
                stop = self.motion_tool.stop_base()
                attempt["stop_base"] = stop.to_dict()
                stop_requested = True

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return rejected(
                    ErrorCode.SAFETY_LIMIT,
                    "下发前机器人持续未静止",
                    self.mode,
                    data={
                        "idle_stabilization": {
                            "speed_tolerance_mps": speed_tolerance,
                            "reports": attempts,
                        },
                        "motion_dispatched": False,
                    },
                )
            time.sleep(min(poll_s, remaining))

    def _open_gripper_max(self) -> ToolResult:
        return self.motion_tool.set_gripper(
            str(self.config.get("end_effector", "right_gripper")),
            float(self.config.get("gripper_open_width_m", 0.11)),
            effort=float(self.config.get("gripper_open_effort", 10.0)),
        )

    def _descent_plan_from_observation(
        self,
        observation: dict[str, typing.Any],
        current_pose: typing.Sequence[float],
    ) -> dict[str, typing.Any]:
        surface = observation.get("surface") or {}
        if not observation.get("eligible_for_precision") or not surface.get("available"):
            return {
                "safe_to_plan": False,
                "reasons": list(
                    observation.get("rejection_reasons")
                    or ["observation_not_eligible_for_descent"]
                ),
            }
        plan = plan_grasp_descent_pose(
            surface["surface_center_m"],
            surface["long_edge_axis_base"],
            current_pose,
            clearance_m=float(self.config.get("grasp_clearance_m", 0.008)),
            max_descent_m=float(
                self.config.get("max_descent_distance_m", 0.12)
            ),
            tool_approach_offset_m=float(
                self.config.get("tool_approach_offset_m", 0.146)
            ),
        )
        if plan.get("safe_to_plan") and bool(
            self.config.get("require_ik_reachability", True)
        ):
            reachable = self.motion_tool.check_ee_pose_reachable(
                plan["descent_pose"],
                chain=str(self.config.get("chain", "right_arm")),
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            plan["ik_preflight"] = reachable.to_dict()
            if not reachable.succeeded:
                plan["safe_to_plan"] = False
                plan.setdefault("reasons", []).append(
                    "descent_pose_unreachable"
                )
        return plan

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
        grasp_plan = plan_diagonal_observation_pose(
            surface["surface_center_m"],
            surface["surface_normal_base"],
            surface["long_edge_axis_base"],
            observation_distance_m=float(
                self.config.get("diagonal_observation_distance_m", 0.05)
            ),
            tilt_from_normal_deg=float(
                self.config.get("diagonal_observation_tilt_deg", 35.0)
            ),
            current_end_effector_pose_base=current_pose,
            min_observation_distance_m=float(
                self.config.get("min_diagonal_observation_distance_m", 0.03)
            ),
            max_observation_distance_m=float(
                self.config.get("max_diagonal_observation_distance_m", 0.08)
            ),
            tool_approach_offset_m=float(
                self.config.get("tool_approach_offset_m", 0.146)
            ),
        )
        delta_m = math.sqrt(
            sum(
                (float(target) - float(current)) ** 2
                for target, current in zip(
                    grasp_plan["observation_pose"][:3],
                    current_pose[:3],
                )
            )
        )
        rotation_delta_deg = rotation_error_deg(
            current_pose,
            grasp_plan["observation_pose"],
        )
        data = {
            "chain": chain,
            "end_effector": str(
                self.config.get("end_effector", "right_gripper")
            ),
            "observation": observation_summary,
            "grasp_plan": grasp_plan,
            "current_end_effector_pose": current_pose,
            "end_effector_goal": grasp_plan["observation_pose"],
            "translation_to_goal_m": round(delta_m, 7),
            "rotation_to_goal_deg": round(rotation_delta_deg, 6),
            "motion_dispatched": False,
            "next_action": "move_to_wrist_charger_pregrasp",
        }
        if not grasp_plan.get("safe_to_plan"):
            self._pending_plan = None
            self._pending_plan_at = None
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "charger 斜上方观察位不在安全规划范围内",
                self.mode,
                data=data,
            )
        if bool(self.config.get("require_ik_reachability", True)):
            reachable = self.motion_tool.check_ee_pose_reachable(
                grasp_plan["observation_pose"],
                chain=chain,
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            data["ik_preflight"] = reachable.to_dict()
            if not reachable.succeeded:
                self._pending_plan = None
                self._pending_plan_at = None
                return rejected(
                    reachable.error_code or ErrorCode.SAFETY_LIMIT,
                    f"腕部视觉位姿未通过只读 IK 预检：{reachable.message}",
                    self.mode,
                    data=data,
                )
        self._pending_plan = dict(data)
        self._pending_plan_at = time.monotonic()
        return ok(
            (
                "腕部 charger 坐标已生成约 "
                f"{self.config.get('diagonal_observation_distance_m', 0.05):.2f} m "
                "斜上方观察位；尚未下发运动"
            ),
            self.mode,
            data=data,
        )

    @_tool
    def move_to_wrist_charger_pregrasp(
        self,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Move to the diagonal observation pose, open, re-observe and plan descent."""
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "charger pre-grasp 末端位姿下发必须由现场监督者确认",
                self.mode,
                data={"motion_dispatched": False},
            )
        idle_check: dict[str, typing.Any] | None = None
        if bool(self.config.get("require_robot_idle", True)):
            idle_result = self._settle_robot_idle()
            if not idle_result.succeeded:
                return idle_result
            idle_check = dict(idle_result.data)

        max_age_s = float(self.config.get("pending_plan_max_age_s", 15.0))
        if (
            self._pending_plan is not None
            and self._pending_plan_at is not None
            and time.monotonic() - self._pending_plan_at <= max_age_s
        ):
            plan_data = dict(self._pending_plan)
            plan_source = "recent_preflight"
        else:
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
            plan_source = "fresh_replan"
        self._pending_plan = None
        self._pending_plan_at = None
        target = [float(v) for v in plan_data["end_effector_goal"]]
        motion = self._move_pose_decomposed(
            target,
            chain=str(plan_data["chain"]),
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
            "plan_source": plan_source,
            "idle_stabilization": idle_check,
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
                "机械臂已移动，但未达到斜上方观察位",
                self.mode,
                data=data,
            )

        open_result = self._open_gripper_max()
        data["gripper_open"] = open_result.to_dict()
        if not open_result.succeeded:
            return failed(
                open_result.error_code or ErrorCode.SDK_ERROR,
                f"到达斜上方观察位后夹爪未能全开：{open_result.message}",
                self.mode,
                data=data,
            )

        observation_result = self.charger_tool.segment_charger_top_wrist()
        if not observation_result.succeeded:
            data["observation_after_diagonal"] = observation_result.to_dict()
            return failed(
                observation_result.error_code or ErrorCode.NOT_READY,
                f"斜上方观察位二次识别失败：{observation_result.message}",
                self.mode,
                data=data,
            )
        observation_after = dict(observation_result.data)
        data["observation_after_diagonal"] = self._observation_summary(
            observation_after
        )

        descent_plan = self._descent_plan_from_observation(
            observation_after,
            actual,
        )
        data["descent_plan"] = descent_plan
        data["grasp_started"] = False
        if not descent_plan.get("safe_to_plan"):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "斜上方二次观测未生成安全下探位",
                self.mode,
                data=data,
            )
        return ok(
            "机械臂已到达约 5 cm 斜上方观察位，夹爪已全开并完成二次识别；"
            "已生成下探位，尚未下降和闭爪",
            self.mode,
            data=data,
        )

    @_tool
    def execute_wrist_charger_grasp(
        self,
        operator_confirmed: bool = False,
    ) -> ToolResult:
        """Open, re-observe, descend in bounded steps, close and lift."""
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "charger 下探夹取必须由现场监督者确认",
                self.mode,
                data={"motion_dispatched": False, "grasp_started": False},
            )
        idle_result = self._settle_robot_idle()
        if not idle_result.succeeded:
            return idle_result

        chain = str(self.config.get("chain", "right_arm"))
        end_effector = str(self.config.get("end_effector", "right_gripper"))
        open_result = self._open_gripper_max()
        if not open_result.succeeded:
            return failed(
                open_result.error_code or ErrorCode.SDK_ERROR,
                f"下探前夹爪未能全开：{open_result.message}",
                self.mode,
                data={
                    "idle_stabilization": idle_result.data,
                    "gripper_open": open_result.to_dict(),
                    "motion_dispatched": False,
                    "grasp_started": False,
                },
            )

        observation_result = self.charger_tool.segment_charger_top_wrist()
        if not observation_result.succeeded:
            return failed(
                observation_result.error_code or ErrorCode.NOT_READY,
                f"下探前二次识别失败：{observation_result.message}",
                self.mode,
                data={
                    "idle_stabilization": idle_result.data,
                    "gripper_open": open_result.to_dict(),
                    "observation": observation_result.to_dict(),
                    "motion_dispatched": False,
                    "grasp_started": False,
                },
            )
        observation = dict(observation_result.data)
        surface = observation.get("surface") or {}
        if not observation.get("eligible_for_precision") or not surface.get("available"):
            return failed(
                ErrorCode.NOT_READY,
                "下探前二次观测未通过精定位门槛",
                self.mode,
                data={
                    "idle_stabilization": idle_result.data,
                    "gripper_open": open_result.to_dict(),
                    "observation": self._observation_summary(observation),
                    "motion_dispatched": False,
                    "grasp_started": False,
                },
            )

        current_result = self.motion_tool.get_end_effector_pose(chain)
        if not current_result.succeeded:
            return failed(
                current_result.error_code or ErrorCode.NOT_READY,
                f"无法读取当前 {chain} 末端位姿：{current_result.message}",
                self.mode,
                data={
                    "observation": self._observation_summary(observation),
                    "motion_dispatched": False,
                    "grasp_started": False,
                },
            )
        current_pose = list(current_result.data["pose"])

        expected_observation = plan_diagonal_observation_pose(
            surface["surface_center_m"],
            surface["surface_normal_base"],
            surface["long_edge_axis_base"],
            observation_distance_m=float(
                self.config.get("diagonal_observation_distance_m", 0.05)
            ),
            tilt_from_normal_deg=float(
                self.config.get("diagonal_observation_tilt_deg", 35.0)
            ),
            current_end_effector_pose_base=current_pose,
            min_observation_distance_m=float(
                self.config.get("min_diagonal_observation_distance_m", 0.03)
            ),
            max_observation_distance_m=float(
                self.config.get("max_diagonal_observation_distance_m", 0.08)
            ),
            tool_approach_offset_m=float(
                self.config.get("tool_approach_offset_m", 0.146)
            ),
        )
        expected_pose = expected_observation["observation_pose"]
        diagonal_position_error = math.sqrt(
            sum(
                (float(target) - float(current)) ** 2
                for target, current in zip(expected_pose[:3], current_pose[:3])
            )
        )
        diagonal_rotation_error = rotation_error_deg(expected_pose, current_pose)
        diagonal_ready = (
            diagonal_position_error
            <= float(self.config.get("diagonal_pose_position_tolerance_m", 0.02))
            and diagonal_rotation_error
            <= float(self.config.get("diagonal_pose_rotation_tolerance_deg", 8.0))
        )
        base_data = {
            "chain": chain,
            "end_effector": end_effector,
            "idle_stabilization": idle_result.data,
            "gripper_open": open_result.to_dict(),
            "observation": self._observation_summary(observation),
            "expected_diagonal_observation_pose": expected_pose,
            "diagonal_position_error_m": round(diagonal_position_error, 7),
            "diagonal_rotation_error_deg": round(diagonal_rotation_error, 6),
            "diagonal_ready": diagonal_ready,
        }
        if not diagonal_ready:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "当前末端不在二次观测确认的斜上方观察位；拒绝直接下探",
                self.mode,
                data={**base_data, "motion_dispatched": False, "grasp_started": False},
            )

        descent_plan = self._descent_plan_from_observation(
            observation,
            current_pose,
        )
        base_data["descent_plan"] = descent_plan
        if not descent_plan.get("safe_to_plan"):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "二次观测未生成安全下探位",
                self.mode,
                data={**base_data, "motion_dispatched": False, "grasp_started": False},
            )

        target_pose = [float(v) for v in descent_plan["descent_pose"]]
        approach_axis = [
            float(v) for v in descent_plan["approach_axis_base"]
        ]
        lift_distance = max(0.0, float(self.config.get("lift_distance_m", 0.03)))
        lift_pose = list(target_pose)
        lift_pose[:3] = [
            float(value) - approach * lift_distance
            for value, approach in zip(target_pose[:3], approach_axis)
        ]
        base_data["lift_pose"] = lift_pose
        if bool(self.config.get("require_ik_reachability", True)):
            lift_reachable = self.motion_tool.check_ee_pose_reachable(
                lift_pose,
                chain=chain,
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            base_data["lift_ik_preflight"] = lift_reachable.to_dict()
            if not lift_reachable.succeeded:
                return rejected(
                    lift_reachable.error_code or ErrorCode.SAFETY_LIMIT,
                    "夹取后的抬升位姿未通过只读 IK 预检",
                    self.mode,
                    data={
                        **base_data,
                        "motion_dispatched": False,
                        "grasp_started": False,
                    },
                )

        descent_distance = float(descent_plan["descent_distance_m"])
        max_step = max(
            0.001,
            min(
                float(self.config.get("descent_step_m", 0.01)),
                float(self.config.get("max_descent_step_m", 0.02)),
            ),
        )
        step_count = max(1, int(math.ceil(descent_distance / max_step)))
        descent_motions: list[dict[str, typing.Any]] = []
        for step_index in range(1, step_count + 1):
            fraction = step_index / float(step_count)
            waypoint = list(target_pose)
            waypoint[:3] = [
                float(start) + (float(target) - float(start)) * fraction
                for start, target in zip(current_pose[:3], target_pose[:3])
            ]
            motion = self.motion_tool.move_ee_to(
                waypoint,
                chain=chain,
                straight_line=False,
                enable_collision_check=True,
                actuate="with_chain_only",
            )
            descent_motions.append(motion.to_dict())
            if not motion.succeeded:
                return self._result_with_data(
                    motion,
                    {
                        **base_data,
                        "descent_motions": descent_motions,
                        "motion_dispatched": True,
                        "grasp_started": True,
                    },
                )

        short_dimension = float(surface.get("short_dimension_m") or 0.0)
        close_width = min(
            float(self.config.get("grasp_close_max_width_m", 0.025)),
            max(
                float(self.config.get("grasp_close_min_width_m", 0.012)),
                short_dimension
                * float(self.config.get("grasp_close_width_ratio", 0.85)),
            ),
        )
        close_result = self.motion_tool.set_gripper(
            end_effector,
            close_width,
            effort=float(self.config.get("grasp_close_effort", 10.0)),
        )
        control_status = str(
            (close_result.data or {}).get("control_status") or ""
        )
        contact_limited = control_status == "TIMEOUT"
        base_data.update(
            {
                "descent_motions": descent_motions,
                "grasp_close_width_m": round(close_width, 6),
                "gripper_close": close_result.to_dict(),
                "close_contact_limited": contact_limited,
            }
        )
        if not close_result.succeeded and not contact_limited:
            return failed(
                close_result.error_code or ErrorCode.SDK_ERROR,
                f"下探后夹爪闭合失败：{close_result.message}",
                self.mode,
                data={
                    **base_data,
                    "motion_dispatched": True,
                    "grasp_started": True,
                },
            )

        lift = self._move_pose_decomposed(lift_pose, chain=chain)
        base_data["lift_motion"] = lift.to_dict()
        if not lift.succeeded:
            return self._result_with_data(
                lift,
                {
                    **base_data,
                    "motion_dispatched": True,
                    "grasp_started": True,
                    "grasp_completed": False,
                },
            )
        return ok(
            "已按二次观测完成下探、闭爪和抬起；抓取结果仍需视觉复核",
            self.mode,
            data={
                **base_data,
                "motion_dispatched": True,
                "grasp_started": True,
                "grasp_state": "closed_and_lifted_unverified",
                "grasp_completed": False,
            },
        )


__all__ = ["ChargerGraspTool"]
