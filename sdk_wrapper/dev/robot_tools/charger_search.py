"""Tool-layer state machine: head strip search -> charger -> wrist handoff."""

from __future__ import annotations

import functools
import json
import math
import os
import time
import typing
from pathlib import Path

from sdk_wrapper.dev.algorithms.charger_search import plan_head_search_action
from sdk_wrapper.dev.algorithms.calibration import rotation_error_deg
from sdk_wrapper.dev.algorithms.observation_pose import (
    end_effector_pose_for_camera,
    plan_map_observation_pose,
    plan_wrist_camera_pose,
)
from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    ToolError,
    ToolResult,
    failed,
    ok,
    rejected,
    running,
    unknown,
)
from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool
from sdk_wrapper.dev.robot_tools.motion import MotionTool
from sdk_wrapper.dev.robot_tools.navigation import NavigationTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool
from sdk_wrapper.dev.robot_tools.power_strip_yolo import PowerStripYoloTool

__all__ = ["ChargerSearchTool"]

_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "charger_search.json"


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    @functools.wraps(method)
    def wrapper(self: "ChargerSearchTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
        try:
            return method(self, *args, **kwargs)
        except ToolError as exc:
            return exc.to_result(self.mode)
        except Exception as exc:  # noqa: BLE001 - tool boundary
            return failed(
                ErrorCode.SDK_ERROR,
                f"{type(exc).__name__}: {exc}",
                self.mode,
                data={"tool": method.__name__, "phase": self.phase},
            )

    return wrapper


class ChargerSearchTool:
    """Advance the visual search one bounded segment at a time."""

    TOOL_NAMES = (
        "observe_charger_search",
        "advance_charger_search",
        "monitor_observation_navigation",
        "align_wrist_from_head",
        "reset_charger_search",
    )

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        strip_tool: typing.Any = None,
        charger_tool: typing.Any = None,
        motion_tool: MotionTool | None = None,
        navigation_tool: NavigationTool | None = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
    ) -> None:
        self.perception = perception
        self.adapter = perception.adapter
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        self.strip_tool = strip_tool or PowerStripYoloTool(perception)
        self.charger_tool = charger_tool or ChargerYoloTool(perception)
        self.motion_tool = motion_tool or MotionTool(
            self.adapter,
            real_authorized=self.mode is RunMode.REAL,
        )
        self.navigation_tool = navigation_tool or NavigationTool(
            self.adapter,
            real_authorized=self.mode is RunMode.REAL,
        )
        self.phase = str(self.config.get("initial_phase", "find_power_strip"))
        self.step_count = 0
        self.travelled_m = 0.0
        self.charger_ready_streak = 0
        self.navigation_task_id: str | None = None
        self.track_miss_count = 0
        self.last_head_charger: dict[str, typing.Any] | None = None

    @property
    def mode(self) -> RunMode:
        return self.perception.mode

    @staticmethod
    def _result_data(result: ToolResult) -> dict[str, typing.Any] | None:
        return dict(result.data) if result.succeeded else None

    @staticmethod
    def _model_backend_error(*results: ToolResult | None) -> ToolResult | None:
        for result in results:
            if (
                result is not None
                and result.error_code is not None
                and "backend unavailable" in result.message.lower()
            ):
                return failed(
                    ErrorCode.NOT_READY,
                    result.message,
                    result.mode,
                )
        return None

    def _attach_observation_plan(self, strip: dict[str, typing.Any] | None) -> None:
        if strip is None or self.phase not in ("find_power_strip", "approach_power_strip"):
            return
        stereo = strip.get("stereo") or {}
        safety = stereo.get("target_safety") or {}
        target_3d = stereo.get("target_3d") or {}
        if not safety.get("safe") or not target_3d.get("available"):
            strip["observation_plan"] = {
                "safe_to_submit": False,
                "path_reachable": False,
                "reasons": list(safety.get("reasons") or ["target_3d_unavailable"]),
            }
            return
        map_pose_result = self.navigation_tool.get_map_pose()
        if not map_pose_result.succeeded:
            strip["observation_plan"] = {
                "safe_to_submit": False,
                "path_reachable": False,
                "reasons": ["slam_map_pose_unavailable"],
                "map_pose_error": map_pose_result.message,
            }
            return
        try:
            plan = plan_map_observation_pose(
                target_3d["center_m"],
                map_pose_result.data["pose"],
                observation_distance_m=float(
                    self.config.get("observation_distance_m", 0.8)
                ),
                min_observation_distance_m=float(
                    self.config.get("min_observation_distance_m", 0.55)
                ),
                max_observation_distance_m=float(
                    self.config.get("max_observation_distance_m", 1.5)
                ),
            )
        except Exception as exc:  # noqa: BLE001
            strip["observation_plan"] = {
                "safe_to_submit": False,
                "path_reachable": False,
                "reasons": [f"observation_pose_failed:{type(exc).__name__}:{exc}"],
            }
            return
        if plan["safe_to_submit"]:
            path_result = self.navigation_tool.check_path_reachability(
                plan["observation_pose_map"],
                map_pose_result.data["pose"],
            )
            plan["path_reachable"] = path_result.succeeded and bool(
                path_result.data.get("reachable")
            )
            if not plan["path_reachable"]:
                plan["reasons"].append("slam_path_not_reachable")
                plan["path_check_message"] = path_result.message
        else:
            plan["path_reachable"] = False
        strip["observation_plan"] = plan

    def _observe(
        self,
    ) -> tuple[
        ToolResult,
        dict[str, typing.Any] | None,
        ToolResult | None,
        dict[str, typing.Any] | None,
    ]:
        strip_result = self.strip_tool.detect_power_strip_head()
        strip = self._result_data(strip_result)
        self._attach_observation_plan(strip)
        if strip is not None and "observation_plan" in strip:
            strip_result.data["observation_plan"] = strip["observation_plan"]
        charger = None
        charger_result = None
        if self.phase in ("find_charger", "approach_charger"):
            charger_result = self.charger_tool.detect_charger_head()
            charger = self._result_data(charger_result)
            if charger is not None:
                self.last_head_charger = dict(charger_result.data)
                charger = dict(charger.get("head_handoff") or {})
                charger["touches_image_edge"] = bool(
                    (charger_result.data.get("image_space") or {}).get("touches_image_edge")
                )
                if charger.get("usable_for_wrist_handoff"):
                    self.charger_ready_streak += 1
                else:
                    self.charger_ready_streak = 0
        else:
            self.charger_ready_streak = 0
        return strip_result, strip, charger_result, charger

    def _snapshot(
        self,
        strip_result: ToolResult,
        charger_result: ToolResult | None,
        plan: dict[str, typing.Any] | None,
    ) -> dict[str, typing.Any]:
        strip_data = self._result_data(strip_result)
        charger_data = None if charger_result is None else self._result_data(charger_result)
        return {
            "phase": self.phase,
            "step_count": self.step_count,
            "travelled_m": round(self.travelled_m, 4),
            "charger_ready_streak": self.charger_ready_streak,
            "power_strip": None if strip_data is None else {
                key: strip_data.get(key)
                for key in (
                    "confidence", "bbox_xyxy", "bbox_width_px", "bbox_center_uv",
                    "horizontal_angle_deg", "touches_image_edge", "usable_for_direction",
                    "stereo", "observation_plan", "observation_ref",
                )
            },
            "head_charger": None if charger_data is None else {
                key: charger_data.get(key)
                for key in (
                    "confidence", "bbox_xyxy", "bbox_center_uv",
                    "image_space", "head_handoff", "observation_ref",
                )
            },
            "plan": plan,
            "base_locked": self.phase == "ready_for_wrist",
        }

    @_tool
    def observe_charger_search(self) -> ToolResult:
        """Observe and return the next proposed action without moving."""
        strip_result, strip, _charger_result, charger = self._observe()
        # If the strip is not detectable but the model itself is unavailable,
        # propagate that issue rather than spinning forever.
        backend_error = self._model_backend_error(strip_result, _charger_result)
        if backend_error is not None:
            backend_error.data["phase"] = self.phase
            return backend_error
        plan = plan_head_search_action(
            self.phase,
            strip=strip,
            charger=charger,
            config=self.config,
            step_count=self.step_count,
        )
        required_streak = int(self.config.get("charger_handoff_streak", 3))
        if plan["action"] == "lock_base_for_wrist" and self.charger_ready_streak < required_streak:
            plan = {
                **plan,
                "action": "wait_for_stable_charger_handoff",
                "next_phase": self.phase,
                "requires_motion": False,
                "reason": "charger_handoff_requires_%d_consecutive_frames" % required_streak,
            }
        payload = self._snapshot(strip_result, _charger_result, plan)
        message = f"[{self.phase}] 下一步：{plan['action']}（{plan['reason']}）"
        return ok(message, self.mode, data=payload)

    @_tool
    def advance_charger_search(self, operator_confirmed: bool = False) -> ToolResult:
        """Execute at most one bounded visual-servo segment, then re-observe."""
        strip_result, strip, charger_result, charger = self._observe()
        backend_error = self._model_backend_error(strip_result, charger_result)
        if backend_error is not None:
            backend_error.data["phase"] = self.phase
            return backend_error
        plan = plan_head_search_action(
            self.phase,
            strip=strip,
            charger=charger,
            config=self.config,
            step_count=self.step_count,
        )
        required_streak = int(self.config.get("charger_handoff_streak", 3))
        if plan["action"] == "lock_base_for_wrist" and self.charger_ready_streak < required_streak:
            plan = {
                **plan,
                "action": "wait_for_stable_charger_handoff",
                "next_phase": self.phase,
                "requires_motion": False,
                "reason": "charger_handoff_requires_%d_consecutive_frames" % required_streak,
            }
        before = self._snapshot(strip_result, charger_result, plan)
        action = str(plan["action"])

        if action == "abort":
            self.motion_tool.stop_base()
            return failed(
                ErrorCode.SAFETY_LIMIT,
                "视觉搜索达到最大步数，已停止底盘",
                self.mode,
                data=before,
            )

        if action == "lock_base_for_wrist":
            stopped = self.motion_tool.stop_base()
            if not stopped.succeeded:
                return unknown(
                    stopped.error_code or ErrorCode.STATE_UNCERTAIN,
                    f"腕部交接前底盘停止状态未知：{stopped.message}",
                    self.mode,
                    data={**before, "stop_result": stopped.to_dict()},
                )
            self.phase = "ready_for_wrist"
            self.last_head_charger = (
                None if charger_result is None else dict(charger_result.data)
            )
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["wrist_handoff_requested"] = True
            return ok(
                "头部 charger 观察满足腕部交接条件，底盘已锁止",
                self.mode,
                data=payload,
            )

        if action == "navigate_to_mapped_observation":
            if operator_confirmed is not True:
                return rejected(
                    ErrorCode.PERMISSION_DENIED,
                    "SLAM 观察位导航必须由现场监督者确认",
                    self.mode,
                    data=before,
                )
            observation_plan = (strip or {}).get("observation_plan") or {}
            target_pose = observation_plan.get("observation_pose_map")
            if not target_pose:
                return failed(
                    ErrorCode.STATE_UNCERTAIN,
                    "观察位规划结果缺失",
                    self.mode,
                    data=before,
                )
            pre_stop = self.motion_tool.stop_base()
            if not pre_stop.succeeded:
                return unknown(
                    pre_stop.error_code or ErrorCode.STATE_UNCERTAIN,
                    "SLAM 导航前底盘停止状态未知",
                    self.mode,
                    data=before,
                )
            nav_result = self.navigation_tool.navigate_to_pose(
                target_pose,
                frame="map",
                speed_ratio=float(self.config.get("navigation_speed_ratio", 0.5)),
                enable_collision_check=True,
            )
            if nav_result.status.value not in ("running", "succeeded"):
                return nav_result
            if nav_result.status.value != "running" or not nav_result.task_id:
                return failed(
                    ErrorCode.STATE_UNCERTAIN,
                    f"SLAM 导航未返回可跟踪任务：{nav_result.message}",
                    self.mode,
                    data={**before, "navigation": nav_result.to_dict()},
                )
            self.phase = "navigating_to_observation"
            self.navigation_task_id = nav_result.task_id
            self.track_miss_count = 0
            payload = self._snapshot(strip_result, charger_result, plan)
            payload.update(
                {
                    "navigation": nav_result.to_dict(),
                    "observation_plan": observation_plan,
                    "tracking_required": True,
                }
            )
            return running(
                "SLAM 观察位导航已下发；下一步调用 monitor_observation_navigation 持续跟踪目标",
                self.mode,
                task_id=nav_result.task_id,
                data=payload,
            )

        if action in ("hold_for_stable_metric_stereo", "hold_for_mapped_observation_pose"):
            stopped = self.motion_tool.stop_base()
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["executed"] = False
            payload["operator_action_required"] = "check_stereo_scale_or_slam_observation_pose"
            if stopped.succeeded:
                return ok(
                    "底盘保持停止；需要完整米制双目和可达观察位后才能 SLAM 导航",
                    self.mode,
                    data=payload,
                )
            return unknown(
                stopped.error_code or ErrorCode.STATE_UNCERTAIN,
                "无法确认底盘已停止",
                self.mode,
                data=payload,
            )

        if action == "wait_for_stable_charger_handoff":
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["executed"] = False
            return ok(
                f"charger 交接帧稳定性 {self.charger_ready_streak}/{required_streak}，继续观察",
                self.mode,
                data=payload,
            )

        if action == "stop_and_scan_charger":
            stopped = self.motion_tool.stop_base()
            if not stopped.succeeded:
                return unknown(
                    stopped.error_code or ErrorCode.STATE_UNCERTAIN,
                    f"观察位停止状态未知：{stopped.message}",
                    self.mode,
                    data=before,
                )
            self.phase = "find_charger"
            after_strip, _after_strip_data, after_charger_result, after_charger = self._observe()
            after_plan = plan_head_search_action(
                self.phase,
                strip=strip,
                charger=after_charger,
                config=self.config,
                step_count=self.step_count,
            )
            if (
                after_plan["action"] == "lock_base_for_wrist"
                and self.charger_ready_streak < required_streak
            ):
                after_plan = {
                    **after_plan,
                    "action": "wait_for_stable_charger_handoff",
                    "next_phase": self.phase,
                    "requires_motion": False,
                    "reason": "charger_handoff_requires_%d_consecutive_frames" % required_streak,
                }
            payload = self._snapshot(after_strip, after_charger_result, after_plan)
            payload["previous_plan"] = plan
            payload["executed"] = False
            payload["observation_point_reached"] = True
            return ok(
                "已到达头部观察带并停止，开始头部 charger 检测",
                self.mode,
                data=payload,
            )

        if action == "adjust_head_to_search_charger":
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["executed"] = False
            payload["operator_action_required"] = "adjust_head_view_then_retry"
            return ok(
                "底盘保持停止；需要调整头部视角后重新检测 charger",
                self.mode,
                data=payload,
            )

        if action == "stop_and_adjust_view":
            self.motion_tool.stop_base()
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["executed"] = False
            payload["operator_action_required"] = "adjust_head_view_then_retry"
            return ok(
                "strip 已接近观察位，停止前进并调整头部视角",
                self.mode,
                data=payload,
            )

        if not plan["requires_motion"]:
            return ok(
                f"无需运动：{plan['reason']}",
                self.mode,
                data=before,
            )
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "每一步底盘视觉靠近都必须由现场监督者确认",
                self.mode,
                data=before,
            )

        state = self.motion_tool.get_robot_state()
        if not state.succeeded:
            return unknown(
                state.error_code or ErrorCode.STATE_UNCERTAIN,
                f"移动前机器人状态不可确认：{state.message}",
                self.mode,
                data=before,
            )
        if state.data.get("motion_state") != "idle":
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"底盘/机器人不是 idle：{state.data.get('motion_state')}",
                self.mode,
                data=before,
            )

        forward_m = float(plan["forward_m"])
        yaw_rad = math.radians(float(plan["yaw_deg"]))
        max_total = float(self.config.get("max_total_travel_m", 2.0))
        if abs(forward_m) > float(self.config.get("max_segment_m", 0.20)):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "视觉靠近单段超过配置上限",
                self.mode,
                data=before,
            )
        if abs(self.travelled_m + forward_m) > max_total:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "视觉靠近累计路程超过配置上限",
                self.mode,
                data=before,
            )

        pre_stop = self.motion_tool.stop_base()
        if not pre_stop.succeeded:
            return unknown(
                pre_stop.error_code or ErrorCode.STATE_UNCERTAIN,
                "移动前无法确认底盘停止",
                self.mode,
                data=before,
            )
        motion = self.motion_tool.move_base(forward_m, yaw_rad)
        if not motion.succeeded:
            return motion
        post_stop = self.motion_tool.stop_base()
        if not post_stop.succeeded:
            return unknown(
                post_stop.error_code or ErrorCode.STATE_UNCERTAIN,
                "分段结束后底盘停止状态未知",
                self.mode,
                data={**before, "motion": motion.to_dict()},
            )

        self.travelled_m += forward_m
        self.step_count += 1
        self.phase = str(plan["next_phase"])

        # Mandatory fresh detection after every segment.
        after_strip, _strip_data, after_charger_result, after_charger = self._observe()
        after_plan = plan_head_search_action(
            self.phase,
            strip=_strip_data,
            charger=after_charger,
            config=self.config,
            step_count=self.step_count,
        )
        if (
            after_plan["action"] == "lock_base_for_wrist"
            and self.charger_ready_streak < required_streak
        ):
            after_plan = {
                **after_plan,
                "action": "wait_for_stable_charger_handoff",
                "next_phase": self.phase,
                "requires_motion": False,
                "reason": "charger_handoff_requires_%d_consecutive_frames" % required_streak,
            }
        payload = self._snapshot(after_strip, after_charger_result, after_plan)
        payload.update(
            {
                "executed": True,
                "executed_plan": plan,
                "motion": motion.to_dict(),
                "post_stop": post_stop.to_dict(),
            }
        )
        return ok(
            f"已执行一段 {forward_m:.3f} m / {float(plan['yaw_deg']):.2f}° 并重新检测",
            self.mode,
            data=payload,
        )

    @_tool
    def monitor_observation_navigation(self, task_id: str | None = None) -> ToolResult:
        """Poll SLAM navigation once while preserving head-camera target tracking."""
        if self.phase != "navigating_to_observation":
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"当前阶段 {self.phase!r} 不是导航观察位阶段",
                self.mode,
            )
        tracked_task_id = task_id or self.navigation_task_id
        if not tracked_task_id:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "缺少 SLAM navigation task_id",
                self.mode,
            )

        track_result = self.strip_tool.track_power_strip_head()
        tracking = self._result_data(track_result)
        backend_error = self._model_backend_error(track_result)
        if tracking is None:
            self.track_miss_count += 1
            if backend_error is not None:
                self.track_miss_count = 99
            max_misses = int(self.config.get("max_tracking_misses", 3))
            if self.track_miss_count >= max_misses:
                cancel = self.navigation_tool.cancel_task(tracked_task_id)
                stopped = self.motion_tool.stop_base()
                self.phase = "find_power_strip"
                return failed(
                    ErrorCode.NOT_READY,
                    f"头部目标连续丢失 {self.track_miss_count} 次，已请求停止导航",
                    self.mode,
                    data={
                        "tracking_result": track_result.to_dict(),
                        "cancel_result": cancel.to_dict(),
                        "stop_result": stopped.to_dict(),
                        "track_miss_count": self.track_miss_count,
                    },
                )
        else:
            self.track_miss_count = 0

        nav_status = self.navigation_tool.get_task_status(tracked_task_id)
        payload = {
            "phase": self.phase,
            "navigation_task_id": tracked_task_id,
            "navigation_status": nav_status.to_dict(),
            "target_track_id": (
                None if tracking is None else tracking.get("target_track_id")
            ),
            "target_tracking_ok": tracking is not None,
            "track_miss_count": self.track_miss_count,
            "power_strip": None if tracking is None else {
                key: tracking.get(key)
                for key in (
                    "bbox_xyxy", "horizontal_angle_deg", "target_track_id",
                    "stereo", "observation_plan", "observation_ref",
                )
            },
        }

        if nav_status.status.value == "running":
            return running(
                "SLAM 导航中，头部目标追踪仍有效",
                self.mode,
                task_id=tracked_task_id,
                data=payload,
            )
        if nav_status.status.value == "unknown":
            self.motion_tool.stop_base()
            return nav_status
        if nav_status.status.value != "succeeded":
            self.motion_tool.stop_base()
            return nav_status

        stopped = self.motion_tool.stop_base()
        if not stopped.succeeded:
            return unknown(
                stopped.error_code or ErrorCode.STATE_UNCERTAIN,
                "SLAM 导航已结束，但底盘停止状态未知",
                self.mode,
                data=payload,
            )
        self.phase = "find_charger"
        payload["phase"] = self.phase
        charger_result = self.charger_tool.detect_charger_head()
        charger_data = self._result_data(charger_result)
        self.last_head_charger = charger_data
        charger = None
        if charger_data is not None:
            charger = dict(charger_data.get("head_handoff") or {})
            charger["touches_image_edge"] = bool(
                (charger_data.get("image_space") or {}).get("touches_image_edge")
            )
            if charger.get("usable_for_wrist_handoff"):
                self.charger_ready_streak += 1
            else:
                self.charger_ready_streak = 0
        plan = plan_head_search_action(
            self.phase,
            strip=tracking,
            charger=charger,
            config=self.config,
            step_count=self.step_count,
        )
        payload["navigation_arrived"] = True
        payload["head_charger"] = (
            None if charger_data is None
            else {
                key: charger_data.get(key)
                for key in (
                    "confidence", "bbox_xyxy", "image_space",
                    "head_handoff", "observation_ref",
                )
            }
        )
        payload["plan"] = plan
        return ok(
            "SLAM 观察到点并停稳，开始头部 charger 检测",
            self.mode,
            data=payload,
        )

    @_tool
    def align_wrist_from_head(self, operator_confirmed: bool = False) -> ToolResult:
        """Use head 3-D target and official hand-eye TF to pose the wrist camera."""
        if self.phase != "ready_for_wrist":
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"当前阶段 {self.phase!r} 不允许腕部摆位",
                self.mode,
            )
        if operator_confirmed is not True:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "腕部从头部视觉摆位必须由现场监督者确认",
                self.mode,
            )
        if self.last_head_charger is None:
            return failed(
                ErrorCode.NOT_READY,
                "缺少头部 charger 观测",
                self.mode,
            )
        handoff = self.last_head_charger.get("head_handoff") or {}
        target_3d = handoff.get("target_3d") or {}
        safety = handoff.get("target_safety") or {}
        if not target_3d.get("available") or not safety.get("safe"):
            return failed(
                ErrorCode.NOT_READY,
                "头部 charger 的米制 3D 目标或安全检查不可用",
                self.mode,
                data={"target_3d": target_3d, "target_safety": safety},
            )
        camera_plan = plan_wrist_camera_pose(
            target_3d["center_m"],
            target_3d["surface_normal_base"],
            target_3d["long_axis_base"],
            camera_standoff_m=float(self.config.get("wrist_camera_standoff_m", 0.30)),
            min_camera_standoff_m=float(
                self.config.get("min_wrist_camera_standoff_m", 0.20)
            ),
            max_camera_standoff_m=float(
                self.config.get("max_wrist_camera_standoff_m", 0.55)
            ),
        )
        if not camera_plan["safe_to_plan"]:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "腕部相机观察位姿超出安全余量",
                self.mode,
                data={"camera_plan": camera_plan},
            )

        current_ee = self.motion_tool.get_chain_frame_pose(
            "right_arm", "EndEffector", "base_link"
        )
        current_camera = self.motion_tool.get_chain_frame_pose(
            "right_arm", "Camera", "base_link"
        )
        if not current_ee.succeeded or not current_camera.succeeded:
            return failed(
                ErrorCode.NOT_READY,
                "无法读取当前腕部相机手眼位姿",
                self.mode,
                data={
                    "current_ee": current_ee.to_dict(),
                    "current_camera": current_camera.to_dict(),
                },
            )
        transferred = end_effector_pose_for_camera(
            camera_plan["camera_pose_base"],
            current_ee.data["pose"],
            current_camera.data["pose"],
        )
        ee_goal = transferred["end_effector_pose_base"]
        motion = self.motion_tool.move_ee_to(
            ee_goal,
            chain="right_arm",
            straight_line=False,
        )
        if not motion.succeeded:
            return motion

        measured = self.motion_tool.get_chain_frame_pose(
            "right_arm", "Camera", "base_link"
        )
        if not measured.succeeded:
            return unknown(
                measured.error_code or ErrorCode.STATE_UNCERTAIN,
                "腕部移动完成，但相机位姿无法确认",
                self.mode,
                data={"motion": motion.to_dict()},
            )
        desired = camera_plan["camera_pose_base"]
        actual = measured.data["pose"]
        translation_error = float(
            math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(actual[:3], desired[:3])))
        )
        rotation_error = rotation_error_deg(actual, desired)
        matched = (
            translation_error <= float(self.config.get("wrist_pose_position_tolerance_m", 0.01))
            and rotation_error <= float(self.config.get("wrist_pose_rotation_tolerance_deg", 3.0))
        )
        data = {
            "phase": self.phase,
            "head_target": target_3d,
            "camera_plan": camera_plan,
            "end_effector_goal": ee_goal,
            "measured_camera_pose": actual,
            "translation_error_m": round(translation_error, 6),
            "rotation_error_deg": round(rotation_error, 6),
            "pose_matched": matched,
        }
        if not matched:
            return failed(
                ErrorCode.NOT_READY,
                "腕部相机未达到头部规划的垂直观察位",
                self.mode,
                data=data,
            )
        self.phase = "wrist_view_aligned"
        data["phase"] = self.phase
        data["wrist_handoff_ready"] = True
        return ok(
            "右腕相机已移动到目标上方并对准表面法向",
            self.mode,
            data=data,
        )

    @_tool
    def reset_charger_search(self) -> ToolResult:
        """Reset workflow counters without moving the robot."""
        self.phase = str(self.config.get("initial_phase", "find_power_strip"))
        self.step_count = 0
        self.travelled_m = 0.0
        self.charger_ready_streak = 0
        self.navigation_task_id = None
        self.track_miss_count = 0
        self.last_head_charger = None
        if hasattr(self.strip_tool, "last_track_id"):
            self.strip_tool.last_track_id = None
        return ok(
            "视觉搜索状态已重置",
            self.mode,
            data={
                "phase": self.phase,
                "step_count": 0,
                "travelled_m": 0.0,
                "charger_ready_streak": 0,
                "navigation_task_id": None,
                "track_miss_count": 0,
            },
        )
