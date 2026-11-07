"""Tool-layer state machine: head strip search -> charger -> wrist handoff."""

from __future__ import annotations

import functools
import json
import math
import os
import typing
from pathlib import Path

from sdk_wrapper.dev.algorithms.charger_search import plan_head_search_action
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
        "reset_charger_search",
    )

    def __init__(
        self,
        perception: PerceptionTool,
        *,
        strip_tool: typing.Any = None,
        charger_tool: typing.Any = None,
        config_path: str | os.PathLike[str] = _CONFIG_PATH,
    ) -> None:
        self.perception = perception
        self.adapter = perception.adapter
        with open(config_path, encoding="utf-8") as fh:
            self.config = json.load(fh)
        self.strip_tool = strip_tool or PowerStripYoloTool(perception)
        self.charger_tool = charger_tool or ChargerYoloTool(perception)
        self.phase = str(self.config.get("initial_phase", "find_power_strip"))
        self.step_count = 0
        self.travelled_m = 0.0
        self.charger_ready_streak = 0

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
        charger = None
        charger_result = None
        if self.phase in ("find_charger", "approach_charger"):
            charger_result = self.charger_tool.detect_charger_head()
            charger = self._result_data(charger_result)
            if charger is not None:
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
                    "stereo", "observation_ref",
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
            self.adapter.stop_base()
            return failed(
                ErrorCode.SAFETY_LIMIT,
                "视觉搜索达到最大步数，已停止底盘",
                self.mode,
                data=before,
            )

        if action == "lock_base_for_wrist":
            stopped = self.adapter.stop_base()
            if not stopped.succeeded:
                return unknown(
                    stopped.error_code or ErrorCode.STATE_UNCERTAIN,
                    f"腕部交接前底盘停止状态未知：{stopped.message}",
                    self.mode,
                    data={**before, "stop_result": stopped.to_dict()},
                )
            self.phase = "ready_for_wrist"
            payload = self._snapshot(strip_result, charger_result, plan)
            payload["wrist_handoff_requested"] = True
            return ok(
                "头部 charger 观察满足腕部交接条件，底盘已锁止",
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
            stopped = self.adapter.stop_base()
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
            self.adapter.stop_base()
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

        state = self.adapter.get_robot_state()
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

        pre_stop = self.adapter.stop_base()
        if not pre_stop.succeeded:
            return unknown(
                pre_stop.error_code or ErrorCode.STATE_UNCERTAIN,
                "移动前无法确认底盘停止",
                self.mode,
                data=before,
            )
        motion = self.adapter.move_base(forward_m, yaw_rad)
        if not motion.succeeded:
            return motion
        post_stop = self.adapter.stop_base()
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
    def reset_charger_search(self) -> ToolResult:
        """Reset workflow counters without moving the robot."""
        self.phase = str(self.config.get("initial_phase", "find_power_strip"))
        self.step_count = 0
        self.travelled_m = 0.0
        self.charger_ready_streak = 0
        return ok(
            "视觉搜索状态已重置",
            self.mode,
            data={
                "phase": self.phase,
                "step_count": 0,
                "travelled_m": 0.0,
                "charger_ready_streak": 0,
            },
        )
