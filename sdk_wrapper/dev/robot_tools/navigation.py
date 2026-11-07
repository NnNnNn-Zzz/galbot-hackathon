"""Allowlisted navigation and task lifecycle tools."""

from __future__ import annotations

import math
import typing

from sdk_wrapper.dev.adapters.base import RobotAdapter
from sdk_wrapper.dev.contracts import RunMode, ToolResult
from sdk_wrapper.dev.robot_tools.safety import RobotSafety


class NavigationTool:
    """Navigate only to registered places and manage returned task IDs."""

    def __init__(
        self,
        adapter: RobotAdapter,
        *,
        safety: RobotSafety | None = None,
        real_authorized: bool = False,
    ) -> None:
        self.adapter = adapter
        self.safety = safety or RobotSafety(
            adapter, real_authorized=real_authorized
        )
        if self.safety.adapter is not adapter:
            raise ValueError("safety and navigation tool must share the same adapter")

    @property
    def mode(self) -> RunMode:
        return self.safety.mode

    def navigate_to_place(self, place_id: str) -> ToolResult:
        """Start navigation to a registered location."""
        return self.safety.start_navigation(place_id)

    def get_map_pose(self) -> ToolResult:
        """Read the chassis map pose used for observation-point composition."""
        return self.adapter.get_map_pose()

    def check_path_reachability(
        self,
        goal_pose: typing.Sequence[float],
        start_pose: typing.Sequence[float] | None = None,
    ) -> ToolResult:
        return self.adapter.check_path_reachability(goal_pose, start_pose)

    def navigate_to_pose(
        self,
        pose: typing.Sequence[float],
        *,
        frame: str = "map",
        speed_ratio: float = 0.5,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        target = [float(v) for v in list(pose)]
        if len(target) != 7 or not all(math.isfinite(v) for v in target):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose 必须是 7 个有限数值",
                mode=self.mode,
            )
        return self.safety.start_navigation_pose(
            target,
            frame=frame,
            speed_ratio=float(speed_ratio),
            enable_collision_check=bool(enable_collision_check),
        )

    def get_task_status(self, task_id: str) -> ToolResult:
        """Query a task and enforce its configured timeout."""
        return self.safety.get_task_status(task_id)

    def cancel_task(self, task_id: str) -> ToolResult:
        """Cancel a task and report cancellation only when confirmed."""
        return self.safety.cancel_task(task_id)


def navigate_to_place(
    place_id: str,
    *,
    adapter: RobotAdapter,
    safety: RobotSafety | None = None,
) -> ToolResult:
    """Functional entry point for callers that inject their adapter explicitly."""
    return NavigationTool(adapter, safety=safety).navigate_to_place(place_id)


def get_task_status(
    task_id: str,
    *,
    adapter: RobotAdapter,
    safety: RobotSafety,
) -> ToolResult:
    """Query a task through the same safety controller that started it."""
    return NavigationTool(adapter, safety=safety).get_task_status(task_id)


def cancel_task(
    task_id: str,
    *,
    adapter: RobotAdapter,
    safety: RobotSafety,
) -> ToolResult:
    """Cancel a task through the same safety controller that started it."""
    return NavigationTool(adapter, safety=safety).cancel_task(task_id)


__all__ = [
    "NavigationTool",
    "cancel_task",
    "get_task_status",
    "navigate_to_place",
]
