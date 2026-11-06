"""Allowlisted navigation and task lifecycle tools."""

from __future__ import annotations

from adapters.base import RobotAdapter
from contracts import RunMode, ToolResult
from robot_tools.safety import RobotSafety


class NavigationTool:
    """Navigate only to registered places and manage returned task IDs."""

    def __init__(
        self,
        adapter: RobotAdapter,
        *,
        safety: RobotSafety | None = None,
        real_authorized: bool = False,
    ) -> None:
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
