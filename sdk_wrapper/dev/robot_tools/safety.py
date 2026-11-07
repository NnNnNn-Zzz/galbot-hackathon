"""Fail-closed software checks shared by motion and navigation tools."""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from sdk_wrapper.dev.adapters.base import RobotAdapter, SUPPORTED_GESTURES, SUPPORTED_PLACES
from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    TaskStatus,
    ToolResult,
    canceled,
    failed,
    rejected,
    running,
    unknown,
)

LOG = logging.getLogger(__name__)


@dataclass
class _TrackedTask:
    started_at: float
    status: TaskStatus


class RobotSafety:
    """Validate commands and track robot activity through one shared instance.

    Real mode is denied unless ``real_authorized=True`` is explicitly set by
    trusted application code. This is a software gate, not a hardware safety
    system or emergency stop.
    """

    def __init__(
        self,
        adapter: RobotAdapter,
        *,
        real_authorized: bool = False,
        expected_mode: RunMode | str | None = None,
        task_timeout_seconds: float = 120.0,
    ) -> None:
        if not isinstance(real_authorized, bool):
            raise TypeError("real_authorized must be a bool")
        if (
            isinstance(task_timeout_seconds, bool)
            or not isinstance(task_timeout_seconds, (int, float))
            or not math.isfinite(task_timeout_seconds)
            or task_timeout_seconds <= 0
        ):
            raise ValueError("task_timeout_seconds must be a finite positive number")

        self.adapter = adapter
        self.mode = RunMode(adapter.mode)
        self.expected_mode = RunMode(expected_mode) if expected_mode is not None else None
        self.real_authorized = real_authorized
        self.task_timeout_seconds = float(task_timeout_seconds)
        self._clock = time.monotonic
        self._lock = threading.RLock()
        self._tasks: dict[str, _TrackedTask] = {}
        self._global_state_unknown = False

    def start_gesture(self, gesture_id: str) -> ToolResult:
        return self._start_task(gesture_id, "gesture", SUPPORTED_GESTURES)

    def authorize(self) -> ToolResult | None:
        """公开的授权检查入口，供运动原语（推/夹爪）复用同一套闸门。

        返回 None 表示放行；否则返回应原样交给调用方的 ``ToolResult``。
        """
        return self._authorize()

    def trip_global_fuse(self, reason: str) -> None:
        """打上全局"不确定态熔断"：之后所有新运动都会被拒（直到人工确认）。"""
        self._global_state_unknown = True
        LOG.warning("全局熔断：%s", reason)

    def start_navigation(self, place_id: str) -> ToolResult:
        return self._start_task(place_id, "navigation", SUPPORTED_PLACES)

    def get_task_status(self, task_id: str) -> ToolResult:
        invalid = self._validate_task_id(task_id)
        if invalid is not None:
            return invalid

        with self._lock:
            authorization = self._authorize()
            if authorization is not None:
                return authorization
            tracked = self._tasks.get(task_id)
            if tracked is None:
                return failed(
                    ErrorCode.UNKNOWN_TASK,
                    f"Unknown task: {task_id}.",
                    mode=self.mode,
                    task_id=task_id,
                )
            if tracked.status is TaskStatus.UNKNOWN:
                return ToolResult(
                    status=TaskStatus.UNKNOWN,
                    mode=self.mode,
                    message="Task state is uncertain; it must not be treated as stopped.",
                    task_id=task_id,
                    data={"task_id": task_id},
                    error_code=ErrorCode.STATE_UNCERTAIN,
                )

            try:
                result = self.adapter.get_task_status(task_id)
            except Exception as exc:
                LOG.exception("Task status query failed for task %s", task_id)
                return self._mark_unknown(
                    task_id,
                    ErrorCode.STATE_UNCERTAIN,
                    f"Task status query raised {type(exc).__name__}; actual execution state is unknown.",
                )

            result = self._validate_adapter_result(result, "task status query", task_id)
            if result.status is TaskStatus.UNKNOWN:
                self._tasks[task_id].status = TaskStatus.UNKNOWN
                return result
            if result.status is TaskStatus.RUNNING:
                if self._clock() - tracked.started_at >= self.task_timeout_seconds:
                    return self._cancel_after_timeout(task_id)
                return result

            self._tasks[task_id].status = result.status
            return result

    def cancel_task(self, task_id: str) -> ToolResult:
        invalid = self._validate_task_id(task_id)
        if invalid is not None:
            return invalid

        with self._lock:
            authorization = self._authorize()
            if authorization is not None:
                return authorization
            tracked = self._tasks.get(task_id)
            if tracked is None:
                return failed(
                    ErrorCode.UNKNOWN_TASK,
                    f"Unknown task: {task_id}.",
                    mode=self.mode,
                    task_id=task_id,
                )
            if tracked.status is TaskStatus.UNKNOWN:
                return ToolResult(
                    status=TaskStatus.UNKNOWN,
                    mode=self.mode,
                    message="Task state is uncertain; cancellation cannot be confirmed.",
                    task_id=task_id,
                    data={"task_id": task_id},
                    error_code=ErrorCode.STATE_UNCERTAIN,
                )
            if tracked.status is not TaskStatus.RUNNING:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"Task {task_id} is not running.",
                    mode=self.mode,
                    data={"task_status": tracked.status.value},
                )

            try:
                result = self.adapter.cancel_task(task_id)
            except Exception as exc:
                LOG.exception("Task cancellation failed for task %s", task_id)
                return self._mark_unknown(
                    task_id,
                    ErrorCode.STATE_UNCERTAIN,
                    f"Cancellation raised {type(exc).__name__}; actual execution state is unknown.",
                )

            result = self._validate_adapter_result(result, "task cancellation", task_id)
            if result.status is TaskStatus.UNKNOWN:
                self._tasks[task_id].status = TaskStatus.UNKNOWN
            elif result.status is TaskStatus.CANCELLED:
                self._tasks[task_id].status = TaskStatus.CANCELLED
            elif result.status is not TaskStatus.RUNNING:
                # A failed/rejected cancel request does not prove that motion stopped.
                return self._resolve_cancel_failure(task_id, result)
            return result

    def _start_task(
        self, target: str, operation: str, allowlist: frozenset[str]
    ) -> ToolResult:
        invalid = self._validate_registered_id(target, operation, allowlist)
        if invalid is not None:
            return invalid

        with self._lock:
            preflight = self._preflight_start()
            if preflight is not None:
                return preflight

            try:
                if operation == "gesture":
                    result = self.adapter.execute_gesture(target)
                else:
                    result = self.adapter.navigate_to_place(target)
            except Exception as exc:
                LOG.exception("%s execution raised before task state was returned", operation)
                self._global_state_unknown = True
                return unknown(
                    ErrorCode.STATE_UNCERTAIN,
                    f"{operation} raised {type(exc).__name__}; actual robot state is unknown.",
                    mode=self.mode,
                    data={"operation": operation, "target": target},
                )

            result = self._validate_adapter_result(result, f"{operation} start")
            if result.status is TaskStatus.RUNNING and result.task_id is not None:
                self._tasks[result.task_id] = _TrackedTask(
                    started_at=self._clock(), status=TaskStatus.RUNNING
                )
            elif result.status is TaskStatus.UNKNOWN:
                self._global_state_unknown = True
                if result.task_id:
                    self._tasks[result.task_id] = _TrackedTask(
                        started_at=self._clock(), status=TaskStatus.UNKNOWN
                    )
            return result

    def _preflight_start(self) -> ToolResult | None:
        authorization = self._authorize()
        if authorization is not None:
            return authorization
        if self._global_state_unknown:
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                "Robot state is uncertain; new motion is blocked.",
                mode=self.mode,
            )
        for task_id, task in self._tasks.items():
            if task.status is TaskStatus.UNKNOWN:
                return ToolResult(
                    status=TaskStatus.UNKNOWN,
                    mode=self.mode,
                    message="A tracked task has unknown state; new motion is blocked.",
                    task_id=task_id,
                    data={"task_id": task_id},
                    error_code=ErrorCode.STATE_UNCERTAIN,
                )
            if task.status is TaskStatus.RUNNING:
                return rejected(
                    ErrorCode.SAFETY_LIMIT,
                    "Another motion task is still running.",
                    mode=self.mode,
                    data={"active_task_id": task_id},
                )

        try:
            state = self.adapter.get_robot_state()
        except Exception as exc:
            LOG.exception("Robot state preflight failed")
            self._global_state_unknown = True
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"Robot state check raised {type(exc).__name__}; motion is blocked.",
                mode=self.mode,
            )

        state = self._validate_adapter_result(state, "robot state preflight")
        if state.status is TaskStatus.UNKNOWN:
            self._global_state_unknown = True
            return state
        if state.status is not TaskStatus.SUCCEEDED:
            self._global_state_unknown = True
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                "Robot state could not be confirmed; new motion is blocked.",
                mode=self.mode,
                data={"state_status": state.status.value},
            )
        motion_state = state.data.get("motion_state")
        if motion_state == "unknown":
            self._global_state_unknown = True
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                "Robot motion state is unknown; new motion is blocked.",
                mode=self.mode,
                data={"robot_state": state.data},
            )
        if motion_state == "moving":
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                "Robot is not idle; new motion is blocked.",
                mode=self.mode,
                data={"motion_state": motion_state},
            )
        if motion_state != "idle":
            self._global_state_unknown = True
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                "Robot state did not provide a recognized idle/moving state; new motion is blocked.",
                mode=self.mode,
                data={"motion_state": motion_state},
            )
        return None

    def _authorize(self) -> ToolResult | None:
        if self.expected_mode is not None and self.mode is not self.expected_mode:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                f"Adapter mode {self.mode.value!r} does not match configured mode {self.expected_mode.value!r}.",
                mode=self.mode,
            )
        if self.mode is RunMode.REAL and not self.real_authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "Real robot access is disabled; trusted application authorization is required.",
                mode=self.mode,
            )
        return None

    def _validate_adapter_result(
        self, result: ToolResult, operation: str, task_id: str | None = None
    ) -> ToolResult:
        if not isinstance(result, ToolResult):
            return self._mark_unknown(
                task_id,
                ErrorCode.STATE_UNCERTAIN,
                f"Adapter {operation} returned {type(result).__name__}, not ToolResult.",
            )
        if result.mode is not self.mode:
            return self._mark_unknown(
                task_id or result.task_id,
                ErrorCode.STATE_UNCERTAIN,
                f"Adapter {operation} reported mode {result.mode.value!r}, expected {self.mode.value!r}.",
            )
        if task_id is not None and result.task_id is None:
            return ToolResult(
                status=result.status,
                mode=result.mode,
                message=result.message,
                task_id=task_id,
                data=result.data,
                error_code=result.error_code,
            )
        return result

    def _cancel_after_timeout(self, task_id: str) -> ToolResult:
        try:
            result = self.adapter.cancel_task(task_id)
        except Exception as exc:
            LOG.exception("Timed-out task cancellation failed for task %s", task_id)
            return self._mark_unknown(
                task_id,
                ErrorCode.TIMEOUT,
                f"Task exceeded its timeout and cancellation raised {type(exc).__name__}; state is unknown.",
            )

        result = self._validate_adapter_result(result, "timeout cancellation", task_id)
        if result.status is TaskStatus.CANCELLED:
            self._tasks[task_id].status = TaskStatus.CANCELLED
            return canceled(
                "Task exceeded its timeout and cancellation was confirmed.",
                mode=self.mode,
                task_id=task_id,
                data={"timeout_seconds": self.task_timeout_seconds},
            )

        self._tasks[task_id].status = TaskStatus.UNKNOWN
        return ToolResult(
            status=TaskStatus.UNKNOWN,
            mode=self.mode,
            message="Task exceeded its timeout; cancellation was not confirmed and motion state is unknown.",
            task_id=task_id,
            data={"task_id": task_id, "timeout_seconds": self.task_timeout_seconds},
            error_code=ErrorCode.TIMEOUT,
        )

    def _resolve_cancel_failure(self, task_id: str, cancel_result: ToolResult) -> ToolResult:
        try:
            state = self.adapter.get_task_status(task_id)
        except Exception as exc:
            LOG.exception("Task state could not be resolved after cancellation failure")
            return self._mark_unknown(
                task_id,
                ErrorCode.STATE_UNCERTAIN,
                f"Cancellation was not confirmed and status query raised {type(exc).__name__}.",
            )

        state = self._validate_adapter_result(state, "post-cancellation status query", task_id)
        if state.status is TaskStatus.RUNNING:
            return running(
                "Cancellation was not confirmed; task is still running.",
                mode=self.mode,
                task_id=task_id,
                data={"cancel_error_code": str(cancel_result.error_code or "")},
            )
        if state.status is TaskStatus.UNKNOWN:
            self._tasks[task_id].status = TaskStatus.UNKNOWN
            return state
        self._tasks[task_id].status = state.status
        return state

    def _mark_unknown(
        self, task_id: str | None, code: ErrorCode, message: str
    ) -> ToolResult:
        if task_id and task_id in self._tasks:
            self._tasks[task_id].status = TaskStatus.UNKNOWN
        else:
            self._global_state_unknown = True
        return ToolResult(
            status=TaskStatus.UNKNOWN,
            mode=self.mode,
            message=message,
            task_id=task_id,
            data={"task_id": task_id} if task_id else {},
            error_code=code,
        )

    def _validate_registered_id(
        self, value: str, operation: str, allowlist: frozenset[str]
    ) -> ToolResult | None:
        parameter = "gesture_id" if operation == "gesture" else "place_id"
        if not isinstance(value, str) or not value.strip():
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"{parameter} must be a non-empty string.",
                mode=self.mode,
                data={"parameter": parameter},
            )
        if value not in allowlist:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Unsupported {parameter}: {value!r}.",
                mode=self.mode,
                data={"parameter": parameter, "supported_ids": sorted(allowlist)},
            )
        return None

    def _validate_task_id(self, task_id: str) -> ToolResult | None:
        if not isinstance(task_id, str) or not task_id.strip():
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "task_id must be a non-empty string.",
                mode=self.mode,
                data={"parameter": "task_id"},
            )
        return None


__all__ = [
    "RobotSafety",
    "SUPPORTED_GESTURES",
    "SUPPORTED_PLACES",
]
