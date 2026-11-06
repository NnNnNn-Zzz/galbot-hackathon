"""双方共享的数据契约（对应 analysis/Galbot_G1_双人开发接口标准_v0.1.md §2）。

本文件是 A（底层执行层）与 B（感知与决策层）之间的"插头标准"。
**改动前必须通知另一方**；字段名、状态拼写、错误码不得单方面修改。

固定 6 个键：
    status / mode / message / task_id / data / error_code

为什么要有这个文件：
    大模型做 Function Calling 时只认得稳定 schema。如果 A 返回 `ok: True`、
    B 返回 `success: 1`，Agent 就必须写 N 套解析逻辑。统一成 ToolResult 之后，
    Agent 只看 `status` 一个字段，其余靠 `data`。

本模块**不依赖 numpy / opencv / galbot_sdk**，因此 Windows 上也毫无问题，
可以被 C++ 侧以外的任何组件安全导入。
"""

from __future__ import annotations

import enum
import json
import typing
from dataclasses import dataclass, field

__all__ = [
    "RunMode",
    "TaskStatus",
    "ErrorCode",
    "ToolResult",
    "ToolError",
    "ok",
    "running",
    "failed",
    "rejected",
    "canceled",
    "unknown",
    "to_json",
]


# --------------------------------------------------------------------------- #
# 枚举（拼写即协议，勿改）
# --------------------------------------------------------------------------- #
class RunMode(str, enum.Enum):
    """执行模式。mock / real 必须显式标记，严禁静默回退。"""

    MOCK = "mock"
    REAL = "real"

    def __str__(self) -> str:  # 让 f"{RunMode.MOCK}" 得到 "mock" 而不是 "RunMode.MOCK"
        return self.value


class TaskStatus(str, enum.Enum):
    """6 种状态，含义见协议 §2。"""

    SUCCEEDED = "succeeded"  # 确实完成（导航不得在下发瞬间使用）
    RUNNING = "running"      # 正在执行，必须带非空 task_id
    FAILED = "failed"        # 执行失败，必须带 error_code
    REJECTED = "rejected"    # 输入/安全/权限未通过，根本没执行
    CANCELLED = "cancelled"  # 已确认取消，不是"收到取消请求"
    UNKNOWN = "unknown"      # 状态不可知（如断网），不得认为机器人已停

    def __str__(self) -> str:
        return self.value


class ErrorCode(str, enum.Enum):
    """9 个错误码。"""

    INVALID_ARGUMENT = "invalid_argument"
    NOT_READY = "not_ready"
    NOT_SUPPORTED = "not_supported"
    PERMISSION_DENIED = "permission_denied"
    SAFETY_LIMIT = "safety_limit"
    SDK_ERROR = "sdk_error"
    TIMEOUT = "timeout"
    UNKNOWN_TASK = "unknown_task"
    STATE_UNCERTAIN = "state_uncertain"

    def __str__(self) -> str:
        return self.value


# 这些状态"必须"给出 error_code，否则 Agent 无法解释失败原因
_REQUIRE_ERROR_CODE = frozenset(
    {TaskStatus.FAILED, TaskStatus.REJECTED, TaskStatus.UNKNOWN}
)
# 这些状态"必须"给出 task_id，否则 Agent 无法后续查询
_REQUIRE_TASK_ID = frozenset({TaskStatus.RUNNING})


# --------------------------------------------------------------------------- #
# JSON 净化：numpy 标量 / 数组 / enum / dict-with-non-str-key 都可能混进 data
# --------------------------------------------------------------------------- #
def _plain(obj: typing.Any) -> typing.Any:
    """递归转成 json.dumps 能吃的结构（不 import numpy，靠鸭子类型）。"""
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, enum.Enum):
        return obj.value
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset)):
        return [_plain(v) for v in obj]
    # numpy 标量：.item()；numpy 数组：.tolist()
    item = getattr(obj, "item", None)
    if callable(item):
        try:
            return _plain(item())
        except Exception:  # noqa: BLE001 - 净化失败就退回字符串
            pass
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        try:
            return _plain(tolist())
        except Exception:  # noqa: BLE001
            pass
    return str(obj)


# --------------------------------------------------------------------------- #
# ToolResult
# --------------------------------------------------------------------------- #
@dataclass
class ToolResult:
    """统一返回结构。永远用 `.to_dict()` 交给 Agent / json.dumps。"""

    status: TaskStatus | str
    mode: RunMode | str
    message: str = ""
    task_id: str | None = None
    data: dict[str, typing.Any] = field(default_factory=dict)
    error_code: ErrorCode | str | None = None

    # ---- 构造后自检：把协议违反变成异常，而不是让 Agent 在运行时瞎猜 ----
    def __post_init__(self) -> None:
        self.status = TaskStatus(self.status)
        self.mode = RunMode(self.mode)
        if self.error_code is not None:
            self.error_code = ErrorCode(self.error_code)
        if self.data is None:
            self.data = {}
        if not isinstance(self.data, dict):
            raise TypeError(f"ToolResult.data 必须是 dict，收到 {type(self.data)!r}")

        if self.status in _REQUIRE_TASK_ID and not self.task_id:
            raise ValueError(f"status={self.status.value} 必须带非空 task_id")
        if self.status in _REQUIRE_ERROR_CODE and self.error_code is None:
            raise ValueError(f"status={self.status.value} 必须带 error_code")

    # ---- 序列化 ----
    def to_dict(self) -> dict[str, typing.Any]:
        """返回 6 个固定键的 JSON 友好 dict（顺序与协议一致）。"""
        return {
            "status": self.status.value,
            "mode": self.mode.value,
            "message": self.message,
            "task_id": self.task_id,
            "data": _plain(self.data),
            "error_code": None if self.error_code is None else self.error_code.value,
        }

    @classmethod
    def from_dict(cls, payload: typing.Mapping[str, typing.Any]) -> "ToolResult":
        return cls(
            status=payload["status"],
            mode=payload.get("mode", RunMode.MOCK),
            message=payload.get("message", ""),
            task_id=payload.get("task_id"),
            data=dict(payload.get("data") or {}),
            error_code=payload.get("error_code"),
        )

    # ---- 便捷判定 ----
    @property
    def succeeded(self) -> bool:
        return self.status is TaskStatus.SUCCEEDED

    def __getitem__(self, key: str) -> typing.Any:
        """允许像 dict 一样读，方便 Agent 代码里 result["status"] 的写法。"""
        return self.to_dict()[key]


def to_json(result: ToolResult | typing.Mapping[str, typing.Any], indent: int | None = None) -> str:
    """紧凑 JSON，用于喂给大模型（ensure_ascii=False 保证中文可读）。"""
    payload = result.to_dict() if isinstance(result, ToolResult) else result
    return json.dumps(payload, ensure_ascii=False, indent=indent)


# --------------------------------------------------------------------------- #
# ToolError：工具内部抛这个，由装饰器统一转成 ToolResult
# --------------------------------------------------------------------------- #
class ToolError(Exception):
    """带 error_code 的业务异常。

    用法：
        raise ToolError(ErrorCode.INVALID_ARGUMENT, f"未注册的相机 {cam!r}")
    """

    def __init__(
        self,
        code: ErrorCode | str,
        message: str,
        data: dict[str, typing.Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = ErrorCode(code)
        self.message = message
        self.data = data or {}

    def to_result(self, mode: RunMode | str) -> ToolResult:
        # 重试类错误用 failed；输入/能力类错误用 rejected 更符合协议 §2
        status = (
            TaskStatus.REJECTED
            if self.code
            in {
                ErrorCode.INVALID_ARGUMENT,
                ErrorCode.NOT_SUPPORTED,
                ErrorCode.PERMISSION_DENIED,
                ErrorCode.SAFETY_LIMIT,
            }
            else TaskStatus.FAILED
        )
        return ToolResult(
            status=status,
            mode=mode,
            message=self.message,
            data=self.data,
            error_code=self.code,
        )


# --------------------------------------------------------------------------- #
# 工厂函数：让各 Tool 的返回代码短、且不可能漏字段
# --------------------------------------------------------------------------- #
def ok(
    message: str,
    mode: RunMode | str,
    data: dict[str, typing.Any] | None = None,
    task_id: str | None = None,
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.SUCCEEDED,
        mode=mode,
        message=message,
        task_id=task_id,
        data=data or {},
    )


def running(
    message: str, mode: RunMode | str, task_id: str, data: dict[str, typing.Any] | None = None
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.RUNNING, mode=mode, message=message, task_id=task_id, data=data or {}
    )


def failed(
    code: ErrorCode | str,
    message: str,
    mode: RunMode | str,
    data: dict[str, typing.Any] | None = None,
    task_id: str | None = None,
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.FAILED,
        mode=mode,
        message=message,
        task_id=task_id,
        data=data or {},
        error_code=code,
    )


def rejected(
    code: ErrorCode | str,
    message: str,
    mode: RunMode | str,
    data: dict[str, typing.Any] | None = None,
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.REJECTED, mode=mode, message=message, data=data or {}, error_code=code
    )


def canceled(
    message: str, mode: RunMode | str, task_id: str | None = None,
    data: dict[str, typing.Any] | None = None,
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.CANCELLED, mode=mode, message=message, task_id=task_id, data=data or {}
    )


def unknown(
    code: ErrorCode | str,
    message: str,
    mode: RunMode | str,
    data: dict[str, typing.Any] | None = None,
) -> ToolResult:
    return ToolResult(
        status=TaskStatus.UNKNOWN, mode=mode, message=message, data=data or {}, error_code=code
    )
