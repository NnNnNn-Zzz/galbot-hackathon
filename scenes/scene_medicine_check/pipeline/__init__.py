"""场景管线：可插拔动作块框架。

对外只暴露三样东西：run_pipeline、build_io、契约类型。
stage 实现放在 stages/（内置基线）与 policies/（队友升级版），
两者用同一套接口，runner 不区分来源。
"""

from __future__ import annotations

from typing import Any, Dict

from .contracts import (  # noqa: F401 - 对外 re-export
    Code,
    Context,
    Stage,
    StageResult,
    Status,
    failed,
    ok,
    unknown,
)


def code_of(result: Dict[str, Any]) -> Code:
    """把 io 返回里的字符串 code 收敛成枚举，缺省按 STAGE_FAILED。"""
    raw = (result or {}).get("code")
    if not raw:
        return Code.STAGE_FAILED
    try:
        return Code(raw)
    except ValueError:
        return Code.STAGE_FAILED


def outcome(result: Dict[str, Any], err_prefix: str, **data: Any) -> StageResult:
    """把 io 的失败返回翻译成 failed 或 **unknown**。

    区别对待是本项目的核心语义：
        UNDETERMINED（读数矛盾、视觉说不清）→ unknown：不猜、也不再发任何动作；
        其余（含 TIMEOUT）→ failed：仍允许归位。

    TIMEOUT 归到 failed 是刻意的取舍：超时只意味着"这条指令没拿到回执"，
    但里程计通常还在，可以闭环回原位；而"位姿与视觉互相矛盾"意味着连自己在哪都不知道，
    此时再发归位运动可能撞东西，所以必须走 unknown。
    """
    code = code_of(result)
    msg = result.get("message") or ""
    text = f"{err_prefix}：{msg}" if msg else err_prefix
    if code is Code.UNDETERMINED:
        return unknown(text, code=code, **data)
    return failed(text, code=code, **data)


__all__ = [
    "Code", "Context", "Stage", "StageResult", "Status",
    "ok", "failed", "unknown", "code_of", "outcome",
]
