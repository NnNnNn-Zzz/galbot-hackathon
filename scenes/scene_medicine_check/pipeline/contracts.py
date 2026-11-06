"""管线冻结契约：StageResult / Context / Stage。

改动规则（协商档，见 docs/plugin_guide.md）：
  1. 只允许**追加可选字段**，不得修改或删除已有字段的语义；
  2. 不得修改 Stage.run(ctx, io) 的签名；
  3. 三个状态语义固定，任何新增实现都必须遵守：
       ok      —— 本步目标达成，继续下一步
       failed  —— 已知失败（依赖缺失、动作被拒），归位后终止
       unknown —— **不可判定**（读数矛盾、视觉说不清、超时无回执），
                  立即终止，不再发出任何动作（包括归位导航）

unknown 与 failed 的区别是本项目最重要的一条：
  failed 是"知道失败了"，可以安全地执行归位；
  unknown 是"不知道现在什么状态"，此时再发动作可能撞东西，所以只能停。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class Status(str, Enum):
    """步骤终态。"""

    OK = "ok"
    FAILED = "failed"
    UNKNOWN = "unknown"


class Code(str, Enum):
    """失败/不可判定原因码，写进 trace 便于复盘与统计。"""

    OK = "OK"
    STAGE_FAILED = "STAGE_FAILED"        # 动作返回失败
    DEP_MISSING = "DEP_MISSING"          # 封装层接口未实现（软降级触发）
    TIMEOUT = "TIMEOUT"                  # 超时无回执
    NO_TARGET = "NO_TARGET"              # 没有符合条件的目标
    NOT_VERIFIED = "NOT_VERIFIED"        # 动作成功但缺少确证手段
    UNDETERMINED = "UNDETERMINED"        # 不可判定（unknown 默认码）
    SAFETY_ABORT = "SAFETY_ABORT"        # 安全闸门触发


@dataclass
class StageResult:
    """单个 stage 的返回值。禁止用异常代替它表达业务失败。"""

    status: Status = Status.OK
    message: str = ""
    code: Code = Code.OK
    data: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # 允许传字符串，统一收敛成枚举，避免调用方比较时踩类型坑
        self.status = Status(self.status)
        self.code = Code(self.code) if not isinstance(self.code, Code) else self.code

    @property
    def ok(self) -> bool:
        return self.status is Status.OK

    @property
    def unknown(self) -> bool:
        return self.status is Status.UNKNOWN


def ok(message: str = "", **data: Any) -> StageResult:
    """本步达成。"""
    return StageResult(status=Status.OK, message=message, code=Code.OK, data=dict(data))


def failed(message: str, code: Code = Code.STAGE_FAILED, **data: Any) -> StageResult:
    """已知失败。"""
    return StageResult(status=Status.FAILED, message=message, code=code, data=dict(data))


def unknown(message: str, code: Code = Code.UNDETERMINED, **data: Any) -> StageResult:
    """不可判定。调用方必须立即终止，不得再发动作。"""
    return StageResult(status=Status.UNKNOWN, message=message, code=code, data=dict(data))


@dataclass
class Context:
    """跨 stage 传递的唯一数据载体。

    字段分工（谁是产出者、谁是消费者，写死在这里，方便队友对接）：
      params      —— 场景入口传入的参数，只读
      objects     —— 感知结果，preflight/perception 产出，select/grasp 消费
      expiry      —— {obj_id: "YYYY-MM-DD" | None}，读日期 stage 产出
      verdict     —— {obj_id: "expired"|"valid"|"unknown"}，判定 stage 产出
      target      —— 选中的目标对象（dict，与 objects 中元素同构），select 产出
      picked      —— 夹取是否成功；None 表示无确证手段（禁止写成 True 糊过去）
      start_odom  —— 起始里程计位姿，preflight 产出，return_home 消费
      flags       —— 各 stage 的私有暂存区，键名请带自身前缀（如 "expiry.ocr_ms"）
      trace       —— 执行轨迹，由 runner 写入，stage 不要动
    """

    params: Dict[str, Any] = field(default_factory=dict)
    objects: List[Dict[str, Any]] = field(default_factory=list)
    expiry: Dict[str, Optional[str]] = field(default_factory=dict)
    verdict: Dict[str, str] = field(default_factory=dict)
    target: Optional[Dict[str, Any]] = None
    picked: Optional[bool] = None
    start_odom: Dict[str, Any] = field(default_factory=dict)
    flags: Dict[str, Any] = field(default_factory=dict)
    trace: List[Dict[str, Any]] = field(default_factory=list)
    stopped_reason: Optional[str] = None


class Stage(ABC):
    """一个可插拔动作块。

    实现约定：
      - 只通过 io 访问硬件（io 是白名单接口，见 pipeline/io.py），
        禁止 import galbot_sdk、禁止 import sdk_wrapper、禁止直连 adapters；
      - 不自己 sleep 等硬件，超时由 io 层负责；
      - 不吞异常：预期内的失败用 failed() 返回，预期外让异常冒泡给 runner。
    """

    #: 注册名，由 @register_stage 写入，不要手写
    stage_name: str = ""
    #: 人类可读的步骤标识，写进 trace
    stage_id: str = "unnamed"

    def __init__(self, params: Optional[Dict[str, Any]] = None) -> None:
        self.params: Dict[str, Any] = dict(params or {})

    def describe(self) -> str:
        """给 plan()/dry_run 用的一行说明。"""
        return self.stage_id

    @abstractmethod
    def run(self, ctx: Context, io: Any) -> StageResult:
        """执行本步。返回 StageResult，不要返回裸 dict。"""
        raise NotImplementedError
