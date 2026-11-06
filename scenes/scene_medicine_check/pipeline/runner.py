"""唯一的控制流。所有 stage 由配置决定顺序，本文件不认识任何具体动作。

三种终态的处理（写死在这里，任何插进来的 stage 都绕不过去）：
  ok      → 继续
  failed  → 停止后续普通步骤，但仍执行 must_run 步骤（归位），最后整体报失败
  unknown → 立即全停，连 must_run 也不执行，只允许硬件级安全停止在场景 finally 里做

为什么 unknown 连归位都不发：归位本身是一次底盘/手臂运动，只有"知道自己在哪"才安全；
unknown 的定义就是"不知道"，此时发运动指令可能撞到人或其他设备。
"""

from __future__ import annotations

import json
import logging
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .contracts import Code, Context, Stage, StageResult, Status, failed
from .registry import get_stage, load_all

logger = logging.getLogger("scene.medicine.runner")

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs")


@dataclass
class StepSpec:
    """配置里的一步。"""

    id: str
    stage: str
    params: Dict[str, Any] = field(default_factory=dict)
    #: 失败后仍然要执行（归位类步骤）
    must_run: bool = False
    #: 失败了只记录、不中止（如"读不到里程计"）
    non_fatal: bool = False


def load_config(name_or_path: str) -> Dict[str, Any]:
    """读管线配置。可以给配置名（medicine.baseline）或绝对路径。"""
    path = name_or_path
    if not path.endswith(".json"):
        path = os.path.join(_CONFIG_DIR, name_or_path + ".json")
    if not os.path.isabs(path):
        path = os.path.join(os.getcwd(), path)
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到管线配置：{path}")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = json.load(fh)
    for key in ("name", "steps"):
        if key not in cfg:
            raise ValueError(f"管线配置 {path} 缺字段 '{key}'")
    return cfg


def build_steps(cfg: Dict[str, Any]) -> List[StepSpec]:
    steps = []
    for i, raw in enumerate(cfg["steps"]):
        if "id" not in raw or "stage" not in raw:
            raise ValueError(f"第 {i} 步缺 id/stage：{raw}")
        steps.append(StepSpec(
            id=str(raw["id"]),
            stage=str(raw["stage"]),
            params=dict(raw.get("params") or {}),
            must_run=bool(raw.get("must_run", False)),
            non_fatal=bool(raw.get("non_fatal", False)),
        ))
    return steps


def describe(cfg: Dict[str, Any]) -> List[Tuple[str, str, str]]:
    """dry_run 用：列出 (步骤 id, 说明, stage 实现)。"""
    load_all()
    out = []
    for spec in build_steps(cfg):
        try:
            cls = get_stage(spec.stage)
            text = cls(spec.params).describe()
            impl = f"{cls.__module__}.{cls.__name__}"
        except KeyError:
            text = "（未注册的实现）"
            impl = f"缺失：{spec.stage}"
        out.append((spec.id, text, impl))
    return out


def _normalize(res: Any, stage_label: str) -> StageResult:
    """stage 返回值兜底。返回裸 dict 属于越界，这里容忍但记警告，不当成功。"""
    if isinstance(res, StageResult):
        return res
    logger.error("stage %s 返回了 %s，不是 StageResult —— 按 failed 处理", stage_label, type(res).__name__)
    return failed(f"stage {stage_label} 返回值类型非法：{type(res).__name__}")


def run_pipeline(ctx: Context, io: Any, cfg: Dict[str, Any]) -> Dict[str, Any]:
    """执行整条管线，返回结构化结果（不含面向用户的中文措辞）。"""
    load_all()
    steps = build_steps(cfg)
    failure: Optional[Tuple[StepSpec, StageResult]] = None
    aborted = False
    counts = {"ok": 0, "failed": 0, "unknown": 0, "skipped": 0, "err": 0}

    for spec in steps:
        if aborted or (failure is not None and not spec.must_run):
            reason = "已中止（unknown）" if aborted else f"前序步骤 {failure[0].id} 失败"
            ctx.trace.append({"step": spec.id, "stage": spec.stage, "status": "skipped",
                              "code": "SKIPPED", "message": reason, "elapsed_s": 0.0})
            counts["skipped"] += 1
            logger.warning("跳过步骤 %s（%s）", spec.id, reason)
            continue

        try:
            stage: Stage = get_stage(spec.stage)(spec.params)
        except KeyError as exc:
            res = failed(str(exc), code=Code.STAGE_FAILED)
            ctx.trace.append({"step": spec.id, "stage": spec.stage, "status": res.status.value,
                              "code": res.code.value, "message": res.message, "elapsed_s": 0.0})
            counts["failed"] += 1
            if failure is None and not spec.non_fatal:
                failure = (spec, res)
            continue

        logger.info("===== 步骤 %s（%s）=====", spec.id, spec.stage)
        t0 = time.time()
        try:
            res = _normalize(stage.run(ctx, io), spec.stage)
        except Exception as exc:  # noqa: BLE001 - 单步异常不能拖垮整场，但要如实记
            logger.exception("步骤 %s 抛异常", spec.id)
            res = failed(f"步骤 {spec.id} 抛异常：{exc}", code=Code.STAGE_FAILED)
            counts["err"] += 1
        elapsed = round(time.time() - t0, 2)

        entry = {"step": spec.id, "stage": spec.stage, "status": res.status.value,
                 "code": res.code.value, "message": res.message, "elapsed_s": elapsed}
        if res.data:
            entry["data"] = res.data
        ctx.trace.append(entry)

        if res.status is Status.OK:
            counts["ok"] += 1
            continue
        if res.status is Status.UNKNOWN:
            counts["unknown"] += 1
            if failure is None:
                failure = (spec, res)
            aborted = True
            ctx.stopped_reason = f"步骤 {spec.id} 不可判定：{res.message}"
            logger.error("步骤 %s 不可判定：%s —— 立即全停，不再发任何动作", spec.id, res.message)
            continue
        counts["failed"] += 1
        if spec.non_fatal:
            logger.warning("步骤 %s 失败但属非致命（%s），继续", spec.id, res.message)
            continue
        if failure is None:
            failure = (spec, res)
        logger.error("步骤 %s 失败：%s —— 停止后续普通步骤，仍执行归位", spec.id, res.message)

    overall = "ok"
    if failure is not None:
        overall = "unknown" if failure[1].status is Status.UNKNOWN else "failed"

    return {
        "status": overall,
        "failed_step": failure[0].id if failure else None,
        "failed_stage": failure[1].message if failure else None,
        "failed_code": failure[1].code.value if failure else None,
        "trace": ctx.trace,
        "counts": counts,
        "aborted_without_safety": aborted,
        "io_missing": list(getattr(io, "missing", lambda: [])()),
    }
