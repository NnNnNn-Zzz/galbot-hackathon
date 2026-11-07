#!/usr/bin/env python3
"""VLM 校验版抓取的专项回归：验证「校验门」在各异常分支下的行为。

覆盖 7 项：
  1. 正常通过             → grasp ok，下爪
  2. 模型未配置           → unknown，停爪不抓（绝不假通过）
  3. 夹爪下无盒           → failed + NO_TARGET，不抓空
  4. 偏移建议             → 按限幅微调后通过，累计修正量被记录
  5. 一路不通过           → 达轮数上限 failed + SAFETY_ABORT，不下爪
  6. 修正量离谱（50cm）    → 被限幅到 3cm，或超累计上限直接拒绝
  7. 置信度过低           → graspable 强制 False，进修正分支

用法：python3 scenes/scene_medicine_check/tests/verify_vlm_grasp.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

from scenes.scene_medicine_check.pipeline import runner as R  # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import (  # noqa: E402
    Code, Context, Status,
)
from scenes.scene_medicine_check.pipeline.io import build_io  # noqa: E402
from scenes.scene_medicine_check.pipeline.perception import vlm_check  # noqa: E402

RESULTS = []


def _run_once(verdict_or_backend, max_rounds: int = 3):
    """跑 grasp_vlm（只取 preflight+grasp 两步），返回 (stage_result, ctx, io)。"""
    cfg = R.load_config("medicine.vlm")
    cfg["steps"] = cfg["steps"][:2]
    cfg["steps"][1]["params"]["max_rounds"] = max_rounds
    # 用 mock IO + 指定 backend
    io = build_io("mock")
    backend = (verdict_or_backend if hasattr(verdict_or_backend, "judge")
               else vlm_check.StaticVLM(verdict_or_backend))
    io._vlm_override = backend  # 供 policy 的 _backend() 读取
    ctx = Context(params={})
    res = R.run_pipeline(ctx, io, cfg)
    grasp = [t for t in ctx.trace if t.get("step") == "grasp"]
    return (grasp[0] if grasp else {}), ctx, io


def check(name: str, cond: bool, detail: str) -> None:
    RESULTS.append((name, cond, detail))
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  —— {detail}")


def v(**kw) -> vlm_check.VLMVerdict:
    base = dict(ok=True, message="", box_present=True, graspable=True,
                offset_x_cm=0.0, offset_y_cm=0.0, confidence=0.9, reason="test")
    base.update(kw)
    return vlm_check.VLMVerdict(**base)


def main() -> int:
    # ---- 1 正常通过
    t, ctx, _ = _run_once(v())
    check("1. 校验通过 → 下爪抓取",
          t.get("status") == "ok" and ctx.picked is True,
          f"status={t.get('status')} picked={ctx.picked}")

    # ---- 2 模型未配置（用真 OpenAI 后端，无 key）
    os.environ.pop("VLM_API_KEY", None)
    os.environ.pop("VLM_BASE_URL", None)
    t, ctx, _ = _run_once(vlm_check.OpenAICompatibleVLM())
    check("2. 模型未配置 → unknown 停爪",
          t.get("status") == Status.UNKNOWN.value,
          f"status={t.get('status')} msg={str(t.get('message'))[:60]}")

    # ---- 3 夹爪下没有盒子
    t, ctx, io = _run_once(v(box_present=False, graspable=False, reason="画面无药盒"))
    check("3. 无盒 → failed + NO_TARGET，不抓空",
          t.get("status") == "failed" and t.get("code") == Code.NO_TARGET.value,
          f"status={t.get('status')} code={t.get('code')}")

    # ---- 4 有偏移，建议 2cm，修正后通过（第二轮给 0）
    class SequenceVLM:
        def __init__(self):
            self.n = 0

        def judge(self, image_path, prompt=""):
            self.n += 1
            if self.n == 1:
                return v(graspable=False, offset_x_cm=2.0, offset_y_cm=-1.0,
                         reason="偏右后")
            return v(reason="已对准")

    t, ctx, _ = _run_once(SequenceVLM())
    fix = ctx.flags.get("grasp.vlm_correction_m")
    check("4. 按建议微调后通过",
          t.get("status") == "ok" and fix and abs(fix - 0.02) < 1e-6,
          f"status={t.get('status')} 累计修正={fix}m")

    # ---- 5 一直不通过，达上限
    t, ctx, _ = _run_once(v(graspable=False, offset_x_cm=1.0, reason="总是偏"),
                          max_rounds=3)
    check("5. 3 轮不过 → SAFETY_ABORT 停止下爪",
          t.get("status") == "failed" and t.get("code") == Code.SAFETY_ABORT.value,
          f"status={t.get('status')} code={t.get('code')} "
          f"修正={ctx.flags.get('grasp.vlm_correction_m', 0)}m")

    # ---- 6 离谱修正量 50cm → 限幅到 3cm
    dx, dy, note = vlm_check.clamp_correction(0.50, -0.50, 0.0)
    check("6. 50cm 离谱建议被限幅到 3cm",
          abs(dx - 0.03) < 1e-9 and abs(dy + 0.03) < 1e-9 and "限幅" in note,
          f"dx={dx} dy={dy} note={note[:40]}")
    # 6b 累计超限则整条拒绝
    dx2, dy2, note2 = vlm_check.clamp_correction(0.02, 0.0, 0.05)
    check("6b 累计超 6cm → 修正被拒绝",
          dx2 == 0.0 and dy2 == 0.0 and "拒绝" in note2,
          f"note={note2[:40]}")

    # ---- 7 置信度低 → graspable 强制 False
    pv = vlm_check.parse_verdict(
        '{"box_present": true, "graspable": true, "offset_x_cm": 0, '
        '"offset_y_cm": 0, "confidence": 0.2, "reason": "不太确定"}')
    check("7. 置信度 0.2 → graspable 强制 False",
          pv.ok and pv.graspable is False and pv.confidence < 0.6,
          f"graspable={pv.graspable} conf={pv.confidence} msg={pv.message}")

    # ---- 附加：解析鲁棒性
    pv2 = vlm_check.parse_verdict('```json\n{"box_present":true,"graspable":true,'
                                 '"confidence":0.9,"reason":"ok"}\n```')
    check("8. 代码块包裹的 JSON 能解析", pv2.ok and pv2.box_present,
          f"ok={pv2.ok}")
    pv3 = vlm_check.parse_verdict("抱歉我无法回答")
    check("9. 非 JSON 返回 → ok=False", pv3.ok is False, f"msg={pv3.message}")

    print("=" * 72)
    passed = sum(1 for _, c, _ in RESULTS if c)
    print(f"合计 {len(RESULTS)} 项，通过 {passed} 项，失败 {len(RESULTS) - passed} 项")
    print("=" * 72)
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
