#!/usr/bin/env python3
"""实验 1：空桌盲抓 —— 只跑 preflight + init_pose + grasp，桌面**不放任何盒子**。

目的
----
验证抓取轨迹本身是安全的：手臂从抱持位伸出去、下到 z=0.77、夹爪开合、
抬起到 z=0.90、再收回来，全程不碰桌子、不碰底座、不触发急停。
桌上没盒，所以夹到空气是**预期结果**，不是失败。

为什么先做这个
--------------
如果连空桌都会撞，那一定是轨迹/坐标系/高度标定错了，与「盒子摆得准不准」
无关。先把这个变量排除掉，后面调盒子位置才有意义。

通过判据（详见 experiments/criteria.py）
---------------------------------------
1. preflight、init_pose 与 grasp 三步都必须是 ok（不能是 unknown、不能被 skipped）
2. 允许 picked 为 False 或 None（无夹爪反馈是本后端正常表现）
3. 若 picked=True -> 视为异常（桌上还有东西 / 假阳性），需重查

用法
----
真机（在 HPU 上，先 source scripts/env_hpu.sh）：
    MED_IO=wrapper python3 scripts/run_exp1_grasp_only.py

mock 干跑（不碰机器人）：
    MED_IO=mock python3 scripts/run_exp1_grasp_only.py

退出码：0 = 通过；1 = 不通过；2 = 启动前置检查未过（未发任何动作）
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.experiments import criteria          # noqa: E402
from scenes.scene_medicine_check.pipeline import runner as pipeline_runner  # noqa: E402
from scenes.scene_medicine_check.pipeline import tuning               # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context    # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io          # noqa: E402


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper")

    _say({"step": "boot", "experiment": 1,
          "title": criteria.EXPERIMENT_TITLES[1],
          "backend": backend,
          "grasp_pose": tuning.GRASP_POSE,
          "note": "桌面必须清空。看到手臂伸向下述坐标即为正确方向。"})

    cfg = pipeline_runner.load_config("medicine.baseline")
    keep = set(criteria.GRASP_ONLY_STEPS)
    cfg["steps"] = [s for s in cfg["steps"] if s["id"] in keep]
    _say({"step": "config", "stages": [s["id"] for s in cfg["steps"]]})
    if len(cfg["steps"]) != len(criteria.GRASP_ONLY_STEPS):
        _say({"step": "abort",
              "message": f"配置里没找齐 {sorted(keep)}，实际只拿到 "
                         f"{[s['id'] for s in cfg['steps']]}",
              "action": "检查 configs/medicine.baseline.json 的步骤 id 是否被改过"})
        return 2

    ctx = Context(params={})
    # 空桌实验的关键一步：mock 后端默认自带三盒，必须显式传空列表，
    # 否则 mock 会「抓到」假盒，让本实验的判据失去意义。真机忽略此参数。
    if backend.strip().lower() == "mock":
        io = build_io(backend, objects=[])
        _say({"step": "mock_table", "objects": 0,
              "note": "mock 已模拟空桌（objects=[]），与真机空桌一致"})
    else:
        io = build_io(backend)

    result = pipeline_runner.run_pipeline(ctx, io, cfg)

    # 逐步打印
    for entry in result.get("trace") or []:
        _say({"stage": entry.get("step"),
              "fate": entry.get("status"),
              "code": entry.get("code"),
              "elapsed_s": entry.get("elapsed_s"),
              "message": str(entry.get("message", ""))[:160]})

    # 把 ctx.picked 补进 result，供判据使用
    # （run_pipeline 的返回值不含 picked，这是它和 Context 的边界）
    result["picked"] = getattr(ctx, "picked", None)

    verdict = criteria.judge_grasp_only(result, experiment=1)
    _say({"step": "verdict",
          "pipeline_status": result.get("status"),
          "passed": verdict["passed"],
          "link_ok": verdict["link_ok"],
          "grabbed": verdict["grabbed"],
          "counts": result.get("counts"),
          "io_missing": result.get("io_missing"),
          "reasons": verdict["reasons"],
          "next_action": verdict["next_action"]})

    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
