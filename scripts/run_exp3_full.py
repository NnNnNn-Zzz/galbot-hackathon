#!/usr/bin/env python3
"""实验 3：全流程低速 —— 7 步完整链路，一人手持急停。

目的
----
把 s0 -> s6 全部串起来跑一遍，验证的不只是「能不能抓」，还有：
  * 抓着盒子左移 0.5 m 时盒子会不会掉
  * 右转 30 度后投放位姿是否还对得上桶
  * 中途任何一步 failed，must_run 的归位是否真的被执行

**本脚本有一道安全闸**
----------------------
tuning.py 里 PRE_DROP_POSE / DROP_POSE 目前是照旧流程抄来的占位值
（注释标着 TODO(标定)）。左移 + 右转之后桶的真实位置没人验证过。
如果直接跑，手臂会朝一个未经验证的坐标伸手，可能撞桶壁或桌沿。

所以脚本启动时先做占位值检测：只要还是占位值，就**拒绝发任何动作**并退出。
放行条件只有一个 —— 你已经用 teach_drop_pose.py 示教并填进了真实值。

流程
----
    0. 占位值闸门检查（不过则退出，什么都不动）
    1. 打印逐步计划，等你按回车确认（真机模式下）
    2. 跑完整 7 步
    3. 按 criteria.judge_full_pipeline 判定

用法
----
    # 真机（HPU 上，先 source scripts/env_hpu.sh）
    MED_IO=wrapper python3 scripts/run_exp3_full.py

    # 只想看闸门会不会拦住（预期：被拦住）
    MED_IO=mock python3 scripts/run_exp3_full.py

    # 确认已经示教过、要跳过交互确认（不推荐）
    MED_IO=wrapper python3 scripts/run_exp3_full.py --yes

退出码：0 = 通过；1 = 不通过；2 = 闸门/前置检查未过（未发任何动作）
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.experiments import criteria          # noqa: E402
from scenes.scene_medicine_check.experiments import placeholder_guard  # noqa: E402
from scenes.scene_medicine_check.pipeline import runner as pipeline_runner  # noqa: E402
from scenes.scene_medicine_check.pipeline import tuning               # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context    # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io          # noqa: E402


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="实验 3：全流程低速验证")
    ap.add_argument("--yes", action="store_true",
                    help="跳过开跑前的交互确认（真机上不建议）")
    args = ap.parse_args()

    backend = os.environ.get("MED_IO", "wrapper")

    cfg = pipeline_runner.load_config("medicine.baseline")
    plan = [(s["id"], s["stage"], bool(s.get("must_run", False))) for s in cfg["steps"]]
    _say({"step": "boot", "experiment": 3,
          "title": criteria.EXPERIMENT_TITLES[3],
          "backend": backend,
          "plan": [{"id": i, "stage": s, "must_run": m} for i, s, m in plan]})

    # ---------------------------------------------------------------- 安全闸
    gate = placeholder_guard.check_drop_poses(tuning.PRE_DROP_POSE, tuning.DROP_POSE)
    _say({"step": "safety_gate",
          "ok": gate["ok"],
          "blockers": gate["blockers"],
          "warnings": gate["warnings"],
          "checked": {"PRE_DROP_POSE": tuning.PRE_DROP_POSE,
                      "DROP_POSE": tuning.DROP_POSE}})

    if not gate["ok"]:
        _say({"step": "abort",
              "message": "投放位姿未标定，拒绝执行 —— 未向机器人发送任何动作。",
              "how_to_unblock": [
                  "1) 在 HPU 上手动把右臂摆到桶口正上方约 5cm（夹爪朝下）",
                  "2) python3 scripts/teach_drop_pose.py            # 读 PRE_DROP",
                  "3) 手动降到盒子刚好接触桶口的高度",
                  "4) python3 scripts/teach_drop_pose.py --tag drop # 读 DROP",
                  "5) 把两次打印的值填进 tuning.py 对应的两行，去掉 TODO(标定) 注释",
                  "6) 重跑本脚本，闸门会自动放行",
              ]})
        return 2

    if gate["warnings"]:
        _say({"step": "safety_gate_warnings", "warnings": gate["warnings"],
              "note": "这些不阻断执行，但请确认是有意的"})

    # ------------------------------------------------------ 真机交互确认
    is_mock = backend.strip().lower() == "mock"
    if not is_mock and not args.yes:
        print("\n即将执行完整 7 步。请确认：", flush=True)
        print("  [ ] 急停按钮在手边，且知道怎么拍", flush=True)
        print("  [ ] 桌面只放一盒，位置已按实验 2 的胶带摆好", flush=True)
        print("  [ ] 桶已放在示教时那个位置（不要挪动）", flush=True)
        print("  [ ] 周围无人站着（底盘要左移 0.5m + 右转 30°）", flush=True)
        try:
            input("\n全部确认后按回车开跑（Ctrl+C 取消）：")
        except (EOFError, KeyboardInterrupt):
            _say({"step": "abort", "message": "用户取消，未发任何动作"})
            return 2

    # ---------------------------------------------------------------- 执行
    ctx = Context(params={})
    if is_mock:
        # mock 需要用「已标定」的投放位姿，否则跑不到 drop 段
        # 这里临时覆盖参数，不写回 tuning.py
        for s in cfg["steps"]:
            if s["id"] == "drop":
                s.setdefault("params", {})
                s["params"]["pre_drop_pose"] = tuning.PRE_DROP_POSE
                s["params"]["drop_pose"] = tuning.DROP_POSE
        io = build_io(backend)
    else:
        io = build_io(backend)

    result = pipeline_runner.run_pipeline(ctx, io, cfg)
    result["picked"] = getattr(ctx, "picked", None)

    for entry in result.get("trace") or []:
        _say({"stage": entry.get("step"),
              "fate": entry.get("status"),
              "code": entry.get("code"),
              "elapsed_s": entry.get("elapsed_s"),
              "message": str(entry.get("message", ""))[:160]})

    verdict = criteria.judge_full_pipeline(result)
    _say({"step": "verdict",
          "pipeline_status": result.get("status"),
          "failed_step": result.get("failed_step"),
          "passed": verdict["passed"],
          "link_ok": verdict["link_ok"],
          "grabbed": verdict["grabbed"],
          "home_executed": verdict["home_executed"],
          "counts": result.get("counts"),
          "io_missing": result.get("io_missing"),
          "reasons": verdict["reasons"],
          "next_action": verdict["next_action"]})

    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
