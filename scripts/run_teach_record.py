#!/usr/bin/env python3
"""示教采点：把手臂摆到位，按回车记录当前位姿/关节角/夹爪宽度。

**全程只读**，一条运动指令都不发。手臂靠人拖/手柄摆到位。

用法（HPU 上，需要真终端）：
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/run_teach_record.py

默认采 5 个点，顺序固定：
    P1_init       初始位：手臂竖直下垂、夹爪张开（每次动作序列的归零点）
    P2_grasp      咬住抽屉把手的位姿（夹爪贴着把手，还没闭爪）
    P3_pull_done  抽屉拉到位的末端位姿（只作校验参照，不作回放目标）
    P3_dive_ref   盲下探起点（28cm 之旅的上端）
    P4_drop       垃圾桶上方的投放点

采完会覆盖 scenes/scene_medicine_check/pipeline/configs/teach_drawer.json，
旧文件自动备份为 .bak。

⚠️ 摆位注意事项：
  1. P1_init 必须是"每次都能安全回到的姿态"，不要贴着桌面或箱口。
  2. P2_grasp 摆的是"夹爪已经套在把手上、但还没用力"的姿态 ——
     这一点的关节角会被原样回放，回放后再闭爪，所以不能摆成已经夹紧的样子。
  3. P3_dive_ref 是下探起点。它往下 28cm 必须仍在桌面上方，
     否则 drawer_dive 的安全闸门会直接拒绝（这是保护，不是 bug）。
"""

from __future__ import annotations

import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logging.basicConfig(level=logging.INFO,
                    format="%(levelname)s %(name)s: %(message)s")


def main() -> int:
    from scenes.scene_medicine_check.pipeline import tuning
    from scenes.scene_medicine_check.pipeline.contracts import Context
    from scenes.scene_medicine_check.pipeline.io import build_io
    from scenes.scene_medicine_check.pipeline.stages.s7_teach_record import TeachRecord

    if not sys.stdin.isatty():
        print("✗ 这个脚本必须在真终端里跑（需要按回车交互）。")
        print("  通过 ssh 直接登录 HPU 终端执行，不要用 nohup / 管道重定向。")
        return 2

    print("=" * 72)
    print("抽屉场景示教采点（全程只读，不发运动指令）")
    print("=" * 72)
    print("将依次采集：P1_init / P2_grasp / P3_pull_done / P3_dive_ref / P4_drop")
    print("每个点：把手臂摆到位 → 按回车记录；s=跳过；q=中止")
    print("=" * 72)

    io = build_io("wrapper")
    params = {
        "chain": "right_arm",
        "gripper_side": tuning.GRIPPER_SIDE,
        "out": "pipeline/configs/teach_drawer.json",
        "points": ["P1_init", "P2_grasp", "P3_pull_done", "P3_dive_ref", "P4_drop"],
        "notes": {
            "P1_init": "初始位：手臂竖直下垂、夹爪张开",
            "P2_grasp": "咬住抽屉把手（未闭爪）",
            "P3_pull_done": "抽屉拉到位（仅作校验参照）",
            "P3_dive_ref": "盲下探起点（28cm 上端）",
            "P4_drop": "垃圾桶上方投放点",
        },
        "interactive": True,
    }

    res = TeachRecord(params).run(Context(params=params), io)
    print()
    print(f"结果：{res.status.value} —— {res.message}")
    if res.data:
        print(f"明细：{res.data}")

    missing = io.missing() if hasattr(io, "missing") else []
    if missing:
        print(f"\n⚠️ 缺失接口：{missing}")
        print("   先跑 scripts/probe_readonly_apis.py 定位问题，别直接上流程。")

    return 0 if res.ok else 1


if __name__ == "__main__":
    sys.exit(main())
