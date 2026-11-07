#!/usr/bin/env python3
"""阶段 A3：只走 approach（PRE_GRASP → GRASP），验证下降路径。

前置条件（必须先满足）
----------------------
末端已经停在 ``PRE_GRASP_POSE``（真机实测 ``[0.34999, -0.10001, 0.84999]``）。
本脚本**不做** pre_grasp，直接从当前位姿直线逼近抓取点。

执行范围
--------
    approach → ``move_line_to(GRASP_POSE)``   ← 唯一动臂的动作

**不执行** close_gripper / lift / carry。到抓取点就停，读位确认高度。

为什么单独跑这一段
------------------
桌面 0.72 m，``GRASP_POSE.z = 0.77``（桌面上方 5 cm）。空桌抓取时这一段
不会碰东西，但它验证的是「末端能否安全降到 0.77 且不擦桌」——正是
实验 2 摆盒后最需要确认的路径。

用法（HPU 上，且末端已在 PRE_GRASP）
------------------------------------
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    MED_IO=wrapper python3 scripts/run_approach_only.py

退出码：0 到达抓取点 / 1 失败 / 2 前置不满足
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline import io as IO  # noqa: E402
from scenes.scene_medicine_check.pipeline import tuning  # noqa: E402

DESK_HEIGHT_M = 0.72
PRE_TOL_CM = 3.0  # 起点容差：偏离 PRE_GRASP 超过这么多就不敢直线下降


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper").strip().lower()
    grasp = tuning.GRASP_POSE
    pre = tuning.PRE_GRASP_POSE

    _say({
        "step": "boot",
        "script": "run_approach_only",
        "backend": backend,
        "from_expected": pre,
        "target": grasp,
        "will_run": ["approach"],
        "will_not_run": ["close_gripper", "lift", "carry"],
        "desk_clearance_target_cm": round((grasp[2] - DESK_HEIGHT_M) * 100, 1),
    })

    try:
        robot_io = IO.build_io(backend)
    except Exception as exc:  # noqa: BLE001
        _say({"step": "abort", "reason": f"build_io 失败：{type(exc).__name__}: {exc}"})
        return 2

    try:
        # 起点必须已经在 PRE_GRASP 附近，否则说明上一段没成功，直线下降会从错位姿出发
        start = None
        reader = getattr(robot_io, "get_ee_pose", None)
        if callable(reader):
            try:
                ret = reader("right_arm")
                if isinstance(ret, dict) and ret.get("success"):
                    start = ret.get("pose")
            except Exception:  # noqa: BLE001
                pass

        if start:
            dev_cm = max(abs((start[i] - pre[i]) * 100) for i in range(3))
            _say({"step": "start_check", "pose": start,
                  "max_dev_from_pre_cm": round(dev_cm, 2),
                  "tolerance_cm": PRE_TOL_CM})
            if dev_cm > PRE_TOL_CM:
                _say({"step": "abort",
                      "reason": (f"当前末端偏离 PRE_GRASP {dev_cm:.1f}cm（>{PRE_TOL_CM}cm）。"
                                 "先跑 scripts/run_pre_grasp_only.py 把它摆回去"),
                      "action": "未发出任何运动指令"})
                return 2
        else:
            _say({"step": "start_check", "pose": None,
                  "note": "读不到当前位姿，按调用方保证的「已在 PRE_GRASP」继续"})

        t0 = time.time()
        ret = robot_io.move_line_to(grasp)
        elapsed = round(time.time() - t0, 2)
        succ = bool(isinstance(ret, dict) and ret.get("success"))
        _say({
            "step": "sub_step",
            "name": "approach",
            "success": succ,
            "elapsed_s": elapsed,
            "status": ret.get("status") if isinstance(ret, dict) else None,
            "code": ret.get("code") if isinstance(ret, dict) else None,
            "message": ret.get("message") if isinstance(ret, dict) else str(ret),
        })
        if not succ:
            _say({"step": "verdict", "passed": False, "failed_at": "approach"})
            return 1

        time.sleep(0.5)
        after = None
        if callable(reader):
            try:
                r2 = reader("right_arm")
                if isinstance(r2, dict) and r2.get("success"):
                    after = r2.get("pose")
            except Exception:  # noqa: BLE001
                pass

        clearance = round((after[2] - DESK_HEIGHT_M) * 100, 1) if after else None
        err = None
        if after:
            err = [round((after[i] - grasp[i]) * 1000, 2) for i in range(3)]

        _say({
            "step": "ee_after",
            "pose": after,
            "err_mm_xyz": err,
            "desk_clearance_cm": clearance,
        })
        _say({
            "step": "verdict",
            "passed": True,
            "next": ("下降路径通、末端到 0.77 且离桌面 "
                     f"{clearance}cm。可以进实验 2（摆盒后完整 grasp）"
                     if clearance is not None and clearance > 2.0
                     else "末端离桌面过近（<2cm），摆盒后下爪会撞，先调 GRASP_POSE.z"),
        })
        return 0
    finally:
        close = getattr(robot_io, "close", None)
        if callable(close):
            try:
                close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
