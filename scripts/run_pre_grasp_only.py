#!/usr/bin/env python3
"""阶段 A2：只走「初始位 → PRE_GRASP」，到预抓取位就停。

为什么单独做这一步
------------------
真机实测（2026-11-07 17:52）：``init_pose`` 后右臂 7 关节偏差全部归零，
但末端只有 ``[0.2495, -0.2437, 0.7306]`` —— 离桌面（0.72 m）仅 **1.06 cm**。

而 ``PRE_GRASP_POSE = [0.35, -0.10, 0.85]`` 要求末端**同时**外伸 10 cm、
左移 14 cm、抬高 12 cm。这条斜线大部分行程都贴着桌面走，
所以在跑完整 grasp 之前，先单独验证这一段路径是否安全、末端能否真的抬到 0.85。

执行范围
--------
    attach_tool   → 挂夹爪（不动臂）
    open_gripper  → 张开（不动臂）
    pre_grasp     → set_arm_pose(PRE_GRASP_POSE)   ← 唯一动臂的动作

**不执行** approach / close_gripper / lift / carry。
到 PRE_GRASP 后就结束，读位确认高度，再决定下一步。

用法（HPU 上）
--------------
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    MED_IO=wrapper python3 scripts/run_pre_grasp_only.py

退出码：0 到达预抓取位 / 1 失败 / 2 前置不满足
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline import io as IO  # noqa: E402
from scenes.scene_medicine_check.pipeline import tuning  # noqa: E402


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def _read_ee(robot_io):
    """读末端位姿；读不到就返回 None，不影响判定。"""
    getter = getattr(robot_io, "get_ee_pose", None)
    if not callable(getter):
        return None
    try:
        ret = getter("right_arm")
        if isinstance(ret, dict) and ret.get("success"):
            return ret.get("pose")
    except Exception as exc:  # noqa: BLE001
        _say({"step": "ee_read_error", "error": f"{type(exc).__name__}: {exc}"})
    return None


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper").strip().lower()
    pre = tuning.PRE_GRASP_POSE

    _say({
        "step": "boot",
        "script": "run_pre_grasp_only",
        "backend": backend,
        "target": pre,
        "will_run": ["attach_tool", "open_gripper", "pre_grasp"],
        "will_not_run": ["approach", "close_gripper", "lift", "carry"],
        "note": "只走到预抓取位，不下爪、不闭合、不抬升",
    })

    try:
        robot_io = IO.build_io(backend)
    except Exception as exc:  # noqa: BLE001
        _say({"step": "abort", "reason": f"build_io 失败：{type(exc).__name__}: {exc}"})
        return 2

    before = _read_ee(robot_io)
    _say({"step": "ee_before", "pose": before})

    def call(name, fn, err):
        t0 = time.time()
        ret = fn()
        ok_ = bool(isinstance(ret, dict) and ret.get("success"))
        _say({
            "step": "sub_step",
            "name": name,
            "success": ok_,
            "elapsed_s": round(time.time() - t0, 2),
            "status": ret.get("status") if isinstance(ret, dict) else None,
            "message": ret.get("message") if isinstance(ret, dict) else str(ret),
        })
        if not ok_:
            _say({"step": "verdict", "passed": False, "failed_at": name, "error": err})
        return ok_

    try:
        if not call("attach_tool", lambda: robot_io.attach_tool("right_arm", "galbot_gripper"),
                    "attach_tool 失败，无法做末端位姿控制"):
            return 1

        if not call("open_gripper", lambda: robot_io.set_gripper(
                tuning.GRIPPER_OPEN_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N), "夹爪张开失败"):
            return 1

        if not call("pre_grasp", lambda: robot_io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S),
                    "移动到预抓取位姿失败"):
            return 1

        time.sleep(0.5)
        after = _read_ee(robot_io)
        _say({"step": "ee_after", "pose": after})

        cleared = None
        if before and after:
            cleared = {
                "dx_cm": round((after[0] - before[0]) * 100, 1),
                "dy_cm": round((after[1] - before[1]) * 100, 1),
                "dz_cm": round((after[2] - before[2]) * 100, 1),
                "final_z_m": round(after[2], 4),
                "desk_clearance_cm": round((after[2] - 0.72) * 100, 1),
            }

        _say({
            "step": "verdict",
            "passed": True,
            "target_z_m": pre[2],
            "reached": cleared,
            "next": ("末端已到预抓取高度。下一步才谈从 0.85 下降到 GRASP 0.77"
                     if cleared and cleared.get("final_z_m", 0) > 0.80
                     else "末端高度可疑（未达到 0.80），不要继续下降"),
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
