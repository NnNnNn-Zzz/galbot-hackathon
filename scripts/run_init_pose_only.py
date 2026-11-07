#!/usr/bin/env python3
"""阶段 A：只跑 preflight + init_pose，把机器人摆回官方初始位。

用途
----
重启/急停之后，末端可能垂得很低（实测 z=0.732，桌面 0.72，只剩 1.2cm）。
``init_pose`` 第一步 ``init_legs`` 会动腿，躯干会升降，手臂垂这么低有蹭桌风险。
所以先单独执行这一步，把手臂抬到安全高度，读位确认后再跑 grasp。

安全边界
--------
只执行配置里 id 为 ``preflight`` 和 ``init_pose`` 的步骤。
``init_pose`` 内部只有两次 ``set_joints``（腿 / 头+双臂），
不伸向桌面、不动夹爪、不闭合、不走笛卡尔直线。

用法（HPU 上）
--------------
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    MED_IO=wrapper python3 scripts/run_init_pose_only.py

退出码：0 通过 / 1 失败 / 2 前置不满足
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline import io as IO  # noqa: E402
from scenes.scene_medicine_check.pipeline import runner as R  # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context  # noqa: E402

KEEP_STEPS = ("preflight", "init_pose")


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper").strip().lower()

    cfg = R.load_config("medicine.baseline")
    all_ids = [s["id"] for s in cfg["steps"]]
    keep = [s for s in cfg["steps"] if s["id"] in KEEP_STEPS]

    _say({
        "step": "boot",
        "script": "run_init_pose_only",
        "backend": backend,
        "full_pipeline": all_ids,
        "will_run": [s["id"] for s in keep],
        "note": "只摆初始位：动腿 + 动头臂，不伸桌面、不动夹爪",
    })

    if [s["id"] for s in keep] != list(KEEP_STEPS):
        _say({"step": "abort", "reason": "配置里找不到 preflight 或 init_pose",
              "found": [s["id"] for s in keep]})
        return 2

    try:
        robot_io = IO.build_io(backend)
    except Exception as exc:  # noqa: BLE001
        _say({"step": "abort", "reason": f"build_io 失败：{type(exc).__name__}: {exc}"})
        return 2

    cfg["steps"] = keep

    try:
        res = R.run_pipeline(Context(params={}), robot_io, cfg)
    finally:
        close = getattr(robot_io, "close", None)
        if callable(close):
            try:
                close()
            except Exception:  # noqa: BLE001
                pass

    trace = res.get("trace") or []
    passed = (
        res.get("status") == "ok"
        and bool(trace)
        and all(getattr(t, "status", None) is None or str(getattr(t, "status", "")) .endswith("OK")
                for t in trace)
    )
    _say({
        "step": "verdict",
        "passed": bool(passed),
        "status": res.get("status"),
        "failed_step": res.get("failed_step"),
        "failed_stage": res.get("failed_stage"),
        "failed_code": res.get("failed_code"),
        "next": ("初始位已摆好。再读一次位姿确认末端抬高，然后才跑 grasp"
                 if passed else "init_pose 失败，不要跑 grasp"),
    })
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
