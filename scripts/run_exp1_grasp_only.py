#!/usr/bin/env python3
"""实验 1：空桌盲抓——只跑 s0_preflight + s1_grasp，桌面不放盒子。

目的：验证抓取轨迹本身安全（无碰撞、无急停）、z 高度合理。
桌上无盒，抓空是预期行为（picked=False/None 不算失败）。

用法：
    # 真机（在 HPU 上，source scripts/env_hpu.sh 后）
    MED_IO=wrapper python3 scripts/run_exp1_grasp_only.py
    # mock 干跑
    MED_IO=mock python3 scripts/run_exp1_grasp_only.py
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline import runner as pipeline_runner  # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context  # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io  # noqa: E402


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper")
    cfg = pipeline_runner.load_config("medicine.baseline")
    cfg["steps"] = cfg["steps"][:2]  # 只保留 preflight + grasp
    print(json.dumps({"step": "boot", "backend": backend,
                      "stages": [s["id"] for s in cfg["steps"]]},
                     ensure_ascii=False), flush=True)

    ctx = Context(params={})
    io = build_io(backend)
    res = pipeline_runner.run_pipeline(ctx, io, cfg)

    for t in ctx.trace:
        print(json.dumps({"stage": t.get("stage"), "status": t.get("status"),
                          "message": str(t.get("message", ""))[:120]},
                         ensure_ascii=False), flush=True)
    print(json.dumps({"step": "done",
                      "success": res.get("success"),
                      "picked": getattr(ctx, "picked", None),
                      "note": "空桌实验：picked=False/None 属预期，轨迹安全走完即通过"},
                     ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
