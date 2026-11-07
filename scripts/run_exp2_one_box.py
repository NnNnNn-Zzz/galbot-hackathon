#!/usr/bin/env python3
"""实验 2：单盒盲抓 —— 桌面上放**一个**盒子，跑 preflight + grasp。

目的
----
实验 1 证明了「空桌轨迹安全」。实验 2 要回答的问题只有一个：
    **固定位姿 GRASP_POSE 到底能不能对准真实的盒子？**

因为不看图像、不识别、不选目标，能不能对准完全取决于「盒子有没有摆在
GRASP_POSE 指定的那个点上」。所以本实验有一半的工作量在现场摆盒，
而不是在代码里。

药盒数据需求（本轮核心问题的答案）
--------------------------------
本实验需要的只有**几何数据**，且全部现场用卷尺量，不需要预先准备任何表格：

    需要的                         不需要的
    -----------------------------  -----------------------------
    盒中心 x = 0.40 m（前后）      药名 / 商品名
    盒中心 y = -0.10 m（左右）     生产日期 / 过期日期 / 批号
    盒腰高度 z = 0.75 m            重量 / 材质 / 颜色
    盒宽 <= 8 cm（否则夹不住）      盒内装什么

日期类数据要等 VLM/OCR 版才用得上，而且那时是从相机图像里读出来的，
不是从任何配置文件里读的。

通过判据（详见 experiments/criteria.py）
---------------------------------------
1. preflight 与 grasp 两步都必须是 ok
2. picked 不作硬要求 —— 本后端读不到夹爪反馈（is_holding() 恒为 None），
   抓没抓住只能靠**目视**。链路走完就算链路通过。
3. 任何 unknown -> 不通过，且必须立即停机

用法
----
真机（在 HPU 上，先 source scripts/env_hpu.sh）：
    MED_IO=wrapper python3 scripts/run_exp2_one_box.py

mock 干跑（默认按真实摆位放一盒）：
    MED_IO=mock python3 scripts/run_exp2_one_box.py

    # 想让 mock 只放一盒而不是默认三盒：
    # 脚本已自动处理，mock 会被设置为「只在 GRASP_POSE 处放一盒」

退出码：0 = 通过；1 = 不通过；2 = 前置检查未过（未发任何动作）
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

# 盒子宽度上限：夹爪开度 GRIPPER_OPEN_M = 0.10 m，留 2 cm 余量给摆放误差
MAX_BOX_WIDTH_M = tuning.GRIPPER_OPEN_M - 0.02


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def _placement_brief() -> dict:
    """把「盒子该摆在哪」翻译成一句能照着做的话。"""
    x, y, z = tuning.GRASP_POSE[0], tuning.GRASP_POSE[1], tuning.GRASP_POSE[2]
    # base_link 的 y 轴指向机器人左侧，y<0 即右手侧
    return {
        "center_x_m": x,
        "center_y_m": y,
        "waist_z_m": z,
        "how_to_place": (
            f"从机器人底座正前沿往里量 {x * 100:.0f} cm，贴一条横向胶带；"
            f"再从底座中线往**右**量 {abs(y) * 100:.0f} cm，贴一条纵向胶带；"
            "盒子的中心压在两条胶带的**交点**上，长边朝向机器人（垂直于手臂伸出方向）。"
        ),
        "width_limit_m": round(MAX_BOX_WIDTH_M, 3),
        "width_note": f"盒宽必须 <= {MAX_BOX_WIDTH_M * 100:.0f} cm，"
                      "超过就换窄盒，否则夹爪闭合时夹不到盒壁、会滑脱。",
    }


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper")

    _say({"step": "boot", "experiment": 2,
          "title": criteria.EXPERIMENT_TITLES[2],
          "backend": backend,
          "placement": _placement_brief()})

    cfg = pipeline_runner.load_config("medicine.baseline")
    keep = set(criteria.GRASP_ONLY_STEPS)
    cfg["steps"] = [s for s in cfg["steps"] if s["id"] in keep]
    _say({"step": "config", "stages": [s["id"] for s in cfg["steps"]]})
    if len(cfg["steps"]) != len(criteria.GRASP_ONLY_STEPS):
        _say({"step": "abort",
              "message": f"配置里没找齐 {sorted(keep)}",
              "action": "检查 configs/medicine.baseline.json 的步骤 id"})
        return 2

    ctx = Context(params={})
    if backend.strip().lower() == "mock":
        # mock 里只放一盒，且位置与 GRASP_POSE 对齐，模拟「摆得很准」的理想情况
        one_box = [{
            "id": "box_single", "label": "white",
            "ee_pose": [tuning.GRASP_POSE[0], tuning.GRASP_POSE[1], tuning.GRASP_POSE[2]],
            "position_m": [tuning.GRASP_POSE[0], tuning.GRASP_POSE[1], tuning.GRASP_POSE[2]],
            "frame": "base_link", "unit": "m",
            "image_ref": "mock_frame_0", "bbox": [0, 0, 60, 40],
            "score": 0.9, "expiry_date": "2026-08-01",
        }]
        io = build_io(backend, objects=one_box)
        _say({"step": "mock_table", "objects": 1,
              "position": [tuning.GRASP_POSE[0], tuning.GRASP_POSE[1], tuning.GRASP_POSE[2]],
              "note": "mock 已模拟单盒，位置与 GRASP_POSE 完全对齐"})
    else:
        io = build_io(backend)

    result = pipeline_runner.run_pipeline(ctx, io, cfg)

    for entry in result.get("trace") or []:
        _say({"stage": entry.get("step"),
              "fate": entry.get("status"),
              "code": entry.get("code"),
              "elapsed_s": entry.get("elapsed_s"),
              "message": str(entry.get("message", ""))[:160]})

    result["picked"] = getattr(ctx, "picked", None)

    # expect_pick 保持 None：本后端无夹爪反馈，不把「抓到」当成硬指标。
    # 想强制要求抓到，改成 criteria.judge_grasp_only(result, 2, expect_pick=True)
    verdict = criteria.judge_grasp_only(result, experiment=2)
    _say({"step": "verdict",
          "pipeline_status": result.get("status"),
          "passed": verdict["passed"],
          "link_ok": verdict["link_ok"],
          "grabbed": verdict["grabbed"],
          "grabbed_note": ("None = 本后端读不到夹爪反馈，请目视确认是否夹住"
                           if verdict["grabbed"] is None else None),
          "counts": result.get("counts"),
          "io_missing": result.get("io_missing"),
          "reasons": verdict["reasons"],
          "next_action": verdict["next_action"]})

    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
