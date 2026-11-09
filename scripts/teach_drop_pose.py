#!/usr/bin/env python3
"""示教：读出投放位姿，生成可直接粘贴进 tuning.py 的代码块。

【这是只读脚本 —— 不会让机器人动一下】
    手动的把手柄/示教模式把右臂摆到「桶口正上方 5cm」，然后运行本脚本，
    它只调 get_ee_pose 读当前值，再把值格式化成 Python 片段打印出来。
    运动 100% 由你手动完成，脚本不参与。

为什么需要它：
    tuning.py 里 PRE_DROP_POSE / DROP_POSE 是 [0.45, -0.35, 0.60/0.50]，
    注释写着 TODO(标定)。这个值是照旧流程抄的，左移 0.5m + 右转 30° 之后
    桶早就不在那儿了。实验 3 若直接跑，手臂会往一个没人验证过的坐标伸手 ——
    这是目前最大的真机风险点。

用法（在 HPU 上，source scripts/env_hpu.sh 之后）：
    1. 手动把右臂摆到桶口正上方约 5cm、夹爪朝下
    2. python3 scripts/teach_drop_pose.py            # 读 PRE_DROP（预备位）
    3. 手动再把臂降到「盒子刚好接触桶口」的高度
    4. python3 scripts/teach_drop_pose.py --tag drop # 读 DROP（投放位）
    5. 两次的值按提示填进 tuning.py

    只想读一次：
    python3 scripts/teach_drop_pose.py --tag grasp   # 顺便还能标抓取位

安全：
    本脚本只调用 get_ee_pose()，不调用任何 set_*/move_*/turn/forward。
    若 get_ee_pose 返回失败，会如实报错，不会编造数值。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _fmt_pose(name: str, pose) -> str:
    vals = ", ".join(f"{float(v):.4f}" for v in pose)
    return f"{name}: List[float] = [{vals}]"


def main() -> int:
    ap = argparse.ArgumentParser(description="读出当前末端位姿（只读，不动臂）")
    ap.add_argument("--tag", default="pre_drop",
                    help="标记这组值要填到哪个常量：pre_drop / drop / grasp / lift / pre_grasp")
    ap.add_argument("--chain", default="right_arm", help="手臂链，默认 right_arm")
    args = ap.parse_args()

    mapping = {
        "pre_drop": "PRE_DROP_POSE",
        "drop": "DROP_POSE",
        "grasp": "GRASP_POSE",
        "lift": "LIFT_POSE",
        "pre_grasp": "PRE_GRASP_POSE",
    }
    const = mapping.get(args.tag, args.tag.upper())

    from sdk_wrapper import wrapper  # 局部 import：mock 模式下也能跑

    ret = wrapper.get_ee_pose(chain=args.chain)
    if not ret.get("success"):
        print(json.dumps({"status": "failed",
                          "message": ret.get("message"),
                          "note": "读不到位姿就绝不编造数值。先确认机械臂已上使能、"
                                  "SDK 已连通（galbot_sdk check-version）。"},
                         ensure_ascii=False))
        return 1

    pose = ret.get("pose") or []
    if len(pose) != 7:
        print(json.dumps({"status": "failed",
                          "message": f"位姿维度不对：{len(pose)} 维，应为 7 维"},
                         ensure_ascii=False))
        return 1

    x, y, z = (float(v) for v in pose[:3])
    print(json.dumps({"status": "ok", "tag": args.tag, "chain": args.chain,
                      "pose": [round(v, 4) for v in pose],
                      "frame": ret.get("frame", "base_link"),
                      "unit": ret.get("unit", "m")}, ensure_ascii=False))

    print("\n--- 粘贴进 scenes/scene_medicine_check/pipeline/tuning.py ---")
    print(_fmt_pose(const, pose))
    print("------------------------------------------------------------")

    # 现场自检：右臂作业区 y<0，桶在右侧
    if y >= 0:
        print(f"[警告] y={y:.3f} >= 0，落到了 base_link 左侧。")
        print("       右臂作业区应在 y<0 一侧，确认没有读错臂。")

    # 高度合理性提示（基于已知桌面 0.72 m）
    if const in ("DROP_POSE", "PRE_DROP_POSE"):
        if z > 0.75:
            print(f"[提示] z={z:.3f} 高于桌面（0.72）。桶若在地上，这个值偏高，"
                  "确认是不是还没降下去。")
        elif z < 0.10:
            print(f"[提示] z={z:.3f} 贴近地面。若桶本身有高度（常见 30-50cm），"
                  "桶口应在 z≈0.3-0.5，请核对。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
