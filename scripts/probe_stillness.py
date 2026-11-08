#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静止检测：判断手臂此刻是否被人（遥操作 / 拖动）占用。**纯只读，零运动指令。**

为什么必须先跑：
  11:35→11:38 我们没发任何指令，关节读数却漂移了 20° 量级。
  在被占用的状态下下发 set_joint_positions，控制器会直接返 FAULT ——
  这跟"路径规划失败"是两件事，混在一起会误判成几何问题（已经误判过一次）。

判据（全精度，单位度）：
  · 所有关节逐位变化 <= STILL_EPS (0.005°)          → STILL，可以下单
  · 任一关节变化 >  MOVE_EPS (0.050°) 且不是来回抖   → OCCUPIED，别下单
  · 在两者之间                                        → JITTER，建议再采一次

用法（HPU 上）：
    cd ~/galbot_drawer_exp1 && source scripts/env_hpu.sh
    MED_IO=wrapper python3 scripts/probe_stillness.py --times 3 --interval 20

只读：全程只调 get_joint_positions / get_ee_pose / get_gripper_width。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

DEG = 180.0 / math.pi
STILL_EPS = 0.005   # 度
MOVE_EPS = 0.050    # 度


def read_once(arms):
    """采样一次，返回 {arm: {joints_deg, ee_pose, gripper}}。"""
    from sdk_wrapper import wrapper

    snap = {}
    for arm in arms:
        rec = {"joints_deg": None, "joints_raw": None, "message": ""}
        try:
            ret = wrapper.get_joint_positions(joint_groups=[arm], joint_names=[])
            if isinstance(ret, dict) and ret.get("success"):
                pos = ret.get("positions") or ret.get("joint_positions") or []
                if pos:
                    rec["joints_raw"] = [float(v) for v in pos]
                    rec["joints_deg"] = [round(float(v) * DEG, 4) for v in pos]
            else:
                rec["message"] = (ret or {}).get("message", "无反馈")
        except Exception as exc:  # noqa: BLE001 - 只读探针要把异常如实记下
            rec["message"] = "抛异常：%s" % exc

        try:
            ee = wrapper.get_ee_pose(arm)
            if isinstance(ee, dict) and ee.get("success"):
                rec["ee_pose"] = ee.get("pose") or ee.get("ee_pose")
        except Exception:  # noqa: BLE001
            pass

        side = "right" if arm.startswith("right") else "left"
        try:
            gw = wrapper.get_gripper_width(side)
            if isinstance(gw, dict) and gw.get("success"):
                rec["gripper_m"] = gw.get("width")
        except Exception:  # noqa: BLE001
            pass
        snap[arm] = rec
    return snap


def compare(a, b):
    """两次采样逐关节比对，返回最大变化量与变化明细。"""
    out = {"max_delta_deg": None, "per_joint_deg": None, "changed": []}
    if not a or not b or len(a) != len(b):
        return out
    d = [abs(x - y) for x, y in zip(a, b)]
    out["per_joint_deg"] = [round(v, 4) for v in d]
    out["max_delta_deg"] = round(max(d), 4)
    out["changed"] = [i + 1 for i, v in enumerate(d) if v > MOVE_EPS]
    return out


def main():
    ap = argparse.ArgumentParser(description="双臂静止检测（只读）")
    ap.add_argument("--times", type=int, default=3, help="采样次数，默认 3")
    ap.add_argument("--interval", type=float, default=20.0, help="采样间隔秒，默认 20")
    ap.add_argument("--arms", default="right_arm,left_arm")
    ap.add_argument("--json", default="", help="结果写入该路径")
    args = ap.parse_args()

    arms = [s.strip() for s in args.arms.split(",") if s.strip()]
    print("=" * 78)
    print("双臂静止检测（纯只读；只调 get_* 接口）")
    print("=" * 78)

    snaps = []
    for i in range(args.times):
        if i:
            time.sleep(args.interval)
        t0 = time.time()
        s = read_once(arms)
        s["_t"] = time.strftime("%H:%M:%S", time.localtime(t0))
        snaps.append(s)
        print("\n[采样 %d/%d] %s" % (i + 1, args.times, s["_t"]))
        for arm in arms:
            r = s[arm]
            if r["joints_deg"] is None:
                print("  %-10s 读不到关节角：%s" % (arm, r["message"]))
            else:
                print("  %-10s %s" % (arm, r["joints_deg"]))
                if r.get("gripper_m") is not None:
                    print("  %-10s 夹爪 %.4f m" % ("", r["gripper_m"]))

    print("\n" + "=" * 78)
    print("逐关节变化（度）")
    print("=" * 78)
    verdicts = {}
    for arm in arms:
        print("\n%s:" % arm)
        worst_all = 0.0
        for k in range(1, len(snaps)):
            c = compare(snaps[k - 1][arm]["joints_deg"], snaps[k][arm]["joints_deg"])
            if c["max_delta_deg"] is None:
                print("  第 %d→%d 次：无法比对（读取失败）" % (k, k + 1))
                continue
            worst_all = max(worst_all, c["max_delta_deg"])
            print("  第 %d→%d 次：最大 %.4f°  逐关节 %s"
                  % (k, k + 1, c["max_delta_deg"], c["per_joint_deg"]))
            if c["changed"]:
                print("      显著变化关节：%s" % ["j%d" % j for j in c["changed"]])

        if worst_all <= STILL_EPS:
            v = "STILL"
            msg = "静止（最大变化 %.4f° ≤ %.3f°），可以下发运动指令" % (worst_all, STILL_EPS)
        elif worst_all <= MOVE_EPS:
            v = "JITTER"
            msg = "微抖（最大 %.4f°），建议再采一次确认" % worst_all
        else:
            v = "OCCUPIED"
            msg = ("仍在被占用（最大变化 %.4f° > %.3f°）："
                   "有人遥操作/拖动，此刻下发运动指令会 FAULT" % (worst_all, MOVE_EPS))
        verdicts[arm] = {"verdict": v, "max_delta_deg": round(worst_all, 4), "message": msg}
        print("  → %s：%s" % (v, msg))

    print("\n" + "=" * 78)
    overall = "STILL" if all(v["verdict"] == "STILL" for v in verdicts.values()) else \
              ("OCCUPIED" if any(v["verdict"] == "OCCUPIED" for v in verdicts.values()) else "JITTER")
    print("总体判定：%s" % overall)
    if overall != "STILL":
        print("→ 不允许下发运动指令。等占用方松手后重跑本脚本。")
    print("=" * 78)

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"overall": overall, "verdicts": verdicts,
                       "snapshots": snaps}, fh, ensure_ascii=False, indent=2, default=str)
        print("结果已存：%s" % os.path.abspath(args.json))

    return 0 if overall == "STILL" else 1


if __name__ == "__main__":
    sys.exit(main())
