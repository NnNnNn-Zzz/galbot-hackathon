# -*- coding: utf-8 -*-
"""把双臂回正到「抱持位」，为实验 1/2/3 准备一个确定的起点。

为什么需要这个脚本（2026-11-08 踩的坑）
--------------------------------------
示教点位 P0/P1/P2 是**手拖着录**的。录完之后只要有人再碰一次手臂，
机器人就停在一个"谁也不知道"的构型上。实测过两次：

    08:19 读：dev_deg 全 0    → 正好在抱持位
    08:40 读：右臂 j5 偏 −74.1°、j1 偏 −15.6°；左臂 j6 偏 +48.8°、j3 偏 −42.1°

在这种情况下直接跑实验 1，第一条 `set_joints` 就得从当前位跨到 P0：
    当前 → P0 一步到位 = j1 193.7°、合计 618° —— 跨度闸门直接拒；
    当前 → 入口 ramp 首点 = j5 76.6°、合计 189.5° —— 离 200° 拦截线只剩 10°。
而且入口 ramp 末端 y 峰值 +0.055 m、已经扫过身体中线，**左臂如果还架在外面就是真碰撞**。
所以：跑实验前先回正，双臂都回。

做法
----
关节空间线性插值（和 `_plan_jointpath.py` / `_gen_entry_points.py` 同一套方法论）：
两端合法则线段合法，不会穿限位；每段压到 ≤ `--max-step-deg`（默认 10°），
逐段下发 + 逐段回读校验，任一段读数不可信就**立刻停**，不自动重试、不自动归位。

用法
----
    # 只读：只看当前离抱持位差多少，一条指令都不发
    python3 scripts/home_to_carry.py --check

    # 只读预演：算完分段、打印每段跨度和预计耗时，一条指令都不发
    python3 scripts/home_to_carry.py --plan

    # 真动（必须显式 --yes，仿 run_exp3_full.py 的双重确认）
    python3 scripts/home_to_carry.py --yes

参数
----
    --arms both|right|left   默认 both。both = 先右臂再左臂，串行，出问题能定位到哪条臂。
    --max-step-deg 10.0      单段最大关节跨度
    --tol-deg 3.0            每段回读校验容差
    --speed 0.12             关节速度 rad/s（比链内 0.2 慢，这是大幅摆动）
    --settle 0.35            每段到位后静置秒数

退出码：0 = 到位；1 = 有失败/不可判定；2 = 参数或环境问题。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from scenes.scene_medicine_check.pipeline import tuning          # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io      # noqa: E402

DEG = 180.0 / math.pi

#: 关节限位（度）—— 只用于**预检**，不参与规划
LIMITS_DEG = {
    "right_arm_joint1": (-172.1, 172.1), "right_arm_joint2": (-92.1, 92.1),
    "right_arm_joint3": (-167.1, 167.1), "right_arm_joint4": (17.2, 147.1),
    "right_arm_joint5": (-167.1, 167.1), "right_arm_joint6": (-42.1, 47.1),
    "right_arm_joint7": (-88.1, 88.1),
}
for _k, _v in list(LIMITS_DEG.items()):          # 左臂按镜像对称补上
    LIMITS_DEG[_k.replace("right_arm", "left_arm")] = (-_v[1], -_v[0])
LIMITS_DEG["left_arm_joint4"] = (-147.1, -17.2)

ARM_TARGET = {
    "right_arm": list(tuning.CARRY_JOINT_POSITIONS),
    "left_arm": [float(v) for v in tuning.UPPER_JOINT_POSITIONS[2:9]],
}
ARM_TARGET_SRC = "tuning.CARRY_JOINT_POSITIONS（右）/ UPPER_JOINT_POSITIONS[2:9]（左镜像），官方 example6"


def _deg_list(vals):
    return [round(float(v) * DEG, 1) for v in vals]


def margin_deg(names, q):
    """返回 (最紧关节名, 余量度数)。"""
    worst = ("", 9e9)
    for n, v in zip(names, q):
        lo, hi = LIMITS_DEG[n]
        d = float(v) * DEG
        m = min(d - lo, hi - d)
        if m < worst[1]:
            worst = (n, m)
    return worst


def read_arm(io, group):
    """读一条臂 7 个关节。读不到或数量不对一律返回 (None, 原因)。"""
    res = io.get_joint_positions(joint_groups=[group], joint_names=[])
    if not res.get("success"):
        return None, f"get_joint_positions({group}) 失败：{res.get('message')}"
    names = [str(n) for n in (res.get("joint_names") or [])]
    pos = [float(v) for v in (res.get("joint_positions") or [])]
    if len(names) != len(pos) or not pos:
        return None, f"{group} 关节名 {len(names)} 个、角度 {len(pos)} 个，不可信"
    want = [f"{group}_joint{i}" for i in range(1, 8)]
    if sorted(names) != sorted(want):
        return None, f"{group} 读回的关节名不是 1..7：{names}"
    idx = [names.index(w) for w in want]
    return [pos[i] for i in idx], None


def plan_arm(group, cur):
    tgt = ARM_TARGET[group]
    names = [f"{group}_joint{i}" for i in range(1, 8)]
    span = [abs(a - b) * DEG for a, b in zip(cur, tgt)]
    return names, tgt, span


def main() -> int:
    ap = argparse.ArgumentParser(description="双臂回正到抱持位（关节 lerp，逐段校验）")
    ap.add_argument("--check", action="store_true", help="只读：只看偏差，不规划不运动")
    ap.add_argument("--plan", action="store_true", help="只读预演：算分段，不运动")
    ap.add_argument("--yes", action="store_true", help="真的下发运动指令（必填）")
    ap.add_argument("--arms", default="both", choices=["both", "right", "left"])
    ap.add_argument("--max-step-deg", type=float, default=10.0)
    ap.add_argument("--tol-deg", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--settle", type=float, default=0.35)
    ap.add_argument("--log-dir", default="logs")
    args = ap.parse_args()

    readonly = args.check or args.plan or (not args.yes)
    if not args.check and not args.plan and not args.yes:
        print("✗ 既没给 --check / --plan，也没给 --yes。")
        print("  本脚本会真的让机器人动，必须显式加 --yes。先跑 --plan 看一眼分段。")
        return 2

    groups = ["right_arm", "left_arm"] if args.arms == "both" else [f"{args.arms}_arm"]

    io = build_io("")            # 尊重 MED_IO 环境变量；不设就是 wrapper（真机）
    backend = type(io).__name__

    print("=" * 78)
    print(f"回正到抱持位 · 后端={backend} · 模式={'只读' if readonly else '★真机运动★'}")
    print(f"目标来源：{ARM_TARGET_SRC}")
    print("=" * 78)

    jobs = []
    for g in groups:
        cur, err = read_arm(io, g)
        if cur is None:
            print(f"✗ {g}：{err}")
            print("  读不到当前关节角就绝不能动 —— 不知道从哪出发等于盲动。")
            return 1
        names, tgt, span = plan_arm(g, cur)
        jobs.append((g, names, cur, tgt, span))

        # ---- 目标本身必须在限位内（否则下发前就拒）----
        tw, tm = margin_deg(names, tgt)
        cur_w, cur_m = margin_deg(names, cur)
        print(f"\n[{g}]")
        print(f"  当前 : {_deg_list(cur)}")
        print(f"  目标 : {_deg_list(tgt)}")
        print(f"  差值 : {_deg_list([b - a for a, b in zip(cur, tgt)])}")
        print(f"  最大跨度 {max(span):.1f}° @ j{span.index(max(span)) + 1}，"
              f"7 关节合计 {sum(span):.1f}°")
        print(f"  当前最紧限位余量 {cur_m:.1f}° @ {cur_w}"
              f"{'  ⚠️ 已贴限位' if cur_m < 5.0 else ''}")
        print(f"  目标最紧限位余量 {tm:.1f}° @ {tw}")
        if tm < 2.0:
            print(f"  ✗ 目标点限位余量 {tm:.1f}° < 2.0°，这个目标本身不可用，停止。")
            return 1

    # ---- 分段 ----
    total_seg = 0
    max_step_all = 0.0
    for g, names, cur, tgt, span in jobs:
        n = max(1, int(math.ceil(max(span) / args.max_step_deg)))
        per = max(span) / n
        total_seg += n
        max_step_all = max(max_step_all, per)
        print(f"\n[{g}] 切 {n} 段，每段最大 {per:.1f}°"
              f"（上限 {args.max_step_deg:.0f}°）")

    est = 0.0
    for g, names, cur, tgt, span in jobs:
        n = max(1, int(math.ceil(max(span) / args.max_step_deg)))
        # 单段步长 = 最大跨度 / 段数（均匀 lerp），不是"到终点的剩余距离"
        per_rad = math.radians(max(span)) / n
        est += n * (per_rad / args.speed + args.settle)
    print(f"\n合计 {total_seg} 段，单段最大 {max_step_all:.1f}°，"
          f"预计 {est:.1f} s（不含进程启动）")

    if args.check or args.plan:
        print(f"\n[{ '--check' if args.check else '--plan' }] 只读模式："
              f"未下发任何运动指令。")
        return 0

    # ---------------- 以下才是真机运动 ----------------
    print("\n" + "=" * 78)
    print("开始下发关节指令。手放急停上。任一段回读异常我会立刻停，不重试。")
    print("=" * 78)

    record = {"ts": datetime.now().isoformat(timespec="seconds"),
              "backend": backend, "args": vars(args), "arms": {}}
    t0 = time.time()

    for g, names, cur, tgt, span in jobs:
        n = max(1, int(math.ceil(max(span) / args.max_step_deg)))
        rows = []
        print(f"\n--- {g}（{n} 段）---")
        for i in range(1, n + 1):
            q = [cur[k] + (tgt[k] - cur[k]) * i / n for k in range(7)]
            step = max(abs(q[k] - cur[k] - (tgt[k] - cur[k]) * (i - 1) / n) for k in range(7)) * DEG
            tag = f"segment {i}/{n}"

            res = io.set_joints(q, names, args.speed)
            if not res.get("success"):
                msg = res.get("message") or res
                print(f"  ✗ {tag} 下发失败：{msg}")
                print("  已停止，**不做自动归位**。当前构型在段 "
                      f"{i - 1}/{n} 与 {i}/{n} 之间，请人工确认。")
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                _save(record, args.log_dir)
                return 1
            if args.settle > 0:
                time.sleep(args.settle)

            got, err = read_arm(io, g)
            if got is None:
                print(f"  ✗ {tag} 回读不可信：{err} —— 状态不可判定，立刻停。")
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                _save(record, args.log_dir)
                return 1
            dev = max(abs(a - b) * DEG for a, b in zip(got, q))
            mj, mv = margin_deg(names, got)
            rows.append({"segment": i, "target_deg": _deg_list(q),
                         "actual_deg": _deg_list(got), "err_deg": round(dev, 2),
                         "margin_deg": round(mv, 1), "margin_joint": mj})
            print(f"  {tag}  步长 {step:5.1f}°  回读偏差 {dev:5.2f}°  "
                  f"最紧余量 {mv:5.1f}° @ {mj}")
            if dev > args.tol_deg:
                print(f"  ✗ {tag} 回读偏差 {dev:.2f}° > 容差 {args.tol_deg:.1f}°，停止。")
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                _save(record, args.log_dir)
                return 1

        # 终点复核
        got, err = read_arm(io, g)
        if got is None:
            print(f"  ✗ {g} 终点复核读不到：{err}")
            record["arms"][g] = {"ok": False, "at": "final", "rows": rows}
            _save(record, args.log_dir)
            return 1
        dev = max(abs(a - b) * DEG for a, b in zip(got, ARM_TARGET[g]))
        mj, mv = margin_deg(names, got)
        print(f"  ✓ {g} 终点：{_deg_list(got)}  对抱持位偏差 {dev:.2f}°  "
              f"最紧余量 {mv:.1f}° @ {mj}")
        record["arms"][g] = {"ok": dev <= args.tol_deg, "final_deg": _deg_list(got),
                             "err_deg": round(dev, 2), "margin_deg": round(mv, 1),
                             "rows": rows}
        if dev > args.tol_deg:
            _save(record, args.log_dir)
            return 1

    record["elapsed_s"] = round(time.time() - t0, 2)
    path = _save(record, args.log_dir)
    print("\n" + "=" * 78)
    print(f"全部到位（{record['elapsed_s']} s）。可以开始实验。日志：{path}")
    print("=" * 78)
    return 0


def _save(record, log_dir):
    try:
        os.makedirs(log_dir, exist_ok=True)
        p = os.path.join(log_dir, f"home_to_carry_{datetime.now():%Y%m%d_%H%M%S}.json")
        with open(p, "w", encoding="utf-8") as fh:
            json.dump(record, fh, ensure_ascii=False, indent=2)
        return p
    except Exception as exc:  # noqa: BLE001 - 日志写不了不该影响结论
        print(f"  （日志写入失败：{exc}）")
        return "(未保存)"


if __name__ == "__main__":
    sys.exit(main())
