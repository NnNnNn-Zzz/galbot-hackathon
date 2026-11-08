#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把某一条臂从**当前构型**按关节插值走到指定示教点。

为什么需要它
------------
`home_to_carry.py` 的目标写死是抱持位，而实验 2 的起点是 P0（示教点）。
手臂被人拖走后停在"谁也不知道"的构型上，需要一条可控、可回读、可中断的路径把它送回去。

判据只用两样东西（用户 2026-11-08 11:47 定的硬约束）：
  1. 本次布置示教记录下来的关节角（`teach_drawer_real.json`）；
  2. URDF 自身的关节限位（本文件 `LIMITS_DEG` 表，与 URDF 一致）。
**不用**任何 `base_link` 下的桌面 / 抽屉 / 桶坐标 —— 底盘与桌子的相对位置每次布置随机，
据此算出的净空是虚假精度。所以本脚本**不做、也不报**任何"离障碍物多远"。

安全闸门（依下发顺序）
----------------------
  1. 读不到当前构型                                   → 拒，零运动
  2. 目标点自身限位余量 < 2°                           → 拒，零运动
  3. 目标点关节数不是 7                                → 拒，零运动
  4. 跨度闸门：单关节 ≥ 90° 或 7 关节合计 ≥ 200°        → 拒，零运动（可用 --allow-wide 放行）
  5. **前后两次读数差 > 0.05°（= 有人还在动/遥操作占用）** → 拒，零运动
  6. 逐段回读偏差 > 容差                               → 立刻停，**不重试、不自动归位**

用法（HPU 上）
--------------
    cd ~/galbot_drawer_exp1 && source scripts/env_hpu.sh

    MED_IO=wrapper python3 scripts/goto_point.py --point P0_measured --arms right --check
    MED_IO=wrapper python3 scripts/goto_point.py --point P0_measured --arms right --plan
    MED_IO=wrapper python3 scripts/goto_point.py --point P0_measured --arms right --yes

退出码：0 = 到位；1 = 失败/不可判定；2 = 参数或环境问题。
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

from scenes.scene_medicine_check.pipeline.teach import load_point   # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io        # noqa: E402

DEG = 180.0 / math.pi

DEFAULT_TEACH = "scenes/scene_medicine_check/pipeline/configs/teach_drawer_real.json"

#: 关节限位（度）—— 只用于预检，不参与规划。与 URDF 一致。
LIMITS_DEG = {
    "right_arm_joint1": (-172.1, 172.1), "right_arm_joint2": (-92.1, 92.1),
    "right_arm_joint3": (-167.1, 167.1), "right_arm_joint4": (17.2, 147.1),
    "right_arm_joint5": (-167.1, 167.1), "right_arm_joint6": (-42.1, 47.1),
    "right_arm_joint7": (-88.1, 88.1),
}
for _k, _v in list(LIMITS_DEG.items()):
    LIMITS_DEG[_k.replace("right_arm", "left_arm")] = (-_v[1], -_v[0])
LIMITS_DEG["left_arm_joint4"] = (-147.1, -17.2)

#: 跨度闸门（与 pipeline 内一致）
SPAN_JOINT_MAX_DEG = 90.0
SPAN_SUM_MAX_DEG = 200.0
#: 静止检测阈值（度）
STILL_EPS_DEG = 0.05


def deg_list(vals):
    return [round(float(v) * DEG, 2) for v in vals]


def margin_deg(names, q):
    worst = ("", 9e9)
    for n, v in zip(names, q):
        lo, hi = LIMITS_DEG[n]
        m = min(float(v) * DEG - lo, hi - float(v) * DEG)
        if m < worst[1]:
            worst = (n, m)
    return worst


def read_arm(io, group):
    """读一条臂 7 个关节（rad）。读不到或数量不对返回 (None, 原因)。"""
    res = io.get_joint_positions(joint_groups=[group], joint_names=[])
    if not res.get("success"):
        return None, "get_joint_positions(%s) 失败：%s" % (group, res.get("message"))
    names = [str(n) for n in (res.get("joint_names") or [])]
    pos = [float(v) for v in (res.get("joint_positions") or [])]
    if len(names) != len(pos) or not pos:
        return None, "%s 关节名 %d 个、角度 %d 个，不可信" % (group, len(names), len(pos))
    want = ["%s_joint%d" % (group, i) for i in range(1, 8)]
    if sorted(names) != sorted(want):
        return None, "%s 读回的关节名不是 1..7：%s" % (group, names)
    idx = [names.index(w) for w in want]
    return [pos[i] for i in idx], None


def main():
    ap = argparse.ArgumentParser(description="关节插值走到指定示教点（逐段校验，失败即停）")
    ap.add_argument("--point", required=True, help="目标示教点名，如 P0_measured")
    ap.add_argument("--file", default=DEFAULT_TEACH, help="示教文件")
    ap.add_argument("--arms", default="right", choices=["right", "left", "both"])
    ap.add_argument("--check", action="store_true", help="只读：只看偏差与闸门，不规划")
    ap.add_argument("--plan", action="store_true", help="只读预演：算分段，不运动")
    ap.add_argument("--yes", action="store_true", help="真的下发运动指令（必填）")
    ap.add_argument("--max-step-deg", type=float, default=10.0)
    ap.add_argument("--tol-deg", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--settle", type=float, default=0.35)
    ap.add_argument("--still-wait", type=float, default=6.0,
                    help="静止复检间隔秒（默认 6）")
    ap.add_argument("--allow-wide", action="store_true",
                    help="放行跨度闸门（单关节≥90° 或合计≥200°）。默认拒。")
    ap.add_argument("--log-dir", default="logs")
    args = ap.parse_args()

    readonly = args.check or args.plan or (not args.yes)
    if not args.check and not args.plan and not args.yes:
        print("✗ 既没给 --check / --plan，也没给 --yes。")
        print("  本脚本会真的让机器人动，必须显式加 --yes。先跑 --plan 看一眼分段。")
        return 2

    groups = ["right_arm", "left_arm"] if args.arms == "both" else ["%s_arm" % args.arms]

    # ---- 目标点 ----
    rec = load_point(args.file, args.point)
    jp = (rec or {}).get("joint_positions") or []
    if len(jp) != 7:
        print("✗ 示教点 %s 的关节角不完整（%d 个）—— 无法作为目标" % (args.point, len(jp)))
        return 2
    tgt_rad = [float(v) for v in jp]

    io = build_io("")
    backend = type(io).__name__

    print("=" * 78)
    print("走到示教点 %s · 后端=%s · 模式=%s" % (args.point, backend, "只读" if readonly else "★真机运动★"))
    print("判据：仅关节空间（示教关节角 + URDF 限位）；不含任何环境坐标")
    print("=" * 78)

    jobs = []
    for g in groups:
        names = ["%s_joint%d" % (g, i) for i in range(1, 8)]
        cur, err = read_arm(io, g)
        if cur is None:
            print("✗ %s：%s" % (g, err))
            print("  读不到当前关节角就绝不能动 —— 不知道从哪出发等于盲动。")
            return 1

        # 闸门 2：目标点自身限位
        tw, tm = margin_deg(names, tgt_rad)
        cw, cm = margin_deg(names, cur)
        span = [abs(a - b) * DEG for a, b in zip(cur, tgt_rad)]
        print("\n[%s]" % g)
        print("  当前 : %s" % deg_list(cur))
        print("  目标 : %s   （示教点 %s）" % (deg_list(tgt_rad), args.point))
        print("  差值 : %s" % [round(b - a, 2) for a, b in zip(deg_list(cur), deg_list(tgt_rad))])
        print("  最大跨度 %.1f° @ j%d，7 关节合计 %.1f°"
              % (max(span), span.index(max(span)) + 1, sum(span)))
        print("  当前最紧限位余量 %5.2f° @ %s%s"
              % (cm, cw, "  ⚠️ 已贴限位" if cm < 5.0 else ""))
        print("  目标最紧限位余量 %5.2f° @ %s%s"
              % (tm, tw, "  ⚠️ 余量偏小" if tm < 5.0 else ""))

        if tm < 2.0:
            print("  ✗ 目标点限位余量 %.2f° < 2.0° —— 这个目标本身不可用，停止。" % tm)
            return 1
        if not args.allow_wide and (max(span) >= SPAN_JOINT_MAX_DEG
                                    or sum(span) >= SPAN_SUM_MAX_DEG):
            print("  ✗ 跨度闸门：单关节 %.1f°（限 %.0f°）/ 合计 %.1f°（限 %.0f°）"
                  % (max(span), SPAN_JOINT_MAX_DEG, sum(span), SPAN_SUM_MAX_DEG))
            print("    拒绝下发。确需放行请加 --allow-wide。")
            return 1
        jobs.append((g, names, cur, span))

    # ---- 分段 ----
    total_seg = 0
    max_step_all = 0.0
    est = 0.0
    for g, names, cur, span in jobs:
        n = max(1, int(math.ceil(max(span) / args.max_step_deg)))
        per = max(span) / n
        total_seg += n
        max_step_all = max(max_step_all, per)
        est += n * (math.radians(per) / args.speed + args.settle)
        print("\n[%s] 切 %d 段，每段最大 %.1f°（上限 %.0f°）"
              % (g, n, per, args.max_step_deg))
    print("\n合计 %d 段，单段最大 %.1f°，预计 %.1f s（不含进程启动）"
          % (total_seg, max_step_all, est))

    if args.check or args.plan:
        print("\n[%s] 只读模式：未下发任何运动指令。"
              % ("--check" if args.check else "--plan"))
        return 0

    # ---- 闸门 5：静止复检（把"被占用"前移到下发之前）----
    print("\n" + "=" * 78)
    print("静止复检：间隔 %.0f s 读两次，逐关节比对" % args.still_wait)
    print("=" * 78)
    before = {}
    for g, names, cur, span in jobs:
        got, err = read_arm(io, g)
        if got is None:
            print("✗ %s 静止复检读取失败：%s —— 停止" % (g, err))
            return 1
        before[g] = got
    time.sleep(args.still_wait)
    for g, names, cur, span in jobs:
        got, err = read_arm(io, g)
        if got is None:
            print("✗ %s 静止复检第二次读取失败：%s —— 停止" % (g, err))
            return 1
        d = [abs(a - b) * DEG for a, b in zip(before[g], got)]
        dmax = max(d)
        print("  %-10s 变化 %s  最大 %.4f°" % (g, [round(v, 4) for v in d], dmax))
        if dmax > STILL_EPS_DEG:
            print("  ✗ %s 在 %.0f s 内变化 %.4f° > %.3f° —— 仍被遥操作/拖动占用，"
                  "此刻下发必然 FAULT。零运动退出。" % (g, args.still_wait, dmax, STILL_EPS_DEG))
            return 1
    print("  ✓ 两次读数一致（均 ≤ %.3f°），手臂静止" % STILL_EPS_DEG)

    # ---------------- 以下才是真机运动 ----------------
    print("\n" + "=" * 78)
    print("开始下发关节指令。手放急停上。任一段回读异常我会立刻停，不重试。")
    print("=" * 78)

    record = {"ts": datetime.now().isoformat(timespec="seconds"), "backend": backend,
              "point": args.point, "teach_file": args.file,
              "args": vars(args), "arms": {}}
    t0 = time.time()

    for g, names, cur, span in jobs:
        n = max(1, int(math.ceil(max(span) / args.max_step_deg)))
        rows = []
        print("\n--- %s（%d 段）---" % (g, n))
        for i in range(1, n + 1):
            q = [cur[k] + (tgt_rad[k] - cur[k]) * i / n for k in range(7)]
            step = max(abs(tgt_rad[k] - cur[k]) for k in range(7)) / n * DEG
            tag = "segment %d/%d" % (i, n)

            res = io.set_joints(q, names, args.speed)
            if not res.get("success"):
                print("  ✗ %s 下发失败：%s" % (tag, res.get("message") or res))
                print("  已停止，**不做自动归位**。当前构型在段 %d/%d 与 %d/%d 之间，"
                      "请人工确认。" % (i - 1, n, i, n))
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                save(record, args.log_dir)
                return 1
            if args.settle > 0:
                time.sleep(args.settle)

            got, err = read_arm(io, g)
            if got is None:
                print("  ✗ %s 回读不可信：%s —— 状态不可判定，立刻停。" % (tag, err))
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                save(record, args.log_dir)
                return 1
            dev = max(abs(a - b) * DEG for a, b in zip(got, q))
            mj, mv = margin_deg(names, got)
            rows.append({"segment": i, "target_deg": deg_list(q), "actual_deg": deg_list(got),
                         "err_deg": round(dev, 2), "margin_deg": round(mv, 2),
                         "margin_joint": mj})
            print("  %s  步长 %5.2f°  回读偏差 %5.2f°  最紧余量 %5.2f° @ %s"
                  % (tag, step, dev, mv, mj))
            if dev > args.tol_deg:
                print("  ✗ %s 回读偏差 %.2f° > 容差 %.1f°，停止。" % (tag, dev, args.tol_deg))
                record["arms"][g] = {"ok": False, "at": tag, "rows": rows}
                save(record, args.log_dir)
                return 1

        got, err = read_arm(io, g)
        if got is None:
            print("  ✗ %s 终点复核读不到：%s" % (g, err))
            record["arms"][g] = {"ok": False, "at": "final", "rows": rows}
            save(record, args.log_dir)
            return 1
        dev = max(abs(a - b) * DEG for a, b in zip(got, tgt_rad))
        mj, mv = margin_deg(names, got)
        print("  ✓ %s 终点：%s  对 %s 偏差 %.2f°  最紧余量 %.2f° @ %s"
              % (g, deg_list(got), args.point, dev, mv, mj))
        record["arms"][g] = {"ok": dev <= args.tol_deg, "final_deg": deg_list(got),
                             "err_deg": round(dev, 2), "margin_deg": round(mv, 2), "rows": rows}
        if dev > args.tol_deg:
            save(record, args.log_dir)
            return 1

    record["elapsed_s"] = round(time.time() - t0, 2)
    path = save(record, args.log_dir)
    print("\n" + "=" * 78)
    print("全部到位（%.2f s）。停在 %s，可以接着跑该点的后续实验。日志：%s"
          % (record["elapsed_s"], args.point, path))
    print("=" * 78)
    return 0


def save(record, log_dir):
    try:
        os.makedirs(log_dir, exist_ok=True)
        p = os.path.join(log_dir, "goto_point_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
        with open(p, "w", encoding="utf-8") as fh:
            json.dump(record, fh, ensure_ascii=False, indent=2)
        return p
    except Exception as exc:  # noqa: BLE001
        print("  （日志写入失败：%s）" % exc)
        return "(未保存)"


if __name__ == "__main__":
    sys.exit(main())
