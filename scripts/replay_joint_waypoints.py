#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按采样序列回放右臂关节轨迹（只动右臂 7 个关节，不动腰/底盘/夹爪）。

★ 本脚本在项目里的固定叫法：**抬手程序**（文氓 2026-11-08 命名）。
   听到「跑抬手程序」= 跑本文件；默认倒序回放（末点 → 首点）。

为什么要在关节空间插值
----------------------
两个采样点之间做「一次跳到底」= 99° 的关节突跳，机械臂会猛地扫一大圈。
本脚本按关节空间线性插值，把每段拆成多小步，每步所有关节**按同一比例**推进，
保证同时到达、每步最大关节变化不超过 --step-deg。

安全设计
--------
1. 每步下发前读实测角，按比例插值，末步自然收敛到目标。
2. 每步 is_blocking + 回读确认，累计误差超限就停。
3. 单步速度默认 0.15 rad/s，单步最大 8°。
4. 只下发 right_arm_joint1..7，绝不碰 leg/chassis/gripper。
5. 任一步状态异常且回读未到位 → 立即停，不再往下走。

用法
----
    python3 scripts/replay_joint_waypoints.py --dry-run          # 只看计划
    python3 scripts/replay_joint_waypoints.py                    # 默认 seq 末点 → 首点（倒序）
    python3 scripts/replay_joint_waypoints.py --order seq08,seq07,seq01
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

DEFAULT_SEQ = os.path.expanduser("~/joint_records/right_arm_sequence.json")
TOL_RAD = math.radians(0.3)
MAX_STEPS_PER_SEG = 60


def _status_name(value) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


def read_arm(robot, names):
    states = list(robot.get_joint_states([], list(names)))
    by = dict((st.joint_name, float(st.position)) for st in states)
    return [by[n] for n in names]


def fmt(vals):
    return "[" + ", ".join("%7.2f" % math.degrees(v) for v in vals) + "]"


def main() -> int:
    ap = argparse.ArgumentParser(description="回放关节采样序列（只动右臂）")
    ap.add_argument("--seq", default=DEFAULT_SEQ, help="序列文件")
    ap.add_argument("--order", default="", help="显式回放顺序，逗号分隔（默认倒序：末点→首点）")
    ap.add_argument("--step-deg", type=float, default=8.0, help="单步最大关节变化（度），默认 8")
    ap.add_argument("--speed", type=float, default=0.15, help="关节速度 rad/s，默认 0.15")
    ap.add_argument("--dwell", type=float, default=0.25, help="每步后稳定等待（秒），默认 0.25")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不下发")
    args = ap.parse_args()

    with open(args.seq, encoding="utf-8") as fh:
        seq = json.load(fh)
    names = seq["joint_order"]
    entries = seq["entries"]
    by_label = dict((e["label"], e) for e in entries)

    if args.order:
        order = [s.strip() for s in args.order.split(",") if s.strip()]
        missing = [l for l in order if l not in by_label]
        if missing:
            print("序列里没有这些点：%s" % missing, file=sys.stderr)
            return 2
    else:
        order = [e["label"] for e in reversed(entries)]

    step_max = math.radians(args.step_deg)
    print("")
    print("  回放顺序：%s" % " → ".join(order))
    print("  关节：%s" % ", ".join(names))

    import galbot_sdk.g1 as gm

    motion = gm.GalbotMotion()
    if not motion.init():
        print("GalbotMotion.init() 失败", file=sys.stderr)
        return 1
    robot = gm.GalbotRobot()
    if not robot.init(set()):
        print("GalbotRobot.init() 失败", file=sys.stderr)
        return 1

    try:
        cur = read_arm(robot, names)
        print("  当前：%s" % fmt(cur))
        print("")

        total_steps = 0
        for label in order:
            target = [math.radians(v) for v in by_label[label]["deg"]]
            max_d = max(abs(t - c) for t, c in zip(target, cur))
            steps_needed = max(1, int(math.ceil(max_d / step_max)))
            print("  ── %s  目标 %s  最大差 %.2f°  需 %d 步"
                  % (label, fmt(target), math.degrees(max_d), steps_needed), flush=True)

            for k in range(MAX_STEPS_PER_SEG):
                cur = read_arm(robot, names)
                delta = [t - c for t, c in zip(target, cur)]
                max_d = max(abs(x) for x in delta)
                if max_d <= TOL_RAD:
                    print("     已到位（残差 %.3f°）" % math.degrees(max_d))
                    break
                scale = min(1.0, step_max / max_d)
                cmd = [c + x * scale for c, x in zip(cur, delta)]
                print("     步%02d  →  %s" % (k + 1, fmt(cmd)), flush=True)

                if args.dry_run:
                    cur = cmd
                    total_steps += 1
                    if k + 1 >= 2:
                        print("     ... dry-run 每段只看前 2 步")
                        break
                    continue

                timeout = max(4.0, (max_d * scale) / max(0.01, args.speed) * 3.0)
                status = robot.set_joint_positions(
                    joint_positions=cmd,
                    joint_groups=[],
                    joint_names=names,
                    is_blocking=True,
                    speed_rad_s=args.speed,
                    timeout_s=timeout,
                )
                sname = _status_name(status)
                total_steps += 1
                time.sleep(args.dwell)
                after = read_arm(robot, names)
                err = max(abs(a - b) for a, b in zip(after, cmd))
                if sname != "SUCCESS":
                    if err <= math.radians(2.0):
                        print("     [info] 状态 %s，但回读已到命令位（差 %.2f°），继续"
                              % (sname, math.degrees(err)))
                        continue
                    print("     [STOP] 状态 %s 且未到位（差 %.2f°）—— 停下，请人工确认"
                          % (sname, math.degrees(err)), file=sys.stderr)
                    return 2
                if err > math.radians(3.0):
                    print("     [STOP] 回读与命令差 %.2f° 过大 —— 停下" % math.degrees(err),
                          file=sys.stderr)
                    return 2
            else:
                print("     [STOP] %s 超过步数上限仍未到位" % label, file=sys.stderr)
                return 2

            if not args.dry_run:
                cur = read_arm(robot, names)
                try:
                    st, ee = motion.get_end_effector_pose_on_chain(
                        "right_arm", "EndEffector", "base_link")
                    print("     到位：%s  末端 xyz=(%.4f, %.4f, %.4f)"
                          % (fmt(cur), float(ee[0]), float(ee[1]), float(ee[2])))
                except Exception:  # noqa: BLE001
                    print("     到位：%s" % fmt(cur))

        print("")
        print("  回放完成，共 %d 步" % total_steps)
        return 0
    finally:
        for n in ("request_shutdown", "wait_for_shutdown", "destroy"):
            f = getattr(robot, n, None)
            if callable(f):
                try:
                    f()
                except Exception:  # noqa: BLE001
                    pass


if __name__ == "__main__":
    sys.exit(main())
