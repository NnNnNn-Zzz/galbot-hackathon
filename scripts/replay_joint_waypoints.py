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

★ 关于「丝滑」（2026-11-08 提速）
----------------------------------
原来每步是「阻塞到位 → wait 0.25 s → 回读 7 个关节」，101 步光等待就 25 s，
加上每步都要完整加速-减速-停住，整体两分钟往上，看起来一顿一顿的。

三个改动：
1. **步数减半**：关节空间是直线插值，分 8 步还是 54 步走的是**同一条轨迹**，
   变的只是启停次数。步长 8°→15°，总步数 101 降到约 54，启停少一半。
   加减速斜坡是每步的固定开销，步数少 = 斜坡开销少 = 看起来连续。
2. **去掉每步的固定等待**：`is_blocking=True` 返回就意味着已经到位，
   再 sleep 0.25 s 纯属白等。默认改成 0.03 s（只给状态回传留一点余量）。
3. **每步回读 → 每 N 步校正**：原来每步读两次 7 关节，101 步 = 200 次查询。
   改成段首读一次、段内信任 is_blocking 的返回值、每 5 步用实测校正一次漂移、
   段末做一次总校验。查询次数从 200 降到约 25。

风险控制（提速之后仍然守住）：
- 状态非 SUCCESS 才回读确认（G-22 假成功类问题照样能抓）
- 每 --verify-every 步用实测值覆盖命令值，防止误差累积
- 段末残差超过 --end-tol 就停，不带着偏差往下一段走
- 步数仍有硬上限，不会无限循环

安全设计
--------
1. 每步下发前按比例插值，末步自然收敛到目标。
2. 单步 is_blocking + 状态检查 + 周期回读校验。
3. 段末回读与目标比对，超限立即停。
4. 只下发 right_arm_joint1..7，绝不碰 leg/chassis/gripper。

用法
----
    python3 scripts/replay_joint_waypoints.py --dry-run          # 只看计划
    python3 scripts/replay_joint_waypoints.py                    # 默认 smooth（推荐）
    python3 scripts/replay_joint_waypoints.py --profile safe      # 回到旧的保守参数
    python3 scripts/replay_joint_waypoints.py --profile fast      # 更快（步长 25°）
    python3 scripts/replay_joint_waypoints.py --step-deg 20 --speed 0.4   # 手动调
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

#: 预设。smooth 是默认；safe 等价于 2026-11-08 之前的原始参数，想复现旧行为就选它。
PROFILES = {
    "smooth": {"step_deg": 15.0, "speed": 0.35, "dwell": 0.03, "end_tol": 3.5},
    "safe": {"step_deg": 8.0, "speed": 0.15, "dwell": 0.25, "end_tol": 3.0},
    "fast": {"step_deg": 25.0, "speed": 0.50, "dwell": 0.0, "end_tol": 4.0},
}

#: 低于这个角度的关节请求，SDK 会直接回 FAULT 且一动不动（2026-11-08 实测 2.23° 就触发）。
#: 所以「请求本身就这么小」不能当失败 —— 那只是走到头了。
MIN_STEP_DEG = 3.0


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


def estimate_time(seg_deltas, step_max, speed, dwell):
    """粗估总耗时（秒）与步数：每步运动时间 + 一次启停余量 + dwell。"""
    total = 0.0
    steps = 0
    for max_d in seg_deltas:
        n = max(1, int(math.ceil(max_d / step_max)))
        steps += n
        per_step = max_d / max(0.01, speed) / n    # 该段平均每步的关节行程时间
        total += n * (per_step + 0.35 + dwell)     # 0.35 s ≈ 每步加减速斜坡
    return total, steps


def main() -> int:
    ap = argparse.ArgumentParser(description="回放关节采样序列（只动右臂）")
    ap.add_argument("--seq", default=DEFAULT_SEQ, help="序列文件")
    ap.add_argument("--order", default="", help="显式回放顺序，逗号分隔（默认倒序：末点→首点）")
    ap.add_argument("--profile", default="smooth", choices=sorted(PROFILES),
                    help="参数预设：smooth(默认，快且连续) / safe(旧参数) / fast(更激进)")
    ap.add_argument("--step-deg", type=float, default=None,
                    help="单步最大关节变化（度）；不给就用 profile 的值")
    ap.add_argument("--speed", type=float, default=None,
                    help="关节速度 rad/s；不给就用 profile 的值")
    ap.add_argument("--dwell", type=float, default=None,
                    help="每步后额外等待（秒）；不给就用 profile 的值")
    ap.add_argument("--end-tol", type=float, default=None,
                    help="段末残差容差（度）；不给就用 profile 的值")
    ap.add_argument("--verify-every", type=int, default=5,
                    help="段内每 N 步用实测校正一次（0=只在段末校验），默认 5")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不下发")
    args = ap.parse_args()

    prof = PROFILES[args.profile]
    step_deg = args.step_deg if args.step_deg is not None else prof["step_deg"]
    speed = args.speed if args.speed is not None else prof["speed"]
    dwell = args.dwell if args.dwell is not None else prof["dwell"]
    end_tol = args.end_tol if args.end_tol is not None else prof["end_tol"]

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

    step_max = math.radians(step_deg)
    print("")
    print("  回放顺序：%s" % " → ".join(order))
    print("  关节：%s" % ", ".join(names))
    print("  参数：profile=%s  单步≤%.1f°  %.2f rad/s(%.1f°/s)  dwell=%.2fs  "
          "每%d步校正  段末容差%.1f°"
          % (args.profile, step_deg, speed, math.degrees(speed), dwell,
             args.verify_every, end_tol))

    import galbot_sdk.g1 as g1m

    motion = g1m.GalbotMotion()
    if not motion.init():
        print("GalbotMotion.init() 失败", file=sys.stderr)
        return 1
    robot = g1m.GalbotRobot()
    if not robot.init(set()):
        print("GalbotRobot.init() 失败", file=sys.stderr)
        return 1

    try:
        cur = read_arm(robot, names)
        print("  当前：%s" % fmt(cur))

        # 先按"假设每段都到位"跑一遍，算计划与粗估
        seg_deltas = []
        probe = list(cur)
        for label in order:
            tgt = [math.radians(v) for v in by_label[label]["deg"]]
            seg_deltas.append(max(abs(t - c) for t, c in zip(tgt, probe)))
            probe = tgt
        est, n_steps = estimate_time(seg_deltas, step_max, speed, dwell)
        print("  预计：约 %d 步，粗估 %.0f s（%.1f 分钟）"
              % (n_steps, est, est / 60.0))
        print("")

        total_steps = 0
        for label in order:
            target = [math.radians(v) for v in by_label[label]["deg"]]
            cur = read_arm(robot, names)
            max_d = max(abs(t - c) for t, c in zip(target, cur))
            steps_needed = max(1, int(math.ceil(max_d / step_max)))
            print("  ── %s  目标 %s  最大差 %.2f°  需 %d 步"
                  % (label, fmt(target), math.degrees(max_d), steps_needed), flush=True)

            k_done = 0
            for k in range(MAX_STEPS_PER_SEG):
                delta = [t - c for t, c in zip(target, cur)]
                max_d = max(abs(x) for x in delta)
                if max_d <= TOL_RAD:
                    print("     已到位（残差 %.3f°）" % math.degrees(max_d))
                    break
                # 剩下的零头比 SDK 能执行的最小关节步还小 → 直接收，别发注定 FAULT 的指令
                if max_d <= math.radians(MIN_STEP_DEG):
                    print("     剩余 %.2f° < 最小有效步长 %.1f°，判为到位（不发指令）"
                          % (math.degrees(max_d), MIN_STEP_DEG))
                    break
                scale = min(1.0, step_max / max_d)
                cmd = [c + x * scale for c, x in zip(cur, delta)]

                if args.dry_run:
                    print("     步%02d  →  %s" % (k + 1, fmt(cmd)))
                    cur = cmd
                    total_steps += 1
                    if k + 1 >= 2:
                        print("     ... dry-run 每段只看前 2 步")
                        break
                    continue

                timeout = max(4.0, (max_d * scale) / max(0.01, speed) * 3.0)
                status = robot.set_joint_positions(
                    joint_positions=cmd,
                    joint_groups=[],
                    joint_names=names,
                    is_blocking=True,
                    speed_rad_s=speed,
                    timeout_s=timeout,
                )
                sname = _status_name(status)
                total_steps += 1
                k_done += 1

                if sname != "SUCCESS":
                    # ★ 小位移请求：SDK 对 <3° 的关节指令会直接 FAULT 且一动不动
                    #   （2026-11-08 实测 2.23° 触发）。这不是失败，是"已经走到头了"，
                    #   再重试也只会一直 FAULT，所以按到位收工。
                    if max_d * scale <= math.radians(MIN_STEP_DEG):
                        print("     [info] 剩余 %.2f° 小于最小有效步长 %.1f°，"
                              "SDK 不收这种小位移（状态 %s），判为到位"
                              % (math.degrees(max_d), MIN_STEP_DEG, sname))
                        break
                    # 只有异常才回读（G-22：状态可能撒谎，以实测为准）
                    time.sleep(0.12)
                    after = read_arm(robot, names)
                    err = max(abs(a - b) for a, b in zip(after, cmd))
                    if err <= math.radians(2.0):
                        print("     [info] 状态 %s，但回读已到命令位（差 %.2f°），继续"
                              % (sname, math.degrees(err)))
                        cur = after
                        continue
                    print("     [STOP] 状态 %s 且未到位（差 %.2f°）—— 停下，请人工确认"
                          % (sname, math.degrees(err)), file=sys.stderr)
                    return 2

                # 信任 is_blocking：返回 SUCCESS 即已到位，直接拿命令值当下一步起点
                cur = cmd
                if dwell > 0:
                    time.sleep(dwell)

                # 周期性用实测校正，避免"命令值漂了但状态一直 SUCCESS"的累积误差
                if args.verify_every and k_done % args.verify_every == 0:
                    cur = read_arm(robot, names)
            else:
                print("     [STOP] %s 超过步数上限仍未到位" % label, file=sys.stderr)
                return 2

            if args.dry_run:
                continue

            # 段末总校验：这一段到底站没站到目标上
            after = read_arm(robot, names)
            err_end = max(abs(a - t) for a, t in zip(after, target))
            tag = "✓" if math.degrees(err_end) <= end_tol else "⚠"
            try:
                st, ee = motion.get_end_effector_pose_on_chain(
                    "right_arm", "EndEffector", "base_link")
                print("     %s 到位：%s  末端 xyz=(%.4f, %.4f, %.4f)  段末残差 %.2f°"
                      % (tag, fmt(after), float(ee[0]), float(ee[1]), float(ee[2]),
                         math.degrees(err_end)))
            except Exception:  # noqa: BLE001
                print("     %s 到位：%s  段末残差 %.2f°"
                      % (tag, fmt(after), math.degrees(err_end)))
            if math.degrees(err_end) > end_tol:
                print("     [STOP] 段末残差 %.2f° 超过容差 %.1f° —— 累积偏差过大，停下"
                      % (math.degrees(err_end), end_tol), file=sys.stderr)
                return 2

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
