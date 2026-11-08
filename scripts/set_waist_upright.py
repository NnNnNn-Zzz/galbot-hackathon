#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把腰部「回正」：分小步把 leg_joint4（偏航）/ leg_joint5（侧倾）归零。

只动指定的 leg 关节，不动腰高（height 由 leg_joint1/2/3 解算，保持原样）。

安全设计
--------
1. 每步读实测角 → 算小步目标 → 下发 → 回读确认，位置不累积漂移。
2. 单步默认 4°（--step-deg 可调），速度默认 0.05 rad/s（很慢）。
3. 逐步只动一个关节，动完一个再动下一个，避免耦合。
4. 状态非 SUCCESS 时先回读：已到位就继续，没到位就停（参照 set_leg_height 会假 FAULT 的坑）。
5. 步数有硬上限，绝不会无限循环。
6. --dry-run 只打印计划，不发指令。

用法
----
    source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 scripts/set_waist_upright.py --dry-run     # 只看计划
    python3 scripts/set_waist_upright.py               # 执行
    python3 scripts/set_waist_upright.py --joints leg_joint4    # 只归零扭转
"""
from __future__ import annotations

import argparse
import math
import sys
import time

DEFAULT_JOINTS = ("leg_joint4", "leg_joint5")
MAX_STEPS_PER_JOINT = 40
STABLE_SLEEP_S = 0.25


def _status_name(value) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


def read_group(robot, names):
    states = list(robot.get_joint_states([], list(names)))
    return dict((st.joint_name, float(st.position)) for st in states)


def read_pose(robot):
    pose, _ts = robot.get_transform("base_link", "head_base_link", timestamp_ns=0, timeout_ms=500)
    qx, qy, qz, qw = (float(v) for v in pose[3:7])
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return ([float(v) for v in pose[:3]],
            [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])


def main() -> int:
    ap = argparse.ArgumentParser(description="腰部回正：把指定 leg 关节分小步归零")
    ap.add_argument("--joints", default=",".join(DEFAULT_JOINTS),
                    help="要归零的关节，逗号分隔（默认 leg_joint4,leg_joint5）")
    ap.add_argument("--target-deg", type=float, default=0.0, help="目标角度（度），默认 0")
    ap.add_argument("--step-deg", type=float, default=4.0, help="单步最大角度（度），默认 4")
    ap.add_argument("--speed", type=float, default=0.05, help="关节速度 rad/s，默认 0.05")
    ap.add_argument("--tol-deg", type=float, default=0.8, help="到位容差（度），默认 0.8")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不下发指令")
    args = ap.parse_args()

    names = [s.strip() for s in args.joints.split(",") if s.strip()]
    tol = math.radians(args.tol_deg)
    step_max = math.radians(args.step_deg)
    target = math.radians(args.target_deg)

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
        cur = read_group(robot, names)
        xyz, rpy = read_pose(robot)
        print("")
        print("  当前状态：")
        for n in names:
            print("    %-12s %9.3f°" % (n, math.degrees(cur[n])))
        print("    head_base_link xyz=(%.4f, %.4f, %.4f) m  rpy=(%.2f, %.2f, %.2f)°"
              % (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
        print("")
        print("  计划：%s → %.1f°，单步 ≤ %.1f°，速度 %.3f rad/s%s"
              % (", ".join(names), args.target_deg, args.step_deg, args.speed,
                 "（dry-run，不下发）" if args.dry_run else ""))
        print("")

        for name in names:
            done = 0
            while done < MAX_STEPS_PER_JOINT:
                cur = read_group(robot, [name])
                now = cur[name]
                err = target - now
                if abs(err) <= tol:
                    print("  %s 已到位：%.3f°" % (name, math.degrees(now)))
                    break
                step = max(-step_max, min(step_max, err))
                nxt = now + step
                print("  %s  %.3f° → %.3f°  (Δ%+.3f°)"
                      % (name, math.degrees(now), math.degrees(nxt), math.degrees(step)),
                      flush=True)
                if args.dry_run:
                    done += 1
                    if done >= 3:
                        print("  ... dry-run 只看前 3 步")
                        break
                    cur[name] = nxt
                    continue

                status = robot.set_joint_positions(
                    joint_positions=[nxt],
                    joint_groups=[],
                    joint_names=[name],
                    is_blocking=True,
                    speed_rad_s=args.speed,
                    timeout_s=10.0,
                )
                sname = _status_name(status)
                time.sleep(STABLE_SLEEP_S)
                after = read_group(robot, [name])[name]
                moved = abs(after - now) > 1e-4
                if sname != "SUCCESS":
                    if abs(target - after) <= tol:
                        print("    [info] 状态 %s，但回读已到位（%.3f°），继续"
                              % (sname, math.degrees(after)))
                        break
                    print("    [STOP] 状态 %s 且回读 %.3f° 未到位 —— 停下，请人工确认"
                          % (sname, math.degrees(after)), file=sys.stderr)
                    return 2
                if not moved:
                    print("    [STOP] 命令返回 SUCCESS 但关节没动（%.3f° 不变）—— 停下"
                          % math.degrees(after), file=sys.stderr)
                    return 2
                done += 1
            else:
                print("  [STOP] %s 超过 %d 步仍未到位" % (name, MAX_STEPS_PER_JOINT), file=sys.stderr)
                return 2

        if not args.dry_run:
            time.sleep(0.4)
            final = read_group(robot, names)
            xyz, rpy = read_pose(robot)
            print("")
            print("  回正后：")
            for n in names:
                print("    %-12s %9.3f°" % (n, math.degrees(final[n])))
            print("    head_base_link xyz=(%.4f, %.4f, %.4f) m  rpy=(%.2f, %.2f, %.2f)°"
                  % (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
            print("    腰高 Z = %.4f m" % xyz[2])
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
