#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把腰部高度（head_base_link 的 Z）调到指定值，小闭环校核。

已知坑（scripts/keyboard_leg_height.py 注释）：
    set_leg_height 可能返回 FAULT 但实际已经到位 → 所以每步都以**回读的 Z** 为准，
    只有「状态异常 且 实测没到位」才判定失败。

注意微小位移：4~5 mm 的改动可能被运动服务当成"无需动作"（G-22 类假成功），
所以这里改用「回读比对 + 最多 3 次补发」，而不是只看返回值。

用法
----
    python3 scripts/set_waist_height.py --target 1.3904
    python3 scripts/set_waist_height.py --target 1.3904 --dry-run
"""
from __future__ import annotations

import argparse
import sys
import time

SDK_MIN, SDK_MAX = 0.96, 1.54
TOL_M = 0.0015
MAX_TRIES = 3


def _status_name(value) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1]


def read_height(robot):
    pose, _ts = robot.get_transform("base_link", "head_base_link", timestamp_ns=0, timeout_ms=500)
    return float(pose[2])


def main() -> int:
    ap = argparse.ArgumentParser(description="设置腰部高度（head_base_link Z）")
    ap.add_argument("--target", type=float, required=True, help="目标腰高（米）")
    ap.add_argument("--duration", type=float, default=3.0, help="运动时长（秒），默认 3")
    ap.add_argument("--dry-run", action="store_true", help="只读当前值并打印计划")
    args = ap.parse_args()

    if not (SDK_MIN <= args.target <= SDK_MAX):
        print("目标 %.4f m 超出 SDK 支持区间 [%.2f, %.2f]" % (args.target, SDK_MIN, SDK_MAX),
              file=sys.stderr)
        return 2

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
        now = read_height(robot)
        print("")
        print("  当前腰高：%.4f m" % now)
        print("  目标腰高：%.4f m   差 %+.1f mm" % (args.target, (args.target - now) * 1000.0))
        if args.dry_run:
            print("  （dry-run，不下发）")
            return 0
        print("")

        for i in range(1, MAX_TRIES + 1):
            now = read_height(robot)
            err = args.target - now
            if abs(err) <= TOL_M:
                print("  已到位：%.4f m（误差 %+.1f mm）" % (now, err * 1000.0))
                return 0
            print("  第 %d 次下发 → %.4f m（当前 %.4f m，Δ%+.1f mm）"
                  % (i, args.target, now, err * 1000.0), flush=True)
            status = robot.set_leg_height(
                args.target, args.duration, is_blocking=True, timeout_s=args.duration + 8.0)
            sname = _status_name(status)
            time.sleep(0.5)
            after = read_height(robot)
            print("    状态 %s，回读 %.4f m（残余 %+.1f mm）"
                  % (sname, after, (args.target - after) * 1000.0))
            if abs(args.target - after) <= TOL_M:
                print("  已到位（以回读为准，状态 %s 不影响判定）" % sname)
                return 0
            if sname != "SUCCESS" and abs(after - now) < 1e-5:
                print("  [STOP] 状态 %s 且高度没变 —— 可能被安全限制拒绝，请人工确认" % sname,
                      file=sys.stderr)
                return 2

        final = read_height(robot)
        print("  [WARN] 尝试 %d 次后仍差 %.1f mm（%.4f m）"
              % (MAX_TRIES, (args.target - final) * 1000.0, final), file=sys.stderr)
        return 3
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
