#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一条命令跑完整套：第一段 5 步 + 第二段 7 步 = 12 步。

    第一段（run_merged_program.py）
      ① 腰部回正              leg_joint4 偏航 / leg_joint5 侧倾 → 0°
      ② 腰高 → 1.3904 m       抬手程序序列首点的高度
      ③ run_sequence 全序列   ~/run_sequence.py（姿态+视觉标定+前进+拉出 27cm）
      ④ 张爪到最大 + 后退 15cm
      ⑤ 抬手程序              右臂 7 关节倒序回放

    第二段（run_second_stage.py）
      ⑥ 腰高 → 1.3270 m
      ⑦ 绿块对准 @1.327       align_box_1327.py --force
      ⑧ 腰高 → 1.2337 m（下降）
      ⑨ 绿块对准 @1.234       align_box_1234.py --force
      ⑩ 腰高 → 1.125 m（继续下降）
      ⑪ 夹爪抓紧到 7 cm
      ⑫ 腰高 → 1.3270 m（上升）

★ 为什么是「调两段」而不是复制成一份 12 步的大脚本
----------------------------------------------------
两段编排器都已经真机 dry-run 过、逻辑分别验证过（第一段还踩过 input 挂死的坑并修好）。
合并时再造一份 12 步的复制品，等于让两处逻辑各自演化、迟早对不上。
这里只做**编排的编排**：统一编号、统一参数、统一汇总，底层仍然调那两个脚本，
它们内部的每一步、每条排查建议、每个 dry-run 分支都原样生效。

★ 编号映射
----------
对外统一 1~12。传 --skip 时按编号自动分派：
  1~5  → 第一段的 --skip（原样）
  6~12 → 第二段的 --skip（减 5）
某一段的步全被跳过时，整段不启动（不会白跑一次 SDK 初始化）。

用法
----
    cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 -u scripts/run_full_program.py --dry-run          # 全链路空跑
    python3 -u scripts/run_full_program.py --yes              # 真跑（ssh 必须带 --yes）
    python3 -u scripts/run_full_program.py --yes --skip 1,2   # 腰已就位，从第③步开始
    python3 -u scripts/run_full_program.py --yes --only 2     # 只跑第二段
    python3 -u scripts/run_full_program.py --yes --first-args "--retreat 0.10"
    python3 -u scripts/run_full_program.py --yes --second-args "--gripper-cm 6.5"

★ 第③步 run_sequence 没有 dry-run 模式，`--dry-run` 时它会被跳过（第一段自己会标注）。
"""
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SDK_PY = os.path.expanduser("~/g1_runtime")
SDK_LIB = "/data/galbot/lib"

#: (段号, 段内步数, 脚本, 描述, 对外起始编号, 对外结束编号)
STAGES = [
    (1, 5, "run_merged_program.py",
     "第一段：腰回正 → 腰高 → run_sequence → 张爪后退 → 抬手程序", 1, 5),
    (2, 7, "run_second_stage.py",
     "第二段：绿块两级对准 → 夹紧 → 抬回 1.327", 6, 12),
]
TOTAL_STEPS = 12
STAGE_GAP_S = 2.0


def find_workdir():
    cand = os.path.dirname(HERE)
    if os.path.isdir(os.path.join(cand, "sdk_wrapper")):
        return cand
    for alt in ("~/galbot_hackathon_dev", "~/galbot_hackathon"):
        p = os.path.expanduser(alt)
        if os.path.isdir(os.path.join(p, "sdk_wrapper")):
            return p
    return cand


def prepare_env():
    env = dict(os.environ)
    pp = env.get("PYTHONPATH", "")
    if SDK_PY not in [p for p in pp.split(os.pathsep) if p]:
        env["PYTHONPATH"] = SDK_PY + (os.pathsep + pp if pp else "")
    lp = env.get("LD_LIBRARY_PATH", "")
    if SDK_LIB not in [p for p in lp.split(os.pathsep) if p]:
        env["LD_LIBRARY_PATH"] = SDK_LIB + (os.pathsep + lp if lp else "")
    env["PYTHONUNBUFFERED"] = "1"
    return env


def run_stage(stage_no, cmd, workdir, env, log_fp):
    print("")
    print("=" * 72)
    print("  【第 %d 段】%s" % (stage_no, STAGES[stage_no - 1][3]))
    print("  $ %s" % " ".join(shlex.quote(c) for c in cmd))
    print("=" * 72, flush=True)

    t0 = time.time()
    try:
        proc = subprocess.Popen(cmd, cwd=workdir, env=env,
                                stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1)
    except Exception as exc:  # noqa: BLE001
        print("  ✘ 起不来：%s" % exc, file=sys.stderr)
        return 127, 0.0

    assert proc.stdout is not None
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        if log_fp is not None:
            log_fp.write(line)
            log_fp.flush()
    rc = proc.wait()
    dt = time.time() - t0
    print("")
    print("  → 第 %d 段返回码 %d，耗时 %.1f s" % (stage_no, rc, dt), flush=True)
    return rc, dt


def confirm(prompt):
    try:
        ans = input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return ans in ("", "y", "yes")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="整套 12 步：第一段 5 步 + 第二段 7 步",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="编号：1腰回正 2腰高1.3904 3run_sequence 4张爪后退15cm 5抬手程序 | "
               "6腰高1.327 7对准@1327 8降1.234 9对准@1234 10降1.125 11夹爪7cm 12升1.327")
    ap.add_argument("--dry-run", action="store_true", help="全链路空跑（第③步会被跳过）")
    ap.add_argument("--yes", action="store_true", help="免确认；非交互环境真跑必须带")
    ap.add_argument("--skip", default="",
                    help="跳过步骤（1~12，逗号分隔），自动分派到所属段")
    ap.add_argument("--only", type=int, choices=[1, 2], default=None,
                    help="只跑某一段（1 或 2）")
    ap.add_argument("--first-args", default="", help="透传给第一段的额外参数")
    ap.add_argument("--second-args", default="", help="透传给第二段的额外参数")
    ap.add_argument("--force", action="store_true", help="透传 --force（不打印残留进程提示）")
    ap.add_argument("--log", default="", help="全过程输出同时落盘")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印环境变量")
    args = ap.parse_args()

    # ── 解析 --skip 并按段分派 ───────────────────────────────
    skip_all = set()
    for tok in args.skip.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            n = int(tok)
        except ValueError:
            print("--skip 里有非法步骤号：%r" % tok, file=sys.stderr)
            return 2
        if not (1 <= n <= TOTAL_STEPS):
            print("--skip 步骤号 %d 超出 1~%d" % (n, TOTAL_STEPS), file=sys.stderr)
            return 2
        skip_all.add(n)

    py = sys.executable or "python3"
    workdir = find_workdir()
    env = prepare_env()
    dry = args.dry_run

    plan = []  # [(stage_no, cmd, local_skips)]
    for stage_no, local_n, script_name, _desc, lo, hi in STAGES:
        if args.only is not None and args.only != stage_no:
            plan.append((stage_no, None, None))
            continue
        script = os.path.join(HERE, script_name)
        if not os.path.exists(script):
            alt = os.path.expanduser("~/" + script_name)
            if os.path.exists(alt):
                script = alt
            else:
                print("  ✘ 找不到 %s（第 %d 段）" % (script_name, stage_no), file=sys.stderr)
                return 2

        local_skips = sorted(n - lo + 1 for n in skip_all if lo <= n <= hi)
        if len(local_skips) >= local_n:
            # 整段都被跳过 → 连 SDK 初始化都不做
            plan.append((stage_no, None, local_skips))
            continue

        cmd = [py, "-u", script, "--yes"]
        if dry:
            cmd.append("--dry-run")
        if local_skips:
            cmd += ["--skip", ",".join(str(n) for n in local_skips)]
        if args.force:
            cmd.append("--force")
        extra = args.first_args if stage_no == 1 else args.second_args
        if extra:
            cmd += extra.split()
        plan.append((stage_no, cmd, local_skips))

    if all(c is None for _s, c, _k in plan):
        print("所有步骤都被跳过了，没什么可跑。", file=sys.stderr)
        return 2

    # ── 计划 ────────────────────────────────────────────────
    print("")
    print("=" * 72)
    print("  整套流程：12 步（第一段 5 + 第二段 7）")
    print("=" * 72)
    print("  工作目录：%s" % workdir)
    print("  Python  ：%s" % py)
    if args.verbose:
        print("  PYTHONPATH      = %s" % env.get("PYTHONPATH", ""))
        print("  LD_LIBRARY_PATH = %s" % env.get("LD_LIBRARY_PATH", ""))
    print("  模式    ：%s" % ("★ dry-run（不动机器人）" if dry else "真机运行"))

    print("")
    print("  步骤：")
    labels = [
        "腰部回正", "腰高 → 1.3904 m", "run_sequence 全序列",
        "张爪到最大 + 后退 15 cm", "抬手程序",
        "腰高 → 1.3270 m", "绿块对准 @1.327", "腰高 → 1.2337 m（下降）",
        "绿块对准 @1.234", "腰高 → 1.125 m（下降）", "夹爪抓紧到 7 cm",
        "腰高 → 1.3270 m（上升）",
    ]
    for i, lab in enumerate(labels, start=1):
        print("    %2d. %s%s" % (i, lab, "   [跳过]" if i in skip_all else ""))
    print("")
    print("  腰高轨迹：1.3904 → 1.3270 → 1.2337 → 1.1250 → 1.3270")

    if dry:
        print("  ⚠ 第③步 run_sequence 没有 dry-run 模式，空跑时会跳过（第一段会标注）")

    if not dry and not args.yes:
        if not sys.stdin.isatty():
            print("")
            print("  ✘ 非交互环境（stdin 不是终端），没法等你确认。", file=sys.stderr)
            print("     真跑请显式加 --yes；想先看计划就加 --dry-run。", file=sys.stderr)
            return 2
        print("")
        if not confirm("  以上 12 步会真的动机器人（升降 + 视觉伺服 + 夹爪 + 手臂回放）。"
                       "确认开始？(y / 回车=开始，q=退出) > "):
            print("  已取消。")
            return 1

    # ── 跑 ──────────────────────────────────────────────────
    log_fp = None
    if args.log:
        lp = os.path.expanduser(args.log)
        log_fp = open(lp, "w", encoding="utf-8")
        print("\n  日志同时写入：%s" % lp)

    results = []
    t_all = time.time()
    rc = 0
    try:
        for stage_no, cmd, local_skips in plan:
            if cmd is None:
                reason = "按 --only 跳过" if args.only is not None else "全部步骤已跳过"
                print("")
                print("=" * 72)
                print("  【第 %d 段】%s —— 跳过（%s）" % (stage_no, STAGES[stage_no - 1][3], reason))
                print("=" * 72, flush=True)
                results.append((stage_no, None, 0.0))
                continue
            rc, dt = run_stage(stage_no, cmd, workdir, env, log_fp)
            results.append((stage_no, rc, dt))
            if rc != 0:
                print("")
                print("  ✘ 第 %d 段失败（返回码 %d），后续段不再执行。" % (stage_no, rc),
                      file=sys.stderr)
                break
            time.sleep(STAGE_GAP_S)   # 给 SDK 收尾留点时间
    finally:
        if log_fp is not None:
            log_fp.close()

    # ── 汇总 ────────────────────────────────────────────────
    print("")
    print("=" * 72)
    print("  整套汇总")
    print("=" * 72)
    for stage_no, r, dt in results:
        _n, local_n, script_name, desc, lo, hi = STAGES[stage_no - 1]
        span = "步骤 %d~%d" % (lo, hi)
        if r is None:
            print("  第 %d 段（%s）  %-40s  跳过" % (stage_no, span, script_name))
        elif r == 0:
            print("  第 %d 段（%s）  %-40s  ✓  %.1f s" % (stage_no, span, script_name, dt))
        else:
            print("  第 %d 段（%s）  %-40s  ✘ 返回码 %d（%.1f s）"
                  % (stage_no, span, script_name, r, dt))
    print("  总耗时 %.1f s（%.1f 分钟）" % (time.time() - t_all, (time.time() - t_all) / 60.0))

    if rc != 0:
        return rc
    if dry:
        print("\n  dry-run 走完，机器人没有动（第③步 run_sequence 无 dry-run 模式，已跳过）。")
        print("  去掉 --dry-run 就是真跑。")
    else:
        print("\n  12 步全部完成。")
        print("  最终状态：绿块在 1.234 处对准后夹紧，手臂/腰部已抬回 1.327 m。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
