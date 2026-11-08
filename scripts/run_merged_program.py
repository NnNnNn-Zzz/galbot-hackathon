#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合并流程总编排（一条命令跑完五步）。

    ① 腰部回正          scripts/set_waist_upright.py       leg_joint4/5 → 0°
    ② 到抬手程序的腰高   scripts/set_waist_height.py        腰高取自 right_arm_sequence.json
    ③ run_sequence 全序列 ~/run_sequence.py                夹爪4cm → 摆姿态 → 视觉标定 →
                                                          前进 5×5cm → 夹爪2cm → 拉出 27cm → 6cm
    ④ 张爪 + 后退 15 cm  scripts/gripper_open_retreat.py    夹爪开到最大，沿夹爪轴退 15 cm
    ⑤ 抬手程序          scripts/replay_joint_waypoints.py   右臂 7 关节分步回放（默认倒序）

★ 为什么用「分进程串行」而不是把逻辑内联进一个进程
------------------------------------------------------
每个子脚本都自建 `gm.GalbotMotion()` / `gm.GalbotRobot()`，跑完在 finally 里
`request_shutdown / wait_for_shutdown / destroy`。SDK 单例**destroy 之后在同一进程里
重新 init 会失败** —— 2026-11-07 架构合并时踩过双单例同进程互踩（wrapper 与 dev
adapter 各持一套），代价是跑一次崩一次。所以这里用 subprocess 逐条串行：
每步开一个干净进程，跑完销毁，天然隔离 SDK 生命周期。
好处是这些脚本各自真机验证过的行为**一字不改**地保留下来，
合并只是把调用顺序和前置条件固定住，不重新发明一遍运动逻辑。

★ 顺序为什么必须是「先回正、再设高、后序列」
------------------------------------------------
- 腰的偏航/侧倾（leg_joint4/5）回正会改变 head_base_link 的高度读数，
  所以**必须先回正、再设腰高**（反过来设完高度会被回正动作带偏）。
- 第③步 run_sequence 会把末端姿态摆成 Rx(-90°)（夹爪水平向前 + 内旋 90°），
  并沿这个轴拉出 27 cm。第④步紧接着沿同一个轴再退 15 cm，
  所以是**接近纯水平**的退 —— 姿态竖直时那种"后退变升降"的情形不会出现。
- 抬手程序只下发 right_arm_joint1..7 的关节角，不动腰。它的采样序列是在
  「腰高 1.3904 m」这个前提下录的，所以前面必须把腰摆回去，否则同样的关节角
  末端位置对不上。

用法
----
    cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 -u scripts/run_merged_program.py --dry-run     # 全程空跑，不动机器人
    python3 -u scripts/run_merged_program.py               # 真跑（交互终端才会等你确认）
    python3 -u scripts/run_merged_program.py --yes         # 真跑且免确认（ssh 一条命令必须带）
    python3 -u scripts/run_merged_program.py --skip 1,2     # 跳过前两步（腰已就位时）
    python3 -u scripts/run_merged_program.py --retreat 0.10 --waist-height 1.405
    python3 -u scripts/run_merged_program.py --sequence-args "--pull 0.20 --fwd 4"

★ 第③步是既有的 ~/run_sequence.py（不在 scripts/ 里，和 wrist_align.py 同住 home）。
  它**没有 dry-run 模式，一跑就会动**，所以 --dry-run 时空跑会跳过这一步。
  给它调参走 --sequence-args，别改 run_sequence.py 本身。

★ 非交互环境（Windows 侧 ssh 一条命令）**必须显式 --yes** 才会真跑。
  不会因为"读不到 stdin"就默默放行 —— 那样用户以为程序在等他确认，
  实际手臂已经抬了 15 cm。
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SEQ = os.path.expanduser("~/joint_records/right_arm_sequence.json")
DEFAULT_BASELINE = os.path.expanduser("~/joint_records/waist_baseline.json")
SDK_PY = os.path.expanduser("~/g1_runtime")
SDK_LIB = "/data/galbot/lib"

#: 第③步用的既有脚本。它住在机器人 home 而不是 scripts/，
#: 因为它 `import wrist_align` 并要求同目录有 wrist_align.py + 基准图 ~/ref_handle.jpg。
DEFAULT_SEQUENCE_SCRIPT = os.path.expanduser("~/run_sequence.py")

#: 与 run_merged_program.py 同目录的子脚本（第③步不在此列，单独校验）
STEP_SCRIPTS = {
    1: "set_waist_upright.py",
    2: "set_waist_height.py",
    4: "gripper_open_retreat.py",
    5: "replay_joint_waypoints.py",
}
STEP_TITLES = {
    1: "腰部回正（leg_joint4 偏航 / leg_joint5 侧倾 → 0°）",
    2: "把腰高设到抬手程序规定的值",
    3: "run_sequence 全序列（姿态 + 视觉标定 + 前进 + 拉出 27 cm）",
    4: "夹爪张到最大 + 沿夹爪朝向后退",
    5: "抬手程序（右臂 7 关节回放，默认倒序）",
}


# ──────────────────────────────────────────────────────────────
# 环境 / 路径
# ──────────────────────────────────────────────────────────────
def find_workdir():
    """子进程的 cwd —— 必须是有 sdk_wrapper 的那个目录（否则 import 失败）。"""
    cand = os.path.dirname(HERE)
    if os.path.isdir(os.path.join(cand, "sdk_wrapper")):
        return cand
    for alt in ("~/galbot_hackathon_dev", "~/galbot_hackathon"):
        p = os.path.expanduser(alt)
        if os.path.isdir(os.path.join(p, "sdk_wrapper")):
            return p
    return cand


def prepare_env():
    """把 env_hpu.sh 该做的事补上，这样忘了 source 也能跑。"""
    env = dict(os.environ)
    pp = env.get("PYTHONPATH", "")
    if SDK_PY not in [p for p in pp.split(os.pathsep) if p]:
        env["PYTHONPATH"] = SDK_PY + (os.pathsep + pp if pp else "")
    lp = env.get("LD_LIBRARY_PATH", "")
    if SDK_LIB not in [p for p in lp.split(os.pathsep) if p]:
        env["LD_LIBRARY_PATH"] = SDK_LIB + (os.pathsep + lp if lp else "")
    # 子进程输出立刻落盘，不攒缓冲（卡住时能看到走到哪一步）
    env["PYTHONUNBUFFERED"] = "1"
    return env


def other_python_procs():
    """找**可能占用 SDK** 的其它 python3 进程。

    机器人上常驻一堆系统服务（nvpmodel_indicator / jtop / networkd-dispatcher），
    它们不碰 SDK，必须排除 —— 否则预检每次都误报，人会条件反射地加 --force，
    这层保护就废了。真正危险的是**项目脚本**（keyboard_leg_height.py、
    gripper_ctrl.py、robot_side/server.py 这类），它们一 init 就把 SDK 占住。
    """
    system_hints = (
        "/usr/bin/python3", "/usr/share/", "/usr/local/bin/", "/opt/",
        "nvpmodel_indicator", "jtop", "networkd", "pgrep",
    )
    try:
        out = subprocess.run(["pgrep", "-af", "python3"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return []
    me = {os.getpid(), os.getppid()}
    bad = []
    for line in out.splitlines():
        parts = line.split(None, 1)
        if len(parts) < 2 or not parts[0].isdigit():
            continue
        pid, cmd = int(parts[0]), parts[1]
        if pid in me or "run_merged_program.py" in cmd:
            continue
        if any(h in cmd for h in system_hints):
            continue
        bad.append(line.strip())
    return bad


# ──────────────────────────────────────────────────────────────
# 腰高来源
# ──────────────────────────────────────────────────────────────
def read_waist_height(seq_path, baseline_path):
    """确定"抬手程序规定的腰高"。

    优先级：序列文件里首点的 waist.height_m（= 录制起点，也是倒序回放的终点）
            → 腰高基线文件 height_m。
    同时把所有采样点的腰高汇总出来，不一致就提醒 —— 跨腰高混拼轨迹会让回放失真。
    """
    seq_path = os.path.expanduser(seq_path)
    if os.path.exists(seq_path):
        try:
            with open(seq_path, encoding="utf-8") as fh:
                data = json.load(fh)
        except Exception as exc:  # noqa: BLE001
            print("  ⚠ 序列文件读不了（%s）：%s" % (seq_path, exc), file=sys.stderr)
            data = None
        if data:
            rows = []
            for ent in data.get("entries") or []:
                w = ent.get("waist") or {}
                h = w.get("height_m")
                if h is not None:
                    rows.append((ent.get("label") or "?", float(h)))
            if rows:
                uniq = sorted({round(h, 4) for _l, h in rows})
                first_label, first_h = rows[0]
                desc = ("序列 %s 首点 %s = %.4f m（共 %d 点）"
                        % (os.path.basename(seq_path), first_label, first_h, len(rows)))
                if len(uniq) > 1:
                    detail = "  ".join("%s→%.4f" % (l, h) for l, h in rows)
                    print("  ⚠ 序列内各点腰高不一致（%s），取首点 %.4f m"
                          % ("/".join("%.4f" % v for v in uniq), first_h),
                          file=sys.stderr)
                    print("    明细：%s" % detail, file=sys.stderr)
                    print("    （跨腰高混拼轨迹时，回放到别的点末端位置对不上，属正常现象）",
                          file=sys.stderr)
                return first_h, desc

    baseline_path = os.path.expanduser(baseline_path)
    if os.path.exists(baseline_path):
        with open(baseline_path, encoding="utf-8") as fh:
            base = json.load(fh)
        h = float(base["height_m"])
        return h, "腰高基线 %s「%s」= %.4f m（%s）" % (
            os.path.basename(baseline_path),
            base.get("alias_cn", base.get("name", "")),
            h, base.get("recorded_at", ""))

    return None, "找不到序列文件和基线文件"


# ──────────────────────────────────────────────────────────────
# 执行
# ──────────────────────────────────────────────────────────────
def run_step(idx, cmd, workdir, env, log_fp):
    print("")
    print("─" * 70)
    print("  [%d/5] %s" % (idx, STEP_TITLES[idx]))
    print("  $ %s" % " ".join(shlex.quote(c) for c in cmd))
    print("─" * 70, flush=True)

    t0 = time.time()
    try:
        # ★ stdin=DEVNULL：子脚本若有 input() 就立即 EOF 走自己的兜底分支，
        #   不会因为读完终端而永久挂住（align_green_box 踩过这个坑）。
        proc = subprocess.Popen(
            cmd, cwd=workdir, env=env, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
    except Exception as exc:  # noqa: BLE001
        print("  ✘ 起不来：%s" % exc, file=sys.stderr)
        return 127, 0.0

    assert proc.stdout is not None
    cancelled = False
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        if log_fp is not None:
            log_fp.write(line)
            log_fp.flush()
        if "已取消" in line:   # 只认取消关键字：dry-run 也会打「未下发」，别误伤
            cancelled = True
    rc = proc.wait()
    dt = time.time() - t0
    if cancelled and rc == 0:
        print("  ✘ 子脚本报了「已取消 / 未下发」但退出码是 0 —— 这一步实际什么都没做",
              file=sys.stderr)
        rc = 90
    print("  → 返回码 %d，耗时 %.1f s" % (rc, dt), flush=True)
    return rc, dt


def confirm(prompt):
    """真机动作前的一次确认（只在交互终端里问）。"""
    try:
        ans = input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return ans in ("", "y", "yes")


# ──────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="合并流程：腰部回正 → 腰高 → run_sequence → 张爪后退 15 cm → 抬手程序",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="步骤编号：1 腰部回正 / 2 腰高 / 3 run_sequence / 4 张爪后退 / 5 抬手程序")
    ap.add_argument("--waist-height", type=float, default=None,
                    help="显式指定腰高（米）；默认从抬手程序序列里读")
    ap.add_argument("--seq", default=DEFAULT_SEQ, help="抬手程序的序列文件")
    ap.add_argument("--baseline", default=DEFAULT_BASELINE, help="腰高基线文件（回退用）")
    ap.add_argument("--retreat", type=float, default=0.15,
                    help="第④步沿夹爪后退距离（米），默认 0.15")
    ap.add_argument("--open-cm", type=float, default=12.0,
                    help="第④步夹爪目标开度（cm），默认 12")
    ap.add_argument("--sequence-script", default=DEFAULT_SEQUENCE_SCRIPT,
                    help="第③步脚本，默认 ~/run_sequence.py")
    ap.add_argument("--sequence-args", default="",
                    help="给第③步的额外参数，如 \"--pull 0.20 --fwd 4\"")
    ap.add_argument("--order", default="",
                    help="第⑤步回放顺序（逗号分隔 label），默认倒序")
    ap.add_argument("--skip", default="", help="跳过步骤，逗号分隔，如 \"1,2\"")
    ap.add_argument("--dry-run", action="store_true", help="全程空跑，不动机器人")
    ap.add_argument("--yes", action="store_true", help="跳过真机动作前的确认")
    ap.add_argument("--force", action="store_true", help="忽略「有别的 python3 在跑」的拦截")
    ap.add_argument("--log", default="", help="把全过程输出同时写到文件")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印环境变量等细节")
    args = ap.parse_args()

    skip = set()
    for tok in args.skip.split(","):
        tok = tok.strip()
        if tok:
            try:
                skip.add(int(tok))
            except ValueError:
                print("--skip 里有非法步骤号：%r" % tok, file=sys.stderr)
                return 2

    py = sys.executable or "python3"
    workdir = find_workdir()
    env = prepare_env()

    # ── 步骤清单 ────────────────────────────────────────────
    dry = args.dry_run
    cmd1 = [py, "-u", os.path.join(HERE, STEP_SCRIPTS[1])]
    if dry:
        cmd1.append("--dry-run")

    waist_h, waist_src = (args.waist_height, "命令行 --waist-height 指定") \
        if args.waist_height is not None else read_waist_height(args.seq, args.baseline)

    cmd2 = [py, "-u", os.path.join(HERE, STEP_SCRIPTS[2])]
    if waist_h is not None:
        cmd2 += ["--target", "%.4f" % waist_h]
    else:
        cmd2.append("--from-baseline")
    if dry:
        cmd2.append("--dry-run")

    # 第③步：既有的 run_sequence.py。
    # 它**没有 dry-run 模式**，一跑就动，所以 dry-run 时整步跳过（见下面执行循环）。
    # 非交互环境必须给 --yes，否则内部的 input() 会 EOF 直接退出。
    seq_script = os.path.expanduser(args.sequence_script)
    cmd3 = [py, "-u", seq_script, "--yes"]
    if args.sequence_args:
        cmd3 += args.sequence_args.split()

    cmd4 = [py, "-u", os.path.join(HERE, STEP_SCRIPTS[4]),
            "--retreat", "%.4f" % args.retreat,
            "--open-cm", "%.2f" % args.open_cm]
    if dry:
        cmd4.append("--dry-run")

    cmd5 = [py, "-u", os.path.join(HERE, STEP_SCRIPTS[5])]
    if args.order:
        cmd5 += ["--order", args.order]
    if dry:
        cmd5.append("--dry-run")

    cmds = {1: cmd1, 2: cmd2, 3: cmd3, 4: cmd4, 5: cmd5}

    # ── 预检 ────────────────────────────────────────────────
    print("")
    print("=" * 70)
    print("  合并流程：腰回正 → 腰高 → run_sequence → 张爪后退 → 抬手程序")
    print("=" * 70)
    print("  工作目录：%s" % workdir)
    print("  Python  ：%s" % py)
    if args.verbose:
        print("  PYTHONPATH      = %s" % env.get("PYTHONPATH", ""))
        print("  LD_LIBRARY_PATH = %s" % env.get("LD_LIBRARY_PATH", ""))
    print("  模式    ：%s" % ("★ dry-run（不动机器人）" if dry else "真机运行"))

    missing = []
    for idx, name in STEP_SCRIPTS.items():
        if idx in skip:
            continue
        p = os.path.join(HERE, name)
        if not os.path.exists(p):
            missing.append("%s（步骤 %d，应与本脚本同目录）" % (name, idx))

    # 第③步：run_sequence.py 及其两个同目录依赖
    if 3 not in skip:
        if not os.path.exists(seq_script):
            missing.append("%s（步骤 3，可用 --sequence-script 指定）" % seq_script)
        else:
            dep_dir = os.path.dirname(seq_script) or "."
            for dep in ("wrist_align.py", "ref_handle.jpg"):
                dp = os.path.join(dep_dir, dep)
                if not os.path.exists(dp):
                    missing.append("%s（步骤 3 依赖，run_sequence 会起不来/中止）" % dp)

    if missing:
        print("")
        print("  ✘ 缺少子脚本，先补齐再跑：", file=sys.stderr)
        for m in missing:
            print("     - %s" % m, file=sys.stderr)
        print("     提示：这些脚本要和 run_merged_program.py 放在**同一个目录**下。",
              file=sys.stderr)
        return 2

    print("")
    print("  步骤安排：")
    for idx in range(1, 6):
        if idx in skip:
            print("    %d. %-46s [跳过]" % (idx, STEP_TITLES[idx]))
            continue
        print("    %d. %s" % (idx, STEP_TITLES[idx]))
        if idx == 2:
            if waist_h is None:
                print("       腰高：%s ← 会失败，请先跑记录程序或指定 --waist-height"
                      % waist_src)
            else:
                print("       腰高：%.4f m   （来源：%s）" % (waist_h, waist_src))
        elif idx == 3:
            print("       脚本：%s" % seq_script)
            if args.sequence_args:
                print("       额外参数：%s" % args.sequence_args)
            if dry:
                print("       ⚠ 该脚本没有 dry-run 模式，本次空跑会跳过这一步")
        elif idx == 4:
            print("       张到 %.1f cm，然后沿夹爪轴退 %.1f cm"
                  % (args.open_cm, args.retreat * 100))
        elif idx == 5:
            print("       序列：%s%s"
                  % (args.seq, "  顺序：%s" % args.order if args.order else "  顺序：默认倒序"))

    if waist_h is None and 2 not in skip:
        print("")
        print("  ✘ 拿不到腰高，第②步必然失败。先跑 scripts/record_right_arm_joints.py，"
              "或用 --waist-height 直接给值。", file=sys.stderr)
        return 2

    # 残留进程检查：只警告不阻断。
    # 这台机器人上常年跑着队友的脚本 / 无线遥控服务，硬拦会逼人无脑加 --force，
    # 保护层就失效了。真要撞，SDK init 会自己报错，那才是硬证据。
    busy = other_python_procs()
    if busy and not args.force:
        print("")
        print("  ⚠ 有 %d 个项目脚本进程正在跑 —— SDK 同一时间只允许一个进程碰：" % len(busy))
        for line in busy[:8]:
            print("       %s" % line)
        if len(busy) > 8:
            print("       ...（还有 %d 个）" % (len(busy) - 8))
        print("     它们若已占住 SDK，下一步 init 会直接失败；届时先 `pgrep -af python3`")
        print("     找到并关掉（尤其 keyboard_leg_height.py / gripper_ctrl.py 这类会动腰和夹爪的）。")

    if not dry and not args.yes:
        if not sys.stdin.isatty():
            # 从 Windows 侧 ssh 一条命令跑时 stdin 不是终端。这时**不能**默默放行 ——
            # 用户很可能以为程序会停下来等他确认，结果手臂直接抬了 15 cm。
            print("")
            print("  ✘ 非交互环境（stdin 不是终端），没法等你确认。", file=sys.stderr)
            print("     真跑请显式加 --yes；想先看计划就加 --dry-run。", file=sys.stderr)
            return 2
        print("")
        if not confirm("  以上计划会真的动机器人。确认开始？(y / 回车=开始，q=退出) > "):
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
        for idx in range(1, 6):
            if idx in skip:
                results.append((idx, None, 0.0))
                continue
            if idx == 3 and dry:
                # run_sequence.py 没有 dry-run 模式，一跑就动，空跑只能整步跳过
                print("")
                print("─" * 70)
                print("  [3/5] %s" % STEP_TITLES[3])
                print("  ⏭ 跳过：run_sequence.py 没有 dry-run 模式（一跑就会动）")
                print("     空跑不覆盖这一步。想看它的计划，单独跑（它会真的动）：")
                print("       python3 -u %s" % seq_script)
                print("─" * 70, flush=True)
                results.append((idx, -1, 0.0))
                continue
            rc, dt = run_step(idx, cmds[idx], workdir, env, log_fp)
            results.append((idx, rc, dt))
            if rc != 0:
                print("")
                print("  ✘ 第 %d 步失败（返回码 %d），后续步骤不再执行。" % (idx, rc),
                      file=sys.stderr)
                print("    排查建议：", file=sys.stderr)
                for tip in _tips(idx):
                    print("      - %s" % tip, file=sys.stderr)
                break
    finally:
        if log_fp is not None:
            log_fp.close()

    # ── 汇总 ────────────────────────────────────────────────
    print("")
    print("=" * 70)
    print("  执行汇总")
    print("=" * 70)
    for idx, r, dt in results:
        if r is None:
            print("  %d. %-44s  跳过" % (idx, STEP_TITLES[idx][:44]))
        elif r == -1:
            print("  %d. %-44s  dry-run 跳过（该步无 dry-run 模式）"
                  % (idx, STEP_TITLES[idx][:44]))
        elif r == 0:
            print("  %d. %-44s  ✓  %.1f s" % (idx, STEP_TITLES[idx][:44], dt))
        else:
            print("  %d. %-44s  ✘ 返回码 %d（%.1f s）"
                  % (idx, STEP_TITLES[idx][:44], r, dt))
    print("  总耗时 %.1f s" % (time.time() - t_all))

    if rc != 0:
        return rc
    if args.dry_run:
        print("\n  dry-run 走完，机器人没有动。")
        print("  （第③步 run_sequence 没有 dry-run 模式，已整步跳过 —— 这一步只有真跑才验证得到）")
        print("  去掉 --dry-run 就是真跑。")
    else:
        print("\n  五步全部完成。")
        print("  提示：抬手程序结束后手臂停在序列首点；腰仍是回正 + 规定腰高，未再改动。")
    return 0


def _tips(idx):
    if idx == 1:
        return ["腰部回正报 FAULT 后没到位：先 probe_waist.py 只读看一眼，确认不是 ABNORMAL 模式",
                "单步 4° 已很慢，别再调快；超 40 步停下说明被限位挡住"]
    if idx == 2:
        return ["set_leg_height 会假 FAULT：以回读的 Z 为准，脚本已自带 3 次补发",
                "差几毫米不动属 G-22 类，检查目标是否已接近当前值"]
    if idx == 3:
        return ["视觉标定没完全收敛（返回 3）会打警告并继续，属既有设计；返回 1 才中止",
                "硬失败多是画面里找不到红把手 —— 先用 jog_ee.py 把把手摆进视野",
                "基准图固定 ~/ref_handle.jpg；换把手就换图：--sequence-args \"--ref 新图\"",
                "拉出 27 cm 那步历史上 6 步里 3 步首次 FAULT、重试后成功，属正常"]
    if idx == 4:
        return ["张爪 TIMEOUT 是顶到限位，属正常；实测宽度 <8 cm 才是真没张开",
                "后退失败会自动重试 3 次；仍失败看打印的起点/目标/当前，多半是姿态漂了"]
    if idx == 5:
        return ["单步超过步数上限未到位：检查序列是否与当前腰姿匹配",
                "回读与命令差 >3° 会主动停，别强推，先手动确认手臂没有干涉"]
    return []


if __name__ == "__main__":
    sys.exit(main())
