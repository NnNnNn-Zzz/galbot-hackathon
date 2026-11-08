#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二段流程：绿块两级对准 + 夹紧 + 抬回 1.327 m。

跑在「抬手程序」之后（即 run_merged_program.py 五步全部完成之后）。

    ① 腰高 → 1.3270 m
    ② scripts/align_box_1327.py --force      标准档案 1.327 的绿块视觉对准
    ③ 腰高 → 1.2337 m（竖直下降）
    ④ scripts/align_box_1234.py --force      标准档案 1.234 的绿块视觉对准
    ⑤ 腰高 → 1.125  m（继续竖直下降）
    ⑥ 夹爪抓紧到 7 cm                        scripts/gripper_ctrl.py --seq 7
    ⑦ 腰高 → 1.3270 m（竖直上升）

★ 为什么第①步要先把腰高设到 1.327 再跑 align_box_1327
--------------------------------------------------------
`align_box_1327.py` 的标准档案（画面中心/长边/倾角）是在腰高 1.327 m 采的，
腰高不对时"画面里绿块该在哪"这件事本身就变了 —— 在 1.3904 m 强跑，等于让手臂
去追一个不存在的构图，越追越偏。脚本自己也会拒绝（差 >2 cm 就退），
`--force` 只是容差保险，不是"换个高度也能对"。
所以顺序是「先到位、再对准」，第①步不能省。

★ 为什么升降都用 set_waist_height 而不是 keyboard_leg_height
--------------------------------------------------------------
`keyboard_leg_height.py` 是键盘遥控，靠字符流判定"还在按"，没法在编排里定到某个
高度；`set_waist_height.py` 走的是同一个 `set_leg_height`（腰部直上直下、不产生
偏航/滚转副作用），但带「回读闭环 + 最多 3 次补发」，正好能"降到 1.234 附近"。

★ 升降行程按长度放大 duration
------------------------------
1.3904 → 1.2337 是 157 mm，用默认 3 s 就是 5.2 cm/s。编排里默认给 5 s（≈3.1 cm/s），
更稳；需要更快就传 --duration。

用法
----
    cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 -u scripts/run_second_stage.py --dry-run    # 七步全空跑，不动机器人
    python3 -u scripts/run_second_stage.py --yes        # 真跑（ssh 一条命令必须带 --yes）
    python3 -u scripts/run_second_stage.py --yes --gripper-cm 6.5
    python3 -u scripts/run_second_stage.py --yes --skip 1,2   # 高度已就位、只跑后面

★ 七步全都支持空跑（set_waist_height / align_green_box 有 --dry-run，gripper_ctrl 有 --dry），
  所以 --dry-run 能完整验证整条调用链，这一点比第一段流程强。
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

#: 与 run_second_stage.py 同目录的子脚本
SCRIPTS = {
    1: ("set_waist_height.py", "腰高 → 1.3270 m（align_box_1327 的标准高度）"),
    2: ("align_box_1327.py", "绿块视觉对准 @1.327（--force）"),
    3: ("set_waist_height.py", "腰高 → 1.2337 m（竖直下降）"),
    4: ("align_box_1234.py", "绿块视觉对准 @1.234（--force）"),
    5: ("set_waist_height.py", "腰高 → 1.125 m（继续竖直下降）"),
    6: ("gripper_ctrl.py", "夹爪抓紧到 7 cm"),
    7: ("set_waist_height.py", "腰高 → 1.3270 m（竖直上升）"),
}


# ──────────────────────────────────────────────────────────────
# 公共（与 run_merged_program.py 刻意重复：不动那份已验证的脚本）
# ──────────────────────────────────────────────────────────────
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


def other_python_procs():
    system_hints = ("/usr/bin/python3", "/usr/share/", "/usr/local/bin/", "/opt/",
                    "nvpmodel_indicator", "jtop", "networkd", "pgrep")
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
        if pid in me or "run_second_stage.py" in cmd or "run_merged_program.py" in cmd:
            continue
        if any(h in cmd for h in system_hints):
            continue
        bad.append(line.strip())
    return bad


def run_step(idx, cmd, workdir, env, log_fp):
    print("")
    print("─" * 70)
    print("  [%d/7] %s" % (idx, SCRIPTS[idx][1]))
    print("  $ %s" % " ".join(shlex.quote(c) for c in cmd))
    print("─" * 70, flush=True)

    t0 = time.time()
    try:
        proc = subprocess.Popen(cmd, cwd=workdir, env=env,
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
    print("  → 返回码 %d，耗时 %.1f s" % (rc, dt), flush=True)
    return rc, dt


def confirm(prompt):
    try:
        ans = input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return ans in ("", "y", "yes")


def _tips(idx):
    if idx in (1, 3, 5, 7):
        return ["set_leg_height 会假 FAULT：脚本按回读 Z 判定 + 3 次补发，看最终高度就行",
                "高度没变且状态异常 → 可能被安全限制拒绝，先 probe_waist.py 只读确认",
                "行程大时把 --duration 调大（默认 5 s），别指望它瞬间到位"]
    if idx in (2, 4):
        return ["返回 3 常见于「腰高不符/画面里没找到绿块」—— 先 --dry-run 看差距",
                "腰高确实到位还报 3，检查相机是否被挡住、绿块是否还在画面里",
                "align 只看画面构图，不保证物理夹持位置；夹紧那步才是真实接触"]
    if idx == 6:
        return ["TIMEOUT 通常是夹住了绿块到不了 7 cm —— 看它打印的实测宽度",
                "抓空了会一路合到最小间距，实测宽度远小于 7 cm",
                "力度默认 10 N·m；绿块软就传 --gripper-effort 8"]
    return []


# ──────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="第二段：腰高1.327对准 → 降1.234对准 → 降1.125夹紧 → 升回1.327",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="步骤：1 高1.327 / 2 align@1327 / 3 降1.234 / 4 align@1234 / "
               "5 降1.125 / 6 夹爪7cm / 7 升1.327")
    ap.add_argument("--h-align-hi", type=float, default=1.3270,
                    help="高档对准腰高（米），默认 1.3270（align_box_1327 的标准）")
    ap.add_argument("--h-align-lo", type=float, default=1.2337,
                    help="低档对准腰高（米），默认 1.2337（align_box_1234 的标准）")
    ap.add_argument("--h-grip", type=float, default=1.125,
                    help="夹爪动作高度（米），默认 1.125")
    ap.add_argument("--gripper-cm", type=float, default=7.0, help="夹爪目标开度（cm），默认 7")
    ap.add_argument("--gripper-effort", type=float, default=10.0,
                    help="夹爪力度 N·m，默认 10")
    ap.add_argument("--duration", type=float, default=5.0,
                    help="每次升降的运动时长（秒），默认 5（比 SDK 默认 3 s 慢，更稳）")
    ap.add_argument("--no-force-align", action="store_true",
                    help="不给 align 加 --force（默认加，按用户要求）")
    ap.add_argument("--skip", default="", help="跳过步骤，逗号分隔，如 \"1,2\"")
    ap.add_argument("--dry-run", action="store_true", help="七步全空跑，不动机器人")
    ap.add_argument("--yes", action="store_true", help="免确认；非交互环境真跑必须带")
    ap.add_argument("--force", action="store_true", help="不打印残留进程提示")
    ap.add_argument("--log", default="", help="全过程输出同时落盘")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印环境变量")
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
    dry = args.dry_run
    force_align = not args.no_force_align

    def script(name):
        p = os.path.join(HERE, name)
        if not os.path.exists(p):
            alt = os.path.expanduser("~/" + name)
            if os.path.exists(alt):
                return alt
        return p

    def height_cmd(target):
        c = [py, "-u", script("set_waist_height.py"), "--target", "%.4f" % target,
             "--duration", "%.1f" % args.duration]
        if dry:
            c.append("--dry-run")
        return c

    def align_cmd(name):
        c = [py, "-u", script(name)]
        if force_align:
            c.append("--force")
        if dry:
            c.append("--dry-run")
        return c

    cmds = {
        1: height_cmd(args.h_align_hi),
        2: align_cmd("align_box_1327.py"),
        3: height_cmd(args.h_align_lo),
        4: align_cmd("align_box_1234.py"),
        5: height_cmd(args.h_grip),
        6: [py, "-u", script("gripper_ctrl.py"), "--effort", "%.1f" % args.gripper_effort,
            "--seq", "%.0f" % args.gripper_cm] + (["--dry"] if dry else []),
        7: height_cmd(args.h_align_hi),
    }

    # ── 预检 ────────────────────────────────────────────────
    print("")
    print("=" * 70)
    print("  第二段：绿块两级对准 + 夹紧 + 抬回 1.327 m")
    print("=" * 70)
    print("  工作目录：%s" % workdir)
    print("  Python  ：%s" % py)
    if args.verbose:
        print("  PYTHONPATH      = %s" % env.get("PYTHONPATH", ""))
        print("  LD_LIBRARY_PATH = %s" % env.get("LD_LIBRARY_PATH", ""))
    print("  模式    ：%s" % ("★ dry-run（不动机器人，七步全覆盖）" if dry else "真机运行"))

    missing = []
    for idx in range(1, 8):
        if idx in skip:
            continue
        name = SCRIPTS[idx][0]
        p = script(name)
        if not os.path.exists(p):
            missing.append("%s（步骤 %d）" % (name, idx))
    # align_box_* 依赖同目录的 align_green_box.py
    for idx in (2, 4):
        if idx in skip:
            continue
        dep = os.path.join(HERE, "align_green_box.py")
        if not os.path.exists(dep):
            missing.append("align_green_box.py（步骤 %d 依赖）" % idx)
    if missing:
        print("")
        print("  ✘ 缺文件，先补齐：", file=sys.stderr)
        for m in missing:
            print("     - %s" % m, file=sys.stderr)
        return 2

    print("")
    print("  步骤安排：")
    for idx in range(1, 8):
        mark = "  [跳过]" if idx in skip else ""
        print("    %d. %s%s" % (idx, SCRIPTS[idx][1], mark))
    print("")
    print("  腰高轨迹：当前 → %.4f → %.4f → %.4f → %.4f"
          % (args.h_align_hi, args.h_align_lo, args.h_grip, args.h_align_hi))
    print("  升降时长：每次 %.1f s（--duration）" % args.duration)
    print("  align 参数：%s" % ("--force（按用户要求）" if force_align else "不加 --force"))

    busy = other_python_procs()
    if busy and not args.force:
        print("")
        print("  ⚠ 有 %d 个项目脚本进程正在跑 —— SDK 同一时间只允许一个进程碰：" % len(busy))
        for line in busy[:8]:
            print("       %s" % line)
        if len(busy) > 8:
            print("       ...（还有 %d 个）" % (len(busy) - 8))

    if not dry and not args.yes:
        if not sys.stdin.isatty():
            print("")
            print("  ✘ 非交互环境（stdin 不是终端），没法等你确认。", file=sys.stderr)
            print("     真跑请显式加 --yes；想先看计划就加 --dry-run。", file=sys.stderr)
            return 2
        print("")
        if not confirm("  以上计划会真的动机器人（升降 + 视觉对准 + 夹爪）。确认开始？"
                       "(y / 回车=开始，q=退出) > "):
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
        for idx in range(1, 8):
            if idx in skip:
                results.append((idx, None, 0.0))
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
            print("  %d. %-42s  跳过" % (idx, SCRIPTS[idx][1][:42]))
        elif r == 0:
            print("  %d. %-42s  ✓  %.1f s" % (idx, SCRIPTS[idx][1][:42], dt))
        else:
            print("  %d. %-42s  ✘ 返回码 %d（%.1f s）"
                  % (idx, SCRIPTS[idx][1][:42], r, dt))
    print("  总耗时 %.1f s" % (time.time() - t_all))

    if rc != 0:
        return rc
    if dry:
        print("\n  dry-run 七步全走完，机器人没有动。去掉 --dry-run 就是真跑。")
    else:
        print("\n  七步全部完成：绿块在 1.234 处对准后夹紧，已抬回 1.327 m。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
