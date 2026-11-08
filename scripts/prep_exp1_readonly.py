# -*- coding: utf-8 -*-
"""实验 1 上机准备的**只读**体检 —— 一条运动指令都不发。

为什么要有这个脚本：
  实验 1 是「空爪、零接触、全程悬空」的示教回放。它最可能失败的地方**不是**动作本身，
  而是下面这些"环境对不对"的问题 —— 它们一旦在现场才发现，就会白占一次真机窗口：

    ① 代码版本不对（HPU 上跑的是旧代码，缺 JNT_* 点位 / 缺 exp_* stage）
    ② 示教文件不是真机回读的那份（是备胎、是 mock 推算）
    ③ 相机标定常量对不上（R_corr / off 是单点标定，换了场地/换了批次就失效）
    ④ 当前关节角已经贴限位，第一条指令就会被闸门拦
    ⑤ 某一问的读数路径根本不通（读不到关节角 → UNDETERMINED）

  本脚本把这些全查一遍，输出 PASS / WARN / FAIL，**不发任何 set_joints / set_gripper /
  move_* / turn / speak**。纯只读。

用法（HPU 上，仓库根目录）：
    source scripts/env_hpu.sh
    python3 scripts/prep_exp1_readonly.py                     # 默认：只读查 + 打印上机方案
    python3 scripts/prep_exp1_readonly.py --json out.json      # 结果另存
    python3 scripts/prep_exp1_readonly.py --skip-hw            # 完全离线，不连 SDK
                                                                 （只查文件与配置，最安全）

退出码：0 = 全 PASS（可能有 WARN）；1 = 有 FAIL，不许上机。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

CONFIG = "exp1_teach_chain"
TEACH = "pipeline/configs/teach_drawer_real.json"

#: 实验 1 真机版链条上的点，顺序即执行顺序（**方向：CARRY → P2 → P1 → P0**）。
#: 2026-11-08 09:16 用户纠正方向后重排：原先按 CARRY→P0→P1→P2 生成的入口 ramp 会让
#: 末端在桌高横扫、刮到桌上的抽屉，已整批删除，换成 34 个 VIA_* 中间点。
CHAIN = ([f"VIA_CARRY_P2_{i}" for i in range(1, 18)]
         + ["P2_measured"]
         + [f"VIA_P2_P1_{i}" for i in range(1, 12)]
         + ["P1_measured"]
         + [f"VIA_P1_P0_{i}" for i in range(1, 7)]
         + ["P0_measured"])

#: 期望存在的点位前缀 —— 少一个就说明代码/数据版本不对
EXPECT_PREFIXES = {
    "VIA_CARRY_P2_": 17,
    "VIA_P2_P1_": 11,
    "VIA_P1_P0_": 6,
    "JNT_P2_ALIGN_": 5,
    "JNT_ALIGN_CARRY_": 6,
}

#: 示教文件里必须存在的 stage（缺了就说明 stage 模块没部署）
EXPECT_STAGES = ["exp_arm_snapshot", "exp_replay_chain", "exp_verify_chain", "preflight"]

rows = []


def add(level, key, msg, data=None):
    rows.append({"level": level, "item": key, "message": msg,
                 "data": data if data is not None else {}})
    print(f"[{level:<4}] {key:<26} {msg}", flush=True)


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- 1 代码/数据
def check_files():
    from scenes.scene_medicine_check.pipeline import runner
    from scenes.scene_medicine_check.pipeline.teach import load_teach, resolve_teach_path
    from scenes.scene_medicine_check.pipeline import registry

    # 1.1 配置能读、步数对
    try:
        cfg = runner.load_config(CONFIG)
    except Exception as exc:  # noqa: BLE001
        add("FAIL", "配置可读", f"读不到 {CONFIG}：{exc}")
        return None
    ids = [s["id"] for s in cfg["steps"]]
    add("PASS", "配置可读", f"{CONFIG} v{cfg.get('version')}，{len(ids)} 步：{' → '.join(ids)}")

    # 1.2 每个 stage 都已注册（用真实 registry API，不复刻内部实现）
    registry.load_all()
    missing = [s["stage"] for s in cfg["steps"]
               if s["stage"] not in registry.list_stages()]
    if missing:
        add("FAIL", "stage 已注册", f"这些 stage 没注册：{missing}（代码版本不对？）")
    else:
        add("PASS", "stage 已注册", f"{len(cfg['steps'])} 步的 stage 全部已注册")

    # 1.3 示教文件 + 点位齐不齐
    real = resolve_teach_path(TEACH)
    teach = load_teach(TEACH)
    if not teach:
        add("FAIL", "示教文件", f"读不到 {real}")
        return cfg
    pts = teach.get("points") or {}
    adds = {k: len([p for p in pts if p.startswith(k) and p[len(k):].isdigit()])
            for k in EXPECT_PREFIXES}
    bad = {k: (adds[k], n) for k, n in EXPECT_PREFIXES.items() if adds[k] != n}
    if bad:
        add("FAIL", "示教点位齐备",
            f"中间点数量不对 {bad}（期望 {EXPECT_PREFIXES}）。"
            f"多半是 deploy 漏了，或跑的是旧版 teach_drawer_real.json")
    else:
        add("PASS", "示教点位齐备",
            f"{len(pts)} 点；JNT_* 三组 {adds}", {"total": len(pts), "groups": adds})

    # 1.4 链条首段的点必须在
    lack = [p for p in CHAIN if p not in pts]
    if lack:
        add("FAIL", "链条点位", f"缺点 {lack}")
    else:
        add("PASS", "链条点位",
            f"CARRY→P2→P1→P0 共 {len(CHAIN)} 点全在"
            f"（17 个 VIA_CARRY_P2 + 11 个 VIA_P2_P1 + 6 个 VIA_P1_P0）")

    # 1.5 示教文件是不是真机回读的那一份
    prov = teach.get("provenance") or {}
    n_measured = sum(1 for k in ("P0_measured", "P1_measured", "P2_measured") if k in prov)
    if n_measured == 3:
        add("PASS", "示教来源", "P0/P1/P2 三个实测点 provenance 齐全（WAYPOINTS.json 回读）")
    else:
        add("FAIL", "示教来源", f"只找到 {n_measured}/3 个实测点的 provenance，可能不是真机回读版")

    # 1.6 标定常量在不在
    cal = teach.get("calibration") or {}
    if cal:
        add("PASS", "相机/末端标定",
            f"{cal.get('method','?')[:28]}…（姿态残差 {cal.get('raw_orientation_error_deg')}°，"
            f"位置残差 {cal.get('fk_position_residual_mm')} mm）",
            cal)
    else:
        add("WARN", "相机/末端标定", "示教文件没有 calibration 段 —— 末端位姿只能当估计值")

    return cfg


def check_limits():
    """离线核对链条上每个点的**静态**限位余量。"""
    from scenes.scene_medicine_check.pipeline.teach import load_teach
    from scenes.scene_medicine_check.pipeline.stages.s11_exp_stages import _margin_deg

    teach = load_teach(TEACH)
    pts = teach.get("points") or {}
    tight = []
    for name in CHAIN:
        rec = pts.get(name) or {}
        ms = {}
        for n, v in zip(rec.get("joint_names") or [], rec.get("joint_positions") or []):
            m = _margin_deg(str(n), float(v))
            if m is not None:
                ms[str(n)] = round(m, 2)
        if not ms:
            continue
        j = min(ms, key=lambda k: ms[k])
        tight.append((name, j, ms[j]))
    if not tight:
        add("FAIL", "静态限位余量", "一个点的关节角都算不出余量")
        return
    worst = min(tight, key=lambda t: t[2])
    lvl = "FAIL" if worst[2] < 2.0 else ("WARN" if worst[2] < 5.0 else "PASS")
    add(lvl, "静态限位余量",
        f"最紧 {worst[0]} 的 {worst[1]} 只剩 {worst[2]:.1f}°"
        f"（{'>=' if worst[2] >= 5 else '<'} 5° 告警线）",
        {"worst": worst, "all": tight})


# ---------------------------------------------------------------- 2 真机只读
def check_hw():
    """只读问硬件：关节角、末端位姿、里程计、夹爪宽、底盘。一律不发动作。"""
    from scenes.scene_medicine_check.pipeline.io import build_io
    from scenes.scene_medicine_check.pipeline.stages.s11_exp_stages import _margin_deg

    io = build_io("wrapper")

    # 2.1 关节角（用 joint_groups 取整组，与 stage 内部 _snapshot 完全一致的调用方式）
    try:
        ret = io.get_joint_positions(joint_groups=["right_arm"], joint_names=[])
    except Exception as exc:  # noqa: BLE001
        add("FAIL", "读右臂关节角", f"抛异常：{exc}")
        return
    names = list(ret.get("joint_names") or [f"right_arm_joint{i}" for i in range(1, 8)])
    pos = ret.get("joint_positions") or []
    if not ret.get("success") or len(pos) != 7:
        add("FAIL", "读右臂关节角",
            f"读不到 7 个：success={ret.get('success')} len={len(pos)} "
            f"msg={ret.get('message')}（读不到关节角 → 第一条指令就会 UNDETERMINED 全停）")
        return
    add("PASS", "读右臂关节角", f"7 轴已读：{[round(float(v), 3) for v in pos]}",
        {"joint_names": names, "joint_positions": [float(v) for v in pos]})

    # 2.2 当前姿态的限位余量（决定第一条指令能不能发）
    ms = {}
    for n, v in zip(names, pos):
        m = _margin_deg(n, float(v))
        if m is not None:
            ms[n] = round(m, 2)
    if ms:
        j = min(ms, key=lambda k: ms[k])
        lvl = "FAIL" if ms[j] < 2.0 else ("WARN" if ms[j] < 5.0 else "PASS")
        add(lvl, "当前限位余量", f"最紧 {j} = {ms[j]:.1f}°"
            + ("（已在 failed 线内，先手动让臂离开限位）" if ms[j] < 2.0 else ""), ms)

    # 2.3 末端位姿
    ee = io.get_ee_pose(chain="right_arm")
    if ee.get("success") and len(ee.get("pose") or []) == 7:
        p = ee["pose"]
        add("PASS", "读末端位姿", f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} "
                                  f"q=({p[3]:.3f},{p[4]:.3f},{p[5]:.3f},{p[6]:.3f})", {"pose": p})
    else:
        add("WARN", "读末端位姿", f"读不到：{ee.get('message')}"
                                   "（实验 1 不靠它判通过，但少了就没有交叉校验）")

    # 2.4 里程计
    od = io.get_odom()
    if od.get("success"):
        add("PASS", "读里程计", f"position={od.get('odom', {}).get('position')}")
    else:
        add("WARN", "读里程计", f"读不到：{od.get('message')}（preflight 会记 failed，但 non_fatal）")

    # 2.5 夹爪宽（实验 1 全程不夹，只用来确认"确实空爪"）
    gw = io.get_gripper_width(side="right")
    if gw.get("success"):
        add("PASS", "读右爪宽度", f"{float(gw.get('width', 0)) * 1000:.1f} mm"
            "（实验 1 全程空爪，不需要动夹爪）")
    else:
        add("WARN", "读右爪宽度", f"读不到：{gw.get('message')}"
                                   "（实验 1 不依赖它）")

    # 2.6 is_holding —— 实验 1 完全不用，但先摸清真机有没有这个接口
    try:
        h = io.is_holding()
        add("PASS", "is_holding 接口", f"存在，返回 {h}")
    except Exception as exc:  # noqa: BLE001
        add("WARN", "is_holding 接口",
            f"不可用（{type(exc).__name__}: {str(exc)[:60]}）—— 实验 1 不需要；实验 3 会需要")


# ---------------------------------------------------------------- 3 上机方案
def print_plan(cfg):
    """把上机要执行的命令原样打出来，供人工逐条核对。"""
    teach_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..",
                     "scenes/scene_medicine_check", TEACH))
    print("\n" + "=" * 78, flush=True)
    print("上机方案（以下命令需要你字面授权后才会执行）", flush=True)
    print("=" * 78, flush=True)
    print("0) 现场确认：工作区半径 0.8 m 内无人无杂物，急停在手边。", flush=True)
    print("   ⚠️ 机器人**当前不一定在抱持位** —— 只要有人拖过手臂就不在。", flush=True)
    print("      2026-11-08 实测踩过：08:19 读还在位，08:40 再读右臂 j5 已偏 −74.1°、", flush=True)
    print("      左臂 j6 偏 +48.8°。不回正直接跑，第一条指令就超跨度闸门。", flush=True)
    print("", flush=True)
    print("1) 只读：看当前离抱持位差多少（零指令）", flush=True)
    print("   MED_IO=wrapper python3 scripts/home_to_carry.py --check", flush=True)
    print("", flush=True)
    print("2) 只读预演：算回正分段与实验分段（零指令）", flush=True)
    print("   MED_IO=wrapper python3 scripts/home_to_carry.py --plan", flush=True)
    print("   MED_IO=mock    python3 scripts/run_exp.py --config "
          "scenes/scene_medicine_check/pipeline/configs/exp1_real_carry_p2_p1_p0.json --plan", flush=True)
    print("   （注：run_exp.py 的 --plan 内部固定走 mock，碰不到 SDK；本次真机预演由"
          " home_to_carry --check 覆盖）", flush=True)
    print("", flush=True)
    print("3) ★真机第一步：双臂回正到抱持位（13 段左右，单段 ≤10°，逐段回读校验）", flush=True)
    print("   MED_IO=wrapper python3 scripts/home_to_carry.py --yes", flush=True)
    print("   通过判据：退出码 0，两条臂对抱持位偏差都 ≤ 3.0°", flush=True)
    print("", flush=True)
    print("4) ★真机第二步：实验 1 第一阶段（抱持位 → P0 → P1 → 取证 → 停在 P1）", flush=True)
    print("   MED_IO=wrapper python3 scripts/run_exp.py --config "
          "scenes/scene_medicine_check/pipeline/configs/exp1_real_carry_p2_p1_p0.json", flush=True)
    print("   预期：总状态 ok，counts 里 8 ok + 1 failed（failed 是 P1→P2 取证步，", flush=True)
    print("         被 SAFETY_ABORT 拦下 —— 拦下本身就是结论，不是故障）", flush=True)
    print("", flush=True)
    print("⚠️ 不要跑 --exp 1（那是 exp1_teach_chain，离线全链版）。", flush=True)
    print("   它含 leg2_jump_to_p2 = 170° 单步关节指令，真机上 0.15 rad/s 要 19.8 s", flush=True)
    print("   > timeout 15 s，而**超时不会让机器人停**。该步已打 require_mock，", flush=True)
    print("   真机 IO 上会被 SAFETY_ABORT 拒掉，但不要拿它上真机。", flush=True)
    print("", flush=True)
    print("⚠️ 真机模式要求交互 TTY（脚本里 sys.stdin.isatty() 不过就返回 2）。", flush=True)
    print("   SSH 远程跑必须加 -tt，并在提示处输入 START；nohup/管道后台起会被拒。", flush=True)
    print("⚠️ 跑完先看 logs/exp1_trace.json 与 logs/home_to_carry_*.json，", flush=True)
    print("   不要凭屏幕滚动的日志下结论。", flush=True)
    print(f"⚠️ 示教文件路径（脚本要读的）：{teach_path}", flush=True)
    if os.path.exists(teach_path):
        print(f"   md5 = {md5(teach_path)}", flush=True)
        add("PASS", "上机方案", "已打印；示教文件 md5 一并给出，便于和 HPU 端核对",
            {"teach_md5": md5(teach_path)})
    else:
        add("FAIL", "上机方案", f"示教文件不存在：{teach_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description="实验 1 上机前只读体检")
    ap.add_argument("--skip-hw", action="store_true",
                    help="只查文件与配置，完全不碰 SDK（离线最安全）")
    ap.add_argument("--json", default="", help="结果另存 JSON 路径")
    ap.add_argument("--quiet-plan", action="store_true", help="不打印上机方案")
    args = ap.parse_args()

    print("=" * 78, flush=True)
    print("实验 1 上机前只读体检（本脚本不发任何运动/夹爪指令）", flush=True)
    print(f"cwd = {os.getcwd()}", flush=True)
    print("=" * 78, flush=True)

    print("\n---- A 代码与数据 ----", flush=True)
    cfg = check_files()
    check_limits()

    if args.skip_hw:
        print("\n---- B 真机只读：已跳过（--skip-hw）----", flush=True)
        add("WARN", "真机只读", "按 --skip-hw 跳过，未验证 SDK 通路")
    else:
        print("\n---- B 真机只读（只读接口，不动臂）----", flush=True)
        try:
            check_hw()
        except Exception as exc:  # noqa: BLE001
            add("FAIL", "真机只读", f"整体失败：{type(exc).__name__}: {exc}")
        # 本地 Windows 上必然缺 aarch64 的 galbot_sdk，这是预期而非故障 —— 分开提示
        if os.name == "nt":
            add("WARN", "运行位置",
                "当前在 Windows 本地跑，缺 aarch64 的 galbot_sdk 是预期的。"
                "真机只读体检必须在 HPU 上跑："
                "ssh galbot@192.168.166.7 → source scripts/env_hpu.sh → 再执行本脚本")

    if cfg and not args.quiet_plan:
        print_plan(cfg)

    n_fail = sum(1 for r in rows if r["level"] == "FAIL")
    n_warn = sum(1 for r in rows if r["level"] == "WARN")
    n_pass = sum(1 for r in rows if r["level"] == "PASS")
    print("\n" + "=" * 78, flush=True)
    print(f"合计 {len(rows)} 项：PASS {n_pass} / WARN {n_warn} / FAIL {n_fail}", flush=True)
    print("结论：" + ("**不许上机**，先修 FAIL 项" if n_fail
                     else "可以进入授权流程（WARN 项要现场确认）"), flush=True)
    print("=" * 78, flush=True)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"rows": rows, "pass": n_pass, "warn": n_warn, "fail": n_fail},
                      fh, ensure_ascii=False, indent=2)
        print(f"已写 {args.json}", flush=True)

    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
