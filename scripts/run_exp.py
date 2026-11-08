#!/usr/bin/env python3
"""抽屉分层实验 runner —— 三个实验共用一个入口。

⚠️ 本脚本在 wrapper 后端下会发真机运动指令。必须用户字面说"开始"/"执行实验 N"
   才允许跑 wrapper。mock 后端随便跑（纯内存，不碰硬件）。

用法：
    # 只读预演：一条运动指令都不发，只算每段要动多少度、哪点骑在限位上
    python3 scripts/run_exp.py --exp 1 --plan

    # 离线彩排（默认就是这个，MED_IO 缺省=mock）
    python3 scripts/run_exp.py --exp 1

    # 指定从哪个示教点作为"机器人当前姿态"起步（mock 专用）
    python3 scripts/run_exp.py --exp 1 --start CARRY_home

    # 上真机（会二次确认）
    MED_IO=wrapper python3 scripts/run_exp.py --exp 1

    # 故障注入：验证证据链在异常时是否如实报
    python3 scripts/run_exp.py --exp 1 --fault set_joints=unknown

三个实验的配置：
    --exp 1  →  configs/exp1_teach_chain.json     示教链回放（零接触）
    --exp 2  →  configs/exp2_drawer_open.json     抽屉开合（接触类）
    --exp 3  →  configs/exp3_dive_drop.json       箱内下探 + 投放
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logging.basicConfig(level=logging.INFO,
                    format="%(levelname)s %(name)s: %(message)s")

#: 实验号 → 配置名（都在 pipeline/configs/ 下）
EXPERIMENTS = {
    "1": "exp1_teach_chain",
    "2": "exp2_drawer_open",
    "3": "exp3_dive_drop",
}

#: mock 起步姿态 → 示教文件名（--start 用）
DEFAULT_TEACH = "teach_drawer_real.json"


def parse_steps(text: str):
    """把 "1-5" / "3" / "1,3,5-7" 解析成 1-based 下标集合。None = 全跑。"""
    if not text:
        return None
    picked = set()
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            a, b = chunk.split("-", 1)
            picked.update(range(int(a), int(b) + 1))
        else:
            picked.add(int(chunk))
    return picked


def parse_faults(items):
    """--fault set_joints=unknown → [{"method":..., "behavior":...}]"""
    out = []
    for raw in items or []:
        if "=" not in raw:
            raise SystemExit(f"--fault 格式应为 method=behavior，收到 {raw!r}")
        method, behavior = raw.split("=", 1)
        out.append({"method": method.strip(), "behavior": behavior.strip()})
    return out


def start_joints_from(point: str, teach_file: str):
    """从示教文件里取某点的关节角，作为 mock 的初始关节角。

    ⚠️ 仅 mock 用。真机必须自己读当前角，绝不允许用示教值冒充实际状态。
    """
    from scenes.scene_medicine_check.pipeline.teach import load_point
    rec = load_point(f"pipeline/configs/{teach_file}", point)
    pos = rec.get("joint_positions")
    if not pos:
        raise SystemExit(f"示教文件 {teach_file} 里 {point} 没有关节角，无法作为起步姿态")
    return [float(v) for v in pos]


def do_plan(cfg, exp_no: str) -> int:
    """只读预演：把配置里的 exp_replay_chain 换成 dry_run_no_motion=True。"""
    from scenes.scene_medicine_check.pipeline import runner
    from scenes.scene_medicine_check.pipeline.contracts import Context
    from scenes.scene_medicine_check.pipeline.io import build_io

    print("=" * 78)
    print(f"实验 {exp_no} 编排预览：{cfg.get('name')}")
    print(f"  {cfg.get('description')}")
    print("=" * 78)
    for i, (step_id, text, impl) in enumerate(runner.describe(cfg), start=1):
        print(f"{i:>2}. [{step_id}] {text}")
        print(f"      ↳ {impl.split('.')[-1]}")

    print("=" * 78)
    print("只读预演：把所有 exp_replay_chain 改成 dry_run_no_motion，一条指令都不发")
    print("=" * 78)
    dry = dict(cfg)
    dry["steps"] = [dict(s) for s in cfg["steps"]]
    for s in dry["steps"]:
        if s["stage"] in ("exp_replay_chain", "drawer_pull", "drawer_dive"):
            s["params"] = dict(s.get("params") or {})
            s["params"]["dry_run_no_motion"] = True
            s["params"]["dry_run"] = True

    ctx = Context(params={})
    io = build_io("mock")
    # 起点：用配置里 experiment.start_point（没有就退回 P0_measured）。
    # 早期版本固定用 P0_measured，但实验 2/3 的起点是 ALIGN_handle / P3_dive_ref，
    # 起步姿态不对会让预演里的安全闸门（下探深度、跨度）拿错 z0 直接误拒。
    teach_file = DEFAULT_TEACH
    for s in cfg["steps"]:
        t = (s.get("params") or {}).get("file")
        if t:
            teach_file = os.path.basename(str(t))
            break
    start_pt = str((cfg.get("experiment") or {}).get("start_point") or "P0_measured")
    try:
        from scenes.scene_medicine_check.pipeline.teach import load_point
        rec = load_point(f"pipeline/configs/{teach_file}", start_pt)
        io.joints = [float(v) for v in rec["joint_positions"]]
        if rec.get("pose"):
            io.ee_pose = [float(v) for v in rec["pose"]]
        print(f"预演起点 = {start_pt}")
    except Exception as exc:  # noqa: BLE001 - 预演拿不到起点就退回默认值并提示
        print(f"⚠️ 预演起点 {start_pt} 取不到（{exc}），退回 P0_measured")
        try:
            io.joints = start_joints_from("P0_measured", teach_file)
        except SystemExit:
            pass
    res = runner.run_pipeline(ctx, io, dry)

    print()
    print("=" * 78)
    print(f"预演总状态：{res['status']}   counts={res['counts']}")
    print("=" * 78)
    for t in res["trace"]:
        print(f"  {t['step']:<16} {t['status']:<8} {t['code']:<15} {t['message'][:88]}")
        for seg in (t.get("data") or {}).get("trace") or []:
            if isinstance(seg, dict) and "max_delta_deg" in seg:
                print(f"       {seg['from']:<14} → {seg['to']:<14} "
                      f"最大 {seg['max_delta_deg']:6.1f}° @ {seg['max_delta_joint']:<18}"
                      f" 目标余量 {seg['target_tightest_deg']:.1f}°")
    return 0 if res["status"] == "ok" else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="抽屉分层实验")
    ap.add_argument("--exp", default="1", choices=sorted(EXPERIMENTS),
                    help="实验号：1=示教链 2=抽屉开合 3=下探投放")
    ap.add_argument("--plan", action="store_true",
                    help="只读预演：算跨度、算限位余量，不发任何指令")
    ap.add_argument("--config", default="",
                    help="直接指定配置名/路径（覆盖 --exp）")
    ap.add_argument("--steps", default="", help="只跑指定步，如 1-5（1-based）")
    ap.add_argument("--start", default="",
                    help=f"mock 起步姿态（示教点名，默认用配置文件里的 start_point）")
    ap.add_argument("--objects", default="",
                    help="mock 桌面场景：default（三盒药，基线用）| drawer（一盒在抽屉里，实验 3 用）")
    ap.add_argument("--fault", action="append", default=[],
                    metavar="METHOD=BEHAVIOR",
                    help="mock 故障注入，如 --fault set_joints=unknown（可重复）")
    ap.add_argument("--out", default="", help="trace 输出路径（默认 logs/exp{N}_trace.json）")
    args = ap.parse_args()

    from scenes.scene_medicine_check.pipeline import runner
    from scenes.scene_medicine_check.pipeline.contracts import Context
    from scenes.scene_medicine_check.pipeline.io import build_io

    cfg_name = args.config or EXPERIMENTS[args.exp]
    cfg = runner.load_config(cfg_name)
    exp_no = args.exp
    if args.config:
        # --config 直接指路时，--exp 的默认值 "1" 跟配置无关，
        # 标题和 trace 文件名会写错实验号。改成从配置的 experiment.layer 取。
        lyr = (cfg.get("experiment") or {}).get("layer")
        if lyr is not None:
            exp_no = str(lyr)
        else:
            stem = os.path.splitext(os.path.basename(cfg_name))[0]
            exp_no = stem.split("_")[0].lstrip("exp") or exp_no

    if args.plan:
        return do_plan(cfg, exp_no)

    # ---------------- 按 --steps 截取 ----------------
    steps = cfg["steps"]
    if args.steps:
        keep = parse_steps(args.steps)
        steps = [s for i, s in enumerate(steps, start=1) if i in keep]
        if not steps:
            print(f"✗ --steps {args.steps} 没匹配到任何步骤（共 {len(cfg['steps'])} 步）")
            return 2
        cfg = dict(cfg, steps=steps)
        print(f"⚠️ 只跑 {len(steps)} 步：{[s['id'] for s in steps]}")

    backend = (os.environ.get("MED_IO") or "mock").strip().lower()

    # ---------------- 真机二次确认 ----------------
    if backend == "wrapper":
        print("=" * 78)
        print(f"⚠️ 即将向真机发送动作指令（实验 {exp_no}）。")
        print("   确认：①急停在手边 ②工作区无人无杂物 ③抽屉/桌面摆位与本批示教数据一致")
        print("=" * 78)
        if sys.stdin.isatty():
            try:
                ans = input("输入 START 继续，其他任何输入取消：").strip()
            except (EOFError, KeyboardInterrupt):
                ans = ""
            if ans != "START":
                print("已取消，未发送任何指令。")
                return 1
        else:
            print("✗ 非交互环境，拒绝直接跑真机。请加 --plan 或把 HOME 从后台藏起来。")
            return 2

    # ---------------- 组装 mock 初值 ----------------
    kwargs = {}
    if backend == "mock":
        teach_file = DEFAULT_TEACH
        for s in cfg["steps"]:
            t = (s.get("params") or {}).get("file")
            if t:
                teach_file = os.path.basename(str(t))
                break
        kwargs["teach_file"] = teach_file
        # 桌面场景：配置里可以写 "mock_scene": "drawer"，命令行 --objects 优先
        scene = args.objects or str((cfg.get("experiment") or {}).get("mock_scene") or "")
        if scene:
            from scenes.scene_medicine_check.pipeline.io import SCENES
            if scene not in SCENES:
                print(f"✗ --objects {scene} 未知（可选 {sorted(SCENES)}）")
                return 2
            kwargs["objects"] = SCENES[scene]()
            print(f"mock 桌面场景 = {scene}（{len(kwargs['objects'])} 个物体）")
        start_pt = args.start or str((cfg.get("experiment") or {}).get("start_point") or "")
        if start_pt:
            try:
                kwargs["start_joints"] = start_joints_from(start_pt, teach_file)
                from scenes.scene_medicine_check.pipeline.teach import load_point
                pose = load_point(f"pipeline/configs/{teach_file}", start_pt).get("pose")
                if pose:
                    kwargs["start_pose"] = [float(v) for v in pose]
                print(f"mock 起步姿态 = {start_pt}（关节角与末端位姿都取自示教文件）")
            except SystemExit as exc:
                print(f"✗ {exc}")
                return 2

    faults = parse_faults(args.fault)
    ctx = Context(params={})
    io = build_io(backend, faults=faults, **kwargs)
    print(f"IO 后端 = {io.backend}" + (f"，注入 {len(faults)} 条故障" if faults else ""))

    res = runner.run_pipeline(ctx, io, cfg)

    # ---------------- 结果汇总 ----------------
    print()
    print("=" * 78)
    print(f"实验 {exp_no} 总状态：{res['status']}")
    print(f"counts：{res['counts']}")
    if res.get("failed_step"):
        print(f"失败步：{res['failed_step']} —— {res['failed_stage']}")
        print(f"失败码：{res['failed_code']}")
    if res.get("aborted_without_safety"):
        print("⚠️ unknown 中止：机器人状态未知，未执行归位，请人工确认后再操作")
    print("=" * 78)
    for t in res["trace"]:
        print(f"  {t['step']:<16} {t['status']:<8} {t['code']:<15} {t['message'][:88]}")

    # 实验 1 的链条明细
    chain = ctx.flags.get("exp.chain") or []
    if chain:
        print("\n" + "-" * 78)
        print("示教链明细（每点回放后实测）")
        print("-" * 78)
        print(f"  {'段':<28} {'回放前跨度':>10} {'回放后偏差':>10} {'容差':>6} {'最紧余量':>10}")
        for r in chain:
            if "err_deg" not in r:
                continue
            print(f"  {r['from']:<12} → {r['to']:<13} "
                  f"{r['pre_delta_deg']:>9.1f}° {r['err_deg']:>9.2f}° "
                  f"{r['tolerance_deg']:>5.1f}° "
                  f"{r['tightest_joint']:>10} {r['tightest_margin_deg']:>6.1f}°")

    if res.get("io_missing"):
        print(f"\n⚠️ 缺失接口：{res['io_missing']}")

    out = args.out or os.path.join(
        os.path.dirname(__file__), "..", "logs", f"exp{exp_no}_trace.json")
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"result": res, "flags": ctx.flags,
                   "backend": io.backend}, fh,
                  ensure_ascii=False, indent=2, default=str)
    print(f"\n轨迹已存：{out}")

    return 0 if res["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
