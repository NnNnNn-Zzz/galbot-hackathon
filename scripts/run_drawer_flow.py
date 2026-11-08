#!/usr/bin/env python3
"""抽屉流程入口：跑 medicine.drawer 配置。

⚠️ 这个脚本会发真机运动指令。必须用户字面说"开始"/"执行实验 N"才允许跑。

用法：
    # 只看编排，不动硬件（推荐每次都先跑这个）
    python3 scripts/run_drawer_flow.py --plan

    # 只跑前 5 步（段1-4：回初始位 + 拉抽屉 + 张爪退开）
    python3 scripts/run_drawer_flow.py --steps 1-5

    # 全流程
    python3 scripts/run_drawer_flow.py

    # 离线彩排（mock 后端，不碰硬件）
    MED_IO=mock python3 scripts/run_drawer_flow.py
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

CONFIG = "medicine.drawer"


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


def main() -> int:
    ap = argparse.ArgumentParser(description="抽屉流程")
    ap.add_argument("--plan", action="store_true", help="只打印编排，不动机器人")
    ap.add_argument("--steps", default="", help="只跑指定步，如 1-5 或 1,3,5-7（1-based）")
    ap.add_argument("--config", default=CONFIG, help=f"配置名，默认 {CONFIG}")
    args = ap.parse_args()

    from scenes.scene_medicine_check.pipeline import runner
    from scenes.scene_medicine_check.pipeline.contracts import Context
    from scenes.scene_medicine_check.pipeline.io import build_io

    cfg = runner.load_config(args.config)

    # ---------------- plan：只读，不动硬件 ----------------
    if args.plan:
        print("=" * 72)
        print(f"编排预览：{cfg.get('name')} —— {cfg.get('description')}")
        print("=" * 72)
        for i, (step_id, text, impl) in enumerate(runner.describe(cfg), start=1):
            print(f"{i:>2}. [{step_id}] {text}")
            print(f"      ↳ {impl}")
        print("=" * 72)
        print(f"共 {len(cfg['steps'])} 步。这是预览，未连接机器人。")
        return 0

    # ---------------- 按 --steps 截取配置 ----------------
    steps = cfg["steps"]
    if args.steps:
        keep = parse_steps(args.steps)
        steps = [s for i, s in enumerate(steps, start=1) if i in keep]
        if not steps:
            print(f"✗ --steps {args.steps} 没匹配到任何步骤（共 {len(cfg['steps'])} 步）")
            return 2
        cfg = dict(cfg)
        cfg["steps"] = steps
        print(f"⚠️ 只跑 {len(steps)} 步：{[s['id'] for s in steps]}")

    backend = (os.environ.get("MED_IO") or "wrapper").strip().lower()
    if backend == "wrapper":
        print("=" * 72)
        print("即将向真机发送运动指令。确认：①急停在手边 ②工作区无人 ③已示教过")
        print("=" * 72)
        if sys.stdin.isatty():
            try:
                ans = input("输入 START 继续，其他任何输入取消：").strip()
            except (EOFError, KeyboardInterrupt):
                ans = ""
            if ans != "START":
                print("已取消，未发送任何指令。")
                return 1
        else:
            print("✗ 非交互环境，拒绝直接跑真机。请加 --plan 或在真终端里跑。")
            return 2

    ctx = Context(params={})
    io = build_io(backend)
    print(f"IO 后端 = {io.backend}")
    res = runner.run_pipeline(ctx, io, cfg)

    print()
    print("=" * 72)
    print(f"总状态：{res['status']}")
    print(f"counts：{res['counts']}")
    if res.get("failed_step"):
        print(f"失败步：{res['failed_step']} —— {res['failed_stage']}")
    if res.get("aborted_without_safety"):
        print("⚠️ unknown 中止：机器人状态未知，未执行归位，请人工确认后再操作")
    print("=" * 72)
    for t in res["trace"]:
        print(f"  {t['step']:<16} {t['status']:<8} {t['code']:<15} {t['message'][:80]}")

    if res.get("io_missing"):
        print(f"\n⚠️ 缺失接口：{res['io_missing']}")

    out = os.path.join(os.path.dirname(__file__), "..", "logs", "drawer_flow_trace.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=2, default=str)
    print(f"\n轨迹已存：{os.path.abspath(out)}")

    return 0 if res["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
