"""命令行入口：能力核对、路线校验、离线跑通、故障矩阵自检。

    python -m drawer_agent.cli caps
    python -m drawer_agent.cli routes
    python -m drawer_agent.cli run --backend offline
    python -m drawer_agent.cli run --backend offline --fault yaw_stuck --trace
    python -m drawer_agent.cli selftest

`run --backend robot` 需要显式 --confirm-hardware 与 --perception-url：
没有这两个参数就拒绝，避免在没接机器人的环境里"看起来跑通了"。
"""

from __future__ import annotations

import argparse
import json
import sys

from .link import LinkError, NEEDED_HINT, REQUIRED, required_capabilities
from .runner import DEFAULT_ROUTES, FAULTS, offline_recipe, read_json, run_flow
from .routes import RouteStore, VERIFIED_KEYS

#: 故障 → 允许的停止步骤。停止点比预期更早也算符合预期（更保守），
#: 但绝不允许跑到更后面的步骤。
EXPECTED_STOP: dict[str, tuple[str, ...]] = {
    "short_yaw": ("rotate_to_drop",),
    "yaw_stuck": ("rotate_to_drop",),
    "drawer_jam": ("open_drawer_release",),
    "empty_grasp": ("close_gripper", "descend", "verify_holding"),
    "stale_frame": ("localize_high",),
    "no_target": ("localize_high",),
    "regrip_miss": ("regrip_handle",),
    "bin_shift": ("verify_drop",),
    "stroke_short": ("push_drawer_closed",),
    "jam_step": ("open_drawer_release",),
}


def _print(obj) -> None:
    print(json.dumps(obj, ensure_ascii=False, indent=2))


def cmd_caps(args) -> int:
    from .framework import load
    from .shared_link import SharedAgentLink
    recipe = read_json(args.recipe) if args.recipe else offline_recipe(extended=args.extended)
    link = SharedAgentLink(recipe)
    rotate, _ = load().flow_options(recipe)
    needed = required_capabilities(rotate=rotate, live=True)
    available = link.caps()
    rows = []
    for name, (prio, purpose) in REQUIRED.items():
        rows.append({"capability": name, "priority": prio,
                     "sdk_wrapper": NEEDED_HINT.get(name, "-"),
                     "implemented": bool(available.get(name)),
                     "required_for_flow": name in needed,
                     "purpose": purpose})
    missing = [r["capability"] for r in rows if r["required_for_flow"] and not r["implemented"]]
    _print({"backend": "shared_agent", "recipe": str(args.recipe),
            "ready": not missing, "missing": missing, "capabilities": rows})
    return 0 if not missing else 2


def cmd_routes(args) -> int:
    recipe = read_json(args.recipe) if args.recipe else offline_recipe(extended=args.extended)
    store = RouteStore.load(args.routes or DEFAULT_ROUTES)
    gaps = store.check(recipe)
    # "未核验"是待办不是错误：结构对了就能先跑离线，现场核验后再置真。
    pending = [g for g in gaps if "未核验" in g]
    blocking = [g for g in gaps if "未核验" not in g]
    _print({"routes": str(args.routes), "summary": store.summary(),
            "blocking_gaps": blocking, "pending_verification": pending,
            "usable": not blocking})
    return 0 if not blocking else 2


def cmd_run(args) -> int:
    if args.backend == "robot":
        if not args.confirm_hardware:
            print("拒绝：真机后端需要 --confirm-hardware，且现场有人在场。", file=sys.stderr)
            return 2
        if not args.perception_url:
            print("拒绝：真机后端需要 --perception-url 指向 YOLO 推理服务。", file=sys.stderr)
            return 2
        if not args.preflight_only:
            print("真机动作须在团队 Agent 同进程调用，CLI 仅提供真机预检。", file=sys.stderr)
            return 2
    if args.preflight_only:
        from .runner import build
        fw, recipe, io = build(args.recipe, args.routes, backend=args.backend,
                               faults=args.fault, perception_url=args.perception_url,
                               extended=args.extended)
        facts = io.preflight(recipe)
        _print({"mode": "preflight_only", "motion_sent": False, "facts": facts})
        return 0 if all(facts.get(k) is True for k in fw.preflight_keys(recipe)) else 2
    result = run_flow(args.recipe, args.routes, backend=args.backend,
                      faults=args.fault, perception_url=args.perception_url,
                      stop_after=args.stop_after, extended=args.extended)
    if args.trace:
        _print(result)
    else:
        _print({k: result[k] for k in ("status", "message", "steps", "data", "faults")})
    return 0 if result["status"] == "ok" else 2


def cmd_teach(args) -> int:
    from . import teach
    recipe = read_json(args.recipe) if args.recipe else offline_recipe(extended=args.extended)
    verified = {key: True for key in VERIFIED_KEYS} if args.verified_all else \
        {key: True for key in args.verified}
    gaps = teach.write(args.session, args.out, setup_id=args.setup_id,
                       gripper_open_m=float(recipe["gripper_open_m"]),
                       drawer_opening_m=float(recipe["drawer_opening_m"]),
                       verified=verified)
    summary = RouteStore.load(args.out).summary()
    _print({"session": args.session, "out": str(args.out), "summary": summary,
            "gaps": gaps, "usable": not gaps})
    return 0 if not gaps else 2


def cmd_selftest(args) -> int:
    cases = []
    # Default matrix retains the original v2 turn/close faults and adds the fixed flow.
    extended = args.recipe is None or args.extended
    base = run_flow(args.recipe, args.routes, backend="offline", extended=extended)
    cases.append({"case": "nominal", "status": base["status"],
                  "stopped_at": None, "expected": None,
                  "ok": base["status"] == "ok"})
    if args.recipe is None:
        fixed = run_flow(backend="offline")
        cases.append({"case": "fixed_nominal", "status": fixed["status"],
                      "ok": fixed["status"] == "ok" and fixed["data"]["drawer_closed"] is False})
    for name in FAULTS:
        if not any(step in base["plan"] for step in EXPECTED_STOP[name]):
            continue
        result = run_flow(args.recipe, args.routes, backend="offline", faults=[name], extended=extended)
        stopped = (result["data"] or {}).get("failed_step")
        allowed = EXPECTED_STOP.get(name, ())
        ok = result["status"] != "ok" and stopped in allowed
        cases.append({"case": name, "status": result["status"], "stopped_at": stopped,
                      "expected": list(allowed), "ok": ok})

    # 未填写的现场模板必须被拒，不能"看起来能跑"。
    template = read_json(args.recipe) if args.recipe else offline_recipe(extended=extended)
    template.update(setup_id=None, model_id=None)
    try:
        from .framework import load as load_framework
        load_framework().validate_recipe(template, "shared_agent")
        template_rejected = False
    except Exception:  # noqa: BLE001 - 这里就是要它抛
        template_rejected = True
    cases.append({"case": "incomplete_recipe", "ok": template_rejected,
                  "status": "rejected" if template_rejected else "accepted"})

    failed = [c for c in cases if not c["ok"]]
    _print({"total": len(cases), "passed": len(cases) - len(failed),
            "failed": len(failed), "cases": cases})
    return 0 if not failed else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="drawer_agent", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p):
        p.add_argument("--recipe")
        p.add_argument("--routes")
        p.add_argument("--extended", action="store_true", help="使用原版含转体和关屉的离线配方")

    p_caps = sub.add_parser("caps", help="核对 sdk_wrapper 侧能力")
    common(p_caps)
    p_caps.set_defaults(func=cmd_caps)

    p_routes = sub.add_parser("routes", help="校验示教路线数据")
    common(p_routes)
    p_routes.set_defaults(func=cmd_routes)

    p_run = sub.add_parser("run", help="跑一次完整流程")
    common(p_run)
    p_run.add_argument("--backend", choices=("offline", "robot"), default="offline")
    p_run.add_argument("--fault", action="append", default=[], choices=sorted(FAULTS))
    p_run.add_argument("--perception-url", default=None)
    p_run.add_argument("--confirm-hardware", action="store_true")
    p_run.add_argument("--trace", action="store_true")
    p_run.add_argument("--preflight-only", action="store_true",
                       help="只做预检，不下发任何动作")
    p_run.add_argument("--stop-after", default=None,
                       help="分段演练：该动作成功后立即锁存停止")
    p_run.set_defaults(func=cmd_run)

    p_teach = sub.add_parser("teach", help="示教 point.json → 路线文件")
    common(p_teach)
    p_teach.add_argument("--session", required=True, help="medicine_dataset/teaching/<session>")
    p_teach.add_argument("--out", default="routes.teach.json")
    p_teach.add_argument("--setup-id", required=True)
    p_teach.add_argument("--verified", action="append", default=[], choices=sorted(VERIFIED_KEYS))
    p_teach.add_argument("--verified-all", action="store_true")
    p_teach.set_defaults(func=cmd_teach)

    p_self = sub.add_parser("selftest", help="正常一遍 + 全部注入故障")
    common(p_self)
    p_self.set_defaults(func=cmd_selftest)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (ValueError, OSError, LinkError, ImportError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
