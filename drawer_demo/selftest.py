"""One-command offline acceptance for this layer.

Runs the success path, every injected fault and both feasibility directions, and
compares each against the outcome it must produce. Any deviation exits non-zero,
so this can gate a handoff or a commit without a human reading the trace.

What it proves: the upper-layer logic is self-consistent and fails closed.
What it does not prove: that any robot motion has been validated.
"""
from __future__ import annotations

import json

from .contracts import Context
from .feasibility import assess
from .mock import MockIO, example_recipe, site_template
from .runtime import EVIDENCE, PLAN, FixedBoxDemo, plan_for

# Preflight and the two camera steps are not dispatched actions.
NON_COMMANDS = ("preflight", "localize_high", "verify_above_box")

# fault -> (result status, steps that must NOT have been dispatched)
CASES = {
    "": ("ok", ()),
    "stale": ("unknown", ("descend",)),
    "multiple": ("unknown", ("descend",)),
    "no_target": ("unknown", ("descend",)),
    "shift": ("unknown", ("descend",)),
    "timeout": ("unknown", ("release_box", "return_home")),
    "empty_grasp": ("unknown", ("release_box", "return_home")),
    "blocked_retreat": ("unknown", ("release_box", "return_home")),
    "yaw_short": ("unknown", ("release_box", "verify_drop")),
    "bin_miss": ("unknown", ("rotate_back",)),
    "rotate_stuck": ("unknown", ("regrip_handle",)),
    "base_drift": ("unknown", ("regrip_handle",)),
    "regrip_empty": ("unknown", ("push_drawer_closed",)),
    "close_blocked": ("unknown", ("release_handle", "return_home")),
}


def _scenario(fault):
    recipe = example_recipe()
    io = MockIO(recipe, fault)
    ctx = Context(f"selftest_{fault or 'clean'}", recipe["setup_id"])
    result = FixedBoxDemo(recipe).run(ctx, io)
    return result, ctx, io


def _structure():
    """The plan, the evidence table and the dispatcher must not drift apart."""
    commands = [step for step in PLAN if step not in NON_COMMANDS]
    problems = []
    if len(PLAN) != len(set(PLAN)):
        problems.append("PLAN contains a repeated step")
    missing = [name for name in commands if name not in EVIDENCE]
    if missing:
        problems.append(f"EVIDENCE missing: {', '.join(missing)}")
    fixed = plan_for(example_recipe(fixed=True))
    supported = (set(PLAN) | set(fixed)) - set(NON_COMMANDS)
    extra = [name for name in EVIDENCE if name not in supported]
    missing_fixed = supported - EVIDENCE.keys()
    if missing_fixed:
        problems.append(f"EVIDENCE missing: {', '.join(sorted(missing_fixed))}")
    if extra:
        problems.append(f"EVIDENCE has entries outside PLAN: {', '.join(extra)}")
    return {"check": "structure", "ok": not problems, "problems": problems,
            "steps": len(PLAN), "dispatched": len(commands)}


def run():
    """Return a report dict; `ok` is the single boolean a caller should read."""
    report = {"schema": "drawer_demo_selftest/1", "cases": [], "problems": []}

    for fault, (status, forbidden) in CASES.items():
        result, ctx, io = _scenario(fault)
        name = fault or "clean"
        problems = []
        if result.status != status:
            problems.append(f"status {result.status} != {status}")
        dispatched = [a.name for a in io.actions]
        for step in forbidden:
            if step in dispatched:
                problems.append(f"{step} was dispatched but must not be")
        if status == "ok":
            if io.stops:
                problems.append("stop requested on the success path")
            if not ctx.flags.get("drawer_demo.drawer_closed"):
                problems.append("drawer_closed flag not set")
        else:
            if not ctx.stopped_reason:
                problems.append("no latched stopped_reason")
            if not io.stops:
                problems.append("no stop requested")
        report["cases"].append({"case": name, "ok": not problems,
                                "status": result.status, "problems": problems,
                                "dispatched": len(dispatched)})

    blocked = _scenario("")[0]
    if blocked.status != "ok":
        report["problems"].append("success path did not complete")

    empty_recipe = site_template()
    io = MockIO(empty_recipe)
    ctx = Context("selftest_template", "unset")
    empty = FixedBoxDemo(empty_recipe).run(ctx, io)
    if empty.status != "failed" or io.actions:
        report["problems"].append("empty site template was not rejected before dispatch")
    report["cases"].append({"case": "site_template_rejected", "ok": empty.status == "failed"
                            and not io.actions, "status": empty.status, "problems": [],
                            "dispatched": len(io.actions)})

    structure = _structure()
    report["structure"] = structure
    report["problems"].extend(structure["problems"])

    synthetic = assess(example_recipe())
    if synthetic["overall"] != "offline_verifiable" or synthetic["blocked_stages"]:
        report["problems"].append("synthetic recipe should be computable in all stages")
    report["cases"].append({"case": "feasibility_synthetic",
                            "ok": synthetic["overall"] == "offline_verifiable",
                            "status": synthetic["overall"], "problems": [],
                            "dispatched": 0})

    empty_assess = assess(site_template())
    if empty_assess["overall"] != "blocked" or not empty_assess["blocked_stages"]:
        report["problems"].append("empty site template should report blocked stages")
    report["cases"].append({"case": "feasibility_template",
                            "ok": empty_assess["overall"] == "blocked",
                            "status": empty_assess["overall"], "problems": [],
                            "dispatched": 0})

    report["ok"] = not report["problems"] and all(c["ok"] for c in report["cases"])
    report["note"] = ("offline logic only; no robot motion, SDK call or network access "
                      "occurred in this run")
    return report


def main(stream=None):
    report = run()
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), file=stream)
    return 0 if report["ok"] else 2
