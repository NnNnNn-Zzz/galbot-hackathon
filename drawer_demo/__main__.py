"""Plan, Mock, feasibility and selftest only. No live flag, no robot dependencies."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .contracts import Context
from .feasibility import assess
from .mock import MockIO, example_recipe, site_template
from .runtime import FixedBoxDemo, plan_for
from .selftest import run as selftest_run

FAULTS = ("", "stale", "multiple", "no_target", "shift", "timeout", "empty_grasp",
          "blocked_retreat", "rotate_stuck", "yaw_short", "bin_miss", "base_drift",
          "regrip_empty", "close_blocked")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("plan", "mock", "site-template", "feasibility", "selftest"))
    parser.add_argument("--fault", default="", choices=FAULTS)
    parser.add_argument("--recipe", default=None,
                        help="recipe JSON to assess; default is the empty site template")
    parser.add_argument("--extended", action="store_true",
                        help="use the original rotating/closing example for plan or mock")
    args = parser.parse_args()
    if args.mode == "plan":
        recipe = json.loads(Path(args.recipe).read_text(encoding="utf-8")) if args.recipe \
            else example_recipe(fixed=not args.extended)
        output, code = {"mode": "plan_only", "steps": plan_for(recipe)}, 0
    elif args.mode == "site-template":
        output, code = site_template(), 0
    elif args.mode == "selftest":
        output = selftest_run()
        code = 0 if output["ok"] else 2
    elif args.mode == "feasibility":
        recipe = json.loads(Path(args.recipe).read_text(encoding="utf-8")) if args.recipe \
            else site_template()
        output = assess(recipe)
        code = 0 if output["overall"] != "blocked" else 2
    else:
        recipe = example_recipe(fixed=not args.extended)
        ctx = Context("synthetic_run", recipe["setup_id"])
        io = MockIO(recipe, args.fault)
        result = FixedBoxDemo(recipe).run(ctx, io)
        output = {"mode": "synthetic_mock_not_robot_validation", "result": asdict(result),
                  "context": asdict(ctx), "commands": [a.name for a in io.actions]}
        code = 0 if result.status == "ok" else 2
    print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
