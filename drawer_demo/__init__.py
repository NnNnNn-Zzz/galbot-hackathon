"""Drawer demo orchestration: pull, observe, grasp, yaw drop, yaw back, push closed.

No SDK imports, no hardware backend, no network. Everything here is either pure
geometry, a state machine over the `DemoIO` protocol, or a synthetic fixture.

Public surface:
    FixedBoxDemo(recipe).run(ctx, io)   the only entry the site adapter wires up
    validate_recipe(recipe, backend)    fail-closed configuration gate
    assess(recipe)                      per-stage offline feasibility report
    PLAN / EVIDENCE                     step order and required evidence facts
"""
from .contracts import Action, Context, DemoIO, Frame, GateError, Receipt, Result
from .feasibility import assess
from .runtime import (EVIDENCE, PLAN, SCHEMA, FixedBoxDemo, flow_options, plan_for,
                      preflight_keys, route_keys, validate_recipe)

__all__ = ["Action", "Context", "DemoIO", "Frame", "GateError", "Receipt", "Result",
           "EVIDENCE", "PLAN", "SCHEMA", "FixedBoxDemo", "validate_recipe", "assess",
           "flow_options", "plan_for", "preflight_keys", "route_keys"]
