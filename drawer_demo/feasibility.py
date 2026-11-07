"""Offline feasibility assessment for the extended drawer demo.

Answers one question per requested stage: can the current recipe and framework
support this action, and what is still missing before anyone may move the robot?

Verdicts are about numbers and interfaces only. `computable` means every field
the state machine consumes exists and the derived geometry is self-consistent;
it never means the robot has done it. `site_evidence` carries that separately.

Nothing here touches hardware or imports numpy/cv2, so it runs on a bare Python.
"""
from __future__ import annotations

import math

from .geometry import yaw_arc_offset_m, yaw_segments
from .runtime import EVIDENCE, SCHEMA, flow_options, plan_for, validate_recipe

# Numbers quoted from existing records, kept here so the report can show its work.
RECORDED = {
    "pull_tcp_travel_m": (0.301137,
                          "docs/drawer_close_context_2026-11-07.md §9: TCP 0.450579 -> 0.751716"),
    "release_tcp_x_m": (0.450579,
                        "galbot-hackathon/scenes/scene_medicine_check/data/"
                        "recorded_drawer_regrip_close_20261107.json"),
    "rebase_shift_limit_m": (0.25, "medicine_frontend/drawer_regrip_close.py REBASE_SHIFT_LIMIT_M"),
    "rebase_yaw_limit_deg": (10.0, "medicine_frontend/drawer_regrip_close.py REBASE_YAW_LIMIT_DEG"),
    "strict_return_shift_m": (0.030, "drawer_regrip_close.base_return_issues default"),
    "strict_return_yaw_deg": (2.0, "drawer_regrip_close.base_return_issues default"),
}

# Hardware facts on record that a feasibility verdict has to carry.
SITE_FACTS = [
    "2026-11-07: set_base_pose returned ControlStatus.TIMEOUT with no displacement; "
    "chassis motion is unproven (galbot-hackathon/scripts/diag_base_motion.py).",
    "2026-11-07: two drawer pushes returned MotionStatus.SUCCESS with ~0.02 mm TCP travel; "
    "the right arm was still owned by right_arm_pvt_ctrl_teleop.",
    "Recorded push recipe has verified_on_robot=false.",
    "No trained box-top segmentation weights exist under runs/ yet; the inference contract "
    "is exercised only by synthetic payloads.",
]


def _field(recipe, path):
    node = recipe
    for key in path.split("."):
        if not isinstance(node, dict):
            return None
        node = node.get(key)
    return node


def _number(recipe, path):
    value = _field(recipe, path)
    return value if type(value) in (int, float) and math.isfinite(value) else None


def _stage(name, requested, gaps, computed=None, site_evidence="pending"):
    return {"stage": name, "requested": requested,
            "verdict": "blocked" if gaps else "computable",
            "site_evidence": site_evidence, "gaps": gaps, "computed": computed or {}}


def _need(recipe, paths, gaps):
    for path in paths:
        if _field(recipe, path) is None:
            gaps.append(f"{path} is null in the recipe")


def assess_pull(recipe):
    gaps = []
    _need(recipe, ["drawer_opening_m", "route_refs.drawer_open_release",
                   "waypoint_refs.HANDLE_CLEAR"], gaps)
    opening = _number(recipe, "drawer_opening_m")
    travel = RECORDED["pull_tcp_travel_m"][0]
    computed = {"target_opening_mm": None if opening is None else round(opening*1000, 1),
                "recorded_tcp_travel_mm": round(travel*1000, 1),
                "note": "TCP travel is not drawer opening; the pull sagged 17 mm in Z and 7 mm in Y."}
    if opening is not None:
        computed["travel_minus_opening_mm"] = round((travel-opening)*1000, 1)
        if abs(travel-opening) > 0.02:
            computed["gap_warning"] = ("TCP travel and target opening differ by more than 20 mm; "
                                      "the 270 mm opening still needs an independent measurement.")
    return _stage("S1_pull_drawer", "拉抽屉到 270 mm 并松开把手", gaps, computed)


def assess_observe(recipe):
    gaps = []
    _need(recipe, ["waypoint_refs.OBSERVE_HIGH", "route_refs.rise_to_observe",
                   "observation_source"], gaps)
    return _stage("S2_reach_observe_high", "沿已验证路线升到标好的最高点",
                  gaps, {"must_be_verified_max_z": True,
                         "preflight_fact": "observe_high_is_verified_max_z"})


def assess_perception(recipe):
    gaps = []
    _need(recipe, ["model_id", "calibration.verified", "calibration.id",
                   "calibration.H_pixel_to_base_xy", "calibration.image_size_wh",
                   "calibration.opening_roi_xy_m", "calibration.nominal_center_xy_m",
                   "calibration.box_top_z_m", "box_size_m", "closing_axis_yaw_rad"], gaps)
    if _field(recipe, "calibration.verified") is False:
        gaps.append("calibration.verified is false")
    computed = {"contract": "drawer_box_seg/1", "class": "medicine_box_top"}
    h = _field(recipe, "calibration.H_pixel_to_base_xy")
    if isinstance(h, (list, tuple)) and len(h) == 9:
        det = (h[0]*(h[4]*h[8]-h[5]*h[7])-h[1]*(h[3]*h[8]-h[5]*h[6])
               + h[2]*(h[3]*h[7]-h[4]*h[6]))
        if abs(det) > 1e-15:
            computed["planar_scale_mm_per_px"] = round(math.sqrt(abs(det))*1000, 2)
    roi = _field(recipe, "calibration.opening_roi_xy_m")
    if isinstance(roi, (list, tuple)) and len(roi) == 4:
        computed["roi_area_mm2"] = round((roi[2]-roi[0])*(roi[3]-roi[1])*1e6, 0)
    return _stage("S3_yolo_localize", "YOLO 观察并识别药盒顶面", gaps, computed)


def assess_grasp(recipe):
    gaps = []
    _need(recipe, ["tool_id", "tcp_frame", "tool_orientation_xyzw", "above_tcp_z_m",
                   "grasp_tcp_z_m", "gripper_open_m", "gripper_close_m",
                   "box_center_to_tcp_xy_m", "waypoint_refs.START",
                   "route_refs.retreat_to_start"], gaps)
    computed = {}
    above, grasp = _number(recipe, "above_tcp_z_m"), _number(recipe, "grasp_tcp_z_m")
    width = _field(recipe, "box_size_m")
    open_m, close_m = _number(recipe, "gripper_open_m"), _number(recipe, "gripper_close_m")
    if None not in (above, grasp):
        computed["descent_depth_mm"] = round((above-grasp)*1000, 1)
    if isinstance(width, (list, tuple)) and len(width) == 3 and None not in (open_m, close_m):
        computed["finger_clearance_open_mm"] = round((open_m-width[1])*1000, 1)
        computed["squeeze_mm"] = round((width[1]-close_m)*1000, 1)
        if open_m <= width[1]:
            gaps.append("gripper_open_m does not straddle the grasp width")
    return _stage("S4_grasp_return_start", "夹取后返回初始位置 START", gaps, computed)


def assess_yaw(recipe):
    gaps = []
    _need(recipe, ["base_motion.allowed", "base_motion.release_mode",
                   "base_motion.drop_yaw_rad", "base_motion.max_segment_rad",
                   "base_motion.yaw_tolerance_rad", "base_motion.drop_radius_m",
                   "base_motion.drop_offset_tolerance_m",
                   "base_motion.rotate_segment_timeout_s",
                   "limits.max_base_yaw_speed_rad_s",
                   "waypoint_refs.START", "waypoint_refs.DROP",
                   "route_refs.rotate_to_drop"], gaps)
    yaw = _number(recipe, "base_motion.drop_yaw_rad")
    step = _number(recipe, "base_motion.max_segment_rad")
    tol = _number(recipe, "base_motion.yaw_tolerance_rad")
    radius = _number(recipe, "base_motion.drop_radius_m")
    budget = _number(recipe, "base_motion.drop_offset_tolerance_m")
    computed = {}
    if None not in (yaw, step):
        segments = yaw_segments(yaw, step)
        computed.update({"commanded_yaw_deg": round(math.degrees(yaw), 2),
                         "sign_convention": "positive is a left turn in set_base_pose, "
                                            "so a right turn must be negative",
                         "segments": len(segments),
                         "segment_deg": [round(math.degrees(s), 2) for s in segments]})
        if yaw > 0:
            gaps.append("drop_yaw_rad is positive, which turns left")
    if None not in (tol, radius, budget):
        offset = yaw_arc_offset_m(radius, tol)
        computed.update({"yaw_tolerance_deg": round(math.degrees(tol), 2),
                         "bin_radius_m": radius,
                         "arc_offset_at_tolerance_mm": round(offset*1000, 1),
                         "drop_offset_budget_mm": round(budget*1000, 1),
                         "margin_mm": round((budget-offset)*1000, 1)})
        if offset > budget:
            gaps.append("yaw tolerance moves the bin outside the drop offset budget")
    return _stage("S5_yaw_right_30", "右转 30° 到固定桶上方", gaps, computed)


def assess_return(recipe):
    gaps = []
    _need(recipe, ["base_motion.return_shift_tolerance_m",
                   "base_motion.return_yaw_tolerance_rad",
                   "route_refs.rotate_back"], gaps)
    shift = _number(recipe, "base_motion.return_shift_tolerance_m")
    yaw_tol = _number(recipe, "base_motion.return_yaw_tolerance_rad")
    computed = {"push_reuse_mode": None}
    if None not in (shift, yaw_tol):
        strict_shift = RECORDED["strict_return_shift_m"][0]
        strict_yaw = math.radians(RECORDED["strict_return_yaw_deg"][0])
        wide_shift = RECORDED["rebase_shift_limit_m"][0]
        wide_yaw = math.radians(RECORDED["rebase_yaw_limit_deg"][0])
        computed.update({"yaw_tolerance_deg": round(math.degrees(yaw_tol), 2),
                         "shift_tolerance_mm": round(shift*1000, 1),
                         "strict_reuse": "recorded push path used as-is"
                         if (shift <= strict_shift and yaw_tol <= strict_yaw) else None,
                         "rebase_reuse": "base_link poses re-expressed in the current base frame"
                         if (shift <= wide_shift and yaw_tol <= wide_yaw) else None})
        if shift > wide_shift or yaw_tol > wide_yaw:
            gaps.append("return tolerance exceeds the re-localising limits of "
                        "drawer_regrip_close, so the recorded push path cannot be reused")
        elif not computed["strict_reuse"]:
            computed["note"] = ("outside the 30 mm / 2 deg strict band, so the push path must be "
                                "rebased before use; drawer_regrip_close already implements it")
    return _stage("S6_yaw_return", "右转后转回原位", gaps, computed)


def assess_push(recipe):
    gaps = []
    _need(recipe, ["waypoint_refs.HANDLE_RELEASE", "route_refs.regrip_handle",
                   "route_refs.push_drawer_closed", "route_refs.close_to_home",
                   "limits.stroke_tolerance_m", "drawer_opening_m",
                   "base_motion.push_timeout_s"], gaps)
    opening = _number(recipe, "drawer_opening_m")
    tol = _number(recipe, "limits.stroke_tolerance_m")
    computed = {}
    if opening is not None and tol is not None:
        computed.update({"stroke_tolerance_mm": round(tol*1000, 1),
                         "target_stroke_mm": round(opening*1000, 1),
                         "recorded_path_tcp_mm": round(RECORDED["pull_tcp_travel_m"][0]*1000, 1),
                         "gate_note": "the stroke gate compares measured drawer travel, "
                                      "not the sum of the recorded TCP path"})
        if tol < 0.005:
            computed["warning"] = "stroke tolerance below 5 mm needs a better drawer sensor"
    return _stage("S7_push_drawer_closed", "重新夹住把手并推回抽屉", gaps, computed)


def assess(recipe, backend=None):
    """Per-stage verdicts plus the adapter obligations the runtime consumes."""
    backend = backend or recipe.get("mode") or "shared_agent"
    rotate, close = flow_options(recipe)
    stages = [assess_pull(recipe), assess_observe(recipe), assess_perception(recipe),
              assess_grasp(recipe)]
    if rotate:
        stages.extend((assess_yaw(recipe), assess_return(recipe)))
    else:
        gaps = []
        _need(recipe, ["route_refs.carry_to_drop", "waypoint_refs.DROP",
                       "base_motion.drop_offset_tolerance_m"], gaps)
        stages.append(_stage("S5_fixed_drop", "固定底盘投放", gaps, {}))
    if close:
        stages.append(assess_push(recipe))
    if "flow" in recipe:
        gaps = []
        key = "close_to_home" if close else "drop_to_home"
        _need(recipe, [f"route_refs.{key}", "waypoint_refs.HOME"], gaps)
        stages.append(_stage("home", "按已核验路线归位", gaps, {}))
    blocked = [s["stage"] for s in stages if s["verdict"] == "blocked"]
    pending = [s["stage"] for s in stages if s["site_evidence"] != "recorded"]
    overall = "blocked" if blocked else ("offline_verifiable" if pending else "site_ready")
    try:
        validate_recipe(recipe, backend)
        schema_ok, schema_error = True, None
    except Exception as exc:
        schema_ok, schema_error = False, f"{type(exc).__name__}: {exc}"
    return {
        "schema": "drawer_demo_feasibility/1",
        "recipe_schema": recipe.get("schema"),
        "expected_recipe_schema": SCHEMA,
        "overall": overall if schema_ok else "blocked",
        "blocked_stages": blocked,
        "plan": plan_for(recipe),
        "recipe_accepted_by_runtime": schema_ok,
        "recipe_rejection": schema_error,
        "stages": stages,
        "adapter_obligations": {name: list(EVIDENCE[name]) for name in plan_for(recipe) if name in EVIDENCE},
        "recorded_references": {k: {"value": v[0], "source": v[1]} for k, v in RECORDED.items()},
        "site_facts_on_record": SITE_FACTS,
    }
