"""Bounded demo state machine: pull, observe, grasp, return, yaw-drop, yaw-back, push closed.

All non-ok outcomes latch a stop. Nothing here retries, auto-homes or opens the
gripper on failure: an operator decides, because a dropped box or a half-open
drawer needs eyes on it.
"""
from __future__ import annotations

import math
import uuid

from .contracts import Action, Context, DemoIO, GateError, Receipt, Result, require
from .geometry import axis_error, finite_vector, localize, yaw_segments


SCHEMA = "drawer_demo_recipe/2"

PLAN = [
    "preflight", "open_drawer_release", "observe_high", "open_gripper",
    "localize_high", "align_above_box", "verify_above_box", "descend",
    "close_gripper", "short_lift", "verify_holding", "retreat_to_start",
    "rotate_to_drop", "release_box", "verify_drop", "rotate_back",
    "regrip_handle", "push_drawer_closed", "release_handle", "return_home",
]

# Evidence facts every DemoIO must produce, per dispatched action. Listed here
# as data so the site adapter and the feasibility report read the same table.
EVIDENCE = {
    "open_drawer_release": ("drawer_open_verified", "handle_released", "arm_clear_of_drawer"),
    "observe_high": ("stationary", "joint_branch_verified"),
    "open_gripper": ("gap_verified",),
    "align_above_box": ("stationary", "joint_branch_verified"),
    "descend": ("joint_branch_verified", "at_grasp_height"),
    "close_gripper": ("contact_verified",),
    "short_lift": ("joint_branch_verified",),
    "verify_holding": ("holding_verified", "box_lift_verified"),
    "retreat_to_start": ("holding_verified", "payload_clear_of_drawer", "at_start"),
    # yaw_measured_rad is the measured base yaw relative to the run-start pose.
    "rotate_to_drop": ("holding_verified", "swept_clear", "base_motion_locked"),
    "move_to_drop": ("holding_verified", "above_bin"),
    "release_box": ("gripper_empty",),
    "verify_drop": ("box_in_bin", "gripper_empty"),
    "rotate_back": ("base_returned", "gripper_empty"),
    "regrip_handle": ("handle_grasped", "at_release_pose"),
    "push_drawer_closed": ("drawer_closed", "stroke_verified", "grip_maintained"),
    "release_handle": ("handle_released", "arm_clear_of_drawer"),
    "return_home": ("gripper_empty", "at_home"),
}


def flow_options(recipe: dict) -> tuple[bool, bool]:
    # Existing v2 offline fixtures represent the original full flow.
    flow = recipe.get("flow", {"rotate_for_drop": True, "close_drawer": True})
    require(isinstance(flow, dict), "Invalid flow options")
    rotate, close = flow.get("rotate_for_drop"), flow.get("close_drawer")
    require(type(rotate) is bool and type(close) is bool, "Flow options must be booleans")
    return rotate, close


def route_keys(recipe: dict) -> tuple[str, ...]:
    rotate, close = flow_options(recipe)
    drop = ("rotate_to_drop", "rotate_back") if rotate else ("carry_to_drop",)
    end = ("regrip_handle", "push_drawer_closed", "close_to_home") if close else ("drop_to_home",)
    if close and "flow" in recipe:
        end += ("release_handle_clear",)
    if recipe.get("drawer_open", {}).get("mode") == "wrist_vision":
        opening = ("wrist_view",) if recipe["drawer_open"].get("use_view_route") is True else ()
    else:
        opening = ("drawer_open_release",)
    return (*opening, "rise_to_observe", "retreat_to_start", *drop, *end)


def plan_for(recipe: dict) -> list[str]:
    rotate, close = flow_options(recipe)
    return [
        "move_to_drop" if name == "rotate_to_drop" and not rotate else name
        for name in PLAN
        if (rotate or name != "rotate_back")
        and (close or name not in ("regrip_handle", "push_drawer_closed", "release_handle"))
    ]


def preflight_keys(recipe: dict) -> tuple[str, ...]:
    rotate, _ = flow_options(recipe)
    common = ("shared_control_ready", "stop_channel_ready", "telemetry_fresh",
              "legs_head_locked_and_base_parked", "tool_tcp_verified",
              "camera_calibration_verified", "routes_verified",
              "observe_high_is_verified_max_z", "bin_fixed_verified")
    if recipe.get("source_kind") == "site":
        common += ("start_pose_verified", "drawer_initial_state_verified",
                   "contact_monitor_ready", "perception_ready")
    if recipe.get("drawer_open", {}).get("mode") == "wrist_vision":
        common += ("wrist_opener_ready",)
    return common + (("chassis_pose_control_owned", "base_yaw_free_path_verified") if rotate else ())


def validate_recipe(r: dict, backend: str) -> None:
    require(r["schema"] == SCHEMA, f"Invalid recipe schema, expected {SCHEMA}")
    require(backend in ("mock", "shared_agent"), "Unsupported IO backend")
    require(r["mode"] == backend, "Mock/site recipe and backend must match")
    for key in ("setup_id", "model_id", "tool_id", "tcp_frame", "observation_source"):
        require(isinstance(r[key], str) and bool(r[key]), f"Missing {key}")
    require(r["reference_frame"] == "base_link", "Wrong reference frame")
    require(r["arm"] == "right_arm", "First demo requires right arm")
    require(r["fixed_camera"] is True, "Fixed head camera required")
    opener = r.get("drawer_open", {})
    require(isinstance(opener, dict) and opener.get("mode", "taught") in ("taught", "wrist_vision"),
            "Invalid drawer opening mode")
    if opener.get("mode") == "wrist_vision":
        for key in ("camera_id", "calibration_id"):
            require(isinstance(opener.get(key), str) and opener[key], f"Wrist opener requires {key}")
        require(type(opener.get("use_view_route", False)) is bool, "Invalid wrist view route switch")
    cal, limits = r["calibration"], r["limits"]
    require(cal["verified"] is True, "Plane calibration not verified")
    for key in ("id", "camera_id", "evidence_ref", "pixel_convention"):
        require(isinstance(cal[key], str) and bool(cal[key]), f"Missing calibration {key}")
    require(cal["pixel_convention"] == "original_distortion_compensated",
            "Homography needs distortion-compensated original-pixel convention")
    require(finite_vector(cal["image_size_wh"], 2)
            and all(type(x) is int and x > 0 for x in cal["image_size_wh"]), "Invalid image size")
    require(finite_vector(cal["H_pixel_to_base_xy"], 9), "Missing homography")
    h = cal["H_pixel_to_base_xy"]
    determinant = h[0]*(h[4]*h[8]-h[5]*h[7])-h[1]*(h[3]*h[8]-h[5]*h[6])+h[2]*(h[3]*h[7]-h[4]*h[6])
    require(abs(determinant) > 1e-15, "Singular homography")
    require(finite_vector(cal["opening_roi_xy_m"], 4), "Missing ROI")
    a, b, c, d = cal["opening_roi_xy_m"]
    require(a < c and b < d, "Invalid opening ROI")
    require(finite_vector(cal["nominal_center_xy_m"], 2), "Missing nominal center")
    require(finite_vector(r["box_size_m"], 3) and min(r["box_size_m"]) > 0, "Invalid box size")
    require(finite_vector(r["box_center_to_tcp_xy_m"], 2), "Missing grasp-center to TCP offset")
    require(r["box_size_m"][0] > r["box_size_m"][1], "Long edge must exceed grasp width")
    require(finite_vector([cal["box_top_z_m"]], 1), "Missing box_top_z_m")
    for key in ("closing_axis_yaw_rad", "grasp_tcp_z_m", "above_tcp_z_m"):
        require(finite_vector([r[key]], 1), f"Missing {key}")
    require(r["above_tcp_z_m"] > r["grasp_tcp_z_m"], "Invalid descent direction")
    require(finite_vector(r["tool_orientation_xyzw"], 4)
            and abs(sum(x*x for x in r["tool_orientation_xyzw"])-1) < 1e-5, "Invalid TCP orientation")
    rotate, close = flow_options(r)
    optional_limits = (("max_base_yaw_speed_rad_s",) if rotate else ()) + (
        ("stroke_tolerance_m",) if close else ())
    for key in ("min_confidence", "min_rectangularity", "max_frame_age_s", "max_receipt_age_s",
                "size_tolerance_m", "max_axis_error_rad", "max_center_shift_m",
                "max_recheck_delta_m", "max_recheck_axis_rad", "perception_timeout_s",
                "action_timeout_s", "max_linear_speed_m_s", "max_joint_speed_rad_s",
                "gripper_speed_m_s", "gripper_effort_n", *optional_limits):
        require(finite_vector([limits[key]], 1) and limits[key] > 0, f"Invalid limit {key}")
    require(limits["min_confidence"] <= 1 and limits["min_rectangularity"] <= 1,
            "Invalid confidence/shape threshold")
    for key in ("short_lift_m", "gripper_open_m", "gripper_close_m", "drawer_opening_m"):
        require(finite_vector([r[key]], 1) and r[key] >= 0, f"Invalid {key}")
    require(r["gripper_open_m"] > r["box_size_m"][1] > r["gripper_close_m"], "Invalid gripper gap")
    require(0 < r["short_lift_m"] < r["above_tcp_z_m"]-r["grasp_tcp_z_m"], "Invalid test lift")

    require(r["drawer_opening_m"] > 0, "Drawer opening must be positive")
    base = r["base_motion"]
    require(isinstance(base, dict), "Missing base_motion block")
    require(base["allowed"] is rotate, "base_motion.allowed must match rotate_for_drop")
    require(base["release_mode"] == "drop",
            "Only a controlled drop is supported; ballistic release has no evidence channel")
    require(finite_vector([base.get("drop_offset_tolerance_m")], 1)
            and base["drop_offset_tolerance_m"] > 0, "Invalid drop offset tolerance")
    if close:
        require(finite_vector([base.get("push_timeout_s")], 1)
                and base["push_timeout_s"] > 0, "Invalid push timeout")
    if rotate:
        _validate_rotation(base)

    for point in ("HOME", "HANDLE_CLEAR", "OBSERVE_HIGH", "START", "DROP", "HANDLE_RELEASE"):
        require(isinstance(r["waypoint_refs"][point], str) and r["waypoint_refs"][point],
                f"Missing taught waypoint {point}")
    for name in route_keys(r):
        require(isinstance(r["route_refs"][name], str) and r["route_refs"][name], f"Missing route {name}")


def _validate_rotation(base: dict) -> None:
    for key in ("drop_yaw_rad", "max_segment_rad", "yaw_tolerance_rad",
                "return_shift_tolerance_m", "return_yaw_tolerance_rad",
                "drop_radius_m", "rotate_segment_timeout_s"):
        require(finite_vector([base.get(key)], 1), f"Missing base_motion {key}")
    require(-math.pi/2 <= base["drop_yaw_rad"] < 0, "Right yaw must be within (-90°, 0°)")
    require(base["max_segment_rad"] > 0, "Rotation step must be positive")
    require(abs(base["drop_yaw_rad"])/base["max_segment_rad"] <= 36, "Too many rotation segments")
    require(0 < base["yaw_tolerance_rad"] < abs(base["drop_yaw_rad"])/2, "Invalid yaw tolerance")
    require(base["return_shift_tolerance_m"] > 0 and base["return_yaw_tolerance_rad"] > 0,
            "Invalid return tolerance")
    require(base["drop_radius_m"] > 0 and base["rotate_segment_timeout_s"] > 0, "Invalid rotation geometry or timeout")
    require(base["yaw_tolerance_rad"]*base["drop_radius_m"] <= base["drop_offset_tolerance_m"],
            "Yaw tolerance cannot keep the bin mouth inside the drop offset budget")


class FixedBoxDemo:
    """Stage.run(ctx, io) boundary; Result is local, not legacy StageResult.

    Context may be adapted to the team's Context. The old runner must not wrap
    this in auto-recovery. Each execute includes a bounded timeout, but only the
    external adapter/watchdog can stop a blocking SDK call in real time.
    """

    def __init__(self, recipe: dict, *, hover_only: bool = False):
        self.recipe = recipe
        self.hover_only = hover_only

    def run(self, ctx: Context, io: DemoIO) -> Result:
        r, step = self.recipe, "preflight"
        ctx.target, ctx.picked = None, None
        for key in list(ctx.flags):
            if key.startswith("drawer_demo."):
                del ctx.flags[key]
        ctx.flags["drawer_demo.payload_clear_of_drawer"] = False
        used_receipts: set[str] = set()
        motion_attempted = False

        def record(name, status, **data):
            ctx.trace.append({"step": name, "status": status, "timestamp_ns": io.now_ns(), **data})

        def command(name, required=(), timeout_s=None, **params):
            nonlocal step, motion_attempted
            step, motion_attempted = name, True
            # Facts are written only from the receipt, so a lost reply cannot
            # leave a stale success behind.
            record(name, "dispatched", params=params)
            common = {"setup_id": ctx.setup_id, "reference_frame": "base_link",
                      "tool_id": r["tool_id"], "tcp_frame": r["tcp_frame"],
                      "base_motion_authorized": rotate, "limits": limits}
            action = Action(name, {**common, **params}, tuple(required), uuid.uuid4().hex)
            receipt = io.execute(action,
                                 timeout_s=timeout_s or limits["action_timeout_s"])
            require(isinstance(receipt, Receipt), "Malformed action receipt")
            require(receipt.action_id and receipt.action_id not in used_receipts, "Reused/missing action receipt")
            require(receipt.action_id == action.request_id, "Receipt does not match dispatched action")
            used_receipts.add(receipt.action_id)
            record(name, receipt.status, receipt_id=receipt.action_id,
                   evidence=receipt.evidence, message=receipt.message)
            require(receipt.status == "ok", f"Action {name}: {receipt.status}: {receipt.message}")
            require(0 <= io.now_ns()-receipt.timestamp_ns <= limits["max_receipt_age_s"]*1e9,
                    "Stale action feedback")
            for fact in ("scene_unchanged", "path_checked", "arrived", *required):
                require(receipt.evidence.get(fact) is True, f"Missing physical evidence: {fact}")
            return receipt

        def number(evidence, key, what):
            value = evidence.get(key)
            require(finite_vector([value], 1), f"{what}: no measured value")
            return value

        def observe(viewpoint):
            nonlocal step
            step = "localize_high" if viewpoint == "OBSERVE_HIGH" else "verify_above_box"
            requested_at = io.now_ns()
            frame = io.capture(viewpoint, timeout_s=limits["perception_timeout_s"])
            require(frame.capture_timestamp_ns >= requested_at,
                    "Capture is from before stationary observation request")
            payload = io.segment(frame, timeout_s=limits["perception_timeout_s"])
            target = localize(payload, frame, r, io.now_ns())
            record(step, "ok", target=target)
            return target

        try:
            require(ctx.stopped_reason is None, "Context has a latched stop; create a reviewed new run")
            require(ctx.setup_id == r["setup_id"], "Context setup mismatch")
            validate_recipe(r, io.backend)
            rotate, close = flow_options(r)
            base, limits = r["base_motion"], r["limits"]
            pre = io.preflight(r)
            for fact in preflight_keys(r):
                require(pre.get(fact) is True, f"Preflight missing: {fact}")
            record(step, "ok", evidence=pre)

            # S1 pull the drawer to the target opening and let go of the handle.
            command("open_drawer_release", EVIDENCE["open_drawer_release"],
                    route_ref=r["route_refs"].get("drawer_open_release"),
                    target_opening_m=r["drawer_opening_m"])
            ctx.flags["drawer_demo.drawer_open_verified"] = True

            # S2 rise along the verified release route to its highest point.
            command("observe_high", EVIDENCE["observe_high"],
                    route_ref=r["route_refs"]["rise_to_observe"],
                    waypoint_ref=r["waypoint_refs"]["OBSERVE_HIGH"])
            command("open_gripper", EVIDENCE["open_gripper"], width_m=r["gripper_open_m"])

            # S3 segment the box top from the highest viewpoint.
            first = observe("OBSERVE_HIGH")

            # S4 align, recheck, descend straight, grip, prove the box came up.
            x, y, _ = first["position_m"]
            x += r["box_center_to_tcp_xy_m"][0]
            y += r["box_center_to_tcp_xy_m"][1]
            pose = {"position_m": [x, y, r["above_tcp_z_m"]],
                    "orientation_xyzw": r["tool_orientation_xyzw"]}
            command("align_above_box", EVIDENCE["align_above_box"],
                    tcp_pose=pose, motion="verified_free_space_then_horizontal")
            second = observe("ABOVE_BOX")
            require(second["image_ref"] != first["image_ref"]
                    and second["capture_timestamp_ns"] > first["capture_timestamp_ns"],
                    "Reused verification frame")
            delta_m = math.dist(first["position_m"], second["position_m"])
            delta_axis = axis_error(first["measured_short_axis_yaw_rad"],
                                    second["measured_short_axis_yaw_rad"])
            require(delta_m <= limits["max_recheck_delta_m"],
                    "Target shifted after alignment")
            require(delta_axis <= limits["max_recheck_axis_rad"],
                    "Target rotated after alignment")
            # No chasing: the aligned first target is frozen for the descent.
            ctx.target = {**first, "verification_frame": second["image_ref"], "xy_frozen": True}
            if self.hover_only:
                record("hover_complete", "ok", delta_m=delta_m, delta_axis_rad=delta_axis)
                return Result("ok", "Verified both images at the hover checkpoint",
                              {"checkpoint": "verify_above_box", "terminal_pose": "ABOVE_BOX",
                               "full_cycle": False, "recheck_delta_m": delta_m,
                               "recheck_delta_axis_rad": delta_axis})
            command("descend", EVIDENCE["descend"],
                    tcp_pose={**pose, "position_m": [x, y, r["grasp_tcp_z_m"]]},
                    motion="vertical_line", vision_timestamp_ns=second["capture_timestamp_ns"])
            command("close_gripper", EVIDENCE["close_gripper"], width_m=r["gripper_close_m"])
            command("short_lift", EVIDENCE["short_lift"],
                    tcp_pose={**pose, "position_m": [x, y, r["grasp_tcp_z_m"]+r["short_lift_m"]]},
                    motion="vertical_line")
            command("verify_holding", EVIDENCE["verify_holding"])
            ctx.picked = True

            # S5 back to the start pose, box held clear of the drawer, then yaw right.
            command("retreat_to_start", EVIDENCE["retreat_to_start"],
                    route_ref=r["route_refs"]["retreat_to_start"],
                    waypoint_ref=r["waypoint_refs"]["START"], target=ctx.target)
            ctx.flags["drawer_demo.payload_clear_of_drawer"] = True
            if rotate:
                segments = yaw_segments(base["drop_yaw_rad"], base["max_segment_rad"])
                turned = command("rotate_to_drop", EVIDENCE["rotate_to_drop"],
                                 yaw_rad=base["drop_yaw_rad"], yaw_segments_rad=segments,
                                 max_speed_rad_s=limits["max_base_yaw_speed_rad_s"],
                                 route_ref=r["route_refs"]["rotate_to_drop"],
                                 waypoint_ref=r["waypoint_refs"]["DROP"],
                                 timeout_s=len(segments)*base["rotate_segment_timeout_s"])
                reached = number(turned.evidence, "yaw_measured_rad", "drop rotation")
                require(abs(reached-base["drop_yaw_rad"]) <= base["yaw_tolerance_rad"],
                        f"Drop yaw residual {math.degrees(reached-base['drop_yaw_rad']):.1f} deg")
                ctx.flags["drawer_demo.drop_yaw_verified"] = True
            else:
                command("move_to_drop", EVIDENCE["move_to_drop"],
                        route_ref=r["route_refs"]["carry_to_drop"],
                        waypoint_ref=r["waypoint_refs"]["DROP"])

            # S6 release above the fixed bin, then confirm the box actually left.
            command("release_box", EVIDENCE["release_box"],
                    width_m=r["gripper_open_m"], release_mode=base["release_mode"])
            ctx.picked = False
            command("verify_drop", EVIDENCE["verify_drop"])
            ctx.flags["drawer_demo.box_in_bin"] = True

            if rotate:
                back_segments = yaw_segments(-base["drop_yaw_rad"], base["max_segment_rad"])
                back = command("rotate_back", EVIDENCE["rotate_back"],
                               yaw_rad=-base["drop_yaw_rad"], yaw_segments_rad=back_segments,
                               max_speed_rad_s=limits["max_base_yaw_speed_rad_s"],
                               route_ref=r["route_refs"]["rotate_back"],
                               timeout_s=len(back_segments)*base["rotate_segment_timeout_s"])
                residual = number(back.evidence, "yaw_measured_rad", "return rotation")
                shift = number(back.evidence, "base_shift_m", "return position")
                require(abs(residual) <= base["return_yaw_tolerance_rad"],
                        f"Base yaw residual {math.degrees(residual):.1f} deg after turning back")
                require(shift <= base["return_shift_tolerance_m"],
                        f"Base drifted {shift*1000:.0f} mm after turning back")
                ctx.flags["drawer_demo.base_return_verified"] = True

            if close:
                command("regrip_handle", EVIDENCE["regrip_handle"],
                        waypoint_ref=r["waypoint_refs"]["HANDLE_RELEASE"],
                        route_ref=r["route_refs"]["regrip_handle"])
                pushed = command("push_drawer_closed", EVIDENCE["push_drawer_closed"],
                                 route_ref=r["route_refs"]["push_drawer_closed"],
                                 target_opening_m=0.0,
                                 timeout_s=base["push_timeout_s"])
                stroke = number(pushed.evidence, "stroke_m", "push stroke")
                require(abs(stroke-r["drawer_opening_m"]) <= limits["stroke_tolerance_m"],
                        f"Push stroke {stroke*1000:.0f} mm outside {r['drawer_opening_m']*1000:.0f} mm tolerance")
                ctx.flags["drawer_demo.drawer_closed"] = True
                release_route = r["route_refs"].get("release_handle_clear")
                command("release_handle", EVIDENCE["release_handle"], route_ref=release_route)
            home_route = "close_to_home" if close else "drop_to_home"
            command("return_home", EVIDENCE["return_home"],
                    route_ref=r["route_refs"][home_route],
                    waypoint_ref=r["waypoint_refs"]["HOME"])
            message = "Box deposited and drawer closed" if close else "Box deposited; drawer left open"
            return Result("ok", message, {"box_in_bin": True, "drawer_closed": close})
        except (Exception, KeyboardInterrupt) as exc:
            # Unexpected errors after dispatch might mean motion completed without acknowledgement.
            status = "unknown" if motion_attempted or not isinstance(exc, GateError) else "failed"
            ctx.picked = None
            ctx.flags["drawer_demo.payload_clear_of_drawer"] = False
            ctx.stopped_reason = f"{step}: {type(exc).__name__}: {exc}"
            record(step, status, reason=ctx.stopped_reason)
            if not motion_attempted:
                return Result(status, ctx.stopped_reason,
                              {"requires_operator_review": True, "failed_step": step,
                               "motion_dispatched": False})
            try:
                stopped = io.stop(ctx.stopped_reason)
            except Exception as stop_exc:
                stopped = {"confirmed": False, "error": str(stop_exc)}
            record("stop", "ok" if stopped.get("confirmed") is True else "unknown", evidence=stopped)
            if stopped.get("confirmed") is not True:
                status = "unknown"
            return Result(status, ctx.stopped_reason,
                          {"requires_operator_review": True, "stop": stopped, "failed_step": step})
