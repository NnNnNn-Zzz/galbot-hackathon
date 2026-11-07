"""Vision-relative right-arm drawer orchestration; no implicit hardware backend.

Backend must continuously enforce force, collision, fixed-body and tracking
limits DURING each blocking motion, with a bounded timeout and physical stop.
Observations use host monotonic seconds. Opening is drawer-relative-to-cabinet,
measured from the closed reference, never inferred from the robot TCP.
"""
import math
import time
import numpy as np
from medicine_frontend.drawer_geometry import grasp_targets, targets, transform

REQUIRED = ("observe", "plan_right_arm", "execute_right_arm", "grip_right", "stop_right_arm")


def run(io, calibration, *, clock=time.monotonic):
    trace, attempted = [], False
    stage = "preflight"
    try:
        if any(not callable(getattr(io, name, None)) for name in REQUIRED):
            raise ValueError("missing_visual_drawer_backend")
        if calibration.get("verified") is not True:
            raise ValueError("handle_relative_calibration_unverified")
        grasp_mode = calibration.get("grasp_mode", "teaching")
        if grasp_mode == "teaching":
            tcp_in_handle = transform(calibration["tcp_in_handle"])
        elif grasp_mode != "midpoint":
            raise ValueError("unknown_grasp_mode:" + str(grasp_mode))

        def observe():
            value = io.observe()
            age = clock() - float(value["sample_monotonic_s"])
            if not math.isfinite(age) or not 0 <= age <= 0.5:
                raise ValueError("stale_observation")
            for key in ("metric_geometry_verified", "cabinet_reference_verified",
                        "body_stationary", "exclusive_right_arm_control", "path_clear",
                        "continuous_guards_ready"):
                if value.get(key) is not True:
                    raise ValueError(key)
            for key in ("opening_m", "opening_uncertainty_m", "available_opening_m"):
                if not math.isfinite(float(value[key])) or value[key] < 0:
                    raise ValueError(key)
            if value["opening_uncertainty_m"] > 0.003:
                raise ValueError("opening_measurement_uncertain")
            if value["drawer_id"] != calibration["drawer_id"]:
                raise ValueError("wrong_drawer")
            return value

        def completed(result):
            if (not isinstance(result, dict) or result.get("success") is not True
                    or result.get("completion") != "completed" or result.get("dry_run")):
                raise ValueError("action_completion_unverified")

        state = observe()
        if state["opening_m"] > 0.003 or state["available_opening_m"] < 0.30:
            raise ValueError("require_closed_drawer_and_30cm_travel")
        if grasp_mode == "midpoint":
            poses = grasp_targets(
                state["handle_to_base"],
                grasp_offset_m=float(calibration.get("grasp_offset_m", 0.0)),
                gripper_in_handle=calibration.get("gripper_in_handle"))
        else:
            poses = targets(state["handle_to_base"], tcp_in_handle)
        # Planner must solve continuous IK and collision checks along ALL segments,
        # including swept volume of the drawer; only seven right-arm joints may vary.
        plan = io.plan_right_arm(poses, fixed_body=state["body_reference"],
                                 cartesian_pull=True, side="right")
        for key in ("success", "right_arm_only", "full_path_verified", "continuous_ik",
                    "collision_checked", "joint_limits_checked", "singularity_checked"):
            if plan.get(key) is not True:
                raise ValueError("planning_failed:" + key)
        planned_handle = transform(state["handle_to_base"])
        latest = observe()  # planning can take long enough to invalidate a snapshot
        if not np.allclose(transform(latest["handle_to_base"]), planned_handle, atol=0.002):
            raise ValueError("scene_changed_during_planning")
        for stage in ("pregrasp", "grasp"):
            attempted = True
            completed(io.execute_right_arm(plan, stage=stage, index=None))
            state = observe()
            if state.get("target_reached") is not True:
                raise ValueError("alignment_not_reached")
            trace.append({"stage": stage})
        stage = "grip"
        attempted = True
        completed(io.grip_right(width_m=0.020, speed_m_s=0.020, effort=10.0))
        state = observe()
        if state.get("grip_verified") is not True or state["opening_m"] > 0.003:
            raise ValueError("grasp_unverified_or_drawer_moved")
        start_tcp = transform(state["tcp_to_base"])[:3, 3]
        normal = np.asarray(poses["normal_base"])
        previous_opening = state["opening_m"]
        for index in range(30):
            stage = "probe" if index == 0 else "pull"
            state = observe()
            if state.get("grip_verified") is not True:
                raise ValueError("grip_lost")
            completed(io.execute_right_arm(plan, stage="pull", index=index))
            state = observe()
            delta = transform(state["tcp_to_base"])[:3, 3] - start_tcp
            along = float(delta @ normal)
            lateral = float(np.linalg.norm(delta-along*normal))
            opening = state["opening_m"]
            trace.append({"stage": stage, "opening_m": opening,
                          "tcp_along_m": along, "tcp_lateral_m": lateral})
            if (state.get("grip_verified") is not True or state.get("force_within_limits") is not True
                    or lateral > 0.005 or abs(along-opening) > 0.008
                    or opening < previous_opening-0.002 or opening > 0.305):
                raise ValueError("pull_tracking_or_contact_failure")
            if opening-previous_opening < 0.002:
                raise ValueError("drawer_not_moving")
            if index == 0 and state.get("rail_direction_verified") is not True:
                raise ValueError("probe_direction_unverified")
            previous_opening = opening
            if abs(opening-0.30) <= 0.003:
                completed(io.stop_right_arm())
                return {"success": True, "feedback": "Measured drawer opening reached 30 cm tolerance",
                        "opening_m": opening, "motion_executed": True, "trace": trace}
        raise ValueError("30cm_actual_opening_not_reached")
    except Exception as error:
        stop = None
        if attempted:
            try:
                stop = io.stop_right_arm()
            except Exception as stop_error:
                stop = {"success": False, "error": str(stop_error)}
        return {"success": False, "feedback": str(error), "stopped_at": stage,
                "motion_executed": attempted, "trace": trace, "stop_result": stop}
