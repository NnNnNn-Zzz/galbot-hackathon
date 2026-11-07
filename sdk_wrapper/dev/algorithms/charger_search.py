"""Deterministic state machine for visual head-camera search and approach."""

from __future__ import annotations

import math
import typing

__all__ = ["PHASES", "plan_head_search_action"]

PHASES = (
    "find_power_strip",
    "approach_power_strip",
    "navigating_to_observation",
    "find_charger",
    "approach_charger",
    "ready_for_wrist",
    "wrist_view_aligned",
)


def _number(mapping: typing.Mapping[str, typing.Any] | None, key: str, default: float) -> float:
    if not mapping:
        return float(default)
    try:
        return float(mapping.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _action(
    *,
    phase: str,
    next_phase: str,
    action: str,
    reason: str,
    forward_m: float = 0.0,
    yaw_deg: float = 0.0,
    metric_distance_used: bool = False,
) -> dict[str, typing.Any]:
    return {
        "phase": phase,
        "next_phase": next_phase,
        "action": action,
        "forward_m": round(float(forward_m), 6),
        "yaw_deg": round(float(yaw_deg), 6),
        "requires_motion": bool(abs(forward_m) > 1e-9 or abs(yaw_deg) > 1e-9),
        "metric_distance_used": bool(metric_distance_used),
        "reason": reason,
        "grasp_coordinate": None,
    }


def plan_head_search_action(
    phase: str,
    *,
    strip: typing.Mapping[str, typing.Any] | None,
    charger: typing.Mapping[str, typing.Any] | None,
    config: typing.Mapping[str, typing.Any],
    step_count: int,
) -> dict[str, typing.Any]:
    """Plan exactly one small visual-servo step.

    The planner uses image bearing and apparent target size, not bbox-center
    distance. Every forward move must be followed by a new detection.
    """
    if phase not in PHASES:
        raise ValueError(f"unknown charger search phase {phase!r}")
    if phase == "ready_for_wrist":
        return _action(
            phase=phase,
            next_phase=phase,
            action="none",
            reason="base_locked_waiting_for_wrist_handoff",
        )
    if phase == "wrist_view_aligned":
        return _action(
            phase=phase,
            next_phase=phase,
            action="none",
            reason="wrist_camera_aligned_waiting_for_depth_handoff",
        )
    if phase == "navigating_to_observation":
        return _action(
            phase=phase,
            next_phase=phase,
            action="track_navigation_task",
            reason="slam_navigation_in_progress",
        )

    max_steps = int(config.get("max_steps", 24))
    if int(step_count) >= max_steps:
        return _action(
            phase=phase,
            next_phase="find_power_strip",
            action="abort",
            reason="maximum_visual_search_steps_reached",
        )

    center_tolerance = _number(config, "center_tolerance_deg", 4.0)
    scan_turn = _number(config, "search_turn_deg", 12.0)
    max_turn = _number(config, "max_turn_command_deg", 15.0)
    backoff = _number(config, "backoff_m", 0.05)

    if phase in ("find_power_strip", "approach_power_strip"):
        if not strip or not strip.get("usable_for_direction"):
            return _action(
                phase=phase,
                next_phase="find_power_strip",
                action="rotate_search",
                yaw_deg=scan_turn,
                reason="power_strip_not_visible_or_unstable",
            )
        angle = _number(strip, "horizontal_angle_deg", 0.0)
        if strip.get("touches_image_edge"):
            return _action(
                phase=phase,
                next_phase="approach_power_strip",
                action="backoff",
                forward_m=-backoff,
                reason="power_strip_clipped_move_back",
            )
        if abs(angle) > center_tolerance:
            # Camera +x/right corresponds to robot negative yaw (right turn).
            yaw = max(-max_turn, min(max_turn, -angle))
            return _action(
                phase=phase,
                next_phase="approach_power_strip",
                action="turn_toward_power_strip",
                yaw_deg=yaw,
                reason="recenter_power_strip_before_forward_step",
            )
        stereo = strip.get("stereo") or {}
        observation = strip.get("observation_plan") or {}
        if (
            observation.get("safe_to_submit")
            and observation.get("path_reachable")
            and observation.get("observation_pose_map")
        ):
            return _action(
                phase=phase,
                next_phase="navigating_to_observation",
                action="navigate_to_mapped_observation",
                reason="metric_target_and_reachable_slam_observation_pose",
                metric_distance_used=True,
            )
        if stereo.get("measurement_stable") and stereo.get("metric_distance_available"):
            return _action(
                phase=phase,
                next_phase="find_charger",
                action="hold_for_mapped_observation_pose",
                reason="stereo_metric_ready_but_slam_observation_pose_unavailable",
                metric_distance_used=True,
            )
        return _action(
            phase=phase,
            next_phase="find_power_strip",
            action="hold_for_stable_metric_stereo",
            reason="target_direction_known_but_metric_stereo_or_slam_pose_unavailable",
        )

    # Once the strip is in the observation band, stop using it as a navigation
    # target. The head charger model now decides if the arm/wrist can take over.
    if charger and charger.get("usable_for_wrist_handoff"):
        return _action(
            phase=phase,
            next_phase="ready_for_wrist",
            action="lock_base_for_wrist",
            reason="charger_top_complete_and_wrist_handoff_criteria_met",
        )

    if charger and charger.get("touches_image_edge"):
        return _action(
            phase=phase,
            next_phase="approach_charger",
            action="backoff",
            forward_m=-backoff,
            reason="charger_clipped_move_back",
        )
    if charger:
        angle = _number(charger, "horizontal_angle_deg", 0.0)
        if abs(angle) > center_tolerance:
            yaw = max(-max_turn, min(max_turn, -angle))
            return _action(
                phase=phase,
                next_phase="approach_charger",
                action="turn_toward_charger",
                yaw_deg=yaw,
                reason="recenter_charger_before_forward_step",
            )

    return _action(
        phase=phase,
        next_phase="find_charger",
        action="adjust_head_to_search_charger",
        reason="charger_not_ready_base_locked_use_head_or_arm_view_adjustment",
    )
