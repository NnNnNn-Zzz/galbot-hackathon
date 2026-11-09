#!/usr/bin/env python3
"""Staged test for wrist charger recognition -> pre-grasp EndEffector pose.

Stages:
    detect  only run YOLO-seg + D405 geometry and print the surface result;
    plan    detect, then generate an absolute EndEffector pre-grasp pose;
    move    detect + plan again, then command the pose after confirmation.

The default stage is ``plan`` so the safe first test never commands the arm.
"""

from __future__ import annotations

import argparse
import json
import sys
import typing
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _print_result(label: str, result: dict[str, typing.Any]) -> None:
    status = result.get("status")
    print(f"\n[{label}] status={status} success={status == 'succeeded'}")
    print(result.get("message", ""))
    if result.get("error_code"):
        print(f"error_code={result['error_code']}")

    data = result.get("data")
    if not isinstance(data, dict):
        data = result
    observation = data.get("observation")
    if isinstance(observation, dict):
        surface = observation.get("surface") or {}
        print(
            "observation:",
            json.dumps(
                {
                    "camera_id": observation.get("camera_id"),
                    "confidence": observation.get("confidence"),
                    "detection_stage": observation.get("detection_stage"),
                    "proposal_confidence": observation.get("proposal_confidence"),
                    "cascade": observation.get("cascade"),
                    "capture_attempt": observation.get("capture_attempt"),
                    "capture_attempt_reports": observation.get(
                        "capture_attempt_reports"
                    ),
                    "target_mode": observation.get("target_mode"),
                    "whole_fallback_sampling": observation.get(
                        "whole_fallback_sampling"
                    ),
                    "eligible_for_precision": observation.get("eligible_for_precision"),
                    "rejection_reasons": observation.get("rejection_reasons"),
                    "surface_center_m": surface.get("surface_center_m"),
                    "surface_normal_base": surface.get("surface_normal_base"),
                    "long_edge_axis_base": surface.get("long_edge_axis_base"),
                    "short_dimension_m": surface.get("short_dimension_m"),
                    "long_dimension_m": surface.get("long_dimension_m"),
                    "overlay_ref": observation.get("overlay_ref"),
                    "observation_ref": observation.get("observation_ref"),
                },
                ensure_ascii=False,
                indent=2,
            ),
        )
    elif "surface" in data:
        surface = data.get("surface") or {}
        print(
            "surface:",
            json.dumps(
                {
                    "eligible_for_precision": data.get("eligible_for_precision"),
                    "detection_stage": data.get("detection_stage"),
                    "proposal_confidence": data.get("proposal_confidence"),
                    "cascade": data.get("cascade"),
                    "capture_attempt": data.get("capture_attempt"),
                    "capture_attempt_reports": data.get(
                        "capture_attempt_reports"
                    ),
                    "target_mode": data.get("target_mode"),
                    "whole_fallback_sampling": data.get(
                        "whole_fallback_sampling"
                    ),
                    "rejection_reasons": data.get("rejection_reasons"),
                    "surface_center_m": surface.get("surface_center_m"),
                    "surface_normal_base": surface.get("surface_normal_base"),
                    "long_edge_axis_base": surface.get("long_edge_axis_base"),
                    "short_dimension_m": surface.get("short_dimension_m"),
                    "long_dimension_m": surface.get("long_dimension_m"),
                    "overlay_ref": data.get("overlay_ref"),
                    "observation_ref": data.get("observation_ref"),
                },
                ensure_ascii=False,
                indent=2,
            ),
        )

    if "end_effector_goal" in data:
        print(
            "end_effector_goal:",
            json.dumps(data["end_effector_goal"], ensure_ascii=False),
        )
        plan = data.get("grasp_plan") or data.get("plan", {}).get("grasp_plan") or {}
        print(
            "grasp_plan:",
            json.dumps(
                {
                    "observation_position_m": plan.get("observation_position_m"),
                    "tcp_observation_position_m": plan.get(
                        "tcp_observation_position_m"
                    ),
                    "observation_distance_m": plan.get("observation_distance_m"),
                    "tilt_from_normal_deg": plan.get("tilt_from_normal_deg"),
                    "translation_to_goal_m": data.get("translation_to_goal_m"),
                    "approach_axis_base": plan.get("approach_axis_base"),
                    "gripper_long_axis_base": plan.get("gripper_long_axis_base"),
                    "jaw_closing_axis_base": plan.get("jaw_closing_axis_base"),
                    "standoff_m": plan.get("standoff_m"),
                    "rotation_from_current_deg": plan.get("rotation_from_current_deg"),
                },
                ensure_ascii=False,
                indent=2,
            ),
        )
    if data.get("descent_plan"):
        print(
            "descent_plan:",
            json.dumps(data["descent_plan"], ensure_ascii=False, indent=2),
        )
    if data.get("gripper_open"):
        print(
            "gripper_open:",
            json.dumps(data["gripper_open"], ensure_ascii=False, indent=2),
        )
    if data.get("gripper_close"):
        print(
            "gripper_close:",
            json.dumps(data["gripper_close"], ensure_ascii=False, indent=2),
        )
    if data.get("ik_preflight"):
        ik = data["ik_preflight"]
        print(
            "ik_preflight:",
            json.dumps(
                {
                    "status": ik.get("status"),
                    "message": ik.get("message"),
                    "reachable": (ik.get("data") or {}).get("reachable"),
                    "motion_status": (ik.get("data") or {}).get(
                        "motion_status"
                    ),
                    "collision_check": (ik.get("data") or {}).get(
                        "collision_check"
                    ),
                },
                ensure_ascii=False,
            ),
        )
    if data.get("motion"):
        print("motion:", json.dumps(data["motion"], ensure_ascii=False))
    if data.get("plan_result"):
        print(
            "replan_result:",
            json.dumps(data["plan_result"], ensure_ascii=False, indent=2),
        )
    if "position_error_m" in data:
        print(
            "pose verification:",
            json.dumps(
                {
                    "position_error_m": data.get("position_error_m"),
                    "orientation_error_deg": data.get("orientation_error_deg"),
                    "pose_matched": data.get("pose_matched"),
                    "grasp_completed": data.get("grasp_completed"),
                },
                ensure_ascii=False,
            ),
        )


def _confirm_motion(arm: str) -> bool:
    print(
        f"\n即将下发{'左' if arm == 'left' else '右'}臂绝对 EndEffector 位姿。"
        "请确认急停可用、手臂活动区无人无物，且目标位姿已在计划输出中核对。"
    )
    try:
        answer = input("输入 MOVE 继续，其它内容取消：").strip()
    except EOFError:
        return False
    return answer == "MOVE"


def _confirm_grasp(arm: str) -> bool:
    print(
        f"\n即将执行{'左' if arm == 'left' else '右'}手斜向观察后的下探、闭爪和抬起。"
        "请确认夹爪已全开、目标仍在视野内、下方区域安全且急停可用。"
    )
    try:
        answer = input("输入 GRASP 继续，其它内容取消：").strip()
    except EOFError:
        return False
    return answer == "GRASP"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="腕部 charger YOLO-seg -> 坐标 -> 末端 pre-grasp 位姿分阶段测试"
    )
    parser.add_argument(
        "--stage",
        choices=("detect", "plan", "move", "grasp"),
        default="plan",
        help=(
            "detect=只识别，plan=生成斜上方观察位，"
            "move=移动到斜上方并全开夹爪后二次观测，"
            "grasp=确认后下探、闭爪和抬起（默认 plan）"
        ),
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="使用 MockAdapter；不连接真机",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过 move 阶段的 MOVE 交互确认；仅建议在 mock 使用",
    )
    parser.add_argument(
        "--arm",
        choices=("left", "right"),
        default="left",
        help="执行和腕部观察使用的手臂（默认 left）",
    )
    args = parser.parse_args(argv)

    if args.yes and not args.mock and args.stage in ("move", "grasp"):
        parser.error("真机 move/grasp 阶段不允许 --yes；必须交互确认")

    from sdk_wrapper.dev import toolhub
    from sdk_wrapper.dev.robot_tools.charger_grasp import ChargerGraspTool
    from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool

    toolhub.configure(mock=bool(args.mock), force=True)
    if not args.mock:
        import atexit

        atexit.register(toolhub.shutdown)
    mode = toolhub.describe()
    print(
        f"mode={mode.get('mode')} adapter={mode.get('adapter')} "
        f"stage={args.stage} arm={args.arm}"
    )
    charger_tool = ChargerYoloTool(toolhub.perception(), arm=args.arm)
    grasp_tool = ChargerGraspTool(
        toolhub.perception(),
        charger_tool=charger_tool,
        motion_tool=toolhub.motion(),
        arm=args.arm,
    )
    if not args.mock:
        wrist = "左腕" if args.arm == "left" else "右腕"
        print(f"真机测试会读取{wrist} RGB/D405 并调用 YOLO-seg；plan 阶段不会下发动作。")

    if args.stage == "detect":
        result = charger_tool.segment_charger_top_wrist().to_dict()
        _print_result("detect", result)
        return 0 if result.get("status") == "succeeded" else 1

    if args.stage == "plan":
        result = grasp_tool.plan_wrist_charger_pregrasp().to_dict()
        _print_result("plan", result)
        return 0 if result.get("status") == "succeeded" else 1

    if args.stage == "grasp":
        if not args.yes and not _confirm_grasp(args.arm):
            print("已取消，未下发运动。")
            return 2
        result = grasp_tool.execute_wrist_charger_grasp(
            operator_confirmed=True
        ).to_dict()
        _print_result("grasp", result)
        return 0 if result.get("status") == "succeeded" else 1

    # Re-observe and preflight IK before asking the operator to authorize motion.
    # Otherwise a failed fresh observation looks like an unexplained rejection
    # after the operator has already typed MOVE.
    preflight = grasp_tool.plan_wrist_charger_pregrasp().to_dict()
    _print_result("move_preflight", preflight)
    if preflight.get("status") != "succeeded":
        print("预检未通过，未要求 MOVE，也未下发运动。")
        return 1

    if not args.yes and not _confirm_motion(args.arm):
        print("已取消，未下发运动。")
        return 2
    result = grasp_tool.move_to_wrist_charger_pregrasp(
        operator_confirmed=True
    ).to_dict()
    _print_result("move", result)
    return 0 if result.get("status") == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
