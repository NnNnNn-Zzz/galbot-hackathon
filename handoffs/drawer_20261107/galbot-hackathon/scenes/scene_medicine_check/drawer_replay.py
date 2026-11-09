"""Supervised, guarded replay of the 2026-11-07 drawer-pull command sequence.

The recorded data contains command/checkpoint pairs, not a continuous arm
trajectory. Real execution requires a shared-process sdk_wrapper adapter with
the four methods in REQUIRED_IO. Missing capabilities fail before any action.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any


RECIPE_PATH = Path(__file__).with_name("data") / "recorded_drawer_pull_20261107.json"
REQUIRED_IO = ("get_drawer_state", "set_gripper_command",
               "move_end_effector_offset", "stop_controller")


def load_recipe(path: Path = RECIPE_PATH) -> dict[str, Any]:
    recipe = json.loads(Path(path).read_text(encoding="utf-8"))
    if recipe.get("schema") != "drawer_replay_recipe/1":
        raise ValueError("Unsupported drawer replay recipe")
    targets = recipe.get("pull_targets_cumulative_x_m")
    if targets != [0.02, 0.07, 0.12, 0.17, 0.22, 0.25, 0.275, 0.30]:
        raise ValueError("Recorded drawer target sequence changed")
    return recipe


def plan(path: Path = RECIPE_PATH) -> dict[str, Any]:
    recipe = load_recipe(path)
    return {"success": True, "motion_executed": False,
            "feedback": "仅列出拉屉示教步骤，未执行机器人动作",
            "suggested_adjustment": None,
            "recipe": str(path), "grip_command": recipe["grip_command"],
            "targets_m": recipe["pull_targets_cumulative_x_m"],
            "recorded_stops_after_steps": recipe["recorded_stop_after_step"],
            "unattended_replay_validated": False}


def _angle_deg(a, b):
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))


def _state(io):
    """Expected keys: tcp_base_link[7], base_map{x,y,qz,qw}, right_gripper,
    right_arm_joints_rad{name: position}, localized=True, sample_ns. All
    values must come from fresh SDK reads, not a cached scene fixture.
    """
    value = io.get_drawer_state()
    tcp = value["tcp_base_link"]
    base = value["base_map"]
    grip = value["right_gripper"]
    joints = value["right_arm_joints_rad"]
    sampled = value.get("sample_ns")
    if (len(tcp) != 7 or not all(math.isfinite(float(x)) for x in tcp)
            or not all(math.isfinite(float(base[k])) for k in ("x", "y", "qz", "qw"))
            or not all(math.isfinite(float(grip[k])) for k in ("width", "effort"))
            or len(joints) != 7 or value.get("localized") is not True
            or not isinstance(sampled, int) or isinstance(sampled, bool)
            or not 0 <= time.time_ns() - sampled <= 3_000_000_000):
        raise ValueError("Incomplete drawer state")
    return value


def _accepted(result):
    if not isinstance(result, dict):
        return False
    if result.get("dry_run") is True:
        return False
    if result.get("success") is True and result.get("completion") == "completed":
        return True
    return (result.get("accepted") is True
            and str(result.get("sdk_result", "")).endswith("SUCCESS"))


def _base_shift(state, reference):
    return math.hypot(state["base_map"]["x"] - reference["x"],
                      state["base_map"]["y"] - reference["y"])


def _grip_ok(state):
    grip = state["right_gripper"]
    return (0.020 <= grip["width"] <= 0.030
            and grip["effort"] >= 4.0 and not grip.get("is_moving", False))


def _stop(io, trace):
    try:
        result = io.stop_controller("right_arm")
        trace.append({"action": "stop_right_arm", "result": result})
    except Exception as error:  # stop failure must be visible to the caller
        trace.append({"action": "stop_right_arm", "error": str(error)})


def _failure(reason, trace, *, motion_executed):
    return {"success": False, "motion_executed": motion_executed,
            "feedback": reason, "trace": trace}


def run(params: dict[str, Any] | None = None, *, io=None,
        recipe_path: Path = RECIPE_PATH) -> dict[str, Any]:
    """Replay the same grip and cumulative X targets from a checked start pose.

    The original 25 and 30 cm checkpoints exceeded fixed-axis tolerance. To
    reproduce the actual continuation, an operator must explicitly permit the
    25 cm rebase and the observed drift; the result then reports that drift.
    """
    params = dict(params or {})
    recipe = load_recipe(recipe_path)
    if params.get("dry_run"):
        return plan(recipe_path)
    if io is None:
        return _failure("缺少已接入的抽屉控制后端，未发送动作", [], motion_executed=False)
    missing = [name for name in REQUIRED_IO if not callable(getattr(io, name, None))]
    if missing:
        return _failure(f"抽屉控制接口缺失：{', '.join(missing)}；未发送动作", [],
                        motion_executed=False)
    if not all(params.get(key) is True for key in (
            "operator_alignment_confirmed", "operator_support_confirmed",
            "operator_travel_confirmed")):
        return _failure("缺少现场对位、扶稳或抽屉行程确认；未发送动作", [],
                        motion_executed=False)

    trace: list[dict[str, Any]] = []
    moved = False
    try:
        state = _state(io)
        reference = recipe["pre_grasp_tcp_base_link"]
        base_reference = recipe["alignment_base_map"]
        arm_reference = recipe["alignment_right_arm_joints_rad"]
        if (math.dist(state["tcp_base_link"][:3], reference[:3]) > 0.015
                or _angle_deg(state["tcp_base_link"][3:], reference[3:]) > 3.0
                or _base_shift(state, base_reference) > 0.04
                or abs(state["right_gripper"]["width"]
                       - recipe["pre_grasp_right_gripper_width_m"]) > 0.01
                or any(abs(state["right_arm_joints_rad"][name] - value) > 0.12
                       for name, value in arm_reference.items())):
            return _failure("起始底盘、右臂、末端或夹爪与示教对位不符；未发送动作", trace,
                            motion_executed=False)

        grip_result = io.set_gripper_command(**recipe["grip_command"])
        moved = True
        trace.append({"action": "close_right_gripper", "result": grip_result})
        if not _accepted(grip_result):
            _stop(io, trace)
            return _failure("右夹爪闭合未确认", trace, motion_executed=True)
        state = _state(io)
        if not _grip_ok(state):
            _stop(io, trace)
            return _failure("右夹爪反馈未确认夹持", trace, motion_executed=True)

        start_tcp = list(state["tcp_base_link"])
        stage_reference = start_tcp
        constraints_met = True
        for index, distance in enumerate(recipe["pull_targets_cumulative_x_m"], 1):
            if index == 7:
                if params.get("operator_extension_confirmed") is not True:
                    _stop(io, trace)
                    return _failure("25 cm 后尚未确认继续拉出，停在当前抽屉开度", trace,
                                    motion_executed=True)
                stage_reference = list(state["tcp_base_link"])
            target_x = start_tcp[0] - distance
            current = state["tcp_base_link"]
            offset = {"arm": "right_arm", "dx": target_x - current[0],
                      "dy": stage_reference[1] - current[1],
                      "dz": stage_reference[2] - current[2],
                      "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            if not -0.1 <= offset["dx"] < 0 or abs(offset["dy"]) > 0.02 or abs(offset["dz"]) > 0.02:
                _stop(io, trace)
                return _failure(f"第 {index} 段偏移超出示教范围", trace, motion_executed=True)
            result = io.move_end_effector_offset(**offset)
            trace.append({"action": "pull_right_tcp", "step": index,
                          "target_cumulative_x_m": distance, "command": offset,
                          "result": result})
            if not _accepted(result):
                _stop(io, trace)
                return _failure(f"第 {index} 段运动未被确认", trace, motion_executed=True)
            deadline = time.monotonic() + 12.0
            while True:
                state = _state(io)
                if abs(state["tcp_base_link"][0] - target_x) <= 0.008:
                    break
                if time.monotonic() >= deadline:
                    _stop(io, trace)
                    return _failure(f"第 {index} 段未到达目标 X", trace, motion_executed=True)
                time.sleep(0.2)
            if not _grip_ok(state) or _base_shift(state, base_reference) > 0.03:
                _stop(io, trace)
                return _failure(f"第 {index} 段夹持或底盘状态异常", trace, motion_executed=True)
            xyz_error = [state["tcp_base_link"][axis] - stage_reference[axis]
                         for axis in (1, 2)]
            angle_error = _angle_deg(state["tcp_base_link"][3:], stage_reference[3:])
            within_axes = max(abs(x) for x in xyz_error) <= 0.005 and angle_error <= 2.0
            trace[-1].update({"observed_tcp_base_link": state["tcp_base_link"],
                              "y_z_error_m": xyz_error, "orientation_error_deg": angle_error,
                              "other_axes_within_tolerance": within_axes})
            if not within_axes:
                constraints_met = False
                _stop(io, trace)
                if index not in (6, 8) or params.get("allow_recorded_axis_drift") is not True:
                    return _failure(f"第 {index} 段末端其他坐标超差，已停右臂", trace,
                                    motion_executed=True)
                if index == 6 and params.get("operator_extension_confirmed") is not True:
                    return _failure("25 cm 处超差，等待现场确认", trace,
                                    motion_executed=True)
        _stop(io, trace)
        return {"success": constraints_met, "motion_executed": True,
                "command_sequence_completed": True, "other_axes_within_tolerance": constraints_met,
                "feedback": "已复刻分段命令；末端其他坐标超差" if not constraints_met
                            else "分段拉抽屉完成，末端约束通过",
                "trace": trace, "final_tcp_base_link": state["tcp_base_link"]}
    except Exception as error:
        if moved:
            _stop(io, trace)
        return _failure(f"抽屉复刻状态不确定：{error}", trace, motion_executed=moved)
