#!/usr/bin/env python3
"""Keyboard control for G1 leg joints 1-3, one joint per command."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper.dev.robot_tools.leg_joint_control import (
    CONTROL_JOINT_NAMES,
    LegJointControlTool,
)

MAX_SESSION_OFFSET_RAD = 0.05


def _joint_number(name: str) -> str:
    return name.rsplit("_", 1)[-1]


def print_step(result) -> None:
    data = result.data
    rpy_deg = [math.degrees(value) for value in data["torso_rpy_delta_rad"]]
    print(
        "OK {} target={:.4f} actual_delta={:+.4f} rad | "
        "height {:+0.2f} mm | torso roll/pitch/yaw {:+0.3f}/{:+.3f}/{:+.3f}° | "
        "other drift {:.4f} rad".format(
            data["joint_name"],
            data["target_joint_rad"],
            data["actual_delta_rad"],
            data["height_delta_m"] * 1000.0,
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
            data["other_leg_joint_max_drift_rad"],
        ),
        flush=True,
    )


def main() -> int:
    if not sys.stdin.isatty():
        print("拒绝运行：必须使用交互终端。", file=sys.stderr)
        return 2

    tool = LegJointControlTool(real_authorized=True)
    initial = tool.get_leg_joint_state()
    if not initial.succeeded:
        print(initial.to_dict(), flush=True)
        return 1
    baseline = {
        name: float(initial.data["leg_joint_positions_rad"][name])
        for name in CONTROL_JOINT_NAMES
    }
    current = dict(baseline)
    selected = CONTROL_JOINT_NAMES[0]

    print("G1 腿部三关节键盘控制：每条命令只命令一个关节。")
    print(
        "初始：{}；高度 {:.4f} m；单关节相对起始偏移最多 ±{:.3f} rad".format(
            ", ".join("%s=%.4f" % (name, value) for name, value in baseline.items()),
            float(initial.data["height_m"]),
            MAX_SESSION_OFFSET_RAD,
        )
    )
    print("1/2/3 选择关节；j/k 减小/增加 0.005 rad；J/K 减小/增加 0.01 rad。")
    print("r 恢复三个关节到起始；q 退出。")
    if input("确认周围安全并由你逐键控制（y）：").strip().lower() != "y":
        print("已取消，未初始化动作控制。")
        return 0

    while True:
        prompt = "{}({:.4f})> ".format(_joint_number(selected), current[selected])
        command = input(prompt).strip()
        if not command:
            continue
        if command in {"1", "2", "3"}:
            selected = CONTROL_JOINT_NAMES[int(command) - 1]
            print("已选择 %s" % selected)
            continue
        if command == "q":
            break
        if command == "r":
            result = tool.restore_leg_joints(
                baseline, operator_confirmed=True
            )
            print(result.to_dict(), flush=True)
            if not result.succeeded:
                return 1
            current = dict(baseline)
            continue
        if command not in {"j", "k", "J", "K"}:
            print("未知命令；使用 1/2/3/j/k/J/K/r/q。")
            continue

        magnitude = 0.005 if command in {"j", "k"} else 0.01
        delta = -magnitude if command in {"j", "J"} else magnitude
        proposed = current[selected] + delta
        if abs(proposed - baseline[selected]) > MAX_SESSION_OFFSET_RAD + 1e-9:
            print("拒绝：%s 相对起始偏移将超过 %.3f rad。" % (selected, MAX_SESSION_OFFSET_RAD))
            continue
        result = tool.move_leg_joint_step(
            selected, delta, operator_confirmed=True
        )
        if not result.succeeded:
            print(result.to_dict(), flush=True)
            print("本步未明确成功；停止后续动作，请先现场确认机器人稳定。")
            return 1
        print_step(result)
        current[selected] = float(result.data["actual_joint_rad"])

    if any(abs(current[name] - baseline[name]) > 1e-4 for name in CONTROL_JOINT_NAMES):
        answer = input("退出前恢复三个关节到起始位置？按 Enter/y 恢复，输入 k 保持：").strip().lower()
        if answer != "k":
            result = tool.restore_leg_joints(
                baseline, operator_confirmed=True
            )
            print(result.to_dict(), flush=True)
            if not result.succeeded:
                return 1
    print("腿部三关节键盘控制结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
