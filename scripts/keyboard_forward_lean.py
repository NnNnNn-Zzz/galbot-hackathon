#!/usr/bin/env python3
"""Keyboard-controlled forward lean using only G1 leg_joint3."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper.dev.robot_tools.forward_lean import (
    LEAN_JOINT_NAME,
    ForwardLeanTool,
)

MAX_SESSION_OFFSET_RAD = 0.05


def format_step_success(result) -> str:
    data = result.data
    return (
        "OK forward_lean {}={:.4f} rad | delta={:+.4f} rad | "
        "head_x={:+.2f} mm | pitch={:+.3f}° | other_drift={:.4f} rad"
    ).format(
        LEAN_JOINT_NAME,
        data["actual_joint_rad"],
        data["actual_delta_rad"],
        data["head_x_delta_m"] * 1000.0,
        math.degrees(data["head_pitch_delta_rad"]),
        data["other_leg_joint_max_drift_rad"],
    )


def main() -> int:
    if not sys.stdin.isatty():
        print("拒绝运行：必须使用交互终端。", file=sys.stderr)
        return 2

    tool = ForwardLeanTool(real_authorized=True)
    initial = tool.get_forward_lean_state()
    if not initial.succeeded:
        print(initial.to_dict(), flush=True)
        return 1
    home = float(initial.data["joint_rad"])
    current = home

    print("G1 前倾控制：只命令 leg_joint3；增加=前倾，减少=回正/后仰。")
    print(
        "起始 leg_joint3={:.4f} rad；本次相对起始偏移最多 ±{:.3f} rad。".format(
            home, MAX_SESSION_OFFSET_RAD
        )
    )
    print("f/F 前倾 0.005/0.01 rad；b/B 回正 0.005/0.01 rad；r 回起始；q 退出。")
    if input("确认周围安全并由你逐键控制（y）：").strip().lower() != "y":
        print("已取消，未发送动作。")
        return 0

    while True:
        command = input("forward({:.4f})> ".format(current)).strip()
        if not command:
            continue
        if command == "q":
            break
        if command == "r":
            result = tool.restore_lean_joint(home, operator_confirmed=True)
            print(result.to_dict(), flush=True)
            if not result.succeeded:
                return 1
            current = float(result.data["actual_rad"])
            continue
        if command not in {"f", "F", "b", "B"}:
            print("未知命令；使用 f/F/b/B/r/q。")
            continue

        magnitude = 0.005 if command in {"f", "b"} else 0.01
        direction = 1 if command in {"f", "F"} else -1
        proposed = current + direction * magnitude
        if abs(proposed - home) > MAX_SESSION_OFFSET_RAD + 1e-9:
            print(
                "拒绝：leg_joint3 相对起始偏移将超过 {:.3f} rad。".format(
                    MAX_SESSION_OFFSET_RAD
                )
            )
            continue
        result = (
            tool.lean_forward_step(magnitude, operator_confirmed=True)
            if direction > 0
            else tool.lean_backward_step(magnitude, operator_confirmed=True)
        )
        if not result.succeeded:
            print(result.to_dict(), flush=True)
            print("本步未明确成功；停止后续动作，请先确认机器人稳定。")
            return 1
        current = float(result.data["actual_joint_rad"])
        print(format_step_success(result), flush=True)

    if abs(current - home) > 1e-4:
        answer = input("退出前恢复 leg_joint3 到起始角度？按 Enter/y 恢复，输入 k 保持：").strip().lower()
        if answer != "k":
            result = tool.restore_lean_joint(home, operator_confirmed=True)
            print(result.to_dict(), flush=True)
            if not result.succeeded:
                return 1
    print("前倾控制结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
