#!/usr/bin/env python3
"""Keyboard-controlled, supervised G1 head pitch motion."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper.dev.robot_tools.head import (
    HEAD_JOINT_NAME,
    HEAD_MEASUREMENT_FRAME,
    MAX_HEAD_PITCH_JOINT_RAD,
    MIN_HEAD_PITCH_JOINT_RAD,
    HeadLookTool,
)


def print_state(result) -> None:
    if not result.succeeded:
        print(result.to_dict(), flush=True)
        return
    data = result.data
    print(
        "head_joint2={:.4f} rad  pitch={:.2f}°  range=[{:.2f}, {:.2f}] rad".format(
            float(data["head_joint2_rad"]),
            math.degrees(float(data["head_rpy_rad"][1])),
            MIN_HEAD_PITCH_JOINT_RAD,
            MAX_HEAD_PITCH_JOINT_RAD,
        ),
        flush=True,
    )


def format_step_success(result) -> str:
    return "OK {}={:.4f} rad  pitch_delta={:+.3f}°  other_head_drift={:.5f} rad".format(
        HEAD_JOINT_NAME,
        float(result.data["actual_joint_rad"]),
        math.degrees(float(result.data["head_pitch_delta_rad"])),
        float(result.data["head_joint1_drift_rad"]),
    )


def main() -> int:
    if not sys.stdin.isatty():
        print("拒绝运行：必须使用交互终端。", file=sys.stderr)
        return 2

    print("G1 头部键盘控制：只控制 %s（head_joint1 不下发目标）。" % HEAD_JOINT_NAME)
    print("姿态校验 frame：%s" % HEAD_MEASUREMENT_FRAME)
    print("j=低头0.01  k=抬头0.01  J=低头0.02  K=抬头0.02  r=回起始  q=退出")
    if input("确认头部周围无遮挡并开始控制（y）：").strip().lower() != "y":
        print("已取消，未初始化 SDK。")
        return 0

    tool = HeadLookTool(real_authorized=True)
    initial = tool.get_head_pose()
    print_state(initial)
    if not initial.succeeded:
        return 1
    home = float(initial.data["head_joint2_rad"])
    current = home

    while True:
        command = input("head> ").strip()
        if not command:
            continue
        if command == "q":
            break
        if command == "r":
            if abs(current - home) <= 1e-4:
                print("已在起始角度。")
                continue
            delta = home - current
            step = min(abs(delta), 0.02)
            result = (
                tool.lower_head_step(step, operator_confirmed=True)
                if delta > 0
                else tool.raise_head_step(step, operator_confirmed=True)
            )
        elif command in {"j", "J", "k", "K"}:
            step = 0.01 if command in {"j", "k"} else 0.02
            result = (
                tool.lower_head_step(step, operator_confirmed=True)
                if command in {"j", "J"}
                else tool.raise_head_step(step, operator_confirmed=True)
            )
        else:
            print("未知按键；使用 j/k/J/K/r/q。")
            continue

        if not result.succeeded:
            print(result.to_dict(), flush=True)
            print("本步未成功；停止后续动作，请现场确认头部已稳定。")
            return 1
        current = float(result.data["actual_joint_rad"])
        print(format_step_success(result), flush=True)

    if abs(current - home) > 1e-4:
        answer = input("退出前恢复初始头部角度？按 Enter/y 恢复，输入 k 保持：").strip().lower()
        if answer != "k":
            while abs(current - home) > 1e-4:
                delta = home - current
                step = min(abs(delta), 0.02)
                result = (
                    tool.lower_head_step(step, operator_confirmed=True)
                    if delta > 0
                    else tool.raise_head_step(step, operator_confirmed=True)
                )
                if not result.succeeded:
                    print(result.to_dict(), flush=True)
                    print("恢复未明确完成；停止动作。")
                    return 1
                current = float(result.data["actual_joint_rad"])
    print("头部键盘控制结束。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
