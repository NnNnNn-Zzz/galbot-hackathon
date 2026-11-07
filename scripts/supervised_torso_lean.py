#!/usr/bin/env python3
"""Interactive, on-site single-joint G1 upper-body lean test."""

from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper.dev.robot_tools.torso_lean import (
    MAX_BACKWARD_JOINT_RAD,
    MAX_FORWARD_JOINT_RAD,
    MAX_STEP_RAD,
    SINGLE_JOINT_PITCH_SUPPORTED,
    TARGET_JOINT_NAME,
    TorsoLeanTool,
)


def show(result):
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2), flush=True)


def main() -> int:
    if not sys.stdin.isatty():
        print("拒绝运行：此动作必须由现场监督者在交互终端逐步确认。", file=sys.stderr)
        return 2

    if not SINGLE_JOINT_PITCH_SUPPORTED:
        print("安全禁用：当前 G1 没有可用的单关节前倾自由度。", file=sys.stderr)
        print(
            "真机已验证 leg_joint4 为偏航，leg_joint5 也造成侧向滚转；"
            "没有可直接用于前倾的单个躯干关节。",
            file=sys.stderr,
        )
        print(
            "本脚本不会初始化 SDK 或发送动作。前倾需要重新设计为多关节协同轨迹。",
            file=sys.stderr,
        )
        return 2

    print("G1 单关节前倾演示：只命令 %s（已排除会偏航的 leg_joint4）。" % TARGET_JOINT_NAME)
    print(
        "单次最多 %.3f rad（约 %.2f°）；软件测试范围 %.2f–%.2f rad。"
        % (MAX_STEP_RAD, math.degrees(MAX_STEP_RAD), MAX_BACKWARD_JOINT_RAD, MAX_FORWARD_JOINT_RAD)
    )
    print("该范围不是机械极限或平衡极限；正方向会同时验证头部 X 位移与俯仰，并拒绝偏航/滚转主导。")
    print("确认脚下平坦、周围空旷、机器人旁有人且急停可立即触及时，")
    if input("请输入“我只动一个关节并现场监督”：").strip() != "我只动一个关节并现场监督":
        print("已取消；未初始化 SDK 或下发动作。")
        return 0

    tool = TorsoLeanTool(real_authorized=True)
    initial = tool.get_torso_lean_state()
    show(initial)
    if not initial.succeeded:
        return 1
    initial_joint = float(initial.data["joint_rad"])
    current_joint = initial_joint
    print(
        "起始 %s=%.6f rad。保持不动，直到你逐步输入 f/b 并确认。"
        % (TARGET_JOINT_NAME, current_joint)
    )

    while True:
        choice = input(
            "当前 %.6f rad；f 前倾一步，b 后仰/回正一步，r 恢复起始，q 停止并选择恢复："
            % current_joint
        ).strip().lower()
        if choice in {"r", "q"}:
            break
        if choice not in {"f", "b"}:
            print("没有发送动作；请选择 f/b/r/q。")
            continue
        if choice == "f" and current_joint >= MAX_FORWARD_JOINT_RAD - 1e-6:
            print("已达到软件前倾边界；请选择 b 回正或 q 停止。")
            continue
        if choice == "b" and current_joint <= MAX_BACKWARD_JOINT_RAD + 1e-6:
            print("已达到软件后仰边界；请选择 f 或 q 停止。")
            continue
        confirm = input(
            "即将移动 %s 一步。观察机器人重心与周围后输入 y 确认；其他输入取消："
            % TARGET_JOINT_NAME
        ).strip().lower()
        if confirm != "y":
            print("已取消本步；没有下发动作。")
            continue
        if choice == "f":
            step = min(MAX_STEP_RAD, MAX_FORWARD_JOINT_RAD - current_joint)
            result = tool.lean_forward_step(step, operator_confirmed=True)
        else:
            step = min(MAX_STEP_RAD, current_joint - MAX_BACKWARD_JOINT_RAD)
            result = tool.lean_backward_step(step, operator_confirmed=True)
        show(result)
        if not result.succeeded:
            print("本步没有得到明确成功状态；停止后续动作，请先现场确认机器人稳定。")
            return 1
        current_joint = float(result.data["actual_joint_rad"])

    if abs(current_joint - initial_joint) <= 1e-4:
        restore = True
    else:
        restore = input(
            "当前 %.6f rad，起始 %.6f rad。按 Enter/y 逐步恢复；输入 k 保留当前姿态："
            % (current_joint, initial_joint)
        ).strip().lower() != "k"

    if not restore:
        print("保留当前躯干姿态；请继续现场观察。")
        return 0

    while abs(current_joint - initial_joint) > 1e-4:
        confirm = input(
            "即将向起始角度移动一步（当前 %.6f rad）。现场确认后输入 y："
            % current_joint
        ).strip().lower()
        if confirm != "y":
            print("恢复暂停；机器人保持在当前角度。")
            return 1
        delta = initial_joint - current_joint
        step = min(MAX_STEP_RAD, abs(delta))
        if delta > 0:
            result = tool.lean_forward_step(step, operator_confirmed=True)
        else:
            result = tool.lean_backward_step(step, operator_confirmed=True)
        show(result)
        if not result.succeeded:
            print("恢复动作状态不明确；停止后续动作并检查机器人。")
            return 1
        current_joint = float(result.data["actual_joint_rad"])

    print("已回到起始单关节角度附近。")
    show(tool.restore_leg_position_controller(operator_confirmed=True))
    print("单关节躯干监督流程结束；本脚本不会关闭或销毁共享 SDK 单例。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
