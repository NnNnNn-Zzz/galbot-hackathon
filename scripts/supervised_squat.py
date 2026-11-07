#!/usr/bin/env python3
"""Interactive, on-site G1 squat demonstration up to the SDK-supported limit."""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper.dev.robot_tools.squat import (
    MAX_STEP_M,
    MIN_LEG_HEIGHT_M,
    MAX_LEG_HEIGHT_M,
    SquatTool,
)


def show(result) -> None:
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))


def main() -> int:
    if not sys.stdin.isatty():
        print("拒绝运行：此动作必须由现场监督者在交互终端逐步确认。", file=sys.stderr)
        return 2

    print("G1 直立下蹲演示：调用官方 GalbotRobot.set_leg_height()。")
    print(
        f"SDK 支持高度 {MIN_LEG_HEIGHT_M:.2f}–{MAX_LEG_HEIGHT_M:.2f} m；"
        f"每次最多移动 {MAX_STEP_M:.2f} m。"
    )
    print("此范围下限是 SDK 支持边界，不代表已验证的机械极限或安全平衡极限。")
    print("确认机器人站在平坦空旷地面、有人现场观察且急停按钮可立即触及时，")
    if input("请输入“我在机器人旁监督”以启用真机控制：").strip() != "我在机器人旁监督":
        print("已取消；未初始化 SDK 或下发动作。")
        return 0

    tool = SquatTool(real_authorized=True)
    initial = tool.get_leg_height()
    show(initial)
    if not initial.succeeded:
        return 1
    initial_height = float(initial.data["height_m"])
    current_height = initial_height
    print(f"起始高度 {initial_height:.3f} m。保持当前高度不动，直到你确认每一步。")

    while current_height > MIN_LEG_HEIGHT_M + 1e-6:
        choice = input(
            f"当前 {current_height:.3f} m；输入 d 下蹲最多 {MAX_STEP_M:.2f} m，"
            "u 上升一步，r 恢复起始高度，q 停止并进入恢复选择："
        ).strip().lower()
        if choice in {"r", "q"}:
            break
        if choice not in {"d", "u"}:
            print("没有发送动作；请选择 d/u/r/q。")
            continue
        if choice == "u" and current_height >= initial_height - 1e-6:
            print("已经达到起始高度；如需向下测试请选 d，或输入 r/q。")
            continue
        direction = "下蹲" if choice == "d" else "上升"
        confirm = input(
            f"即将{direction}一步。现场观察四周及机器人平衡正常后输入 y 确认；其他输入取消："
        ).strip().lower()
        if confirm != "y":
            print("已取消本步；没有下发动作。")
            continue
        result = (
            tool.lower_leg_height_step(
                min(MAX_STEP_M, current_height - MIN_LEG_HEIGHT_M),
                operator_confirmed=True,
            )
            if choice == "d"
            else tool.raise_leg_height_step(
                min(MAX_STEP_M, initial_height - current_height),
                operator_confirmed=True,
            )
        )
        show(result)
        if not result.succeeded:
            print("本步没有得到明确成功状态；停止自动流程，请现场确认机器人已稳定。")
            return 1
        current_height = float(result.data["actual_height_m"])
        if current_height <= MIN_LEG_HEIGHT_M + 1e-6:
            print("已到达 SDK 支持下限；这不是对机械极限的认证。")

    if abs(current_height - initial_height) <= 0.01:
        restore = True
    else:
        restore = input(
            f"当前 {current_height:.3f} m，起始 {initial_height:.3f} m。"
            "按 Enter 或输入 y 逐步恢复；只有输入 k 才保持当前高度并结束："
        ).strip().lower() != "k"

    if not restore:
        print("保留当前高度；请保持现场观察。未切换腿部控制器。")
        return 0

    while current_height < initial_height - 0.005:
        confirm = input(
            f"即将上升最多 {MAX_STEP_M:.2f} m（当前 {current_height:.3f} m）。"
            "确认现场状态正常后输入 y："
        ).strip().lower()
        if confirm != "y":
            print("恢复暂停；机器人保持在当前 SDK 目标高度。")
            return 1
        result = tool.raise_leg_height_step(
            min(MAX_STEP_M, initial_height - current_height), operator_confirmed=True
        )
        show(result)
        if not result.succeeded:
            print("恢复动作状态不明确；停止后续动作并检查机器人。")
            return 1
        current_height = float(result.data["actual_height_m"])

    if input("已回到起始高度附近。按官方示例切回 leg_pvt_ctrl？输入 y：").strip().lower() == "y":
        show(tool.restore_leg_position_controller(operator_confirmed=True))
    print("下蹲监督流程结束；本脚本不会关闭或销毁共享 SDK 单例。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
