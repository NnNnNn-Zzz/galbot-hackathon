#!/usr/bin/env python3
"""药品场景硬件冒烟（实验 0）：夹爪开合 + 位姿读回 + 播报，最小可见反馈。

目的：在跑任何完整流程之前，先用 5 个最小动作证明
「封装层 → SDK → 真机」链路是通的，且每步结果落日志供复核。

步骤（每步打印一行 JSON，方便脚本/人读取确认）：
    S1 attach_tool   挂载 galbot_gripper 到 right_arm
    S2 gripper_open  夹爪张开（可见反馈①）
    S3 gripper_close 夹爪闭合（可见反馈②）
    S4 ee_pose       读右末端位姿（顺手产出示教第一个数据点）
    S5 speak         播报一句（PCM 缺失时降级 played=False，不算失败）

用法：
    # mock 干跑（无真机，任何时候安全）
    MED_IO=mock python3 scripts/smoke_medicine_hw.py

    # 真机（PC 侧连机器人，先 source SDK 环境）
    MED_IO=wrapper python3 scripts/smoke_medicine_hw.py

退出码：S1-S4 任一失败为 1；S5 播报降级不阻塞（输出 played=False 供日志复核）。
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline.io import build_io  # noqa: E402

CHAIN = "right_arm"
TOOL = "galbot_gripper"
SIDE = "right"
GRIPPER_OPEN_M = 0.08
GRIPPER_CLOSE_M = 0.075  # 与 tuning 一致；按 7.5 cm 药盒宽度闭合，避免挤扁盒体
SPEAK_TEXT = "夹爪测试完成"


def emit(step: str, ret: dict) -> bool:
    ok = bool(ret.get("success"))
    print(json.dumps({"step": step, "success": ok,
                      "status": ret.get("status"),
                      "completion": ret.get("completion"),
                      "message": ret.get("message", "")},
                     ensure_ascii=False), flush=True)
    return ok


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper")
    print(json.dumps({"step": "boot", "backend": backend}, ensure_ascii=False), flush=True)
    if backend == "mock":
        # 场景 IO 走 MockIO，但 S4 直接调 wrapper 层，两边 mock 开关要同步
        from sdk_wrapper import wrapper
        wrapper.set_mock_mode(True)
    io = build_io(backend)
    failures = 0

    # S1 挂载夹爪（后续一切末端控制的前提）
    if not emit("S1_attach_tool", io.attach_tool(CHAIN, TOOL)):
        failures += 1
        print(json.dumps({"step": "abort", "reason": "attach 失败，后续无意义"},
                         ensure_ascii=False), flush=True)
        return 1

    # S2/S3 夹爪开合（肉眼可见反馈）
    if not emit("S2_gripper_open", io.set_gripper(GRIPPER_OPEN_M, SIDE)):
        failures += 1
    time.sleep(2.0)
    if not emit("S3_gripper_close", io.set_gripper(GRIPPER_CLOSE_M, SIDE)):
        failures += 1

    # S4 位姿读回（示教数据点：把这条 pose 抄进 tuning.py）
    from sdk_wrapper import wrapper
    pose_ret = wrapper.get_ee_pose(CHAIN)
    if not emit("S4_ee_pose", pose_ret):
        failures += 1
    else:
        print(json.dumps({"step": "S4_pose_value", "pose": pose_ret.get("pose"),
                          "frame": pose_ret.get("frame")},
                         ensure_ascii=False), flush=True)

    # S5 播报（降级不阻塞：played=False 仅说明 PCM 素材缺失）
    speak_ret = io.speak(SPEAK_TEXT)
    emit("S5_speak", speak_ret)
    if not speak_ret.get("success"):
        print(json.dumps({"step": "S5_note",
                          "note": "播报降级：仅日志可见，现场无声音。"
                                  "用 scripts/make_pcm 预生成 PCM 后重跑"},
                         ensure_ascii=False), flush=True)

    print(json.dumps({"step": "done", "failures": failures}, ensure_ascii=False), flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
