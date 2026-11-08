#!/usr/bin/env python3
"""只读探针：验证抽屉场景依赖的三个读接口是否真的可用。

**一条运动指令都不发**（只调 get_* 系列），可以放心先跑。

为什么必须先跑这个：
  抽屉场景的示教采点、下探起点、夹取判定全都依赖读接口。
  而这三个接口（尤其 get_joint_names 的参数签名、get_gripper_state 的字段名）
  我是照官方 day1/05 示例抄的，**没在真机上验过**。
  如果签名不对，示教采点会静默拿到空数据 —— 比报错更危险，所以先验。

用法（HPU 上）：
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/probe_readonly_apis.py
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

RESULTS = []


def check(name: str, fn, *args, **kwargs) -> bool:
    """跑一项检查，异常也算失败（不中断后续检查）。"""
    try:
        ret = fn(*args, **kwargs)
        ok = bool(isinstance(ret, dict) and ret.get("success"))
        RESULTS.append({"item": name, "ok": ok,
                        "message": (ret or {}).get("message") if isinstance(ret, dict) else str(ret),
                        "data": ret})
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        if isinstance(ret, dict):
            for k, v in ret.items():
                if k in ("success", "message"):
                    continue
                print(f"        {k} = {v}")
        return ok
    except Exception as exc:  # noqa: BLE001 - 探针要尽力跑完所有项
        RESULTS.append({"item": name, "ok": False, "message": f"抛异常：{exc}"})
        print(f"[FAIL] {name} —— 抛异常：{exc}")
        return False


def main() -> int:
    from sdk_wrapper import wrapper

    print("=" * 72)
    print("抽屉场景只读探针（全程不发运动指令）")
    print("=" * 72)

    # ---- 0. 先挂工具？不挂也能读位姿，但 get_ee_pose 需要 chain 名 ----
    print("\n--- 1) 末端位姿 get_ee_pose ---")
    check("get_ee_pose(right_arm)", wrapper.get_ee_pose, "right_arm")

    print("\n--- 2) 关节角 get_joint_positions（最关键，签名可能不对）---")
    ok_joints = check("get_joint_positions(joint_groups=['right_arm'], joint_names=[])",
                      wrapper.get_joint_positions,
                      joint_groups=["right_arm"], joint_names=[])

    print("\n--- 3) 夹爪宽度 get_gripper_width ---")
    check("get_gripper_width('right')", wrapper.get_gripper_width, "right")

    print("\n--- 4) 顺带验一下夹爪状态原始返回（找 width 字段名）---")
    try:
        import galbot_sdk.g1 as gm
        robot = wrapper._get_robot()
        gname = getattr(gm.G1JointGroup, "right_gripper", None)
        print(f"        right_gripper 名 = {gname!r}")
        if isinstance(gname, str):
            st = robot.get_gripper_state(gname)
            print(f"        get_gripper_state 返回类型 = {type(st).__name__}")
            print(f"        可用属性 = {[a for a in dir(st) if not a.startswith('_')]}")
            print(f"        width = {getattr(st, 'width', '<无此字段>')}")
    except Exception as exc:  # noqa: BLE001
        print(f"[WARN] 原始夹爪状态探测失败：{exc}")

    print("\n" + "=" * 72)
    n_ok = sum(1 for r in RESULTS if r["ok"])
    print(f"合计 {len(RESULTS)} 项，通过 {n_ok} 项，失败 {len(RESULTS) - n_ok} 项")
    if not ok_joints:
        print("\n⚠️ get_joint_positions 不可用 → 示教采点会拿不到关节角")
        print("   需要按实际 SDK 改 sdk_wrapper/wrapper.py 里 get_joint_positions 的调用方式")
    print("=" * 72)

    out = os.path.join(os.path.dirname(__file__), "..", "logs", "probe_readonly.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(RESULTS, fh, ensure_ascii=False, indent=2, default=str)
    print(f"结果已存：{os.path.abspath(out)}")

    return 0 if n_ok == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
