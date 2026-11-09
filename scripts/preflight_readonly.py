#!/usr/bin/env python3
"""交接后第一步：**不发任何运动的**只读预检（约 20 秒）。

为什么需要它
------------
交接现场最怕两件事：① 站起来发现环境没 source、SDK 导入不了；
② 机器人当前姿态是上一个操作者留下的未知姿态，直接开抓风险高。

本脚本把 ① 的检查全部做掉，并把 ② 的姿态用「只读」方式记下来，
作为实验 1 的对照基准。**全程不调用 set_arm_pose / move_line_to /
set_joints / set_gripper，不动底盘，不动夹爪。**

它检查什么
----------
  1. SDK 可导入（galbot_sdk 能否 import）+ Python/机器架构
  2. robot.init() 能否成功，关节名能否读出（读不到 = DDS 没通）
  3. 当前 21 关节角 —— 与 tuning.UPPER_JOINT_POSITIONS 逐关节对比，
     算出每关节差多少，用来判断「当前是不是已经在初始位附近」
  4. 当前右末端位姿 —— 记录抓手在哪里，实验后对比
  5. 配置/判据自检 —— 确认脚本跑的是 8 步新流程而不是旧版

用法（HPU 上）
--------------
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/preflight_readonly.py

    # 把结果同时落盘，便于交接后回看
    python3 scripts/preflight_readonly.py 2>&1 | tee handover_preflight.log

退出码：
    0 = 只读预检全部通过，可以立刻进实验 1
    1 = 有阻塞项（见输出里的 blockers），**先不要动臂**
    2 = mock 模式（用来验证本脚本自身逻辑，不接真机）
"""

from __future__ import annotations

import json
import math
import os
import platform
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def _deg(rad: float) -> float:
    return round(math.degrees(rad), 1)


def _check_pipeline() -> dict:
    """确认代码是「8 步 + init_pose」的新流程，不是旧版。"""
    from scenes.scene_medicine_check.experiments import criteria
    from scenes.scene_medicine_check.pipeline import runner as R

    cfg = R.load_config("medicine.baseline")
    ids = [s["id"] for s in cfg["steps"]]
    want = criteria.GRASP_ONLY_STEPS
    blockers = []
    if "init_pose" not in ids:
        blockers.append("配置里没有 init_pose —— 部署的可能是旧代码，抓取会从遗留姿态开始")
    elif "grasp" not in ids:
        blockers.append("配置里没有 grasp —— 配置文件不完整")
    elif ids.index("init_pose") > ids.index("grasp"):
        blockers.append("init_pose 排在 grasp 之后 —— 配置顺序被改错了")
    if want != ["preflight", "init_pose", "grasp"]:
        blockers.append(f"实验 1/2 判据步骤异常：{want}")
    if len(ids) != 8:
        blockers.append(f"全流程步数为 {len(ids)}，期望 8 步")
    return {
        "step": "pipeline_config",
        "version": cfg.get("version"),
        "all_steps": ids,
        "grasp_only_steps": want,
        "blockers": blockers,
        "ok": not blockers,
    }


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper").strip().lower()

    _say({"step": "boot", "script": "preflight_readonly",
          "backend": backend,
          "note": "本脚本只读，不会发出任何运动指令"})

    blockers = []

    # ---------------------------------------------------- 1. 静态信息
    _say({"step": "env",
          "python": sys.version.split()[0],
          "machine": platform.machine(),
          "host": platform.node()})
    if backend == "mock":
        _say({"step": "note", "message": "mock 模式：只验证脚本逻辑，不做真机判断"})

    try:
        import galbot_sdk  # noqa: F401
        sdk_ok = True
        sdk_path = getattr(galbot_sdk, "__file__", "?")
    except Exception as exc:  # noqa: BLE001
        sdk_ok = False
        sdk_path = ""
        if backend == "mock":
            # mock 只验证脚本自身逻辑，没有 SDK 是正常的，不算阻塞
            sdk_path = f"（mock 模式跳过 SDK 检查：{exc}）"
        else:
            blockers.append(
                f"galbot_sdk 导入失败：{exc} —— 是不是忘了 source scripts/env_hpu.sh")
    _say({"step": "sdk_import", "ok": sdk_ok or backend == "mock", "path": sdk_path})

    # ---------------------------------------------------- 2. 配置自检
    cfg_check = _check_pipeline()
    _say(cfg_check)
    blockers.extend(cfg_check["blockers"])

    # 流程自检不过就没必要碰机器人
    if blockers:
        _say({"step": "abort", "blockers": blockers,
              "action": "先解决上面这几项，不要动臂。部署问题: rsync 重推后再跑本脚本"})
        return 1

    # ---------------------------------------------------- 3. 只读连机器人
    from scenes.scene_medicine_check.pipeline import tuning

    if backend == "mock":
        _say({"step": "skip_zhenji", "ok": True,
              "note": "mock 模式：跳过关节/末端读取（本机无 SDK）。"
                      "脚本逻辑自检已通过（配置 8 步 + init_pose 在位）。"})
        _say({"step": "verdict", "ok": True,
              "next": "脚本可用。真机上跑同一条命令会做完整只读检查。"})
        return 2

    try:
        from sdk_wrapper import wrapper
        robot = wrapper.get_robot()      # 内部会 init，但**不发运动**
        joints = robot.get_joint_names() or []
    except Exception as exc:  # noqa: BLE001
        _say({"step": "connect", "ok": False, "error": str(exc)[:300],
              "action": "robot.init() 失败。真机首次 init 可能等 100s+，这是服务发现超时；"
                        "若确实失败，先查 DDS/网线，不要盲试动作"})
        return 1

    if not joints:
        _say({"step": "abort",
              "blockers": ["get_joint_names() 返回空 —— DDS 没有真正连上，读数不可信"],
              "action": "不要发任何动作。先排查 DDS / 网络 / robot_server 是否在跑"})
        return 1

    # 按关节名逐组读取。绝不能拿 get_joint_positions() 的空参数返回按下标配对：
    # SDK 明确写着空参数时顺序是 chassis/head/left_arm/right_arm/leg，
    # 且实测长度与 name 列表不同（25 vs 23），硬配会得到错位的假角度。
    # 2026-11-07 真机踩过这个坑：错位后「腿全 0、手臂差 180°」全是伪影。
    by_name = {}
    for group_names in (tuning.UPPER_JOINT_NAMES, tuning.LEG_JOINT_NAMES):
        want = [n for n in group_names if n in joints]
        if not want:
            continue
        try:
            vals = robot.get_joint_positions([], list(want))
        except Exception as exc:  # noqa: BLE001
            _say({"step": "read_error", "group": want[0], "error": str(exc)[:160]})
            continue
        for n, v in zip(want, vals):
            by_name[n] = float(v)

    _say({"step": "connect", "ok": True,
          "joint_count": len(joints), "read_by_name": len(by_name),
          "how_read": "按关节名逐组读取，不按下标配对"})

    if len(by_name) < 19:
        _say({"step": "abort",
              "blockers": [f"按名只读到 {len(by_name)} 个关节（应有 21）"],
              "action": "不要发任何动作。先排查 DDS / robot_server"})
        return 1

    # ---------------------------------------------------- 4. 关节对比
    rows = []
    max_dev = 0.0
    max_name = ""
    for name, target in zip(tuning.UPPER_JOINT_NAMES, tuning.UPPER_JOINT_POSITIONS):
        if name not in by_name:
            continue
        cur = by_name[name]
        dev = cur - target
        rows.append({"joint": name, "current_rad": round(cur, 4),
                     "init_rad": round(target, 4),
                     "dev_deg": _deg(dev)})
        if abs(dev) > abs(max_dev):
            max_dev, max_name = dev, name

    for name, target in zip(tuning.LEG_JOINT_NAMES, tuning.LEG_JOINT_POSITIONS):
        if name not in by_name:
            continue
        cur = by_name[name]
        rows.append({"joint": name, "current_rad": round(cur, 4),
                     "init_rad": round(target, 4), "dev_deg": _deg(cur - target)})

    _say({"step": "joint_compare",
          "how_to_read": ("dev_deg 是「当前 - 初始位」的角度差。"
                          "全部在 ±5° 内 = 已经在初始位附近；"
                          "某个关节差很多 = 机器人正停在别的姿态（正常，init_pose 会纠正）"),
          "max_dev_joint": max_name, "max_dev_deg": _deg(max_dev),
          "near_init_pose": abs(_deg(max_dev)) <= 5.0,
          "rows": rows})

    # ---------------------------------------------------- 5. 末端位姿
    try:
        ee = wrapper.get_ee_pose("right_arm")
        _say({"step": "ee_pose", "success": bool(ee.get("success")),
              "pose": ee.get("pose"), "frame": ee.get("frame"),
              "message": str(ee.get("message", ""))[:120],
              "note": "记下来，实验后再读一次：两次差值就是这条轨迹让末端走了多远"})
    except Exception as exc:  # noqa: BLE001
        _say({"step": "ee_pose", "success": False, "error": str(exc)[:200],
              "note": "读不到末端位姿不阻塞实验 1，但会少一个对照数据点"})

    _say({"step": "verdict", "ok": True,
          "next": "预检通过。交接后可以直接跑 scripts/run_handover_exp1.sh"})
    if backend == "mock":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
