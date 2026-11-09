#!/usr/bin/env python3
"""交接第一件事：**只读**读取机器人当前位置，一个字都不写、一个动作都不发。

场景
----
队友刚把控制权交过来，**刚才有人拖过手臂**。此时关节角未知，
必须先如实记下"我接手时它停在哪"，才能判断后面 init_pose 到底纠正了多少。

安全承诺
--------
本脚本**只调用读接口**，代码里不存在以下任何调用：
    set_joints / set_arm_pose / move_line_to / set_gripper
    turn / move_forward / move_backward / stop_base / speak /
    attach_tool（涉及服务端状态写入）
不初始化任何控制器、不切换控制模式。

读哪些量
--------
  1. 关节名清单 —— 读不到 = DDS 没通，后面一切读数都不可信
  2. 21 个关节当前角 —— 与 tuning 初始位逐关节比对，给出偏差
  3. 右臂 7 关节单独列出 —— 人工拖动最常改的就是这 7 个
  4. 头部/腿部 —— 拖动时是否顺带被碰
  5. 右末端位姿 —— 抓手在哪，实验后可对比
  6. 底盘里程计 —— 有人拖过手臂一般不动底盘，但要确认

同时把一份人可读的快照写到 logs/handover_pose_<HHMMSS>.json，
方便交接后回看，也方便我据此判断。

用法（HPU 上）
--------------
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/read_current_pose.py

    # 想同时留一份文本日志
    python3 scripts/read_current_pose.py 2>&1 | tee handover_pose.log

退出码：
    0 = 读取成功（数据可信）
    1 = 读取失败或数据不可信（**不要动臂**）
    2 = mock 模式（只验证脚本逻辑）
"""

from __future__ import annotations

import json
import math
import os
import platform
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scenes.scene_medicine_check.pipeline import tuning  # noqa: E402

# 允许被拖动后关节角超出初始位很多，所以偏差阈值只用于「打标」不用于「判错」
NEAR_INIT_DEG = 5.0
FAR_INIT_DEG = 30.0


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def _deg(rad) -> float:
    return round(math.degrees(float(rad)), 1)


def _project_root() -> str:
    """返回项目根目录。优先用本文件真实路径，退化为当前工作目录。

    不用裸 __file__：脚本被 exec/粘贴执行时 __file__ 可能是 "<string>"，
    abspath 之后会算出 /logs 这种根目录路径（演练时真踩到过）。
    """
    this = globals().get("__file__") or ""
    if this and not this.startswith("<"):
        real = os.path.abspath(this)
        # /tmp/read_current_pose.py 这种拷出去执行的情况，dirname/.. 会算成 /，
        # 再拼 logs 就是 /logs（真机踩过 PermissionError）。必须校验确实是项目根。
        for cand in (os.path.dirname(real), os.path.abspath(os.path.join(os.path.dirname(real), ".."))):
            if cand != os.sep and os.path.isdir(os.path.join(cand, "scenes", "scene_medicine_check")):
                return cand
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, "scenes", "scene_medicine_check")):
        return cwd
    return cwd


def _snapshot_path() -> str:
    out = os.path.join(_project_root(), "logs")
    os.makedirs(out, exist_ok=True)
    return os.path.join(out, f"handover_pose_{time.strftime('%H%M%S')}.json")


def _read_joints_by_name(robot, names):
    """按**关节名**逐组读取，返回 {name: rad}。

    为什么不能按下标配对：
        SDK stub 明确写着 get_joint_positions() 空参数时 G1 返回顺序是
        chassis, head, left_arm, right_arm, leg；而 get_joint_names() 的顺序
        未定义。实测两者长度都不一样（names 23 个、positions 25 个），
        硬按下标 zip 会得到一整套错位的假角度。
    2026-11-07 真机踩过：错位后「腿部全 0、手臂差 180°」全是伪影。
    """
    out = {}
    groups = [
        (tuning.LEG_JOINT_NAMES, "legs"),
        (tuning.UPPER_JOINT_NAMES[:2], "head"),
        (tuning.UPPER_JOINT_NAMES[2:9], "left_arm"),
        (tuning.UPPER_JOINT_NAMES[9:16], "right_arm"),
    ]
    for group_names, label in groups:
        names_to_read = [n for n in group_names if n in names]
        if not names_to_read:
            continue
        try:
            vals = robot.get_joint_positions([], list(names_to_read))
        except Exception as exc:  # noqa: BLE001
            _say({"step": "read_error", "group": label,
                  "error": f"{type(exc).__name__}: {exc}"})
            continue
        for n, v in zip(names_to_read, vals):
            out[n] = float(v)
    return out


def main() -> int:
    backend = os.environ.get("MED_IO", "wrapper").strip().lower()
    _say({"step": "boot", "script": "read_current_pose", "backend": backend,
          "readonly": True,
          "note": "只读脚本：不含任何运动/夹爪/底盘调用，不调 attach_tool"})

    # ---------------------------------------------------------- mock 分支
    if backend == "mock":
        _say({"step": "env", "python": sys.version.split()[0],
              "machine": platform.machine(), "host": platform.node()})
        _say({"step": "mock_note",
              "message": "mock 模式：跳过真机读取。真机上同一条命令会读 21 关节 + 末端 + 里程计",
              "target_lists": {
                  "upper_joint_count": len(tuning.UPPER_JOINT_NAMES),
                  "leg_joint_count": len(tuning.LEG_JOINT_NAMES),
                  "right_arm_joints": tuning.CARRY_JOINT_NAMES,
              }})
        _say({"step": "verdict", "ok": True, "snapshot": None})
        return 2

    # ---------------------------------------------------------- 真机只读
    _say({"step": "env", "python": sys.version.split()[0],
          "machine": platform.machine(), "host": platform.node()})

    try:
        from sdk_wrapper import wrapper
        robot = wrapper.get_robot()   # init 只建立通信，不发运动
    except Exception as exc:  # noqa: BLE001
        _say({"step": "connect", "ok": False, "error": str(exc)[:300],
              "action": "robot.init() 失败。首次可能等 100s+（服务发现超时）；"
                        "确实失败先查 DDS/网线，不要发任何动作"})
        return 1

    try:
        name_list = list(robot.get_joint_names() or [])
    except Exception as exc:  # noqa: BLE001
        _say({"step": "read_joints", "ok": False, "error": str(exc)[:300]})
        return 1

    by_name = _read_joints_by_name(robot, name_list)

    _say({"step": "connect", "ok": True,
          "joint_name_count": len(name_list),
          "joints_read_by_name": len(by_name),
          "how_read": "按关节名逐组读取（不按下标配对，避免 SDK 顺序不一致）"})

    if len(by_name) < 19:
        _say({"step": "verdict", "ok": False,
              "reason": f"按名只读到 {len(by_name)} 个关节（至少应有 19），"
                        "DDS 或服务端可能没就绪，读数不可信",
              "action": "不要动臂。先排查 DDS / robot_server"})
        return 1

    # 兼容后续代码：pos 通过名字取，不再按下标
    class _Named:
        def __init__(self, m):
            self._m = m

        def index_of(self, n):
            return self._m.get(n)

    idx = _Named(by_name)

    # ---- 2/3/4 关节对照
    def compare(group_names, target_positions, group_label):
        rows = []
        for name, target in zip(group_names, target_positions):
            cur = idx.index_of(name)
            if cur is None:
                rows.append({"joint": name, "missing": True})
                continue
            rows.append({"joint": name,
                         "current_deg": _deg(cur), "current_rad": round(float(cur), 4),
                         "init_deg": _deg(target),
                         "dev_deg": _deg(cur - target)})
        valid = [r["dev_deg"] for r in rows if "dev_deg" in r]
        max_dev = max((abs(d) for d in valid), default=0.0)
        worst = next((r["joint"] for r in rows
                      if "dev_deg" in r and abs(r["dev_deg"]) == max_dev), "")
        if max_dev <= NEAR_INIT_DEG:
            verdict = "near_init"
        elif max_dev <= FAR_INIT_DEG:
            verdict = "mildly_off"
        else:
            verdict = "far_from_init"
        return {"group": group_label, "max_dev_deg": round(max_dev, 1),
                "worst_joint": worst, "verdict": verdict, "rows": rows}

    upper = compare(tuning.UPPER_JOINT_NAMES, tuning.UPPER_JOINT_POSITIONS, "upper_body")
    legs = compare(tuning.LEG_JOINT_NAMES, tuning.LEG_JOINT_POSITIONS, "legs")
    right_arm = compare(tuning.CARRY_JOINT_NAMES, tuning.CARRY_JOINT_POSITIONS,
                        "right_arm(7)")

    for blk in (upper, legs, right_arm):
        _say({"step": "joints", **{k: blk[k] for k in
                                   ("group", "max_dev_deg", "worst_joint", "verdict")},
              "how_to_read": ("dev_deg = 当前角 - 初始位角。"
                              "near_init(≤5°) 说明几乎在初始位；"
                              "far_from_init(>30°) 说明被明显拖过或停在别处。"
                              "这**不是错误**，只是记录接手时的真实状态。"),
              "rows": blk["rows"]})

    # ---- 5 末端位姿（只读；显式传 chain，不依赖 _attached_chain）
    ee = None
    try:
        for chain in ("right_arm", ""):
            ret = wrapper.get_ee_pose(chain)
            if ret.get("success"):
                ee = {"success": True, "chain": chain or "(default)",
                      "pose": ret.get("pose"), "frame": ret.get("frame"), "unit": "m"}
                break
            ee = {"success": False, "chain": chain or "(default)",
                  "status": ret.get("status"),
                  "message": str(ret.get("message", ""))[:160]}
    except Exception as exc:  # noqa: BLE001
        ee = {"success": False, "error": str(exc)[:200]}
    ee = ee or {"success": False, "message": "未尝试"}
    ee["note"] = "this_is_where_the_gripper_actually_is_right_now"
    _say({"step": "ee_pose", **ee})

    # ---- 6 里程计（只读）
    odom = None
    getter = getattr(robot, "get_odom", None) or getattr(robot, "get_base_pose", None)
    if callable(getter):
        try:
            ret = getter()
            odom = ret if isinstance(ret, dict) else {"raw": str(ret)[:200]}
        except Exception as exc:  # noqa: BLE001
            odom = {"error": str(exc)[:160]}
    else:
        odom = {"message": "robot 对象没有 get_odom/get_base_pose，跳过（不阻塞）"}
    _say({"step": "odom", "readonly": True, "data": odom,
          "note": "有人拖过手臂通常不动底盘；里程计若与实验 1 起点差异很大要警惕"})

    # ---- 快照落盘
    snap = {
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host": platform.node(), "python": sys.version.split()[0],
        "backend": backend,
        "joint_names": name_list,
        "joint_positions_by_name_rad": {k: round(v, 4) for k, v in by_name.items()},
        "upper_body": upper, "legs": legs, "right_arm": right_arm,
        "ee_pose": ee, "odom": odom,
        "tuning_at_capture": {
            "UPPER_JOINT_POSITIONS": tuning.UPPER_JOINT_POSITIONS,
            "LEG_JOINT_POSITIONS": tuning.LEG_JOINT_POSITIONS,
            "GRASP_POSE": tuning.GRASP_POSE,
        },
    }
    path = _snapshot_path()
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(snap, f, ensure_ascii=False, indent=2)
        _say({"step": "snapshot_saved", "path": path})
    except Exception as exc:  # noqa: BLE001
        _say({"step": "snapshot_saved", "path": None, "error": str(exc)[:160]})

    # ---- 汇总判读（只给建议，不下令）
    hints = []
    if right_arm["verdict"] == "far_from_init":
        hints.append("右臂明显偏离初始位（最偏离 "
                     f"{right_arm['worst_joint']} {right_arm['max_dev_deg']}°）——"
                     "与「刚才有人拖过」一致。init_pose 会把它拉回，这一步能真实检验修正效果。")
    if upper["verdict"] == "near_init" and legs["verdict"] == "near_init":
        hints.append("全身基本在初始位附近 —— 那这次拖动幅度不大，"
                     "init_pose 的对照价值会弱一些，记录即可。")
    if not ee.get("success"):
        hints.append("末端位姿没读到（可能未挂工具/服务端状态未就绪）。"
                     "不阻塞只读记录，但实验后就没有末端对照点。")

    _say({"step": "verdict", "ok": True,
          "snapshot": path,
          "right_arm_state": right_arm["verdict"],
          "hints": hints,
          "next": "位置已记录。要开实验时再说「开始」，我会在 init_pose 前再读一次做对照"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
