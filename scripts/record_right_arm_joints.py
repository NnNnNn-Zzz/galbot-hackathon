#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读记录一条手臂上全部自由度的当前关节参数。

**不做任何运动指令**：只调用
  get_joint_names / get_joint_states / get_end_effector_pose_on_chain / get_wbc_end_effector_poses

用法（在机器人上）::

    source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 scripts/record_right_arm_joints.py                 # 右臂，默认 9 帧
    python3 scripts/record_right_arm_joints.py --group left_arm
    python3 scripts/record_right_arm_joints.py --samples 1     # 只要一帧快照

输出：
  1. 终端一张表（名称 / 角度 rad / 角度 deg / 速度 / 力矩 / 电流 / 帧间抖动）
  2. JSON 落到 --out-dir（默认 ~/joint_records），文件名带时间戳
  3. 追加一条到同目录的 <group>_sequence.json —— 按采集顺序排队，
     后续拼连续轨迹直接读这个文件（--rebuild-seq 可从目录里已有 JSON 重建）
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
import traceback

DEFAULT_GROUP = "right_arm"
DEFAULT_SAMPLES = 9
DEFAULT_INTERVAL_S = 0.15
EE_FRAME = "EndEffector"
REF_FRAME = "base_link"


# --------------------------------------------------------------------------- #
# SDK 初始化 / 释放（与 dev 层同一套顺序：先 motion.init() 再 robot.init()）
# --------------------------------------------------------------------------- #
def init_sdk():
    import galbot_sdk.g1 as gm

    motion = gm.GalbotMotion()
    if not motion.init():
        raise RuntimeError("GalbotMotion.init() 返回 False")

    robot = gm.GalbotRobot()
    # 只读关节状态不需要任何相机/雷达 → 传空集合减少开销
    if not robot.init(set()):
        raise RuntimeError("GalbotRobot.init() 返回 False（通信总线未建立）")
    return gm, robot, motion


def release(robot) -> None:
    for name in ("request_shutdown", "wait_for_shutdown", "destroy"):
        func = getattr(robot, name, None)
        if callable(func):
            try:
                func()
            except Exception as exc:  # noqa: BLE001 - 退出路径不该二次崩溃
                print("  [warn] %s() 失败: %s" % (name, exc), file=sys.stderr)


# --------------------------------------------------------------------------- #
# 读状态
# --------------------------------------------------------------------------- #
def _f(value, default=float("nan")) -> float:
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        return default


def sample_once(robot, group: str):
    """返回 [(name, pos_rad, vel, effort, current, accel), ...]"""
    states = list(robot.get_joint_states([group], []))
    if not states:
        # 退化路径：用名字列表 + 位置列表（某些固件版本 states 为空）
        names = list(robot.get_joint_names(True, [group]))
        pos = list(robot.get_joint_positions([group], []))
        if not names or len(names) != len(pos):
            raise RuntimeError("get_joint_states 与 get_joint_names/positions 都拿不到 %s 的关节" % group)
        return [(n, _f(p), float("nan"), float("nan"), float("nan"), float("nan"))
                for n, p in zip(names, pos)]

    rows = []
    for st in states:
        name = getattr(st, "joint_name", None)
        if name is None:
            name = getattr(st, "name", "?")
        rows.append((
            str(name),
            _f(getattr(st, "position", float("nan"))),
            _f(getattr(st, "velocity", float("nan"))),
            _f(getattr(st, "effort", float("nan"))),
            _f(getattr(st, "current", float("nan"))),
            _f(getattr(st, "acceleration", float("nan"))),
        ))
    return rows


def read_ee(motion, group: str):
    out = {}
    try:
        status, pose = motion.get_end_effector_pose_on_chain(group, EE_FRAME, REF_FRAME)
        out["chain"] = group
        out["frame"] = EE_FRAME
        out["reference_frame"] = REF_FRAME
        out["status"] = str(status)
        out["pose_xyz_m"] = [_f(v) for v in list(pose)[:3]]
        out["pose_quat_xyzw"] = [_f(v) for v in list(pose)[3:7]]
        out["pose_rpy_deg"] = quat_to_rpy_deg(list(pose)[3:7])
    except Exception as exc:  # noqa: BLE001
        out["error"] = "%s: %s" % (type(exc).__name__, exc)
    return out


def quat_to_rpy_deg(q):
    try:
        qx, qy, qz, qw = (_f(v) for v in q)
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 1e-9:
            return None
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
    except Exception:  # noqa: BLE001
        return None


# --------------------------------------------------------------------------- #
# 汇总
# --------------------------------------------------------------------------- #
def summarise(frames):
    """frames: list[list[row]] → 每个关节一行汇总（中位数 + 帧间极差）。"""
    n = len(frames)
    names = [r[0] for r in frames[0]]
    by_name = {}
    for name in names:
        series = []
        for fr in frames:
            for r in fr:
                if r[0] == name:
                    series.append(r)
                    break
        pos = [r[1] for r in series]
        vel = [r[2] for r in series]
        eff = [r[3] for r in series]
        cur = [r[4] for r in series]

        def stat(vals):
            good = [v for v in vals if math.isfinite(v)]
            if not good:
                return float("nan"), float("nan"), float("nan")
            med = statistics.median(good)
            return med, min(good), max(good)

        p_med, p_min, p_max = stat(pos)
        v_med, _, _ = stat(vel)
        e_med, _, _ = stat(eff)
        c_med, _, _ = stat(cur)
        by_name[name] = {
            "position_rad": p_med,
            "position_deg": math.degrees(p_med) if math.isfinite(p_med) else float("nan"),
            "position_rad_min": p_min,
            "position_rad_max": p_max,
            "position_spread_mrad": (p_max - p_min) * 1000.0 if math.isfinite(p_max - p_min) else float("nan"),
            "velocity_rad_s": v_med,
            "effort_nm": e_med,
            "current_a": c_med,
            "samples": len([v for v in pos if math.isfinite(v)]),
        }
    return names, by_name


def print_table(names, by_name, group, n_frames, ee):
    print("")
    print("═" * 92)
    print("  %s 关节参数（只读快照，%d 帧采样）" % (group, n_frames))
    print("═" * 92)
    header = "  %-22s %12s %12s %11s %11s %10s" % ("joint_name", "rad", "deg", "vel(rad/s)", "effort(Nm)", "I(A)")
    print(header)
    print("  " + "-" * 88)
    for name in names:
        d = by_name[name]
        print("  %-22s %12.5f %12.3f %11.5f %11.4f %10.4f" % (
            name, d["position_rad"], d["position_deg"], d["velocity_rad_s"], d["effort_nm"], d["current_a"],
        ))
    print("  " + "-" * 88)
    spread = [by_name[n]["position_spread_mrad"] for n in names if math.isfinite(by_name[n]["position_spread_mrad"])]
    if spread:
        print("  帧间角度极差：max %.3f mrad（%.4f°）→ %s" % (
            max(spread), max(spread) / 1000.0 * 180.0 / math.pi,
            "静止" if max(spread) < 5.0 else "⚠ 机器人在动，快照不代表稳定位姿"))
    dof = len(names)
    print("  自由度：%d DOF" % dof)
    if ee and "pose_xyz_m" in ee:
        xyz = ee["pose_xyz_m"]
        rpy = ee.get("pose_rpy_deg") or [float("nan")] * 3
        print("  末端位姿(%s→%s)：xyz=(%.4f, %.4f, %.4f) m  rpy=(%.2f, %.2f, %.2f)°" % (
            ee.get("reference_frame"), ee.get("frame"), xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
    elif ee and "error" in ee:
        print("  末端位姿读取失败：%s" % ee["error"])
    print("═" * 92)


# --------------------------------------------------------------------------- #
# 序列归档：把目录里所有单点 JSON 按采集时间排成一条采样序列
# --------------------------------------------------------------------------- #
def _is_point_file(fn):
    return fn.endswith(".json") and "_joints_" in fn and not fn.endswith("_sequence.json")


def rebuild_sequence(out_dir, group, verbose=True):
    """扫描 out_dir 下所有 <group>_joints_*.json，按 recorded_at 排序重写序列文件。"""
    entries = []
    if not os.path.isdir(out_dir):
        return None
    for fn in sorted(os.listdir(out_dir)):
        if not _is_point_file(fn):
            continue
        try:
            with open(os.path.join(out_dir, fn), encoding="utf-8") as fh:
                d = json.load(fh)
        except Exception:  # noqa: BLE001 - 坏文件跳过，不阻塞归档
            continue
        if d.get("group") != group:
            continue
        ee = d.get("end_effector") or {}
        order = d.get("joint_order", [])
        entries.append({
            "recorded_at": d.get("recorded_at", ""),
            "json": fn,
            "note": d.get("note", ""),
            "dof": d.get("dof"),
            "joint_order": order,
            "rad": [d["joints"][n]["position_rad"] for n in order],
            "deg": [d["joints"][n]["position_deg"] for n in order],
            "ee_xyz_m": ee.get("pose_xyz_m"),
            "ee_rpy_deg": ee.get("pose_rpy_deg"),
        })
    entries.sort(key=lambda e: (e["recorded_at"], e["json"]))
    for i, e in enumerate(entries, 1):
        e["label"] = "seq%02d" % i

    seq_path = os.path.join(out_dir, "%s_sequence.json" % group)
    payload = {
        "group": group,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "count": len(entries),
        "joint_order": entries[0]["joint_order"] if entries else [],
        "entries": entries,
    }
    with open(seq_path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)

    if verbose:
        print("")
        print("  采样序列（%d 点）→ %s" % (len(entries), seq_path))
        for e in entries:
            print("    %s  %s  %s  %s" % (
                e["label"], e["recorded_at"][:19], e["json"],
                ("(%s)" % e["note"]) if e["note"] else ""))
    return seq_path


def main() -> int:
    ap = argparse.ArgumentParser(description="只读记录右臂（或指定组）全部自由度的关节参数")
    ap.add_argument("--group", default=DEFAULT_GROUP, help="关节组，默认 right_arm（可选 left_arm）")
    ap.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="采样帧数，默认 9")
    ap.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S, help="帧间隔秒，默认 0.15")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/joint_records"), help="JSON 输出目录")
    ap.add_argument("--no-ee", action="store_true", help="跳过末端位姿读取")
    ap.add_argument("--note", default="", help="给这个采样点加备注，写进序列（如 '举起'）")
    ap.add_argument("--rebuild-seq", action="store_true",
                    help="不连机器人，只按目录里已有 JSON 重建采样序列")
    args = ap.parse_args()

    if args.rebuild_seq:
        p = rebuild_sequence(args.out_dir, args.group)
        print("序列已重建：%s" % p)
        return 0

    if args.samples < 1:
        print("--samples 必须 ≥ 1", file=sys.stderr)
        return 2

    robot = None
    try:
        gm, robot, motion = init_sdk()

        try:
            groups = list(robot.get_joint_group_names())
            print("可用关节组：%s" % (groups or "（空）"))
        except Exception as exc:  # noqa: BLE001
            print("get_joint_group_names 失败：%s" % exc, file=sys.stderr)

        frames = []
        for i in range(args.samples):
            frames.append(sample_once(robot, args.group))
            if i + 1 < args.samples:
                time.sleep(args.interval)

        names, by_name = summarise(frames)
        ee = {} if args.no_ee else read_ee(motion, args.group)
        print_table(names, by_name, args.group, len(frames), ee)

        ts = time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(args.out_dir, exist_ok=True)
        path = os.path.join(args.out_dir, "%s_joints_%s.json" % (args.group, ts))
        payload = {
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "group": args.group,
            "dof": len(names),
            "joint_order": names,
            "joints": dict((n, {
                k: (None if not math.isfinite(v) else round(v, 6))
                for k, v in by_name[n].items()
            }) for n in names),
            "samples": len(frames),
            "interval_s": args.interval,
            "note": args.note,
            "raw_frames": [[
                {"name": r[0],
                 "position_rad": None if not math.isfinite(r[1]) else round(r[1], 6),
                 "velocity_rad_s": None if not math.isfinite(r[2]) else round(r[2], 6),
                 "effort_nm": None if not math.isfinite(r[3]) else round(r[3], 6),
                 "current_a": None if not math.isfinite(r[4]) else round(r[4], 6)}
                for r in fr] for fr in frames],
            "end_effector": ee,
        }
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        print("JSON 已保存：%s" % path)
        seq_path = rebuild_sequence(args.out_dir, args.group)

        # 便于 shell 抓取的一行摘要
        print("SUMMARY %s" % json.dumps({
            "group": args.group,
            "dof": len(names),
            "deg": [round(by_name[n]["position_deg"], 3) for n in names],
            "json": path,
            "sequence": seq_path,
        }, ensure_ascii=False))
        return 0
    except Exception as exc:  # noqa: BLE001
        print("失败：%s: %s" % (type(exc).__name__, exc), file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        if robot is not None:
            release(robot)


if __name__ == "__main__":
    sys.exit(main())
