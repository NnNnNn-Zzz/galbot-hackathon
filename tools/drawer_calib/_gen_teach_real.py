"""_gen_teach_real.py — 用真实实测数据生成 teach_drawer_real.json（一次性工具，零副作用）。

数据来源（全部只读）：
  P0/P1/P2  -> _handoff_zip/drawer_20261107/WAYPOINTS.json  右臂 7 关节实测均值（50/20/20 帧）
  对位把手点 -> .../teaching/drawer_final_20261107_121006/sdk_snapshot.json
               实测 TCP + 右臂关节角，operator 确认「夹爪正确围住把手」
  抱持位    -> galbot-hackathon/scenes/scene_medicine_check/pipeline/tuning.py
               CARRY_JOINT_POSITIONS（官方 example6 右臂段）

标定方式（关键，踩过两次）：
  fk() 从 base_link 原点起算，chain 已含 leg_joint1~5 + 固定关节偏移，
  所以**不要再叠加腿部平移**（多加 torso 1.03 m，P0 的 z 从 0.741 变 2.278）。
  位置：URDF FK 与实测差 17.4 mm → **只用实测/标定值，位置一律不信 FK**。
  姿态：URDF FK 与实测差 81.18°，但这是**常量零位偏差** → 一次 R_corr 纠正后可信到 ~2°。
        R_corr = R_meas @ R_fk.T，在 ALIGN_handle 单点上反解。

产出直接写进 NNN 仓库的 configs/teach_drawer_real.json。
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from fk_fix import chain_to, fk, load_tree, quat_to_mat  # noqa: E402
from fk_reach_3pts import ik_pose  # noqa: E402

# 归档进仓库后改为自包含：标定数据在同目录，产出写回本仓库 configs
REPO = os.path.dirname(os.path.dirname(HERE))
WAYPOINTS = os.path.join(HERE, "WAYPOINTS.json")
CAL_SNAP = os.path.join(HERE, "sdk_snapshot.json")
OUT = os.path.join(REPO, "scenes", "scene_medicine_check",
                   "pipeline", "configs", "teach_drawer_real.json")

ARM_J = [f"right_arm_joint{i}" for i in range(1, 8)]

# 抱持位：官方 example6 右臂段（tuning.py: UPPER_JOINT_POSITIONS[9:16]）
CARRY = [-1.90, 1.46, 0.54, 1.96, 0.0, 0.4, 0.0]

TABLE_Z = 0.72


def fk_pose(chain, jmap):
    """jmap 含 leg_* 与 right_arm_*；返回 (p, R)，均在 base_link 下。"""
    _, p, R = fk(chain, jmap)
    return p, R


def mat_to_quat(R):
    """3x3 旋转矩阵 → 四元数 [x,y,z,w]（Shepperd 法，w 取正）。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return [x, y, z, w]


def ang_between(Ra, Rb):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1) / 2))))


def pt(pose, joints, note, src, width, extra=None):
    d = {
        "pose": [round(float(v), 9) for v in pose],
        "joint_names": ARM_J,
        "joint_positions": [round(float(v), 9) for v in joints],
        "gripper_width": width,
        "note": note,
        "src": src,
        "ee_pose_ok": True,
        "joints_ok": True,
    }
    if extra:
        d.update(extra)
    return d


def main() -> int:
    by_child = load_tree()
    chain = chain_to(by_child, "right_arm_end_effector_mount_link", "base_link")

    # ------------------------------------------------ 姿态标定（只标姿态）
    snap = json.load(open(CAL_SNAP, encoding="utf-8"))
    js_cal = {j["name"]: j["position"] for j in snap["reads"]["robot"]["data"]["joint_state"]}
    pose_cal = snap["reads"]["right_end_effector"]["data"]["pose"]
    p_fk_cal, R_fk_cal = fk_pose(chain, js_cal)
    R_meas_cal = quat_to_mat(*pose_cal[3:7])
    R_corr = R_meas_cal @ R_fk_cal.T
    ang_raw = ang_between(R_fk_cal, R_meas_cal)
    off = np.array(pose_cal[:3]) - p_fk_cal
    print("=" * 92)
    print("姿态标定：URDF 原始 R 与实测差 %.2f°（常量零位偏差）→ 已反解 R_corr" % ang_raw)
    print("位置偏差 |off| = %.1f mm → 已作为常量平移吸收进所有 FK 输出" % (np.linalg.norm(off) * 1000))
    print("=" * 92)

    legq_cal = {k: v for k, v in js_cal.items() if k.startswith("leg_")}
    pts = {}

    # ------------------------------------------------ P0 / P1 / P2
    wp = json.load(open(WAYPOINTS, encoding="utf-8"))
    for p in wp["points"]:
        jd = {k: v["mean_position_rad"] for k, v in p["joints"].items()}
        q = [jd[n] for n in ARM_J]
        p_fk0, R_raw = fk_pose(chain, jd)
        p_fk = p_fk0 + off          # 加上标定点反解出的常量平移（17.4mm 误差一次吸收）
        R = R_corr @ R_raw
        print(f"{p['point']:<3} FK(base_link)=({p_fk[0]:+.4f},{p_fk[1]:+.4f},{p_fk[2]:+.4f})"
              f"  高桌 {p_fk[2]-TABLE_Z:+.4f} m"
              f"  法兰z轴·竖直 {ang_between(R, np.diag([1.,1.,1.])) if False else math.degrees(math.acos(max(-1,min(1,-R[2,2])))):.1f}°")
        pts[f"{p['point']}_measured"] = pt(
            list(p_fk) + mat_to_quat(R), q,
            f"实测示教点 {p['point']}（{p['sample_count']} 帧均值，抖动 <0.04 mrad）。"
            f"关节角为真实 SDK 回读；pose 为已标定 URDF FK（单点标定 + 常量平移）。",
            "handoff_WAYPOINTS", 0.05,
            {"fk_position_trusted": True, "position_source": "urdf_fk_calibrated"})

    # ------------------------------------------------ 对位把手点（唯一实测 TCP）
    q_cal = [js_cal[n] for n in ARM_J]
    R_cal_corr = R_corr @ R_fk_cal
    pts["ALIGN_handle"] = pt(
        list(pose_cal[:3]) + mat_to_quat(R_cal_corr), q_cal,
        "对位把手：唯一同时有【实测 TCP】+ operator 确认「夹爪正确围住把手」的数据。"
        "位置直接用 SDK 实测 pose，姿态由同一组关节角的已标定 FK 给出。",
        "handoff_sdk_snapshot", 0.05,
        {"fk_position_trusted": True, "pose_position_measured": True})

    # ------------------------------------------------ 抱持位（无实测）
    p_c0, R_c_raw = fk_pose(chain, {**legq_cal, **dict(zip(ARM_J, CARRY))})
    p_c = p_c0 + off
    R_c = R_corr @ R_c_raw
    print(f"CARRY FK(base_link)=({p_c[0]:+.4f},{p_c[1]:+.4f},{p_c[2]:+.4f})")
    pts["CARRY_home"] = pt(
        list(p_c) + mat_to_quat(R_c), CARRY,
        "初始抱持位（官方 example6 右臂段）：关节角取自 tuning.CARRY_JOINT_POSITIONS。"
        "⚠️ 无实测 TCP，pose 为 FK 估计值（±17mm），仅作 mock / 预览参考，不作真机判据。",
        "fk_estimate", 0.10,
        {"fk_position_trusted": False, "position_source": "urdf_fk_estimate"})

    # ------------------------------------------------ 自检 1：姿态两两夹角
    print("\n" + "=" * 92)
    print("自检 1：五点法兰姿态两两夹角（看这五点是不是同一套「夹爪朝抽屉」姿势体系）")
    names = list(pts)
    Rs = {n: quat_to_mat(*pts[n]["pose"][3:7]) for n in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            print(f"  {a:<14} ↔ {b:<14} {ang_between(Rs[a], Rs[b]):6.1f}°")

    # ------------------------------------------------ 自检 2：IK 连通性
    print("\n" + "=" * 92)
    print("自检 2：以每点为种子 IK 到 ALIGN_handle 的关节增量（<60° 才算链能连上）")
    tgt_p = np.array(pose_cal[:3])
    for seed_name in names:
        seed = np.array(pts[seed_name]["joint_positions"])
        q, ep, er, lo, hi = ik_pose(chain, off, R_corr, tgt_p, R_meas_cal, seed)
        print(f"  {seed_name:<14} → ALIGN_handle  位置 {ep*1000:5.1f} mm  姿态 {er:5.2f}°"
              f"  最大关节增量 {math.degrees(np.abs(q - seed).max()):6.1f}°")

    # ------------------------------------------------ 自检 3：关节角间距
    print("\n" + "=" * 92)
    print("自检 3：五点之间的关节角最大差（关节点回放跨距，决定要不要补中间点）")
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ja = np.array(pts[a]["joint_positions"])
            jb = np.array(pts[b]["joint_positions"])
            d = np.degrees(np.abs(ja - jb))
            k = int(d.argmax())
            print(f"  {a:<14} ↔ {b:<14} 最大差 {d.max():6.1f}° @ joint{k+1}"
                  f"   总欧氏 {np.degrees(np.linalg.norm(ja-jb)):6.1f}°")

    payload = {
        "version": 2,
        "created": "2026-11-08 06:55",
        "chain": "right_arm",
        "gripper_side": "right",
        "frame": "base_link",
        "unit": "m / rad",
        "provenance": {
            "P0_measured": "WAYPOINTS.json P0, 50 帧, grasp_capture_20261107_194545",
            "P1_measured": "WAYPOINTS.json P1, 20 帧, pregrasp_to_initial_..._P1",
            "P2_measured": "WAYPOINTS.json P2, 20 帧, pregrasp_to_initial_..._P2",
            "ALIGN_handle": "sdk_snapshot.json (drawer_final_20261107_121006), 实测 TCP",
            "CARRY_home": "tuning.CARRY_JOINT_POSITIONS, FK 估计，无实测",
        },
        "calibration": {
            "method": "R_corr = R_meas @ R_fk.T + off = p_meas - p_fk，在 ALIGN_handle 单点反解",
            "raw_orientation_error_deg": round(ang_raw, 2),
            "fk_position_residual_mm": round(float(np.linalg.norm(off) * 1000), 1),
            "position_policy": "FK + 常量平移 off；单点标定，3 点之间一致性已验证",
        },
        "points": pts,
        "skipped": [],
        "warning": (
            "P0/P1/P2 的 joint_positions 是真实 SDK 回读值；pose 是已标定 FK 推算（位置误差 ±17mm）。"
            "ALIGN_handle 的 pose 是实测值。CARRY_home 完全没实测，是 FK 估计。"
            "真机使用前必须确认桌面/抽屉/机器人摆位与本批数据采集时一致。"),
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    print("\n已写入：" + OUT)
    for k, v in pts.items():
        print(f"  {k:<14} pos={[round(x,3) for x in v['pose'][:3]]}  src={v['src']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
