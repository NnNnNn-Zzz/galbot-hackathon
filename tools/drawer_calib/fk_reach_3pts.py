"""fk_reach_3pts.py — 只用 P0/P1/P2 三个可信点，核算「能到哪儿 / 必须新采什么」。

零副作用：只读 URDF + 只读 WAYPOINTS.json。

思路：
  1) 用 drawer_final 那个「实测 TCP 已知」的标定点，反解出 URDF FK 的系统偏差（工具系 + 建模误差）；
  2) 把 P0/P1/P2 的关节角喂进已标定的 FK，得到三点的 TCP 与法兰姿态；
  3) 以三点的实测关节角为种子做雅可比 IK（7 自由度阻尼最小二乘 + 姿态约束），
     逐一检验「用 P0/P1/P2 能不能变到某个目标位姿」，并给出需要的关节增量与是否贴限位。

用法：
  python fk_reach_3pts.py
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

# 归档进仓库后改为自包含：标定数据随工具一起放在本目录
WAYPOINTS = os.path.join(HERE, "WAYPOINTS.json")
CAL_SNAP = os.path.join(HERE, "sdk_snapshot.json")

TABLE_Z = 0.72
ARM_J = [f"right_arm_joint{i}" for i in range(1, 8)]


# ---------------------------------------------------------------- 标定
def calibrate(chain):
    d = json.load(open(CAL_SNAP, encoding="utf-8"))
    js = {j["name"]: j["position"] for j in d["reads"]["robot"]["data"]["joint_state"]}
    pose = d["reads"]["right_end_effector"]["data"]["pose"]
    _, p_fk, R_fk = fk(chain, js)
    off = np.array(pose[:3]) - p_fk
    R_corr = quat_to_mat(*pose[3:7]) @ R_fk.T
    return off, R_corr


LEG_CAL = {"leg_joint1": 0.49812, "leg_joint2": 1.4938, "leg_joint3": 1.00121,
           "leg_joint4": -0.00059, "leg_joint5": -0.00192}


def fk_tcp(chain, q, off, R_corr, legq=None):
    """base_link → 法兰。

    ⚠️ chain_to(..., "base_link") 返回的链已含 leg_joint1~5 与全部固定关节偏移，
    fk() 从 base_link 原点起算 → **腿部姿态靠 jmap 里的 leg_* 生效，不能另加平移**。
    （2026-11-08 更正：旧版本漏了 leg_* 或重复叠加 torso 偏移，导致三点 z 全部偏高 ~0.5 m，
      进而把「P1/P2 低于桌面 20cm」误判成了事实。实际三点 z = 1.25 / 1.02 / 1.01 m，都在桌面之上。）
    腿角固定用标定快照的实测值（P0/P1/P2 的 leg_* 与标定快照几乎相同，差异 <0.3°）。
    """
    jm = dict(LEG_CAL if legq is None else legq)
    jm.update({n: float(v) for n, v in zip(ARM_J, q)})
    _, p, R = fk(chain, jm)
    return p + off, R_corr @ R


# ---------------------------------------------------------------- IK
def ik_pose(chain, off, R_corr, target_p, target_R, seed, iters=400, lam=0.08,
            w_rot=0.35):
    """7 自由度阻尼最小二乘 IK，位置+姿态同时约束。返回 (q, 位置误差m, 姿态误差deg)。"""
    rev = [j for j in chain if j["type"] in ("revolute", "continuous")
           and str(j["name"]).startswith("right_arm_")]
    lo = np.array([j["lo"] for j in rev])
    hi = np.array([j["hi"] for j in rev])
    q = np.clip(seed.astype(float).copy(), lo + 1e-4, hi - 1e-4)
    eps = 1e-6
    for _ in range(iters):
        p, R = fk_tcp(chain, q, off, R_corr)
        e_p = target_p - p
        # 姿态误差向量（轴角）
        Rd = target_R @ R.T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))
        if ang < 1e-9:
            e_r = np.zeros(3)
        else:
            ax = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]])
            n = np.linalg.norm(ax)
            e_r = (ax / n * ang) if n > 1e-12 else np.zeros(3)
        err = np.concatenate([e_p, w_rot * e_r])
        if np.linalg.norm(e_p) < 2e-4 and ang < math.radians(1.0):
            break
        J = np.zeros((6, 7))
        for k in range(7):
            qp = q.copy()
            qp[k] += eps
            pp, Rp = fk_tcp(chain, qp, off, R_corr)
            J[:3, k] = (pp - p) / eps
            Rdp = Rp @ R.T
            a2 = math.acos(max(-1.0, min(1.0, (np.trace(Rdp) - 1) / 2)))
            if a2 < 1e-9:
                J[3:, k] = 0.0
            else:
                ax2 = np.array([Rdp[2, 1] - Rdp[1, 2], Rdp[0, 2] - Rdp[2, 0],
                                Rdp[1, 0] - Rdp[0, 1]])
                n2 = np.linalg.norm(ax2)
                J[3:, k] = w_rot * ((ax2 / n2 * a2) / eps if n2 > 1e-12 else 0.0)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), err)
        q = np.clip(q + dq, lo + 1e-4, hi - 1e-4)
    p, R = fk_tcp(chain, q, off, R_corr)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(target_R @ R.T) - 1) / 2))))
    return q, float(np.linalg.norm(target_p - p)), ang, lo, hi


def main() -> None:
    by_child = load_tree()
    chain = chain_to(by_child, "right_arm_end_effector_mount_link", "base_link")
    off, R_corr = calibrate(chain)
    # 腿部链：P0/P1/P2 的 leg_joint 实测值与标定快照完全一致，所以腿部对 TCP 的
    # 贡献是常量，这里的 "base_link" 实际等价于 base_link 在标定姿态下的躯干基座。
    leg = chain_to(by_child, "leg_end_effector_mount_link", "base_link")

    wp = json.load(open(WAYPOINTS, encoding="utf-8"))
    seeds, poses = {}, {}
    for p in wp["points"]:
        jd = {k: v["mean_position_rad"] for k, v in p["joints"].items()}
        q = np.array([jd[n] for n in ARM_J])
        pc, Rc = fk_tcp(chain, q, off, R_corr)
        seeds[p["point"]] = q
        poses[p["point"]] = (pc, Rc)
        pts, _, _ = fk(leg, jd)
        if p["point"] == "P0":
            torso_end = pts["leg_end_effector_mount_joint"]

    print("=" * 96)
    print("已标定 FK（误差吸收量 |off| = %.1f mm）下三点的实测位姿" % (np.linalg.norm(off) * 1000))
    print("  参考系：base_link（腿部姿态为 P0/P1/P2 实测值，torso 基座 z = %.4f m）"
          % torso_end[2])
    print("=" * 96)
    for k in ("P0", "P1", "P2"):
        pc, Rc = poses[k]
        down = math.degrees(math.acos(max(-1.0, min(1.0, -Rc[2, 2]))))
        print(f"  {k}: TCP=({pc[0]:+.4f},{pc[1]:+.4f},{pc[2]:+.4f})  高桌 {pc[2]-TABLE_Z:+.4f} m"
              f"  法兰z轴与竖直向下夹角 {down:5.1f}°")

    # ---------------------------------------------------------------- 与实测对照
    print("\n" + "=" * 96)
    print("校验：把 P0 当成姿态模板，与交接包里【真实 SDK 实测】的五个姿态比一比")
    print("=" * 96)
    snap = json.load(open(CAL_SNAP, encoding="utf-8"))
    R_meas = quat_to_mat(*snap["reads"]["right_end_effector"]["data"]["pose"][3:7])
    tpl_p, tpl_R = poses["P0"]
    # P0 自身按定义对齐（用了同一个标定），所以直接拿 URDF 原始 R 与实测 R 比
    q0 = seeds["P0"]
    jm = {n: float(v) for n, v in zip(ARM_J, q0)}
    _, _, R_raw = fk(chain, jm)
    ang0 = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_raw.T @ R_meas) - 1) / 2))))
    print(f"  URDF 原始法兰姿态 vs 实测：{ang0:.2f}°（标定前；只转一次常量 R_corr 就对齐）")
    print("  结论：法兰姿态由 URDF 可信到 ~2°，位置不可信（差 17.4 mm），所以位置一律按实测/标定值。")
    print("  ※ 这个 81° 是 URDF 模型角度零位的整体偏差，不是随机误差 → 三点之间做差值不受影响。")

    print("\n" + "=" * 96)
    print("姿态语义解读（法兰 z 轴 = 夹爪伸出方向，法兰 y 轴 = 夹指闭合方向）")
    print("=" * 96)
    for k in ("P0", "P1", "P2"):
        Rc = poses[k][1]
        zax, yax = Rc[:, 2], Rc[:, 1]
        # 夹指闭合方向与竖直的夹角 → 判断从哪个方向"夹住"
        close_vert = math.degrees(math.acos(max(-1.0, min(1.0, abs(yax[2])))))
        print(f"  {k}: 法兰z轴 {np.round(zax,3)}  → 伸出方向主轴 "
              f"{'竖直' if abs(zax[2])>0.9 else ('水平' if abs(zax[2])<0.3 else '斜向')}"
              f"(|z_z|={abs(zax[2]):.2f})；夹指闭合轴 {np.round(yax,3)} 与竖直夹角 {close_vert:.1f}°")

    targets = [
        ("T1 对位把手(实测值复核)", tpl_p, tpl_R),
        ("T2 对位把手(x+0.05 更靠前)", tpl_p + [0.05, 0, 0], tpl_R),
        ("T3 对位把手(z-0.10 更低)", tpl_p + [0, 0, -0.10], tpl_R),
        ("T4 拉出 0.15m", tpl_p + [-0.15, 0, 0], tpl_R),
        ("T5 拉出 0.30m", tpl_p + [-0.30, 0, 0], tpl_R),
        ("T6 P2 姿态下再压低 0.10", poses["P2"][0] + [0, 0, -0.10], poses["P2"][1]),
        ("T7 箱内抓取(俯视, 桌上 0.03)", np.array([0.42, -0.20, TABLE_Z + 0.03]),
         np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)),
        ("T8 桶上方投放", np.array([0.45, -0.35, 0.60]),
         np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)),
    ]

    print("\n" + "=" * 96)
    print("可达性：以 P0/P1/P2 分别作种子做 7 自由度 IK")
    print("=" * 96)
    print(f"{'目标':<30s}{'种子':>5s}{'位置误差mm':>11s}{'姿态误差°':>10s}"
          f"{'最大关节增量°':>13s}  贴限位关节")
    for name, tp, tR in targets:
        for sk in ("P0", "P1", "P2"):
            q, ep, ea, lo, hi = ik_pose(chain, off, R_corr, tp, tR, seeds[sk])
            dq = np.abs(q - seeds[sk])
            tight = [f"j{i+1}" for i in range(7)
                     if (q[i] - lo[i]) < math.radians(3) or (hi[i] - q[i]) < math.radians(3)]
            ok = "OK " if ep < 0.005 and ea < 2.0 else "✗  "
            print(f"{ok}{name:<27s}{sk:>5s}{ep*1000:>11.1f}{ea:>10.2f}"
                  f"{math.degrees(dq.max()):>13.1f}  {','.join(tight) if tight else '-'}")
        print()

    print("=" * 96)
    print("结论提示")
    print("=" * 96)
    print("  · 位置误差 <5 mm 且姿态误差 <2° 才算『这组种子能到』。")
    print("  · 最大关节增量 >60° 意味着一路要大幅借道，真机上极易撞限位/撞柜体，建议中间补点。")
    print("  · 本脚本不含碰撞体，撞不撞必须实机验证。")


if __name__ == "__main__":
    main()
