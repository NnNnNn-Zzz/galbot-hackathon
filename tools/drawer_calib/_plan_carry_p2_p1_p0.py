"""_plan_carry_p2_p1_p0.py — 按【正确方向】重排实验 1 的上段：CARRY_home → P2 → P1 → P0。

为什么重排（用户 2026-11-08 09:16 澄清 + 真机 09:04 实证）
----------------------------------------------------------
真实作业流程（拉开抽屉之后）：
    拉屉终点 P2 → 后撤退开（与桌面/把手指向错开）→ P1 → P0（最高点，准备视觉下探）
实验 1 要复现的正是 **抱持位 → P2 → P1 → P0** 这一段。

原实现是反的（抱持位 → P0 → P1 → P2），后果：
  · CARRY(−108.9°) 一步冲到 P0(+69.3°)，单段 j1 178.2°；
  · 末端直线 (0.223,−0.215,0.802) → (0.524,−0.220,1.258) 的 y 几乎不变，
    t≈0.3 就进入 x>0.26 / z<0.98 的抽屉占位区 —— **正好刮过桌上的抽屉**；
  · 真机 09:04 实测：entry 12 点勉强全过，一离开 P0 走 JNT_P0_P1_3 即 ControlStatus.FAULT。

正确顺序的几何优势：j1 单调递增 −108.9 → −91.3 → +32.6 → +69.3，
且 CARRY→P2 的中途 y 已经被拉到 −0.46 以下，x 才刚过 0.26 —— 从右侧绕开。

方法
----
与 _plan_via.py 同一套：末端位置线性 + 姿态球面插值，**上一点解作下一点 IK 种子**
（保证分支连续），逐步加密直到相邻构型最大关节差 ≤ TARGET_STEP_DEG。

比 _plan_via 多做两件事：
  1. 三段分别看，逐段报告；某段末端直线走不通就退回关节 lerp 并显式标注；
  2. **抽屉占位检查**：不只查 TCP，还查腕心（right_arm_joint7 原点）与肘
     （right_arm_joint5 原点）是否进入 DRAWER_BOX。TCP 不撞不代表手臂不撞。

⚠️ 仍是离线近似：无碰撞体、无奇异检测。产出点为**候选**，必须真机低速验证。

用法
----
    python _plan_carry_p2_p1_p0.py            # 只看方案
    python _plan_carry_p2_p1_p0.py --write    # 写进 teach_drawer_real.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from fk_fix import chain_to, fk, load_tree, rot_axis  # noqa: E402
from fk_reach_3pts import ARM_J, calibrate, fk_tcp, ik_pose  # noqa: E402
from _gen_teach_real import mat_to_quat  # noqa: E402

# 归档进仓库后：本目录 = <repo>/tools/drawer_calib，仓库根 = 上两级
REPO = os.path.dirname(os.path.dirname(HERE))
TEACH = os.path.join(REPO, "scenes", "scene_medicine_check",
                     "pipeline", "configs", "teach_drawer_real.json")

DEG = 180.0 / math.pi

#: 相邻构型最大关节差目标（远小于 runner 的 90° danger 线）
TARGET_STEP_DEG = 15.0
#: 每段最多切几步
MAX_SPLIT = 40
#: IK 收敛判据
POS_TOL_M = 0.005
ROT_TOL_DEG = 2.0

LIMITS_DEG = {
    "right_arm_joint1": (-172.1, 172.1), "right_arm_joint2": (-92.1, 92.1),
    "right_arm_joint3": (-167.1, 167.1), "right_arm_joint4": (17.2, 147.1),
    "right_arm_joint5": (-167.1, 167.1), "right_arm_joint6": (-42.1, 47.1),
    "right_arm_joint7": (-88.1, 88.1),
}

#: 抽屉占位盒（**只按 TCP 判**，见下方 note）
#: 标定依据：ALIGN_handle 实测把手 TCP x=0.7517（抽屉已拉出 30 cm）
#:   → 未拉出时把手/外壳前端面 x≈0.45；外壳宽按 0.40 包封（中心 y≈−0.24）；桌面 z=0.72。
#: 两套口径只差 z 顶：BASE 按"外壳顶不超过桌面 33 cm"，TALL 按 48 cm，做敏感性分析。
BOX_BASE = {"x": (0.42, 0.80), "y": (-0.46, 0.00), "z": (0.70, 1.05)}
BOX_TALL = {"x": (0.42, 0.80), "y": (-0.46, 0.00), "z": (0.70, 1.20)}

#: ⚠️ 只查 TCP。URDF 里 j5/j6/j7 交于腕心同一点，但算出的腕心位置与"法兰沿夹爪轴
#: 外伸 0.109 m"自相矛盾（P0 处腕心比 TCP 低 39 cm），该段 FK 不可信 → 不做腕/肘检查。

#: (起点, 终点, 生成的中间点前缀)
SEGS = [
    ("CARRY_home", "P2_measured", "VIA_CARRY_P2"),
    ("P2_measured", "P1_measured", "VIA_P2_P1"),
    ("P1_measured", "P0_measured", "VIA_P1_P0"),
]
#: 方向错、必须清掉的旧点
STALE_PREFIXES = ("JNT_ENTRY_", "JNT_P0_P1_")


def ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1) / 2))))


def slerp(R1, R2, t):
    qd = R2 @ R1.T
    a = math.acos(max(-1.0, min(1.0, (np.trace(qd) - 1) / 2)))
    if a < 1e-9:
        return R1.copy()
    ax = np.array([qd[2, 1] - qd[1, 2], qd[0, 2] - qd[2, 0], qd[1, 0] - qd[0, 1]])
    n = np.linalg.norm(ax)
    if n < 1e-12:
        return R1.copy()
    return rot_axis(ax / n, a * t) @ R1


def margin(q):
    out = []
    for n, v in zip(ARM_J, q):
        lo, hi = LIMITS_DEG[n]
        d = float(v) * DEG
        out.append((n.replace("right_arm_", ""), min(d - lo, hi - d)))
    out.sort(key=lambda x: x[1])
    return out[0]


def in_box(p, box):
    return (box["x"][0] <= p[0] <= box["x"][1]
            and box["y"][0] <= p[1] <= box["y"][1]
            and box["z"][0] <= p[2] <= box["z"][1])


def chain_pts(chain, q):
    """返回 {关节名: base_link 下原点位置}，供碰撞近似检查。复用 fk_fix.fk。"""
    jmap = {n: float(v) for n, v in zip(ARM_J, q)}
    pts, _p, _R = fk(chain, jmap)
    return pts


def plan_segment_cart(chain, off, Rc, pa, Ra, pb, Rb, qa, qb, target):
    """末端直线 + slerp + 种子继承。返回 (中间点[(q,t)], worst, n, max_ep, max_er) 或 None。"""
    for n in range(2, MAX_SPLIT + 1):
        seed = qa.copy()
        mid, worst, ok = [], 0.0, True
        max_ep = max_er = 0.0
        for k in range(1, n):
            t = k / n
            tp = pa + (pb - pa) * t
            tR = slerp(Ra, Rb, t)
            q, ep, er, lo, hi = ik_pose(chain, off, Rc, tp, tR, seed, iters=600, lam=0.06)
            max_ep = max(max_ep, float(ep))
            max_er = max(max_er, float(er))
            dq = float(np.degrees(np.abs(q - seed)).max())
            worst = max(worst, dq)
            if ep > POS_TOL_M or er > ROT_TOL_DEG:
                ok = False
                break
            mid.append((q.copy(), t))
            seed = q
        if not ok:
            continue
        worst = max(worst, float(np.degrees(np.abs(qb - seed)).max()))
        if worst <= target:
            return mid, worst, n, max_ep, max_er
    return None


def plan_segment_joint(qa, qb, target):
    """关节 lerp 兜底。返回 (中间点[(q,t)], worst, n)。"""
    span = float(np.degrees(np.abs(qb - qa)).max())
    n = max(2, int(math.ceil(span / target)))
    mid = []
    for k in range(1, n):
        mid.append(((qa + (qb - qa) * (k / n)).copy(), k / n))
    return mid, span / n, n



MIN_MARGIN_DEG = 2.0  # s11_exp_stages.LIMIT_FAIL_DEG


def evaluate(chain, off, rc, Q, a, b, mid):
    """给定中间点，评估这条段的质量（跨度 / 限位 / 抽屉占位命中）。"""
    qa, qb = Q[a], Q[b]
    seq = [qa] + [q for q, _ in mid] + [qb]
    worst = 0.0
    for q0, q1 in zip(seq, seq[1:]):
        worst = max(worst, float(np.degrees(np.abs(q1 - q0)).max()))
    min_m = ("", 9e9, 0)
    zmin = 9e9
    hits_b = hits_t = 0
    rows = []
    for i, q in enumerate(seq):
        p, R = fk_tcp(chain, q, off, rc)
        mj, mv = margin(q)
        if mv < min_m[1]:
            min_m = (mj, mv, i)
        zmin = min(zmin, float(p[2]))
        bw = ["TCP"] if in_box(p, BOX_BASE) else []
        bt = ["TCP"] if in_box(p, BOX_TALL) else []
        if bw:
            hits_b += 1
        if bt:
            hits_t += 1
        rows.append((p, R, mj, mv, bw, bt))
    return dict(worst=worst, min_m=min_m, zmin=zmin, hits_b=hits_b, hits_t=hits_t,
                rows=rows, seq=seq, n=len(mid) + 1)


def score(ev):
    """越小越好。硬约束(限位<2°)排最后；再比窄盒命中、宽盒命中、最低 z、单步跨度。"""
    hard = 0 if ev["min_m"][1] >= MIN_MARGIN_DEG else 1
    return (hard, ev["hits_b"], ev["hits_t"], -ev["zmin"], ev["worst"])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--target-step-deg", type=float, default=TARGET_STEP_DEG)
    args = ap.parse_args()
    target = args.target_step_deg

    by = load_tree()
    chain = chain_to(by, "right_arm_end_effector_mount_link", "base_link")
    off, rc = calibrate(chain)

    with open(TEACH, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    pts = doc["points"]

    KEYS = ("CARRY_home", "P2_measured", "P1_measured", "P0_measured")
    Q = {k: np.array(pts[k]["joint_positions"], dtype=float) for k in KEYS}
    TCP = {k: fk_tcp(chain, Q[k], off, rc) for k in KEYS}

    print("=" * 100)
    print("实验 1 上段重排：CARRY_home → P2 → P1 → P0（正确方向）")
    print(f"目标单步 ≤ {target:.0f}°；限位硬线 {MIN_MARGIN_DEG:.0f}°；每段同时评估"
          f"【末端直线+slerp】与【关节lerp】两方案，按抽屉占位命中数自动选优")
    print("=" * 100)
    print(f"{'点':>12s} {'TCP(x,y,z)':>28s} {'z-桌面mm':>10s} {'j1':>9s}  夹爪x轴")
    for k in KEYS:
        p, R = TCP[k]
        print(f"{k:>12s} ({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f}) "
              f"{(p[2] - 0.72) * 1000:10.1f} {Q[k][0] * DEG:+9.1f}  {np.round(R[:, 0], 3)}")

    out_mid = {}
    chosen = {}
    seg_meta = {}
    for a, b, prefix in SEGS:
        qa, qb = Q[a], Q[b]
        (pa, Ra), (pb, Rb) = TCP[a], TCP[b]
        span = np.degrees(np.abs(qb - qa))
        print("\n" + "-" * 100)
        print(f"{a} → {b}   端点跨度 max {span.max():.1f}° @ j{span.argmax() + 1} / "
              f"sum {span.sum():.1f}°；末端直线 {np.linalg.norm(pb - pa) * 100:.1f} cm，"
              f"姿态 {ang(Ra, Rb):.1f}°")
        cands = []
        res = plan_segment_cart(chain, off, rc, pa, Ra, pb, Rb, qa, qb, target)
        if res is not None:
            mid, worst, n, max_ep, max_er = res
            cands.append(("末端直线", mid, worst, n, f"IK≤{max_ep*1000:.2f}mm/{max_er:.2f}°"))
        else:
            print(f"  [末端直线] {MAX_SPLIT} 段内解不出连续路径（过肘奇异，分支跳变）→ 不采用")
        mid_j, worst_j, n_j = plan_segment_joint(qa, qb, target)
        cands.append(("关节lerp", mid_j, worst_j, n_j, "两端合法则线段合法"))

        evaluated = []
        for mode, mid, worst, n, note in cands:
            ev = evaluate(chain, off, rc, Q, a, b, mid)
            evaluated.append((mode, mid, ev, note))
            print(f"  [{mode:<8s}] {n:>3d} 段 / 单步 {worst:5.1f}° / "
                  f"限位最紧 {ev['min_m'][1]:5.1f}° @ {ev['min_m'][0]:<7s} / "
                  f"TCP 最低 {ev['zmin']:.3f} m / 盒命中 顶1.05m:{ev['hits_b']:2d} 顶1.20m:{ev['hits_t']:2d}"
                  f"   ({note})")
        evaluated.sort(key=lambda x: score(x[2]))
        mode, mid, ev, note = evaluated[0]
        print(f"  ★ 采纳：{mode}（{len(mid)} 个中间点）")
        if ev["min_m"][1] < MIN_MARGIN_DEG:
            print(f"    ⚠️ 仍贴限位 {ev['min_m'][1]:.1f}° @ {ev['min_m'][0]}，真机会被判 failed")
        chosen[prefix] = (mode, mid)
        seg_meta[prefix] = dict(a=a, b=b, mode=mode, n=len(mid) + 1,
                                worst=ev["worst"], min_m=ev["min_m"],
                                zmin=ev["zmin"], hits_b=ev["hits_b"], hits_t=ev["hits_t"],
                                rows=ev["rows"])

        # 明细
        for i, (p, R, mj, mv, bw, bt) in enumerate(ev["rows"]):
            tag = "起点" if i == 0 else ("终点" if i == len(ev["rows"]) - 1 else f"{prefix}_{i}")
            flagw = "/".join(bw) if bw else "—"
            print(f"    {tag:<18s} ({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                  f"z-桌 {(p[2] - 0.72) * 1000:7.1f}mm  {mv:6.1f}° @{mj:<7s} 宽盒:{flagw}")

    # ---- 落盘用的中间点（只保留中间，不含端点） ----
    for prefix, (mode, mid) in chosen.items():
        for k, (q, t) in enumerate(mid, start=1):
            p, R = fk_tcp(chain, q, off, rc)
            mj, mv = margin(q)
            out_mid[f"{prefix}_{k}"] = dict(q=q.copy(), p=p.copy(), R=R.copy(),
                                            t=t, margin=(mj, mv),
                                            note=f"{prefix} 第 {k}/{len(mid)} 点（{mode}，t={t:.3f}）")

    # ---- 全链跨度复核 ----
    print("\n" + "=" * 100)
    print("全链单步跨度（runner 闸门：max≥90° 或 sum≥200° 判 danger → SAFETY_ABORT）")
    print("=" * 100)
    order = ["CARRY_home"]
    for a, b, prefix in SEGS:
        ks = sorted([k for k in out_mid if k.startswith(prefix + "_")],
                    key=lambda s: int(s.rsplit("_", 1)[1]))
        order.extend(ks)
        order.append(b)
    qseq = [(n, Q[n] if n in Q else out_mid[n]["q"]) for n in order]
    worst_all = 0.0
    bad_edges = []
    for (n0, q0), (n1, q1) in zip(qseq, qseq[1:]):
        d = np.degrees(np.abs(q1 - q0))
        worst_all = max(worst_all, float(d.max()))
        risk = "danger" if (d.max() >= 90 or d.sum() >= 200) else (
            "warn" if (d.max() >= 45 or d.sum() >= 120) else "ok")
        if risk != "ok":
            bad_edges.append((n0, n1, float(d.max()), float(d.sum()), risk))
        print(f"  {n0:<18s} → {n1:<18s} max {d.max():5.1f}° @ j{d.argmax() + 1}  "
              f"sum {d.sum():6.1f}°  {risk}")

    print("\n" + "=" * 100)
    print(f"汇总：全链 {len(order)} 点 / {len(order) - 1} 段，最大单步 {worst_all:.1f}°")
    print("=" * 100)
    ok = True
    for prefix, m in seg_meta.items():
        hard = m["min_m"][1] >= MIN_MARGIN_DEG
        clean = (m["hits_b"] == 0)
        flag = "✓" if (hard and clean) else "⚠️"
        if not hard:
            ok = False
        print(f"  {flag} {m['a']} → {m['b']}：{m['mode']} / {m['n']} 段 / 单步 {m['worst']:.1f}° / "
              f"限位最紧 {m['min_m'][1]:.1f}° @ {m['min_m'][0]} / TCP 最低 {m['zmin']:.3f} m / "
              f"盒命中 顶1.05m:{m['hits_b']} 顶1.20m:{m['hits_t']}")
    if bad_edges:
        ok = False
        print("  ⚠️ warn/danger 边：")
        for e in bad_edges:
            print(f"      {e}")
    print(f"\n结论：{'链路可用（离线口径）' if ok else '存在需现场确认的项，见上 ⚠️'}")

    if not args.write:
        print("\n（未写文件。加 --write 才落盘）")
        return 0

    dropped = [k for k in list(pts) if k.startswith(STALE_PREFIXES)]
    for k in dropped:
        pts.pop(k, None)
    print(f"\n清掉 {len(dropped)} 个方向错误的旧点（{sorted(dropped)[:3]}…）")

    for name, rec in out_mid.items():
        q, p, R = rec["q"], rec["p"], rec["R"]
        mj, mv = rec["margin"]
        pts[name] = {
            "pose": [round(float(v), 9) for v in list(p) + mat_to_quat(R)],
            "joint_names": list(ARM_J),
            "joint_positions": [round(float(v), 9) for v in q],
            "gripper_width": 0.05,
            "note": (f"实验1上段中间点（方向 CARRY→P2→P1→P0，与真实作业轨迹同向）。"
                     f"{rec['note']}。最紧余量 {mv:.1f}° @ {mj}。"
                     f"**离线候选，未做碰撞检查、未上真机**。"),
            "src": "ik_subdivision_carry_p2_p1_p0",
            "ee_pose_ok": True,
            "joints_ok": True,
            "fk_position_trusted": False,
            "position_source": "urdf_fk_calibrated",
            "measured": False,
        }
    doc["provenance"]["VIA_CARRY_P2/P2_P1/P1_P0"] = (
        "实验 1 上段按【正确方向】CARRY_home → P2 → P1 → P0 重排（用户 2026-11-08 09:16 澄清："
        "拉开抽屉后是先退开错位、经 P1 到最高点 P0）。三段各自在"
        "【末端直线+slerp+种子继承】与【关节 lerp】间按抽屉占位命中数自动选优，"
        f"相邻构型最大关节差 ≤ {target:.0f}°。抽屉占位盒 x{BOX_BASE['x']} y{BOX_BASE['y']}，"
        f"z 顶做 1.05/1.20 m 两套敏感性；只判 TCP（该段腕心 FK 不可信）。"
        "⚠️ 离线近似：无碰撞体、无奇异检测，真机须低速并现场盯。")
    doc["provenance"]["P0_P1_P2_direction"] = (
        "方向纠正：P2 = 拉完抽屉的终点，P0 = 准备下探的最高点，正确回收顺序 P2→P1→P0。"
        "原 JNT_ENTRY_*/JNT_P0_P1_* 按反方向生成，其中 CARRY→P0 的末端路径 y 几乎不变、"
        "t≈0.3 即进入抽屉占位区，会刮到桌上的抽屉；真机 2026-11-08 09:04 亦在 JNT_P0_P1_3 "
        "返回 ControlStatus.FAULT。该批点已删除。")

    with open(TEACH, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    print(f"已写入 {TEACH}（共 {len(pts)} 个点，新增 {len(out_mid)} 个中间点）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
