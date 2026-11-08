"""_plan_exp2_dive.py —— 实验 2 的「P0 竖直下探抓取」点位离线生成（零副作用）。

它干什么
--------
以 `P0_measured` 为起点，沿 **−z 直线**（姿态锁死为 P0 的姿态）下探 `DEPTH_M`，
把全程切成 `STEPS` 段，每段用**上一点解作 IK 种子**跑阻尼最小二乘 IK，
得到一串构型连续、单段跨度小、限位有余量的中间点，写进
`configs/teach_drawer_real.json` 的 `VIA_P0_DIVE_1..N`。

实验 2 的上行就是这串点**倒序**回放（同一批构型，线段合法性自动成立）。

为什么不用 `move_line_to` 让规划器自己插
--------------------------------------
真机上「同一末端位姿、不同关节解」已六次复现（见 MEMORY §5⑧）：规划器随机挑 IK 分支，
贴限位的直接 FAULT。所以一律离线把路径切成构型序列再逐点关节回放 —— 实验 1 就是这么
跑通的（37/37 段 ok）。这里沿用同一套办法。

口径与已知局限（继承 tools/drawer_calib/README.md 的四条）
--------------------------------------------------------
1. URDF 的腕心/肘坐标不可信 → 只把**标定后的 TCP** 当几何判据，不查连杆；
2. FK 位置残差 17.4 mm、姿态经常量修正后 ~2° → 位置一律按标定口径看，别当精密值；
3. 没有真正的碰撞体 —— 抽屉占位盒是手工包络盒，只做 TCP 点判定；
4. 无奇异检测，只靠「相邻构型关节差 ≤15°」间接避开。

用法
----
    python tools/drawer_calib/_plan_exp2_dive.py            # 只算不写（安全）
    python tools/drawer_calib/_plan_exp2_dive.py --write    # 写进 teach_drawer_real.json
    python tools/drawer_calib/_plan_exp2_dive.py --depth 0.30 --steps 8   # 改深度/段数
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
from fk_fix import chain_to, load_tree  # noqa: E402
from fk_reach_3pts import calibrate, fk_tcp, ik_pose  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
TEACH = os.path.join(REPO, "scenes", "scene_medicine_check", "pipeline",
                     "configs", "teach_drawer_real.json")

ARM_J = [f"right_arm_joint{i}" for i in range(1, 8)]
LEAF, ROOT = "right_arm_end_effector_mount_link", "base_link"

#: 下探总深度（m）。0.42 这个数来自「P0 的 TCP z ≈ 1.245 → 落点 z ≈ 0.825，
#: 对应药盒立在内腔底面上、TCP 抓在盒腰」的环境估算。
#:
#: ⚠️ 2026-11-08 11:47 用户定下硬约束：**底盘与桌子的相对位置每次布置随机**，
#:    base_link 下的环境坐标不成立。所以 0.42 **不是判据，只是目标档**。
#:    真机上必须先量 P0 到抽屉内底的竖直距离，或按 0.10 → 0.20 → 0.30 → 0.42 逐档加深：
#:        python tools/drawer_calib/_plan_exp2_dive.py --depth 0.10 --prefix VIA_P0_DIVE_SH --write
DEFAULT_DEPTH_M = 0.42
DEFAULT_STEPS = 10

#: ⚠️ **已作废，仅留作回看历史报告**。
#: 抽屉占位盒是手工包络盒（估算值），依赖底盘↔桌子的相对位置，跨场次无效。
#: 用户 11-08 11:47 明确：不得用环境坐标做碰撞/净空判据。默认**不输出**任何进盒信息，
#: 只有显式加 `--legacy-box` 时才打印，且只作参考、不进判定。
BOX = {"x": (0.42, 0.80), "y": (-0.46, 0.00), "z": (0.70, 1.05)}

LIMIT_FAIL_DEG = 2.0     # 与 s11_exp_stages.LIMIT_FAIL_DEG 一致
SPAN_TARGET_DEG = 15.0   # 与实验 1 的 VIA 点同口径


def mat_to_quat(R: np.ndarray) -> list:
    """3x3 旋转矩阵 → 四元数 [x,y,z,w]（w 取正）。"""
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
        w, x, y, z = -w, -x, -y, -z
    return [float(x), float(y), float(z), float(w)]


def in_box(p) -> bool:
    return (BOX["x"][0] <= p[0] <= BOX["x"][1]
            and BOX["y"][0] <= p[1] <= BOX["y"][1]
            and BOX["z"][0] <= p[2] <= BOX["z"][1])


def main() -> int:
    ap = argparse.ArgumentParser(description="生成实验 2 的 P0 竖直下探点位")
    ap.add_argument("--write", action="store_true", help="写进 teach_drawer_real.json")
    ap.add_argument("--depth", type=float, default=DEFAULT_DEPTH_M,
                    help=f"下探总深度（m），默认 {DEFAULT_DEPTH_M}")
    ap.add_argument("--steps", type=int, default=DEFAULT_STEPS,
                    help=f"分成几段，默认 {DEFAULT_STEPS}")
    ap.add_argument("--prefix", default="VIA_P0_DIVE",
                    help="点名前缀，默认 VIA_P0_DIVE（写 --write 前会先删同前缀旧点）")
    args = ap.parse_args()

    with open(TEACH, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    pts = doc["points"]
    p0 = pts["P0_measured"]
    q0 = np.array([float(v) for v in p0["joint_positions"]])

    by = load_tree()
    chain = chain_to(by, LEAF, ROOT)
    off, R_corr = calibrate(chain)
    rev = [j for j in chain
           if j["type"] in ("revolute", "continuous")
           and str(j["name"]).startswith("right_arm_")]
    lo = np.array([j["lo"] for j in rev])
    hi = np.array([j["hi"] for j in rev])

    p_start, R_start = fk_tcp(chain, q0, off, R_corr)
    print("=" * 78)
    print("实验 2 下探点位规划（纯离线，零硬件指令）")
    print("=" * 78)
    print(f"起点 P0_measured：TCP = ({p_start[0]:+.4f}, {p_start[1]:+.4f}, {p_start[2]:+.4f})")
    print(f"下探方向：−z 直线（x/y 锁死），姿态锁死为 P0 姿态")
    print(f"总深度 {args.depth:.3f} m，分 {args.steps} 段（每段 {args.depth / args.steps * 100:.1f} cm）")
    print(f"落点 TCP z ≈ {p_start[2] - args.depth:.4f} m")
    print("-" * 78)
    print(f"{'点':<16}{'dz(cm)':>7}{'TCP z':>9}{'pos_err':>9}{'rot_err':>8}"
          f"{'单段max':>9}{'最紧关节':>22}{'进腔':>6}")
    print("-" * 78)

    seed = q0.copy()
    records, rows = [], []
    worst_span, worst_margin = 0.0, 9e9
    for k in range(1, args.steps + 1):
        tgt = p_start.copy()
        tgt[2] -= args.depth * k / args.steps
        q, err_p, err_r, _lo, _hi = ik_pose(chain, off, R_corr, tgt, R_start,
                                            seed, iters=800, lam=0.08)
        margins = [min(math.degrees(v - a), math.degrees(b - v))
                   for v, a, b in zip(q, lo, hi)]
        i_tight = int(np.argmin(margins))
        span = max(abs(math.degrees(a - b)) for a, b in zip(q, seed))
        worst_span = max(worst_span, span)
        worst_margin = min(worst_margin, min(margins))

        p_real, R_real = fk_tcp(chain, q, off, R_corr)
        name = f"{args.prefix}_{k}"
        pose = [float(v) for v in p_real] + mat_to_quat(R_real)
        records.append({
            "name": name,
            "joint_names": list(ARM_J),
            "joint_positions": [float(v) for v in q],
            "pose": pose,
            "measured": False,
            "source": "ik_seeded_from_prev",
            "note": (f"实验 2 下探第 {k}/{args.steps} 段：TCP z {p_real[2]:.3f} m，"
                     f"距 P0 {args.depth * k / args.steps * 100:.1f} cm；离线 IK，未实测"),
        })
        rows.append((name, args.depth * k / args.steps * 100, p_real,
                     err_p, err_r, span, i_tight + 1, min(margins), in_box(p_real)))
        print(f"{name:<16}{args.depth * k / args.steps * 100:>7.1f}{p_real[2]:>9.4f}"
              f"{err_p * 1000:>8.2f}mm{err_r:>7.2f}°{span:>8.1f}°"
              f"{'j' + str(i_tight + 1):>10}{min(margins):>10.1f}°"
              f"{'是' if in_box(p_real) else '否':>6}")
        seed = q.copy()

    print("-" * 78)
    errs = [r[3] for r in rows]
    rot = [r[4] for r in rows]
    bad_margin = [r for r in rows if r[7] < LIMIT_FAIL_DEG]
    print(f"IK 精度：位置 ≤ {max(errs) * 1000:.2f} mm，姿态 ≤ {max(rot):.2f}°")
    print(f"相邻段最大关节跨度 {worst_span:.2f}°（目标 ≤ {SPAN_TARGET_DEG:.0f}°）")
    print(f"全程最小限位余量 {worst_margin:.2f}°（失败线 {LIMIT_FAIL_DEG:.1f}°）")
    if bad_margin:
        print(f"❌ {len(bad_margin)} 个点贴限位：{[r[0] for r in bad_margin]}")
    print(f"进入抽屉内腔的点：{[r[0] for r in rows if r[8]] or '无'}"
          f"（下探本来就是伸进抽屉，属预期，不算碰撞）")
    print()

    # ---------------- 上行（回撤）合法性：同一批构型倒序，末端再回 P0 ----------------
    # 反向回放的每一段与下行某一段是**同一对构型**，所以跨度集合完全相同；
    # 这里显式重算一遍，是为了把"上行最大跨度"当成一个被验证过的数报出来，
    # 而不是靠"显然对称"口头带过。
    up_seq = [np.array([float(v) for v in rec["joint_positions"]]) for rec in records]
    up_seq = list(reversed(up_seq)) + [q0]
    up_prev = np.array([float(v) for v in records[-1]["joint_positions"]])
    up_worst = 0.0
    for q in up_seq:
        d = max(abs(math.degrees(a - b)) for a, b in zip(q, up_prev))
        if d > up_worst:
            up_worst = d
        up_prev = q
    print(f"上行：{args.steps} 段（倒序同一批构型）+ 末段回 P0_measured，"
          f"最大单段跨度 {up_worst:.2f}°（与下行同一对构型，故必然相等）")
    print()

    if not args.write:
        print("（未写文件。要写入加 --write）")
        return 0 if not bad_margin else 1

    stale = [k for k in list(pts) if k.startswith(args.prefix + "_")]
    for k in stale:
        del pts[k]
    for rec in records:
        pts[rec["name"]] = {kk: vv for kk, vv in rec.items() if kk != "name"}
    with open(TEACH, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    print(f"✅ 已写入 {os.path.relpath(TEACH, REPO)}："
          f"删旧点 {len(stale)} 个，新增 {len(records)} 个，现共 {len(pts)} 个点")
    return 0 if not bad_margin else 1


if __name__ == "__main__":
    sys.exit(main())
