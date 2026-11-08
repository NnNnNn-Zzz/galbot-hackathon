#!/usr/bin/env python3
"""只读：检查从当前位姿沿夹爪轴后退 27 cm 是否可达。"""
import math
import sys

import galbot_sdk.g1 as gm

import sdk_wrapper.wrapper as W

CHAIN = "right_arm"
PULL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.27


def quat_to_R(q):
    x, y, z, w = [float(v) for v in q]
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


r = W.get_ee_pose(CHAIN)
p = [float(v) for v in r["pose"]]
ax = [quat_to_R(p[3:])[i][0] for i in range(3)]
print(f"当前 xyz = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]")
print(f"夹爪轴   = [{ax[0]:.4f}, {ax[1]:.4f}, {ax[2]:.4f}]")

motion = gm.GalbotMotion()
print("motion.init() ->", motion.init())

print(f"\n后退路径上各个点的 IK（目标 = 当前 - {PULL*100:.0f}cm × 轴）:")
print(f"{'已退':>8} {'目标 xyz':>30} {'IK带碰撞':>12} {'IK不查':>10}")
fails = []
for i in range(0, int(round(PULL * 100)) + 1, 3):
    frac = i / 100.0
    tgt = [p[j] - frac * ax[j] for j in range(3)] + list(p[3:])
    row = [f"{i:>6}cm", f"[{tgt[0]:.3f},{tgt[1]:.3f},{tgt[2]:.3f}]".rjust(30)]
    for check in (True, False):
        try:
            s, _ = motion.inverse_kinematics(tgt, [CHAIN], "EndEffector", "base_link",
                                             enable_collision_check=bool(check))
            name = str(s).split(".")[-1]
        except Exception as exc:  # noqa: BLE001
            name = "EXC"
        row.append(f"{name:>12}" if check else f"{name:>10}")
        if check and name != "SUCCESS":
            fails.append(i)
    print(" ".join(row))

if fails:
    print(f"\n⚠ 带碰撞检查失败的退距: {fails} cm —— 连续拉可能中途 FAULT")
else:
    print(f"\n✓ 0~{PULL*100:.0f} cm 全程单点可达")
print("注：单点可达 ≠ 整段直线可行，真实执行仍可能中途 FAULT（会被自动重试）。")
