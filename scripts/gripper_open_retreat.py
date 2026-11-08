#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""张爪到最大 → 沿夹爪朝向后退指定距离（默认 15 cm）。

这是合并流程的第 ④ 步（probe_pull 预检之后、抬手程序之前）。

流程
----
  1. 读当前末端位姿 + 夹爪轴 + 实际开度（只读）
  2. 夹爪开到最大 —— 走 dev adapter 直发，绕开 wrapper 的 8.5 cm 截断；
     顶到物理限位会返回 TIMEOUT，按**回读宽度**判定，不算失败
  3. 沿夹爪 +X 轴后退 15 cm（move_along_gripper 负距离，纯平移 auto_lift=False）
  4. 复测：实测位移 / 沿轴投影（方向余弦）/ 垂直分量 / Δz

安全设计
--------
1. **后退不抬升**：auto_lift=False 时既不抬升也不拒绝，走的就是夹爪轴那条直线
   （G-20 系列教训：auto_lift=True 会先竖直抬 6 cm，把"后退"变成"斜向下"）。
2. **夹爪轴不水平时后退必带竖直分量** —— 这是轴向的几何必然，不是 bug。
   Δz 超过 --max-dz-cm 会打警告，让操作者知道手往下掉了多少。
3. 单次行程上限 0.30 m（dev 层硬限），超了会**拒绝**而不是静默截断。
4. 张爪失败（夹住东西没松开）直接停，不带着夹持物后退。

用法
----
    cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 -u scripts/gripper_open_retreat.py --dry-run   # 只读，看计划
    python3 -u scripts/gripper_open_retreat.py             # 张爪 + 后退 15 cm
    python3 -u scripts/gripper_open_retreat.py --retreat 0.10 --open-cm 10
"""
from __future__ import annotations

import argparse
import math
import sys
import time

CHAIN = "right_arm"
EE = "right_gripper"
M_TO_CM = 100.0

_ROBOT = None


# ──────────────────────────────────────────────────────────────
# 基础工具
# ──────────────────────────────────────────────────────────────
def quat_to_R(q):
    """四元数 (x, y, z, w) → 3x3 旋转矩阵（行优先 list of list）。"""
    x, y, z, w = [float(v) for v in q[:4]]
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def gripper_axis(pose):
    """夹爪指向 = EndEffector 系的 +X 轴 = 旋转矩阵第 0 列。"""
    R = quat_to_R(pose[3:])
    return [R[0][0], R[1][0], R[2][0]]


def _ee():
    import sdk_wrapper.wrapper as W

    r = W.get_ee_pose(CHAIN)
    if not r.get("success"):
        raise RuntimeError("读末端位姿失败：%s" % r.get("message"))
    return [float(v) for v in r["pose"]]


def _robot():
    """拿 SDK robot 单例读夹爪状态。

    ★ 必须先让 wrapper 把 adapter / SDK 初始化好，否则
      `gm.GalbotRobot().get_gripper_state()` 会**段错误**（2026-11-08 真机实测）。
    """
    global _ROBOT
    if _ROBOT is None:
        import galbot_sdk.g1 as gm

        _ee()  # 初始化 adapter 单例
        _ROBOT = gm.GalbotRobot()
    return _ROBOT


def width_cm():
    try:
        s = _robot().get_gripper_state(EE)
        return round(float(s.width) * M_TO_CM, 2)
    except Exception:  # noqa: BLE001
        return None


def show_pose(tag=""):
    p = _ee()
    ax = gripper_axis(p)
    tilt = abs(math.degrees(math.asin(max(-1.0, min(1.0, -ax[2])))))
    print("  %sxyz = [%.4f, %.4f, %.4f]" % (tag, p[0], p[1], p[2]))
    print("  %s夹爪轴 = [%.4f, %.4f, %.4f]   相对水平偏离 %.2f°"
          % (tag, ax[0], ax[1], ax[2], tilt))
    return p, ax


# ──────────────────────────────────────────────────────────────
# ① 张爪
# ──────────────────────────────────────────────────────────────
def open_gripper(target_cm, effort, min_cm, dry_run):
    """把夹爪开到 target_cm。返回 True / False。"""
    if dry_run:
        now = width_cm()
        print("  [dry] 请求开度 %.2f cm（当前 %.2f cm）—— 不下发"
              % (target_cm, now if now is not None else float("nan")))
        return True

    from sdk_wrapper.dev.toolhub import get_adapter

    _ee()  # 确保单例就绪
    r = get_adapter().set_gripper_width(
        EE, float(target_cm) / M_TO_CM, effort=float(effort), is_blocking=True
    )
    sname = getattr(getattr(r, "status", None), "value", str(r.status))
    msg = str(getattr(r, "message", ""))

    if sname == "succeeded":
        print("  夹爪 -> 请求 %.2f cm，已下发" % target_cm)
    elif "TIMEOUT" in msg.upper():
        # 顶到物理限位（空夹全开就是到不了 12 cm），靠回读宽度判断，不算失败
        print("  夹爪未到 %.2f cm（TIMEOUT）—— 顶到物理限位是正常现象，看实测宽度"
              % target_cm)
    else:
        print("  ✘ 夹爪设定失败：status=%s  %s" % (sname, msg), file=sys.stderr)
        return False

    time.sleep(0.5)
    actual = width_cm()
    if actual is None:
        print("  ✘ 读不到夹爪实际宽度 —— 无法确认是否已张开", file=sys.stderr)
        return False
    print("  实测宽度 %.2f cm" % actual)
    if actual < min_cm:
        print("  ✘ 实测宽度 %.2f cm < 最低要求 %.2f cm —— 可能夹住东西没松开，停下"
              % (actual, min_cm), file=sys.stderr)
        return False
    return True


# ──────────────────────────────────────────────────────────────
# ② 沿夹爪轴后退
# ──────────────────────────────────────────────────────────────
def retreat(distance_m, use_torso, dry_run):
    import sdk_wrapper.wrapper as W

    p0, ax = show_pose("起点  ")
    tilt = abs(math.degrees(math.asin(max(-1.0, min(1.0, -ax[2])))))
    tgt = [p0[i] - distance_m * ax[i] for i in range(3)]
    dz_cm = (tgt[2] - p0[2]) * M_TO_CM
    print("  后退方向 = -夹爪轴 = [%.4f, %.4f, %.4f]" % (-ax[0], -ax[1], -ax[2]))
    print("  目标 xyz = [%.4f, %.4f, %.4f]   （z 变化 %+.1f mm）"
          % (tgt[0], tgt[1], tgt[2], dz_cm * 10.0))

    # ★ 姿态预警：夹爪轴一竖，"后退"就变成"升降"，和直觉完全不同
    if abs(ax[2]) > 0.55:
        print("")
        print("  ⚠⚠ 夹爪轴接近竖直（z 分量 %+.3f，偏离水平 %.1f°）" % (ax[2], tilt))
        print("      所以这次「沿夹爪朝向后退」实际是%s %.1f cm，**不是水平拉出**。"
              % ("向上抬" if dz_cm > 0 else "向下压", abs(dz_cm)))
        print("      若你要的是水平往外拉，先把夹爪摆水平再跑"
              "（jog_ee.py / set_horizontal_pose.py）。")
        print("")

    if dry_run:
        print("  [dry] 不下发")
        return True

    print("  下发中 ...", flush=True)
    res = W.move_along_gripper(-float(distance_m), CHAIN,
                               use_torso=bool(use_torso), auto_lift=False)
    if not res.get("success"):
        p1 = _ee()
        print("  ✘ 沿夹爪后退失败：status=%s  %s"
              % (res.get("status"), res.get("message")), file=sys.stderr)
        print("    起点 [%.4f, %.4f, %.4f]" % (p0[0], p0[1], p0[2]), file=sys.stderr)
        print("    目标 [%.4f, %.4f, %.4f]" % (tgt[0], tgt[1], tgt[2]), file=sys.stderr)
        print("    当前 [%.4f, %.4f, %.4f]" % (p1[0], p1[1], p1[2]), file=sys.stderr)
        print("    （adapter 内部已自动重试 3 次：2 次直线 + 1 次绕行规划）",
              file=sys.stderr)
        return False

    time.sleep(0.25)
    p1 = _ee()
    dv = [p1[i] - p0[i] for i in range(3)]
    dist = math.sqrt(sum(v * v for v in dv))
    # 沿轴投影（负值 = 确实在后退）
    proj = sum(dv[i] * ax[i] for i in range(3))
    # 垂直于轴的分量
    v_par = [proj * ax[i] for i in range(3)]
    v_perp = [dv[i] - v_par[i] for i in range(3)]
    perp = math.sqrt(sum(v * v for v in v_perp))
    # 方向余弦：实测方向与"理论后退方向"的贴合度，1.0 = 完全沿轴
    if dist > 1e-6:
        cos = (-proj) / dist
    else:
        cos = float("nan")

    print("")
    print("  终点 xyz = [%.4f, %.4f, %.4f]" % (p1[0], p1[1], p1[2]))
    print("  实测位移 = %.2f cm   （目标 %.2f cm，executed=%s）"
          % (dist * M_TO_CM, distance_m * M_TO_CM, res.get("executed_m")))
    print("  沿夹爪轴 = %+.2f cm   方向余弦 %.4f" % (proj * M_TO_CM, cos))
    print("  垂直偏离 = %.2f cm    Δz = %+.2f cm"
          % (perp * M_TO_CM, dv[2] * M_TO_CM))
    return True


# ──────────────────────────────────────────────────────────────
def main() -> int:
    global CHAIN

    ap = argparse.ArgumentParser(
        description="张爪到最大 + 沿夹爪朝向后退（默认 15 cm）")
    ap.add_argument("--open-cm", type=float, default=12.0,
                    help="夹爪目标开度（cm），默认 12（顶到物理限位约 10.x cm）")
    ap.add_argument("--effort", type=float, default=10.0, help="夹持力度 N·m，默认 10")
    ap.add_argument("--min-open-cm", type=float, default=8.0,
                    help="张爪后实测宽度下限（cm），低于此判为没张开，默认 8")
    ap.add_argument("--retreat", type=float, default=0.15,
                    help="后退距离（米），默认 0.15；上限 0.30")
    ap.add_argument("--chain", default=CHAIN, help="手臂链，默认 right_arm")
    ap.add_argument("--no-torso", action="store_true",
                    help="纯手臂（with_chain_only），不让躯干协助")
    ap.add_argument("--max-dz-cm", type=float, default=3.0,
                    help="Δz 超过多少 cm 就打警告，默认 3.0")
    ap.add_argument("--dry-run", action="store_true", help="只读，只打印计划")
    args = ap.parse_args()

    CHAIN = args.chain

    if abs(args.retreat) > 0.30:
        print("后退距离 %.3f m 超过单次上限 0.30 m —— 请拆成多次调"
              % args.retreat, file=sys.stderr)
        return 2
    if args.retreat <= 0:
        print("--retreat 必须为正数（负号由程序内部加）", file=sys.stderr)
        return 2

    print("")
    print("=" * 66)
    print("  张爪 + 沿夹爪朝向后退")
    print("=" * 66)
    print("  参数：开度 %.1f cm / 力度 %.0f N·m / 后退 %.1f cm / %s%s"
          % (args.open_cm, args.effort, args.retreat * M_TO_CM,
             "纯手臂" if args.no_torso else "腰臂协同",
             "（dry-run）" if args.dry_run else ""))

    print("")
    print("── ① 当前状态 " + "─" * 48)
    p0, _ax = show_pose("  ")
    w0 = width_cm()
    print("  夹爪开度 %.2f cm" % w0 if w0 is not None else "  夹爪开度读不到")

    print("")
    print("── ② 张爪到最大 " + "─" * 46)
    if not open_gripper(args.open_cm, args.effort, args.min_open_cm, args.dry_run):
        print("\n✘ 张爪失败，中止（不带着没松开的夹爪后退）", file=sys.stderr)
        return 2

    print("")
    print("── ③ 沿夹爪朝向后退 %.1f cm " % (args.retreat * M_TO_CM) + "─" * 30)
    if not retreat(args.retreat, not args.no_torso, args.dry_run):
        return 2

    if args.dry_run:
        print("\n（dry-run 完成，机器人没有动）")
        return 0

    # Δz 警告
    p1 = _ee()
    dz_cm = (p1[2] - p0[2]) * M_TO_CM
    print("")
    print("=" * 66)
    if abs(dz_cm) > args.max_dz_cm:
        print("  ⚠ 后退过程中高度变化 %+.2f cm（超 %.1f cm 阈值）—— "
              "夹爪轴不水平时的几何必然，注意别蹭到桌面" % (dz_cm, args.max_dz_cm))
    else:
        print("  ✓ 完成：后退 %.1f cm，Δz %+.2f cm" % (args.retreat * M_TO_CM, dz_cm))
    print("  终点 xyz = [%.4f, %.4f, %.4f]" % (p1[0], p1[1], p1[2]))
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
