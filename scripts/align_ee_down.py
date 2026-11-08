#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 G1 末端摆到「接近轴竖直向下 + 夹爪绕接近轴内旋 N 度」的标准姿态。

末端轴约定（2026-11-07 用腕部相机 + 挂载 TCP 实测标定，见
docs/charger_grasp_design.md 第 125 行）::

    local +X = 接近轴（法兰 -> TCP，TCP 沿 +X 偏置约 0.146 m）
    local +Y = 两指闭合轴
    local +Z = 夹爪长轴

于是两个要求可以精确翻译：

* 「末端方向竖直向下」: local +X 指向 base_link 的 ``[0, 0, -1]``。
* 「夹爪内旋 θ」: 绕 +X 自转，使闭合轴 +Y 从「朝前(+X)」转向「朝身体中线」。
  θ=0   -> 闭合轴朝前
  θ=90  -> 闭合轴朝中线（右臂转向 +Y，左臂转向 -Y）

位置**保持不变**，只改姿态（可用 --dz 微调高度）。下发前一定先做只读 IK 预检，
预检不过就拒绝，不会把命令送进运动服务。

用法
----
    source ~/galbot_hackathon/scripts/env_hpu.sh

    # 只看计划，不动（推荐第一步）
    python3 scripts/align_ee_down.py --dry-run

    # 双手都摆：竖直向下 + 内旋 90°
    python3 scripts/align_ee_down.py

    # 只摆右臂，内旋 45°
    python3 scripts/align_ee_down.py --chain right --roll-deg 45

    # 只要求竖直向下，不管自转
    python3 scripts/align_ee_down.py --roll-deg 0

注意
----
* 程序会打印每条臂的**原始 7 维位姿**，想手动回退时照着 `move_ee_to` 用。
* 纯姿态旋转会让整条手臂重构，动作前确认周围无人、急停在手边。
* 夹爪两指反了（闭合轴差 180°）时物理外观几乎一样，但本程序按语义严格校验。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

import numpy as np

from sdk_wrapper.dev.adapters.galbot_adapter import GalbotAdapter
from sdk_wrapper.dev.algorithms import geometry as geo

#: base_link 下的「竖直向下」
DOWN = np.array([0.0, 0.0, -1.0])

#: 内旋角为 0 时闭合轴的朝向（机器人正前方）
FORWARD = np.array([1.0, 0.0, 0.0])

#: 别名 -> SDK 的 chain 名
CHAIN_ALIAS = {"right": "right_arm", "left": "left_arm", "both": None}

#: 身体中线在 base_link Y 轴上的符号（右臂在 -Y 侧，中线在 +Y 侧）
MIDLINE_SIGN = {"right_arm": 1.0, "left_arm": -1.0}

#: 已经符合目标时，允许的偏差（度）
DEFAULT_TOL_DEG = 2.0

#: 单位向量最大偏差（自检用）
UNIT_TOL = 1e-6


def say(msg: str = "") -> None:
    print(msg, flush=True)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("零向量无法归一化")
    return v / n


def _angle_deg(u: np.ndarray, v: np.ndarray) -> float:
    """两向量夹角（度），内部会归一化。"""
    uu, vv = _unit(u), _unit(v)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(uu @ vv)))))


def _signed_angle_about(axis: np.ndarray, u: np.ndarray, v: np.ndarray) -> float:
    """在垂直于 axis 的平面内，从 u 转到 v 的有符号角（度），右手定则。"""
    a = _unit(axis)
    up = u - float(u @ a) * a
    vp = v - float(v @ a) * a
    nu, nv = float(np.linalg.norm(up)), float(np.linalg.norm(vp))
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    up, vp = up / nu, vp / nv
    return math.degrees(math.atan2(float(np.cross(up, vp) @ a), float(up @ vp)))


def build_target_rotation(chain: str, roll_deg: float):
    """构造目标旋转矩阵与三轴。

    返回 (R, x_axis, y_axis, z_axis)，R 的列依次是局部 +X/+Y/+Z 在 base_link 下的方向。
    """
    x_axis = DOWN.copy()

    midline = np.array([0.0, MIDLINE_SIGN[chain], 0.0])
    theta = math.radians(float(roll_deg))
    y_axis = _unit(math.cos(theta) * FORWARD + math.sin(theta) * midline)

    # 保证 +Y 真的垂直于 +X（理论上已垂直，这里防御性正交化）
    y_axis = _unit(y_axis - float(y_axis @ x_axis) * x_axis)

    z_axis = _unit(np.cross(x_axis, y_axis))
    # 用 z = x X y 重建 y，彻底保证右手正交基
    y_axis = _unit(np.cross(z_axis, x_axis))

    R = np.column_stack([x_axis, y_axis, z_axis])

    # 自检：正交、单位、右手
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9), "目标旋转矩阵非正交"
    assert np.allclose(np.cross(x_axis, y_axis), z_axis, atol=1e-9), "目标轴不是右手系"
    return R, x_axis, y_axis, z_axis


def fmt_vec(v, digits: int = 3) -> str:
    fmt = "%+." + str(int(digits)) + "f"
    return "[" + ", ".join(fmt % float(x) for x in v) + "]"


def find_lower_feasible(ad: GalbotAdapter, chain: str, roll_deg: float,
                        pose: List[float], floor_z: float, step_m: float):
    """位置不可行时，保持 x/y 不变、逐步降低 z，找第一个 IK 通过的高度。

    实测（2026-11-08）：右臂在 z=1.196 时，「竖直向下 + 内旋90°」无 IK 解；
    降到 z<=1.04 后可行。原因是手臂近乎伸直时腕部没有扭转余量。

    返回 (target_pose_or_None, chosen_z_or_None, tried_list)。
    """
    R, _x, _y, _z = build_target_rotation(chain, roll_deg)
    q = geo.matrix_to_quaternion(R)
    tried = []
    z = float(pose[2]) - float(step_m)
    while z >= float(floor_z) - 1e-9:
        tgt = [float(pose[0]), float(pose[1]), round(z, 6), *q]
        res = ad.check_end_effector_reachable(chain, tgt)
        tried.append((round(z, 4), bool(res.succeeded)))
        if res.succeeded:
            return tgt, round(z, 6), tried
        z -= float(step_m)
    return None, None, tried


def describe_plan(ad: GalbotAdapter, chain: str, roll_deg: float,
                  dz: float, tol_deg: float,
                  allow_drop: bool = False, drop_floor_z: float = 0.96,
                  drop_step_m: float = 0.03) -> dict:
    """读位姿 -> 算目标 -> IK 预检。返回一个 dict，不改任何东西。"""
    out = {"chain": chain, "ok": False, "reason": None}

    res = ad.get_end_effector_pose(chain)
    if not res.succeeded:
        out["reason"] = "读当前位姿失败：%s" % (res.message or res.code)
        return out

    pose = [float(v) for v in res.data["pose"]]
    R_cur = geo.quaternion_to_matrix(*pose[3:7])
    x_cur, y_cur, z_cur = R_cur[:, 0], R_cur[:, 1], R_cur[:, 2]

    R_tgt, x_tgt, y_tgt, z_tgt = build_target_rotation(chain, roll_deg)

    tilt_now = _angle_deg(x_cur, x_tgt)
    roll_now = _signed_angle_about(x_tgt, y_cur, y_tgt)

    target = [pose[0], pose[1], pose[2] + float(dz),
              *geo.matrix_to_quaternion(R_tgt)]

    already = (tilt_now <= tol_deg) and (abs(roll_now) <= tol_deg)
    reachable = ad.check_end_effector_reachable(chain, target)

    dropped = None
    drop_tried = []
    if (not reachable.succeeded) and allow_drop and abs(float(dz)) < 1e-9:
        new_target, chosen_z, drop_tried = find_lower_feasible(
            ad, chain, roll_deg, pose, drop_floor_z, drop_step_m)
        if new_target is not None:
            dropped = {"from_z": round(pose[2], 6), "to_z": chosen_z,
                       "drop_m": round(pose[2] - chosen_z, 6)}
            target = new_target
            reachable = ad.check_end_effector_reachable(chain, target)

    out.update({
        "ok": bool(reachable.succeeded),
        "already": already,
        "original_pose": pose,
        "target_pose": target,
        "tilt_now_deg": tilt_now,
        "roll_now_deg": roll_now,
        "approach_cur": x_cur,
        "closing_cur": y_cur,
        "long_cur": z_cur,
        "approach_tgt": x_tgt,
        "closing_tgt": y_tgt,
        "long_tgt": z_tgt,
        "reachable": bool(reachable.succeeded),
        "ik_message": reachable.message,
        "ik_status": (reachable.data or {}).get("motion_status")
        if isinstance(reachable.data, dict) else None,
        "dropped": dropped,
        "drop_tried": drop_tried,
    })
    if not reachable.succeeded:
        out["reason"] = "IK / 碰撞预检未通过：%s" % (reachable.message or "")
    return out


def print_plan(p: dict, roll_deg: float, tol_deg: float) -> None:
    chain = p["chain"]
    say("─" * 68)
    say("%s" % chain)
    if p.get("original_pose") is None:
        say("  ✗ %s" % p["reason"])
        return
    say("  当前位置   [%.4f, %.4f, %.4f] m" % tuple(p["original_pose"][:3]))
    say("  原始位姿   [%s]  ← 需要手动回退时用这 7 个数" %
        ", ".join("%.6f" % v for v in p["original_pose"]))
    say("  当前接近轴 %s   与竖直向下差 %.1f°" %
        (fmt_vec(p["approach_cur"]), p["tilt_now_deg"]))
    say("  当前闭合轴 %s" % fmt_vec(p["closing_cur"]))
    say("  目标接近轴 %s" % fmt_vec(p["approach_tgt"]))
    say("  目标闭合轴 %s （内旋 %.1f°）" % (fmt_vec(p["closing_tgt"]), roll_deg))
    say("  目标长轴   %s" % fmt_vec(p["long_tgt"]))
    if p.get("already"):
        say("  → 已符合目标（容差 %.1f°），无需动作" % tol_deg)
        return
    say("  需要调整   俯仰修正 %.1f°，绕接近轴自转 %+.1f°" %
        (p["tilt_now_deg"], p["roll_now_deg"]))
    if p.get("dropped"):
        d = p["dropped"]
        say("  位置调整   z %.4f → %.4f（降低 %.1f cm；原高度下该姿态无 IK 解）"
            % (d["from_z"], d["to_z"], d["drop_m"] * 100.0))
    if p["reachable"]:
        say("  IK 预检    ✓ 有解且通过碰撞预检")
    else:
        say("  IK 预检    ✗ %s" % (p.get("ik_message") or p.get("ik_status")))
        if p.get("drop_tried"):
            tried = p["drop_tried"]
            say("             已试降高 %d 个高度（%s），仍不可行"
                % (len(tried), ", ".join("%.2f" % t[0] for t in tried[:6])))
        say("             处置：用 --allow-drop 让它自动降高，")
        say("                   或用 move_ee_delta 先把末端降到 z≈0.98 再重跑")


def verify_after(ad: GalbotAdapter, p: dict, roll_deg: float, tol_deg: float) -> bool:
    """回读实测姿态，确认真的到位了。"""
    chain = p["chain"]
    res = ad.get_end_effector_pose(chain)
    if not res.succeeded:
        say("  验证失败：读不到位姿（%s）" % (res.message or res.code))
        return False
    pose = [float(v) for v in res.data["pose"]]
    R = geo.quaternion_to_matrix(*pose[3:7])
    x_cur, y_cur, z_cur = R[:, 0], R[:, 1], R[:, 2]

    tilt = _angle_deg(x_cur, p["approach_tgt"])
    roll = _signed_angle_about(p["approach_tgt"], y_cur, p["closing_tgt"])
    pos_err = math.sqrt(sum((pose[i] - p["target_pose"][i]) ** 2 for i in range(3)))

    say("  实测结果   接近轴 %s  与竖直向下差 %.2f°" % (fmt_vec(x_cur), tilt))
    say("             闭合轴 %s  自转残差 %+.2f°" % (fmt_vec(y_cur), roll))
    say("             长轴   %s  位置偏差 %.1f mm" % (fmt_vec(z_cur), pos_err * 1000.0))

    ok = True
    if tilt > tol_deg:
        say("  ✗ 俯仰未达标（%.2f° > %.1f°）" % (tilt, tol_deg))
        ok = False
    if abs(roll) > max(tol_deg, 3.0):
        if abs(abs(roll) - 180.0) <= max(tol_deg, 3.0):
            say("  ! 自转差 180°：两指互换，物理外观基本一致（夹爪对称性）")
        else:
            say("  ✗ 自转未达标（%+.2f°）" % roll)
            ok = False
    if pos_err > 0.005:
        say("  ✗ 位置漂移 %.1f mm 超过 5 mm" % (pos_err * 1000.0))
        ok = False
    if ok:
        say("  ✓ 达标")
    return ok


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="把 G1 末端摆成「接近轴竖直向下 + 夹爪内旋 N 度」",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--chain", choices=["right", "left", "both"], default="both",
                    help="哪条臂")
    ap.add_argument("--roll-deg", type=float, default=90.0,
                    help="绕接近轴的内旋角度；0=闭合轴朝前，90=朝身体中线")
    ap.add_argument("--dz", type=float, default=0.0,
                    help="目标高度微调（米），默认保持原高度")
    ap.add_argument("--allow-drop", action="store_true",
                    help="位置不可行时自动降低末端高度找可行点（保持 x/y）")
    ap.add_argument("--drop-floor", type=float, default=0.96,
                    help="自动降高的下限 z（米）")
    ap.add_argument("--drop-step", type=float, default=0.03,
                    help="自动降高的搜索步长（米）")
    ap.add_argument("--tol-deg", type=float, default=DEFAULT_TOL_DEG,
                    help="判定达标的角度容差")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不下发运动")
    ap.add_argument("--yes", action="store_true", help="跳过执行前确认")
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    chains: List[str] = ([CHAIN_ALIAS["right"], CHAIN_ALIAS["left"]]
                         if args.chain == "both" else [CHAIN_ALIAS[args.chain]])

    say("末端目标：接近轴(local +X) 竖直向下，夹爪内旋 %.1f°" % args.roll_deg)
    say("竖直向下 = local +X -> base [0,0,-1]；内旋角 0=闭合轴朝前，90=朝身体中线")
    say("正在初始化 SDK…")

    try:
        ad = GalbotAdapter(enable_real_control=True, real_authorized=True)
    except Exception as exc:  # noqa: BLE001
        say("初始化失败：%s: %s" % (type(exc).__name__, exc))
        return 1

    plans = []
    for chain in chains:
        plan = describe_plan(ad, chain, args.roll_deg, args.dz, args.tol_deg,
                             allow_drop=args.allow_drop,
                             drop_floor_z=args.drop_floor,
                             drop_step_m=args.drop_step)
        plans.append(plan)
        print_plan(plan, args.roll_deg, args.tol_deg)

    todo = [p for p in plans if p.get("original_pose") is not None
            and not p.get("already")]
    runnable = [p for p in todo if p.get("reachable")]
    blocked = [p for p in todo if not p.get("reachable")]

    say("─" * 68)
    say("汇总：%d 条臂需要动作，其中 %d 条 IK 通过，%d 条被拦下"
        % (len(todo), len(runnable), len(blocked)))
    for p in blocked:
        say("  ✗ %s 未通过预检，不会下发（%s）" % (p["chain"], p.get("reason")))

    if not runnable:
        say("没有可以执行的臂。")
        return 0

    if args.dry_run:
        say("--dry-run：未下发任何运动命令。")
        return 0

    say("")
    say("提醒：纯姿态旋转会让整条手臂重构。请确认周围无人、急停在手边。")
    if not args.yes:
        try:
            answer = input("确认执行 %d 条臂的姿态调整？(输入 y 继续)："
                           % len(runnable)).strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = ""
        if answer != "y":
            say("已取消，未下发任何运动命令。")
            return 0

    results = []
    for p in runnable:
        chain = p["chain"]
        say("")
        say(">>> 正在调整 %s …" % chain)
        try:
            res = ad.move_ee_to(
                chain, p["target_pose"],
                straight_line=False,        # 纯旋转，不强制直线
                enable_collision_check=True,
                timeout_s=20.0,
            )
        except Exception as exc:  # noqa: BLE001
            say("  下发时抛错：%s: %s；机器人是否已动未知，请现场确认"
                % (type(exc).__name__, exc))
            results.append((chain, False))
            continue

        if not res.succeeded:
            say("  运动未成功：%s（状态 %s）"
                % (res.message or res.code,
                   (res.data or {}).get("motion_status")
                   if isinstance(res.data, dict) else "?"))
            results.append((chain, False))
            continue

        say("  运动返回成功，回读校验…")
        ok = verify_after(ad, p, args.roll_deg, args.tol_deg)
        results.append((chain, ok))

    say("")
    say("=" * 68)
    for chain, ok in results:
        say("  %s  %s" % ("✓" if ok else "✗", chain))
    all_ok = all(ok for _c, ok in results)
    say("全部达标。" if all_ok else "有未达标项，请查看上面的实测数值。")
    return 0 if all_ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
