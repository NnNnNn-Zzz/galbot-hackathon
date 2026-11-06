"""回归初始位姿。

两段：
  a) 手臂回初始关节位、夹爪保持张开 —— 官方 example6 的关节初值
  b) 底盘回起点 —— 用起始里程计位姿与当前位姿做逆运算，再相对移动到原位
底盘部分依赖 get_odom()，无里程计时只做手臂归位并告警。

配置里标 must_run + non_fatal：前序失败也要执行归位；它自己的成败不改变整体结论
（与改动前的行为一致：原实现块 8 不计入 failed）。
"""

from __future__ import annotations

import logging
import math
from typing import Any, Dict, List

from .. import code_of, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.return_home")


def yaw_from_quat(q: Any) -> float:
    """四元数 [x, y, z, w] 取 yaw（弧度），只关心绕 z 轴。"""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(angle: float) -> float:
    """把角度归一到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


@register_stage("return_home")
class ReturnHome(Stage):
    stage_id = "return_home"

    def describe(self) -> str:
        return "手臂回初始关节位 + 底盘按里程计逆运算回原点"

    def run(self, ctx: Context, io) -> StageResult:
        arm_ok = bool(io.set_joints(tuning.CARRY_JOINT_POSITIONS, tuning.CARRY_JOINT_NAMES,
                                    tuning.ARM_SPEED_RAD_S).get("success"))

        base_ok = False
        detail = ""
        cur = io.get_odom()
        start_odom = ctx.start_odom

        if not cur.get("success") or not cur.get("odom"):
            detail = "无里程计数据，底盘未回原点"
            logger.warning(detail)
        elif not start_odom:
            detail = "未记录起始位姿，底盘未回原点"
            logger.warning(detail)
        else:
            sp = start_odom["position"]
            sq = start_odom["orientation"]
            cp = cur["odom"]["position"]
            cq = cur["odom"]["orientation"]
            dx, dy = sp[0] - cp[0], sp[1] - cp[1]
            dyaw = wrap_pi(yaw_from_quat(sq) - yaw_from_quat(cq))
            # 把 odom 系的位移转到当前 base 系：绕 -yaw 旋转
            yaw_cur = yaw_from_quat(cq)
            c, s = math.cos(yaw_cur), math.sin(yaw_cur)
            fx = c * dx + s * dy
            fy = -s * dx + c * dy
            ret = io.move_to_pose(fx, fy, dyaw)
            base_ok = bool(ret.get("success"))
            detail = (f"底盘回原点 {'成功' if base_ok else '失败'}："
                      f"Δ=({fx:.2f}, {fy:.2f})m, Δyaw={math.degrees(dyaw):.1f}°")
            logger.info(detail)
            ctx.flags["home.delta"] = {"dx": fx, "dy": fy, "dyaw": dyaw}

        if arm_ok and base_ok:
            return ok(detail or "手臂已回初始位", arm_ok=True, base_ok=True)

        reasons = []
        if not arm_ok:
            reasons.append("手臂未回初始位")
        if not base_ok:
            reasons.append(detail or "底盘未回原点")
        return failed("；".join(reasons) or "归位未完成",
                      code=Code.NOT_VERIFIED, arm_ok=arm_ok, base_ok=base_ok)
