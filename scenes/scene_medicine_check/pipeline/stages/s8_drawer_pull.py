"""步骤 3+4：抓住抽屉把手 → 竖直后拉出外壳。

和基线 s1_grasp 的区别：
  s1 抓的是自由放在桌上的药盒，终点位姿是"标定好的绝对坐标"，可以用
  set_arm_pose + move_line_to 走。
  抽屉把手则必须"先咬住再拉"，而且拉的这一下是**沿着抽屉滑轨方向**的直线。
  因此拆成两段：
    a) 咬住：示教点 P2_grasp 的关节角回放（关节空间，箱外自由空间，稳）
    b) 后拉：从咬住时的**实测末端位姿**出发，沿 -x（base_link 前方为 +x）
            分段平移 pull_cm，每段后 sleep —— 保证是直线，且中途可停

为什么不直接回放到 P3_pull_done（拉完位）：
  队友实测过：同一个末端位姿可以有多种关节解（TCP 差 0.036 mm、joint3 差 18°）。
  如果咬住姿态和拉完姿态的解分支不同，关节插值会把夹爪绕着把手转半圈，
  抽屉直接被扭歪。**只能用笛卡尔直线，把姿态锁死在咬住那一刻。**
 P3_pull_done 因此只作为"拉了多少距离"的校验参照，不作为回放目标。

⚠️ 力：拉抽屉靠夹爪的 effort（tuning.GRIPPER_EFFORT_N = 10 N）。
   10 N 对滑轨静摩擦可能不够，这是**只能实测**的量，代码无法预判。
   若拉不到位，move_line_to 会超时或返回失败，本 stage 如实报 failed。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage
from ..teach import _safe_read, load_point as _load_point

logger = logging.getLogger("scene.medicine.stage.drawer_pull")

DEG = 57.29577951308232




@register_stage("drawer_pull")
class DrawerPull(Stage):
    stage_id = "drawer_pull"

    def describe(self) -> str:
        bite = self.params.get("bite_point", "P2_grasp")
        pull = float(self.params.get("pull_cm", 30.0))
        return f"回放到 {bite} 咬住把手 → 沿 -x 直线后拉 {pull:.0f} cm"

    def run(self, ctx: Context, io) -> StageResult:
        teach_file = str(self.params.get("file", "pipeline/configs/teach_drawer.json"))
        bite_pt = str(self.params.get("bite_point", "P2_grasp"))
        pull_cm = float(self.params.get("pull_cm", 30.0))
        step_cm = float(self.params.get("step_cm", 5.0))
        settle_s = float(self.params.get("settle_s", 0.4))
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))
        close_m = float(self.params.get("close_m", tuning.DRAWER_GRIPPER_CLOSE_M))
        open_m = float(self.params.get("open_m", tuning.GRIPPER_OPEN_M))
        chain = str(self.params.get("chain", "right_arm"))
        # 已在示教时咬住了把手（P2_grasp 采的是"夹爪贴着把手、还没闭爪"的姿态）→ 这里负责闭爪
        do_close = bool(self.params.get("close_here", True))

        # ---------------- a) 咬住 ----------------
        rec = _load_point(teach_file, bite_pt)
        names = rec.get("joint_names")
        pos = rec.get("joint_positions")
        if not names or not pos:
            return failed(f"取不到示教点 {bite_pt} 的关节角（{teach_file}）。"
                          f"抄不了姿态就无法保证咬住把手，请先 teach_record",
                          code=Code.DEP_MISSING, file=teach_file, point=bite_pt)

        # 先张爪，再过去 —— 顺序反了会把把手撞歪
        ret = io.set_gripper(open_m, side, tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "进攻前张开夹爪失败")

        ret = io.set_joints(pos, names, float(self.params.get("speed_rad_s", tuning.ARM_SPEED_RAD_S)))
        if not ret.get("success"):
            return outcome(ret, f"回放到咬住姿态 {bite_pt} 失败")
        if settle_s > 0:
            time.sleep(settle_s)

        # 咬住时的实测位姿 —— 后续直线后拉全部以它为原点
        cur = _safe_read(io, "get_ee_pose", chain=chain)
        if not cur.get("success") or len(cur.get("pose") or []) != 7:
            return failed(f"咬住后读不到末端位姿（{cur.get('message')}），"
                          f"无法规划直线后拉 —— 不发动作，请人工确认",
                          code=Code.UNDETERMINED)
        bite = [float(v) for v in cur["pose"]]
        ctx.flags["pull.bite_pose"] = bite
        logger.info("咬住位置：x=%.3f y=%.3f z=%.3f", bite[0], bite[1], bite[2])

        # ---------------- 闭爪 ----------------
        if do_close:
            ret = io.set_gripper(close_m, side, tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
            if not ret.get("success"):
                return outcome(ret, "闭爪咬住把手失败")
            if settle_s > 0:
                time.sleep(settle_s)
            gw = _safe_read(io, "get_gripper_width", side=side)
            if gw.get("success"):
                ctx.flags["pull.grip_width"] = gw.get("width")
                logger.info("咬住后夹爪宽度 %.3f m（小于 close_m=%.3f 说明碰到把手了）",
                            float(gw["width"]), close_m)
            else:
                logger.warning("读不到夹爪宽度（%s），是否咬住把手未确证", gw.get("message"))

        # ---------------- b) 直线后拉 ----------------
        if pull_cm <= 0:
            return ok("仅咬住，后拉量 0（按配置）", bite_pose=bite, pulled_cm=0.0)

        n = max(1, int(round(abs(pull_cm) / step_cm)))
        step = pull_cm / n
        log: List[Dict[str, Any]] = []
        done = 0
        for i in range(1, n + 1):
            x = bite[0] - step * i / 100.0
            target = [x, bite[1], bite[2], bite[3], bite[4], bite[5], bite[6]]
            ret = io.move_line_to(target)
            row = {"seg": i, "x": round(x, 4), "ok": bool(ret.get("success"))}
            log.append(row)
            logger.info("后拉第 %d/%d 段 → x=%.3f : %s", i, n, x,
                        "ok" if ret.get("success") else ret.get("message"))
            if not ret.get("success"):
                ctx.flags["pull.log"] = log
                # 拉不动 = 力不够 或 抽屉到底了。这是 failed（位姿仍可读，能归位）
                return failed(
                    f"后拉第 {i} 段失败，已拉 {log[-2]['x'] - bite[0] if len(log) > 1 else 0:.3f} m。"
                    f"大概率是夹爪 effort（{tuning.GRIPPER_EFFORT_N} N）不够拉开滑轨静摩擦，"
                    f"或已到行程末端",
                    code=Code.STAGE_FAILED, bite_pose=bite, log=log)
            done = i
            if settle_s > 0:
                time.sleep(settle_s)

        ctx.flags["pull.log"] = log
        pulled = step * done / 100.0
        ctx.flags["pull.pulled_m"] = pulled

        end = _safe_read(io, "get_ee_pose", chain=chain)
        if end.get("success"):
            logger.info("后拉结束位置：x=%.3f y=%.3f z=%.3f（Δx=%.3f）",
                        end["pose"][0], end["pose"][1], end["pose"][2], end["pose"][0] - bite[0])
            ctx.flags["pull.end_pose"] = end["pose"]

        return ok(f"已后拉 {pulled*100:.0f} cm", pulled_m=round(pulled, 4),
                  bite_pose=bite, segments=done)


@register_stage("drawer_release_pull")
class DrawerReleasePull(Stage):
    """张爪 → 再后拉一小段，彻底退出抽屉外壳（避免回位时把抽屉带回去）。

    参数 `extra_cm` 为负时表示**向前推**（回到抽屉口方向，对齐未拉出时的位姿）。
    这一条在实验 2 的收尾里很有用：拉完 30 cm 后手臂停在 x≈0.45，
    而 ALIGN_handle 在 x=0.752 —— 直接关节回放 ALIGN 要转 207°，被安全跨度拦。
    改用笛卡尔沿 +x 推回，末端走直线、姿态锁死，规划器不需要猜关节构型。
    """

    stage_id = "drawer_release_pull"

    def describe(self) -> str:
        d = float(self.params.get("extra_cm", 8.0))
        verb = "后拉" if d >= 0 else "前推"
        return f"张爪 → 再{verb} {abs(d):.0f} cm"

    def run(self, ctx: Context, io) -> StageResult:
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))
        extra_cm = float(self.params.get("extra_cm", 8.0))
        step_cm = float(self.params.get("step_cm", 4.0))
        settle_s = float(self.params.get("settle_s", 0.4))
        chain = str(self.params.get("chain", "right_arm"))

        ret = io.set_gripper(tuning.GRIPPER_OPEN_M, side,
                             tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "张爪失败，可能仍夹着把手")
        if settle_s > 0:
            time.sleep(settle_s)

        cur = _safe_read(io, "get_ee_pose", chain=chain)
        if not cur.get("success") or len(cur.get("pose") or []) != 7:
            return failed(f"张爪后读不到末端位姿（{cur.get('message')}）", code=Code.UNDETERMINED)
        here = [float(v) for v in cur["pose"]]
        ctx.flags["release.start_pose"] = here

        if abs(extra_cm) < 1e-6:
            return ok("已张爪（后退量 0，按配置）", extra_cm=0.0, start_pose=here)

        n = max(1, int(round(abs(extra_cm) / step_cm)))
        step = extra_cm / n
        log: List[Dict[str, Any]] = []
        for i in range(1, n + 1):
            # step 的符号决定方向：负 = 沿 +x 推回去（抽屉口方向）
            target = [here[0] - step * i / 100.0, here[1], here[2],
                      here[3], here[4], here[5], here[6]]
            ret = io.move_line_to(target)
            log.append({"seg": i, "x": round(target[0], 4), "ok": bool(ret.get("success"))})
            if not ret.get("success"):
                ctx.flags["release.log"] = log
                return outcome(ret, f"{'前推' if step < 0 else '后拉'}第 {i} 段失败")
            if settle_s > 0:
                time.sleep(settle_s)

        ctx.flags["release.log"] = log
        end = _safe_read(io, "get_ee_pose", chain=chain)
        if end.get("success"):
            ctx.flags["release.end_pose"] = end["pose"]
            dx = float(end["pose"][0]) - here[0]
            logger.info("张爪后移动 %.1f cm（Δx=%+.3f m），现在 x=%.3f",
                        extra_cm, dx, float(end["pose"][0]))

        return ok(f"已张爪并{'前推' if extra_cm < 0 else '后拉'} {abs(extra_cm):.0f} cm",
                  extra_cm=extra_cm, segments=n, end_pose=end.get("pose"))
