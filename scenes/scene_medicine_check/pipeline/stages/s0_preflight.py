"""步骤 0：记录起始里程计位姿（回原点要用）。"""

from __future__ import annotations

import logging

from .. import code_of, outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.preflight")


@register_stage("preflight")
class Preflight(Stage):
    """非致命：读不到里程计也照常开跑，只是最后只能做手臂归位。"""

    stage_id = "preflight"

    def describe(self) -> str:
        return "记录起始里程计位姿（读不到也不阻塞）"

    def run(self, ctx: Context, io) -> StageResult:
        ret = io.get_odom()
        if ret.get("success") and ret.get("odom"):
            ctx.start_odom = ret["odom"]
            pos = ctx.start_odom.get("position")
            logger.info("起始位姿已记录: position=%s", pos)
            return ok("已记录起始里程计位姿", position=pos, odom_available=True)

        logger.warning("无里程计数据（%s），最后回原点将只做手臂归位", ret.get("message"))
        return failed("无里程计数据，最后回原点将只做手臂归位",
                      code=code_of(ret) if ret.get("code") else Code.NOT_VERIFIED,
                      odom_available=False)


@register_stage("initialize_pose")
class InitializePose(Stage):
    """抓取前显式进入固定关节位，消除上一次实验遗留姿态的影响。

    对齐官方 tutorials/example6_pick_and_place.py 的 move_to_initial_pose：
    先动腿，再动头部和双臂。笛卡尔预抓取依赖当前关节状态，不能从任意
    人工拖动位、上次失败位或上次收臂位直接开始。
    """

    stage_id = "init_pose"

    def describe(self) -> str:
        return "先到固定任务初始位（先腿部，再头部和双臂），再开始抓取"

    def run(self, ctx: Context, io) -> StageResult:
        plan = [
            ("init_legs", tuning.LEG_JOINT_POSITIONS, tuning.LEG_JOINT_NAMES,
             "腿部初始位移动失败"),
            ("init_upper_body", tuning.UPPER_JOINT_POSITIONS, tuning.UPPER_JOINT_NAMES,
             "头部和双臂初始位移动失败"),
        ]
        for name, positions, names, err in plan:
            ret = io.set_joints(positions, names, tuning.ARM_SPEED_RAD_S)
            if not ret.get("success"):
                return outcome(ret, f"{err}（{name}），禁止继续抓取", sub_step=name)
        ctx.flags["init_pose.done"] = True
        return ok("已到固定任务初始位，右臂使用调宽后的安全抱持角",
                  right_arm_joint2=tuning.UPPER_JOINT_POSITIONS[10])
