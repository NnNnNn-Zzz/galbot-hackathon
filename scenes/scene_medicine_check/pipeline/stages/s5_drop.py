"""伸向垃圾桶 → 松开夹爪 → 收手。"""

from __future__ import annotations

import logging
import time

from .. import outcome, tuning
from ..contracts import Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.drop")


@register_stage("drop")
class Drop(Stage):
    stage_id = "drop"

    def describe(self) -> str:
        return "伸向垃圾桶 → 松开夹爪 → 收手"

    def run(self, ctx: Context, io) -> StageResult:
        pre = self.params.get("pre_drop_pose", tuning.PRE_DROP_POSE)
        drop = self.params.get("drop_pose", tuning.DROP_POSE)
        settle_s = float(self.params.get("settle_s", 0.8))

        ret = io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S)
        if not ret.get("success"):
            return outcome(ret, "移动到投放预备位姿失败")

        ret = io.move_line_to(drop)
        if not ret.get("success"):
            return outcome(ret, "下降到投放位姿失败")

        # 放手：夹爪张开到最大，药盒自由落体进桶
        ret = io.set_gripper(tuning.GRIPPER_OPEN_M, tuning.GRIPPER_SIDE,
                             tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "夹爪松开失败，药盒可能还夹着")

        if settle_s > 0:
            time.sleep(settle_s)     # 等药盒落桶

        ret = io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S)
        if not ret.get("success"):
            logger.warning("投完后收手失败，交由 finally 归位兜底")

        return ok("已松开夹爪，药盒投入垃圾桶", released=True)
