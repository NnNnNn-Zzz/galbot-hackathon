"""步骤 10：在示教点 P4 投放（示教版 drop）。

和基线 s5_drop 的区别：基线用 tuning 里两个硬编码位姿（PRE_DROP / DROP），
这里直接用示教点 P4_drop 的关节角回放到位，然后张爪。
示教点本身就采在"桶口正上方" —— 现场摆桶时对准即可，不需要再算坐标。

⚠️ 张爪后药盒靠重力落桶，如果 P4 采得太高，药盒会弹出来；太低会碰桶沿。
   现场判断标准：夹爪底部离桶沿 5–10 cm。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage
from ..teach import _safe_read, load_point as _load_point

logger = logging.getLogger("scene.medicine.stage.teach_drop")




@register_stage("teach_drop")
class TeachDrop(Stage):
    stage_id = "teach_drop"

    def describe(self) -> str:
        pt = self.params.get("pre_point", "P4_drop")
        return f"回放到投放点 {pt} → 张爪松手"

    def run(self, ctx: Context, io) -> StageResult:
        teach_file = str(self.params.get("file", "pipeline/configs/teach_drawer.json"))
        pt = str(self.params.get("pre_point", "P4_drop"))
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))
        settle_s = float(self.params.get("settle_s", 0.8))
        speed = float(self.params.get("speed_rad_s", tuning.ARM_SPEED_RAD_S))

        rec = _load_point(teach_file, pt)
        names = rec.get("joint_names")
        pos = rec.get("joint_positions")
        if not names or not pos:
            return failed(f"取不到投放点 {pt} 的关节角（{teach_file}），"
                          f"不接受硬编码坐标兜底 —— 请先示教该点",
                          code=Code.DEP_MISSING, file=teach_file, point=pt)

        ret = io.set_joints(pos, names, speed)
        if not ret.get("success"):
            return outcome(ret, f"移动到投放点 {pt} 失败")

        ret = io.set_gripper(tuning.GRIPPER_OPEN_M, side,
                             tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "夹爪松开失败，药盒可能还夹着")

        if settle_s > 0:
            time.sleep(settle_s)

        # 松手后复核：宽度应该接近全开
        gw = _safe_read(io, "get_gripper_width", side=side)
        width = gw.get("width") if gw.get("success") else None
        if width is not None and width < tuning.GRIPPER_OPEN_M * 0.8:
            logger.warning("松手后夹爪宽度只有 %.3f m，可能没完全张开或仍夹着东西",
                           float(width))

        # 是否真的放下了？有 is_holding 就采信它，没有就如实记 None（不可判定）
        holding = None
        try:
            holding = io.is_holding()
        except Exception:  # noqa: BLE001 - 该接口在部分后端缺失，不影响投放本身
            holding = None
        ctx.flags["drop.point"] = pt
        ctx.flags["drop.gripper_width"] = width
        ctx.flags["drop.holding_after"] = holding
        ctx.flags["drop.done"] = bool(holding is False) if holding is not None else None

        ctx.picked = None
        return ok(f"已在 {pt} 松手投放", point=pt, gripper_width=width,
                  holding_after=holding, released=(holding is False) if holding is not None else None)
