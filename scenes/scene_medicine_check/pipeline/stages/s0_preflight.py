"""步骤 0：记录起始里程计位姿（回原点要用）。"""

from __future__ import annotations

import logging

from .. import code_of
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
