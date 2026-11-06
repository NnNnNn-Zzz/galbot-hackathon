"""直行固定距离，按安全上限自动分段。"""

from __future__ import annotations

import logging

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.drive")


@register_stage("drive")
class Drive(Stage):
    stage_id = "drive"

    def _distance(self, ctx: Context) -> float:
        return float(self.params.get("distance_cm", ctx.params.get("drive_cm",
                                                                  tuning.DRIVE_DISTANCE_CM)))

    def describe(self) -> str:
        return f"直行 {self.params.get('distance_cm', tuning.DRIVE_DISTANCE_CM)} cm（按安全上限自动分段）"

    def run(self, ctx: Context, io) -> StageResult:
        total = self._distance(ctx)
        segments = tuning.split_distance(total)
        moved = 0.0
        for i, seg in enumerate(segments, 1):
            ret = io.move_forward(seg)
            if not ret.get("success"):
                return outcome(
                    ret,
                    f"第 {i}/{len(segments)} 段直行失败（已走 {moved:.0f} cm）",
                    moved_cm=moved, segment=i, total_segments=len(segments))
            moved += seg
            logger.info("直行段 %d/%d 完成：%.0f cm", i, len(segments), seg)
        ctx.flags["drive.moved_cm"] = moved
        return ok(f"直行 {moved:.0f} cm 完成（分 {len(segments)} 段）",
                  moved_cm=moved, segments=len(segments))
