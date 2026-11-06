"""左移固定距离（底盘侧移，不转身）。替代旧基线的「转身 180° + 直行 3m」。

base_link 系：左 = +y。侧移过程中朝向保持不变，
后续「右转 30° → 投放」的相对几何因此与旧流程不同，投放位姿需重新示教。

距离取值优先级：场景入口 params 的 shift_m > 配置里的 distance_m > tuning.SHIFT_LEFT_M。
真机注意：侧移要求底盘支持横向运动（全向轮/麦克纳姆轮），
差速底盘实现不了就得改回「转 90°→前进→转回」，属于真机待确认项。
"""

from __future__ import annotations

import logging

from .. import outcome, tuning
from ..contracts import Context, Stage, StageResult, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.shift")


@register_stage("shift_left")
class ShiftLeft(Stage):
    stage_id = "shift_left"

    def _distance(self, ctx: Context) -> float:
        return float(self.params.get("distance_m",
                                     ctx.params.get("shift_m", tuning.SHIFT_LEFT_M)))

    def describe(self) -> str:
        return f"左移 {self.params.get('distance_m', tuning.SHIFT_LEFT_M)} m（侧移，不转身）"

    def run(self, ctx: Context, io) -> StageResult:
        d = self._distance(ctx)
        ret = io.move_left(d)
        if not ret.get("success"):
            return outcome(ret, "左移失败")
        actual = float(ret.get("actual_distance_m", d))
        ctx.flags["shift.moved_m"] = actual
        logger.info("左移完成：%.2f m", actual)
        return ok(f"左移 {actual:.2f} m 完成", moved_m=actual)
