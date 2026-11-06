"""原地转向。参数化，配置里可以出现两次（转身 180° / 到桌前右转 30°）。

角度取值优先级：场景入口 params 里同名键 > 配置里的 default_deg。
这样现场 `run({"turn_deg": 175})` 依然能临时改角度，不用动配置。
"""

from __future__ import annotations

import logging

from .. import outcome
from ..contracts import Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.turn")


@register_stage("turn")
class Turn(Stage):
    """deg 为负表示右转。"""

    stage_id = "turn"

    def _deg(self, ctx: Context) -> float:
        key = self.params.get("deg_key")
        if key and ctx.params.get(key) is not None:
            return float(ctx.params[key])
        return float(self.params.get("default_deg", self.params.get("deg", 180.0)))

    def describe(self) -> str:
        label = self.params.get("label") or "转向"
        return f"{label} {self.params.get('default_deg', 180.0)}°"

    def run(self, ctx: Context, io) -> StageResult:
        deg = self._deg(ctx)
        label = self.params.get("label") or "转向"
        ret = io.turn(deg)
        if not ret.get("success"):
            return outcome(ret, f"{label}失败")
        actual = ret.get("actual_angle_deg", deg)
        return ok(f"已{label} {abs(float(actual)):.0f}°", requested_deg=deg, actual_deg=actual)
