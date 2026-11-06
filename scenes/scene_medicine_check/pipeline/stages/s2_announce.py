"""步骤 2：播报过期提示并停顿等人回应。

播报走 io.speak()（封装层内部应是「预合成 PCM + write_audio_stream_output」）。
SDK 本身没有 TTS，只有 PCM 播放通道，所以语音素材必须提前生成。

这是**可替换点 #2**：以后要接"人说了好才继续"，在这里换成带 ASR 的实现，
若 3 秒内没听到回应就返回 unknown（不可判定），而不是硬等。
"""

from __future__ import annotations

import logging
import time

from .. import tuning
from ..contracts import Context, Stage, StageResult, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.announce")


@register_stage("announce")
class Announce(Stage):
    """播报失败不阻塞流程（现场听不到声音也必须能继续），但会如实记 played=False。"""

    stage_id = "announce"

    def describe(self) -> str:
        name = self.params.get("item_name") or "药品名"
        tpl = self.params.get("template") or tuning.ANNOUNCE_TEMPLATE
        return f"播报「{tpl.format(name=name)}」后停顿 {self.params.get('pause_s', tuning.ANNOUNCE_PAUSE_S)}s"

    def run(self, ctx: Context, io) -> StageResult:
        template = (self.params.get("template")
                    or ctx.params.get("announce_template")
                    or tuning.ANNOUNCE_TEMPLATE)
        name = (ctx.params.get("item_name")
                or self.params.get("item_name")
                or tuning.ITEM_NAME)
        try:
            pause_s = float(self.params.get("pause_s", ctx.params.get("pause_s",
                                                                     tuning.ANNOUNCE_PAUSE_S)))
        except (TypeError, ValueError):
            pause_s = tuning.ANNOUNCE_PAUSE_S

        text = template.format(name=name)
        logger.info("[播报] %s", text)
        ret = io.speak(text)
        played = bool(ret.get("success"))
        if not played:
            logger.warning("播报未生效（%s），现场将听不到声音，仅日志可见", ret.get("message"))

        if pause_s > 0:
            time.sleep(pause_s)     # 场景描述：停顿五秒等人说「好」

        ctx.flags["announce.text"] = text
        return ok(f"已播报「{text}」，停顿 {pause_s}s", text=text, played=played)
