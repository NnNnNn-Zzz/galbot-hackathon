"""robot_tools/audio.py —— 语音提示（播放赛前合成的 PCM）。

════════════════════════════════════════════════════════════════════════════
⚠️ GalbotSDK **没有 TTS**，只有音频输出流：
    write_audio_stream_output(chunk: bytes, stream_id) -> bool
    格式固定：16000 Hz / 16-bit little-endian / 单声道
    官方示例按 2560 字节（= 80 ms）分块，块间 sleep 0.05
所以文案必须**赛前合成**成 ``.pcm`` 随仓库提交（见下方合成命令）。

合成（Windows，输出直接就是 16k/16bit/mono，不需要 ffmpeg）：
    Add-Type -AssemblyName System.Speech
    $fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
        16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
        [System.Speech.AudioFormat.AudioChannel]::Mono)
    $s = New-Object System.Speech.Synthesis.SpeechSynthesizer
    $s.SetOutputToWaveFile("cup_unsafe.wav", $fmt); $s.Speak("检测到杯子靠近桌沿，请注意安全。")
    $s.Dispose()
    # wav → 裸 pcm：去掉 44 字节头
    $b = [IO.File]::ReadAllBytes("cup_unsafe.wav")
    [IO.File]::WriteAllBytes("cup_unsafe.pcm", $b[44..($b.Length-1)])

════════════════════════════════════════════════════════════════════════════
两个诚实性设计
════════════════════════════════════════════════════════════════════════════
1. 素材缺失时**不静默失败**：退回"把文案打到终端"，并在 data 里写
   ``mode="text_fallback"``，让调用方知道没真的出声。
2. ``write_audio_stream_output`` 返回 True 只代表"写入并下发播放任务"，
   不代表播完。所以要按 ``len(bytes)/32000`` 秒估算时长再返回，
   否则机器人会一边说话一边动。
"""

from __future__ import annotations

import logging
import os
import time
import typing

from contracts import ErrorCode, RunMode, ToolError, ToolResult, ok
from contracts import failed

LOG = logging.getLogger(__name__)

__all__ = ["AudioTool", "PROMPTS", "PROMPT_TEXTS"]

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ASSET_DIR = os.path.join(_PROJECT_ROOT, "assets", "audio")

#: prompt_id -> 音频文件名（白名单：模型不能传任意路径）
PROMPTS: dict[str, str] = {
    "cup_unsafe": "cup_unsafe.pcm",
    "moving": "moving.pcm",
    "safe": "safe.pcm",
    "need_help": "need_help.pcm",
}

#: 文案。素材缺失时用它做文本兜底，也用于日志与文档
PROMPT_TEXTS: dict[str, str] = {
    "cup_unsafe": "检测到有杯子靠近桌子边缘，请注意安全。",
    "moving": "正在把杯子移到安全位置。",
    "safe": "杯子已移到安全位置，请放心。",
    "need_help": "我无法安全移动这个杯子，请人工协助。",
}

#: 16 kHz × 16 bit × mono = 32000 字节/秒
_BYTES_PER_SECOND = 32000
_DEFAULT_CHUNK = 2560  # 80 ms，官方示例的块大小


class AudioTool:
    """播放预录语音。只接受白名单 prompt_id。"""

    TOOL_NAMES = ("speak",)

    def __init__(
        self,
        adapter: typing.Any,
        *,
        asset_dir: str | os.PathLike[str] | None = None,
        volume: float = 80.0,
        chunk_size: int = _DEFAULT_CHUNK,
        wait_until_done: bool = True,
        mode: RunMode | str | None = None,
    ) -> None:
        self.adapter = adapter
        self.asset_dir = os.fspath(asset_dir) if asset_dir else DEFAULT_ASSET_DIR
        self.volume = float(volume)
        self.chunk_size = max(1, int(chunk_size))
        self.wait_until_done = bool(wait_until_done)
        self.mode = RunMode(mode) if mode is not None else self._infer_mode(adapter)
        self._volume_set = False

    @staticmethod
    def _infer_mode(adapter: typing.Any) -> RunMode:
        try:
            return RunMode(getattr(adapter, "mode"))
        except ValueError as exc:
            raise ValueError("adapter.mode 必须是 'mock' 或 'real'") from exc

    # ------------------------------------------------------------------ #
    def speak(self, prompt_id: str) -> ToolResult:
        """播放一条预录提示。返回时音频**已经播完**（或明确说明了原因）。"""
        try:
            name = self._resolve(prompt_id)
        except ToolError as exc:
            return exc.to_result(self.mode)

        path = os.path.join(self.asset_dir, name)
        text = PROMPT_TEXTS[prompt_id]
        if not os.path.exists(path):
            # 不假装播过：如实标注 fallback
            LOG.warning("语音素材缺失：%s（改用文本兜底）", path)
            return ok(
                f"[无音频素材，文本兜底] {text}",
                self.mode,
                data={
                    "prompt_id": prompt_id,
                    "text": text,
                    "mode": "text_fallback",
                    "asset": None,
                    "played": False,
                },
            )

        try:
            payload = open(path, "rb").read()
        except OSError as exc:
            return failed(ErrorCode.NOT_READY, f"读取 {name} 失败：{exc}", self.mode)

        stream_id = f"prompt_{prompt_id}"
        try:
            if not self._volume_set:
                self.adapter.set_volume(self.volume)
                self._volume_set = True
            for offset in range(0, len(payload), self.chunk_size):
                chunk = payload[offset : offset + self.chunk_size]
                if not self.adapter.write_audio_stream_output(chunk, stream_id):
                    return failed(
                        ErrorCode.SDK_ERROR,
                        f"第 {offset // self.chunk_size + 1} 块写入失败",
                        self.mode,
                        data={"prompt_id": prompt_id, "bytes_written": offset},
                    )
                time.sleep(0.05)
        except Exception as exc:  # noqa: BLE001
            return failed(ErrorCode.SDK_ERROR, f"播放 {name} 时出错：{exc}", self.mode)

        duration = len(payload) / _BYTES_PER_SECOND
        if self.wait_until_done:
            # 写成功 ≠ 播完：等它真的播完，否则机器人会边说边动
            time.sleep(min(duration + 0.2, 30.0))
        return ok(
            text,
            self.mode,
            data={
                "prompt_id": prompt_id,
                "text": text,
                "mode": "audio",
                "asset": name,
                "played": True,
                "bytes": len(payload),
                "duration_s": round(duration, 2),
            },
        )

    # ------------------------------------------------------------------ #
    @staticmethod
    def _resolve(prompt_id: str) -> str:
        if not isinstance(prompt_id, str) or not prompt_id.strip():
            raise ToolError(ErrorCode.INVALID_ARGUMENT, "prompt_id 必须是非空字符串")
        key = prompt_id.strip()
        if key not in PROMPTS:
            raise ToolError(
                ErrorCode.INVALID_ARGUMENT,
                f"未登记的提示语 {prompt_id!r}",
                {"legal_prompt_ids": sorted(PROMPTS)},
            )
        return PROMPTS[key]
