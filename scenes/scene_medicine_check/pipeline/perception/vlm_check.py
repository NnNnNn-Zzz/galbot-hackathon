"""下爪前视觉校验：拍一张照片，问多模态大模型「目标盒子在不在点上、偏多少」。

设计红线（务必遵守，否则会把 VLM 变成危险源）：
  1. **只读不写**：本模块不发任何硬件指令，只吃图片路径、吐结构化判断；
  2. **建议权无执行权**：输出的偏移量必须经 `clamp_correction()` 限幅，
     由调用方（policy）决定是否采用，且修正次数有硬上限；
  3. **失败即保守**：模型超时/返回乱码/置信度低 → `ok=False` + 原因，
     调用方据此**停爪**，绝不当成"可以抓"；
  4. **不许编造**：没配 API Key 就明确返回"未配置"，不返回假通过。

模型接入方式（三选一，按现场条件切换）：
  - `OpenAICompatibleVLM`：任何 OpenAI 兼容 /chat/completions 服务
    （DeepSeek-VL、通义千问 VL、豆包、GPT-4o、本地 vLLM 均可）；
  - `StaticVLM`：测试用，返回预先写死的判断（mock 回归靠它）；
  - 自定义：实现 `VLMBackend.judge()` 即可。

用法：
    backend = OpenAICompatibleVLM(base_url=..., api_key=..., model="qwen-vl-max")
    verdict = backend.judge(image_path, prompt)
    if not verdict.ok: 停爪
    dx, dy = clamp_correction(verdict.offset_x_cm, verdict.offset_y_cm, ...)
"""

from __future__ import annotations

import base64
import json
import logging
import os
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Protocol

logger = logging.getLogger("scene.medicine.perception.vlm")

#: 单次修正的位移上限（米），超出说明判断不可信或盒子离得太远
MAX_CORRECTION_M = 0.03
#: 累计修正上限（米），超过就放弃，不再下爪
MAX_TOTAL_CORRECTION_M = 0.06
#: 判定为「可以下爪」的置信度门槛（0~1）
MIN_CONFIDENCE = 0.6

DEFAULT_PROMPT = """你是机器人抓取前的视觉质检员。请只看图片回答，不要猜测。

任务：判断画面中央的机械夹爪正下方是否有一个药盒，且夹爪张开后能把它夹住。

请严格输出 JSON（不要 markdown 代码块、不要解释），字段：
{
  "box_present": true/false,        // 夹爪下方是否有药盒
  "graspable": true/false,          // 若现在闭合夹爪，能否夹住（没有被遮挡/不歪太多）
  "offset_x_cm": 数字,              // 药盒中心相对夹爪中心的前后偏差，向前为正；看不清填 null
  "offset_y_cm": 数字,              // 左右偏差，机器人左侧为正；看不清填 null
  "confidence": 0~1 的数字,         // 你对以上判断的把握
  "reason": "一句话说明依据"
}"""


@dataclass
class VLMVerdict:
    """VLM 的结构化判断结果。ok=False 时调用方必须停爪。"""

    ok: bool
    message: str = ""
    box_present: bool = False
    graspable: bool = False
    offset_x_cm: Optional[float] = None
    offset_y_cm: Optional[float] = None
    confidence: float = 0.0
    reason: str = ""
    raw: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "box_present": self.box_present,
            "graspable": self.graspable,
            "offset_x_cm": self.offset_x_cm,
            "offset_y_cm": self.offset_y_cm,
            "confidence": round(float(self.confidence), 3),
            "reason": self.reason,
        }


class VLMBackend(Protocol):
    """任何后端只要实现 judge() 就能插进来。"""

    def judge(self, image_path: str, prompt: str = DEFAULT_PROMPT) -> VLMVerdict:
        ...


# --------------------------------------------------------------------------- #
# 结果解析
# --------------------------------------------------------------------------- #
def _strip_code_fence(text: str) -> str:
    """模型爱包 ```json ... ```，剥掉它。"""
    t = (text or "").strip()
    if t.startswith("```"):
        t = t.split("\n", 1)[-1] if "\n" in t else t
        if t.rstrip().endswith("```"):
            t = t.rstrip()[:-3]
    return t.strip()


def _to_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_verdict(text: str) -> VLMVerdict:
    """把模型返回的文本解析成 VLMVerdict。解析失败一律 ok=False（保守）。"""
    cleaned = _strip_code_fence(text)
    if not cleaned:
        return VLMVerdict(ok=False, message="VLM 返回空内容", raw=text)

    try:
        payload = json.loads(cleaned)
    except json.JSONDecodeError:
        # 尝试从文本里抠出第一个 {...}
        start, end = cleaned.find("{"), cleaned.rfind("}")
        if start < 0 or end <= start:
            return VLMVerdict(ok=False, message="VLM 返回非 JSON，无法判定", raw=text)
        try:
            payload = json.loads(cleaned[start:end + 1])
        except json.JSONDecodeError:
            return VLMVerdict(ok=False, message="VLM 返回 JSON 解析失败", raw=text)

    if not isinstance(payload, dict):
        return VLMVerdict(ok=False, message="VLM 返回不是对象", raw=text)

    confidence = _to_float(payload.get("confidence")) or 0.0
    verdict = VLMVerdict(
        ok=True,
        message="判定完成",
        box_present=bool(payload.get("box_present")),
        graspable=bool(payload.get("graspable")),
        offset_x_cm=_to_float(payload.get("offset_x_cm")),
        offset_y_cm=_to_float(payload.get("offset_y_cm")),
        confidence=max(0.0, min(1.0, confidence)),
        reason=str(payload.get("reason") or ""),
        raw=text,
    )
    if verdict.confidence < MIN_CONFIDENCE:
        # 置信度不够：ok 仍为 True（模型说得出话），但 graspable 强制置 False
        verdict.graspable = False
        verdict.message = f"置信度 {verdict.confidence:.2f} 低于门槛 {MIN_CONFIDENCE}"
    return verdict


# --------------------------------------------------------------------------- #
# 限幅：VLM 只能建议小幅修正
# --------------------------------------------------------------------------- #
def clamp_correction(dx_m: float, dy_m: float, total_done_m: float = 0.0
                     ) -> tuple:
    """把 VLM 建议的修正量限幅。返回 (dx, dy, note)。"""
    note = ""
    if abs(dx_m) > MAX_CORRECTION_M or abs(dy_m) > MAX_CORRECTION_M:
        note = (f"单次修正超过 ±{MAX_CORRECTION_M * 100:.0f}cm，"
                f"原文 dx={dx_m * 100:.1f}cm dy={dy_m * 100:.1f}cm，已限幅")
        dx_m = max(-MAX_CORRECTION_M, min(MAX_CORRECTION_M, dx_m))
        dy_m = max(-MAX_CORRECTION_M, min(MAX_CORRECTION_M, dy_m))
    if total_done_m + max(abs(dx_m), abs(dy_m)) > MAX_TOTAL_CORRECTION_M:
        note += f"；累计修正将超 {MAX_TOTAL_CORRECTION_M * 100:.0f}cm，拒绝"
        return 0.0, 0.0, note
    return dx_m, dy_m, note


# --------------------------------------------------------------------------- #
# 后端实现
# --------------------------------------------------------------------------- #
class OpenAICompatibleVLM:
    """OpenAI 兼容 /chat/completions 的多模态后端（纯标准库，无第三方依赖）。"""

    def __init__(self, base_url: str = "", api_key: str = "", model: str = "",
                 timeout_s: float = 30.0) -> None:
        self.base_url = (base_url or os.environ.get("VLM_BASE_URL", "")).rstrip("/")
        self.api_key = api_key or os.environ.get("VLM_API_KEY", "")
        self.model = model or os.environ.get("VLM_MODEL", "")
        self.timeout_s = timeout_s

    def configured(self) -> bool:
        return bool(self.base_url and self.api_key and self.model)

    def judge(self, image_path: str, prompt: str = DEFAULT_PROMPT) -> VLMVerdict:
        if not self.configured():
            # 诚实：没配置就说没配置，绝不返回假通过
            return VLMVerdict(
                ok=False,
                message="VLM 未配置：需设置 VLM_BASE_URL / VLM_API_KEY / VLM_MODEL",
            )
        try:
            with open(image_path, "rb") as f:
                b64 = base64.b64encode(f.read()).decode("ascii")
        except OSError as exc:
            return VLMVerdict(ok=False, message=f"读图失败：{exc}")

        body = {
            "model": self.model,
            "messages": [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            ]}],
            "temperature": 0.0,
            "max_tokens": 400,
        }
        import urllib.error
        import urllib.request

        req = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.api_key}"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                detail = exc.read().decode("utf-8", "replace")[:200]
            except Exception:  # noqa: BLE001
                pass
            return VLMVerdict(ok=False, message=f"VLM HTTP {exc.code}: {detail}")
        except Exception as exc:  # noqa: BLE001 - 网络层任何异常都保守处理
            return VLMVerdict(ok=False, message=f"VLM 请求失败：{exc}")

        try:
            text = payload["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError):
            return VLMVerdict(ok=False, message=f"VLM 响应结构异常：{str(payload)[:200]}")
        return parse_verdict(text if isinstance(text, str) else str(text))


class StaticVLM:
    """测试/离线用：返回预先给定的判断，不联网。

    mock 回归里用它，保证流水线测试不依赖外部 API。
    """

    def __init__(self, verdict: Optional[VLMVerdict] = None) -> None:
        self._verdict = verdict or VLMVerdict(
            ok=True, message="静态桩：默认通过",
            box_present=True, graspable=True,
            offset_x_cm=0.0, offset_y_cm=0.0, confidence=0.95,
            reason="static backend",
        )

    def judge(self, image_path: str, prompt: str = DEFAULT_PROMPT) -> VLMVerdict:
        return self._verdict


def build_backend(kind: str = "", **kwargs: Any) -> VLMBackend:
    """工厂：按名字造后端。默认按环境变量是否有配置决定。"""
    name = (kind or os.environ.get("VLM_BACKEND") or "").strip().lower()
    if name in ("static", "mock", "stub"):
        return StaticVLM(**kwargs)
    if name in ("openai", "openai_compatible", ""):
        backend = OpenAICompatibleVLM(**kwargs)
        if backend.configured():
            return backend
        logger.warning("VLM 后端未配置（缺 VLM_BASE_URL/VLM_API_KEY/VLM_MODEL），"
                       "校验将一律返回 ok=False → policy 会停爪")
        return backend
    raise ValueError(f"未知 VLM 后端：{name}（支持 static | openai）")
