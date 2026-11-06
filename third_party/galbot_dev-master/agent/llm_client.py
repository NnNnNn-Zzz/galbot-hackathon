"""agent/llm_client.py —— 多模态模型的接入层（零第三方依赖）。

设计取舍：
  * 只用标准库 `urllib`，不依赖 openai/requests —— 赛场上装不上包是常态；
  * 接口按 OpenAI `/chat/completions` 的 function-calling 规范，
    vLLM / Ollama / DeepSeek / 通义 / 智谱 / 商汤 都是这套协议，改 base_url 即可；
  * 模型不支持视觉时**自动退化为纯文本**（结构化场景描述本来就是文本），
    所以即使拿不到多模态配额，整条链路也能演示。

`ScriptedLLM` 是**离线兜底**：它不是模型，只是关键词规则。
存在的理由是"评委演示时网络挂了也不能白屏"，绝不能被当成真模型。
"""

from __future__ import annotations

import base64
import json
import logging
import os
import typing
import urllib.error
import urllib.request
from dataclasses import dataclass, field

__all__ = [
    "ToolCall",
    "LLMResponse",
    "LLMError",
    "OpenAICompatibleClient",
    "ScriptedLLM",
    "build_client",
    "image_data_url",
]

LOG = logging.getLogger(__name__)


class LLMError(RuntimeError):
    """模型调用失败。调用方应把 error_code 映射为 sdk_error / timeout。"""


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: dict[str, typing.Any] = field(default_factory=dict)

    @classmethod
    def parse(cls, raw: typing.Mapping[str, typing.Any]) -> "ToolCall":
        fn = raw.get("function") or {}
        args = fn.get("arguments")
        if isinstance(args, str):
            try:
                args = json.loads(args) if args.strip() else {}
            except json.JSONDecodeError:
                args = {"__unparsed__": args}
        if not isinstance(args, dict):
            args = {}
        return cls(
            id=str(raw.get("id") or f"call_{fn.get('name', 'unknown')}"),
            name=str(fn.get("name") or ""),
            arguments=args,
        )

    def as_message_part(self) -> dict[str, typing.Any]:
        return {
            "id": self.id,
            "type": "function",
            "function": {"name": self.name, "arguments": json.dumps(self.arguments, ensure_ascii=False)},
        }


@dataclass
class LLMResponse:
    content: str = ""
    tool_calls: list[ToolCall] = field(default_factory=list)
    finish_reason: str = "stop"

    @property
    def wants_tools(self) -> bool:
        return bool(self.tool_calls)


def image_data_url(path: str | os.PathLike[str], max_side: int = 768) -> str:
    """图片 -> data URL（给支持视觉的模型）。会等比缩小，避免 token 爆炸。"""
    import cv2

    img = cv2.imread(os.fspath(path), cv2.IMREAD_COLOR)
    if img is None:
        raise LLMError(f"读不到图像: {path}")
    h, w = img.shape[:2]
    scale = min(1.0, float(max_side) / max(h, w))
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise LLMError(f"JPEG 编码失败: {path}")
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")


# ═══════════════════════════════════════════════════════════════════════════ #
# OpenAI 兼容客户端
# ═══════════════════════════════════════════════════════════════════════════ #
class OpenAICompatibleClient:
    """POST {base_url}/chat/completions。"""

    def __init__(
        self,
        model: str,
        base_url: str | None = None,
        api_key: str | None = None,
        *,
        timeout_s: float = 60.0,
        temperature: float = 0.2,
        supports_vision: bool = False,
        extra_headers: dict[str, str] | None = None,
    ) -> None:
        self.model = model
        self.base_url = (base_url or os.environ.get("OPENAI_BASE_URL") or "https://api.openai.com/v1").rstrip("/")
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY") or ""
        self.timeout_s = float(timeout_s)
        self.temperature = float(temperature)
        self.supports_vision = bool(supports_vision)
        self.extra_headers = dict(extra_headers or {})
        if not self.api_key:
            LOG.warning("未提供 api_key：仅当本地 vLLM/Ollama 不需要鉴权时才能工作")

    # ---- 底层 HTTP ----
    def _post(self, payload: dict[str, typing.Any]) -> dict[str, typing.Any]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}",
            "Accept": "application/json",
        }
        headers.update(self.extra_headers)
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions", data=body, headers=headers, method="POST"
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")[:800]
            raise LLMError(f"HTTP {exc.code} from {self.base_url}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise LLMError(f"无法连接 {self.base_url}: {exc.reason}") from exc
        except TimeoutError as exc:
            raise LLMError(f"请求超时 {self.timeout_s}s") from exc
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            raise LLMError(f"返回不是 JSON（前 400 字符）：{raw[:400]}") from exc

    # ---- 对外 ----
    def complete(
        self,
        messages: list[dict[str, typing.Any]],
        tools: list[dict[str, typing.Any]] | None = None,
    ) -> LLMResponse:
        payload: dict[str, typing.Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
        }
        if tools:
            payload["tools"] = tools
            payload["tool_choice"] = "auto"
        data = self._post(payload)
        choices = data.get("choices") or []
        if not choices:
            raise LLMError(f"模型没有返回 choices: {json.dumps(data, ensure_ascii=False)[:400]}")
        choice = choices[0]
        message = choice.get("message") or {}
        return LLMResponse(
            content=(message.get("content") or "").strip(),
            tool_calls=[ToolCall.parse(tc) for tc in (message.get("tool_calls") or [])],
            finish_reason=str(choice.get("finish_reason") or "stop"),
        )


# ═══════════════════════════════════════════════════════════════════════════ #
# 离线兜底：启发式规则引擎
# ═══════════════════════════════════════════════════════════════════════════ #
_COLOR_WORDS = {
    "红": "red", "红色": "red", "red": "red",
    "橙": "orange", "orange": "orange",
    "黄": "yellow", "黄色": "yellow", "yellow": "yellow",
    "绿": "green", "绿色": "green", "green": "green",
    "蓝": "blue", "蓝色": "blue", "blue": "blue",
    "紫": "purple", "紫色": "purple", "purple": "purple",
    "黑": "black", "黑色": "black", "black": "black",
    "白": "white", "白色": "white", "white": "white",
}


class ScriptedLLM:
    """离线关键词规则引擎（**不是模型**）。

    行为：第一轮按关键词发出一个工具调用；收到工具结果后把它翻译成人话。
    这样即使没有网络、没有 API key，`python -m agent.app --demo` 也能完整走一遍
    LLM -> tool -> result -> answer 的闭环，用来验证协议而不是验证模型。
    """

    name = "scripted-offline"

    def __init__(self, supports_vision: bool = False) -> None:
        self.supports_vision = supports_vision
        #: 由 AgentApp 注入当前实际注册的工具名，用来解释"为什么这件事做不了"
        self.tool_names: set[str] = set()

    def complete(
        self,
        messages: list[dict[str, typing.Any]],
        tools: list[dict[str, typing.Any]] | None = None,
    ) -> LLMResponse:
        last = messages[-1] if messages else {}

        if last.get("role") == "tool":
            return LLMResponse(content=self._summarize(last.get("content", "")))
        if last.get("role") == "assistant" and last.get("tool_calls"):
            return LLMResponse(content="（工具已执行，等待结果）")

        text = str(last.get("content") or "")
        if isinstance(last.get("content"), list):
            text = " ".join(
                str(part.get("text", "")) for part in last["content"] if isinstance(part, dict)
            )

        call = self._pick_tool(text, messages)
        if call is not None:
            return LLMResponse(content="", tool_calls=[call], finish_reason="tool_calls")

        # 想做但对应工具没注册 -> 明确说清能力缺口，而不是瞎编一个动作
        gap = self._explain_gap(text)
        if gap is not None:
            return LLMResponse(content=gap)

        return LLMResponse(
            content=(
                "我是离线规则引擎（没有配置模型）。可以这样问我：\n"
                "  · 你看到了什么？        -> analyze_scene\n"
                "  · 红色方块在哪？        -> find_object\n"
                "  · 有哪些相机可用？      -> list_cameras\n"
                "  · 原点(640,512)多远？   -> measure_distance\n"
                "  · 机器人状态怎么样？    -> get_robot_state\n"
                "  · 挥个手 / 去 demo_table / 取消任务\n"
                "已注册工具：" + ("、".join(sorted(self.tool_names)) or "（无）") + "\n"
                "配置 OPENAI_API_KEY / --model 后即切换为真实多模态模型。"
            )
        )

    # ---- 能力缺口说明 ----
    _INTENT_RULES: tuple[tuple[tuple[str, ...], str, str], ...] = (
        (("挥手", "打招呼", "做个动作", "挥一下", "wave"), "execute_gesture", "动作"),
        (("导航", "带我去", "前往", "走到", "去"), "navigate_to_place", "导航"),
        (("任务", "到了吗", "完成了吗", "进度"), "get_task_status", "任务查询"),
        (("取消", "停止", "停下", "别动"), "cancel_task", "取消"),
        (("状态", "电量", "在动吗", "忙吗"), "get_robot_state", "机器人状态"),
    )

    def _explain_gap(self, text: str) -> str | None:
        for keywords, tool, label in self._INTENT_RULES:
            if tool in self.tool_names:
                continue
            if any(one in text for one in keywords):
                return (
                    f"{label}类工具目前**没有注册**（`{tool}` 不可用）："
                    f"A 侧的 adapters/、motion.py、navigation.py 尚未就绪，我不会假装机器人已经执行。\n"
                    f"当前可用工具：{('、'.join(sorted(self.tool_names)) or '（无）')}"
                )
        return None

    @staticmethod
    def _last_task_id(messages: list[dict[str, typing.Any]]) -> str | None:
        """从历史里找最近一次工具返回的 task_id，供"任务完成了吗"这类追问使用。"""
        for message in reversed(messages):
            if message.get("role") != "tool":
                continue
            payload = message.get("content")
            try:
                data = json.loads(payload) if isinstance(payload, str) else payload
            except (json.JSONDecodeError, TypeError):
                continue
            if isinstance(data, dict) and data.get("task_id"):
                return str(data["task_id"])
        return None

    @staticmethod
    def _guess_place(text: str) -> str:
        """把"去前台那边"剥成 "前台"，交给 A 的白名单去判定（不替它猜合法名）。"""
        import re

        cleaned = re.sub(
            r"(带我去|导航到|移动到|去往|导航|前往|走到|去|到|一下|那边|那里|吧|呢|，|。|？|\?)",
            "",
            text,
        )
        return cleaned.strip() or text.strip()

    # ---- 规则 ----
    def _pick_tool(
        self, text: str, messages: list[dict[str, typing.Any]] | None = None
    ) -> ToolCall | None:
        lowered = text.lower()

        # ── A 侧：执行类。顺序即优先级：取消 > 任务状态 > 动作 > 导航 > 本体状态 ──
        task_id = self._last_task_id(messages or [])
        if "cancel_task" in self.tool_names and task_id and any(
            k in text for k in ("取消", "停止", "停下", "别动")
        ):
            return ToolCall(id="call_cancel_task", name="cancel_task", arguments={"task_id": task_id})
        if "get_task_status" in self.tool_names and task_id and any(
            k in text for k in ("任务", "到了吗", "完成了吗", "进度", "刚才")
        ):
            return ToolCall(id="call_task_status", name="get_task_status", arguments={"task_id": task_id})
        if "execute_gesture" in self.tool_names and any(
            k in text for k in ("挥", "打招呼", "做个动作", "wave")
        ):
            return ToolCall(
                id="call_execute_gesture",
                name="execute_gesture",
                arguments={"gesture_id": "wave_right"},
            )
        if "navigate_to_place" in self.tool_names and any(
            k in text for k in ("导航", "带我去", "前往", "走到", "去")
        ):
            place = "demo_table" if any(
                k in text for k in ("demo_table", "桌", "工位", "台面")
            ) else self._guess_place(text)
            return ToolCall(
                id="call_navigate", name="navigate_to_place", arguments={"place_id": place}
            )
        if "get_robot_state" in self.tool_names and any(
            k in text for k in ("状态", "电量", "怎么样了", "在动吗", "忙吗")
        ):
            return ToolCall(id="call_robot_state", name="get_robot_state", arguments={})

        # ── B 侧：相机 / 视觉 ──
        if any(k in text for k in ("相机", "机位", "摄像头")) or "camera" in lowered:
            return ToolCall(id="call_list_cameras", name="list_cameras", arguments={})

        color = next((v for k, v in _COLOR_WORDS.items() if k in lowered or k in text), None)
        if color and any(k in text for k in ("哪", "找", "在", "位置", "抓", "拿", "取", "方块", "球")):
            return ToolCall(
                id="call_find_object",
                name="find_object",
                arguments={"label": color, "max_distance_m": 1.5},
            )
        if any(k in text for k in ("最近", "前面", "前景", "最前")) and any(
            k in text for k in ("什么", "哪个", "物体", "东西", "找")
        ):
            return ToolCall(
                id="call_find_object", name="find_object", arguments={"label": "foreground"}
            )
        if any(k in text for k in ("看到", "看见", "场景", "有什么", "描述", "观察", "画面")):
            return ToolCall(id="call_analyze_scene", name="analyze_scene", arguments={})

        pixel = self._parse_pixel(text)
        if pixel is not None:
            return ToolCall(
                id="call_measure_distance",
                name="measure_distance",
                arguments={"u": pixel[0], "v": pixel[1]},
            )
        return None

    @staticmethod
    def _parse_pixel(text: str) -> tuple[int, int] | None:
        import re

        match = re.search(r"\(?\s*(\d{1,4})\s*[,，]\s*(\d{1,4})\s*\)?", text)
        if match is None:
            return None
        return int(match.group(1)), int(match.group(2))

    @staticmethod
    def _summarize(tool_content: typing.Any) -> str:
        try:
            payload = json.loads(tool_content) if isinstance(tool_content, str) else dict(tool_content)
        except (json.JSONDecodeError, TypeError, ValueError):
            return f"工具结果无法解析：{str(tool_content)[:300]}"

        status = payload.get("status")
        mode = payload.get("mode")
        message = payload.get("message", "")
        data = payload.get("data") or {}
        error_code = payload.get("error_code")
        header = f"[{mode}/{status}]" if not error_code else f"[{mode}/{status}:{error_code}]"

        if status == "running":
            # 任务已下发 ≠ 完成。把 task_id 明确交出来，agent 才知道下一步要轮询。
            task_id = payload.get("task_id")
            if task_id:
                return (
                    f"{header} {message}\n"
                    f"  task_id：{task_id}（必须用 get_task_status 轮询到 succeeded 才算完成）"
                )
            return f"{header} {message}"

        if status != "succeeded":
            hint = {
                "not_ready": "没读到数据，检查该机位是否 enable / 是否装配",
                "not_supported": "该机位不支持这个能力",
                "invalid_argument": "参数不合法",
                "sdk_error": "SDK 侧报错，看 warnings",
            }.get(payload.get("error_code") or "", "")
            extra = ""
            if data.get("supported_ids"):
                extra = f"；可用：{data['supported_ids']}"
            elif data.get("legal_labels"):
                extra = f"；可用：{data['legal_labels']}"
            elif data.get("legal_camera_ids"):
                extra = f"；可用：{data['legal_camera_ids']}"
            return f"{header} {message}" + (f"（{hint}）" if hint else "") + extra

        objects = data.get("objects")
        if isinstance(objects, list) and objects:
            lines = [header + " 视野内目标："]
            for obj in objects:
                depth = obj.get("depth_m")
                dist = f"{depth} m" if depth is not None else "距离未知"
                lines.append(
                    f"  · {obj.get('label')} 于像素 {obj.get('centroid_uv')}，{dist}"
                    + (f"，base_link={obj['base_link_pose_m']}" if obj.get("base_link_pose_m") else "")
                )
            if data.get("depth_summary"):
                lines.append(f"  深度：{data['depth_summary']}")
            return "\n".join(lines)

        if data.get("target"):
            target = data["target"]
            return (
                f"{header} 找到 {target.get('label')}：像素 {target.get('centroid_uv')}，"
                f"距离 {target.get('depth_m')} m，相机系 {target.get('center_cam_m')}"
                + (f"，base_link {data['base_link_pose_m']}" if data.get("base_link_pose_m") else "")
            )
        if data.get("distance_m"):
            distance = data["distance_m"]
            return f"{header} 该点距离约 {distance.get('z')} m，相机系坐标 {[distance.get('x'), distance.get('y'), distance.get('z')]}"
        if data.get("cameras"):
            lines = [header + " 可用机位："]
            for cam in data["cameras"]:
                lines.append(
                    f"  · {cam['camera_id']}: RGB={cam['has_rgb']} 深度={cam['has_depth']} ({cam['kind']})"
                )
            return "\n".join(lines)

        # A 侧执行类结果：operation / target / 任务 ID
        if data.get("operation") or data.get("motion_state"):
            parts = [f"{header} {message}"]
            if data.get("operation"):
                parts.append(f"  操作：{data['operation']} → {data.get('target')}")
            if payload.get("task_id"):
                parts.append(f"  task_id：{payload['task_id']}")
            if data.get("motion_state"):
                parts.append(f"  运动状态：{data['motion_state']}")
            if data.get("active_task_ids"):
                parts.append(f"  在跑的任务：{data['active_task_ids']}")
            if data.get("connection_state"):
                parts.append(f"  连接：{data['connection_state']}")
            return "\n".join(parts)

        return f"{header} {message}"


# ═══════════════════════════════════════════════════════════════════════════ #
def build_client(
    model: str | None = None,
    base_url: str | None = None,
    api_key: str | None = None,
    *,
    offline: bool = False,
    supports_vision: bool = False,
) -> OpenAICompatibleClient | ScriptedLLM:
    """有 key/model 就上真模型，否则退回离线规则引擎（且大声说出来）。"""
    model = model or os.environ.get("LLM_MODEL")
    api_key = api_key or os.environ.get("OPENAI_API_KEY") or os.environ.get("LLM_API_KEY")
    if offline or not model:
        LOG.warning("未配置模型（--model / LLM_MODEL），使用离线规则引擎")
        return ScriptedLLM(supports_vision=supports_vision)
    return OpenAICompatibleClient(
        model=model, base_url=base_url, api_key=api_key, supports_vision=supports_vision
    )
