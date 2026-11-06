"""多模态大模型客户端抽象：顶层 Agent 的大脑接口。

设计目标：
- 连接的可以是纯文本 LLM，也可以是**多模态大模型**（视觉-语言模型 VLM）。
  接口统一：observation 既可以是纯文本，也可以携带图片（相机帧）。
- Agent 架构不写死，模型提供方可随时替换，只需实现 decide()。
- 默认实现 OpenAICompatibleClient：兼容 OpenAI GPT-4o / Qwen-VL / GLM-4V /
  DeepSeek / 本地 vLLM 等一切 OpenAI 格式的 API，通过环境变量配置：
    export LLM_API_KEY="sk-..."
    export LLM_BASE_URL="https://api.openai.com/v1"   # 可选
    export LLM_MODEL="gpt-4o-mini"                     # 可选（多模态选 VLM 型号）
- 未配置 API Key 时自动降级为 RuleBasedClient（仅用于调通流程，没有智能）。

多模态说明：
- observation 传 dict 时可带 images（本地图片路径列表），客户端自动转 base64
  data URL，按 OpenAI vision 消息格式发送（content 数组含 image_url）。
- 模型端是否支持图片由所选模型决定；纯文本模型收到图片会报错，客户端会
  捕获并降级为"仅文本 + 图片占位描述"，保证主循环不崩。
"""

import base64
import json
import logging
import mimetypes
import os
import urllib.request

logger = logging.getLogger("agent.llm")

# observation 的两种合法形态：
#   str                                  —— 纯文本观察
#   {"text": str, "images": [路径, ...]}  —— 多模态观察（文本 + 相机帧）


def _encode_image(image_path: str) -> dict | None:
    """本地图片 → OpenAI vision 格式的 image_url 块。读不到返回 None。"""
    if not os.path.exists(image_path):
        logger.warning("图片不存在，跳过: %s", image_path)
        return None
    mime = mimetypes.guess_type(image_path)[0] or "image/jpeg"
    with open(image_path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return {"type": "image_url",
            "image_url": {"url": f"data:{mime};base64,{b64}"}}


def _build_user_message(observation) -> dict:
    """把观察统一构造成 OpenAI 消息（多模态用 content 数组）。"""
    if isinstance(observation, str):
        observation = {"text": observation, "images": []}

    text = observation.get("text", "")
    instruction = (
        "\n请输出 JSON 决策："
        '{"action": "call_scene"|"call_wrapper"|"wait"|"finish", '
        '"target": "...", "params": {...}, "reason": "..."}')

    images = observation.get("images") or []
    if not images:
        return {"role": "user", "content": f"当前观察：{text}{instruction}"}

    # 多模态：content 数组 = 文本块 + 图片块
    content = [{"type": "text", "text": f"当前观察（含 {len(images)} 张相机画面）："
                                        f"{text}{instruction}"}]
    n_ok = 0
    for path in images:
        block = _encode_image(path)
        if block:
            content.append(block)
            n_ok += 1
    if n_ok == 0:
        # 图片全挂了，退化为纯文本，别让请求崩掉
        return {"role": "user",
                "content": f"当前观察：{text}（{len(images)} 张图片读取失败）{instruction}"}
    return {"role": "user", "content": content}


class LLMClient:
    """模型客户端基类。换大脑只需继承本类实现 decide()。"""

    supports_images = False  # 子类声明自己是否支持图片输入

    def decide(self, system_prompt: str, observation, history: list) -> dict:
        """根据提示词 + 当前观察（可含图片）+ 历史，决定下一步动作。

        Args:
            system_prompt: 用户提示词 + 能力清单（由 main.py 拼好）
            observation:   str 或 {"text": str, "images": [图片路径, ...]}
            history:       最近若干条交互（OpenAI 消息格式）

        Returns:
            {"action": "call_scene" | "call_wrapper" | "wait" | "finish",
             "target": "场景名 或 封装函数名",
             "params": {...},
             "reason": "决策理由（写入 task_log）"}
        """
        raise NotImplementedError


class OpenAICompatibleClient(LLMClient):
    """OpenAI 兼容 API 客户端（零依赖，纯 urllib）。支持多模态模型。"""

    supports_images = True

    def __init__(self):
        self.api_key = os.environ.get("LLM_API_KEY", "")
        self.base_url = os.environ.get("LLM_BASE_URL", "https://api.openai.com/v1")
        self.model = os.environ.get("LLM_MODEL", "gpt-4o-mini")
        # 单张图超过该大小就拒绝发送，防止相机原图把 token/带宽打爆
        self.max_image_bytes = int(os.environ.get("LLM_MAX_IMAGE_BYTES",
                                                  str(4 * 1024 * 1024)))

    def _chat(self, messages: list) -> dict:
        body = json.dumps({
            "model": self.model,
            "messages": messages,
            "response_format": {"type": "json_object"},
            "temperature": 0.2,
        }).encode("utf-8")
        req = urllib.request.Request(
            f"{self.base_url}/chat/completions", data=body,
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.api_key}"})
        with urllib.request.urlopen(req, timeout=60) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return json.loads(data["choices"][0]["message"]["content"])

    def decide(self, system_prompt: str, observation, history: list) -> dict:
        user_msg = _build_user_message(observation)

        # 图片过大预检：超大图片抽掉再降级为纯文本
        if isinstance(user_msg.get("content"), list):
            kept = [user_msg["content"][0]]
            for block in user_msg["content"][1:]:
                if len(block["image_url"]["url"]) * 3 // 4 > self.max_image_bytes:
                    logger.warning("图片超过 %d 字节，本次不发送",
                                   self.max_image_bytes)
                    continue
                kept.append(block)
            if len(kept) == 1:
                user_msg = {"role": "user",
                            "content": kept[0]["text"] + "（图片超限未发送）"}
            else:
                user_msg["content"] = kept

        messages = [{"role": "system", "content": system_prompt}]
        messages.extend(history[-10:])  # 只带最近 10 条，防爆 token
        messages.append(user_msg)

        try:
            return self._chat(messages)
        except Exception as e:  # noqa: BLE001
            # 多模态请求失败（常见于所选模型不支持图片）→ 降级纯文本重试一次
            if isinstance(user_msg.get("content"), list):
                logger.warning("多模态请求失败（%s），降级为纯文本重试", e)
                text_only = user_msg["content"][0]["text"] + "（图片发送失败，本次仅凭文字判断）"
                messages[-1] = {"role": "user", "content": text_only}
                return self._chat(messages)
            raise


class RuleBasedClient(LLMClient):
    """规则降级版：仅用于无 API Key 时调通流程。忽略所有输入只待机。"""

    def decide(self, system_prompt: str, observation, history: list) -> dict:
        return {"action": "wait", "target": None, "params": {},
                "reason": "未配置 LLM_API_KEY，规则模式只待机。请配置环境变量后重启。"}


def create_llm_client() -> LLMClient:
    """工厂函数：有 Key 用真模型，没有则降级。"""
    if os.environ.get("LLM_API_KEY"):
        model = os.environ.get("LLM_MODEL", "gpt-4o-mini")
        logger.info("使用 OpenAICompatibleClient (model=%s, 多模态=支持)", model)
        return OpenAICompatibleClient()
    logger.warning("未检测到 LLM_API_KEY，降级为规则模式（无智能，仅调试用）")
    return RuleBasedClient()
