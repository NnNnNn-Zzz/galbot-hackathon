"""YOLO 推理服务的客户端侧契约（`drawer_box_seg/1`）。

感知是独立进程，机器人侧只传结构化结果 —— 这条规矩写在仓库
`docs/DEVELOPMENT_RULES.md` 里，适配器照办，不在本进程加载 torch。

同机部署时默认 `http://127.0.0.1:8010`，请求体：

    POST /segment
    {"schema": "drawer_box_seg/1", "image_ref": "<本机绝对路径>",
     "frame_id": "...", "camera_id": "...", "capture_timestamp_ns": 123,
     "image_size_wh": [w, h], "model_id": "..."}

响应必须是同 schema 的完整结果，`frame_id` / `camera_id` /
`capture_timestamp_ns` / `image_size_wh` / `model_id` 原样回传。回传不一致
说明服务串帧了，适配器直接拒，不做补偿。
"""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

from .link import LinkError

SCHEMA = "drawer_box_seg/1"


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class YoloHttpClient:
    def __init__(self, url: str, *, model_id: str, timeout_s: float = 3.0,
                 perception_backend: str | None = None, weights_sha256: str | None = None):
        parsed = urllib.parse.urlsplit(url)
        if (parsed.scheme != "http" or parsed.hostname != "127.0.0.1"
                or parsed.username or parsed.password or parsed.path not in ("", "/")
                or parsed.query or parsed.fragment):
            raise ValueError("感知服务必须是同机 http://127.0.0.1:<port>")
        self.url = url.rstrip("/")
        self.model_id = model_id
        self.perception_backend = perception_backend
        self.weights_sha256 = weights_sha256
        self.timeout_s = float(timeout_s)
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())

    def health(self) -> bool:
        payload = self._json(urllib.request.Request(self.url + "/health"), self.timeout_s)
        return (payload.get("schema") == SCHEMA and payload.get("status") == "ok"
                and self._matches_model(payload))

    def _matches_model(self, payload: dict) -> bool:
        return (payload.get("model_id") == self.model_id
                and (self.perception_backend is None or payload.get("perception_backend") == self.perception_backend)
                and (self.weights_sha256 is None or payload.get("weights_sha256") == self.weights_sha256))

    def segment(self, frame, timeout_s: float) -> dict[str, Any]:
        body = json.dumps({
            "schema": SCHEMA,
            "image_ref": frame.image_ref,
            "frame_id": frame.frame_id,
            "camera_id": frame.camera_id,
            "capture_timestamp_ns": frame.capture_timestamp_ns,
            "image_size_wh": list(frame.image_size_wh),
            "model_id": self.model_id,
        }, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            self.url + "/segment", data=body, method="POST",
            headers={"Content-Type": "application/json; charset=utf-8"})
        budget = min(self.timeout_s, timeout_s) if timeout_s > 0 else self.timeout_s
        payload = self._json(request, budget)
        if not self._matches_model(payload):
            raise LinkError("感知结果的模型编号、后端或权重摘要与配方不符")
        return payload

    def _json(self, request, budget: float) -> dict[str, Any]:
        try:
            with self._opener.open(request, timeout=budget) as response:
                raw = response.read(1_000_001)
                if len(raw) > 1_000_000:
                    raise LinkError("感知服务返回过大")
        except urllib.error.HTTPError as exc:
            raise LinkError(f"感知服务返回 HTTP {exc.code}") from None
        except (urllib.error.URLError, TimeoutError) as exc:
            raise LinkError(f"感知服务不可达：{exc}") from None
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise LinkError("感知服务返回的不是 JSON") from None
        if not isinstance(payload, dict):
            raise LinkError("感知服务返回结构非法")
        return payload
