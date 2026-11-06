"""algorithms/place_resolver.py —— 自然语言地名 -> place_id。

协议 §3 / §5 明确规定：导航工具只接受**已登记的 place_id**，
所以"把前台那个地方解析成 place_id"这件事必须在 B 侧做完，
绝不能让大模型直接生成一个坐标丢给 navigate_to_place（那是安全漏洞）。

本模块只做**字符串到白名单 ID**的映射；真实可导航性由 A 的 navigation.py 决定。
找不到就返回 None —— 调用方必须回 rejected + invalid_argument，而不是猜一个。
"""

from __future__ import annotations

import json
import os
import typing
from difflib import SequenceMatcher

__all__ = ["PlaceResolver", "PlaceSpec"]

_DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "places.json"
)


class PlaceSpec(typing.NamedTuple):
    place_id: str
    aliases: tuple[str, ...]
    description: str


class PlaceResolver:
    """地名 -> place_id。支持中文别名、模糊匹配、否定式提示（返回候选）。"""

    def __init__(self, places: typing.Iterable[PlaceSpec], default_threshold: float = 0.72) -> None:
        self._places = {p.place_id: p for p in places}
        self._alias_index: dict[str, str] = {}
        for spec in self._places.values():
            self._alias_index[spec.place_id.lower()] = spec.place_id
            for alias in spec.aliases:
                self._alias_index[str(alias).strip().lower()] = spec.place_id
        self.threshold = float(default_threshold)

    # ---- 构造 ----
    @classmethod
    def from_dict(cls, payload: typing.Mapping[str, typing.Any]) -> "PlaceResolver":
        raw = payload.get("places") or {}
        specs: list[PlaceSpec] = []
        if isinstance(raw, dict):
            for pid, meta in raw.items():
                meta = meta or {}
                specs.append(
                    PlaceSpec(
                        place_id=str(pid),
                        aliases=tuple(str(a) for a in (meta.get("aliases") or [])),
                        description=str(meta.get("description") or ""),
                    )
                )
        else:  # list 形式
            for meta in raw:
                pid = str(meta["place_id"])
                specs.append(
                    PlaceSpec(
                        place_id=pid,
                        aliases=tuple(str(a) for a in (meta.get("aliases") or [])),
                        description=str(meta.get("description") or ""),
                    )
                )
        return cls(specs, float(payload.get("match_threshold", 0.72)))

    @classmethod
    def from_file(cls, path: str | os.PathLike[str] = _DEFAULT_PATH) -> "PlaceResolver":
        """文件不存在时返回**空**解析器（不是报错），此时任何地名都会被拒绝。"""
        if not os.path.exists(path):
            return cls([])
        with open(path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))

    @classmethod
    def empty(cls) -> "PlaceResolver":
        return cls([])

    # ---- 查询 ----
    def __len__(self) -> int:
        return len(self._places)

    def legal_ids(self) -> list[str]:
        return sorted(self._places)

    def describe_all(self) -> list[dict[str, typing.Any]]:
        return [
            {"place_id": p.place_id, "aliases": list(p.aliases), "description": p.description}
            for p in self._places.values()
        ]

    def resolve(self, text: str | None) -> str | None:
        """成功返回 place_id；失败返回 None（调用方负责拒绝本次导航）。"""
        if not self._places or not isinstance(text, str):
            return None
        query = text.strip().lower()
        if not query:
            return None
        if query in self._alias_index:
            return self._alias_index[query]

        # 包含关系：'去前台' 命中 '前台'
        contained = [
            (len(alias), pid)
            for alias, pid in self._alias_index.items()
            if len(alias) >= 2 and (alias in query or query in alias)
        ]
        if contained:
            return max(contained)[1]

        # 模糊匹配兜底（错别字 / 口音转写）
        best_pid, best_score = None, 0.0
        for alias, pid in self._alias_index.items():
            score = SequenceMatcher(None, query, alias).ratio()
            if score > best_score:
                best_pid, best_score = pid, score
        if best_pid is not None and best_score >= self.threshold:
            return best_pid
        return None

    def candidates(self, text: str, top_k: int = 3) -> list[dict[str, typing.Any]]:
        """解析失败时给模型看的候选建议，避免它瞎猜。"""
        query = (text or "").strip().lower()
        scored = [
            {"place_id": pid, "score": round(SequenceMatcher(None, query, alias).ratio(), 3)}
            for alias, pid in self._alias_index.items()
        ]
        scored.sort(key=lambda item: item["score"], reverse=True)
        seen: set[str] = set()
        out: list[dict[str, typing.Any]] = []
        for item in scored:
            if item["place_id"] in seen:
                continue
            seen.add(item["place_id"])
            out.append(item)
            if len(out) >= top_k:
                break
        return out
