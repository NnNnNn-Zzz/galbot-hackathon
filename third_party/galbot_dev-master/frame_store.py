"""帧缓存（FrameStore）：把"图"和"给大模型的引用 ID"解耦。

协议 §3.2 的要求：
    1. capture_scene 拿到真实帧，放进可信的本地帧缓存；
    2. 返回的图像/深度是**引用 ID**，不是模型可以随便拼的文件路径或 URL；
    3. app.py 通过可信的读取接口取出像素，再按模型 API 的格式喂进去。

所以本模块对外只承认两种引用形式：
    "<capture_id>:rgb"    -> RGB 帧
    "<capture_id>:depth"  -> 深度帧（float32 米）
capture_id 会被严格正则校验，任何 `..`、绝对路径、盘符都会被拒绝。
模型即便幻觉出 "C:/Windows/system32" 也读不到东西。
"""

from __future__ import annotations

import json
import os
import re
import secrets
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

__all__ = ["FrameStore", "CaptureRecord", "FrameRefError"]

# capture_id 只允许这些字符，长度受限，杜绝路径穿越
_ID_RE = re.compile(r"^cap_[0-9T]{15}_[A-Za-z0-9_]{1,24}_[0-9a-f]{4}$")
_REF_RE = re.compile(r"^(cap_[0-9T]{15}_[A-Za-z0-9_]{1,24}_[0-9a-f]{4}):(rgb|depth)$")


class FrameRefError(KeyError):
    """引用非法 / 已过期 —— 调用方应转成 rejected + invalid_argument。"""


@dataclass
class CaptureRecord:
    capture_id: str
    camera_id: str
    captured_at_unix: float
    rgb_path: str | None = None
    depth_path: str | None = None
    depth_preview_path: str | None = None
    meta: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "capture_id": self.capture_id,
            "camera_id": self.camera_id,
            "captured_at_unix": round(self.captured_at_unix, 6),
            "rgb_ref": f"{self.capture_id}:rgb" if self.rgb_path else None,
            "depth_ref": f"{self.capture_id}:depth" if self.depth_path else None,
            "meta": self.meta or {},
        }


class FrameStore:
    """落盘 + 内存索引的帧仓库。线程安全（Agent 循环和工具可能不同线程）。"""

    def __init__(self, root: str | os.PathLike[str], max_captures: int = 200) -> None:
        self.root = os.path.abspath(os.fspath(root))
        os.makedirs(self.root, exist_ok=True)
        self.max_captures = int(max_captures)
        self._lock = threading.RLock()
        self._records: dict[str, CaptureRecord] = {}
        self._order: list[str] = []  # 插入顺序，用于淘汰

    # ------------------------------------------------------------------ #
    # 写
    # ------------------------------------------------------------------ #
    def new_capture(self, camera_id: str) -> CaptureRecord:
        """分配一个新的 capture_id 并建目录。不落任何像素，先占位。"""
        stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime())
        safe_cam = re.sub(r"[^A-Za-z0-9_]", "_", camera_id)[:24] or "cam"
        capture_id = f"cap_{stamp}_{safe_cam}_{secrets.token_hex(2)}"
        rec = CaptureRecord(
            capture_id=capture_id, camera_id=camera_id, captured_at_unix=time.time()
        )
        os.makedirs(os.path.join(self.root, capture_id), exist_ok=True)
        with self._lock:
            self._records[capture_id] = rec
            self._order.append(capture_id)
        return rec

    def attach_rgb_png(self, rec: CaptureRecord, bgr: np.ndarray) -> str:
        """保存 RGB（OpenCV 的 BGR 布局）为 PNG。无外部依赖的 PNG 编码。"""
        import cv2  # 局部导入：只有真正要落盘时才需要 opencv

        path = os.path.join(self.root, rec.capture_id, "rgb.png")
        if not cv2.imwrite(path, bgr):
            raise FrameRefError(f"写入 RGB 失败: {path}")
        rec.rgb_path = path
        return path

    def attach_depth_npy(self, rec: CaptureRecord, depth_m: np.ndarray) -> tuple[str, str]:
        """保存深度：`.npy` 保精度（float32 米，NaN=无效），另存一张伪彩预览图给人看。

        ⚠️ 预览图只用于 debug，**不要**拿它做测距——它的量纲是 0..255 的归一化伪彩。
        """
        import cv2

        folder = os.path.join(self.root, rec.capture_id)
        depth_path = os.path.join(folder, "depth_m.npy")
        np.save(depth_path, depth_m.astype(np.float32, copy=False))
        rec.depth_path = depth_path

        valid = np.isfinite(depth_m) & (depth_m > 0)
        preview_path = os.path.join(folder, "depth_preview.png")
        if valid.any():
            lo, hi = float(np.percentile(depth_m[valid], 2)), float(np.percentile(depth_m[valid], 98))
            if hi <= lo:
                hi = lo + 1e-3
            norm = np.clip((np.nan_to_num(depth_m, nan=hi) - lo) / (hi - lo), 0, 1)
            preview = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            cv2.imwrite(preview_path, preview)
            rec.depth_preview_path = preview_path
        return depth_path, preview_path

    def update_meta(self, rec: CaptureRecord, meta: dict[str, Any], *, persist: bool = True) -> None:
        """更新登记信息。``persist=False`` 时只留在内存（``--no-save-frames`` 用）。"""
        rec.meta = dict(meta)
        if persist:
            path = os.path.join(self.root, rec.capture_id, "meta.json")
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(rec.to_dict(), fh, ensure_ascii=False, indent=2, default=str)
        self._evict_if_needed()

    # ------------------------------------------------------------------ #
    # 读
    # ------------------------------------------------------------------ #
    def record(self, capture_id: str) -> CaptureRecord:
        with self._lock:
            rec = self._records.get(capture_id)
        if rec is None:
            raise FrameRefError(f"未知或已过期的 capture_id: {capture_id!r}")
        return rec

    def parse_ref(self, ref: str) -> tuple[CaptureRecord, str]:
        """'cap_xxx:rgb' -> (record, 'rgb')。非法输入直接抛 FrameRefError。"""
        if not isinstance(ref, str):
            raise FrameRefError(f"引用必须是字符串，收到 {type(ref).__name__}")
        match = _REF_RE.match(ref.strip())
        if match is None:
            raise FrameRefError(
                f"非法帧引用 {ref!r}；合法形式为 '<capture_id>:rgb' 或 '<capture_id>:depth'"
            )
        return self.record(match.group(1)), match.group(2)

    def load_rgb(self, ref: str) -> np.ndarray:
        import cv2

        rec, kind = self.parse_ref(ref)
        if kind != "rgb":
            raise FrameRefError(f"{ref!r} 不是 RGB 引用")
        if not rec.rgb_path or not os.path.exists(rec.rgb_path):
            raise FrameRefError(f"{ref!r} 没有 RGB 数据")
        img = cv2.imread(rec.rgb_path, cv2.IMREAD_COLOR)
        if img is None:
            raise FrameRefError(f"{ref!r} RGB 解码失败")
        return img

    def load_depth(self, ref: str) -> np.ndarray:
        rec, kind = self.parse_ref(ref)
        if kind != "depth":
            raise FrameRefError(f"{ref!r} 不是深度引用")
        if not rec.depth_path or not os.path.exists(rec.depth_path):
            raise FrameRefError(f"{ref!r} 没有深度数据")
        return np.load(rec.depth_path)

    def latest(self, camera_id: str | None = None) -> CaptureRecord | None:
        with self._lock:
            for cid in reversed(self._order):
                rec = self._records.get(cid)
                if rec is not None and (camera_id is None or rec.camera_id == camera_id):
                    return rec
        return None

    def list_recent(self, limit: int = 10) -> list[dict[str, Any]]:
        with self._lock:
            ids = self._order[-limit:]
            return [self._records[i].to_dict() for i in ids if i in self._records]

    # ------------------------------------------------------------------ #
    # 淘汰
    # ------------------------------------------------------------------ #
    def _evict_if_needed(self) -> None:
        import shutil

        with self._lock:
            while len(self._order) > self.max_captures:
                old = self._order.pop(0)
                self._records.pop(old, None)
                shutil.rmtree(os.path.join(self.root, old), ignore_errors=True)
