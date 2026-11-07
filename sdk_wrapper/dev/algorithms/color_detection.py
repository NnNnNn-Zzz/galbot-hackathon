"""algorithms/color_detection.py —— 纯 OpenCV 的颜色目标检测。

为什么先用颜色：比赛现场可控、零训练、CPU 实时（1280x1024 约 15-25 ms），
而且"把红色方块拿起来"这类赛题用颜色检索命中率极高。
需要更强语义时，把 `detect()` 换成 YOLO / GroundingDINO 实现即可，
`scene_analyzer` 只依赖 Detection 这个结构，不依赖具体算法（留好了替换点）。
"""

from __future__ import annotations

import typing
from dataclasses import dataclass, field

import cv2
import numpy as np

__all__ = [
    "Detection",
    "HSV_RANGES",
    "LABEL_ALIASES",
    "available_labels",
    "canonical_label",
    "detect",
    "detect_mask",
]

# 一个 H 区间用 ((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi)) 表示；红色跨 0 边界，用两段
_HSV = typing.Tuple[typing.Tuple[int, int, int], typing.Tuple[int, int, int]]

HSV_RANGES: dict[str, list[_HSV]] = {
    "red": [((0, 110, 60), (9, 255, 255)), ((170, 110, 60), (180, 255, 255))],
    "orange": [((10, 120, 70), (22, 255, 255))],
    "yellow": [((23, 110, 80), (34, 255, 255))],
    "green": [((35, 90, 50), (85, 255, 255))],
    "cyan": [((86, 90, 60), (100, 255, 255))],
    "blue": [((101, 100, 50), (130, 255, 255))],
    "purple": [((131, 70, 50), (155, 255, 255))],
    "magenta": [((156, 80, 60), (169, 255, 255))],
    "black": [((0, 0, 0), (180, 255, 60))],
    "white": [((0, 0, 205), (180, 45, 255))],
}

LABEL_ALIASES: dict[str, str] = {
    # 中文
    "红": "red", "红色": "red", "红的": "red",
    "橙": "orange", "橙色": "orange", "橘色": "orange",
    "黄": "yellow", "黄色": "yellow",
    "绿": "green", "绿色": "green",
    "青": "cyan", "青色": "cyan",
    "蓝": "blue", "蓝色": "blue",
    "紫": "purple", "紫色": "purple",
    "洋红": "magenta", "品红": "magenta",
    "黑": "black", "黑色": "black",
    "白": "white", "白色": "white",
    # 英文同义词
    "orange": "orange", "amber": "orange",
    "lime": "green", "teal": "cyan",
    "violet": "purple", "pink": "magenta",
    "grey": "white", "gray": "white",
    "colour": "red",
}


@dataclass
class Detection:
    """一个候选目标。mask 保留下来供 scene_analyzer 复用做深度统计（不重复阈值分割）。"""

    label: str
    mask: np.ndarray
    bbox: tuple[int, int, int, int]  # x, y, w, h
    centroid_uv: tuple[float, float]
    area_px: int
    source: str = "color"
    extras: dict[str, typing.Any] = field(default_factory=dict)

    def to_dict(self, image_area: int | None = None) -> dict[str, typing.Any]:
        x, y, w, h = self.bbox
        out: dict[str, typing.Any] = {
            "label": self.label,
            "source": self.source,
            "bbox_xywh": [int(x), int(y), int(w), int(h)],
            "centroid_uv": [round(self.centroid_uv[0], 1), round(self.centroid_uv[1], 1)],
            "area_px": int(self.area_px),
            "aspect": round(w / h, 3) if h else 0.0,
            "fill_ratio": round(self.area_px / float(w * h), 3) if w * h else 0.0,
        }
        if image_area:
            out["area_ratio"] = round(self.area_px / float(image_area), 5)
        out.update({k: v for k, v in self.extras.items() if k not in out})
        return out


def available_labels() -> list[str]:
    """可供大模型选择的颜色标签（不含别名）。"""
    return sorted(HSV_RANGES)


def canonical_label(label: str) -> str | None:
    """'红色' / 'RED ' / 'red' -> 'red'；无法识别返回 None。"""
    if not isinstance(label, str):
        return None
    key = label.strip().lower()
    if key in HSV_RANGES:
        return key
    return LABEL_ALIASES.get(key)


def detect_mask(bgr: np.ndarray, label: str) -> np.ndarray | None:
    """单标签 -> bool 掩膜。label 先经 canonical_label 归一化。"""
    canon = canonical_label(label)
    if canon is None:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lower, upper in HSV_RANGES[canon]:
        mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    return mask.astype(bool)


def detect(
    bgr: np.ndarray,
    labels: typing.Sequence[str] | None = None,
    *,
    min_area_px: int = 300,
    max_objects_per_label: int = 3,
    kernel: int = 5,
) -> list[Detection]:
    """在图中找指定（或全部）颜色的连通块，按面积降序。

    参数：
        labels: None = 遍历所有颜色（慢一点但省得 Agent 猜）；只查一个颜色时快得多。
        min_area_px: 过滤噪点。1280x1024 下 300 px 约等于 1.7 cm 的方块。
    """
    if bgr is None or bgr.size == 0:
        return []
    image_area = int(bgr.shape[0] * bgr.shape[1])

    if labels:
        targets: list[str] = []
        for label in labels:
            canon = canonical_label(label)
            if canon and canon not in targets:
                targets.append(canon)
    else:
        targets = available_labels()

    kernel_size = max(1, int(kernel) | 1)  # 保证是奇数
    morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    detections: list[Detection] = []

    for label in targets:
        mask = detect_mask(bgr, label)
        if mask is None:
            continue
        mask_u8 = mask.astype(np.uint8)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, morph)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, morph)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)

        label_dets: list[Detection] = []
        for idx in range(1, int(count)):  # 0 是背景
            x, y, w, h, area = (int(v) for v in stats[idx][:5])
            if area < min_area_px:
                continue
            if area > 0.6 * image_area:  # 整屏同色 = 场景照明，不是目标
                continue
            component = np.zeros_like(mask_u8, bool)
            component[y : y + h, x : x + w] = mask_u8[y : y + h, x : x + w].astype(bool)
            if not component.any():
                continue
            label_dets.append(
                Detection(
                    label=label, mask=component, bbox=(x, y, w, h),
                    centroid_uv=(float(centroids[idx][0]), float(centroids[idx][1])),
                    area_px=area, source="color",
                )
            )
        label_dets.sort(key=lambda d: d.area_px, reverse=True)
        detections.extend(label_dets[: max(1, int(max_objects_per_label))])

    detections.sort(key=lambda d: d.area_px, reverse=True)
    return detections
