"""algorithms/scene_analyzer.py —— 把"像素"翻译成"大模型能读的场景"。

这是 B 真正交付价值的地方：模型看不到 numpy 数组，只能读到几十个 token 的结论。
所以本模块的输出必须：
  * 小（默认 ≤8 个目标，无大数组）；
  * 自带单位（字段名带 _m，显式 depth_unit）；
  * 自带坐标系（每个 3D 点都说明在哪个 frame）；
  * 会把"没把握"说出来（warnings + valid_ratio），绝不假装成功。

设计上刻意与检测算法解耦：`color_detection.Detection` 是唯一接口，
换成 YOLO / Grounding DINO / 自训练模型时，本文件不需要改。
"""

from __future__ import annotations

import typing
from dataclasses import dataclass

import cv2
import numpy as np

import algorithms.color_detection as color_detection
import algorithms.geometry as geometry

__all__ = [
    "FOREGROUND_ALIASES",
    "is_known_label",
    "legal_labels",
    "analyze",
    "find_one",
    "nearest_depth_band",
]

#: 表示"不按颜色、按最近的前景找"的标签
FOREGROUND_ALIASES: frozenset[str] = frozenset(
    {"foreground", "nearest", "front", "object", "any", "物体", "前景", "最近", "最前面", "前面", "目标"}
)

_DEFAULT_OPTIONS: dict[str, typing.Any] = {
    "min_area_px": 300,
    "max_objects_per_label": 3,
    "max_objects": 8,
    "max_fusion_depth_m": 3.0,
    "erode_px": 3,
    "sort": "nearest",  # nearest | area
}


def _options(overrides: typing.Mapping[str, typing.Any] | None) -> dict[str, typing.Any]:
    merged = dict(_DEFAULT_OPTIONS)
    if overrides:
        merged.update({k: v for k, v in overrides.items() if v is not None})
    return merged


def is_known_label(label: str | None) -> bool:
    if not isinstance(label, str) or not label.strip():
        return False
    key = label.strip().lower()
    return key in FOREGROUND_ALIASES or color_detection.canonical_label(key) is not None


def _legal_labels_hint() -> list[str]:
    return sorted(set(color_detection.available_labels()) | {"foreground"})


def legal_labels() -> list[str]:
    """给大模型/调用方看的合法 label 列表。"""
    return _legal_labels_hint()


# ═══════════════════════════════════════════════════════════════════════════ #
# 深度辅助
# ═══════════════════════════════════════════════════════════════════════════ #
def nearest_depth_band(
    depth_m: np.ndarray, *, bin_m: float = 0.05, min_ratio: float = 0.02, band_m: float = 0.08
) -> tuple[np.ndarray, float] | None:
    """找出"最近的一块显著表面"的掩膜（例如桌上离我最近的物体）。

    做法：有效深度直方图 -> 从近往远找第一个占比超过 min_ratio 的 bin
          -> 取该 bin ±band_m 的所有像素。
    比"取全局最小值"稳得多：全局最小值往往是一个噪声点或一根线缆。
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = geometry.depth_valid_mask(depth)
    if not valid.any():
        return None
    values = depth[valid]
    lo = float(np.percentile(values, 0.5))
    hi = float(np.percentile(values, 99.5))
    if hi <= lo:
        return None

    step = max(1e-3, float(bin_m))
    counts, edges = np.histogram(values, bins=max(2, int((hi - lo) / step) + 1), range=(lo, hi))
    threshold = max(3, int(counts.sum() * float(min_ratio)))
    for idx, count in enumerate(counts):
        if count >= threshold:
            center = float((edges[idx] + edges[idx + 1]) / 2)
            band = np.abs(depth - center) <= float(band_m)
            mask = valid & band
            if mask.sum() >= threshold:
                return mask, center
    return None


def _mask_to_detection(
    mask: np.ndarray, label: str, *, source: str
) -> color_detection.Detection | None:
    mask_u8 = mask.astype(np.uint8)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    best: color_detection.Detection | None = None
    for idx in range(1, int(count)):
        x, y, w, h, area = (int(v) for v in stats[idx][:5])
        if area <= 0:
            continue
        if best is not None and area <= best.area_px:
            continue
        component = np.zeros_like(mask_u8, bool)
        component[y : y + h, x : x + w] = mask_u8[y : y + h, x : x + w].astype(bool)
        best = color_detection.Detection(
            label=label, mask=component, bbox=(x, y, w, h),
            centroid_uv=(float(centroids[idx][0]), float(centroids[idx][1])),
            area_px=area, source=source,
        )
    return best


def _depth_of_mask(
    depth_m: np.ndarray, mask: np.ndarray, erode_px: int
) -> dict[str, typing.Any]:
    """掩膜内的稳健深度。先腐蚀去掉边缘（边缘会混进背景）。"""
    work = mask.astype(np.uint8)
    k = int(erode_px)
    if k > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
        eroded = cv2.erode(work, kernel)
        if eroded.sum() >= max(16, 0.2 * work.sum()):
            work = eroded
    return geometry.depth_stats(depth_m, work.astype(bool))


def _fuse_3d(
    detection: color_detection.Detection,
    depth_m: np.ndarray | None,
    intrinsics: typing.Any | None,
    options: dict[str, typing.Any],
) -> dict[str, typing.Any]:
    """给一个 2D 检测补上 3D 信息。任一步骤不成立就显式说明原因。"""
    info: dict[str, typing.Any] = {
        "depth_m": None,
        "center_cam_m": None,
        "coordinate_frame": None,
        "depth_stats": None,
        "fusion": "none",
    }
    if depth_m is None:
        info["fusion"] = "no_depth_sensor"
        return info
    if depth_m.shape[:2] != detection.mask.shape[:2]:
        info["fusion"] = (
            f"shape_mismatch:depth{depth_m.shape[1]}x{depth_m.shape[0]}"
            f"_rgb{detection.mask.shape[1]}x{detection.mask.shape[0]}"
        )
        return info

    stats = _depth_of_mask(depth_m, detection.mask, options.get("erode_px", 3))
    info["depth_stats"] = stats
    if not stats.get("valid"):
        info["fusion"] = "no_valid_depth_in_mask"
        return info

    z = float(stats["median_m"])
    info["depth_m"] = round(z, 4)
    if intrinsics is not None and getattr(intrinsics, "fx", 0):
        point = geometry.deproject(detection.centroid_uv[0], detection.centroid_uv[1], z, intrinsics)
        info["center_cam_m"] = [round(float(v), 4) for v in point]
        info["coordinate_frame"] = getattr(intrinsics, "frame_id", "") or "camera_optical_frame"
        info["fusion"] = "camera_frame"
    else:
        info["fusion"] = "depth_only_no_intrinsics"
    return info


# ═══════════════════════════════════════════════════════════════════════════ #
# 主入口
# ═══════════════════════════════════════════════════════════════════════════ #
def analyze(
    bgr: np.ndarray | None,
    depth_m: np.ndarray | None,
    intrinsics: typing.Any | None,
    want_labels: typing.Sequence[str] | None = None,
    max_objects: int | None = None,
    options: typing.Mapping[str, typing.Any] | None = None,
) -> dict[str, typing.Any]:
    """一次分析出结构化场景。bgr / depth 至少给一个。"""
    opts = _options(options)
    warnings: list[str] = []
    result: dict[str, typing.Any] = {
        "width": int(bgr.shape[1]) if bgr is not None else (int(depth_m.shape[1]) if depth_m is not None else None),
        "height": int(bgr.shape[0]) if bgr is not None else (int(depth_m.shape[0]) if depth_m is not None else None),
        "depth_unit": "m" if depth_m is not None else None,
        "coordinate_frame": getattr(intrinsics, "frame_id", None) or None,
        "objects": [],
        "depth_summary": None,
        "warnings": warnings,
        "intrinsics": None,
        "summary": "",
    }
    if bgr is None and depth_m is None:
        warnings.append("既没有彩色图也没有深度图，无法分析")
        result["summary"] = "没有可分析的图像数据"
        return result
    if intrinsics is not None:
        try:
            result["intrinsics"] = intrinsics.to_dict() if hasattr(intrinsics, "to_dict") else None
        except Exception:  # noqa: BLE001
            result["intrinsics"] = None
    if depth_m is not None:
        result["depth_summary"] = geometry.depth_stats(depth_m)
        if intrinsics is not None:
            result["coordinate_frame"] = result["coordinate_frame"] or getattr(intrinsics, "frame_id", None)

    # ---- 1. 2D 检测 ----
    detections: list[color_detection.Detection] = []
    if bgr is not None:
        labels = list(want_labels) if want_labels else None
        detections = color_detection.detect(
            bgr,
            labels,
            min_area_px=int(opts["min_area_px"]),
            max_objects_per_label=int(opts["max_objects_per_label"]),
        )
        # 没指定颜色且颜色也没命中 -> 退回"最近前景"，保证永远有可用的候选
        if not detections and labels:
            unknown = [l for l in labels if color_detection.canonical_label(l) is None]
            if unknown and depth_m is not None and any(str(l).lower() in FOREGROUND_ALIASES for l in unknown):
                detections.extend(_foreground_detections(depth_m, opts, warnings))

    # ---- 2. 3D 融合 ----
    fused: list[dict[str, typing.Any]] = []
    for det in detections:
        info = _fuse_3d(det, depth_m, intrinsics, opts)
        entry = det.to_dict(image_area=(result["width"] or 0) * (result["height"] or 0))
        entry.update(info)
        fused.append(entry)

    # ---- 3. 排序 / 截断 ----
    if opts.get("sort") == "nearest" and any(e.get("depth_m") for e in fused):
        fused.sort(key=lambda e: (e.get("depth_m") is None, e.get("depth_m") or 1e9))
    else:
        fused.sort(key=lambda e: e["area_px"], reverse=True)
    limit = int(max_objects if max_objects is not None else opts["max_objects"])
    dropped = max(0, len(fused) - limit)
    fused = fused[: max(1, limit)]
    for idx, entry in enumerate(fused, start=1):
        entry["id"] = f"obj{idx}"
    result["objects"] = fused
    if dropped:
        warnings.append(f"另有 {dropped} 个目标未列出（max_objects={limit}）")

    # ---- 4. 人话总结 ----
    result["summary"] = _summarize(result, opts)
    return result


def _foreground_detections(
    depth_m: np.ndarray, opts: dict[str, typing.Any], warnings: list[str]
) -> list[color_detection.Detection]:
    found = nearest_depth_band(depth_m)
    if found is None:
        warnings.append("深度图里没有找到显著前景")
        return []
    mask, center = found
    det = _mask_to_detection(mask, "foreground", source="depth_band")
    if det is None:
        return []
    det.extras["band_center_m"] = round(center, 3)
    return [det]


def _summarize(result: dict[str, typing.Any], opts: dict[str, typing.Any]) -> str:
    objects = result.get("objects") or []
    if not objects:
        base = "视野内未检测到目标"
        if result.get("depth_summary") and result["depth_summary"].get("valid"):
            base += f"；有效深度中位数 {result['depth_summary']['median_m']} m"
        return base
    parts: list[str] = []
    for entry in objects:
        label = entry["label"]
        if entry.get("depth_m") is not None:
            parts.append(f"{label} @ {entry['depth_m']} m（像素 {entry['centroid_uv']}）")
        else:
            parts.append(f"{label}（像素 {entry['centroid_uv']}，距离未知）")
    return f"检测到 {len(objects)} 个目标：" + "；".join(parts)


# ═══════════════════════════════════════════════════════════════════════════ #
# 单目标检索
# ═══════════════════════════════════════════════════════════════════════════ #
def find_one(
    bgr: np.ndarray | None,
    depth_m: np.ndarray | None,
    intrinsics: typing.Any | None,
    label: str,
    max_distance_m: float = 1.5,
    options: typing.Mapping[str, typing.Any] | None = None,
) -> dict[str, typing.Any] | None:
    """按颜色或"最近前景"找**一个**最合适的目标。

    打分：在 max_distance_m 内的优先，其次面积大（检测更可信）。
    """
    if not is_known_label(label):
        raise ValueError(
            f"不认识的 label {label!r}；可用：{_legal_labels_hint()}"
        )
    opts = _options(options)
    key = label.strip().lower()
    warnings: list[str] = []

    if key in FOREGROUND_ALIASES:
        if depth_m is None:
            return None
        dets = _foreground_detections(depth_m, opts, warnings)
    else:
        if bgr is None:
            return None
        dets = color_detection.detect(
            bgr, [key],
            min_area_px=int(opts["min_area_px"]),
            max_objects_per_label=int(opts["max_objects_per_label"]),
        )
    if not dets:
        return None

    scored: list[tuple[float, color_detection.Detection, dict[str, typing.Any]]] = []
    for det in dets:
        info = _fuse_3d(det, depth_m, intrinsics, opts)
        depth = info.get("depth_m")
        if depth is not None and depth > float(max_distance_m):
            continue  # 明确超出可及范围的目标不参与
        score = float(det.area_px) / 1000.0
        if depth is not None:
            score += max(0.0, (float(max_distance_m) - depth)) * 10.0
        scored.append((score, det, info))

    if not scored:
        return None
    scored.sort(key=lambda item: item[0], reverse=True)
    _, best, info = scored[0]
    payload = best.to_dict()
    payload.update(info)
    payload["label_requested"] = label
    if warnings:
        payload["warnings"] = warnings
    return payload
