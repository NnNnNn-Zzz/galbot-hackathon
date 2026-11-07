"""Head-camera power-strip box analysis for coarse approach only.

The far detector answers "which direction is the strip?" It deliberately does
not promote the bbox center to a metric grasp target. Stereo/depth validation
belongs to the next tool stage and must qualify the measurement before any
base translation uses metric distance.
"""

from __future__ import annotations

import math
import typing

import cv2
import numpy as np

from sdk_wrapper.dev.algorithms.charger_yolo import YoloInstance

__all__ = [
    "analyze_power_strip_box",
    "analyze_power_strip_stereo",
    "draw_power_strip_overlay",
]


def _intrinsics_values(intrinsics: typing.Any) -> tuple[float, float, float, float] | None:
    if intrinsics is None:
        return None
    if isinstance(intrinsics, dict):
        if "K" in intrinsics:
            k = list(intrinsics["K"])
            return float(k[0]), float(k[4]), float(k[2]), float(k[5])
        return (
            float(intrinsics["fx"]),
            float(intrinsics["fy"]),
            float(intrinsics["cx"]),
            float(intrinsics["cy"]),
        )
    return (
        float(intrinsics.fx),
        float(intrinsics.fy),
        float(intrinsics.cx),
        float(intrinsics.cy),
    )


def analyze_power_strip_box(
    instance: YoloInstance,
    image_shape: typing.Sequence[int],
    *,
    intrinsics: typing.Any = None,
    min_confidence: float = 0.30,
    min_width_px: float = 12.0,
    min_height_px: float = 8.0,
    edge_margin_px: int = 3,
) -> dict[str, typing.Any]:
    """Return a coarse, image-space strip observation.

    Even a successful result remains a direction/approach candidate. The tool
    output never labels its bbox center as a metric grasp coordinate.
    """
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height, width = int(image_shape[0]), int(image_shape[1])
    x1, y1, x2, y2 = [float(v) for v in instance.bbox_xyxy]
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
        raise ValueError("bbox contains non-finite values")
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"invalid bbox {instance.bbox_xyxy!r}")

    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bw, bh = x2 - x1, y2 - y1
    margin = max(0, int(edge_margin_px))
    touched = []
    if x1 <= margin:
        touched.append("left")
    if y1 <= margin:
        touched.append("top")
    if x2 >= width - 1 - margin:
        touched.append("right")
    if y2 >= height - 1 - margin:
        touched.append("bottom")

    reasons: list[str] = []
    if instance.confidence < float(min_confidence):
        reasons.append("confidence_below_minimum")
    if bw < float(min_width_px):
        reasons.append("bbox_width_below_minimum")
    if bh < float(min_height_px):
        reasons.append("bbox_height_below_minimum")
    if touched:
        reasons.append("bbox_touches_image_edge")

    dx, dy = cx - (width - 1) / 2.0, cy - (height - 1) / 2.0
    bearing_camera: list[float] | None = None
    horizontal_angle_deg: float | None = None
    vertical_angle_deg: float | None = None
    kv = _intrinsics_values(intrinsics)
    if kv is not None:
        fx, fy, icx, icy = kv
        dx, dy = cx - icx, cy - icy
        ray = np.array([dx / fx, dy / fy, 1.0], dtype=np.float64)
        ray /= np.linalg.norm(ray)
        bearing_camera = [round(float(v), 8) for v in ray]
        horizontal_angle_deg = round(math.degrees(math.atan2(float(ray[0]), float(ray[2]))), 4)
        vertical_angle_deg = round(math.degrees(math.atan2(float(ray[1]), float(ray[2]))), 4)

    area_px = bw * bh
    result = {
        "usable_for_direction": not reasons,
        "usable_for_metric_distance": False,
        "rejection_reasons": reasons,
        "class_name": instance.class_name,
        "class_id": int(instance.class_id),
        "confidence": round(float(instance.confidence), 6),
        "image_width": width,
        "image_height": height,
        "bbox_xyxy": [round(v, 3) for v in (x1, y1, x2, y2)],
        "bbox_width_px": round(bw, 3),
        "bbox_height_px": round(bh, 3),
        "bbox_area_px": round(area_px, 3),
        "bbox_area_ratio": round(area_px / float(width * height), 8),
        "bbox_center_uv": [round(cx, 3), round(cy, 3)],
        "image_center_offset_px": [round(dx, 3), round(dy, 3)],
        "bearing_camera_optical": bearing_camera,
        "horizontal_angle_deg": horizontal_angle_deg,
        "vertical_angle_deg": vertical_angle_deg,
        "touches_image_edge": bool(touched),
        "touched_edges": touched,
        "fully_visible": not touched,
        "metric_target_base_m": None,
        "grasp_coordinate": None,
        "semantics": "coarse direction only; depth/stereo qualification required",
    }
    return result


def analyze_power_strip_stereo(
    depth_raw: np.ndarray,
    bbox_xyxy: typing.Sequence[float],
    rgb_shape: typing.Sequence[int],
    *,
    rgb_timestamp_ns: int = 0,
    depth_timestamp_ns: int = 0,
    raw_to_m: float | None = None,
    min_valid_ratio: float = 0.60,
    max_relative_spread: float = 0.35,
    max_timestamp_skew_ms: float = 300.0,
) -> dict[str, typing.Any]:
    """Measure stability and optional metric range inside a detector bbox.

    ``FOUNDATION_STEREO`` units remain undocumented. Without ``raw_to_m`` the
    result is useful only as a stability signal, never as a navigation range.
    """
    depth = np.asarray(depth_raw)
    if depth.ndim != 2 or depth.size == 0:
        return {"available": False, "reason": "invalid_stereo_depth_shape"}
    if len(rgb_shape) < 2:
        raise ValueError("rgb_shape must contain height and width")
    rgb_h, rgb_w = int(rgb_shape[0]), int(rgb_shape[1])
    depth_h, depth_w = depth.shape[:2]
    x1, y1, x2, y2 = [float(v) for v in bbox_xyxy]
    sx, sy = depth_w / float(rgb_w), depth_h / float(rgb_h)
    dx1 = max(0, min(depth_w - 1, int(math.floor(x1 * sx))))
    dy1 = max(0, min(depth_h - 1, int(math.floor(y1 * sy))))
    dx2 = max(dx1 + 1, min(depth_w, int(math.ceil(x2 * sx))))
    dy2 = max(dy1 + 1, min(depth_h, int(math.ceil(y2 * sy))))
    crop = depth[dy1:dy2, dx1:dx2].astype(np.float64)
    valid = np.isfinite(crop) & (crop > 0)
    values = crop[valid]
    valid_ratio = float(valid.mean()) if valid.size else 0.0
    skew_ms = None
    if int(rgb_timestamp_ns) > 0 and int(depth_timestamp_ns) > 0:
        skew_ms = abs(int(depth_timestamp_ns) - int(rgb_timestamp_ns)) / 1_000_000.0
    reasons: list[str] = []
    if values.size < 3:
        reasons.append("insufficient_valid_stereo_pixels")
        values = np.asarray([], dtype=np.float64)
    if valid_ratio < float(min_valid_ratio):
        reasons.append("stereo_valid_ratio_below_minimum")
    if values.size:
        p10, median, p90 = np.percentile(values, [10, 50, 90])
        relative_spread = float((p90 - p10) / max(median, 1e-12))
    else:
        p10 = median = p90 = math.nan
        relative_spread = math.inf
    if relative_spread > float(max_relative_spread):
        reasons.append("stereo_relative_spread_too_large")
    if skew_ms is not None and skew_ms > float(max_timestamp_skew_ms):
        reasons.append("rgb_stereo_timestamp_skew_too_large")

    metric_distance = None
    if raw_to_m is not None and math.isfinite(float(raw_to_m)) and values.size:
        metric_distance = float(median) * float(raw_to_m)
    return {
        "available": values.size > 0,
        "measurement_stable": not reasons,
        "reasons": reasons,
        "unit": "m" if metric_distance is not None else "sdk_raw_unverified",
        "metric_distance_m": metric_distance,
        "metric_distance_available": metric_distance is not None,
        "raw_median": None if not values.size else float(median),
        "raw_p10": None if not values.size else float(p10),
        "raw_p90": None if not values.size else float(p90),
        "relative_spread": (
            None if not math.isfinite(relative_spread) else round(relative_spread, 6)
        ),
        "valid_ratio": round(valid_ratio, 6),
        "rgb_bbox_xyxy": [round(v, 3) for v in (x1, y1, x2, y2)],
        "depth_bbox_xyxy": [dx1, dy1, dx2, dy2],
        "rgb_shape": [rgb_h, rgb_w],
        "depth_shape": [depth_h, depth_w],
        "rgb_timestamp_ns": int(rgb_timestamp_ns),
        "depth_timestamp_ns": int(depth_timestamp_ns),
        "timestamp_skew_ms": None if skew_ms is None else round(skew_ms, 3),
    }


def draw_power_strip_overlay(
    bgr: np.ndarray,
    instance: YoloInstance,
    analysis: typing.Mapping[str, typing.Any],
) -> np.ndarray:
    out = np.asarray(bgr).copy()
    x1, y1, x2, y2 = [int(round(v)) for v in instance.bbox_xyxy]
    color = (0, 220, 0) if analysis["usable_for_direction"] else (0, 0, 255)
    cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
    center = tuple(int(round(v)) for v in analysis["bbox_center_uv"])
    image_center = (out.shape[1] // 2, out.shape[0] // 2)
    cv2.arrowedLine(out, image_center, center, (0, 180, 255), 2, tipLength=0.15)
    cv2.circle(out, center, 4, (255, 255, 255), -1)
    cv2.putText(
        out,
        "power_strip %.3f %s" % (
            float(instance.confidence),
            "DIRECTION" if analysis["usable_for_direction"] else "REJECT",
        ),
        (max(0, x1), max(16, y1 - 5)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        color,
        1,
        cv2.LINE_AA,
    )
    return out
