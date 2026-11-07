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
from sdk_wrapper.dev.algorithms import geometry as geo

__all__ = [
    "analyze_power_strip_box",
    "analyze_power_strip_stereo",
    "validate_head_target_3d",
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
        "track_id": instance.track_id,
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
    rgb_intrinsics: typing.Any = None,
    base_from_camera: typing.Sequence[float] | None = None,
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
    target_3d: dict[str, typing.Any] = {"available": False}
    if (
        metric_distance is not None
        and rgb_intrinsics is not None
        and base_from_camera is not None
        and values.size >= 3
    ):
        kv = _intrinsics_values(rgb_intrinsics)
        if kv is not None:
            fx, fy, cx, cy = kv
            fx, cx = fx * sx, cx * sx
            fy, cy = fy * sy, cy * sy
            if isinstance(rgb_intrinsics, dict):
                distortion_model = str(
                    rgb_intrinsics.get("distortion_model") or "plumb_bob"
                )
                distortion = [
                    float(v) for v in (rgb_intrinsics.get("D") or [])
                ]
            else:
                distortion_model = str(
                    getattr(rgb_intrinsics, "distortion_model", "plumb_bob")
                )
                distortion = [
                    float(v)
                    for v in getattr(rgb_intrinsics, "distortion", ())
                ]
            ys, xs = np.nonzero(valid)
            all_z = crop[ys, xs].astype(np.float64) * float(raw_to_m)
            seed_limit = float(np.percentile(all_z, 35))
            seed = all_z[all_z <= seed_limit]
            seed_center = float(np.median(seed))
            seed_mad = float(np.median(np.abs(seed - seed_center)))
            tolerance = max(0.05 * seed_center, 3.0 * 1.4826 * seed_mad)
            inlier = np.abs(all_z - seed_center) <= tolerance
            ys, xs, all_z = ys[inlier], xs[inlier], all_z[inlier]
            if all_z.size >= 3:
                gx = xs.astype(np.float64) + dx1
                gy = ys.astype(np.float64) + dy1
                normalized_x = (gx - cx) / fx
                normalized_y = (gy - cy) / fy
                if (
                    distortion_model.lower() in {"kannala_brandt", "fisheye"}
                    and distortion
                ):
                    raw_points = np.column_stack(
                        [normalized_x, normalized_y]
                    ).reshape(-1, 1, 2)
                    normalized = cv2.fisheye.undistortPoints(
                        raw_points.astype(np.float64),
                        np.asarray(
                            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                            dtype=np.float64,
                        ),
                        np.asarray(distortion, dtype=np.float64),
                    ).reshape(-1, 2)
                    normalized_x = normalized[:, 0]
                    normalized_y = normalized[:, 1]
                cam_points = np.column_stack(
                    [normalized_x * all_z, normalized_y * all_z, all_z]
                )
                base_points = geo.transform_points(cam_points, base_from_camera)
                center = np.median(base_points, axis=0)
                centered = base_points - center
                covariance = centered.T @ centered / max(1, len(centered) - 1)
                eigvals, eigvecs = np.linalg.eigh(covariance)
                normal = eigvecs[:, int(np.argmin(eigvals))]
                if normal[2] < 0:
                    normal = -normal
                normal /= max(float(np.linalg.norm(normal)), 1e-12)
                long_axis = eigvecs[:, int(np.argmax(eigvals))]
                long_axis = long_axis - float(long_axis @ normal) * normal
                long_axis /= max(float(np.linalg.norm(long_axis)), 1e-12)
                short_axis = np.cross(normal, long_axis)
                projected_long = centered @ long_axis
                projected_short = centered @ short_axis
                target_3d = {
                    "available": True,
                    "frame": "base_link",
                    "center_m": [round(float(v), 6) for v in center],
                    "surface_normal_base": [round(float(v), 8) for v in normal],
                    "long_axis_base": [round(float(v), 8) for v in long_axis],
                    "short_axis_base": [round(float(v), 8) for v in short_axis],
                    "long_dimension_m": round(
                        float(np.percentile(projected_long, 99) - np.percentile(projected_long, 1)),
                        6,
                    ),
                    "short_dimension_m": round(
                        float(np.percentile(projected_short, 99) - np.percentile(projected_short, 1)),
                        6,
                    ),
                    "plane_rms_m": round(
                        float(np.sqrt(np.mean((centered @ normal) ** 2))), 6
                    ),
                    "points": int(len(base_points)),
                }
    return {
        "available": values.size > 0,
        "measurement_stable": not reasons,
        "reasons": reasons,
        "unit": "m" if metric_distance is not None else "sdk_raw_unverified",
        "metric_distance_m": metric_distance,
        "metric_distance_available": metric_distance is not None,
        "target_3d": target_3d,
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


def validate_head_target_3d(
    target_3d: typing.Mapping[str, typing.Any] | None,
    *,
    min_range_m: float = 0.45,
    max_range_m: float = 4.0,
    min_z_m: float = -0.05,
    max_z_m: float = 1.6,
    max_plane_rms_m: float = 0.04,
    max_normal_tilt_deg: float = 35.0,
) -> dict[str, typing.Any]:
    """Safety-check a target point already transformed into ``base_link``."""
    reasons: list[str] = []
    if not target_3d or not target_3d.get("available"):
        return {
            "safe": False,
            "reasons": ["target_3d_unavailable"],
            "target_center_m": None,
        }
    center = [float(v) for v in target_3d.get("center_m") or []]
    normal = [float(v) for v in target_3d.get("surface_normal_base") or []]
    if len(center) != 3 or not all(math.isfinite(v) for v in center):
        reasons.append("target_center_invalid")
        center = []
    if len(normal) != 3 or not all(math.isfinite(v) for v in normal):
        reasons.append("target_normal_invalid")
        normal = []
    if center:
        horizontal_range = math.hypot(center[0], center[1])
        if horizontal_range < float(min_range_m):
            reasons.append("target_too_close")
        if horizontal_range > float(max_range_m):
            reasons.append("target_too_far")
        if center[2] < float(min_z_m) or center[2] > float(max_z_m):
            reasons.append("target_height_outside_safe_band")
    else:
        horizontal_range = None
    if float(target_3d.get("plane_rms_m") or 0.0) > float(max_plane_rms_m):
        reasons.append("target_plane_instability")
    normal_tilt_deg = None
    if normal:
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-9:
            reasons.append("target_normal_invalid")
        else:
            n = np.asarray(normal, dtype=np.float64) / normal_norm
            normal_tilt_deg = float(math.degrees(math.acos(max(-1.0, min(1.0, n[2])))))
            if normal_tilt_deg > float(max_normal_tilt_deg):
                reasons.append("target_normal_not_upward_enough")
    return {
        "safe": not reasons,
        "reasons": reasons,
        "target_center_m": center or None,
        "horizontal_range_m": (
            None if horizontal_range is None else round(float(horizontal_range), 6)
        ),
        "normal_tilt_deg": (
            None if normal_tilt_deg is None else round(float(normal_tilt_deg), 3)
        ),
        "limits": {
            "min_range_m": float(min_range_m),
            "max_range_m": float(max_range_m),
            "min_z_m": float(min_z_m),
            "max_z_m": float(max_z_m),
            "max_plane_rms_m": float(max_plane_rms_m),
            "max_normal_tilt_deg": float(max_normal_tilt_deg),
        },
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
