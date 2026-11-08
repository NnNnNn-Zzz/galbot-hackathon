"""YOLO-seg charger-top recognition and geometry validation.

This module deliberately stops before robot motion. It turns a YOLO
segmentation instance into a structured observation:

* image-space completeness and mask geometry;
* depth coverage inside the mask;
* a robust top-surface plane, center, normal, long edge and short edge in
  ``base_link`` when camera intrinsics/extrinsics are available;
* a safe point above the surface for the next motion-planning layer.

Patch area alone is never enough: clipped masks, shallow short edges and
invalid depth all make an observation ineligible for precision grasping.
"""

from __future__ import annotations

import math
import os
import typing
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from sdk_wrapper.dev.algorithms import geometry as geo

__all__ = [
    "YoloInstance",
    "UltralyticsYoloBackend",
    "UltralyticsSegmentationBackend",
    "discover_latest_charger_seg_weights",
    "analyze_charger_top",
    "analyze_charger_whole_fallback",
    "annotation_overlay",
]


@dataclass
class YoloInstance:
    """One normalized YOLO instance; masks are full-resolution bool arrays."""

    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: np.ndarray | None = None
    track_id: int | None = None
    stage: str = "refined"
    proposal_confidence: float | None = None


def _canonical_axis(vector: np.ndarray) -> np.ndarray:
    """Stable sign for command generation: +x, then +y, then +z."""
    vec = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise ValueError("axis vector has zero norm")
    vec = vec / norm
    for component in vec:
        if abs(float(component)) > 1e-9:
            if component < 0:
                vec = -vec
            break
    return vec


def _touch_edges(mask: np.ndarray, margin_px: int) -> list[str]:
    """Return image edges touched by the mask, including an inward safety band."""
    if mask.size == 0:
        return ["top", "bottom", "left", "right"]
    m = max(0, int(margin_px))
    touched: list[str] = []
    if np.any(mask[: min(m + 1, mask.shape[0]), :]):
        touched.append("top")
    if np.any(mask[max(0, mask.shape[0] - m - 1) :, :]):
        touched.append("bottom")
    if np.any(mask[:, : min(m + 1, mask.shape[1])]):
        touched.append("left")
    if np.any(mask[:, max(0, mask.shape[1] - m - 1) :]):
        touched.append("right")
    return touched


def _mask_2d_geometry(mask: np.ndarray, margin_px: int) -> dict[str, typing.Any]:
    ys, xs = np.nonzero(mask)
    if xs.size < 3:
        raise ValueError("segmentation mask has fewer than 3 pixels")

    points = np.column_stack([xs, ys]).astype(np.float64)
    center = np.median(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered / max(1, len(points) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    long_axis = _canonical_axis(np.r_[eigenvectors[:, order[0]], 0.0])[:2]
    short_axis = np.array([-long_axis[1], long_axis[0]], dtype=np.float64)

    long_projection = centered @ long_axis
    short_projection = centered @ short_axis
    long_lo, long_hi = np.percentile(long_projection, [0.5, 99.5])
    short_lo, short_hi = np.percentile(short_projection, [0.5, 99.5])
    long_px = float(max(1.0, long_hi - long_lo))
    short_px = float(max(1.0, short_hi - short_lo))
    if short_px > long_px:
        long_axis, short_axis = short_axis, long_axis
        long_px, short_px = short_px, long_px

    bbox = geo.mask_bbox(mask)
    if bbox is None:
        raise ValueError("segmentation mask has no bounding box")
    bx, by, bw, bh = bbox
    fill_ratio = float(xs.size) / float(max(1, bw * bh))
    touches = _touch_edges(mask, margin_px)
    angle_deg = math.degrees(math.atan2(float(long_axis[1]), float(long_axis[0])))
    angle_deg = (angle_deg + 90.0) % 180.0 - 90.0

    return {
        "mask_area_px": int(xs.size),
        "mask_area_ratio": float(xs.size) / float(mask.size),
        "mask_bbox_xywh": [int(bx), int(by), int(bw), int(bh)],
        "mask_fill_ratio": round(fill_ratio, 6),
        "mask_center_uv": [round(float(center[0]), 3), round(float(center[1]), 3)],
        "long_edge_px": round(long_px, 3),
        "short_edge_px": round(short_px, 3),
        "long_edge_angle_deg": round(angle_deg, 3),
        "long_axis_px": [round(float(v), 8) for v in long_axis],
        "short_axis_px": [round(float(v), 8) for v in short_axis],
        "touches_image_edge": bool(touches),
        "touched_edges": touches,
        "fully_visible": not touches,
    }


def _intrinsics_values(intrinsics: typing.Any) -> tuple[float, float, float, float]:
    if isinstance(intrinsics, dict):
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


def _surface_3d_geometry(
    depth_m: np.ndarray,
    mask: np.ndarray,
    intrinsics: typing.Any,
    extrinsic: typing.Sequence[float],
    *,
    max_depth_m: float | None,
    safe_standoff_m: float,
    sample_stride: int,
) -> dict[str, typing.Any]:
    valid = np.isfinite(depth_m) & (depth_m > 0) & mask
    if max_depth_m is not None:
        valid &= depth_m <= float(max_depth_m)
    ys, xs = np.nonzero(valid)
    if xs.size < 3:
        return {"available": False, "reason": "fewer_than_3_valid_depth_pixels"}

    stride = max(1, int(sample_stride))
    keep = ((xs % stride) == 0) & ((ys % stride) == 0)
    xs, ys = xs[keep], ys[keep]
    if xs.size < 3:
        return {"available": False, "reason": "fewer_than_3_sampled_depth_points"}

    fx, fy, cx, cy = _intrinsics_values(intrinsics)
    z = depth_m[ys, xs].astype(np.float64)
    camera_points = np.column_stack(
        [(xs.astype(np.float64) - cx) * z / fx,
         (ys.astype(np.float64) - cy) * z / fy,
         z]
    )
    base_points = geo.transform_points(camera_points, extrinsic)

    # Remove isolated upper-tail depth returns without deleting mask pixels.
    # Median/MAD is used only for fitting; depth coverage is measured earlier.
    distance = np.linalg.norm(camera_points, axis=1)
    median_distance = float(np.median(distance))
    mad = float(np.median(np.abs(distance - median_distance)))
    if mad > 1e-9:
        inlier = np.abs(distance - median_distance) <= 4.5 * 1.4826 * mad
        if int(inlier.sum()) >= 3:
            base_points = base_points[inlier]

    center = np.median(base_points, axis=0)
    centered = base_points - center
    covariance = centered.T @ centered / max(1, len(base_points) - 1)
    values, vectors = np.linalg.eigh(covariance)
    normal = vectors[:, int(np.argmin(values))]
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm < 1e-12:
        return {"available": False, "reason": "surface_normal_has_zero_norm"}
    normal = normal / normal_norm
    if normal[2] < 0:
        normal = -normal
    # A generic first-nonzero-axis sign canonicalization can flip an almost
    # vertical normal upside down when its small x component is negative.

    candidate_long = vectors[:, int(np.argmax(values))]
    long_axis = candidate_long - float(candidate_long @ normal) * normal
    if np.linalg.norm(long_axis) < 1e-9:
        fallback = np.array([1.0, 0.0, 0.0])
        if abs(float(fallback @ normal)) > 0.95:
            fallback = np.array([0.0, 1.0, 0.0])
        long_axis = fallback - float(fallback @ normal) * normal
    long_axis = _canonical_axis(long_axis)
    short_axis = _canonical_axis(np.cross(normal, long_axis))

    long_projection = centered @ long_axis
    short_projection = centered @ short_axis
    long_lo, long_hi = np.percentile(long_projection, [1.0, 99.0])
    short_lo, short_hi = np.percentile(short_projection, [1.0, 99.0])
    long_m = float(max(1e-6, long_hi - long_lo))
    short_m = float(max(1e-6, short_hi - short_lo))
    if short_m > long_m:
        long_axis, short_axis = short_axis, long_axis
        long_m, short_m = short_m, long_m

    plane_residual = np.abs(centered @ normal)
    surface_center = center
    standoff = max(0.0, float(safe_standoff_m))
    pregrasp = surface_center + normal * standoff

    return {
        "available": True,
        "frame": "base_link",
        "fit_points": int(len(base_points)),
        "surface_center_m": [round(float(v), 6) for v in surface_center],
        "surface_normal_base": [round(float(v), 8) for v in normal],
        "long_edge_axis_base": [round(float(v), 8) for v in long_axis],
        "short_edge_axis_base": [round(float(v), 8) for v in short_axis],
        # The gripper body follows this axis; its fingers close across short_edge_axis_base.
        "gripper_long_axis_base": [round(float(v), 8) for v in long_axis],
        "jaw_closing_axis_base": [round(float(v), 8) for v in short_axis],
        "long_dimension_m": round(long_m, 6),
        "short_dimension_m": round(short_m, 6),
        "surface_plane_rms_m": round(float(np.sqrt(np.mean(plane_residual ** 2))), 6),
        "surface_plane_p95_residual_m": round(float(np.percentile(plane_residual, 95)), 6),
        "pregrasp_position_m": [round(float(v), 6) for v in pregrasp],
        "safe_standoff_m": round(standoff, 6),
    }


def analyze_charger_top(
    instance: YoloInstance,
    image_shape: typing.Sequence[int],
    *,
    depth_m: np.ndarray | None = None,
    intrinsics: typing.Any = None,
    extrinsic: typing.Sequence[float] | None = None,
    min_area_px: int = 600,
    min_area_ratio: float = 0.002,
    min_short_edge_px: float = 24.0,
    min_mask_fill_ratio: float = 0.30,
    edge_margin_px: int = 3,
    min_depth_valid_ratio: float = 0.80,
    max_depth_m: float | None = 1.5,
    min_long_dimension_m: float = 0.0,
    min_short_dimension_m: float = 0.0,
    max_surface_tilt_deg: float = 180.0,
    safe_standoff_m: float = 0.12,
    depth_sample_stride: int = 2,
) -> tuple[dict[str, typing.Any], np.ndarray]:
    """Validate one charger-top instance and derive a precision-grasp frame.

    The output remains useful when the instance is rejected: callers receive
    the measured geometry and explicit rejection reasons instead of a bare
    success/failure.
    """
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height, width = int(image_shape[0]), int(image_shape[1])
    reasons: list[str] = []

    raw_mask = instance.mask
    if raw_mask is None:
        raise ValueError("charger-top analysis requires a segmentation mask")
    mask = np.asarray(raw_mask)
    if mask.ndim == 3:
        mask = np.squeeze(mask)
    if mask.ndim != 2:
        raise ValueError(f"segmentation mask must be 2-D, got {mask.shape}")
    mask = mask.astype(bool, copy=False)
    if mask.shape != (height, width):
        mask = cv2.resize(
            mask.astype(np.uint8),
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    geom_2d = _mask_2d_geometry(mask, int(edge_margin_px))
    if geom_2d["mask_area_px"] < int(min_area_px):
        reasons.append("mask_area_below_minimum")
    if geom_2d["mask_area_ratio"] < float(min_area_ratio):
        reasons.append("mask_area_ratio_below_minimum")
    if geom_2d["short_edge_px"] < float(min_short_edge_px):
        reasons.append("short_edge_below_minimum")
    if geom_2d["mask_fill_ratio"] < float(min_mask_fill_ratio):
        reasons.append("mask_fill_ratio_below_minimum")
    if not geom_2d["fully_visible"]:
        reasons.append("target_touches_image_edge")

    depth_report: dict[str, typing.Any] = {"available": False}
    surface: dict[str, typing.Any] = {"available": False}
    if depth_m is not None:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.shape != (height, width):
            reasons.append("depth_rgb_shape_mismatch")
            depth_report = {
                "available": False,
                "reason": "depth_rgb_shape_mismatch",
                "depth_shape": list(depth.shape),
                "rgb_shape": [height, width],
            }
        else:
            valid = np.isfinite(depth) & (depth > 0)
            if max_depth_m is not None:
                valid &= depth <= float(max_depth_m)
            inside = valid & mask
            mask_pixels = int(mask.sum())
            valid_pixels = int(inside.sum())
            valid_ratio = valid_pixels / float(max(1, mask_pixels))
            values = depth[inside]
            depth_report = {
                "available": valid_pixels > 0,
                "valid_pixels": valid_pixels,
                "mask_pixels": mask_pixels,
                "valid_ratio": round(valid_ratio, 6),
                "min_m": round(float(values.min()), 6) if values.size else None,
                "median_m": round(float(np.median(values)), 6) if values.size else None,
                "max_m": round(float(values.max()), 6) if values.size else None,
            }
            if valid_ratio < float(min_depth_valid_ratio):
                reasons.append("depth_valid_ratio_below_minimum")
            if intrinsics is not None and extrinsic is not None:
                surface = _surface_3d_geometry(
                    depth,
                    mask,
                    intrinsics,
                    extrinsic,
                    max_depth_m=max_depth_m,
                    safe_standoff_m=safe_standoff_m,
                    sample_stride=depth_sample_stride,
                )
                if not surface.get("available"):
                    reasons.append("surface_3d_unavailable")
                else:
                    if float(surface["long_dimension_m"]) < float(
                        min_long_dimension_m
                    ):
                        reasons.append("surface_long_dimension_below_minimum")
                    if float(surface["short_dimension_m"]) < float(
                        min_short_dimension_m
                    ):
                        reasons.append("surface_short_dimension_below_minimum")
                    normal_z = max(
                        -1.0,
                        min(1.0, float(surface["surface_normal_base"][2])),
                    )
                    surface_tilt_deg = math.degrees(math.acos(normal_z))
                    surface["tilt_from_base_up_deg"] = round(surface_tilt_deg, 6)
                    if surface_tilt_deg > float(max_surface_tilt_deg):
                        reasons.append("surface_normal_tilt_above_maximum")
            else:
                reasons.append("surface_3d_intrinsics_or_extrinsics_unavailable")
    else:
        reasons.append("depth_unavailable")

    bbox = [float(v) for v in instance.bbox_xyxy]
    bbox_touches = (
        bbox[0] <= float(edge_margin_px)
        or bbox[1] <= float(edge_margin_px)
        or bbox[2] >= width - 1 - float(edge_margin_px)
        or bbox[3] >= height - 1 - float(edge_margin_px)
    )
    if bbox_touches and "target_touches_image_edge" not in reasons:
        reasons.append("bbox_touches_image_edge")

    result = {
        "eligible_for_precision": not reasons,
        "rejection_reasons": reasons,
        "class_name": instance.class_name,
        "class_id": int(instance.class_id),
        "confidence": round(float(instance.confidence), 6),
        "bbox_xyxy": [round(v, 3) for v in bbox],
        "bbox_touches_image_edge": bbox_touches,
        "image_width": width,
        "image_height": height,
        "image_space": geom_2d,
        "depth": depth_report,
        "surface": surface,
        "grasp_frame_semantics": {
            "gripper_long_axis": "parallel to the charger top's long edge",
            "jaw_closing_axis": "across the short dimension toward both long edges",
            "approach_axis": "negative surface_normal_base",
        },
    }
    return result, mask


def analyze_charger_whole_fallback(
    instance: YoloInstance,
    image_shape: typing.Sequence[int],
    *,
    depth_m: np.ndarray | None = None,
    intrinsics: typing.Any = None,
    extrinsic: typing.Sequence[float] | None = None,
    min_area_px: int = 600,
    min_area_ratio: float = 0.002,
    min_mask_fill_ratio: float = 0.20,
    edge_margin_px: int = 3,
    min_depth_valid_ratio: float = 0.50,
    max_depth_m: float | None = 1.5,
    max_surface_tilt_deg: float = 50.0,
    max_local_plane_rms_m: float = 0.02,
    safe_standoff_m: float = 0.12,
    depth_sample_stride: int = 2,
    sample_attempts: int = 64,
    sample_radius_px: int = 12,
    sample_seed: int = 20261108,
) -> tuple[dict[str, typing.Any], np.ndarray]:
    """Sample a target inside a whole-charger mask when top segmentation fails.

    The sampler excludes the object-edge band, draws reproducible random depth
    pixels inside the remaining mask, and accepts a point only when a local
    plane has enough support, low residual, and an upward-facing normal. The
    whole-mask plane axes and dimensions are retained as the best available
    gripper orientation estimate.
    """
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height, width = int(image_shape[0]), int(image_shape[1])
    if instance.mask is None:
        raise ValueError("whole-charger fallback requires a segmentation mask")
    mask = np.asarray(instance.mask)
    if mask.ndim == 3:
        mask = np.squeeze(mask)
    if mask.ndim != 2:
        raise ValueError(f"segmentation mask must be 2-D, got {mask.shape}")
    mask = mask.astype(bool, copy=False)
    if mask.shape != (height, width):
        mask = cv2.resize(
            mask.astype(np.uint8),
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    reasons: list[str] = []
    geom_2d = _mask_2d_geometry(mask, int(edge_margin_px))
    if geom_2d["mask_area_px"] < int(min_area_px):
        reasons.append("mask_area_below_minimum")
    if geom_2d["mask_area_ratio"] < float(min_area_ratio):
        reasons.append("mask_area_ratio_below_minimum")
    if geom_2d["mask_fill_ratio"] < float(min_mask_fill_ratio):
        reasons.append("mask_fill_ratio_below_minimum")
    if not geom_2d["fully_visible"]:
        reasons.append("target_touches_image_edge")

    depth_report: dict[str, typing.Any] = {"available": False}
    whole_surface: dict[str, typing.Any] = {"available": False}
    sampled_surface: dict[str, typing.Any] | None = None
    sampling_report: dict[str, typing.Any] = {
        "strategy": "whole_mask_random_sample",
        "seed": int(sample_seed),
        "sample_attempts": max(1, int(sample_attempts)),
        "sample_radius_px": max(3, int(sample_radius_px)),
        "valid_samples": 0,
        "sampled_uv": None,
        "rejection_counts": {},
    }
    if depth_m is None:
        reasons.append("depth_unavailable")
    else:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.shape != (height, width):
            reasons.append("depth_rgb_shape_mismatch")
            depth_report = {
                "available": False,
                "reason": "depth_rgb_shape_mismatch",
                "depth_shape": list(depth.shape),
                "rgb_shape": [height, width],
            }
        else:
            valid = np.isfinite(depth) & (depth > 0)
            if max_depth_m is not None:
                valid &= depth <= float(max_depth_m)
            inside = valid & mask
            valid_ratio = int(inside.sum()) / float(max(1, int(mask.sum())))
            values = depth[inside]
            depth_report = {
                "available": bool(values.size),
                "valid_pixels": int(values.size),
                "mask_pixels": int(mask.sum()),
                "valid_ratio": round(valid_ratio, 6),
                "min_m": round(float(values.min()), 6) if values.size else None,
                "median_m": round(float(np.median(values)), 6) if values.size else None,
                "max_m": round(float(values.max()), 6) if values.size else None,
            }
            if valid_ratio < float(min_depth_valid_ratio):
                reasons.append("depth_valid_ratio_below_minimum")
            if intrinsics is None or extrinsic is None:
                reasons.append("surface_3d_intrinsics_or_extrinsics_unavailable")
            else:
                whole_surface = _surface_3d_geometry(
                    depth,
                    mask,
                    intrinsics,
                    extrinsic,
                    max_depth_m=max_depth_m,
                    safe_standoff_m=safe_standoff_m,
                    sample_stride=depth_sample_stride,
                )

                distance = cv2.distanceTransform(
                    mask.astype(np.uint8),
                    cv2.DIST_L2,
                    5,
                )
                radius = max(
                    3,
                    min(
                        int(sample_radius_px),
                        max(4, int(0.15 * geom_2d["short_edge_px"])),
                    ),
                )
                sampling_report["sample_radius_px"] = radius
                eligible = inside & (
                    distance >= float(radius + max(0, int(edge_margin_px)))
                )
                flat_indices = np.flatnonzero(eligible.reshape(-1))
                rng = np.random.default_rng(int(sample_seed))
                if flat_indices.size > int(sample_attempts):
                    flat_indices = rng.choice(
                        flat_indices,
                        size=max(1, int(sample_attempts)),
                        replace=False,
                    )
                else:
                    rng.shuffle(flat_indices)

                yy, xx = np.ogrid[-radius : radius + 1, -radius : radius + 1]
                disk = (xx * xx + yy * yy) <= radius * radius
                valid_samples: list[tuple[float, dict[str, typing.Any], tuple[int, int]]] = []
                rejection_counts: dict[str, int] = {}

                def reject(reason: str) -> None:
                    rejection_counts[reason] = rejection_counts.get(reason, 0) + 1

                for flat_index in flat_indices.tolist():
                    v = int(flat_index // width)
                    u = int(flat_index % width)
                    if depth[v, u] <= 0 or not math.isfinite(float(depth[v, u])):
                        reject("invalid_center_depth")
                        continue
                    local_mask = np.zeros((height, width), dtype=bool)
                    y0 = max(0, v - radius)
                    y1 = min(height, v + radius + 1)
                    x0 = max(0, u - radius)
                    x1 = min(width, u + radius + 1)
                    local_mask[y0:y1, x0:x1] = disk[
                        y0 - (v - radius) : y1 - (v - radius),
                        x0 - (u - radius) : x1 - (u - radius),
                    ]
                    local_mask &= mask
                    local = _surface_3d_geometry(
                        depth,
                        local_mask,
                        intrinsics,
                        extrinsic,
                        max_depth_m=max_depth_m,
                        safe_standoff_m=safe_standoff_m,
                        sample_stride=1,
                    )
                    if not local.get("available"):
                        reject(str(local.get("reason") or "local_surface_unavailable"))
                        continue
                    normal_z = max(
                        -1.0,
                        min(1.0, float(local["surface_normal_base"][2])),
                    )
                    tilt_deg = math.degrees(math.acos(normal_z))
                    if tilt_deg > float(max_surface_tilt_deg):
                        reject("local_surface_tilt_above_maximum")
                        continue
                    if float(local["surface_plane_rms_m"]) > float(
                        max_local_plane_rms_m
                    ):
                        reject("local_plane_rms_above_maximum")
                        continue
                    valid_samples.append((tilt_deg, local, (u, v)))

                sampling_report["valid_samples"] = len(valid_samples)
                sampling_report["rejection_counts"] = rejection_counts
                if valid_samples:
                    chosen_index = int(rng.integers(0, len(valid_samples)))
                    tilt_deg, surface, sampled_uv = valid_samples[chosen_index]
                    normal = _canonical_axis(
                        np.asarray(surface["surface_normal_base"], dtype=np.float64)
                    )
                    if normal[2] < 0:
                        normal = -normal
                    long_axis = np.asarray(
                        surface["long_edge_axis_base"],
                        dtype=np.float64,
                    )
                    if whole_surface.get("available"):
                        long_axis = np.asarray(
                            whole_surface["long_edge_axis_base"],
                            dtype=np.float64,
                        )
                    long_axis = long_axis - float(long_axis @ normal) * normal
                    if float(np.linalg.norm(long_axis)) < 1e-8:
                        fallback = np.array([1.0, 0.0, 0.0])
                        if abs(float(fallback @ normal)) > 0.95:
                            fallback = np.array([0.0, 1.0, 0.0])
                        long_axis = fallback - float(fallback @ normal) * normal
                    long_axis = _canonical_axis(long_axis)
                    short_axis = _canonical_axis(np.cross(normal, long_axis))
                    standoff = max(0.0, float(safe_standoff_m))
                    center = np.asarray(
                        surface["surface_center_m"],
                        dtype=np.float64,
                    )
                    sampled_surface = dict(surface)
                    sampled_surface.update(
                        {
                            "surface_normal_base": [
                                round(float(v), 8) for v in normal
                            ],
                            "long_edge_axis_base": [
                                round(float(v), 8) for v in long_axis
                            ],
                            "short_edge_axis_base": [
                                round(float(v), 8) for v in short_axis
                            ],
                            "gripper_long_axis_base": [
                                round(float(v), 8) for v in long_axis
                            ],
                            "jaw_closing_axis_base": [
                                round(float(v), 8) for v in short_axis
                            ],
                            "long_dimension_m": whole_surface.get(
                                "long_dimension_m",
                                surface["long_dimension_m"],
                            ),
                            "short_dimension_m": whole_surface.get(
                                "short_dimension_m",
                                surface["short_dimension_m"],
                            ),
                            "tilt_from_base_up_deg": round(tilt_deg, 6),
                            "pregrasp_position_m": [
                                round(float(v), 6)
                                for v in center + normal * standoff
                            ],
                            "safe_standoff_m": round(standoff, 6),
                            "sampled_target": True,
                            "dimension_source": (
                                "whole_mask_plane_estimate"
                                if whole_surface.get("available")
                                else "local_sample_plane"
                            ),
                        }
                    )
                    sampling_report.update(
                        {
                            "sampled_uv": [int(sampled_uv[0]), int(sampled_uv[1])],
                            "selected_local_plane_rms_m": sampled_surface[
                                "surface_plane_rms_m"
                            ],
                            "selected_tilt_from_base_up_deg": round(tilt_deg, 6),
                        }
                    )
                else:
                    reasons.append("whole_fallback_sampling_failed")

    surface = sampled_surface or {"available": False}
    result = {
        "eligible_for_precision": not reasons,
        "rejection_reasons": reasons,
        "target_mode": "whole_mask_random_sample",
        "class_name": instance.class_name,
        "class_id": int(instance.class_id),
        "confidence": round(float(instance.confidence), 6),
        "bbox_xyxy": [round(float(v), 3) for v in instance.bbox_xyxy],
        "image_width": width,
        "image_height": height,
        "image_space": geom_2d,
        "depth": depth_report,
        "surface": surface,
        "whole_fallback_sampling": sampling_report,
        "grasp_frame_semantics": {
            "gripper_long_axis": "whole-mask long-axis estimate projected onto the sampled local plane",
            "jaw_closing_axis": "across the whole-mask short dimension",
            "approach_axis": "negative sampled local surface normal",
        },
    }
    return result, mask


def annotation_overlay(
    bgr: np.ndarray,
    instance: YoloInstance,
    analysis: typing.Mapping[str, typing.Any],
) -> np.ndarray:
    """Draw mask, axes and camera-side safe point for human inspection."""
    out = np.asarray(bgr).copy()
    mask = instance.mask
    if mask is None:
        return out
    mask = np.asarray(mask, dtype=bool)
    if mask.shape != out.shape[:2]:
        mask = cv2.resize(
            mask.astype(np.uint8),
            (out.shape[1], out.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)

    tint = out.copy()
    tint[mask] = (30, 210, 60)
    out = cv2.addWeighted(out, 0.68, tint, 0.32, 0)
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(out, contours, -1, (0, 255, 0), 2)

    x1, y1, x2, y2 = [int(round(v)) for v in instance.bbox_xyxy]
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 255), 1)
    center = analysis["image_space"]["mask_center_uv"]
    long_axis = np.asarray(analysis["image_space"]["long_axis_px"], np.float64)
    length = max(25.0, 0.30 * analysis["image_space"]["long_edge_px"])
    p0 = tuple(np.round(np.asarray(center) - long_axis * length).astype(int).tolist())
    p1 = tuple(np.round(np.asarray(center) + long_axis * length).astype(int).tolist())
    cv2.arrowedLine(out, p0, p1, (255, 80, 40), 2, tipLength=0.18)
    cv2.circle(out, tuple(np.round(center).astype(int).tolist()), 3, (255, 255, 255), -1)
    sampling = analysis.get("whole_fallback_sampling") or {}
    sampled_uv = sampling.get("sampled_uv")
    if sampled_uv:
        su, sv = [int(round(float(value))) for value in sampled_uv]
        cv2.drawMarker(
            out,
            (su, sv),
            (0, 255, 255),
            cv2.MARKER_CROSS,
            14,
            2,
        )
    stage_label = {
        "proposal_only": " PROPOSAL ONLY",
        "top_surface": " TOP",
        "whole_fallback": " WHOLE FALLBACK",
    }.get(instance.stage, "")
    label = "%s %.3f %s%s" % (
        instance.class_name,
        float(instance.confidence),
        "READY" if analysis.get("eligible_for_precision") else "REJECT",
        stage_label,
    )
    cv2.putText(
        out,
        label,
        (max(0, x1), max(16, y1 - 5)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 255, 0) if analysis.get("eligible_for_precision") else (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    return out


def discover_latest_charger_seg_weights(
    roots: typing.Sequence[str | os.PathLike[str]],
    *,
    glob_pattern: str = "**/weights/best.pt",
) -> list[Path]:
    """Return ``best.pt`` candidates ordered by modification time, newest first."""
    found: dict[str, Path] = {}
    for root in roots:
        root_path = Path(root).expanduser()
        if not root_path.exists():
            continue
        for path in root_path.glob(glob_pattern):
            if path.is_file():
                found[str(path.resolve())] = path.resolve()
    return sorted(found.values(), key=lambda p: p.stat().st_mtime, reverse=True)


def _install_torchvision_nms_fallback() -> str | None:
    """Use a Python NMS when a custom PyTorch build cannot load torchvision ops.

    NVIDIA Jetson PyTorch builds can be ABI-compatible with their matching
    custom torchvision only. A generic torchvision wheel may import but fail
    when Ultralytics calls ``torchvision.ops.nms``. Ultralytics also ships a
    data-dependent NMS fast path, but that path is not selected when the
    torchvision module is already imported. Patch only the broken callable.
    """
    try:
        import torch
        import torchvision
    except Exception:  # noqa: BLE001 - YOLO may still work without this module
        return None
    try:
        probe = torchvision.ops.nms(
            torch.tensor([[0.0, 0.0, 1.0, 1.0]], dtype=torch.float32),
            torch.tensor([1.0], dtype=torch.float32),
            0.5,
        )
        if probe.numel() == 1:
            return None
    except Exception:
        pass

    def _nms(
        boxes: typing.Any,
        scores: typing.Any,
        iou_threshold: float,
    ) -> typing.Any:
        if boxes.numel() == 0:
            return torch.empty((0,), dtype=torch.long, device=boxes.device)
        order = scores.argsort(descending=True)
        keep: list[typing.Any] = []
        while order.numel():
            index = order[0]
            keep.append(index)
            if order.numel() == 1:
                break
            rest = order[1:]
            selected = boxes[index]
            candidates = boxes[rest]
            inter_x1 = torch.maximum(selected[0], candidates[:, 0])
            inter_y1 = torch.maximum(selected[1], candidates[:, 1])
            inter_x2 = torch.minimum(selected[2], candidates[:, 2])
            inter_y2 = torch.minimum(selected[3], candidates[:, 3])
            inter = (
                (inter_x2 - inter_x1).clamp(min=0)
                * (inter_y2 - inter_y1).clamp(min=0)
            )
            area_selected = (
                (selected[2] - selected[0]).clamp(min=0)
                * (selected[3] - selected[1]).clamp(min=0)
            )
            area_candidates = (
                (candidates[:, 2] - candidates[:, 0]).clamp(min=0)
                * (candidates[:, 3] - candidates[:, 1]).clamp(min=0)
            )
            union = area_selected + area_candidates - inter
            iou = inter / union.clamp(min=torch.finfo(inter.dtype).eps)
            order = rest[iou <= float(iou_threshold)]
        return torch.stack(keep)

    torchvision.ops.nms = _nms
    return "torchvision_nms_python_fallback"


class UltralyticsYoloBackend:
    """Lazy Ultralytics YOLO backend with task/class validation.

    ``weights=None`` selects the newest candidate whose model metadata reports
    the requested task and target class. Detection and segmentation tasks share
    the same normalized output; masks are populated only for segment models.
    Loading every stale experiment is undesirable, but is necessary for
    fail-closed discovery.
    """

    def __init__(
        self,
        roots: typing.Sequence[str | os.PathLike[str]],
        *,
        weights: str | os.PathLike[str] | None = None,
        target_class: str = "charger",
        required_task: str = "segment",
        imgsz: int = 640,
        confidence: float = 0.30,
        iou: float = 0.70,
        device: str = "cpu",
    ) -> None:
        self.roots = [os.fspath(root) for root in roots]
        self.weights = os.fspath(weights) if weights else None
        self.target_class = str(target_class)
        self.required_task = str(required_task)
        self.imgsz = int(imgsz)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.device = str(device)
        self._model: typing.Any = None
        self._class_id: int | None = None
        self.model_info: dict[str, typing.Any] = {}

    def _load(self) -> typing.Any:
        if self._model is not None:
            return self._model
        try:
            from ultralytics import YOLO  # type: ignore[import-not-found]
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "ultralytics is not installed in this Python environment; "
                "use the repository perception runtime or install it via dependency owner"
            ) from exc
        nms_fallback = _install_torchvision_nms_fallback()

        candidates = (
            [Path(self.weights).expanduser()]
            if self.weights
            else discover_latest_charger_seg_weights(self.roots)
        )
        errors: list[str] = []
        for path in candidates:
            if not path.is_file():
                errors.append(f"{path}: missing")
                continue
            try:
                model = YOLO(str(path))
                task = str(getattr(model, "task", ""))
                names = dict(getattr(model, "names", {}) or {})
                reverse = {str(name): int(class_id) for class_id, name in names.items()}
                if task != self.required_task:
                    errors.append(f"{path}: task={task!r}")
                    continue
                if self.target_class not in reverse:
                    errors.append(f"{path}: classes={sorted(reverse)}")
                    continue
                self._model = model
                self._class_id = reverse[self.target_class]
                stat = path.stat()
                self.model_info = {
                    "weights": str(path),
                    "modified_ns": int(stat.st_mtime_ns),
                    "task": task,
                    "classes": names,
                    "target_class_id": self._class_id,
                    "imgsz": self.imgsz,
                    "confidence": self.confidence,
                    "iou": self.iou,
                }
                if nms_fallback:
                    self.model_info["runtime_workaround"] = nms_fallback
                return model
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{path}: {type(exc).__name__}: {exc}")
        raise RuntimeError(
            "no usable charger segmentation weights found; candidates: "
            + "; ".join(errors[:8])
        )

    def describe(self) -> dict[str, typing.Any]:
        self._load()
        return dict(self.model_info)

    def predict(self, bgr: np.ndarray) -> list[YoloInstance]:
        model = self._load()
        image = np.asarray(bgr)
        if image.ndim != 3 or image.dtype != np.uint8:
            raise ValueError("bgr must be HxWx3 uint8")
        results = model.predict(
            source=image,
            imgsz=self.imgsz,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []
        return self._normalize_result(results[0], image, model)

    def track(self, bgr: np.ndarray, *, persist: bool = True) -> list[YoloInstance]:
        """Track the configured class across sequential frames.

        Ultralytics ByteTrack owns the track IDs. Callers must pass frames in
        order and keep one backend instance for the whole navigation segment.
        """
        model = self._load()
        image = np.asarray(bgr)
        if image.ndim != 3 or image.dtype != np.uint8:
            raise ValueError("bgr must be HxWx3 uint8")
        results = model.track(
            source=image,
            imgsz=self.imgsz,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            persist=bool(persist),
            tracker="bytetrack.yaml",
            verbose=False,
        )
        if not results:
            return []
        return self._normalize_result(results[0], image, model)

    def _normalize_result(
        self,
        result: typing.Any,
        image: np.ndarray,
        model: typing.Any,
    ) -> list[YoloInstance]:
        if result.boxes is None or len(result.boxes) == 0:
            return []

        boxes = result.boxes.xyxy.detach().cpu().numpy()
        confidences = result.boxes.conf.detach().cpu().numpy()
        class_ids = result.boxes.cls.detach().cpu().numpy().astype(int)
        track_ids = None
        if getattr(result.boxes, "id", None) is not None:
            track_ids = result.boxes.id.detach().cpu().numpy().astype(int)
        masks = None
        if result.masks is not None and result.masks.data is not None:
            masks = result.masks.data.detach().cpu().numpy()

        names = dict(getattr(result, "names", {}) or getattr(model, "names", {}) or {})
        instances: list[YoloInstance] = []
        for index, (box, score, class_id) in enumerate(zip(boxes, confidences, class_ids)):
            if self._class_id is not None and int(class_id) != int(self._class_id):
                continue
            mask = None
            if masks is not None and index < len(masks):
                mask_u8 = (masks[index] >= 0.5).astype(np.uint8)
                if mask_u8.shape != image.shape[:2]:
                    mask_u8 = cv2.resize(
                        mask_u8,
                        (image.shape[1], image.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                mask = mask_u8.astype(bool)
            instances.append(
                YoloInstance(
                    class_id=int(class_id),
                    class_name=str(names.get(int(class_id), class_id)),
                    confidence=float(score),
                    bbox_xyxy=tuple(float(v) for v in box.tolist()),
                    mask=mask,
                    track_id=(
                        None
                        if track_ids is None or index >= len(track_ids)
                        else int(track_ids[index])
                    ),
                )
            )
        return instances


# Backward-compatible name for callers that only use segmentation weights.
UltralyticsSegmentationBackend = UltralyticsYoloBackend
