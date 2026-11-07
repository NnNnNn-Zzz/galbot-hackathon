"""Step-1 stationary drawer perception analysis; no robot connection here.

Inputs are decoded RGB/depth arrays and calibration recorded by
tools/collect_drawer_stationary.py at several parked positions. RGB-to-depth
pixel alignment, depth units and extrinsic quaternion order must be verified
against recorded metadata before any result feeds drawer_open; this module
reports evidence and never marks perception verified by itself.
"""
import math
import numpy as np

from medicine_frontend.drawer_geometry import backproject, locate, transform
from medicine_frontend.drawer_perception import segment_handle


def pose7_to_matrix(pose):
    """SDK extrinsic [x,y,z,qx,qy,qz,qw] -> 4x4, per the SDK 1.10 pose order."""
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError("Expected [x,y,z,qx,qy,qz,qw]")
    q = p[3:]
    norm = float(np.linalg.norm(q))
    if not np.isclose(norm, 1.0, atol=1e-3):
        raise ValueError("Extrinsic quaternion is not unit length")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = p[:3]
    return transform(out)


def panel_mask(shape, handle_bbox, *, expand=4.0):
    """Drawer-front ROI grown around the handle; the handle area is excluded."""
    if not np.isfinite(expand) or not 2.0 <= expand <= 8.0:
        raise ValueError("Invalid panel ROI expand factor")
    height, width = shape
    x, y, w, h = (int(v) for v in handle_bbox)
    if w <= 0 or h <= 0 or x < 0 or y < 0 or x + w > width or y + h > height:
        raise ValueError("Handle bbox outside image")
    cx, cy = x + w / 2.0, y + h / 2.0
    half_w, half_h = w * expand / 2.0, h * expand / 2.0
    x0 = max(0, int(round(cx - half_w)))
    x1 = min(width, int(round(cx + half_w)))
    y0 = max(0, int(round(cy - half_h)))
    y1 = min(height, int(round(cy + half_h)))
    mask = np.zeros((height, width), dtype=bool)
    mask[y0:y1, x0:x1] = True
    margin_x = max(2, int(round(0.10 * w)))
    margin_y = max(2, int(round(0.25 * h)))
    mask[max(0, y - margin_y):min(height, y + h + margin_y),
         max(0, x - margin_x):min(width, x + w + margin_x)] = False
    return mask


def analyze_frame(bgr, depth_m, intrinsics, camera_to_base, *, roi=None, expand=4.0):
    """Locate handle pose and panel normal in base_link for one stationary frame.

    depth_m must be metres on the RGB pixel grid. Right-wrist RGB and depth
    are separate sensors; alignment must be verified before output is trusted.
    """
    image = np.asarray(bgr)
    depth = np.asarray(depth_m, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected BGR image")
    if depth.shape != image.shape[:2] or k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError("Depth/RGB/intrinsics mismatch")
    t = transform(camera_to_base)
    candidate = segment_handle(image, roi)
    handle_mask = candidate["mask"]
    panel = panel_mask(depth.shape, candidate["bbox"], expand=expand)
    handle_points = backproject(depth, handle_mask, k, metres_per_unit=1.0)
    panel_points = backproject(depth, panel, k, metres_per_unit=1.0)
    found = locate(panel_points, handle_points, t, t[:3, 3], unit="m")
    return {"handle_to_base": found["handle_to_base"],
            "normal_base": found["normal_base"],
            "plane_inlier_ratio": found["plane_inlier_ratio"],
            "handle_bbox": [int(v) for v in candidate["bbox"]],
            "handle_pixels": int(handle_mask.sum()),
            "panel_pixels": int(panel.sum())}


def _unit(value, name):
    v = np.asarray(value, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all() or not np.isclose(np.linalg.norm(v), 1.0):
        raise ValueError(f"{name} must be a finite unit vector")
    return v


def summarize(frames, *, position_tolerance_m=0.02, normal_tolerance_deg=3.0,
              allow_base_frame=False):
    """Cross-position stability of the measured handle pose.

    Each frame: {name, handle_to_base, normal_base, base_to_map?}. The same
    physical drawer must map to one position; without base_to_map a
    cross-position comparison is meaningless, so the summary fails unless
    allow_base_frame is set (single-position repeatability checks only).
    """
    if not np.isfinite(position_tolerance_m) or not 0 < position_tolerance_m <= 0.1:
        raise ValueError("Invalid position tolerance")
    if not np.isfinite(normal_tolerance_deg) or not 0 < normal_tolerance_deg <= 15:
        raise ValueError("Invalid normal tolerance")
    frames = list(frames)
    if len(frames) < 2:
        raise ValueError("Need at least two stationary frames")
    per_frame, positions, normals = [], [], []
    map_ready = True
    for entry in frames:
        handle = transform(entry["handle_to_base"])
        normal = _unit(entry["normal_base"], "normal_base")
        record = {"name": str(entry.get("name", len(per_frame))),
                  "handle_position_base": handle[:3, 3].tolist(),
                  "normal_base": normal.tolist(),
                  "plane_inlier_ratio": entry.get("plane_inlier_ratio")}
        base_to_map = entry.get("base_to_map")
        if base_to_map is None:
            map_ready = False
            position, direction = handle[:3, 3], normal
        else:
            m = transform(base_to_map)
            position = m[:3, :3] @ handle[:3, 3] + m[:3, 3]
            direction = m[:3, :3] @ normal
            direction /= np.linalg.norm(direction)
            record["handle_position_map"] = position.tolist()
            record["normal_map"] = direction.tolist()
        positions.append(position)
        normals.append(direction)
        per_frame.append(record)
    spread = max(float(np.linalg.norm(a - b))
                 for i, a in enumerate(positions) for b in positions[i + 1:])
    angle = max(math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))
                for i, a in enumerate(normals) for b in normals[i + 1:])
    failures = []
    if not map_ready and not allow_base_frame:
        failures.append("map_transform_unavailable:record base pose per position, "
                        "or set allow_base_frame for single-position repeatability")
    if spread > position_tolerance_m:
        failures.append(f"handle_position_spread_m:{spread:.4f}>{position_tolerance_m}")
    if angle > normal_tolerance_deg:
        failures.append(f"panel_normal_spread_deg:{angle:.2f}>{normal_tolerance_deg}")
    return {"passed": not failures,
            "comparison_frame": "map" if map_ready else "base_link",
            "frame_count": len(frames),
            "position_spread_m": spread,
            "normal_spread_deg": angle,
            "tolerances": {"position_m": position_tolerance_m,
                           "normal_deg": normal_tolerance_deg},
            "failures": failures,
            "per_frame": per_frame}
