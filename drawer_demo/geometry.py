"""Plane localization from original-pixel top masks; fixed grasp orientation.

H maps original image pixels (with the exact calibration distortion convention)
to base_link XY on the measured horizontal box-top plane. Moving camera/head,
base, legs, drawer or plane invalidates H. No historic depth is consumed.
"""
from __future__ import annotations

import math

from .contracts import Frame, require


def finite_vector(value, length):
    return (isinstance(value, (list, tuple)) and len(value) == length
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


def signed_area(poly):
    return sum(a[0]*b[1] - b[0]*a[1]
               for a, b in zip(poly, poly[1:] + poly[:1])) / 2


def convex_hull(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    return half(points)[:-1] + half(reversed(points))[:-1]


def project(point, h):
    u, v = point
    w = h[6]*u + h[7]*v + h[8]
    require(abs(w) > 1e-9, "Pixel ray is parallel to plane")
    p = ((h[0]*u + h[1]*v + h[2])/w, (h[3]*u + h[4]*v + h[5])/w)
    require(finite_vector(p, 2), "Invalid projected coordinate")
    return p


def rectangle(poly):
    hull = convex_hull(poly)
    require(len(hull) >= 3, "Degenerate mask")
    candidates = []
    for a, b in zip(hull, hull[1:] + hull[:1]):
        theta = math.atan2(b[1]-a[1], b[0]-a[0])
        c, s = math.cos(theta), math.sin(theta)
        uv = [(c*x+s*y, -s*x+c*y) for x, y in hull]
        lo = [min(p[k] for p in uv) for k in (0, 1)]
        hi = [max(p[k] for p in uv) for k in (0, 1)]
        w, h = hi[0]-lo[0], hi[1]-lo[1]
        u, v = (lo[0]+hi[0])/2, (lo[1]+hi[1])/2
        candidates.append((w*h, min(w, h), max(w, h),
                           theta if w <= h else theta+math.pi/2,
                           (c*u-s*v, s*u+c*v)))
    return min(candidates, key=lambda r: r[0])


def axis_error(a, b):
    return abs((a-b+math.pi/2) % math.pi-math.pi/2)


def yaw_segments(total_rad, max_step_rad):
    """Split one yaw move into equal, bounded steps.

    A single 30 deg base command is not verifiable while it executes: the only
    honest check is odometry after each segment. Equal steps of at most
    max_step_rad give the adapter one place to compare commanded and measured
    yaw before the next segment starts.
    """
    require(finite_vector([total_rad, max_step_rad], 2), "Invalid yaw segment input")
    require(max_step_rad > 0, "Rotation step must be positive")
    count = max(1, math.ceil(abs(total_rad)/max_step_rad-1e-9))
    return [total_rad/count]*count


def yaw_arc_offset_m(radius_m, yaw_error_rad):
    """Lateral error of a fixed world point caused by a base yaw error.

    The bin sits at `radius_m` from the base axis, so a residual yaw epsilon
    slides the taught drop aim sideways by r*epsilon. That product is the whole
    feasibility argument for the 30 deg turn, so it is computed, not asserted.
    """
    require(finite_vector([radius_m, yaw_error_rad], 2), "Invalid yaw offset input")
    require(radius_m > 0, "Radius must be positive")
    return abs(radius_m*yaw_error_rad)


def localize(payload: dict, frame: Frame, recipe: dict, now_ns: int) -> dict:
    """Exactly one plausible, complete top mask inside the calibrated opening."""
    cal, limits = recipe["calibration"], recipe["limits"]
    require(frame.setup_id == recipe["setup_id"], "Capture setup mismatch")
    require(frame.camera_id == cal["camera_id"] and frame.calibration_id == cal["id"],
            "Capture calibration mismatch")
    require(list(frame.image_size_wh) == cal["image_size_wh"], "Image size mismatch")
    require(0 <= now_ns-frame.capture_timestamp_ns <= limits["max_frame_age_s"]*1e9,
            "Stale or future image")
    require(payload.get("schema") == "drawer_box_seg/1", "Unsupported segmentation schema")
    for key, expected in (("frame_id", frame.frame_id), ("camera_id", frame.camera_id),
                          ("capture_timestamp_ns", frame.capture_timestamp_ns),
                          ("image_size_wh", list(frame.image_size_wh)),
                          ("model_id", recipe["model_id"])):
        require(payload.get(key) == expected, f"Segmentation mismatch: {key}")
    require(payload.get("status") == "ok", "Segmentation missing or unsuccessful")
    detections = payload.get("detections")
    require(isinstance(detections, list), "Invalid detections")
    candidates = [d for d in detections if isinstance(d, dict)
                  and d.get("class_name") == "medicine_box_top"]
    # Conservative single-box policy: even low-confidence duplicates are ambiguous.
    require(len(candidates) == 1, "Expected exactly one box-top candidate")
    det = candidates[0]
    score = det.get("confidence")
    require(type(score) in (int, float) and math.isfinite(score)
            and limits["min_confidence"] <= score <= 1, "Low/invalid confidence")
    polygon = det.get("mask_polygon_xy_px")
    require(isinstance(polygon, list) and 4 <= len(polygon) <= 10000, "Invalid top polygon")
    width, height = frame.image_size_wh
    require(all(finite_vector(p, 2) and 0 <= p[0] < width and 0 <= p[1] < height
                for p in polygon), "Mask outside original image")
    xy = [project(p, cal["H_pixel_to_base_xy"]) for p in polygon]
    xmin, ymin, xmax, ymax = cal["opening_roi_xy_m"]
    require(all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in xy),
            "Top mask outside opening ROI")
    area, short, long, angle, center = rectangle(xy)
    require(area > 1e-9 and limits["min_rectangularity"] <= abs(signed_area(xy))/area <= 1.01,
            "Incomplete or invalid top contour")
    require(abs(short-recipe["box_size_m"][1]) <= limits["size_tolerance_m"]
            and abs(long-recipe["box_size_m"][0]) <= limits["size_tolerance_m"],
            "Top dimensions inconsistent with fixed box")
    require(axis_error(angle, recipe["closing_axis_yaw_rad"]) <= limits["max_axis_error_rad"],
            "Box rotation exceeds fixed-grasp allowance")
    require(math.dist(center, cal["nominal_center_xy_m"]) <= limits["max_center_shift_m"],
            "Target outside calibrated correction region")
    return {"id": "fixed_box", "position_m": [*center, cal["box_top_z_m"]],
            "frame": "base_link", "unit": "m", "image_ref": frame.frame_id,
            "capture_timestamp_ns": frame.capture_timestamp_ns,
            "closing_axis_yaw_rad": recipe["closing_axis_yaw_rad"],
            "measured_short_axis_yaw_rad": angle, "calibration_id": cal["id"],
            "model_id": payload["model_id"], "mask_polygon_xy_px": polygon}
