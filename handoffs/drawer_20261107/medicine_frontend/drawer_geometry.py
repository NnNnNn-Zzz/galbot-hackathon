"""Metric drawer geometry. Inputs are segmented, calibrated points, not detections."""
import numpy as np


def backproject(depth, mask, intrinsics, *, metres_per_unit):
    """Backproject an aligned depth mask; scale must be independently verified."""
    d = np.asarray(depth, dtype=float)
    m = np.asarray(mask, dtype=bool)
    k = np.asarray(intrinsics, dtype=float)
    if (d.ndim != 2 or m.shape != d.shape or k.shape != (3, 3)
            or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.isfinite(metres_per_unit) or metres_per_unit <= 0):
        raise ValueError("Invalid aligned depth/calibration")
    v, u = np.nonzero(m & np.isfinite(d) & (d > 0))
    z = d[v, u] * metres_per_unit
    rays = np.linalg.solve(k, np.stack((u, v, np.ones_like(u))))
    return points((rays * z).T)


def measured_opening(handle_to_base, cabinet_to_base, closed_handle_in_cabinet,
                     outward_in_cabinet):
    """Cabinet-relative travel cancels robot/cabinet rigid motion, unlike TCP travel."""
    current = np.linalg.inv(transform(cabinet_to_base)) @ transform(handle_to_base)
    closed = transform(closed_handle_in_cabinet)
    n = np.asarray(outward_in_cabinet, dtype=float)
    if n.shape != (3,) or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n), 1):
        raise ValueError("Expected unit rail direction")
    delta = current[:3, 3]-closed[:3, 3]
    distance = float(delta @ n)
    return {"opening_m": distance, "lateral_m": float(np.linalg.norm(delta-distance*n))}


def transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
        raise ValueError("Expected rigid transform")
    return t


def points(value):
    p = np.asarray(value, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 12 or not np.isfinite(p).all():
        raise ValueError("Need at least 12 finite metric points")
    return p


def locate(panel_points_m, handle_points_m, camera_to_base, camera_origin_base,
           *, unit, threshold_m=0.004):
    """RANSAC panel plane plus handle PCA; masks must exclude cabinet/other handles.

    Handle x is its major axis, z points out of the panel. Resolve PCA sign by
    base-frame axis ordering; the same convention must be used in calibration.
    """
    if unit != "m" or not 0 < threshold_m <= 0.01:
        raise ValueError("Verified metre units and bounded plane threshold required")
    t = transform(camera_to_base)
    panel = points(panel_points_m) @ t[:3, :3].T + t[:3, 3]
    handle = points(handle_points_m) @ t[:3, :3].T + t[:3, 3]
    camera = np.asarray(camera_origin_base, dtype=float)
    if camera.shape != (3,) or not np.isfinite(camera).all():
        raise ValueError("Invalid camera origin")
    rng = np.random.default_rng(0)
    best = np.zeros(len(panel), dtype=bool)
    for _ in range(128):
        a, b, c = panel[rng.choice(len(panel), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        mask = np.abs((panel-a) @ n) <= threshold_m
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < 0.75 or best.sum() < 12:
        raise ValueError("Panel plane has insufficient inliers")
    center = panel[best].mean(axis=0)
    _, singular, vt = np.linalg.svd(panel[best]-center, full_matrices=False)
    if singular[1] / np.sqrt(best.sum()) < 0.015:
        raise ValueError("Panel support is too narrow")
    n = vt[-1]
    if np.dot(n, camera-center) < 0:
        n = -n
    if abs(np.dot(n, camera-center)) < 0.05:
        raise ValueError("Panel viewed too obliquely")
    hc = np.median(handle, axis=0)
    _, hs, hv = np.linalg.svd(handle-hc, full_matrices=False)
    if hs[0] < 2 * hs[1]:
        raise ValueError("Handle axis ambiguous")
    x = hv[0] - np.dot(hv[0], n)*n
    if np.linalg.norm(x) < 0.5:
        raise ValueError("Handle axis inconsistent with panel")
    x /= np.linalg.norm(x)
    if x[np.argmax(np.abs(x))] < 0:
        x = -x
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x, np.cross(n, x), n))
    result[:3, 3] = hc
    return {"handle_to_base": result.tolist(), "normal_base": n.tolist(),
            "plane_inlier_ratio": float(best.mean())}


def calibrate(handle_to_base, tcp_to_base):
    """Only use synchronized handle and TCP observations from the same teaching pose."""
    return (np.linalg.inv(transform(handle_to_base)) @ transform(tcp_to_base)).tolist()


def grasp_targets(handle_to_base, *, grasp_offset_m=0.0, gripper_in_handle=None,
                  clearance_m=0.06):
    """Midpoint grasp: position at the handle centre, axes from the handle frame.

    grasp_offset_m shifts the TCP along the handle normal (+z points outward,
    so a negative value sinks the TCP toward the panel for finger wrap-around).
    gripper_in_handle is a rotation-only fixed offset of the TCP frame relative
    to the handle frame; identity means TCP axes equal handle axes and must be
    confirmed by the on-site pre-grasp alignment check before any pull.
    """
    if not np.isfinite(grasp_offset_m) or not -0.05 <= grasp_offset_m <= 0.05:
        raise ValueError("Invalid grasp offset along handle normal")
    if not 0.02 <= clearance_m <= 0.15:
        raise ValueError("Invalid pregrasp clearance")
    h = transform(handle_to_base)
    grip = h.copy()
    if gripper_in_handle is not None:
        rotation = transform(gripper_in_handle)
        if np.linalg.norm(rotation[:3, 3]) > 1e-6:
            raise ValueError("Gripper offset must be rotation-only")
        grip = h @ rotation
    grip[:3, 3] = h[:3, 3] + grasp_offset_m * h[:3, 2]
    pre = grip.copy()
    pre[:3, 3] += clearance_m * h[:3, 2]
    pull = []
    for distance in np.linspace(0.01, 0.30, 30):
        pose = grip.copy()
        pose[:3, 3] += distance * h[:3, 2]
        pull.append(pose.tolist())
    return {"pregrasp": pre.tolist(), "grasp": grip.tolist(), "pull": pull,
            "normal_base": h[:3, 2].tolist()}


def targets(handle_to_base, tcp_in_handle, *, clearance_m=0.06):
    if not 0.02 <= clearance_m <= 0.15:
        raise ValueError("Invalid pregrasp clearance")
    h = transform(handle_to_base)
    grip = h @ transform(tcp_in_handle)
    pre = grip.copy()
    pre[:3, 3] += clearance_m*h[:3, 2]
    pull = []
    for distance in np.linspace(0.01, 0.30, 30):
        pose = grip.copy()
        pose[:3, 3] += distance*h[:3, 2]
        pull.append(pose.tolist())
    return {"pregrasp": pre.tolist(), "grasp": grip.tolist(), "pull": pull,
            "normal_base": h[:3, 2].tolist()}
