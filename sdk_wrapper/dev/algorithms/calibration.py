"""Runtime calibration checks shared by RGB-D and eye-in-hand tools."""

from __future__ import annotations

import math
import typing

import numpy as np

from sdk_wrapper.dev.algorithms.geometry import quaternion_to_matrix

__all__ = ["rotation_error_deg", "compare_rgbd_calibration"]


def _intr_values(intr: typing.Any) -> tuple[float, float, float, float, int, int, list[float]]:
    if isinstance(intr, dict):
        k = list(intr.get("K") or [])
        if len(k) >= 9:
            d = [float(v) for v in (intr.get("D") or [])]
            return (
                float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                int(intr.get("width") or 0), int(intr.get("height") or 0), d,
            )
    return (
        float(intr.fx), float(intr.fy), float(intr.cx), float(intr.cy),
        int(getattr(intr, "width", 0) or 0),
        int(getattr(intr, "height", 0) or 0),
        [float(v) for v in getattr(intr, "distortion", ())],
    )


def rotation_error_deg(q_a: typing.Sequence[float], q_b: typing.Sequence[float]) -> float:
    """Relative rotation magnitude between two scalar-last quaternions."""
    ra = quaternion_to_matrix(*[float(v) for v in q_a[3:7]])
    rb = quaternion_to_matrix(*[float(v) for v in q_b[3:7]])
    rel = ra.T @ rb
    cosine = max(-1.0, min(1.0, (float(np.trace(rel)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def compare_rgbd_calibration(
    rgb_intrinsics: typing.Any,
    depth_intrinsics: typing.Any,
    rgb_extrinsic: typing.Sequence[float],
    depth_extrinsic: typing.Sequence[float],
    *,
    max_translation_delta_m: float = 0.002,
    max_rotation_delta_deg: float = 0.5,
    max_focal_delta_px: float = 1.0,
    max_principal_delta_px: float = 1.0,
) -> dict[str, typing.Any]:
    """Check whether color and depth share one calibrated optical frame.

    This validates the official runtime calibration values. It is not a
    substitute for a live checkerboard / edge-alignment acceptance image.
    """
    rfx, rfy, rcx, rcy, rw, rh, rd = _intr_values(rgb_intrinsics)
    dfx, dfy, dcx, dcy, dw, dh, dd = _intr_values(depth_intrinsics)
    rt = np.asarray(rgb_extrinsic[:3], dtype=np.float64)
    dt = np.asarray(depth_extrinsic[:3], dtype=np.float64)

    focal_delta = max(abs(rfx - dfx), abs(rfy - dfy))
    principal_delta = max(abs(rcx - dcx), abs(rcy - dcy))
    translation_delta = float(np.linalg.norm(rt - dt))
    rotation_delta = rotation_error_deg(rgb_extrinsic, depth_extrinsic)
    distortion_delta = 0.0
    if rd and dd:
        common = min(len(rd), len(dd))
        distortion_delta = float(np.max(np.abs(np.asarray(rd[:common]) - np.asarray(dd[:common]))))

    checks = {
        "resolution_match": (rw, rh) == (dw, dh) and rw > 0 and rh > 0,
        "focal_match": focal_delta <= float(max_focal_delta_px),
        "principal_point_match": principal_delta <= float(max_principal_delta_px),
        "translation_match": translation_delta <= float(max_translation_delta_m),
        "rotation_match": rotation_delta <= float(max_rotation_delta_deg),
        "distortion_match": distortion_delta <= 1e-6,
    }
    return {
        "calibration_match": all(checks.values()),
        "checks": checks,
        "rgb_size": [rw, rh],
        "depth_size": [dw, dh],
        "focal_delta_px": round(focal_delta, 6),
        "principal_delta_px": round(principal_delta, 6),
        "translation_delta_m": round(translation_delta, 8),
        "rotation_delta_deg": round(rotation_delta, 8),
        "distortion_delta": round(distortion_delta, 10),
        "note": "runtime calibration comparison; still requires one live RGB-D alignment acceptance check",
    }
