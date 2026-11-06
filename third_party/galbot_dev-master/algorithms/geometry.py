"""algorithms/geometry.py —— 3D 几何小工具（无 ros/无 pcl 依赖）。

为什么自己写：官方 SDK **不提供**"深度图 -> PointCloud"的接口
（PointCloud 只出现在算法层 DetectionResult 里），
所以反投影必须自己实现。官方公式在 tutorial 里出现过：
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

约定（与协议 §1 一致）：
  * 长度 m，角度 rad，四元数 **scalar-last**（x, y, z, w）—— 与 SDK 的
    get_sensor_extrinsic / get_transform 返回顺序一致。
"""

from __future__ import annotations

import math
import typing

import numpy as np

__all__ = [
    "quaternion_to_matrix",
    "extrinsic_to_matrix",
    "transform_point",
    "transform_points",
    "matrix_to_quaternion",
    "deproject",
    "deproject_points",
    "depth_valid_mask",
    "depth_stats",
    "mask_bbox",
    "mask_centroid",
    "bbox_from_uv",
    "angle_between_deg",
]


# ═══════════════════════════════════════════════════════════════════════════ #
# 四元数 / 变换
# ═══════════════════════════════════════════════════════════════════════════ #
def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """单位四元数(x,y,z,w) -> 3x3 旋转矩阵。非法/未归一化会自动归一化。"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def extrinsic_to_matrix(transform: typing.Sequence[float]) -> np.ndarray:
    """SDK 的 7 维外参 [x,y,z,qx,qy,qz,qw] -> 4x4 齐次矩阵。"""
    tf = np.asarray(transform, dtype=np.float64).reshape(-1)
    if tf.size != 7:
        raise ValueError(f"外参必须是 7 维 [x,y,z,qx,qy,qz,qw]，收到 {tf.size} 维")
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quaternion_to_matrix(*tf[3:7])
    mat[:3, 3] = tf[:3]
    return mat


def matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 -> (qx, qy, qz, qw)，scalar-last。"""
    m = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def transform_point(point: typing.Sequence[float], transform: typing.Sequence[float]) -> np.ndarray:
    """把相机系 1 个点变换到外参给出的参考系（SDK 外参 reference_frame，默认 base_link）。"""
    mat = extrinsic_to_matrix(transform)
    p = np.asarray(point, dtype=np.float64).reshape(-1)[:3]
    return (mat[:3, :3] @ p) + mat[:3, 3]


def transform_points(points: np.ndarray, transform: typing.Sequence[float]) -> np.ndarray:
    """N x 3 批量变换。"""
    mat = extrinsic_to_matrix(transform)
    arr = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return arr @ mat[:3, :3].T + mat[:3, 3]


# ═══════════════════════════════════════════════════════════════════════════ #
# 反投影
# ═══════════════════════════════════════════════════════════════════════════ #
def deproject(u: float, v: float, z: float, intrinsics: typing.Any) -> np.ndarray:
    """单像素反投影。intrinsics 需要 fx/fy/cx/cy（Intrinsics 数据类或 dict）。"""
    fx, fy, cx, cy = (float(intrinsics.fx), float(intrinsics.fy),
                      float(intrinsics.cx), float(intrinsics.cy))
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def deproject_points(
    depth_m: np.ndarray, intrinsics: typing.Any, *, stride: int = 8, max_depth_m: float | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """整张深度图 -> (N,3) 点 + (N,2) 像素坐标。stride>1 用于降采样，避免几十万点打爆 JSON。"""
    fx, fy, cx, cy = (float(intrinsics.fx), float(intrinsics.fy),
                      float(intrinsics.cx), float(intrinsics.cy))
    step = max(1, int(stride))
    depth = np.asarray(depth_m, dtype=np.float32)[::step, ::step]
    rows, cols = np.nonzero(np.isfinite(depth) & (depth > 0))
    z = depth[rows, cols].astype(np.float64)
    if max_depth_m is not None:
        keep = z <= float(max_depth_m)
        rows, cols, z = rows[keep], cols[keep], z[keep]

    u = (cols * step).astype(np.float64)
    v = (rows * step).astype(np.float64)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1), np.stack([u, v], axis=1)


def depth_valid_mask(depth_m: np.ndarray) -> np.ndarray:
    return np.isfinite(depth_m) & (np.asarray(depth_m) > 0)


def depth_stats(depth_m: np.ndarray, mask: np.ndarray | None = None) -> dict[str, typing.Any]:
    """深度统计。返回值为米，字段名显式带 _m，防止单位混用。"""
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = depth_valid_mask(depth)
    if mask is not None:
        valid &= np.asarray(mask, dtype=bool)
    if not valid.any():
        return {"valid": False, "count": 0}
    values = depth[valid]
    return {
        "valid": True,
        "count": int(values.size),
        "min_m": round(float(values.min()), 4),
        "max_m": round(float(values.max()), 4),
        "median_m": round(float(np.median(values)), 4),
        "p10_m": round(float(np.percentile(values, 10)), 4),
        "p90_m": round(float(np.percentile(values, 90)), 4),
        "std_m": round(float(values.std()), 4),
    }


# ═══════════════════════════════════════════════════════════════════════════ #
# 掩膜工具
# ═══════════════════════════════════════════════════════════════════════════ #
def mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    """掩膜 -> (x, y, w, h)；空掩膜返回 None。"""
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any() or not cols.any():
        return None
    y0, y1 = int(np.argmax(rows)), int(len(rows) - 1 - np.argmax(rows[::-1]))
    x0, x1 = int(np.argmax(cols)), int(len(cols) - 1 - np.argmax(cols[::-1]))
    return x0, y0, x1 - x0 + 1, y1 - y0 + 1


def mask_centroid(mask: np.ndarray) -> tuple[float, float] | None:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return float(xs.mean()), float(ys.mean())


def bbox_from_uv(u: float, v: float, size: int = 40, width: int = 0, height: int = 0) -> tuple[int, int, int, int]:
    x = int(max(0, min(width - size if width else 0, round(u - size / 2))))
    y = int(max(0, min(height - size if height else 0, round(v - size / 2))))
    return x, y, size, size


def angle_between_deg(a: typing.Sequence[float], b: typing.Sequence[float]) -> float:
    va, vb = np.asarray(a, np.float64), np.asarray(b, np.float64)
    na, nb = np.linalg.norm(va), np.linalg.norm(vb)
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return float(math.degrees(math.acos(max(-1.0, min(1.0, float(va @ vb) / (na * nb))))))
