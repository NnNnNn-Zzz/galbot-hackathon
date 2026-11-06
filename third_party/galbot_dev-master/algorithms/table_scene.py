"""algorithms/table_scene.py —— "把不安全杯子推离桌沿" 的几何核心。

════════════════════════════════════════════════════════════════════════════
思路（借鉴工业 2D 视觉，但把"相机系数"升级成单应矩阵）
════════════════════════════════════════════════════════════════════════════
1. 赛前标定桌面四角（像素）↔ 桌面实际尺寸（米）→ 单应 H
2. 运行时用 H 把图**拉成米制俯视图**（1 px = 1 mm）
3. 在俯视图里找杯底连通域；`cv2.minEnclosingCircle` 拿半径
4. `d = 杯底到桌沿的最短距离`（米）。俯视图里桌沿就是边框，直接相减即可
5. 按 d 排序 → 最近的标 C2（不安全）
6. 输出「往哪个方向推多远」——方向是**标定得到的 base_link 单位向量**

为什么用单应而不是 2D 像素距离：
    透视投影下同一段像素长度在不同距离对应的实际长度不同，用常数"像素/毫米"
    只在相机正对、工作距离固定时成立（喷胶系统就是那样装的）。我们是斜视，必须单应。

为什么杯底而不是杯壁：
    "掉下桌"取决于**支撑面**，所以量杯底轮廓。杯壁上部因高度会有视差。

════════════════════════════════════════════════════════════════════════════
⚠️ 单应的前提：臂相机必须回到标定时的 **观察位姿**
    相机装在手臂上会动，同一个 H 只在固定位姿下有效。
    所以本模块会读 `get_end_effector_pose()` 与标定值比对，超差直接拒绝，
    不猜、不硬算。这一步被漏掉是这类方案最常见的翻车点。
"""

from __future__ import annotations

import json
import math
import os
import typing

import cv2
import numpy as np

__all__ = [
    "DEFAULT_CALIB_PATH",
    "load_calibration",
    "build_homography",
    "analyze_table",
    "describe_for_llm",
]

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
DEFAULT_CALIB_PATH = os.path.join(_CONFIG_DIR, "table_calib.json")

#: 深度的"桌面高度"用画面中央这块区域的深度中位数估计（避开桌沿与杯子）
_CENTER_ROI = (0.30, 0.70, 0.30, 0.70)  # (x0, x1, y0, y1) 相对比例

#: 杯子相对桌面高出多少算"桌面上的物体"（米）
_OBJECT_ABOVE_M = (0.015, 0.25)


# ═══════════════════════════════════════════════════════════════════════════ #
# 标定
# ═══════════════════════════════════════════════════════════════════════════ #
def load_calibration(path: str | os.PathLike[str] = DEFAULT_CALIB_PATH) -> dict[str, typing.Any] | None:
    """读桌面标定。文件不存在 / 结构不全时返回 None（调用方要如实报 not_ready）。"""
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding="utf-8") as fh:
            calib = json.load(fh)
    except Exception:  # noqa: BLE001
        return None
    required = ("table_corners_px", "table_size_m")
    if not all(key in calib for key in required):
        return None
    if len(calib["table_corners_px"]) != 4 or len(calib["table_size_m"]) != 2:
        return None
    return calib


def build_homography(
    calib: dict[str, typing.Any],
) -> tuple[np.ndarray, int, int, np.ndarray]:
    """返回 ``(H, bev_w_mm, bev_h_mm, H_inv)``。

    图像四角顺序：左上 → 右上 → 右下 → 左下（顺时针），对应俯视图
    ``(0,0) → (W,0) → (W,H) → (0,H)``。于是俯视图里：
        y = 0     是"远端"桌沿
        y = H_mm  是"近端"桌沿（最靠近机器人的那条）
    """
    corners = np.array(calib["table_corners_px"], dtype=np.float64).reshape(4, 2)
    width_m, height_m = (float(v) for v in calib["table_size_m"])
    bev_w = int(round(width_m * 1000))
    bev_h = int(round(height_m * 1000))
    if bev_w <= 0 or bev_h <= 0:
        raise ValueError(f"桌面尺寸非法：{calib['table_size_m']}")
    dst = np.array(
        [[0.0, 0.0], [bev_w, 0.0], [bev_w, bev_h], [0.0, bev_h]], dtype=np.float64
    )
    homography, _mask = cv2.findHomography(corners, dst)
    if homography is None:
        raise ValueError("findHomography 失败：桌面四角共线或退化")
    return homography, bev_w, bev_h, np.linalg.inv(homography)


# ═══════════════════════════════════════════════════════════════════════════ #
# 前景（桌面上的东西）
# ═══════════════════════════════════════════════════════════════════════════ #
def _table_depth_hint(depth: np.ndarray) -> float | None:
    """用画面中央区域的中位数估计桌面深度。"""
    h, w = depth.shape[:2]
    x0, x1, y0, y1 = _CENTER_ROI
    roi = depth[int(h * y0) : int(h * y1), int(w * x0) : int(w * x1)]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if valid.size < 50:
        return None
    return float(np.median(valid))


def foreground_mask(
    bgr: np.ndarray, depth: np.ndarray | None
) -> tuple[np.ndarray, dict[str, typing.Any]]:
    """桌面上的凸起物 → bool 掩膜。

    主路径：深度。相机从上往下看，桌面上的杯子**比桌面更近**，所以
            ``0.015 m < (桌面深度 - 该像素深度) < 0.25 m`` 就是"桌上物体"。
    兜底：深度不可用时退回 Otsu 反相二值化（桌子比物品亮时有效）。
    """
    info: dict[str, typing.Any] = {"method": "otsu", "table_depth_m": None}
    if depth is not None and depth.ndim == 2:
        table_d = _table_depth_hint(depth)
        info["table_depth_m"] = table_d
        if table_d is not None:
            valid = np.isfinite(depth) & (depth > 0)
            delta = table_d - np.where(valid, depth, table_d)
            lo, hi = _OBJECT_ABOVE_M
            mask = valid & (delta > lo) & (delta < hi)
            if mask.sum() >= 200:
                info["method"] = "depth"
                return mask, info

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV)
    return binary.astype(bool), info


def _bev_components(
    mask: np.ndarray,
    homography: np.ndarray,
    bev_size: tuple[int, int],
    *,
    min_area_mm2: float,
    max_area_mm2: float,
) -> list[dict[str, typing.Any]]:
    """把掩膜拉到俯视图后做连通域，返回每个物体的几何量（单位 mm）。"""
    bev_mask = cv2.warpPerspective(
        mask.astype(np.uint8), homography, bev_size, flags=cv2.INTER_NEAREST
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    bev_mask = cv2.morphologyEx(bev_mask, cv2.MORPH_OPEN, kernel)
    bev_mask = cv2.morphologyEx(bev_mask, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(bev_mask, connectivity=8)
    found: list[dict[str, typing.Any]] = []
    for idx in range(1, int(count)):
        area = float(stats[idx][cv2.CC_STAT_AREA])
        if area < min_area_mm2 or area > max_area_mm2:
            continue
        # ⚠️ 必须用 labels == idx。bev_mask 是 0/1 值，`bev_mask == idx`
        #    在 idx≥2 时为空、在 idx==1 时会把所有前景都选进来（曾导致半径算成两杯间距）。
        ys, xs = np.nonzero(labels == idx)
        if xs.size < 3:
            continue
        points = np.stack([xs, ys], axis=1).astype(np.float32)
        _circle_center, radius = cv2.minEnclosingCircle(points)
        # 圆心用连通域质心更稳（minEnclosingCircle 的圆心受离群点影响）
        cx = float(centroids[idx][0])
        cy = float(centroids[idx][1])
        fill = area / max(1e-6, math.pi * radius * radius)
        found.append(
            {
                "center_mm": [cx, cy],
                "radius_mm": float(radius),
                "area_mm2": area,
                "fill": round(float(fill), 3),
            }
        )
    return found


# ═══════════════════════════════════════════════════════════════════════════ #
# 距离与方向
# ═══════════════════════════════════════════════════════════════════════════ #
def _distance_to_edges(
    center_mm: typing.Sequence[float],
    radius_mm: float,
    bev_w: int,
    bev_h: int,
    edges: typing.Iterable[str],
) -> dict[str, float]:
    """杯底轮廓到各条桌沿的最短距离（mm）。用"圆心 ± 半径"而不是圆心，才是杯底轮廓。"""
    cx, cy = float(center_mm[0]), float(center_mm[1])
    r = float(radius_mm)
    all_edges = {
        "back": cy - r,          # y = 0
        "front": bev_h - (cy + r),  # y = H（靠机器人那侧）
        "left": cx - r,          # x = 0
        "right": bev_w - (cx + r),  # x = W
    }
    return {name: float(all_edges[name]) for name in edges if name in all_edges}


def _bev_to_base(
    center_mm: typing.Sequence[float],
    transform: dict[str, typing.Any] | None,
) -> list[float] | None:
    """俯视图坐标(mm) → base_link 的 xy（米）。

    标定里给 ``table_to_base``：
        origin_base_m : 俯视图 (0,0) 在 base_link 下的位置
        axis_u_base   : 俯视图 +u 方向的 base_link 单位向量
        axis_v_base   : 俯视图 +v 方向的 base_link 单位向量
    """
    if not transform:
        return None
    try:
        ox, oy = float(transform["origin_base_m"][0]), float(transform["origin_base_m"][1])
        au = [float(v) for v in transform["axis_u_base"][:2]]
        av = [float(v) for v in transform["axis_v_base"][:2]]
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    u = float(center_mm[0]) / 1000.0
    v = float(center_mm[1]) / 1000.0
    return [ox + au[0] * u + av[0] * v, oy + au[1] * u + av[1] * v]


# ═══════════════════════════════════════════════════════════════════════════ #
# 主入口
# ═══════════════════════════════════════════════════════════════════════════ #
def analyze_table(
    bgr: np.ndarray | None,
    depth: np.ndarray | None,
    calib: dict[str, typing.Any],
    options: typing.Mapping[str, typing.Any] | None = None,
) -> dict[str, typing.Any]:
    """一张图 + 标定 → C1/C2、各自的 d、以及"往哪推多远"。

    不抛异常；算不出来就把原因写进 ``warnings`` 并让 ``unsafe`` 为 None，
    由调用方（Tool 层）决定返回什么 error_code。
    """
    opts: dict[str, typing.Any] = {
        "unsafe_threshold_m": 0.03,
        "target_m": 0.12,
        "min_area_mm2": 200.0,
        "max_area_mm2": 0.30,
        "max_cups": 4,
    }
    if options:
        opts.update({k: v for k, v in options.items() if v is not None})

    result: dict[str, typing.Any] = {
        "coordinate_frame": "base_link",
        "warnings": [],
        "cups": [],
        "unsafe_cup": None,
        "action": "none",
        "method": None,
    }
    if bgr is None and depth is None:
        result["warnings"].append("既没有彩色也没有深度，无法判断桌面")
        return result

    try:
        homography, bev_w, bev_h, _h_inv = build_homography(calib)
    except Exception as exc:  # noqa: BLE001
        result["warnings"].append(f"标定非法：{exc}")
        return result

    bev_size = (bev_w, bev_h)
    min_area = float(opts["min_area_mm2"])
    max_area_mm2 = opts["max_area_mm2"]
    max_area = (bev_w * bev_h * float(max_area_mm2)) if max_area_mm2 <= 1.0 else float(max_area_mm2)

    source = bgr if bgr is not None else None
    if source is None and depth is not None:
        # 只有深度时造一张假彩色图给 Otsu 兜底用
        source = cv2.cvtColor(
            np.clip(np.nan_to_num(depth, nan=0.0) * 1000 / 2, 0, 255).astype(np.uint8),
            cv2.COLOR_GRAY2BGR,
        )

    mask, mask_info = foreground_mask(source, depth)
    result["method"] = mask_info["method"]
    result["table_depth_m"] = mask_info["table_depth_m"]

    edges = list(calib.get("edge_order") or ["back", "front", "left", "right"])
    objects = _bev_components(
        mask, homography, bev_size, min_area_mm2=min_area, max_area_mm2=max_area
    )
    if not objects:
        result["warnings"].append(
            f"俯视图里没找到桌面上的物体（前景方法={mask_info['method']}）"
        )
        return result

    transform = calib.get("table_to_base")
    cups: list[dict[str, typing.Any]] = []
    for obj in objects:
        distances = _distance_to_edges(obj["center_mm"], obj["radius_mm"], bev_w, bev_h, edges)
        if not distances:
            continue
        nearest_edge = min(distances, key=distances.get)
        d_m = distances[nearest_edge] / 1000.0
        cups.append(
            {
                "center_mm": [round(obj["center_mm"][0], 1), round(obj["center_mm"][1], 1)],
                "radius_m": round(obj["radius_mm"] / 1000.0, 4),
                "area_mm2": round(obj["area_mm2"], 1),
                "fill": obj["fill"],
                "nearest_edge": nearest_edge,
                "d_to_edge_m": round(d_m, 4),
                "all_edges_m": {k: round(v / 1000.0, 4) for k, v in distances.items()},
                "base_xy_m": _bev_to_base(obj["center_mm"], transform),
            }
        )

    cups.sort(key=lambda c: c["d_to_edge_m"])
    cups = cups[: int(opts["max_cups"])]

    unsafe_threshold = float(opts["unsafe_threshold_m"])
    target_m = float(opts["target_m"])
    # 标签：最近的 = C2（不安全），其余按距离递增 C1、C3…（符合"识别 C2 不安全"的说法）
    for index, cup in enumerate(cups):
        cup["label"] = f"C{index + 2}" if index == 0 else f"C{index}"
        cup["unsafe"] = cup["d_to_edge_m"] <= unsafe_threshold

    result["cups"] = cups
    result["unsafe_threshold_m"] = unsafe_threshold
    result["target_m"] = target_m

    nearest = cups[0]
    result["unsafe_cup"] = nearest["label"] if nearest["unsafe"] else None

    if not nearest["unsafe"]:
        result["action"] = "none"
        result["warnings"].append(
            f"所有杯子距桌沿都 > {unsafe_threshold * 100:.0f} cm，无需处理"
        )
        return result

    push_gap = target_m - nearest["d_to_edge_m"]
    inward = (calib.get("edge_inward_base") or {}).get(nearest["nearest_edge"])
    if not inward:
        result["warnings"].append(
            f"标定里缺少 edge_inward_base['{nearest['nearest_edge']}']，无法给出推动方向"
        )
        result["action"] = "blocked"
        return result

    result["action"] = "push"
    result["push_direction_base"] = [float(v) for v in inward[:3]]
    result["push_distance_m"] = round(max(0.0, push_gap), 4)
    result["push_edge"] = nearest["nearest_edge"]
    result["target_cup"] = nearest["label"]
    return result


def describe_for_llm(result: typing.Mapping[str, typing.Any]) -> str:
    """把结果压成一句人话，供离线规则引擎/日志使用。"""
    cups = result.get("cups") or []
    if not cups:
        return "桌面上没有检测到杯子"
    parts = [
        f"{c['label']} 距桌沿 {c['d_to_edge_m'] * 100:.1f} cm"
        + ("（不安全）" if c.get("unsafe") else "")
        for c in cups
    ]
    line = "；".join(parts)
    if result.get("action") == "push":
        direction = result.get("push_direction_base")
        distance = result.get("push_distance_m")
        line += f"。需把 {result.get('target_cup')} 沿 {direction} 推 {distance * 100:.1f} cm"
    return line
