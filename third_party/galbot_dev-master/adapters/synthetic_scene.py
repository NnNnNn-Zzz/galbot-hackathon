"""合成场景：给 MockAdapter 提供**与真机同格式**的相机 payload。

为什么放在 adapters/ 而不是 agent/：
    它属于"硬件替身"，和 mock_adapter 是同一层；放这里才不会出现
    adapters → agent 的反向依赖。

关键点：输出的是**编码后的字节**（JPEG / 裸 16UC1），不是 numpy 数组。
这样 Mock 和真机走完全相同的解码路径，才不会"模拟通过、真机炸"。
"""

from __future__ import annotations

import math
import typing

import cv2
import numpy as np

__all__ = [
    "WIDTH", "HEIGHT", "FX", "FY", "CX", "CY", "DEPTH_SCALE", "SCENE_OBJECTS",
    "render_scene", "render_cup_table_scene", "SyntheticCameraMixin",
]

WIDTH, HEIGHT = 640, 512
FX = FY = 520.0
CX, CY = 320.0, 256.0
DEPTH_SCALE = 1000  # 1000 counts = 1 m，与真机常见 depth_scale 一致

#: 合成目标：label -> (中心 u, v, 半径, 深度 m)
SCENE_OBJECTS: dict[str, tuple[int, int, int, float]] = {
    "red": (230, 300, 46, 0.45),
    "blue": (430, 320, 36, 0.62),
    "green": (330, 215, 26, 0.95),
}
_BGR = {"red": (40, 40, 210), "blue": (200, 90, 30), "green": (60, 170, 70)}
BACKGROUND_DEPTH_M = 1.2
TABLE_DEPTH_M = 0.80
TABLE_TOP_Y = 340

_TIMESTAMP_NS = 1_730_000_000_000_000_000


def render_scene() -> tuple[np.ndarray, np.ndarray]:
    """渲染 (BGR uint8, depth float32 米)。确定性，不依赖随机种子。"""
    rng = np.random.default_rng(20260627)

    ramp = np.linspace(70, 130, HEIGHT, dtype=np.float32)
    base = np.repeat(ramp[:, None, None], WIDTH, axis=1)
    base = np.repeat(base, 3, axis=2)
    noise = rng.integers(-8, 9, size=base.shape, dtype=np.int16)
    bgr = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    bgr[TABLE_TOP_Y:] = (95, 100, 105)  # 桌面

    for label, (u, v, radius, _depth) in SCENE_OBJECTS.items():
        if label == "green":
            cv2.circle(bgr, (u, v), radius, _BGR[label], -1)
        else:
            cv2.rectangle(bgr, (u - radius, v - radius), (u + radius, v + radius), _BGR[label], -1)
        cv2.line(bgr, (u - radius, v), (u + radius, v), (250, 250, 250), 1)

    depth = np.full((HEIGHT, WIDTH), BACKGROUND_DEPTH_M, np.float32)
    depth[TABLE_TOP_Y:] = TABLE_DEPTH_M
    for label, (u, v, radius, obj_depth) in SCENE_OBJECTS.items():
        mask = np.zeros((HEIGHT, WIDTH), np.uint8)
        if label == "green":
            cv2.circle(mask, (u, v), radius, 255, -1)
        else:
            cv2.rectangle(mask, (u - radius, v - radius), (u + radius, v + radius), 255, -1)
        depth[mask.astype(bool)] = obj_depth

    holes = rng.random((HEIGHT, WIDTH)) < 0.01
    depth[holes] = 0.0  # 1% 无效像素，解码后应变成 NaN
    return bgr, depth


# ═══════════════════════════════════════════════════════════════════════════ #
# "两个杯子 + 桌沿" 场景（推杯子场景用）
# ═══════════════════════════════════════════════════════════════════════════ #
#: 桌面四角（图像像素，顺序 左上→右上→右下→左下）。
#: ⚠️ 必须与 config/table_calib.json 的 table_corners_px 保持一致。
CUP_TABLE_CORNERS_PX = ((90, 110), (550, 110), (610, 450), (30, 450))
#: 桌面实际尺寸（毫米）
CUP_TABLE_SIZE_MM = (800, 600)
_TABLE_DEPTH_M = 0.85
_CUP_TOP_DEPTH_M = 0.78        # 杯顶比桌面高 7 cm
_BACKGROUND_DEPTH_M = 2.0      # 桌外（地面）远得多，会被前景掩膜排除
#: 俯视图里的杯底 (cx_mm, cy_mm, r_mm)。y=600 是"靠近机器人"的那条桌沿。
CUP_BEV_MM = (
    (400, 544, 35),   # C2：距近端桌沿 600-(544+35)=21 mm → 不安全
    (200, 220, 35),   # C1：最近只到左边 200-35=165 mm → 安全
)
_CUP_BEV_COLOR = (60, 60, 70)
_TABLE_BEV_COLOR = (205, 205, 210)

#: 俯视图 +u / +v 单位向量在 base_link 下的方向（与 config/table_calib.json 一致）。
#: Mock 的"粗略物理"靠它把末端位移换算成杯子在俯视图里的移动。
CUP_TABLE_AXIS_U_BASE = (0.0, -1.0)
CUP_TABLE_AXIS_V_BASE = (-1.0, 0.0)


def base_delta_to_bev_mm(dx: float, dy: float) -> tuple[float, float]:
    """base_link 下的平面位移(米) → 俯视图里应移动的 (du_mm, dv_mm)。"""
    u = dx * CUP_TABLE_AXIS_U_BASE[0] + dy * CUP_TABLE_AXIS_U_BASE[1]
    v = dx * CUP_TABLE_AXIS_V_BASE[0] + dy * CUP_TABLE_AXIS_V_BASE[1]
    return (u * 1000.0, v * 1000.0)


def _cup_table_homography() -> tuple[np.ndarray, np.ndarray]:
    """桌面四角(px) ↔ 俯视图(mm)。与 algorithms/table_scene.build_homography 同一套映射。"""
    src = np.array(CUP_TABLE_CORNERS_PX, dtype=np.float64)
    w_mm, h_mm = CUP_TABLE_SIZE_MM
    dst = np.array([[0.0, 0.0], [w_mm, 0.0], [w_mm, h_mm], [0.0, h_mm]], dtype=np.float64)
    homography, _ = cv2.findHomography(src, dst)
    return homography, np.linalg.inv(homography)


def render_cup_table_scene(
    offsets_mm: typing.Sequence[typing.Sequence[float]] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """渲染"桌面上两个杯子"的 (BGR, depth)。

    做法：先在**米制俯视图**里画桌子和杯子，再用单应逆变换回相机视角。
    好处是杯子在俯视图里的位置就是真值（C2 距桌沿 21 mm），测试可以直接对上。

    ``offsets_mm`` 给每个杯子一个 ``(du, dv)`` 位移，用于让 Mock 的"粗略物理"
    能反映"推了之后杯子动了"——否则闭环复测永远是同一个数。
    """
    w_mm, h_mm = CUP_TABLE_SIZE_MM
    rng = np.random.default_rng(20260628)

    bev = np.full((h_mm, w_mm, 3), _TABLE_BEV_COLOR, np.uint8)
    noise = rng.integers(-4, 5, size=(h_mm, w_mm, 1), dtype=np.int16)
    bev = np.clip(bev.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    bev_depth = np.full((h_mm, w_mm), _TABLE_DEPTH_M, np.float32)
    for index, (cx, cy, radius) in enumerate(CUP_BEV_MM):
        du = dv = 0.0
        if offsets_mm is not None and index < len(offsets_mm):
            du, dv = float(offsets_mm[index][0]), float(offsets_mm[index][1])
        px = int(round(cx + du))
        py = int(round(cy + dv))
        cv2.circle(bev, (px, py), radius, _CUP_BEV_COLOR, -1)
        cv2.circle(bev_depth, (px, py), radius, _CUP_TOP_DEPTH_M, -1)

    _homography, h_inv = _cup_table_homography()
    bgr = cv2.warpPerspective(bev, h_inv, (WIDTH, HEIGHT), borderValue=(70, 70, 78))
    depth = cv2.warpPerspective(
        bev_depth, h_inv, (WIDTH, HEIGHT),
        flags=cv2.INTER_NEAREST, borderValue=_BACKGROUND_DEPTH_M,
    )
    return bgr, depth


class SyntheticCameraMixin:
    """按真机 dict 形状返回相机数据。混入任何 adapter 即可获得"眼睛"。

    class attribute 可被实例覆盖：
        scene_name              -> "objects"（默认，三个色块）| "cup_table"（两个杯子 + 桌沿）
        fail_rgb / fail_depth   -> 模拟传感器未就绪（返回空 dict）
        depth_as_png            -> 走 compressedDepth PNG 分支
    """

    scene_name: str = "objects"
    fail_rgb: bool = False
    fail_depth: bool = False
    depth_as_png: bool = False
    #: 每个杯子在俯视图里的额外位移（mm）。Mock 的"粗略物理"会往里累加。
    cup_offsets_mm: list[list[float]] | None = None

    def _scene(self) -> tuple[np.ndarray, np.ndarray]:
        name = str(getattr(self, "scene_name", "objects"))
        if name == "cup_table":
            return render_cup_table_scene(getattr(self, "cup_offsets_mm", None))
        return render_scene()

    def push_in_scene(self, dx: float, dy: float) -> None:
        """粗略物理：把离近端桌沿最近的杯子按 (dx,dy) 米推走。

        只为让 Mock 的闭环复测能看到"距离真的变大了"。真机不需要这个。
        """
        if str(getattr(self, "scene_name", "objects")) != "cup_table":
            return
        offsets = getattr(self, "cup_offsets_mm", None)
        if offsets is None:
            offsets = [[0.0, 0.0] for _ in CUP_BEV_MM]
            self.cup_offsets_mm = offsets
        nearest = max(
            range(len(CUP_BEV_MM)),
            key=lambda i: CUP_BEV_MM[i][1] + offsets[i][1],
        )
        du, dv = base_delta_to_bev_mm(dx, dy)
        offsets[nearest][0] += du
        offsets[nearest][1] += dv

    # ---- 协议实现 ----
    def get_rgb_data(self, sensor: str) -> dict[str, typing.Any]:
        if self.fail_rgb:
            return {}
        bgr, _ = self._scene()
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            return {}
        return {
            "format": "jpeg",
            "width": WIDTH,
            "height": HEIGHT,
            "stride_bytes": [0],
            "plane_count": 1,
            "data": encoded.tobytes(),
            "timestamp_ns": _TIMESTAMP_NS,
            "header": {"frame_id": f"{str(sensor).lower()}_optical_frame",
                       "timestamp_ns": _TIMESTAMP_NS},
        }

    def get_depth_data(self, sensor: str) -> dict[str, typing.Any]:
        if self.fail_depth:
            return {}
        _, depth = self._scene()
        raw = np.clip(depth * DEPTH_SCALE, 0, 65535).astype(np.uint16)
        if self.depth_as_png:
            ok, encoded = cv2.imencode(".png", raw)
            data = encoded.tobytes() if ok else raw.tobytes()
            fmt = "16UC1; compressedDepth png"
        else:
            data = raw.tobytes()
            fmt = "16UC1"
        return {
            "format": fmt,
            "width": WIDTH,
            "height": HEIGHT,
            "depth_scale": DEPTH_SCALE,
            "data": data,
            "timestamp_ns": _TIMESTAMP_NS,
            "header": {"frame_id": f"{str(sensor).lower()}_optical_frame",
                       "timestamp_ns": _TIMESTAMP_NS},
        }

    def get_camera_intrinsic(self, sensor: str) -> dict[str, typing.Any]:
        return {
            "width": WIDTH,
            "height": HEIGHT,
            "K": [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0],
            "D": [0.0, 0.0, 0.0, 0.0, 0.0],
            "distortion_model": "plumb_bob",
            "camera_type": "synthetic_rgbd",
            "header": {"frame_id": f"{str(sensor).lower()}_optical_frame",
                       "timestamp_ns": _TIMESTAMP_NS},
        }

    def get_sensor_extrinsic(
        self, sensor: str, reference_frame: str = "base_link"
    ) -> tuple[list[float], int]:
        # 相机装在左臂末端附近：前 0.35 m、左 0.12 m、高 0.55 m，绕 z 转 10°
        yaw = math.radians(10.0)
        return (
            [0.35, 0.12, 0.55, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
            _TIMESTAMP_NS,
        )
