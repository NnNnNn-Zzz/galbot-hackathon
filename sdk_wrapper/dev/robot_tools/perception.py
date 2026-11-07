"""robot_tools/perception.py —— B（"眼睛"）负责的感知层。

════════════════════════════════════════════════════════════════════════════
这个文件回答三个问题
════════════════════════════════════════════════════════════════════════════
1. 官方 SDK 到底返回什么？
     robot.get_rgb_data(SensorType.X, RgbOutputFormat.JPEG, True) -> dict
         {"header": Header, "format": "jpeg"/..., "width", "height",
          "stride_bytes", "plane_offset_bytes", "plane_count", "data": bytes}
     robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)     -> dict
         {"header", "format", "depth_scale", "width", "height", "data": bytes}
     robot.get_camera_intrinsic(SensorType.X)                   -> dict
         {"header", "width", "height", "K": [9], "D": [...], ...}
     robot.get_sensor_extrinsic(SensorType.X, "base_link")       -> ([x,y,z,qx,qy,qz,qw], ts)

2. 谁负责把 dict 变成 numpy？
     B（本文件）。A 的 adapter 只做"透传 SDK 原始 payload"，
     这样 Mock 和真机走**同一套解码代码**，不会出现"Mock 通过、真机炸"。

3. 大模型看到什么？
     只看到 capture_id / rgb_ref / depth_ref / 结构化 3D 结果，
     永远看不到也没法传入任意文件路径。

"""

from __future__ import annotations

import functools
import inspect
import json
import math
import os
import time
import typing
from dataclasses import dataclass

import cv2
import numpy as np

from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    TaskStatus,
    ToolError,
    ToolResult,
    failed,
    ok,
)
from sdk_wrapper.dev.frame_store import FrameStore

__all__ = [
    "CameraSpec",
    "CameraRegistry",
    "CameraAdapter",
    "Intrinsics",
    "PerceptionTool",
    "decode_rgb_payload",
    "decode_depth_payload",
    "normalize_payload",
]

DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "cameras.json")


# ═══════════════════════════════════════════════════════════════════════════ #
# 1. 相机白名单
# ═══════════════════════════════════════════════════════════════════════════ #
@dataclass(frozen=True)
class CameraSpec:
    """一个已登记的机位。camera_id 是别名，sensor 是 SDK SensorType 名。"""

    camera_id: str
    sensor: str
    depth_sensor: str | None = None
    kind: str = "rgb"
    depth_alignment: str = "none"
    notes: tuple[str, ...] = ()

    @property
    def has_rgb(self) -> bool:
        return not self.kind.startswith("depth_only")

    @property
    def has_depth(self) -> bool:
        return self.depth_sensor is not None


# SensorType 的合法成员名（对照 galbot_sdk/g1/__init__.pyi:5128-5176 抄的）
SENSOR_TYPE_NAMES: frozenset[str] = frozenset(
    {
        "HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA",
        "LEFT_ARM_CAMERA", "RIGHT_ARM_CAMERA",
        "LEFT_ARM_DOWN_CAMERA", "RIGHT_ARM_DOWN_CAMERA",
        "LEFT_ARM_DEPTH_CAMERA", "RIGHT_ARM_DEPTH_CAMERA",
        "LEFT_ARM_INFRA_CAMERA_1", "LEFT_ARM_INFRA_CAMERA_2",
        "RIGHT_ARM_INFRA_CAMERA_1", "RIGHT_ARM_INFRA_CAMERA_2",
        "LEFT_FRONT_SURROUND_CAMERA", "RIGHT_FRONT_SURROUND_CAMERA",
        "LEFT_REAR_SURROUND_CAMERA", "RIGHT_REAR_SURROUND_CAMERA",
        "BASE_LIDAR", "HEAD_LIDAR", "BACK_LIDAR", "CHASSIS_LIDAR",
        "HEAD_IMU", "BACK_IMU", "CHASSIS_IMU", "TORSO_IMU", "LIDAR_IMU",
        "BASE_ULTRASONIC",
    }
)


class CameraRegistry:
    """相机白名单 + 别名解析。

    接受三种写法，都是同一个机位：
        "left_arm"            (别名，config 里的 key)
        "LEFT_ARM_CAMERA"     (SDK SensorType 名)
        "左臂"                (aliases 表里的中文)
    """

    def __init__(self, cameras: dict[str, CameraSpec], aliases: dict[str, str], default: str) -> None:
        self._cameras = dict(cameras)
        self._aliases = {k.lower(): v for k, v in aliases.items()}
        self.default = default
        # SDK 名 -> 别名，允许直接用 SensorType 名调用
        self._by_sensor = {spec.sensor.lower(): cid for cid, spec in self._cameras.items()}
        self._by_depth_sensor = {
            spec.depth_sensor.lower(): cid
            for cid, spec in self._cameras.items()
            if spec.depth_sensor
        }

    # ---- 构造 ----
    @classmethod
    def from_dict(cls, payload: typing.Mapping[str, typing.Any]) -> "CameraRegistry":
        cameras: dict[str, CameraSpec] = {}
        for cid, raw in (payload.get("cameras") or {}).items():
            sensor = raw["sensor"]
            if sensor not in SENSOR_TYPE_NAMES:
                raise ValueError(
                    f"camera {cid!r} 的 sensor={sensor!r} 不是合法的 SensorType 成员名；"
                    f"可用名见 galbot_sdk.g1.SensorType"
                )
            notes = raw.get("note") or []
            if isinstance(notes, str):
                notes = [notes]
            if raw.get("risk"):
                notes = list(notes) + [raw["risk"]]
            if raw.get("scale_hint"):
                notes = list(notes) + [raw["scale_hint"]]
            cameras[cid] = CameraSpec(
                camera_id=cid,
                sensor=sensor,
                depth_sensor=raw.get("depth_sensor"),
                kind=raw.get("kind", "rgb"),
                depth_alignment=raw.get("depth_alignment", "none"),
                notes=tuple(notes),
            )
        if not cameras:
            raise ValueError("cameras.json 里 cameras 为空")
        default = payload.get("default_camera") or next(iter(cameras))
        if default not in cameras:
            raise ValueError(f"default_camera={default!r} 未在 cameras 中登记")
        return cls(cameras, payload.get("aliases") or {}, default)

    @classmethod
    def from_file(cls, path: str | os.PathLike[str] = DEFAULT_CONFIG_PATH) -> "CameraRegistry":
        with open(path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))

    # ---- 查询 ----
    @property
    def camera_ids(self) -> list[str]:
        return list(self._cameras)

    def resolve(self, camera_id: typing.Any) -> CameraSpec:
        """把用户/大模型给的东西解析成 CameraSpec；失败抛 rejected+invalid_argument。"""
        if isinstance(camera_id, CameraSpec):
            return camera_id
        if camera_id is None:
            return self._cameras[self.default]
        if not isinstance(camera_id, str) or not camera_id.strip():
            raise ToolError(
                ErrorCode.INVALID_ARGUMENT,
                f"camera_id 必须是非空字符串，收到 {camera_id!r}",
                {"legal_camera_ids": self.camera_ids},
            )
        key = camera_id.strip()
        lowered = key.lower()
        if key in self._cameras:
            return self._cameras[key]
        if lowered in self._aliases:
            return self._cameras[self._aliases[lowered]]
        if lowered in self._by_sensor:
            return self._cameras[self._by_sensor[lowered]]
        if lowered in self._by_depth_sensor:
            return self._cameras[self._by_depth_sensor[lowered]]
        raise ToolError(
            ErrorCode.INVALID_ARGUMENT,
            f"未注册的相机 {camera_id!r}；请使用 {self.camera_ids} 之一",
            {"legal_camera_ids": self.camera_ids},
        )

    def describe_all(self) -> list[dict[str, typing.Any]]:
        return [
            {
                "camera_id": spec.camera_id,
                "sensor": spec.sensor,
                "has_rgb": spec.has_rgb,
                "has_depth": spec.has_depth,
                "depth_sensor": spec.depth_sensor,
                "kind": spec.kind,
                "notes": list(spec.notes),
            }
            for spec in self._cameras.values()
        ]


# ═══════════════════════════════════════════════════════════════════════════ #
# 2. Adapter 协议（A 负责实现）
# ═══════════════════════════════════════════════════════════════════════════ #
class CameraAdapter(typing.Protocol):
    """A 的 adapters/base.py 需要满足的**最小**能力，用于感知。

    约定（写给 A 看）：
      * 入参 `sensor` 是 **SDK SensorType 的成员名字符串**，如 "LEFT_ARM_CAMERA"。
        真机适配器内部做 `SensorType.__members__[sensor]`；
        注意 `SensorType("LEFT_ARM_CAMERA")` 会 TypeError —— 它只吃 int。
      * `mode` 属性必须是 "mock" 或 "real"，且**不允许**在 real 失败时改回 mock。
      * 返回 SDK 原始 dict 即可（本文件的 normalize_* 会兼容）；也可返回已归一化 dict。
      * 取不到数据时返回 None / 空 dict，本文件会转成 not_ready，而不是伪造成功。
    """

    mode: str

    def get_rgb_data(self, sensor: str) -> typing.Any: ...
    def get_depth_data(self, sensor: str) -> typing.Any: ...
    def get_camera_intrinsic(self, sensor: str) -> typing.Any: ...
    def get_sensor_extrinsic(self, sensor: str, reference_frame: str = "base_link") -> typing.Any: ...


# ═══════════════════════════════════════════════════════════════════════════ #
# 3. payload 归一化（吃 SDK dict / 吃归一化 dict 都行）
# ═══════════════════════════════════════════════════════════════════════════ #
def _attr(obj: typing.Any, name: str, default: typing.Any = None) -> typing.Any:
    """SDK 的 Header 是 pybind11 对象；Mock 可能是 dict。两种都读。"""
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def _header_timestamp_ns(payload: typing.Mapping[str, typing.Any]) -> int:
    hdr = payload.get("header")
    ts = _attr(hdr, "timestamp_ns", None)
    if ts is None:
        ts = payload.get("timestamp_ns")
    try:
        return int(ts or 0)
    except (TypeError, ValueError):
        return 0


def _header_frame_id(payload: typing.Mapping[str, typing.Any]) -> str:
    hdr = payload.get("header")
    frame = _attr(hdr, "frame_id", None) or payload.get("frame_id") or ""
    return str(frame)


def normalize_payload(kind: str, payload: typing.Any) -> dict[str, typing.Any]:
    """把 adapter 给的东西统一成内部格式。kind ∈ {rgb, depth, intrinsic, extrinsic}。"""
    if payload is None:
        raise ToolError(ErrorCode.NOT_READY, f"{kind}: adapter 返回 None（传感器未启用或没数据）")
    if isinstance(payload, (bytes, bytearray)):
        payload = {"data": payload}
    # SDK 的 get_sensor_extrinsic 返回 (list[7], timestamp_ns) 元组，先于通用分支处理
    if (
        kind == "extrinsic"
        and isinstance(payload, (tuple, list))
        and len(payload) == 2
        and isinstance(payload[0], (list, tuple))
    ):
        transform, ts = payload
        return {
            "transform": [float(v) for v in transform[:7]],
            "timestamp_ns": int(ts or 0),
            "reference_frame": "base_link",
            "_timestamp_ns": int(ts or 0),
            "_frame_id": "",
        }
    if not isinstance(payload, typing.Mapping):
        # pybind11 的 RgbData/DepthData 对象
        payload = {name: _attr(payload, name) for name in
                   ("data", "format", "width", "height", "depth_scale",
                    "stride_bytes", "plane_layouts", "header")}
        payload = {k: v for k, v in payload.items() if v is not None}
    if not payload:
        raise ToolError(ErrorCode.NOT_READY, f"{kind}: adapter 返回空 payload（传感器未启用或没数据）")

    out: dict[str, typing.Any] = dict(payload)
    out["_timestamp_ns"] = _header_timestamp_ns(payload)
    out["_frame_id"] = _header_frame_id(payload)

    if kind in ("rgb", "depth"):
        fmt = out.get("format", out.get("output_format"))
        if not isinstance(fmt, str):
            # RgbOutputFormat.JPEG 之类的枚举
            fmt = str(getattr(fmt, "name", fmt) or "")
        out["format"] = fmt.lower() or "jpeg"
        for key in ("width", "height"):
            try:
                out[key] = int(out.get(key) or 0)
            except (TypeError, ValueError) as exc:
                raise ToolError(ErrorCode.SDK_ERROR, f"{kind}: {key} 非法 ({out.get(key)!r})") from exc
        if not isinstance(out.get("data"), (bytes, bytearray, memoryview)):
            raise ToolError(
                ErrorCode.SDK_ERROR,
                f"{kind}: data 必须是 bytes，收到 {type(out.get('data')).__name__}",
            )
        out["data"] = bytes(out["data"])
        strides = out.get("stride_bytes") or []
        out["stride_bytes"] = [int(s) for s in strides if isinstance(s, (int, float))]

    if kind == "extrinsic":
        # SDK 返回 (list[7], timestamp) 元组；归一化成 dict
        if isinstance(payload, (tuple, list)) and len(payload) == 2 and isinstance(payload[0], (list, tuple)):
            transform, ts = payload
            out = {"transform": list(transform), "timestamp_ns": int(ts or 0),
                   "reference_frame": "base_link", "_timestamp_ns": int(ts or 0), "_frame_id": ""}
        elif "transform" in out:
            out["transform"] = list(out["transform"])
        elif "data" in out and isinstance(out["data"], (list, tuple)):
            out["transform"] = list(out["data"])

    return out


# ═══════════════════════════════════════════════════════════════════════════ #
# 4. 解码器（Mock 与真机共用同一段代码 ★ 这是整个文件最重要的设计）
# ═══════════════════════════════════════════════════════════════════════════ #
def _unpack_u8_planes(
    data: bytes, height: int, width: int, channels: int, stride: int = 0
) -> np.ndarray:
    """把 tightly-packed / 带 stride 的 8bit 多通道 payload 变成 (H, W, C)。"""
    row_bytes = width * channels
    stride = stride or row_bytes
    if stride < row_bytes:
        raise ToolError(ErrorCode.SDK_ERROR, f"stride({stride}) < 行字节数({row_bytes})，payload 非法")
    need = stride * height
    buf = np.frombuffer(data, np.uint8)
    if buf.size < need:
        raise ToolError(
            ErrorCode.SDK_ERROR,
            f"payload 太短：期望 ≥{need} 字节，实际 {buf.size}（{width}x{height}x{channels}, stride={stride}）",
        )
    rows = buf[:need].reshape(height, stride)[:, :row_bytes]
    if channels == 1:
        return rows.reshape(height, width)
    return rows.reshape(height, width, channels)


def _nv12_to_bgr(data: bytes, height: int, width: int, strides: list[int]) -> np.ndarray:
    """NV12 (YUV420SP) -> BGR。SDK 的 stride_bytes 按 plane 给。"""
    y_stride = strides[0] if strides else width
    uv_stride = strides[1] if len(strides) > 1 else width
    buf = np.frombuffer(data, np.uint8)
    y_need = y_stride * height
    uv_need = uv_stride * (height // 2)
    if buf.size < y_need + uv_need:
        raise ToolError(
            ErrorCode.SDK_ERROR,
            f"NV12 payload 太短：期望 ≥{y_need + uv_need}，实际 {buf.size}",
        )
    y_plane = buf[:y_need].reshape(height, y_stride)[:, :width]
    uv_plane = buf[y_need : y_need + uv_need].reshape(height // 2, uv_stride)[:, :width]
    yuv = np.empty((height + height // 2, width), np.uint8)
    yuv[:height] = y_plane
    yuv[height:] = uv_plane
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def decode_rgb_payload(payload: typing.Any) -> tuple[np.ndarray, dict[str, typing.Any]]:
    """RGB payload -> (BGR uint8 HxWx3, meta)。

    ⚠️ 官方示例把 "rgb8" 也丢进 cv2.imdecode（get_camera_data.py:35），
       只有真机跑过才知道对不对。这里按字节布局正确解，不抄那个可疑分支。
    """
    p = normalize_payload("rgb", payload)
    data, h, w, fmt = p["data"], int(p["height"]), int(p["width"]), p["format"]
    if h <= 0 or w <= 0:
        raise ToolError(ErrorCode.SDK_ERROR, f"RGB 尺寸非法: {w}x{h}")
    strides = p.get("stride_bytes") or []

    if fmt in ("jpeg", "jpg", "mjpg"):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ToolError(
                ErrorCode.SDK_ERROR,
                f"JPEG 解码失败（{len(data)} 字节）。若 adapter 用了 RgbOutputFormat.JPEG 之外的格式请同步改这里",
            )
    elif fmt in ("bgr8", "bgr", "8uc3"):
        img = _unpack_u8_planes(data, h, w, 3, strides[0] if strides else 0)
    elif fmt in ("rgb8", "rgb"):
        raw = _unpack_u8_planes(data, h, w, 3, strides[0] if strides else 0)
        img = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    elif fmt in ("nv12", "yuv420sp"):
        img = _nv12_to_bgr(data, h, w, strides)
    else:
        raise ToolError(
            ErrorCode.NOT_SUPPORTED,
            f"未支持的 RGB 格式 {fmt!r}；请让 adapter 请求 JPEG（RgbOutputFormat.JPEG）",
        )

    meta = {
        "format": fmt,
        "width": int(img.shape[1]),
        "height": int(img.shape[0]),
        "timestamp_ns": p["_timestamp_ns"],
        "frame_id": p["_frame_id"],
        "bytes": len(data),
    }
    return img, meta


def decode_depth_payload(payload: typing.Any) -> tuple[np.ndarray, dict[str, typing.Any]]:
    """Depth payload -> (float32 HxW 单位米, NaN=无效, meta)。

    官方口径（g1/__init__.pyi:2416-2420）：
        depth_m = pixel_value / depth_scale
        depth_scale = 1000 表示 1000 counts = 1.0 m
        解码后是 uint16（16UC1），不是 float32；0 表示无效
    """
    p = normalize_payload("depth", payload)
    data = p["data"]
    h, w = int(p["height"]), int(p["width"])
    if h <= 0 or w <= 0:
        raise ToolError(
            ErrorCode.SDK_ERROR,
            f"深度尺寸非法: {w}x{h}（format={p.get('format')!r}）。真机上多为此机位未装配深度",
        )
    scale = p.get("depth_scale")
    try:
        scale = float(scale)
    except (TypeError, ValueError) as exc:
        raise ToolError(ErrorCode.SDK_ERROR, f"depth_scale 缺失或非法: {p.get('depth_scale')!r}") from exc
    if scale <= 0:
        raise ToolError(ErrorCode.SDK_ERROR, f"depth_scale 必须 > 0，收到 {scale}")

    raw = _decode_depth_bytes(data, h, w)
    depth_m = raw.astype(np.float32) / scale
    depth_m[raw == 0] = np.nan  # 0 = 无效/超量程

    valid = np.isfinite(depth_m)
    meta = {
        "format": p["format"],
        "width": int(depth_m.shape[1]),
        "height": int(depth_m.shape[0]),
        "depth_scale": scale,
        "unit": "m",
        "timestamp_ns": p["_timestamp_ns"],
        "frame_id": p["_frame_id"],
        "valid_ratio": float(valid.mean()) if valid.size else 0.0,
        "min_m": float(np.nanmin(depth_m)) if valid.any() else None,
        "max_m": float(np.nanmax(depth_m)) if valid.any() else None,
    }
    return depth_m, meta


def _decode_depth_bytes(data: bytes, height: int, width: int) -> np.ndarray:
    """深度字节 -> uint16 (H, W)。

    两条真实存在的路径都覆盖：
      A) 裸 16UC1：len(data) == h*w*2                     -> frombuffer 直读
      B) PNG/compressedDepth：len(data) != h*w*2           -> cv2.imdecode(IMREAD_UNCHANGED)
    `format` 字段可能长成 "16UC1; compressedDepth png"，
    所以**不能**用 format 字符串做唯一判据 —— 用它做提示，用长度做主判据最稳。
    """
    n = height * width
    if len(data) >= n * 2:
        arr = np.frombuffer(data, dtype=np.uint16, count=n)
        return arr.reshape(height, width).copy()

    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None and len(data) > 12:
        # 少数编码器在压缩深度前塞了 12 字节 cv::Mat 头
        img = cv2.imdecode(np.frombuffer(data[12:], np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ToolError(
            ErrorCode.SDK_ERROR,
            f"深度解码失败：{len(data)} 字节既不是裸 16UC1（期望 {n * 2}）也不是可解的 PNG",
        )
    if img.ndim == 3:
        img = img[..., 0]
    if img.dtype == np.uint16:
        decoded = img
    elif img.dtype == np.uint8:
        decoded = img.astype(np.uint16)  # 极少数 8bit 深度，单位会失真，需当场核对 depth_scale
    else:
        decoded = np.nan_to_num(img, nan=0.0).astype(np.uint16)

    if decoded.shape != (height, width):
        try:
            decoded = cv2.resize(decoded, (width, height), interpolation=cv2.INTER_NEAREST)
        except cv2.error as exc:
            raise ToolError(
                ErrorCode.SDK_ERROR,
                f"深度尺寸不一致：payload 声明 {width}x{height}，解码得到 {decoded.shape[1]}x{decoded.shape[0]}",
            ) from exc
    return decoded.astype(np.uint16, copy=False)


# ═══════════════════════════════════════════════════════════════════════════ #
# 5. 内参 / 反投影
# ═══════════════════════════════════════════════════════════════════════════ #
@dataclass(frozen=True)
class Intrinsics:
    """针孔内参。Python dict 用**大写** K/D（C++ 是小写，别搞混）。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: tuple[float, ...] = ()
    frame_id: str = ""
    timestamp_ns: int = 0

    @classmethod
    def from_payload(cls, payload: typing.Any, *, fallback_frame: str = "") -> "Intrinsics":
        p = normalize_payload("intrinsic", payload)
        k = p.get("K", p.get("k"))
        if k is None or len(k) < 9:
            raise ToolError(
                ErrorCode.NOT_READY,
                "内参缺失或缺 K[9]；get_camera_intrinsic 返回空 dict 通常说明相机未 enable",
            )
        d = p.get("D", p.get("d")) or []
        try:
            fx, cx = float(k[0]), float(k[2])
            fy, cy = float(k[4]), float(k[5])
        except (TypeError, ValueError, IndexError) as exc:
            raise ToolError(ErrorCode.SDK_ERROR, f"K 矩阵非法: {k!r}") from exc
        if fx <= 0 or fy <= 0:
            raise ToolError(ErrorCode.SDK_ERROR, f"焦距必须 > 0，收到 fx={fx}, fy={fy}")
        return cls(
            fx=fx, fy=fy, cx=cx, cy=cy,
            width=int(p.get("width") or 0),
            height=int(p.get("height") or 0),
            distortion=tuple(float(v) for v in d),
            frame_id=str(p.get("_frame_id") or fallback_frame),
            timestamp_ns=int(p.get("_timestamp_ns") or 0),
        )

    def to_dict(self) -> dict[str, typing.Any]:
        return {
            "fx": round(self.fx, 4), "fy": round(self.fy, 4),
            "cx": round(self.cx, 4), "cy": round(self.cy, 4),
            "width": self.width, "height": self.height,
            "frame_id": self.frame_id,
            "distortion_model": "plumb_bob" if self.distortion else "none",
        }


def pixel_to_camera_point(
    u: float,
    v: float,
    depth_m: np.ndarray,
    intr: Intrinsics,
    *,
    radius: int = 2,
    undistort: bool = True,
) -> dict[str, typing.Any] | None:
    """像素 + 深度 -> 相机坐标系 3D 点（米）。

    工程细节（初版没做，真机上会抖）：
      * 单像素取深度噪声极大 -> 用 (2r+1)² 窗口的**中位数**；
      * 深度与内参必须来自**同一相机**，否则 cx/cy 不对应；
      * D 非零时先 undistortPoints，否则边缘误差可达几厘米。
    """
    h, w = depth_m.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        raise ToolError(
            ErrorCode.INVALID_ARGUMENT,
            f"像素 ({ui},{vi}) 超出图像范围 {w}x{h}",
        )

    r = max(0, int(radius))
    window = depth_m[max(0, vi - r) : vi + r + 1, max(0, ui - r) : ui + r + 1]
    finite = window[np.isfinite(window) & (window > 0)]
    if finite.size == 0:
        return None
    z = float(np.median(finite))

    # 归一化坐标 -> 去畸变 -> 像素坐标
    if undistort and any(abs(float(c)) > 1e-9 for c in intr.distortion):
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], np.float64)
        d = np.array(intr.distortion, np.float64)
        und = cv2.undistortPoints(pts, k, d, P=k).reshape(-1, 2)
        uu, vv = float(und[0][0]), float(und[0][1])
    else:
        uu, vv = float(u), float(v)

    x = (uu - intr.cx) * z / intr.fx
    y = (vv - intr.cy) * z / intr.fy
    return {
        "u": ui, "v": vi,
        "x": round(x, 6), "y": round(y, 6), "z": round(z, 6),
        "frame_id": intr.frame_id or "camera_optical_frame",
        "depth_samples": int(finite.size),
        "depth_spread_m": round(float(np.ptp(finite)), 6),
    }


# ═══════════════════════════════════════════════════════════════════════════ #
# 6. PerceptionTool
# ═══════════════════════════════════════════════════════════════════════════ #
@dataclass
class _Snapshot:
    """一次抓帧的完整产物：内存里的帧 + 给模型的登记信息。

    存在的意义：让"落盘归档"和"分析"解耦。分析只依赖内存里的数组，
    因此 `--no-save-frames` 也能正常工作，真机上也不必每次提问都走一遍磁盘。
    """

    spec: CameraSpec
    rec: typing.Any
    bgr: np.ndarray | None
    depth: np.ndarray | None
    intr: "Intrinsics | None"
    data: dict[str, typing.Any]
    warnings: list[str]


def _tool(method: typing.Callable[..., ToolResult]) -> typing.Callable[..., ToolResult]:
    """把 ToolError / 意外异常统一翻译成 ToolResult。绝不把裸异常丢给 Agent。"""

    @functools.wraps(method)
    def wrapper(self: "PerceptionTool", *args: typing.Any, **kwargs: typing.Any) -> ToolResult:
        try:
            return method(self, *args, **kwargs)
        except ToolError as exc:
            return exc.to_result(self.mode)
        except cv2.error as exc:
            return failed(ErrorCode.SDK_ERROR, f"OpenCV 错误：{exc}", self.mode)
        except Exception as exc:  # noqa: BLE001 - 工具边界必须兜住
            return failed(
                ErrorCode.SDK_ERROR, f"{type(exc).__name__}: {exc}", self.mode,
                data={"tool": method.__name__},
            )

    return wrapper


class PerceptionTool:
    """B 的感知工具集。所有 public 方法返回 ToolResult（调用方 .to_dict()）。"""

    #: 工具注册表：名字 -> 方法。agent 用它生成 function-calling schema
    TOOL_NAMES = (
        "capture_scene",
        "analyze_scene",
        "find_object",
        "measure_distance",
        "list_cameras",
        "inspect_table",
    )

    def __init__(
        self,
        adapter: CameraAdapter,
        *,
        registry: CameraRegistry | None = None,
        config_path: str | os.PathLike[str] = DEFAULT_CONFIG_PATH,
        frame_store: FrameStore | None = None,
        save_frames: bool = True,
        analysis_options: dict[str, typing.Any] | None = None,
        table_calib_path: str | os.PathLike[str] | None = None,
        table_options: dict[str, typing.Any] | None = None,
    ) -> None:
        self.adapter = adapter
        self.registry = registry or CameraRegistry.from_file(config_path)
        self.save_frames = save_frames
        self.frames = frame_store or FrameStore(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs", "frames")
        )
        self.analysis = dict(analysis_options or {})
        self.table_calib_path = os.fspath(table_calib_path) if table_calib_path else None
        self.table_options = dict(table_options or {})
        self.mode = self._infer_mode(adapter)

    @staticmethod
    def _infer_mode(adapter: typing.Any) -> RunMode:
        raw = getattr(adapter, "mode", None)
        try:
            return RunMode(raw)
        except ValueError as exc:
            raise ValueError(
                f"adapter.mode 必须是 'mock' 或 'real'，收到 {raw!r}。"
                f"协议禁止静默回退，Mock 不得冒充真机"
            ) from exc

    # ──────────────────────────────────────────────────────────────── #
    # 6.1 底层取图（给 algorithms/ 和测试用，不直接暴露给大模型）
    # ──────────────────────────────────────────────────────────────── #
    def _call_adapter(self, method: str, *args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        fn = getattr(self.adapter, method, None)
        if fn is None:
            raise ToolError(
                ErrorCode.NOT_SUPPORTED,
                f"adapter 未实现 {method}()；请让 A 在 adapters/galbot_adapter.py 中补齐",
            )
        try:
            return fn(*args, **kwargs)
        except ToolError:
            raise
        except Exception as exc:  # noqa: BLE001 - SDK 侧异常统一标记
            raise ToolError(ErrorCode.SDK_ERROR, f"adapter.{method}() 抛错: {exc}") from exc

    def rgb_bgr(self, camera_id: typing.Any) -> tuple[np.ndarray, dict[str, typing.Any]]:
        spec = self.registry.resolve(camera_id)
        if not spec.has_rgb:
            raise ToolError(
                ErrorCode.NOT_SUPPORTED,
                f"{spec.camera_id} 只有深度没有彩色；可用的 RGB 机位："
                f"{[c for c in self.registry.camera_ids if self.registry.resolve(c).has_rgb]}",
            )
        raw = self._call_adapter("get_rgb_data", spec.sensor)
        return decode_rgb_payload(raw)

    def depth_m(self, camera_id: typing.Any) -> tuple[np.ndarray, dict[str, typing.Any]]:
        spec = self.registry.resolve(camera_id)
        if not spec.has_depth:
            raise ToolError(
                ErrorCode.NOT_SUPPORTED,
                f"{spec.camera_id} 没有深度相机；可用的深度机位："
                f"{[c for c in self.registry.camera_ids if self.registry.resolve(c).has_depth]}",
            )
        raw = self._call_adapter("get_depth_data", spec.depth_sensor)
        return decode_depth_payload(raw)

    def intrinsics(self, camera_id: typing.Any) -> Intrinsics:
        spec = self.registry.resolve(camera_id)
        sensor = spec.sensor if spec.has_rgb else (spec.depth_sensor or spec.sensor)
        raw = self._call_adapter("get_camera_intrinsic", sensor)
        return Intrinsics.from_payload(raw, fallback_frame=f"{sensor.lower()}_optical_frame")

    def extrinsic(self, camera_id: typing.Any, reference_frame: str = "base_link") -> dict[str, typing.Any] | None:
        """外参 [x,y,z,qx,qy,qz,qw]。失败返回 None（调用方必须显式处理，不许假装有）。"""
        spec = self.registry.resolve(camera_id)
        sensor = spec.sensor if spec.has_rgb else (spec.depth_sensor or spec.sensor)
        fn = getattr(self.adapter, "get_sensor_extrinsic", None)
        if fn is None:
            return None
        try:
            payload = fn(sensor, reference_frame)
        except Exception:  # noqa: BLE001 - 外参缺失不应让整个工具崩
            return None
        try:
            norm = normalize_payload("extrinsic", payload)
        except ToolError:
            return None
        tf = norm.get("transform") or []
        if len(tf) < 7:
            return None
        return {
            "reference_frame": reference_frame,
            "transform": [float(v) for v in tf[:7]],
            "timestamp_ns": int(norm.get("timestamp_ns") or 0),
        }

    def camera_point_to_base(self, point: dict[str, typing.Any], extrinsic: dict[str, typing.Any]) -> list[float]:
        """相机系 3D 点 -> base_link。外参是 base<-camera 的变换。"""
        from sdk_wrapper.dev.algorithms import geometry as geo  # 局部导入，避免循环依赖

        p_cam = np.array([point["x"], point["y"], point["z"]], np.float64)
        return [float(v) for v in geo.transform_point(p_cam, extrinsic["transform"])]

    # ──────────────────────────────────────────────────────────────── #
    # 6.2 抓帧内核：内存帧 + 登记信息一次拿全
    # ──────────────────────────────────────────────────────────────── #
    def _grab(
        self,
        camera_id: typing.Any = None,
        *,
        include_depth: bool = True,
        note: str = "",
    ) -> _Snapshot:
        """抓一帧，返回内存中的 bgr/depth 与登记信息。

        ⚠️ 帧**留在内存里**，`analyze_scene` / `find_object` 直接复用，不再从磁盘回读。
        这样：① `--no-save-frames` 也能正常分析（以前会返回"没有可分析的图像数据"）；
              ② 真机上省掉每次提问"写 1.3 MB PNG → 读回 → 解码"的往返。
        落盘只是归档，不是分析的前提。
        """
        spec = self.registry.resolve(camera_id)
        rec = self.frames.new_capture(spec.camera_id)
        data: dict[str, typing.Any] = {
            "capture_id": rec.capture_id,
            "camera_id": spec.camera_id,
            "sensor": spec.sensor,
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "rgb_ref": None,
            "depth_ref": None,
            "depth_unit": "m",
            "coordinate_frame": None,
            "width": None,
            "height": None,
            "depth_available": False,
            "depth_alignment": spec.depth_alignment,
        }
        warnings: list[str] = []
        bgr: np.ndarray | None = None
        depth: np.ndarray | None = None
        intr: Intrinsics | None = None

        if spec.has_rgb:
            bgr, rgb_meta = self.rgb_bgr(spec.camera_id)
            data["width"], data["height"] = rgb_meta["width"], rgb_meta["height"]
            data["rgb_format"] = rgb_meta["format"]
            data["rgb_timestamp_ns"] = rgb_meta["timestamp_ns"]
            if self.save_frames:
                self.frames.attach_rgb_png(rec, bgr)
                data["rgb_ref"] = f"{rec.capture_id}:rgb"

        try:
            intr = self.intrinsics(spec.camera_id)
            data["intrinsics"] = intr.to_dict()
            data["coordinate_frame"] = intr.frame_id or None
        except ToolError as exc:
            warnings.append(f"内参不可用({exc.code.value}): {exc.message}")

        if include_depth and spec.has_depth:
            try:
                depth, depth_meta = self.depth_m(spec.camera_id)
                data["depth_available"] = True
                data["depth_scale"] = depth_meta["depth_scale"]
                data["depth_valid_ratio"] = round(depth_meta["valid_ratio"], 4)
                data["depth_range_m"] = [depth_meta["min_m"], depth_meta["max_m"]]
                data["depth_timestamp_ns"] = depth_meta["timestamp_ns"]
                if data["width"] and (
                    depth_meta["width"] != data["width"] or depth_meta["height"] != data["height"]
                ):
                    warnings.append(
                        f"深度 {depth_meta['width']}x{depth_meta['height']} 与彩色 "
                        f"{data['width']}x{data['height']} 尺寸不一致，跨相机融合会错位"
                    )
                if self.save_frames:
                    self.frames.attach_depth_npy(rec, depth)
                    data["depth_ref"] = f"{rec.capture_id}:depth"
            except ToolError as exc:
                # 深度拿不到是"部分失败"，不是整体失败：彩色仍然有效
                warnings.append(f"深度不可用({exc.code.value}): {exc.message}")
        elif include_depth and not spec.has_depth:
            warnings.append(f"{spec.camera_id} 无深度相机（kind={spec.kind}）")

        if bgr is None and depth is None:
            raise ToolError(
                ErrorCode.NOT_READY,
                f"{spec.camera_id} 既没有彩色也没有深度可用",
                {"camera_id": spec.camera_id, "warnings": warnings},
            )

        if data["coordinate_frame"] is None:
            data["coordinate_frame"] = f"unverified:{spec.sensor.lower()}_optical_frame"
            warnings.append("内参里没有 frame_id，坐标系名未经验证，只作占位")

        data["warnings"] = warnings
        if note:
            data["note"] = note
        self.frames.update_meta(rec, data, persist=self.save_frames)
        return _Snapshot(
            spec=spec, rec=rec, bgr=bgr, depth=depth, intr=intr, data=data, warnings=warnings
        )

    @_tool
    def capture_scene(
        self,
        camera_id: str | None = None,
        include_depth: bool = True,
        note: str = "",
    ) -> ToolResult:
        """抓一帧并登记到帧缓存，返回引用 ID（不是像素本身）。"""
        snap = self._grab(camera_id, include_depth=include_depth, note=note)
        message = f"{snap.spec.camera_id} 抓帧完成（{self.mode.value}）"
        if snap.warnings:
            message += f"，{len(snap.warnings)} 条警告"
        return ok(message, self.mode, data=snap.data)

    # ──────────────────────────────────────────────────────────────── #
    # 6.3 工具 2：analyze_scene（一次拿到"看得懂"的结构化场景）
    # ──────────────────────────────────────────────────────────────── #
    @_tool
    def analyze_scene(
        self,
        camera_id: str | None = None,
        want_labels: list[str] | None = None,
        max_objects: int = 8,
    ) -> ToolResult:
        """抓帧 + 检测 + 反投影，返回紧凑的 JSON 场景描述（不含大数组）。"""
        from sdk_wrapper.dev.algorithms import scene_analyzer as scene

        snap = self._grab(camera_id)
        data = snap.data
        result = scene.analyze(
            bgr=snap.bgr,
            depth_m=snap.depth,
            intrinsics=snap.intr,
            want_labels=want_labels,
            max_objects=int(max_objects),
            options=self.analysis,
        )
        result["capture_id"] = data["capture_id"]
        result["rgb_ref"] = data.get("rgb_ref")
        result["depth_ref"] = data.get("depth_ref")
        result["camera_id"] = data["camera_id"]
        result["coordinate_frame"] = data.get("coordinate_frame")
        result["warnings"] = list(data.get("warnings") or []) + list(result.get("warnings") or [])
        return ok(
            f"{data['camera_id']}: {result.get('summary', '')}",
            self.mode,
            data=result,
        )

    # ──────────────────────────────────────────────────────────────── #
    # 6.4 工具 3：find_object（说"红色方块在哪"）
    # ──────────────────────────────────────────────────────────────── #
    @_tool
    def find_object(
        self,
        label: str,
        camera_id: str | None = None,
        max_distance_m: float = 1.5,
        with_base_link: bool = True,
    ) -> ToolResult:
        """按颜色/前景找到目标，给出图像坐标 + 相机系 3D + （可选）base_link 3D。"""
        from sdk_wrapper.dev.algorithms import scene_analyzer as scene

        if not isinstance(label, str) or not label.strip():
            raise ToolError(ErrorCode.INVALID_ARGUMENT, "label 必须是非空字符串，如 'red' / 'foreground'")
        if not scene.is_known_label(label):
            raise ToolError(
                ErrorCode.INVALID_ARGUMENT,
                f"不认识的 label {label!r}",
                {"legal_labels": scene.legal_labels()},
            )
        if float(max_distance_m) <= 0:
            raise ToolError(
                ErrorCode.INVALID_ARGUMENT, f"max_distance_m 必须 > 0，收到 {max_distance_m}"
            )

        snap = self._grab(camera_id)
        data = snap.data
        if snap.bgr is None:
            raise ToolError(
                ErrorCode.NOT_SUPPORTED,
                f"{data['camera_id']} 没有彩色图，无法做颜色检索",
            )

        found = scene.find_one(
            bgr=snap.bgr, depth_m=snap.depth, intrinsics=snap.intr,
            label=label, max_distance_m=float(max_distance_m), options=self.analysis,
        )
        if found is None:
            return failed(
                ErrorCode.NOT_READY,
                f"{data['camera_id']} 视野内没找到 {label!r}（{data.get('width')}x{data.get('height')}）",
                self.mode,
                data={"capture_id": data["capture_id"], "rgb_ref": data.get("rgb_ref"),
                      "depth_ref": data.get("depth_ref"), "label": label},
            )

        payload: dict[str, typing.Any] = {
            "capture_id": data["capture_id"],
            "rgb_ref": data.get("rgb_ref"),
            "depth_ref": data.get("depth_ref"),
            "camera_id": data["camera_id"],
            "coordinate_frame": data.get("coordinate_frame"),
            "target": found,
            "warnings": list(data.get("warnings") or []),
        }
        if with_base_link and found.get("center_cam_m") and found.get("coordinate_frame"):
            ext = self.extrinsic(data["camera_id"])
            if ext is None:
                payload["base_link_pose_m"] = None
                payload["warnings"].append("外参不可用，无法给出 base_link 坐标")
            else:
                pt = {"x": found["center_cam_m"][0], "y": found["center_cam_m"][1], "z": found["center_cam_m"][2]}
                payload["base_link_pose_m"] = self.camera_point_to_base(pt, ext)
                payload["base_link_frame"] = ext["reference_frame"]

        return ok(
            f"找到 {label}: 像素({found['centroid_uv'][0]}, {found['centroid_uv'][1]}) "
            f"距离 {found.get('depth_m')} m",
            self.mode,
            data=payload,
        )

    # ──────────────────────────────────────────────────────────────── #
    # 6.5 工具 4：measure_distance（"那个东西离我多远"）
    # ──────────────────────────────────────────────────────────────── #
    @_tool
    def measure_distance(
        self, u: int, v: int, camera_id: str | None = None, radius: int = 2
    ) -> ToolResult:
        spec = self.registry.resolve(camera_id)
        if not spec.has_depth:
            raise ToolError(
                ErrorCode.NOT_SUPPORTED,
                f"{spec.camera_id} 无深度相机；深度机位："
                f"{[c for c in self.registry.camera_ids if self.registry.resolve(c).has_depth]}",
            )
        depth, meta = self.depth_m(spec.camera_id)
        intr = None
        try:
            intr = self.intrinsics(camera_id)
        except ToolError:
            pass
        point = pixel_to_camera_point(u, v, depth, intr, radius=radius) if intr else None
        if point is None:
            return failed(
                ErrorCode.NOT_READY,
                f"像素 ({u},{v}) 附近没有有效深度（可能超出量程或为镜面/黑体）",
                self.mode,
                data={"camera_id": spec.camera_id, "u": int(u), "v": int(v)},
            )
        return ok(
            f"({u},{v}) 距离约 {point['z']:.3f} m",
            self.mode,
            data={"camera_id": spec.camera_id, "distance_m": point, "depth_meta": {
                "depth_scale": meta["depth_scale"], "valid_ratio": round(meta["valid_ratio"], 4)}},
        )

    # ──────────────────────────────────────────────────────────────── #
    # 6.6 工具 6：inspect_table（"桌上哪个杯子不安全、往哪推多远"）
    # ──────────────────────────────────────────────────────────────── #
    @_tool
    def inspect_table(self, camera_id: str | None = None) -> ToolResult:
        """判桌面：每个杯子到桌沿的距离、C1/C2、以及往哪推多远。

        依赖 `config/table_calib.json`（赛前标定）。同一个单应只在**标定时的末端位姿**
        下有效，所以本工具会先核对末端位姿，偏离就拒绝——不猜、不硬算。
        """
        from sdk_wrapper.dev.algorithms import table_scene as table_scene

        calib_path = self.table_calib_path or table_scene.DEFAULT_CALIB_PATH
        calib = table_scene.load_calibration(calib_path)
        if calib is None:
            raise ToolError(
                ErrorCode.NOT_READY,
                f"桌面标定缺失或格式不对：{calib_path}；请先按 docs/cup_scenario.md 做一次标定",
            )

        chain = str(calib.get("chain") or "left_arm")
        observe = calib.get("observe_pose")
        tol_m = float(calib.get("observe_pose_tol_m", 0.03))
        if observe and len(observe) >= 3:
            pose_result = self._call_adapter("get_end_effector_pose", chain)
            if getattr(pose_result, "status", None) is None:
                raise ToolError(
                    ErrorCode.NOT_READY,
                    f"读末端位姿返回了 {type(pose_result).__name__}，应返回 ToolResult",
                )
            if pose_result.status is not TaskStatus.SUCCEEDED:
                raise ToolError(
                    ErrorCode.NOT_READY,
                    f"读不到末端位姿，无法确认相机在观察位姿：{pose_result.message}",
                )
            current = pose_result.data.get("pose") or []
            if len(current) >= 3:
                deviation = math.dist(
                    [float(v) for v in current[:3]], [float(v) for v in observe[:3]]
                )
                if deviation > tol_m:
                    raise ToolError(
                        ErrorCode.NOT_READY,
                        f"末端偏离标定观察位姿 {deviation * 100:.1f} cm（上限 {tol_m * 100:.0f} cm）。"
                        f"单应矩阵只在固定位姿下成立，请先让 {chain} 回到观察位姿再测",
                        data={"chain": chain, "deviation_m": round(deviation, 4)},
                    )

        snap = self._grab(camera_id or calib.get("camera_id") or None)
        result = table_scene.analyze_table(snap.bgr, snap.depth, calib, self.table_options)
        result["capture_id"] = snap.data["capture_id"]
        result["rgb_ref"] = snap.data.get("rgb_ref")
        result["depth_ref"] = snap.data.get("depth_ref")
        result["camera_id"] = snap.data["camera_id"]
        result["warnings"] = list(snap.warnings) + list(result.get("warnings") or [])
        # 把标定时的观察位姿一并交出去：单应只在固定位姿下成立，
        # 场景编排"推完 → 回位 → 复测"需要它。
        result["observe_pose"] = [float(v) for v in observe] if observe else None
        result["observe_pose_tol_m"] = tol_m
        result["chain"] = chain

        cups = result.get("cups") or []
        if not cups:
            return failed(
                ErrorCode.NOT_READY,
                f"{snap.data['camera_id']}: 桌面上没检测到杯子/物体"
                f"（方法={result.get('method')}）",
                self.mode,
                data=result,
            )
        return ok(table_scene.describe_for_llm(result), self.mode, data=result)

    @_tool
    def list_cameras(self) -> ToolResult:
        return ok(
            f"{len(self.registry.camera_ids)} 个已登记机位",
            self.mode,
            data={
                "default_camera": self.registry.default,
                "cameras": self.registry.describe_all(),
                "note": "登记 ≠ 本机已装配；未装配机位调用会返回 not_ready",
            },
        )
