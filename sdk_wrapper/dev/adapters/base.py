"""Shared execution interface for mock and real robot adapters."""

from __future__ import annotations

import json
import os
from abc import ABC, abstractmethod

from sdk_wrapper.dev.contracts import RunMode, ToolResult

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")

#: 兜底白名单。真实白名单从 config/ 读（协议 §0：config/ 管合法目标点与动作 ID）
_FALLBACK_GESTURES = frozenset({"wave_right"})
_FALLBACK_PLACES = frozenset({"demo_table"})


def _load_registered_ids(filename: str, section: str, fallback: frozenset[str]) -> frozenset[str]:
    """从 config/*.json 读白名单；文件缺失/损坏时退回内置兜底值。"""
    path = os.path.join(_CONFIG_DIR, filename)
    if not os.path.exists(path):
        return fallback
    try:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
        entries = payload.get(section) or {}
        ids = frozenset(str(key) for key in entries) if isinstance(entries, dict) else frozenset()
        return ids or fallback
    except Exception:  # noqa: BLE001 - 配置坏了不应该让 import 失败
        return fallback


SUPPORTED_GESTURES: frozenset[str] = _load_registered_ids(
    "gestures.json", "gestures", _FALLBACK_GESTURES
)
SUPPORTED_PLACES: frozenset[str] = _load_registered_ids("places.json", "places", _FALLBACK_PLACES)


class RobotAdapter(ABC):
    """Contract for robot state, motion, navigation, and raw camera access.

    Implementations return the shared ``ToolResult`` contract. Any operation
    that starts a task must return its queryable ``task_id`` while running.
    Position, angle, and duration values in result data use meters, radians,
    and seconds. ``unknown`` is not evidence that motion has stopped.
    Successful state snapshots use ``data["motion_state"]`` with ``"idle"``
    or ``"moving"``; uncertain snapshots return status ``unknown``.

    相机部分是**原始 payload 透传**：实现只负责对接 SDK / 造合成数据；
    解码、反投影、深度单位换算一律由感知层（``robot_tools/perception.py``）做。
    这样 Mock 与真机走同一段解码代码，不会出现"模拟通过、真机炸"。
    """

    @property
    @abstractmethod
    def mode(self) -> RunMode:
        """Return the explicit execution mode; never fall back between modes."""

    # ---- 执行 ----
    @abstractmethod
    def get_robot_state(self) -> ToolResult:
        """Return a structured snapshot of robot state."""

    @abstractmethod
    def execute_gesture(self, gesture_id: str) -> ToolResult:
        """Start a registered gesture."""

    @abstractmethod
    def navigate_to_place(self, place_id: str) -> ToolResult:
        """Start navigation to a registered place."""

    @abstractmethod
    def get_task_status(self, task_id: str) -> ToolResult:
        """Query a previously started task."""

    @abstractmethod
    def cancel_task(self, task_id: str) -> ToolResult:
        """Request cancellation and report only a confirmed cancellation."""

    # ---- 相机（原始 payload，不解码）----
    @abstractmethod
    def get_rgb_data(self, sensor: str) -> object:
        """Return the SDK-shaped RGB payload; ``sensor`` is a SensorType member name."""

    @abstractmethod
    def get_depth_data(self, sensor: str) -> object:
        """Return the SDK-shaped depth payload for an arm depth camera."""

    @abstractmethod
    def get_camera_intrinsic(self, sensor: str) -> object:
        """Return CameraInfo-shaped intrinsics (uppercase ``K`` / ``D`` in the Python dict)."""

    @abstractmethod
    def get_sensor_extrinsic(self, sensor: str, reference_frame: str = "base_link") -> object:
        """Return ``([x, y, z, qx, qy, qz, qw], timestamp_ns)``."""

    # ---- 手臂运动原语（供"推/抓"这类操作组合）----
    # 三者都是**末端笛卡尔**级别，不接受关节角/速度/力矩，避免把底层控制暴露给大模型。
    @abstractmethod
    def get_end_effector_pose(self, chain: str = "left_arm") -> ToolResult:
        """读末端位姿。``data["pose"]`` 为 base_link 下 ``[x,y,z,qx,qy,qz,qw]``（米）。"""

    @abstractmethod
    def get_chain_frame_pose(
        self,
        chain: str = "right_arm",
        frame_id: str = "Camera",
        reference_frame: str = "base_link",
    ) -> ToolResult:
        """Read a semantic chain frame: ``EndEffector``, ``Camera`` or ``TCP``."""

    @abstractmethod
    def move_ee_delta(
        self,
        chain: str,
        dx: float,
        dy: float,
        dz: float,
        *,
        straight_line: bool = True,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """让末端在 base_link 下平移 ``(dx, dy, dz)`` 米，姿态保持不变。

        ``straight_line=True`` 走直线（推杯子必须直线，否则方向会飘）；
        ``False`` 走一般规划（可绕行避障）。成功只代表**到位**，不代表推到了东西。
        """

    @abstractmethod
    def move_ee_to(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        straight_line: bool = True,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """把末端移到 base_link 下的**绝对**位姿 ``[x,y,z,qx,qy,qz,qw]``。

        用于"测完推走了、回到观察位姿再复测"这类回位动作。
        """

    @abstractmethod
    def set_gripper_width(
        self,
        end_effector: str = "left_gripper",
        width_m: float = 0.0,
        *,
        velocity_mps: float = 0.05,
        effort: float = 10.0,
        is_blocking: bool = True,
    ) -> ToolResult:
        """设置夹爪开度（米）。推杯子时闭到 0 用侧面推，抓取时闭到杯径。"""

    # ---- 语音：播放预生成的 16 kHz / 16-bit / mono PCM ----
    # SDK 没有 TTS，只能播 PCM 文件；所以文案必须在赛前合成好随仓库提交。
    @abstractmethod
    def write_audio_stream_output(self, audio_chunk: bytes, stream_id: str = "") -> bool:
        """写一块 PCM 到播放流。返回 True 只代表"写入并下发"，不代表已播完。"""

    @abstractmethod
    def set_volume(self, volume: float) -> bool:
        """设置系统音量（0–100）。"""
