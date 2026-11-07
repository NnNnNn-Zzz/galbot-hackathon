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

#: 沿夹爪方向执行时的建议高度（米），按参与的链分档。真机实测：纯手臂伸展时
#: 肘部下沉更多，z=0.834 会让前臂扫掠被判碰撞（第 4 步 FAULT），抬到 0.865 才
#: 走完全程；带躯干 0.85 就够。mock 与真机共用，避免两套阈值漂移。
ALONG_SAFE_Z: dict[str, float] = {"with_torso": 0.85, "with_chain_only": 0.865}

#: 沿夹爪方向单次调用的总行程上限（米）。太长则一次失败后难以回退。
ALONG_MAX_TRAVEL_M: float = 0.30

#: 通用最低高度（米）：不管哪种模式、往哪个方向，末端都不应该低于这个值。
#: 桌面实测约 0.709，留 5 cm 余量。后退时用它做门槛，避免为了"抬高"而多做一个
#: 手臂已伸展时容易 FAULT 的抬升动作。
ALONG_FLOOR_Z: float = 0.76


def quat_to_rot(q: typing.Sequence[float]) -> list[list[float]]:
    """四元数 ``(qx, qy, qz, qw)`` -> 3x3 旋转矩阵（行优先）。

    mock 与真机共用这一份实现：两边算出的"夹爪指向"必须完全一致，
    否则会出现"模拟通过、真机走歪"。列向量即该轴在参考系下的指向，
    所以 ``rot[*][0]``（第一列）就是夹爪的逼近方向 —— 真机 FK 实测
    EndEffector 的 +X 指向 TCP 相对法兰的偏移方向。
    """
    x, y, z, w = [float(v) for v in q]
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


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
    def get_map_pose(self) -> ToolResult:
        """Read the localized chassis pose in the map frame."""

    @abstractmethod
    def check_path_reachability(
        self,
        goal_pose: typing.Sequence[float],
        start_pose: typing.Sequence[float] | None = None,
    ) -> ToolResult:
        """Ask SLAM navigation whether a collision-free map path exists."""

    @abstractmethod
    def navigate_to_pose(
        self,
        pose: typing.Sequence[float],
        *,
        frame: str = "map",
        speed_ratio: float = 0.5,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        """Start one absolute SLAM navigation task."""

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
    def check_end_effector_reachable(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        """Read-only IK preflight for an absolute ``EndEffector`` target."""

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
    def move_along_gripper(
        self,
        chain: str,
        distance_m: float,
        *,
        actuate: str = "with_torso",
        auto_lift: bool = True,
        max_step_m: float = 0.05,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """沿**夹爪指向**直线走 ``distance_m`` 米（正=伸出逼近，负=后退），姿态不变。

        SDK 没有"沿工具轴运动"的原语：参考系只支持 base_link/map/world，
        ``is_relative_pose`` 也拿不到工具系。所以这里读当前位姿 → 取旋转矩阵
        第一列（EndEffector 的 +X，真机 FK 实测即 TCP 相对法兰的偏移方向）
        → 换成 base_link 下的位移 → 用 move_line 下发绝对目标。

        Args:
            actuate: ``"with_torso"``（默认，躯干协助，伸得更远）或
                ``"with_chain_only"``（纯手臂，躯干零借用）。
            auto_lift: 路径最低点低于该模式的建议高度时，先垂直抬升再走。
                **两档建议高度不同**（纯手臂肘部下沉更多，需要更高），
                共用同一个阈值会在中途莫名 FAULT。

        注意：夹爪指向通常带 z 分量（实测相对 +Z 倾角 95.3°），所以"前进"
        同时会下降；不加 auto_lift 直接走长距离很容易贴桌被碰撞检查拒绝。
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
