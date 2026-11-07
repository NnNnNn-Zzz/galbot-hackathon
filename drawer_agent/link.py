"""共享 Agent 适配层要拿到的能力清单，以及链接口的契约定义。

`REQUIRED` 是唯一数据源：能力探测、预检事实、交接文档三处都读它，
避免文档写一套、代码查另一套。优先级含义：
  P0 —— 缺了流程根本不能起步，预检直接拒。
  P1 —— 缺了能走到某一段，但该段及之后必须停，不能宣称完成。
"""

from __future__ import annotations

import math
import time
from typing import Any, Protocol

REQUIRED: dict[str, tuple[str, str]] = {
    "rig_status": ("P0", "腿部/头部锁定、底盘驻车、控制器归属、停止通道、相机标定 id"),
    "arm_state": ("P0", "右臂 TCP(base_link)、7 关节、夹爪宽度与电流、采样时间戳"),
    "move_tcp": ("P0", "base_link 绝对位姿直线运动，整段 IK 与碰撞检查"),
    "set_gripper": ("P0", "宽度+速度+力控，返回实测宽度与电流"),
    "base_pose": ("P0", "里程计位姿 x/y/yaw，用于转角核验与返航漂移"),
    "set_base_pose_rel": ("P0", "相对位姿底盘运动，禁止内部自动重试"),
    "stop": ("P0", "独立于命令队列的立即停止（底盘与右臂分别可停）"),
    "capture": ("P0", "固定头部相机取图，带帧号/标定 id/单调时钟时间戳"),
    "drawer_state": ("P0", "抽屉实际开度与不确定度，独立测量"),
    "hold_state": ("P0", "是否夹住物体的物理证据，不能只看闭合指令成功"),
    "bin_state": ("P1", "盒是否入桶、距桶心距离"),
    "move_joints": ("P1", "关节空间运动，示教回放兜底"),
}

#: 只靠 P0 就能跑完整流程，P1 缺失时对应段落必须停止而不是估算。
REQUIRED_P0 = tuple(name for name, (prio, _) in REQUIRED.items() if prio == "P0")

#: 适配器能力 → sdk_wrapper 侧应提供的方法名。给负责人的对照表，避免只报
#: "缺 arm_state" 却不说要写哪个函数。
NEEDED_HINT: dict[str, str] = {
    "rig_status": "get_rig_status",
    "arm_state": "get_right_arm_state",
    "move_tcp": "set_arm_pose_base",
    "set_gripper": "set_gripper_command",
    "base_pose": "get_odom",
    "set_base_pose_rel": "set_base_pose_rel",
    "stop": "stop_controller",
    "capture": "capture_frame",
    "drawer_state": "get_drawer_state",
    "hold_state": "get_hold_state",
    "bin_state": "get_bin_state",
    "move_joints": "set_arm_joints",
}


def required_capabilities(*, rotate: bool, live: bool) -> tuple[str, ...]:
    names = tuple(name for name in REQUIRED_P0 if rotate or name != "set_base_pose_rel")
    return (*names, "bin_state") if live else names


class LinkError(RuntimeError):
    """链路调用失败；调用方必须当作未知结果处理，不得重试。"""


class MissingCapability(LinkError):
    """现场后端没有实现该能力。预检阶段抛出，运动前失败。"""

    def __init__(self, names):
        self.names = list(names)
        super().__init__("共享 Agent 缺少能力：" + ", ".join(self.names))


class RobotLink(Protocol):
    """适配器需要的全部硬件动作。两种实现：SharedAgentLink / OfflineRigLink。

    返回约定（所有方法）：
      accepted    bool   —— 命令被接收
      completed   bool   —— 返回时**物理已完成**（不是"已下发"）
      status      str    —— SDK 状态名原文，失败时原样透传，不翻译成布尔
      stamp_ns    int    —— 读数采样时刻，与 clock_ns() 同一时钟
    """

    kind: str

    def caps(self) -> dict[str, bool]: ...
    def clock_ns(self) -> int: ...
    def rig_status(self) -> dict: ...
    def base_pose(self) -> dict: ...
    def set_base_pose_rel(self, x_m: float, yaw_rad: float, timeout_s: float) -> dict: ...
    def arm_state(self) -> dict: ...
    def move_tcp(self, pose: list[float], timeout_s: float, *, straight: bool = True,
                 hold: str = "none", expected_joints_rad=None) -> dict: ...
    def move_joints(self, positions: list[float], timeout_s: float) -> dict: ...
    def set_gripper(self, width_m: float, speed_m_s: float, effort_n: float,
                    timeout_s: float) -> dict: ...
    def drawer_state(self) -> dict: ...
    def hold_state(self) -> dict: ...
    def bin_state(self) -> dict: ...
    def capture(self, camera_id: str, timeout_s: float) -> dict: ...
    def stop(self, chain: str) -> dict: ...


def finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in values)


def norm_quat(q: list[float]) -> list[float]:
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n <= 1e-9:
        raise LinkError("四元数模长为零")
    return [float(v) / n for v in q]


def fresh(stamp_ns: Any, now_ns: int, limit_s: float) -> bool:
    return (isinstance(stamp_ns, int) and not isinstance(stamp_ns, bool)
            and 0 <= now_ns - stamp_ns <= limit_s * 1e9)


def now_ns() -> int:
    """单调时钟。相机、里程计、夹爪的采样时间戳必须落在同一时钟域。

    用 monotonic 而不是系统墙钟：现场会有人调时区/对时，墙钟跳变会让
    "帧是否新鲜"判断失效，而单调时钟只受进程存活影响。
    """
    return time.monotonic_ns()
