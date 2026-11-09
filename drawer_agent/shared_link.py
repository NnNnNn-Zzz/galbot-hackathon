"""共享 Agent 链路：把 `sdk_wrapper` 暴露成适配器需要的 12 个方法。

这里不 import `galbot_sdk`，也不 init/destroy 任何单例 —— 只用
`sdk_wrapper.wrapper.get_robot()` / `get_nav()` 已初始化的共享实例，
与仓库既有约定一致（见 sdk_wrapper/wrapper.py 顶部说明）。

方法名是**契约**：`sdk_wrapper` 侧按 `NEEDED` 里的名字与返回字段实现，
适配器这边一行不用改。缺哪个就在 caps() 里为 False，预检会把名字打出来。
"""

from __future__ import annotations

import time
from .link import NEEDED_HINT, MissingCapability

#: sdk_wrapper 侧需要提供的方法 → 适配器用途。这份表同时是给负责人的清单。
NEEDED: dict[str, str] = {
    "get_rig_status": "腿部/头部锁定、底盘驻车、控制器归属、停止通道、标定 id",
    "get_odom": "里程计位姿 x/y/yaw（相对本次运行起点）",
    "set_base_pose_rel": "相对位姿底盘运动，retries=0，原样返回 SDK 状态名",
    "get_right_arm_state": "TCP(base_link)[7] + 7 关节 + 夹爪宽度/电流 + 采样时间戳",
    "set_arm_pose_base": "base_link 绝对位姿直线运动，开碰撞检查，阻塞到物理到位",
    "set_arm_joints": "关节空间运动（示教回放兜底）",
    "set_gripper_command": "右夹爪 宽度/速度/力，返回实测宽度与电流",
    "stop_controller": "按链路立即停止（'right_arm' / 'base'）",
    "capture_frame": "固定头部相机取图，返回路径/帧号/标定 id/单调时钟时间戳",
    "get_drawer_state": "抽屉开度与不确定度（独立测量）",
    "get_hold_state": "夹持物证：是否夹住、夹的是把手还是盒",
    "get_bin_state": "桶内是否有盒、距桶心距离",
}


def _load_wrapper():
    from sdk_wrapper import wrapper  # 延迟导入：离线环境不触发 SDK 引导
    return wrapper


class SharedAgentLink:
    kind = "shared_agent"

    def __init__(self, recipe: dict, *, wrapper=None):
        self.recipe = recipe
        self.wrapper = wrapper if wrapper is not None else _load_wrapper()

    # ------------------------------------------------------------------ 能力
    def caps(self) -> dict[str, bool]:
        return {capability: callable(getattr(self.wrapper, method, None))
                for capability, method in NEEDED_HINT.items()}

    def _fn(self, name: str):
        fn = getattr(self.wrapper, name, None)
        if not callable(fn):
            raise MissingCapability([name])
        return fn

    def clock_ns(self) -> int:
        return time.monotonic_ns()

    # ------------------------------------------------------------------ 读数
    def rig_status(self) -> dict:
        return dict(self._fn("get_rig_status")())

    def base_pose(self) -> dict:
        return dict(self._fn("get_odom")())

    def arm_state(self) -> dict:
        return dict(self._fn("get_right_arm_state")())

    def drawer_state(self) -> dict:
        return dict(self._fn("get_drawer_state")())

    def hold_state(self) -> dict:
        return dict(self._fn("get_hold_state")())

    def bin_state(self) -> dict:
        return dict(self._fn("get_bin_state")())

    def capture(self, camera_id: str, timeout_s: float) -> dict:
        return dict(self._fn("capture_frame")(camera_id, timeout_s))

    # ------------------------------------------------------------------ 动作
    def set_base_pose_rel(self, x_m: float, yaw_rad: float, timeout_s: float) -> dict:
        # retries 必须为 0：本层自己判断成败，不允许封装层偷偷重发。
        return dict(self._fn("set_base_pose_rel")(
            float(x_m), float(yaw_rad), timeout_s=float(timeout_s), retries=0,
            max_yaw_speed_rad_s=self.recipe["limits"]["max_base_yaw_speed_rad_s"]))

    def move_tcp(self, pose: list[float], timeout_s: float, *, straight: bool = True,
                 hold: str = "none", expected_joints_rad=None) -> dict:
        return dict(self._fn("set_arm_pose_base")(
            "right_arm", [float(v) for v in pose], "base_link",
            timeout_s=float(timeout_s), straight=bool(straight),
            collision_check=True, hold=hold,
            max_speed_m_s=self.recipe["limits"]["max_linear_speed_m_s"],
            max_joint_speed_rad_s=self.recipe["limits"]["max_joint_speed_rad_s"],
            max_contact_force_n=self.recipe["limits"]["max_contact_force_n"],
            expected_joints_rad=expected_joints_rad))

    def move_joints(self, positions: list[float], timeout_s: float) -> dict:
        return dict(self._fn("set_arm_joints")(
            "right_arm", [float(v) for v in positions], timeout_s=float(timeout_s)))

    def set_gripper(self, width_m: float, speed_m_s: float, effort_n: float,
                    timeout_s: float) -> dict:
        return dict(self._fn("set_gripper_command")(
            "right", float(width_m), float(speed_m_s), float(effort_n),
            timeout_s=float(timeout_s)))

    def stop(self, chain: str) -> dict:
        return dict(self._fn("stop_controller")(chain))
