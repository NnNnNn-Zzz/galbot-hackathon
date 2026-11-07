"""统一封装层（薄壳）：能力全部落在 ``sdk_wrapper.dev``，本模块只做三件事。

    ┌──────────────────────────────────────────────────────────────┐
    │ 1. 名字兼容：保留全部历史函数名与参数（老脚本一行不用改）        │
    │ 2. 安全限幅：按 safety_config.json 截断距离/角度/位移           │
    │ 3. 单位换算：对外厘米/度 ↔ dev 层米/弧度                        │
    └──────────────────────────────────────────────────────────────┘

════════════════════════════════════════════════════════════════════════════
为什么要改成薄壳（这是本文件第 N 版）
════════════════════════════════════════════════════════════════════════════
老版本自己 ``GalbotRobot()`` / ``GalbotNavigation()`` / ``GalbotMotion()`` 各 init 一套，
dev 层 ``GalbotAdapter`` 又 init 一套 —— 同进程**两套 SDK 单例互踩**（纪律④），
于是能力被割裂成"底盘在 wrapper、手臂/相机/感知在 dev"两块，接新能力时要在两边重复实现。

现在改成：

    agent / 场景脚本 / 老脚本
              │   本文件（薄壳：限幅 + 单位换算 + dict 返回）
              ▼
    sdk_wrapper.dev.toolhub（dev 层统一工具入口）
              ▼
    GalbotAdapter / MockAdapter   ← 全进程**唯一**的 SDK 单例持有者

⚠️ 老代码若 import 了本模块的私有单例（``_get_robot`` / ``get_nav``），
   仍可用，但它们现在返回的是 **adapter 内部的那一份**，不再是另外 new 的。
   新代码请直接调本模块的函数，或 ``sdk_wrapper.dev.toolhub``。
"""

from __future__ import annotations

import logging
import math
import os

from sdk_wrapper.config import load_safety

logger = logging.getLogger("sdk_wrapper")

#: 兜底开关：环境变量 GALBOT_MOCK=1 与 --mock 等效（保留旧行为）
_mock_mode = os.environ.get("GALBOT_MOCK", "0") == "1"

#: 夹爪默认运动速度（米/秒）与力档位。沿用 dev adapter 真机验证值（D5 通过）。
GRIPPER_DEFAULT_VELOCITY_MPS = 0.05
GRIPPER_DEFAULT_EFFORT = 10.0

#: 手臂单次平移限幅（米）。
ARM_MAX_STEP_M = 0.10

#: 夹爪物理开度上限（米）。dev 层 D5 真机验证 0~0.12m 均 SUCCESS，保守取 0.085m。
GRIPPER_MAX_WIDTH_M = 0.085


# ---------------------------------------------------------------------------
# 内部：拿到 dev 层统一入口
# ---------------------------------------------------------------------------
def _hub():
    """懒加载 dev 层工具入口。

    懒加载是必须的：``sdk_wrapper/__init__`` → wrapper → dev.toolhub → wrapper
    如果在模块顶层 import 会形成循环。
    """
    from sdk_wrapper.dev import toolhub

    return toolhub


def set_mock_mode(enabled: bool):
    """由 agent/main.py 在启动时根据 --mock 参数调用。"""
    global _mock_mode
    _mock_mode = bool(enabled)
    _hub().configure(mock=bool(enabled))


# ---------------------------------------------------------------------------
# 单例入口（兼容保留）
# ---------------------------------------------------------------------------
def get_robot():
    """公开入口：共享机器人单例。

    真机返回 ``GalbotRobot``（adapter 内部那一份，已 init）；
    mock 返回 ``MockAdapter``（它自带 stop_base / set_gripper 等方法）。

    ⚠️ 场景脚本**禁止**自己 init()/destroy()，必须从这里拿。
    """
    hub = _hub()
    adapter = hub.get_adapter()
    if hub._is_mock():
        return adapter
    try:
        robot, _motion, _nav = adapter._ensure_interfaces(need_nav=False)
        return robot
    except Exception as exc:  # noqa: BLE001 - 兼容路径，拿不到就退回 adapter
        logger.warning("取 GalbotRobot 单例失败，退回 adapter：%s", exc)
        return adapter


def _get_robot():
    """历史私有名字，等价 ``get_robot()``（baseline.py / 老脚本仍在用）。"""
    return get_robot()


class _MockNavigation:
    """mock 下的导航替身：老代码会调 ``nav.stop_navigation()``。"""

    def __init__(self) -> None:
        self.stopped = 0

    def stop_navigation(self):
        self.stopped += 1
        return True, "SUCCESS"

    def is_localized(self) -> bool:
        return True


_mock_nav = _MockNavigation()


def get_nav():
    """公开入口：共享导航单例（真机是 adapter 的 GalbotNavigation）。

    ⚠️ 直接调 nav 没有 wrapper 层的自动限幅，请自觉遵守 safety_config.json。
    """
    hub = _hub()
    adapter = hub.get_adapter()
    if hub._is_mock():
        return _mock_nav
    _robot, _motion, nav = adapter._ensure_interfaces(need_nav=True)
    return nav


def _get_nav():
    """历史私有名字，等价 ``get_nav()``。"""
    return get_nav()


def shutdown():
    """真机 SDK 清理（程序退出时必须调用）。

    注意：SDK destroy() 后同进程无法再 init()，重启 agent 请重新起进程。
    """
    try:
        _hub().shutdown()
    except Exception as exc:  # noqa: BLE001 - 清理失败也要让退出流程继续
        logger.error("SDK 清理异常: %s", exc)


# ---------------------------------------------------------------------------
# 内部：限幅
# ---------------------------------------------------------------------------
def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        logger.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# 底盘运动接口（直控 set_base_pose，不需要地图/定位）
# ---------------------------------------------------------------------------
def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米，负值即后退）。自动限幅，阻塞执行。

    Returns:
        {"success": bool, "actual_distance_cm": float, "message": str}
    """
    cfg = load_safety()
    distance_m = _clamp(distance_cm / 100.0, cfg["max_base_distance_m"], "distance")
    r = _hub().move_base(distance_m, 0.0)
    r["actual_distance_cm"] = round(distance_m * 100, 2)
    return r


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(distance_cm))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    cfg = load_safety()
    angle_deg = _clamp(angle_deg, cfg["max_turn_deg"], "turn_angle")
    r = _hub().move_base(0.0, math.radians(angle_deg))
    r["actual_angle_deg"] = round(angle_deg, 2)
    return r


def stop_base() -> dict:
    """立即停止底盘运动（场景退出前建议调用）。"""
    return _hub().stop_base()


def read_sensor(sensor_name: str):
    """读取传感器（相机名会直接转成"抓一帧"）。"""
    return _hub().read_sensor(sensor_name)


# ---------------------------------------------------------------------------
# 夹爪接口
# ---------------------------------------------------------------------------
def set_gripper(end_effector: str = "left_gripper", width_cm: float = 0.0,
                velocity_mps: float = GRIPPER_DEFAULT_VELOCITY_MPS,
                effort: float = GRIPPER_DEFAULT_EFFORT) -> dict:
    """设置夹爪开度。单位：厘米（0=完全闭合，约 8.5cm=最大张开）。

    ``velocity_mps`` 仅为兼容旧签名保留（dev 层用固定速度 0.05 m/s）。
    """
    if abs(velocity_mps - GRIPPER_DEFAULT_VELOCITY_MPS) > 1e-9:
        logger.debug("set_gripper 的 velocity_mps 参数已忽略（dev 层固定 0.05 m/s）")
    return _hub().set_gripper(end_effector, float(width_cm), effort=float(effort))


def open_gripper(end_effector: str = "left_gripper") -> dict:
    """夹爪完全张开。"""
    return _hub().open_gripper(end_effector)


def close_gripper(end_effector: str = "left_gripper",
                  effort: float = GRIPPER_DEFAULT_EFFORT) -> dict:
    """夹爪合拢（抓取 / 用侧面推）。"""
    return _hub().close_gripper(end_effector, effort=float(effort))


# ---------------------------------------------------------------------------
# 手臂运动接口
# ---------------------------------------------------------------------------
def get_ee_pose(chain: str = "right_arm") -> dict:
    """读末端位姿 [x,y,z,qx,qy,qz,qw]（base_link，米/四元数）。"""
    return _hub().get_ee_pose(chain)


def move_arm_to(pose, chain: str = "right_arm", timeout_s: float = 15.0) -> dict:
    """手臂到 base_link 绝对位姿。pose=[x,y,z,qx,qy,qz,qw]（米/四元数）。"""
    pose = [float(v) for v in pose]
    if len(pose) != 7:
        return {"success": False, "message": f"pose 需 7 元，收到 {len(pose)}"}
    return _hub().move_arm_to(pose, chain=chain, straight_line=False)


def move_arm_delta(dx: float, dy: float, dz: float, chain: str = "right_arm",
                   timeout_s: float = 15.0) -> dict:
    """末端平移 (dx,dy,dz) 米（base_link 系），姿态不变。单轴限幅 ARM_MAX_STEP_M。"""
    dx = _clamp(dx, ARM_MAX_STEP_M, "arm_dx")
    dy = _clamp(dy, ARM_MAX_STEP_M, "arm_dy")
    dz = _clamp(dz, ARM_MAX_STEP_M, "arm_dz")
    return _hub().move_arm_delta(dx, dy, dz, chain=chain, straight_line=False)


# ---------------------------------------------------------------------------
# 感知接口
# ---------------------------------------------------------------------------
def capture_right_wrist(save_path: str = "/tmp/grasp_rgb.jpg") -> dict:
    """抓右腕相机 RGB + 深度 + 内参 + 外参，供反投影。

    ⚠️ 深度与彩色是否同一光心未验证（cameras.json: unverified）。
       真机先比对 width/height，不一致则同一像素取深度会错位，需先做对齐。

    Returns:
        {"success": bool, "image_path": str, "depth_m": np.ndarray,
         "intrinsic": dict, "extrinsic": [7], "width": int, "height": int, ...}
        depth_m 是 numpy 数组（米），仅供场景脚本内部 deproject 用，不给 LLM。
    """
    return _hub().capture_right_wrist(save_path)


def capture_frame(camera_id: str = "right_arm",
                  save_path: str = "/tmp/frame.jpg") -> dict:
    """抓任意相机一帧（right_arm / left_arm）。"""
    return _hub().capture_frame(camera_id, save_path)


def find_object(label: str, camera_id: str | None = None,
                max_distance_m: float = 1.5) -> dict:
    """按颜色/前景找目标（label 见 dev.algorithms.scene_analyzer）。"""
    return _hub().find_object(label, camera_id, max_distance_m)


def inspect_table(camera_id: str | None = None) -> dict:
    """判桌面杯子到桌沿的距离（需 table_calib.json 标定）。"""
    return _hub().inspect_table(camera_id)


def list_cameras() -> dict:
    """列出可用相机与能力（有无 RGB/深度）。"""
    return _hub().list_cameras()


def push_cup() -> dict:
    """把不安全的杯子推离桌沿（dev 层 CupScenario 闭环）。"""
    return _hub().push_cup()
