"""SDK 封装层：提供带安全限幅的高层接口 + 共享单例入口。

安全策略：
- 高层接口（move_forward / turn / ...）按 safety_config.json 限幅（clamp），超限不报错、静默截断并记日志。
- galbot_sdk 全局单例由本模块管理；场景脚本**允许**自己组装 SDK 封装
  （import galbot_sdk），但实例必须从本模块的 get_robot()/get_nav() 拿，
  严禁自行 init()/destroy()（会打死主进程单例和急停监听）。
- 支持 MOCK 模式（无真机/无 SDK 时开发调试）：
    方式1: 命令行 `python agent/main.py --mock`
    方式2: 环境变量 `GALBOT_MOCK=1`

真机接口说明（Galbot SDK v1.10.0 / G1）：
- 底盘移动走 GalbotNavigation.move_straight_to(goal_pose, is_blocking, timeout)，
  goal_pose = [x, y, z, qx, qy, qz, qw]，相对当前 base_link（odom 系，单位米/四元数）。
- 用导航前必须先把控制器切到 CHASSIS_POSE_CTRL（本模块在 _get_nav() 里自动完成）。
- SDK 不在项目 venv 里，_sdk_bootstrap() 负责把 SDK 包目录注入 sys.path；
  原生依赖（libfastcdr 等）需先执行 `sudo ~/GalbotSDK/install.sh`，
  或 source scripts/env.sh 设置 LD_LIBRARY_PATH。
"""

from __future__ import annotations
import glob
import logging
import math
import os
import platform
import sys

from sdk_wrapper.config import load_safety

logger = logging.getLogger("sdk_wrapper")

_robot = None
_nav = None
_mock_mode = os.environ.get("GALBOT_MOCK", "0") == "1"


def set_mock_mode(enabled: bool):
    """由 agent/main.py 在启动时根据 --mock 参数调用。"""
    global _mock_mode
    _mock_mode = enabled


def get_robot():
    """公开入口：给场景自定义封装用的共享机器人单例。

    真机返回 GalbotRobot（已 init），mock 返回 MockRobot。
    场景禁止自己 GalbotRobot() init / destroy——必须从这里拿。
    """
    return _get_robot()


def get_nav():
    """公开入口：给场景自定义封装用的共享导航单例（已切 CHASSIS_POSE_CTRL）。

    注意：直接调 nav.move_straight_to 没有 wrapper 层的自动限幅，
    请自觉遵守 safety_config.json 的量级。
    """
    return _get_nav()


def _sdk_bootstrap() -> str:
    """把 GalbotSDK 的 Python 包目录注入 sys.path，返回包目录路径。

    环境变量 GALBOT_SDK_ROOT 可覆盖 SDK 位置（默认 ~/GalbotSDK）。
    如果 galbot_sdk 已可通过 PYTHONPATH 导入（如 HPU 机器人端部署：
    PYTHONPATH=~/g1_runtime），则直接跳过注入。
    注意：本函数只解决 Python 包可见性；原生依赖请提前用 install.sh 安装
    或 source scripts/env.sh / env_hpu.sh（LD_LIBRARY_PATH 须进程启动前生效）。
    """
    import importlib.util
    if importlib.util.find_spec('galbot_sdk') is not None:
        return ''  # 已可用（PYTHONPATH 提供）
    sdk_root = os.environ.get("GALBOT_SDK_ROOT", os.path.expanduser("~/GalbotSDK"))
    arch = platform.machine()  # x86_64 / aarch64
    candidates = sorted(glob.glob(
        os.path.join(sdk_root, "galbot_sdk", f"linux-{arch}-gcc*", "lib", "python")))
    if not candidates:
        raise ImportError(
            f"未找到 galbot_sdk 包目录（arch={arch}），"
            f"请检查 GALBOT_SDK_ROOT（当前={sdk_root}）")
    pkg_dir = candidates[0]
    if pkg_dir not in sys.path:
        sys.path.insert(0, pkg_dir)
    return pkg_dir


def _get_robot():
    """获取全局机器人单例。mock 模式下返回 MockRobot。"""
    global _robot
    if _robot is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockRobot
            _robot = MockRobot()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotRobot
            _robot = GalbotRobot()
        _robot.init()
        logger.info("机器人实例已初始化 (mock=%s)", _mock_mode)
    return _robot


def _get_nav():
    """获取全局导航单例，并确保底盘控制器已切到 CHASSIS_POSE_CTRL。"""
    global _nav
    if _nav is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockNavigation
            _nav = MockNavigation()
            _nav.init()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import (ControlStatus, G1ControllerName,
                                       GalbotNavigation)
            _nav = GalbotNavigation()
            _nav.init()
            robot = _get_robot()
            res = robot.switch_controller(G1ControllerName.CHASSIS_POSE_CTRL)
            if res != ControlStatus.SUCCESS:
                raise RuntimeError(f"切换底盘控制器失败: {res}")
            logger.info("底盘控制器已切换到 CHASSIS_POSE_CTRL")
        logger.info("导航实例已初始化 (mock=%s)", _mock_mode)
    return _nav


def shutdown():
    """真机 SDK 清理（程序退出时必须调用）。mock 模式下仅记录日志。

    注意：SDK destroy() 后同进程无法再 init()，重启 agent 请重新起进程。
    """
    global _robot, _nav
    if _robot is None and _nav is None:
        return
    try:
        if _mock_mode:
            if _nav is not None:
                _nav.stop_navigation()
            logger.info("[MOCK] shutdown()")
        else:
            if _nav is not None:
                _nav.stop_navigation()
            if _robot is not None:
                _robot.request_shutdown()
                _robot.wait_for_shutdown()
                _robot.destroy()
            logger.info("SDK 资源已释放")
    except Exception as e:  # noqa: BLE001 - 清理失败也要让退出流程继续
        logger.error("SDK 清理异常: %s", e)
    finally:
        _robot = None
        _nav = None


def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        logger.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


def _yaw_to_quaternion(yaw_rad: float) -> tuple:
    """偏航角（弧度）→ 四元数 (qx, qy, qz, qw)。"""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _move_straight(x_m: float, yaw_rad: float) -> dict:
    """相对当前位姿走直线 + 转角的公共实现（阻塞执行）。

    Args:
        x_m:     沿当前朝向的位移（米），正值向前
        yaw_rad: 相对转角（弧度），正值左转
    """
    cfg = load_safety()
    nav = _get_nav()
    qx, qy, qz, qw = _yaw_to_quaternion(yaw_rad)
    # 按限速估一个超时：理论耗时 ×2 再加 5s 余量，防止慢速时误判超时
    speed = max(cfg["max_base_speed_mps"], 0.05)
    timeout = abs(x_m) / speed * 2 + 5.0
    try:
        import numpy as np
        goal = np.array([x_m, 0.0, 0.0, qx, qy, qz, qw], dtype=np.float64)
    except ImportError:
        goal = [x_m, 0.0, 0.0, qx, qy, qz, qw]
    ok, status = nav.move_straight_to(goal, is_blocking=True, timeout=timeout)
    return {"success": bool(ok), "status": str(status),
            "message": "ok" if ok else f"move_straight_to: {status}"}


# ---------------------------------------------------------------------------
# 底盘运动接口
# ---------------------------------------------------------------------------

def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米，负值即后退）。自动限幅，阻塞执行。

    Returns:
        {"success": bool, "actual_distance_cm": float, "message": str}
    """
    cfg = load_safety()
    distance_m = _clamp(distance_cm / 100.0, cfg["max_base_distance_m"], "distance")
    r = _move_straight(distance_m, 0.0)
    r["actual_distance_cm"] = distance_m * 100
    return r


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(distance_cm))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    cfg = load_safety()
    angle_deg = _clamp(angle_deg, cfg["max_turn_deg"], "turn_angle")
    r = _move_straight(0.0, math.radians(angle_deg))
    r["actual_angle_deg"] = angle_deg
    return r


def stop_base() -> dict:
    """立即停止底盘运动（场景退出前建议调用）。"""
    if _nav is not None and hasattr(_nav, "stop_navigation"):
        _nav.stop_navigation()
    else:
        _get_nav().stop_navigation()
    return {"success": True, "message": "base stopped"}


# ---------------------------------------------------------------------------
# 感知接口
# ---------------------------------------------------------------------------

def read_sensor(sensor_name: str):
    """读取传感器数据。

    Args:
        sensor_name: 传感器名称，如 'front_camera' / 'lidar' / 'imu'

    Returns:
        传感器数据（具体格式取决于传感器类型）

    TODO(负责人A): 按 Galbot SDK v1.10.0 实际接口补齐相机订阅、雷达读取等。
    真机可用接口参考：robot.get_imu_data(sensor_id) / get_depth_data(camera_id)。
    """
    robot = _get_robot()
    if hasattr(robot, "get_sensor_data"):
        return robot.get_sensor_data(sensor_name)
    raise NotImplementedError(f"传感器 '{sensor_name}' 接口尚未封装，请联系负责人 A")
