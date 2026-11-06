"""SDK 封装层：所有对 galbot_sdk 的调用都必须经过这里。

安全策略：
- 所有运动参数按 safety_config.json 限幅（clamp），超限不报错、静默截断并记日志。
- galbot_sdk 全局单例，场景脚本严禁自行 init()/destroy()。
- 支持 MOCK 模式（无真机/无 SDK 时开发调试）：
    方式1: 命令行 `python agent/main.py --mock`
    方式2: 环境变量 `GALBOT_MOCK=1`
"""

import logging
import os

from sdk_wrapper.config import load_safety

logger = logging.getLogger("sdk_wrapper")

_robot = None
_mock_mode = os.environ.get("GALBOT_MOCK", "0") == "1"


def set_mock_mode(enabled: bool):
    """由 agent/main.py 在启动时根据 --mock 参数调用。"""
    global _mock_mode
    _mock_mode = enabled


def _get_robot():
    """获取全局机器人单例。mock 模式下返回 MockRobot。"""
    global _robot
    if _robot is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockRobot
            _robot = MockRobot()
        else:
            import galbot_sdk  # 仅封装层允许 import
            _robot = galbot_sdk.GalbotRobot()
        _robot.init()
        logger.info("机器人实例已初始化 (mock=%s)", _mock_mode)
    return _robot


def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        logger.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# 底盘运动接口
# ---------------------------------------------------------------------------

def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米）。自动限幅，阻塞执行。

    Returns:
        {"success": bool, "actual_distance_cm": float, "message": str}
    """
    cfg = load_safety()
    distance_m = abs(distance_cm) / 100.0
    distance_m = _clamp(distance_m, cfg["max_base_distance_m"], "distance")
    robot = _get_robot()
    # frame_id='rel(0)' 表示相对当前位置
    result = robot.set_base_pose(x=distance_m, y=0, yaw=0,
                                 frame_id='rel(0)', is_blocking=True)
    return {"success": bool(result), "actual_distance_cm": distance_m * 100,
            "message": "ok" if result else "set_base_pose failed"}


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(distance_cm))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    import math
    cfg = load_safety()
    angle_deg = _clamp(angle_deg, cfg["max_turn_deg"], "turn_angle")
    robot = _get_robot()
    result = robot.set_base_pose(x=0, y=0, yaw=math.radians(angle_deg),
                                 frame_id='rel(0)', is_blocking=True)
    return {"success": bool(result), "actual_angle_deg": angle_deg,
            "message": "ok" if result else "set_base_pose failed"}


def stop_base() -> dict:
    """立即停止底盘运动（场景退出前建议调用）。"""
    robot = _get_robot()
    if hasattr(robot, "stop_base"):
        robot.stop_base()
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
    """
    robot = _get_robot()
    if hasattr(robot, "get_sensor_data"):
        return robot.get_sensor_data(sensor_name)
    raise NotImplementedError(f"传感器 '{sensor_name}' 接口尚未封装，请联系负责人 A")
