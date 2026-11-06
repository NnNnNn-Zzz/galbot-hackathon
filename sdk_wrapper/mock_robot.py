"""Mock 机器人：无真机时用于开发和测试。

模拟 Galbot SDK v1.10.0 (G1) 的最小接口：
- MockRobot      ↔ galbot_sdk.g1.GalbotRobot（init / switch_controller / 关机序列）
- MockNavigation ↔ galbot_sdk.g1.GalbotNavigation（move_straight_to / stop_navigation）

记录所有调用并返回成功。场景负责人应先用 mock 测试自己的 run() 和归位逻辑。
"""

import logging
import math

logger = logging.getLogger("sdk_wrapper.mock")

# 与真机 galbot_sdk.g1.ControlStatus.SUCCESS 对齐的哨兵值
MOCK_SUCCESS = "SUCCESS"


class MockRobot:
    def __init__(self):
        self.initialized = False
        self.call_history = []  # 记录所有调用，便于测试断言
        self.controller = None

    def init(self):
        self.initialized = True
        logger.info("[MOCK] robot.init()")
        return True

    def switch_controller(self, controller_name):
        self.controller = str(controller_name)
        self.call_history.append(("switch_controller", {"name": self.controller}))
        logger.info("[MOCK] switch_controller(%s)", self.controller)
        return MOCK_SUCCESS

    def request_shutdown(self):
        self.call_history.append(("request_shutdown", {}))
        logger.info("[MOCK] request_shutdown()")

    def wait_for_shutdown(self):
        self.call_history.append(("wait_for_shutdown", {}))
        logger.info("[MOCK] wait_for_shutdown()")

    def destroy(self):
        self.call_history.append(("destroy", {}))
        logger.info("[MOCK] destroy()")

    def get_sensor_data(self, sensor_name):
        logger.info("[MOCK] get_sensor_data(%s)", sensor_name)
        return None


class MockNavigation:
    def __init__(self):
        self.initialized = False
        self.call_history = []
        # 累计位姿（相对出发点），便于 mock 测试断言
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def init(self):
        self.initialized = True
        logger.info("[MOCK] nav.init()")
        return True

    def move_straight_to(self, goal_pose, is_blocking=True, timeout=8.0):
        """goal_pose = [x, y, z, qx, qy, qz, qw]，相对当前 base_link（米/四元数）。"""
        x, y, z, qx, qy, qz, qw = [float(v) for v in goal_pose]
        # 从四元数反解 yaw，日志可读
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.call_history.append(("move_straight_to",
                                  {"x": x, "yaw_deg": math.degrees(yaw),
                                   "timeout": timeout}))
        # 简易位姿积分：先转后走
        self.pose["yaw"] += yaw
        self.pose["x"] += x * math.cos(self.pose["yaw"])
        self.pose["y"] += x * math.sin(self.pose["yaw"])
        logger.info("[MOCK] move_straight_to x=%.3f m, yaw=%.1f deg "
                    "(blocking=%s, timeout=%.1f) → 累计位姿 (%.2f, %.2f, %.1f°)",
                    x, math.degrees(yaw), is_blocking, timeout,
                    self.pose["x"], self.pose["y"], math.degrees(self.pose["yaw"]))
        return (True, "MOCK OK")

    def stop_navigation(self):
        self.call_history.append(("stop_navigation", {}))
        logger.info("[MOCK] stop_navigation()")
        return (True, "MOCK OK")
