"""Mock 机器人：无真机时用于开发和测试。

模拟 Galbot SDK v1.10.0 (G1) 的最小接口：
- MockRobot      ↔ galbot_sdk.g1.GalbotRobot（init / switch_controller / 关机序列）
- MockNavigation ↔ galbot_sdk.g1.GalbotNavigation（move_straight_to / stop_navigation）

记录所有调用并返回成功。场景负责人应先用 mock 测试自己的 run() 和归位逻辑。
"""

from __future__ import annotations
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
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}  # 累计位姿

    def init(self):
        self.initialized = True
        logger.info("[MOCK] robot.init()")
        return True

    def switch_controller(self, controller_name):
        self.controller = str(controller_name)
        self.call_history.append(("switch_controller", {"name": self.controller}))
        logger.info("[MOCK] switch_controller(%s)", self.controller)
        return MOCK_SUCCESS

    def set_base_pose(self, x=0, y=0, yaw=0, frame_id='rel(0)',
                      reference_frame_id='odom', is_blocking=True, timeout_s=15.0,
                      **kwargs):
        """直接底盘位姿控制（对齐 GalbotRobot.set_base_pose）。"""
        self.call_history.append(("set_base_pose", {"x": x, "yaw": yaw, "frame": frame_id}))
        self.pose["x"] += x * math.cos(self.pose["yaw"])
        self.pose["y"] += x * math.sin(self.pose["yaw"])
        self.pose["yaw"] += yaw
        logger.info("[MOCK] set_base_pose x=%.3f yaw=%.3f (%s) → 累计位姿 (%.2f, %.2f, %.1f°)",
                    x, yaw, frame_id,
                    self.pose["x"], self.pose["y"], math.degrees(self.pose["yaw"]))
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

    def set_gripper_command(self, end_effector, width_m, velocity_mps,
                            effort, is_blocking=True):
        """夹爪开度控制（对齐 GalbotRobot.set_gripper_command）。mock 恒成功。"""
        self.call_history.append(("set_gripper_command",
                                  {"ee": end_effector, "width_m": width_m,
                                   "vel": velocity_mps, "effort": effort,
                                   "blocking": is_blocking}))
        logger.info("[MOCK] set_gripper_command ee=%s width=%.3fm vel=%.3f effort=%.1f",
                    end_effector, width_m, velocity_mps, effort)
        return MOCK_SUCCESS


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
        return (True, "SUCCESS")

    def navigate_with_velocity(self, vx, vy, vyaw, duration_s, enable_collision_check=True):
        self.call_history.append(("navigate_with_velocity",
                                  {"vx": vx, "vyaw": vyaw, "duration": duration_s}))
        # 简易位姿积分：按速度×时间走
        self.pose["x"] += vx * duration_s * math.cos(self.pose["yaw"])
        self.pose["y"] += vx * duration_s * math.sin(self.pose["yaw"])
        self.pose["yaw"] += vyaw * duration_s
        logger.info("[MOCK] navigate_with_velocity vx=%.3f vyaw=%.3f dur=%.1fs"
                    " → 累计位姿 (%.2f, %.2f, %.1f°)",
                    vx, vyaw, duration_s,
                    self.pose["x"], self.pose["y"], math.degrees(self.pose["yaw"]))
        return (True, "SUCCESS")

    def get_navigation_status(self):
        """返回一个带 .name 的对象，对齐真机 NavigationTaskStatus.SUCCESS。"""
        class _St:
            name = "SUCCESS"
        return _St()

    def stop_navigation(self):
        self.call_history.append(("stop_navigation", {}))
        logger.info("[MOCK] stop_navigation()")
        return (True, "SUCCESS")


class MockMotion:
    """Mock 运动规划（对齐 GalbotMotion 的手臂运动接口）。

    记录末端位姿，供场景编排逻辑的 mock 验证。
    """

    def __init__(self):
        # 右臂观测位姿（base_link 系，米）：右前方桌面上方
        self.pose = [0.45, -0.20, 0.60, 0.0, 0.0, 0.0, 1.0]
        self.call_history = []

    def get_ee_pose(self, chain="right_arm"):
        self.call_history.append(("get_ee_pose", chain))
        return {"success": True, "pose": list(self.pose),
                "frame": "base_link", "message": "ok"}

    def move_to(self, pose, chain="right_arm"):
        self.call_history.append(("move_to", {"chain": chain,
                                              "pose": [round(v, 3) for v in pose]}))
        self.pose = [float(v) for v in pose]
        return {"success": True, "status": "SUCCESS",
                "pose": list(self.pose), "message": "ok"}
