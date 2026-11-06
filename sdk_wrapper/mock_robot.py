"""Mock 机器人：无真机时用于开发和测试。

模拟 GalbotRobot 的最小接口，记录所有调用并返回成功。
场景负责人应先用 mock 测试自己的 run() 和归位逻辑。
"""

import logging

logger = logging.getLogger("sdk_wrapper.mock")


class MockRobot:
    def __init__(self):
        self.initialized = False
        self.call_history = []  # 记录所有调用，便于测试断言
        self.base_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def init(self):
        self.initialized = True
        logger.info("[MOCK] robot.init()")
        return True

    def set_base_pose(self, x=0, y=0, yaw=0, frame_id="rel(0)", is_blocking=True):
        self.call_history.append(("set_base_pose", dict(x=x, y=y, yaw=yaw,
                                                        frame_id=frame_id)))
        self.base_pose = {"x": x, "y": y, "yaw": yaw}
        logger.info("[MOCK] set_base_pose x=%.3f y=%.3f yaw=%.3f (%s)", x, y, yaw, frame_id)
        return True

    def stop_base(self):
        self.call_history.append(("stop_base", {}))
        logger.info("[MOCK] stop_base()")
        return True

    def get_sensor_data(self, sensor_name):
        logger.info("[MOCK] get_sensor_data(%s)", sensor_name)
        return None
