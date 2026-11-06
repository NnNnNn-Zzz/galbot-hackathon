"""基线状态归位：每个场景结束前必须调用（或自行保证等效效果）。

基线状态定义：底盘停止运动、手臂处于折叠安全位、夹爪释放。
"""

from __future__ import annotations
import logging

from sdk_wrapper.wrapper import _get_robot

logger = logging.getLogger("sdk_wrapper.baseline")


def reset_to_baseline() -> bool:
    """让机器人回到基线状态。返回是否成功。"""
    robot = _get_robot()
    ok = True
    try:
        # 1. 底盘停止
        if hasattr(robot, "stop_base"):
            robot.stop_base()
        # 2. TODO(负责人A): 手臂回到折叠安全位
        #    按 Galbot SDK 实际接口调用关节控制或预设姿态，例如：
        #    robot.set_arm_pose(robot.FOLD_POSE, is_blocking=True)
        # 3. TODO(负责人A): 夹爪释放
        #    robot.set_gripper(open=True)
        logger.info("已归位到基线状态")
    except Exception as e:  # noqa: BLE001 - 归位失败也要让流程继续
        logger.error("归位失败: %s", e)
        ok = False
    return ok


def check_baseline() -> dict:
    """轻量状态检查：读当前状态，判断是否在基线附近。

    用于：场景切换前检查上一个场景是否正确归位。
    结果只记录不阻塞（见 docs/DEVELOPMENT_RULES.md 补充建议 2）。
    """
    robot = _get_robot()
    # TODO(负责人A): 读底盘速度、关节位置，与基线阈值对比
    return {"at_baseline": True, "detail": "未实现，默认通过", "robot": type(robot).__name__}
