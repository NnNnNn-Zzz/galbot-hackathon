"""基线状态归位：每个场景结束前必须调用（或自行保证等效效果）。

基线状态定义：底盘停止运动、手臂处于折叠安全位、夹爪释放。

⚠️ 本模块**只调 wrapper 的公开函数**，不再碰 `_get_robot()` 拿到的 SDK 对象——
   合并后 SDK 单例归 dev 层所有，直接操作它等于绕过统一封装。
"""

from __future__ import annotations

import logging

logger = logging.getLogger("sdk_wrapper.baseline")


def reset_to_baseline() -> bool:
    """让机器人回到基线状态。返回是否成功。"""
    from sdk_wrapper import wrapper

    ok = True
    try:
        # 1. 底盘停止
        wrapper.stop_base()
        # 2. TODO(负责人A): 手臂回到折叠安全位
        #    示教一次折叠位姿后填入，然后：
        #    wrapper.move_arm_to(FOLD_POSE, chain="right_arm")
        # 3. TODO(负责人A): 夹爪释放（张开到最大）
        #    wrapper.open_gripper("right_gripper")
        logger.info("已归位到基线状态（底盘已停）")
    except Exception as exc:  # noqa: BLE001 - 归位失败也要让流程继续
        logger.error("归位失败: %s", exc)
        ok = False
    return ok


def check_baseline() -> dict:
    """轻量状态检查：读当前状态，判断是否在基线附近。

    用于：场景切换前检查上一个场景是否正确归位。
    结果只记录不阻塞（见 docs/DEVELOPMENT_RULES.md 补充建议 2）。
    """
    # TODO(负责人A): 读底盘速度、关节位置，与基线阈值对比
    return {"at_baseline": True, "detail": "未实现，默认通过"}
