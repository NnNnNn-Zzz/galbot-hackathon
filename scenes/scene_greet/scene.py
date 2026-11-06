"""场景：打招呼。检测到有人接近时挥手。

负责人: (填名字)
"""

import logging

from sdk_wrapper import baseline, wrapper

logger = logging.getLogger("scene.greet")


def run(params: dict) -> dict:
    """场景入口，必须保持这个签名。

    Args:
        params: Agent 调用时传入的参数（本场景暂不使用）

    Returns:
        {"success": bool, "feedback": str, "suggested_adjustment": str | None}
    """
    try:
        # TODO: 接视觉感知确认前方有人
        # TODO: 调封装层挥手机械臂动作（需要负责人 A 新增 wave_hand() 封装）

        # 演示动作：原地小转 15 度再转回，表示"打了个招呼"
        wrapper.turn(15)
        wrapper.turn(-15)

        return {"success": True,
                "feedback": "已挥手打招呼",
                "suggested_adjustment": None}
    except Exception as e:  # noqa: BLE001
        return {"success": False,
                "feedback": f"打招呼失败: {e}",
                "suggested_adjustment": "检查手臂是否被阻挡"}
    finally:
        # 归位是硬性规范：无论成功失败都要执行
        baseline.reset_to_baseline()
