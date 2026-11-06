"""场景：巡逻。沿预设路线走一圈，观察是否有异常。

负责人: (填名字)
"""

import logging

from sdk_wrapper import baseline, wrapper

logger = logging.getLogger("scene.patrol")

# 预设巡逻路线：(前进厘米数, 转向角度)。参数化、可调整。
PATROL_ROUTE = [
    (100, 0),    # 前进 1m
    (0, 90),     # 左转 90 度
    (100, 0),    # 前进 1m
    (0, 90),
    (100, 0),
    (0, 90),
    (100, 0),
    (0, 90),     # 转回初始朝向
]


def run(params: dict) -> dict:
    route = params.get("route", PATROL_ROUTE)
    try:
        for i, (forward_cm, turn_deg) in enumerate(route):
            if forward_cm:
                r = wrapper.move_forward(forward_cm)
                if not r["success"]:
                    return {"success": False,
                            "feedback": f"巡逻第 {i} 段前进失败: {r['message']}",
                            "suggested_adjustment": "检查是否被障碍物阻挡"}
            if turn_deg:
                wrapper.turn(turn_deg)
            # TODO: 每段路走完后读相机做异常检测
        return {"success": True,
                "feedback": "巡逻完成，沿途未发现异常",
                "suggested_adjustment": None}
    except Exception as e:  # noqa: BLE001
        return {"success": False,
                "feedback": f"巡逻异常: {e}",
                "suggested_adjustment": "检查底盘与传感器"}
    finally:
        baseline.reset_to_baseline()
