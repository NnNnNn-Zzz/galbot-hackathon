"""现场标定常量：所有需要示教/测量的量集中在这里，换桌子只改这个文件。

原来这些散在 baseline_flow.py 顶部，拆分 stage 后统一搬到此处，
baseline_flow 仍然 re-export 同名常量，外部按老路径 import 不会断。
"""

from __future__ import annotations

import logging
import math
from typing import List

logger = logging.getLogger("scene.medicine.tuning")

# ===========================================================================
# 1. 起始位姿：官方 example6 的 21 关节初值
# ===========================================================================
LEG_JOINT_NAMES: List[str] = [
    "leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4", "leg_joint5",
]
LEG_JOINT_POSITIONS: List[float] = [0.4, 1.2, 0.8, 0.0, 0.0]

UPPER_JOINT_NAMES: List[str] = [
    "head_joint1", "head_joint2",
    "left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
    "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7",
    "right_arm_joint1", "right_arm_joint2", "right_arm_joint3",
    "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7",
]
UPPER_JOINT_POSITIONS: List[float] = [
    0.0, 0.0,
    1.90, -1.46, -0.54, -1.96, 0.0, -0.4, 0.0,
    -1.90, 1.34, 0.54, 1.96, 0.0, 0.4, 0.0,
]

# 右臂初始位 = 抱持位：抓取前先到这里，抓取后也回到这里。
# 2026-11-07 真机反馈：原抱持位肘部距身体仅约 1 cm。按 URDF 离线 FK，
# joint2 1.46 -> 1.34 rad 让肘部向体外约 +4 cm，目标间隙约 5 cm；真机需目视复核。
# 两处共用同一数组，避免初始位仍用旧角度而只有收臂位调宽。
CARRY_JOINT_NAMES: List[str] = UPPER_JOINT_NAMES[9:16]
CARRY_JOINT_POSITIONS: List[float] = UPPER_JOINT_POSITIONS[9:16]

# ===========================================================================
# 2. 抓取位姿（base_link 系，[x, y, z, qx, qy, qz, qw]，单位 m）
# z 已按实测桌面高度 0.72 m 标定（2026-11-07，用户现场确认）：
#   初次 GRASP z=0.75 真机偏低，按下爪反馈抬高 0.02 -> 0.77（桌面上方 5 cm）；
#   PRE_GRASP=0.85 / LIFT=0.90 保留实验 1 已验证可达值，不随下爪点额外抬高。
# x/y 仍为官方 example6 占位值（x 0.40 / y -0.10），靠现场摆放对准，允许微调。
# 注意：base_link 的 y 轴指向机器人**左侧**，右臂作业区在 y<0 一侧。
# ===========================================================================
PRE_GRASP_POSE: List[float] = [0.35, -0.10, 0.85, 0.0, 0.0, 0.0, 1.0]
GRASP_POSE: List[float] = [0.40, -0.10, 0.77, 0.0, 0.0, 0.0, 1.0]
LIFT_POSE: List[float] = [0.40, -0.10, 0.90, 0.0, 0.0, 0.0, 1.0]

# ===========================================================================
# 3. 垃圾桶投放位姿（base_link 系）
# TODO(标定): 右转 30° 后垃圾桶落在右前下方，这里填末端到达位姿
# ===========================================================================
PRE_DROP_POSE: List[float] = [0.45, -0.35, 0.60, 0.0, 0.0, 0.0, 1.0]
DROP_POSE: List[float] = [0.45, -0.35, 0.50, 0.0, 0.0, 0.0, 1.0]

# ===========================================================================
# 4. 夹爪
# ===========================================================================
GRIPPER_SIDE = "right"         # left | right（2026-11-07 起全程右臂）
GRIPPER_OPEN_M = 0.10          # G1 夹爪量程 0~0.12 m
GRIPPER_CLOSE_M = 0.075        # 药盒实测宽 7.5 cm：不再闭合到 2 cm 挤压盒体
GRIPPER_VELOCITY_MPS = 0.05
GRIPPER_EFFORT_N = 10.0

# ===========================================================================
# 5. 运动距离与角度
# ===========================================================================
TURN_AROUND_DEG = 180.0        # 转身（旧流程保留，基线已不再使用）
DRIVE_DISTANCE_CM = 300.0      # 直行 3 m（旧流程保留，基线已不再使用）
TURN_RIGHT_TO_BIN_DEG = -30.0  # 右转 30°（负值 = 右转）
SHIFT_LEFT_M = 0.5             # 左移 0.5 m：侧移不转身，替代旧的「转身180°+直行3m」

# ===========================================================================
# 6. 播报
# ===========================================================================
ITEM_NAME = "999 感冒灵"        # TODO(填写): 固定过期药的名称
ANNOUNCE_TEMPLATE = "识别{name}过期，帮你清理掉吧~"
ANNOUNCE_PAUSE_S = 5.0         # 场景描述：停顿五秒等人说「好」

# ===========================================================================
# 7. 动作参数
# ===========================================================================
MOVE_TIMEOUT_S = 20.0          # 单次末端位姿移动超时
ARM_SPEED_RAD_S = 0.2          # 关节运动速度上限


def split_distance(total_cm: float) -> List[float]:
    """把总距离按安全上限切段，返回每段厘米数。

    safety_config.json 的 max_base_distance_m 默认 2.0，而基线要直行 3 m。
    这里主动分段，避免 wrapper 静默截断成 2 m 导致走不到另一张桌子。
    """
    limit_cm = 200.0
    try:
        from sdk_wrapper.config import load_safety  # 局部 import：mock 模式下也不需要真机

        limit_cm = float(load_safety().get("max_base_distance_m", 2.0)) * 100.0
    except Exception as exc:  # noqa: BLE001 - 读不到安全配置就退回 2 m 上限
        logger.warning("读安全配置失败（%s），按默认 2 m 切段", exc)
    total = abs(float(total_cm))
    if total <= limit_cm:
        return [total]
    n = int(math.ceil(total / limit_cm))
    return [total / n] * n
