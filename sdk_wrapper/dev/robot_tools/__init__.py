"""robot_tools/ —— 暴露给大模型的 Tool 层。

按分工（analysis/Galbot_G1_双人开发接口标准_v0.1.md §0）：

    已实现                      暂未实现
    ----------------------     -----------------------
    perception.py              state.py
    motion.py
    navigation.py
    safety.py

所有 Tool 都必须返回 contracts.ToolResult，并用 .to_dict() 输出。
"""

from __future__ import annotations

__all__ = [
    "motion",
    "navigation",
    "perception",
    "safety",
    "stereo_depth",
    "leg_joint_control",
]
