"""algorithms/ —— B（大脑与眼睛）的算法层。

模块划分：
    geometry.py        四元数/外参/反投影/掩膜统计（纯 numpy）
    color_detection.py 纯 OpenCV 颜色目标检测（可替换为 YOLO 等）
    scene_analyzer.py  2D 检测 + 深度融合 -> 给大模型的结构化场景
    place_resolver.py  自然语言地名 -> 白名单 place_id

依赖方向（禁止反向）：
    agent/app.py -> robot_tools/perception.py -> algorithms/*
    algorithms/* 不 import robot_tools，也不 import galbot_sdk。
这样算法层可以在 Windows 上单测，不需要机器人。
"""

from __future__ import annotations

__all__ = [
    "geometry",
    "color_detection",
    "scene_analyzer",
    "place_resolver",
    "calibration",
    "observation_pose",
    "charger_search",
    "charger_yolo",
    "power_strip_yolo",
]
