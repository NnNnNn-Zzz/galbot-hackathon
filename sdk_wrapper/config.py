"""安全参数加载。每次调用实时读文件，支持人工修改后即时生效。

注意：场景脚本不得在运行时修改 safety_config.json。
"""

from __future__ import annotations
import json
import os

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "safety_config.json")


def load_safety() -> dict:
    with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)
