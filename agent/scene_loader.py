"""场景加载器：读取 registry/scenes.yaml，动态 import 场景模块并调用 run()。

场景脚本与 Agent 运行在同一 Python 进程（见 docs/DEVELOPMENT_RULES.md），
因此这里用 importlib 直接加载，不起子进程。
"""

from __future__ import annotations
import importlib.util
import logging
import os
import sys

import yaml

logger = logging.getLogger("agent.scene_loader")

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_REGISTRY_PATH = os.path.join(_ROOT, "registry", "scenes.yaml")


def load_registry() -> list:
    """读取场景注册表，返回场景条目列表。"""
    with open(_REGISTRY_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or []


def registry_summary() -> str:
    """生成给 LLM 看的场景能力描述文本。"""
    lines = []
    for s in load_registry():
        lines.append(
            f"- 场景 {s['name']}\n"
            f"  何时调用: {s.get('when_to_call', '未说明')}\n"
            f"  成功反馈: {s.get('expected_feedback', {}).get('success', '')}\n"
            f"  失败反馈: {s.get('expected_feedback', {}).get('failure', '')}\n"
            f"  失败恢复: {s.get('recovery_on_failure', '')}\n"
            f"  退出状态: {s.get('expected_exit_state', '')}")
    return "\n".join(lines) if lines else "（暂无已注册场景）"


def run_scene(scene_name: str, params: dict | None = None) -> dict:
    """按名字加载并执行场景，返回场景结果 dict。

    场景必须提供 run(params: dict) -> dict 入口，返回格式：
        {"success": bool, "feedback": str, "suggested_adjustment": str | None}
    """
    registry = {s["name"]: s for s in load_registry()}
    if scene_name not in registry:
        return {"success": False, "feedback": f"场景 '{scene_name}' 未注册",
                "suggested_adjustment": "检查 registry/scenes.yaml"}

    entry = registry[scene_name]
    scene_path = os.path.join(_ROOT, entry["path"])
    module_name = f"scene_{scene_name}"

    spec = importlib.util.spec_from_file_location(module_name, scene_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
        logger.info("开始执行场景: %s", scene_name)
        result = module.run(params or {})
    except Exception as e:  # noqa: BLE001 - 场景异常不能拖垮主进程
        logger.exception("场景 %s 执行异常", scene_name)
        result = {"success": False, "feedback": f"场景异常: {e}",
                  "suggested_adjustment": "检查场景代码异常路径"}
    return result
