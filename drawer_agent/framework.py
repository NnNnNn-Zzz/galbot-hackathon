"""定位并加载上层框架 drawer_demo（状态机与几何），本仓库不复制其源码。

单一来源仍在 `medicine_frontend/drawer_demo`。按顺序解析：
  1. 环境变量 DRAWER_DEMO_ROOT 指向的目录（其下应有 drawer_demo/__init__.py）
  2. 已能 import 的 drawer_demo（PYTHONPATH / 已安装）
  3. 仓库同级的 ../medicine_frontend

解析不到直接报错，不做降级：拿不到状态机就没有可运行的流程，
静默跳过只会让人以为跑通了。
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from types import SimpleNamespace

PACKAGE = "drawer_demo"
PROBE = Path(__file__).resolve().parents[1]


def _candidates() -> list[Path]:
    roots = []
    env = os.environ.get("DRAWER_DEMO_ROOT")
    if env:
        roots.append(Path(env))
    roots.append(PROBE.parent / "medicine_frontend")
    roots.append(PROBE / "medicine_frontend")
    return roots


def _root_of(package_dir: Path) -> Path:
    return package_dir.parent if package_dir.name == PACKAGE else package_dir


def locate() -> Path | None:
    """返回 sys.path 上应追加的目录（drawer_demo 的父目录）。"""
    for root in _candidates():
        if (root / PACKAGE / "__init__.py").is_file():
            return root
    return None


def load():
    """返回框架对象集合。字段缺失即版本不匹配，立即失败。"""
    try:
        module = importlib.import_module(PACKAGE)
    except ImportError:
        root = locate()
        if root is None:
            raise ImportError(
                "找不到 drawer_demo 框架包。设置 DRAWER_DEMO_ROOT 指向"
                " 含 drawer_demo/ 的目录（通常是 medicine_frontend），"
                " 或把仓库与 medicine_frontend 放在同一父目录下。") from None
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        module = importlib.import_module(PACKAGE)

    needed = ("FixedBoxDemo", "validate_recipe", "assess", "PLAN", "EVIDENCE",
              "Context", "Result", "Frame", "Action", "Receipt", "DemoIO", "GateError",
              "flow_options", "plan_for", "preflight_keys", "route_keys")
    missing = [n for n in needed if not hasattr(module, n)]
    if missing:
        raise ImportError(f"drawer_demo 版本不匹配，缺少：{', '.join(missing)}")
    return SimpleNamespace(module=module, path=Path(module.__file__).resolve(),
                           **{n: getattr(module, n) for n in needed})


def schema() -> str:
    from importlib import import_module
    return import_module(f"{PACKAGE}.runtime").SCHEMA
