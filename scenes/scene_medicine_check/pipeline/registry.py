"""stage 注册表：名字 → 实现类。

队友接入只需要两步，都不碰 runner：
  1. 在 pipeline/policies/ 下新建一个 .py，用 @register_stage("你的名字") 装饰类；
  2. 在 pipeline/configs/ 里复制一份配置，把那一步的 "stage" 改成你的名字。
policies/ 下的模块会被自动发现，不需要改任何 __init__.py。
"""

from __future__ import annotations

import importlib
import logging
import pkgutil
from typing import Dict, List, Type

from .contracts import Stage

logger = logging.getLogger("scene.medicine.registry")

_REGISTRY: Dict[str, Type[Stage]] = {}
_LOADED = False


def register_stage(name: str):
    """把一个 Stage 实现注册成 name。重复注册同名不同类会直接报错（防误覆盖）。"""

    def deco(cls: Type[Stage]) -> Type[Stage]:
        if not isinstance(cls, type) or not issubclass(cls, Stage):
            raise TypeError(f"{cls!r} 不是 Stage 的子类")
        exist = _REGISTRY.get(name)
        if exist is not None and exist is not cls:
            raise RuntimeError(
                f"stage 名冲突：'{name}' 已被 {exist.__module__}.{exist.__name__} 注册，"
                f"现在又要被 {cls.__module__}.{cls.__name__} 覆盖")
        cls.stage_name = name
        _REGISTRY[name] = cls
        return cls

    return deco


def _autodiscover(pkg_name: str) -> List[str]:
    """导入包内所有非下划线开头的模块，触发其中的注册装饰器。"""
    loaded = []
    try:
        pkg = importlib.import_module(pkg_name)
    except ImportError as exc:
        logger.debug("跳过不存在的包 %s：%s", pkg_name, exc)
        return loaded
    path = getattr(pkg, "__path__", None)
    if not path:
        return loaded
    for mod in pkgutil.iter_modules(path):
        if mod.name.startswith("_"):
            continue
        full = f"{pkg_name}.{mod.name}"
        importlib.import_module(full)
        loaded.append(full)
    return loaded


def load_all(force: bool = False) -> Dict[str, Type[Stage]]:
    """加载内置 stages 与队友的 policies。幂等。"""
    global _LOADED
    if _LOADED and not force:
        return _REGISTRY
    base = __package__ or "pipeline"
    got = _autodiscover(f"{base}.stages")
    got += _autodiscover(f"{base}.policies")
    _LOADED = True
    logger.info("stage 注册表已加载 %d 个实现（模块 %d 个）", len(_REGISTRY), len(got))
    return _REGISTRY


def get_stage(name: str) -> Type[Stage]:
    load_all()
    cls = _REGISTRY.get(name)
    if cls is None:
        raise KeyError(f"未注册的 stage：'{name}'；已注册：{sorted(_REGISTRY)}")
    return cls


def list_stages() -> List[str]:
    load_all()
    return sorted(_REGISTRY)
