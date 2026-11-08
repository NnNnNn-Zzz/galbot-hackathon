"""抽屉场景的公共工具：示教文件读写路径解析 + 点位读取。

6 个 stage 都要从 teach_drawer.json 里取点，路径解析写 6 遍必然有一份算错。
统一放这里，改一处即全部生效。

路径基准 = **仓库根**（galbot-hackathon/），因为配置里写的
"pipeline/configs/teach_drawer.json" 是相对场景目录的语义，
所以这里先把相对路径补成 **scenes/scene_medicine_check/** 下的完整路径。
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Dict, List, Optional

logger = logging.getLogger("scene.medicine.teach")

#: 本文件在 scenes/scene_medicine_check/pipeline/ 下，上溯 1 级 = 场景目录
_SCENE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
#: 再上溯 2 级 = 仓库根 galbot-hackathon/
_REPO_ROOT = os.path.abspath(os.path.join(_SCENE_DIR, "..", ".."))


def resolve_teach_path(path: str) -> str:
    """把配置里的示教文件路径解析成绝对路径。

    依次尝试：已是绝对路径 → 相对仓库根 → 相对场景目录。
    哪条都存在就返回第一条；都不存在则返回"相对场景目录"那一条（便于报错信息定位）。
    """
    if not path:
        return path
    if os.path.isabs(path):
        return path
    cand_a = os.path.abspath(os.path.join(_REPO_ROOT, path))
    cand_b = os.path.abspath(os.path.join(_SCENE_DIR, path))
    if os.path.exists(cand_a):
        return cand_a
    if os.path.exists(cand_b):
        return cand_b
    return cand_b


def load_teach(path: str) -> Dict[str, Any]:
    """读整个示教文件。失败返回 {} 并记日志（不抛）。"""
    real = resolve_teach_path(path)
    if not os.path.exists(real):
        logger.error("示教文件不存在：%s", real)
        return {}
    try:
        with open(real, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except Exception as exc:  # noqa: BLE001 - 文件坏了要如实报，不许假装读到
        logger.error("示教文件解析失败（%s）：%s", real, exc)
        return {}


def load_point(path: str, point: str) -> Dict[str, Any]:
    """取一个示教点记录。取不到返回 {}。"""
    data = load_teach(path)
    pts = data.get("points") or {}
    rec = pts.get(point) or {}
    if not rec:
        logger.error("示教点 %s 不存在（已有：%s）", point, sorted(pts))
    return rec


def point_names(path: str) -> List[str]:
    return sorted((load_teach(path).get("points") or {}).keys())


def joint_arrays(rec: Dict[str, Any]) -> Optional[tuple]:
    """从点位记录里取 (names, positions)，不完整就返回 None。

    数量和名字必须成对，缺一个都算不可用 —— 宁可报缺，不许按索引猜。
    """
    names = rec.get("joint_names")
    pos = rec.get("joint_positions")
    if not names or not pos or len(names) != len(pos):
        return None
    return list(names), [float(v) for v in pos]


def pose_array(rec: Dict[str, Any]) -> Optional[List[float]]:
    """从点位记录里取末端位姿 [7]。不完整返回 None。"""
    pose = rec.get("pose")
    if not pose or len(pose) != 7:
        return None
    return [float(v) for v in pose]


def write_teach(path: str, payload: Dict[str, Any]) -> str:
    """写示教文件，返回实际写入的绝对路径。已存在则先备份为 .bak。"""
    real = resolve_teach_path(path)
    os.makedirs(os.path.dirname(real), exist_ok=True)
    if os.path.exists(real):
        bak = real + ".bak"
        try:
            os.replace(real, bak)
            logger.warning("已有示教文件被备份为 %s", bak)
        except OSError as exc:
            logger.warning("备份旧示教文件失败（%s），将直接覆盖", exc)
    with open(real, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return real


def _safe_read(io: Any, method: str, **kwargs: Any) -> Dict[str, Any]:
    """调只读接口并兜住异常。

    为什么需要：MockIO 的故障注入会真的抛 RuntimeError（模拟真机 SDK 抛异常），
    而"读一下夹爪宽度"失败不该让整个阶段抛异常 —— 那会被 runner 记成 err，
    丢掉"这一步其实动作都做完了，只是没读到反馈"这个信息。
    统一转成 {"success": False, "message": ...} 让调用方按软失败处理。
    """
    fn = getattr(io, method, None)
    if not callable(fn):
        return {"success": False, "message": f"io 无 {method} 方法"}
    try:
        ret = fn(**kwargs)
    except Exception as exc:  # noqa: BLE001 - 只读失败一律软处理
        logger.warning("%s 抛异常（按读取失败处理）：%s", method, exc)
        return {"success": False, "message": f"{method} 抛异常：{exc}"}
    return ret if isinstance(ret, dict) else {"success": bool(ret), "message": str(ret)}
