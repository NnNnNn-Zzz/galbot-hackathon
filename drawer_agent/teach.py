"""把示教采集结果（`teach_drawer_demo` 的 point.json）转成适配器用的路线文件。

现场只做两件事：在示教点停稳、按 Enter 记录；再量一次抽屉实际开度。
坐标、四元数、关节角和采样时间都来自控制台只读接口，本模块只做映射与校验，
不猜任何缺失值 —— 缺哪个点就报哪个点，绝不拿相邻点插值顶替。

点 → 路线语义（与 `routes.py` 的六个路点对应）：

    HOME          → HOME           本轮原始安全位
    HANDLE_CLEAR  → HANDLE_CLEAR   松开把手后的撤离位
    RISE_MID/BOX_PRE 中 Z 更高者 → OBSERVE_HIGH
    CARRY         → START          持盒撤离位
    PULL_END      → HANDLE_RELEASE 满开时仍夹着把手的位置（重夹回这里）
    DROP          → DROP           固定桶口上方
"""

from __future__ import annotations

import json
import math
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path

from .link import finite, norm_quat
from .routes import RouteStore, SCHEMA

REQUIRED_POINTS = ("HOME", "HANDLE_PRE", "HANDLE_GRIP", "PULL_START", "PULL_END",
                   "HANDLE_CLEAR", "RISE_MID", "BOX_PRE", "BOX_HELD", "CARRY",
                   "DROP", "HOME_END")
PULL_POINT = re.compile(r"PULL_(\d{2})CM\Z")
GRIP_WIDTH_M = 0.020


class TeachError(ValueError):
    """示教数据不足以生成路线，或数据自相矛盾。"""


def _point(folder: Path, name: str) -> dict:
    path = folder / name / "point.json"
    if not path.is_file():
        raise TeachError(f"缺少示教点 {name}（{path}）")
    data = json.loads(path.read_text(encoding="utf-8"))
    state = data.get("state") or {}
    tcp = state.get("tcp")
    if not (isinstance(tcp, list) and len(tcp) == 7 and finite(*tcp)):
        raise TeachError(f"示教点 {name} 的 TCP 不是 7 维有限值")
    pose = [float(v) for v in tcp]
    pose[3:] = norm_quat(pose[3:])
    return {"pose": pose, "opening_mm": data.get("operator_measured_opening_mm"),
            "recorded_ns": data.get("recorded_ns"), "raw": data}


def _pull_series(folder: Path) -> list[tuple[int, dict]]:
    series = []
    for path in folder.iterdir():
        match = PULL_POINT.fullmatch(path.name)
        if match and (path / "point.json").is_file():
            series.append((int(match.group(1)), _point(folder, path.name)))
    series.sort(key=lambda item: item[0])
    return series


def build(session_dir, *, setup_id: str, gripper_open_m: float, drawer_opening_m: float,
          verified: dict | None = None) -> tuple[dict, list[str]]:
    """返回 (routes/1 数据, 缺口列表)。缺口非空时文件仍可生成，但不可放行。"""
    folder = Path(session_dir)
    if not folder.is_dir():
        raise TeachError(f"示教目录不存在：{folder}")
    points = {name: _point(folder, name) for name in REQUIRED_POINTS}
    pulls = _pull_series(folder)
    gaps: list[str] = []
    # PULL_END 是最后一段的实测终点（满开时仍夹着把手），并入行程曲线。
    pulls.insert(0, (0, points["PULL_START"]))
    if points["PULL_END"]["opening_mm"] is not None:
        pulls.append((99, points["PULL_END"]))

    if len(pulls) < 3:
        raise TeachError("拉出段至少需要 3 个 PULL_xxCM 采样点，否则行程曲线无法核对")
    openings = [p["opening_mm"] for _, p in pulls]
    if any(not isinstance(o, (int, float)) or o < 0 for o in openings):
        raise TeachError("PULL 点必须逐点记录卷尺实测开度（--opening-mm）")
    if any(b <= a for a, b in zip(openings, openings[1:])):
        raise TeachError("PULL 点实测开度没有单调增加，采集顺序或读数有问题")

    target_mm = drawer_opening_m * 1000.0
    if abs(float(openings[-1]) - target_mm) > 10:
        gaps.append(f"PULL_END 实测开度 {openings[-1]:.0f} mm 与配方目标 {target_mm:.0f} mm 相差超 10 mm")
    if float(points["PULL_START"]["opening_mm"] or 0) > 3:
        gaps.append("PULL_START 记录的开度不为零，抽屉起点不是闭合状态")

    top = max(("RISE_MID", "BOX_PRE"), key=lambda n: points[n]["pose"][2])
    if points["BOX_PRE"]["pose"][2] > points["RISE_MID"]["pose"][2] + 0.003:
        top = "BOX_PRE"
    for low, high in (("HANDLE_CLEAR", "RISE_MID"), ("RISE_MID", "BOX_PRE")):
        rise = points[high]["pose"][2] - points[low]["pose"][2]
        if rise <= 0.003:
            gaps.append(f"{low}→{high} 的 Z 没有上升（{rise * 1000:+.1f} mm），点序或采集有误")

    waypoints = {
        "HOME": points["HOME"]["pose"],
        "HANDLE_CLEAR": points["HANDLE_CLEAR"]["pose"],
        "OBSERVE_HIGH": points[top]["pose"],
        "START": points["CARRY"]["pose"],
        "DROP": points["DROP"]["pose"],
        "HANDLE_RELEASE": points["PULL_END"]["pose"],
    }

    pull_steps = []
    for (_, prev), (_, curr) in zip(pulls, pulls[1:]):
        dx = curr["pose"][0] - prev["pose"][0]
        dy = curr["pose"][1] - prev["pose"][1]
        dz = curr["pose"][2] - prev["pose"][2]
        pull_steps.append({"op": "move_rel", "dx": round(dx, 5),
                           "dy": round(dy, 5), "dz": round(dz, 5)})
    push_steps = [{"op": "move_rel", "dx": round(-s["dx"], 5),
                   "dy": round(-s["dy"], 5), "dz": round(-s["dz"], 5)}
                  for s in reversed(pull_steps)]

    routes = {
        "drawer_open_release": {"steps": [
            {"op": "move_tcp", "pose": points["HANDLE_PRE"]["pose"]},
            {"op": "move_tcp", "pose": points["HANDLE_GRIP"]["pose"]},
            {"op": "settle", "ms": 200},
            {"op": "grip", "width_m": GRIP_WIDTH_M},
            *pull_steps,
            {"op": "release", "width_m": gripper_open_m},
            {"op": "move_tcp", "pose": points["HANDLE_CLEAR"]["pose"]},
        ]},
        "rise_to_observe": {"steps": [
            {"op": "move_tcp", "pose": points["RISE_MID"]["pose"]},
            {"op": "move_tcp", "pose": waypoints["OBSERVE_HIGH"]},
        ]},
        "retreat_to_start": {"steps": [{"op": "move_tcp", "pose": points["CARRY"]["pose"]}]},
        "rotate_to_drop": {"steps": [{"op": "move_tcp", "pose": points["DROP"]["pose"]}]},
        "carry_to_drop": {"steps": [{"op": "move_tcp", "pose": points["DROP"]["pose"]}]},
        "drop_to_home": {"steps": [
            {"op": "move_tcp", "pose": points["HOME_END"]["pose"]},
            {"op": "move_tcp", "pose": points["HOME"]["pose"]},
        ]},
        "rotate_back": {"steps": [{"op": "move_tcp", "pose": points["CARRY"]["pose"]}]},
        "regrip_handle": {"steps": [
            {"op": "move_tcp", "pose": points["PULL_END"]["pose"]},
            {"op": "settle", "ms": 200},
            {"op": "grip", "width_m": GRIP_WIDTH_M},
        ]},
        "push_drawer_closed": {"steps": [{"op": "settle", "ms": 300}, *push_steps]},
        "close_to_home": {"steps": [
            {"op": "move_tcp", "pose": points["HOME_END"]["pose"]},
            {"op": "move_tcp", "pose": points["HOME"]["pose"]},
        ]},
    }

    data = {
        "schema": SCHEMA, "setup_id": setup_id, "frame": "base_link",
        "captured_at": datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds"),
        "source": f"teach_drawer_demo session {folder.name}",
        "source_kind": "teaching_candidate",
        "commissioned": False,
        "pull_openings_mm": openings,
        "verified": dict(verified or {}),
        "waypoints": waypoints,
        "waypoint_joints_rad": {
            name: points[point]["raw"]["state"]["right_arm_joints_rad"]
            for name, point in (("HOME", "HOME"), ("OBSERVE_HIGH", top), ("START", "CARRY"),
                                ("DROP", "DROP"), ("HANDLE_CLEAR", "HANDLE_CLEAR"),
                                ("HANDLE_RELEASE", "PULL_END"))
            if isinstance(points[point]["raw"]["state"].get("right_arm_joints_rad"), dict)
        },
        "routes": routes,
    }
    return data, gaps


def write(session_dir, out_path, **kwargs) -> list[str]:
    """生成路线文件并立即用 RouteStore 自检，返回剩余缺口。"""
    data, gaps = build(session_dir, **kwargs)
    Path(out_path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")
    store = RouteStore(json.loads(Path(out_path).read_text(encoding="utf-8")))
    return gaps + [g for g in store.check({"setup_id": data["setup_id"],
                                           "drawer_opening_m": kwargs["drawer_opening_m"]})
                   if "未核验" not in g]
