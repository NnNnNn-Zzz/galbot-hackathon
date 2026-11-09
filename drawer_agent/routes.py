"""示教线路库：把示教得到的路点与分段动作读成可校验的数据。

路线是数据不是代码 —— 现场重录一次不该改 Python。格式 `drawer_routes/1`：

    {"schema": "drawer_routes/1", "setup_id": "...", "frame": "base_link",
     "verified": {"observe_high_max_z": false, ...},
     "waypoints": {"HOME": [x,y,z,qx,qy,qz,qw], ...},
     "routes": {"drawer_open_release": {"steps": [{"op": "move_tcp", "pose": [...]}, ...]}}}

`check()` 只做能离线判定的检查（字段齐全、位姿有限、四元数归一、拉出方向为负、
行程与配方一致）。它不判断"这条路线会不会撞"——那是规划器在真机上的事。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

SCHEMA = "drawer_routes/1"
FRAME = "base_link"

WAYPOINTS = ("HOME", "HANDLE_CLEAR", "OBSERVE_HIGH", "START", "DROP", "HANDLE_RELEASE")
ROUTES = ("drawer_open_release", "rise_to_observe", "retreat_to_start", "rotate_to_drop",
          "rotate_back", "regrip_handle", "push_drawer_closed", "close_to_home")
VERIFIED_KEYS = ("observe_high_max_z", "bin_fixed", "base_yaw_free_path")

OPS = ("move_tcp", "move_rel", "grip", "release", "settle")


class RouteError(ValueError):
    """路线数据不可用。属于配置错误，运动前失败。"""


def _finite_seq(value, n):
    return (isinstance(value, (list, tuple)) and len(value) == n
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in value))


class RouteStore:
    def __init__(self, data: dict[str, Any]):
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise RouteError(f"路线 schema 必须是 {SCHEMA}")
        self.data = data

    @classmethod
    def load(cls, path: str | Path) -> "RouteStore":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    # ------------------------------------------------------------------ 读取
    @property
    def setup_id(self) -> str | None:
        return self.data.get("setup_id")

    def waypoint(self, name: str) -> list[float]:
        try:
            pose = self.data["waypoints"][name]
        except KeyError:
            raise RouteError(f"缺少示教路点 {name}") from None
        if not _finite_seq(pose, 7):
            raise RouteError(f"路点 {name} 不是 7 维有限位姿")
        return [float(v) for v in pose]

    def route(self, name: str) -> list[dict]:
        try:
            steps = self.data["routes"][name]["steps"]
        except KeyError:
            raise RouteError(f"缺少示教路线 {name}") from None
        if not isinstance(steps, list) or not steps:
            raise RouteError(f"路线 {name} 没有步骤")
        return steps

    def verified(self, key: str) -> bool:
        return self.data.get("verified", {}).get(key) is True

    def joint_target(self, waypoint: str) -> dict[str, float] | None:
        value = self.data.get("waypoint_joints_rad", {}).get(waypoint)
        if (not isinstance(value, dict) or len(value) != 7
                or not all(isinstance(name, str) and name for name in value)
                or not _finite_seq(list(value.values()), 7)):
            return None
        return value

    def grip_width(self, route: str, key: str = "grip") -> float | None:
        for step in self.route(route):
            if step.get("op") == key:
                return float(step["width_m"])
        return None

    def open_plan_gaps(self, steps) -> list[str]:
        if not isinstance(steps, list) or not 5 <= len(steps) <= 60:
            return ["开屉计划须包含 5–60 个路线动作"]
        gaps = [gap for index, step in enumerate(steps, 1)
                for gap in self._step_gaps("开屉计划", index, step)]
        if gaps:
            return gaps
        ops = [s["op"] for s in steps if s["op"] != "settle"]
        if ops.count("grip") != 1 or ops.count("release") != 1:
            return ["开屉计划须恰好夹持和释放各一次"]
        grip, release = ops.index("grip"), ops.index("release")
        if (grip == 0 or release <= grip + 1 or release == len(ops)-1
                or any(op != "move_tcp" for op in ops[:grip] + ops[release+1:])
                or any(op != "move_rel" for op in ops[grip+1:release])
                or any(s["dx"] >= 0 for s in steps if s["op"] == "move_rel")):
            return ["开屉计划顺序应为接近、夹持、负 X 分段拉动、松爪、撤离，检查步骤或方向反了"]
        return []

    def check_observation_endpoint(self, recipe: dict) -> None:
        """Fail before motion if the rise route does not actually end at the bound waypoint."""
        waypoint = recipe["waypoint_refs"]["OBSERVE_HIGH"]
        target = self.waypoint(waypoint)
        route = recipe["route_refs"]["rise_to_observe"]
        steps = self.route(route)
        poses = [step.get("pose") for step in steps if step.get("op") == "move_tcp"]
        if (any(step.get("op") not in ("move_tcp", "settle") for step in steps)
                or not poses or not _finite_seq(poses[-1], 7)):
            raise RouteError(f"抬升路线 {route} 缺少可核对的末端位姿")
        final = poses[-1]
        angle_matches = min(math.dist(final[3:], target[3:]), math.dist(final[3:], [-q for q in target[3:]])) <= 1e-6
        if math.dist(final[:3], target[:3]) > 1e-6 or not angle_matches:
            raise RouteError(f"抬升路线 {route} 的终点与观察点 {waypoint} 不一致")
        if self.joint_target(waypoint) is None:
            raise RouteError(f"观察点 {waypoint} 缺少七轴命名关节")

    # ------------------------------------------------------------------ 校验
    def check(self, recipe: dict) -> list[str]:
        """返回缺口列表；空列表才允许把 routes_verified 置真。"""
        gaps: list[str] = []
        if self.data.get("frame") != FRAME:
            gaps.append(f"路线坐标系必须是 {FRAME}")
        if self.setup_id != recipe.get("setup_id"):
            gaps.append("路线 setup_id 与配方不一致")

        from .framework import load
        fw = load()
        rotate, close = fw.flow_options(recipe)
        waypoint_refs = recipe.get("waypoint_refs", {})
        route_refs = recipe.get("route_refs", {})
        for key in WAYPOINTS:
            name = waypoint_refs.get(key, key)
            try:
                pose = self.waypoint(name)
            except RouteError as exc:
                gaps.append(str(exc))
                continue
            q = pose[3:]
            if abs(math.sqrt(sum(v * v for v in q)) - 1.0) > 1e-3:
                gaps.append(f"路点 {name} 四元数未归一")

        for key in fw.route_keys(recipe):
            name = route_refs.get(key, key)
            try:
                steps = self.route(name)
            except RouteError as exc:
                gaps.append(str(exc))
                continue
            for index, step in enumerate(steps, 1):
                gaps.extend(self._step_gaps(name, index, step))
            if key == "drawer_open_release":
                gaps.extend(self.open_plan_gaps(steps))
            if key not in ("drawer_open_release", "regrip_handle", "push_drawer_closed"):
                if any(isinstance(s, dict) and s.get("op") in ("grip", "release") for s in steps):
                    gaps.append(f"路线 {name} 只能移动，不能改变夹持")
            if key == "push_drawer_closed" and any(
                    not isinstance(s, dict) or s.get("op") not in ("move_rel", "settle") for s in steps):
                gaps.append(f"路线 {name} 只能使用已示教的分段接触位移")
        if not gaps and recipe.get("drawer_open", {}).get("mode") != "wrist_vision":
            gaps.extend(self._stroke_gaps(recipe))
        if close and "flow" in recipe and not self.verified("push_route_verified"):
            gaps.append("现场未核验独立推回路线 push_route_verified")
        keys = VERIFIED_KEYS if rotate else ("observe_high_max_z", "bin_fixed")
        if self.data.get("source_kind") == "site":
            keys += ("loaded_reverse_verified",)
            if rotate:
                keys += ("base_yaw_return_free_path",)
            if not gaps:
                try:
                    self.check_observation_endpoint(recipe)
                except RouteError as exc:
                    gaps.append(str(exc))
        for key in keys:
            if not self.verified(key):
                gaps.append(f"现场未核验 {key}")
        return gaps

    def _step_gaps(self, route: str, index: int, step: dict) -> list[str]:
        where = f"{route} 第 {index} 步"
        if not isinstance(step, dict):
            return [f"{where} 动作格式非法"]
        op = step.get("op")
        if op not in OPS:
            return [f"{where} 未知操作 {op}"]
        if op == "move_tcp":
            pose = step.get("pose")
            if not _finite_seq(pose, 7) or abs(math.hypot(*pose[3:]) - 1) > 1e-3:
                return [f"{where} 位姿或四元数非法"]
        if op == "move_rel":
            delta = [step.get(k) for k in ("dx", "dy", "dz")]
            if not _finite_seq(delta, 3):
                return [f"{where} 相对位移非法"]
            if math.hypot(*delta) > 0.06:
                return [f"{where} 单步位移过大"]
        if op in ("grip", "release"):
            if not _finite_seq([step.get("width_m")], 1) or step["width_m"] < 0:
                return [f"{where} 夹爪宽度非法"]
        if op == "settle" and (not _finite_seq([step.get("ms")], 1) or not 0 <= step["ms"] <= 2000):
            return [f"{where} 等待时间非法"]
        return []

    def _stroke_gaps(self, recipe: dict) -> list[str]:
        """拉出方向的净位移应等于抽屉行程。方向反了会推抽屉而不是拉。"""
        try:
            pull = self.route(recipe.get("route_refs", {}).get("drawer_open_release", "drawer_open_release"))
        except RouteError:
            return []
        strokes = [s["dx"] for s in pull if s["op"] == "move_rel"]
        net = sum(strokes)
        opening = recipe.get("drawer_opening_m")
        if not isinstance(opening, (int, float)):
            return ["配方缺 drawer_opening_m，无法核对拉出行程"]
        if not strokes or any(dx >= 0 for dx in strokes):
            return [f"拉屉路线净位移 {net:+.3f} m 非负：方向反了会推不会拉"]
        if self.data.get("source_kind") == "site":
            measured = self.data.get("pull_openings_mm")
            if (not isinstance(measured, list) or len(measured) != len(strokes) + 1
                    or not _finite_seq(measured, len(measured))
                    or not 0 <= measured[0] <= 3
                    or any(b <= a for a, b in zip(measured, measured[1:]))
                    or abs(measured[-1] / 1000 - opening) > 0.01):
                return ["缺少与各分段匹配、独立测量且单调增加的抽屉开度记录"]
        elif abs(abs(net) - opening) > 0.01:
            return [f"拉屉路线净位移 {abs(net):.3f} m 与抽屉行程 {opening:.3f} m 不符"]
        return []

    # ------------------------------------------------------------------ 摘要
    def summary(self) -> dict:
        return {"setup_id": self.setup_id, "frame": self.data.get("frame"),
                "waypoints": sorted(self.data.get("waypoints", {})),
                "routes": {name: len(self.data["routes"][name].get("steps", []))
                           for name in sorted(self.data.get("routes", {}))},
                "verified": {k: self.verified(k) for k in VERIFIED_KEYS}}
