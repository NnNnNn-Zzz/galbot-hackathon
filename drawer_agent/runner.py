"""装配与一次完整运行。CLI 和测试共用这里，避免两份不同的接线方式。"""

from __future__ import annotations

import json
import math
import uuid
from pathlib import Path

from .framework import load as load_framework
from .io import SharedAgentIO
from .link import MissingCapability, finite, required_capabilities
from .perception import YoloHttpClient
from .routes import RouteStore
from .sim_link import ImageMockPerception, OfflinePerception, OfflineRig

HERE = Path(__file__).resolve().parent
DEFAULT_RECIPE = HERE / "recipe.offline_rig.json"
DEFAULT_ROUTES = HERE / "routes.offline_rig.json"

#: 离线装置可注入的故障 → 说明。这些只影响链路读数，不改状态机。
FAULTS: dict[str, str] = {
    "short_yaw": "底盘只转了指令的 60%，但回执是 SUCCESS",
    "yaw_stuck": "底盘完全没转，回执仍是 SUCCESS",
    "drawer_jam": "拉屉时夹持移动，抽屉实际没动",
    "empty_grasp": "夹爪闭合成功，但没夹到盒",
    "stale_frame": "取图时间戳比请求早 2 秒",
    "no_target": "分割返回 no_target",
    "regrip_miss": "重夹把手时夹空",
    "bin_shift": "桶偏了 120 mm",
    "stroke_short": "抽屉卡在离闭合 20 mm 处",
    "jam_step": "TCP 运动未收到到位回执",
}


def read_json(path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def apply_fault(rig: OfflineRig, name: str) -> None:
    if name not in FAULTS:
        raise ValueError(f"未知故障 {name}，可选：{', '.join(sorted(FAULTS))}")
    if name == "short_yaw":
        rig.inject(name, fraction=0.6)
    elif name == "bin_shift":
        rig.inject(name, dx=0.12)
    else:
        rig.inject(name)


def offline_recipe(*, extended: bool = False) -> dict:
    recipe = read_json(DEFAULT_RECIPE)
    if not extended:
        recipe["flow"] = {"rotate_for_drop": False, "close_drawer": False}
        recipe["base_motion"]["allowed"] = False
        recipe["route_refs"].update(carry_to_drop="carry_to_drop", drop_to_home="drop_to_home")
    return recipe


def validate_site(recipe, routes, fw):
    if (recipe.get("source_kind") != "site" or recipe.get("commissioned") is not True
            or "flow" not in recipe):
        raise ValueError("真机需要显式 flow、source_kind=site、commissioned=true 的现场配方")
    if (routes.data.get("source_kind") != "site" or routes.data.get("commissioned") is not True
            or not isinstance(routes.data.get("evidence_ref"), str)
            or not routes.data["evidence_ref"].strip()):
        raise ValueError("真机需要单独核验并附 evidence_ref 的现场路线")
    if (not isinstance(recipe.get("evidence_ref"), str) or not recipe["evidence_ref"].strip()
            or str(recipe.get("setup_id", "")).upper().startswith(("OFFLINE", "SYNTHETIC", "MOCK"))):
        raise ValueError("配方必须绑定真实场景和验收记录")
    for name in ("max_contact_force_n", "joint_tolerance_rad", "tcp_angle_tolerance_rad"):
        value = recipe["limits"].get(name)
        if not finite(value) or value <= 0:
            raise ValueError(f"现场配方缺少有效限值 {name}")
    if recipe["limits"]["tcp_angle_tolerance_rad"] >= math.pi:
        raise ValueError("TCP 角度容差必须小于 π")
    for point in ("HOME", "OBSERVE_HIGH", "START", "DROP", "HANDLE_CLEAR"):
        joints = routes.joint_target(recipe["waypoint_refs"][point])
        if joints is None:
            raise ValueError(f"现场路点 {point} 缺少命名关节分支")
    gaps = routes.check(recipe)
    if gaps:
        raise ValueError("现场路线未通过校验： " + "；".join(gaps))


def build(recipe_path=None, routes_path=None, *, backend: str = "offline", faults=(),
          perception_url: str | None = None, extended: bool = False, wrapper=None, opener=None,
          image_mock: bool = False):
    fw = load_framework()
    if backend == "robot" and (recipe_path is None or routes_path is None or faults or extended or image_mock):
        raise ValueError("真机必须显式指定现场配方和路线，禁止使用离线选项")
    recipe = read_json(recipe_path) if recipe_path is not None else offline_recipe(extended=extended)
    routes = RouteStore.load(routes_path if routes_path is not None else DEFAULT_ROUTES)
    fw.validate_recipe(recipe, "shared_agent")

    if backend == "offline":
        link = OfflineRig(recipe, routes, tool_id=recipe["tool_id"],
                          tcp_frame=recipe["tcp_frame"])
        for name in faults:
            apply_fault(link, name)
        perception = (ImageMockPerception if image_mock else OfflinePerception)(link)
    elif backend == "robot":
        from .shared_link import SharedAgentLink
        validate_site(recipe, routes, fw)
        link = SharedAgentLink(recipe, wrapper=wrapper)
        caps = link.caps()
        rotate, _ = fw.flow_options(recipe)
        missing = [name for name in required_capabilities(rotate=rotate, live=True)
                   if not caps.get(name)]
        if missing:
            raise MissingCapability(missing)
        if not perception_url:
            raise RuntimeError("真机后端必须给出 --perception-url（YOLO 推理服务地址）")
        perception = YoloHttpClient(perception_url, model_id=recipe["model_id"],
                                    perception_backend=recipe.get("perception_backend"),
                                    weights_sha256=recipe.get("weights_sha256"))
    else:
        raise ValueError(f"未知后端 {backend}（offline | robot）")

    io = SharedAgentIO(link, perception, routes, framework=fw, opener=opener)
    return fw, recipe, io


def run_flow(recipe_path=None, routes_path=None, *,
             backend: str = "offline", faults=(), perception_url: str | None = None,
             run_id: str | None = None, stop_after: str | None = None,
             extended: bool = False, wrapper=None, opener=None, hover_only: bool = False,
             image_mock: bool = False) -> dict:
    if backend == "robot" and wrapper is None:
        raise ValueError("真机动作须由 Agent 同进程注入共享封装")
    fw, recipe, io = build(recipe_path, routes_path, backend=backend,
                           faults=faults, perception_url=perception_url,
                           extended=extended, wrapper=wrapper, opener=opener, image_mock=image_mock)
    plan = fw.plan_for(recipe)
    if hover_only:
        plan = plan[:plan.index("verify_above_box") + 1]
    if stop_after is not None and stop_after not in set(plan) & fw.EVIDENCE.keys():
        raise ValueError(f"当前流程没有可分段停止的动作 {stop_after}")
    io.stop_after = stop_after
    ctx = fw.Context(run_id=run_id or uuid.uuid4().hex, setup_id=recipe["setup_id"])
    result = fw.FixedBoxDemo(recipe, hover_only=hover_only).run(ctx, io)
    return {"status": result.status, "message": result.message, "run_id": ctx.run_id,
            "setup_id": ctx.setup_id, "plan": plan,
            "verified_progress": ctx.flags,
            "data": result.data, "trace": ctx.trace,
            "steps": len(ctx.trace), "adapter": io.report(),
            "backend": backend, "faults": list(faults), "stop_after": stop_after}
