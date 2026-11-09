"""Three bounded drawer experiments. CLI operates on offline fixtures only."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
from importlib import import_module
import json
import math
import re
import uuid
from pathlib import Path

from .framework import load
from .io import DRAWER_TOL_M, SharedAgentIO
from .link import MissingCapability, finite, fresh
from .routes import RouteStore
from .runner import DEFAULT_ROUTES, apply_fault, offline_recipe, read_json, run_flow
from .shared_link import SharedAgentLink
from .perception import YoloHttpClient
from .sim_link import ImageMockPerception, OfflineRig

RETURN_POINTS = ("P0", "P1", "P2", "INITIAL_CARRY")
VISUAL_RETURN_STAGES = ("vision", "retreat")

STAGES = {
    1: {"vision": (), "retreat": ("visual_return",),
        "preview": (), "open": ("open_drawer_release",)},
    2: {
        "hover": ("observe_high", "open_gripper", "align_above_box"),
        "dry": ("observe_high", "open_gripper", "align_above_box", "dry_descend_return"),
        "carry": ("observe_high", "open_gripper", "align_above_box", "descend",
                  "close_gripper", "short_lift", "verify_holding", "retreat_to_start"),
        "drop": ("observe_high", "open_gripper", "align_above_box", "descend",
                 "close_gripper", "short_lift", "verify_holding", "retreat_to_start",
                 "move_to_drop", "release_box", "verify_drop", "return_home"),
    },
    3: {"hover": (), "full": ()},
}


class BenchIO(SharedAgentIO):
    def __init__(self, *args, allowed, **kwargs):
        super().__init__(*args, **kwargs)
        self.allowed = allowed

    def _action_enabled(self, name):
        return name in self.allowed

    def _do_descend(self, params, timeout_s):
        # Known, physically checked test fixture; no fabricated vision timestamp.
        return self._descend_to_pose(params, timeout_s)

    def _do_visual_return(self, params, timeout_s):
        reached = ["P0"]
        samples = [{"waypoint": "P0", "tcp": self._tcp()}]
        for name in RETURN_POINTS[1:]:
            if self._hold()["object"] is not None:
                raise ValueError("实验 1 回程只允许空爪")
            self._move(self.routes.waypoint(name), timeout_s,
                       expected_joints=self.routes.joint_target(name))
            reached.append(name)
            samples.append({"waypoint": name, "tcp": self._tcp()})
        return {"arrived": True, "ordered_return_verified": True,
                "reached": reached, "terminal_pose": reached[-1],
                "pose_samples": samples,
                "target_frame_id": params["target_frame_id"]}

    def _do_dry_descend_return(self, params, timeout_s):
        if len(self._approach) < 2 or self._hold()["object"] is not None:
            raise ValueError("Dry return requires recorded approach and an empty gripper")
        self._descend_to_pose(params, timeout_s)
        for point in reversed(self._approach):
            self._move(point["tcp"], timeout_s, expected_joints=point["joints"])
        self._check_waypoint("HANDLE_CLEAR")
        if self._hold()["object"] is not None:
            raise ValueError("Dry run gripper is no longer empty")
        return {"arrived": True, "dry_path_traced": True, "gripper_empty": True,
                "reversed_approach_points": len(self._approach)}


def _approve(io, recipe, experiment, stage):
    fw, routes = io.fw, io.routes
    visual_return = experiment == 1 and stage in VISUAL_RETURN_STAGES
    read_only = stage in ("preview", "vision")
    if recipe.get("flow") != {"rotate_for_drop": False, "close_drawer": False} or recipe["base_motion"]["allowed"] is not False:
        raise ValueError("Experiments require fixed base and no automatic drawer closure")
    if io.live:
        approval = recipe.get("experiment_approval", {})
        if (recipe.get("source_kind") != "site" or routes.data.get("source_kind") != "site"
                or approval.get("experiment") != experiment or stage not in approval.get("stages", [])
                or not isinstance(approval.get("evidence_ref"), str) or not approval["evidence_ref"].strip()
                or routes.setup_id != recipe.get("setup_id") or routes.data.get("frame") != "base_link"
                or str(recipe.get("setup_id", "")).upper().startswith(("OFFLINE", "SYNTHETIC", "MOCK"))):
            raise ValueError("Missing site approval for this experiment and stage")
        keys = ("max_contact_force_n", "joint_tolerance_rad", "tcp_angle_tolerance_rad",
                "max_linear_speed_m_s", "max_joint_speed_rad_s", "action_timeout_s",
                "max_frame_age_s", "max_receipt_age_s", "gripper_speed_m_s", "gripper_effort_n")
        for key in keys:
            value = recipe["limits"].get(key)
            if not finite(value) or value <= 0:
                raise ValueError(f"Invalid site limit {key}")
        needed = {"rig_status", "arm_state", "base_pose", "drawer_state", "hold_state"}
        if not read_only:
            needed.update(("move_tcp", "set_gripper", "stop"))
        if visual_return:
            needed.discard("set_gripper")
        if experiment == 1:
            needed.add("capture")
        if stage == "drop":
            needed.add("bin_state")
        caps = io.link.caps()
        missing = sorted(n for n in needed if not caps.get(n))
        if missing:
            raise MissingCapability(missing)
        rig = io.link.rig_status()
        facts = ("legs_locked", "head_locked", "base_parked", "exclusive_right_arm", "dock_pose_verified")
        if not read_only:
            facts += ("force_monitor_ready", "independent_stop_ready", "stop_channel")
        if (any(rig.get(k) is not True for k in facts)
                or not fresh(rig.get("stamp_ns"), io.now_ns(), recipe["limits"]["max_receipt_age_s"])
                or rig.get("site_setup_id") != recipe["setup_id"]
                or rig.get("tool_id") != recipe["tool_id"] or rig.get("tcp_frame") != recipe["tcp_frame"]):
            raise ValueError("Site rig, tool, control or stop evidence is incomplete")
        if experiment == 1 and not visual_return and rig.get("wrist_calibration_id") != recipe["drawer_open"].get("calibration_id"):
            raise ValueError("Wrist calibration does not match the site")
        if visual_return and (rig.get("camera_calibration_id") != recipe["calibration"]["id"]
                              or recipe["calibration"]["verified"] is not True
                              or not io.perception.health()):
            raise ValueError("P0 相机标定或视觉服务未就绪")
        if stage == "retreat" and approval.get("visual_return_path_reviewed") is not True:
            raise ValueError("请先验收 P0、P1、P2 至初始抱持位各段路径")
        if stage == "open" and (approval.get("wrist_preview_reviewed") is not True
                                or approval.get("short_pull_verified") is not True):
            raise ValueError("Review wrist preview and verify a supervised short pull before full opening")
        if stage in ("carry", "drop") and (approval.get("dry_path_verified") is not True
                                          or approval.get("loaded_trial_path_reviewed") is not True):
            raise ValueError("Review the empty path and loaded trial path before grasping")
        if stage == "drop" and approval.get("bin_path_reviewed") is not True:
            raise ValueError("Review the bin path before release")
        if stage == "dry" and (approval.get("empty_drawer_verified") is not True
                               or approval.get("dry_path_reviewed") is not True):
            raise ValueError("Verify empty drawer and approve the planned dry path")

    io.recipe = recipe
    io._run_start_base = io._base()
    start = "P0" if visual_return else ("HOME" if experiment == 1 else "HANDLE_CLEAR")
    if experiment == 1 and stage == "preview" and recipe["drawer_open"].get("use_view_route"):
        start = "WRIST_VIEW"
    if io.live:
        io._check_waypoint(start)
    if not io._still():
        raise ValueError("Arm must be stationary at the experiment start")
    drawer = io._drawer()["opening_m"]
    if io._hold()["object"] is not None or io._arm()["gripper"]["width"] < recipe["gripper_open_m"] - 0.005:
        raise ValueError("Start with empty, open gripper")
    if ((experiment == 1 and not visual_return and drawer > 0.003)
            or ((visual_return or experiment == 2) and abs(drawer-recipe["drawer_opening_m"]) > DRAWER_TOL_M)):
        raise ValueError("Incorrect initial drawer opening")
    if visual_return:
        if recipe["waypoint_refs"].get("P0") != "P0":
            raise ValueError("实验 1 的起点必须绑定 P0")
        if stage == "retreat":
            poses = []
            for name in RETURN_POINTS:
                pose = routes.waypoint(name)
                if abs(math.hypot(*pose[3:]) - 1) > 1e-3 or (io.live and routes.joint_target(name) is None):
                    raise ValueError(f"回程点 {name} 缺少有效 TCP 或七轴关节记录")
                poses.append(pose)
            if not poses[0][2] > poses[1][2] > poses[2][2]:
                raise ValueError("P0、P1、P2 的实测 Z 必须与下降回程顺序一致")
        io._ready = True
        return
    if experiment == 1:
        config = recipe["drawer_open"]
        if (config.get("mode") != "wrist_vision" or not config.get("camera_id")
                or not config.get("calibration_id") or not callable(getattr(io.opener, "plan", None))):
            raise ValueError("Wrist planner, camera and calibration are required")
        route_keys = ("wrist_view",) if stage == "open" and config.get("use_view_route") else ()
    else:
        route_keys = ("rise_to_observe",)
        if stage in ("carry", "drop"):
            route_keys += ("retreat_to_start",)
        if stage == "drop":
            route_keys += ("carry_to_drop", "drop_to_home")
        if io.live and not routes.verified("observe_high_max_z"):
            raise ValueError("The highest observation waypoint is unverified")
        if io.live:
            routes.check_observation_endpoint(recipe)
    for key in route_keys:
        name = recipe["route_refs"][key]
        for index, step in enumerate(routes.route(name), 1):
            if routes._step_gaps(name, index, step) or step["op"] not in ("move_tcp", "settle"):
                raise ValueError(f"Invalid arm-only route {name}, step {index}")
    io._ready = True


def _targets(recipe, center):
    offset = recipe["box_center_to_tcp_xy_m"]
    rotation = recipe["tool_orientation_xyzw"]
    high, low, lift = (recipe[k] for k in ("above_tcp_z_m", "grasp_tcp_z_m", "short_lift_m"))
    if (len(center) != 2 or len(offset) != 2 or len(rotation) != 4
            or not finite(*center, *offset, *rotation, high, low, lift)
            or abs(math.hypot(*rotation)-1) > 1e-5 or not 0 < lift < high-low):
        raise ValueError("Invalid measured box/TCP geometry")
    x, y = (a+b for a,b in zip(center, offset))
    return {key: {"tcp_pose": [x, y, z, *rotation]} for key, z in
            (("align_above_box", high), ("descend", low), ("short_lift", low+lift))}


def _run_bench(experiment, stage, recipe, io):
    trace, active = [], "preflight"
    dispatched = False
    timeout = recipe["limits"]["action_timeout_s"]
    try:
        _approve(io, recipe, experiment, stage)
        if experiment == 1 and stage in VISUAL_RETURN_STAGES:
            active = "p0_localization"
            timeout_s = recipe["limits"]["perception_timeout_s"]
            frame = io.capture("OBSERVE_HIGH", timeout_s)
            geometry = import_module(f"{io.fw.module.__name__}.geometry")
            target = geometry.localize(io.segment(frame, timeout_s), frame, recipe, io.now_ns())
            trace.append({"step": active, "status": "ok", "frame_id": frame.frame_id, "target": target})
            if stage == "vision":
                return {"status": "ok", "motion_dispatched": False, "trace": trace,
                        "terminal_stage": active}
            active = "visual_return"
            action = io.fw.Action(active, {"target_frame_id": frame.frame_id},
                                  ("ordered_return_verified",), uuid.uuid4().hex)
            dispatched = True
            receipt = io.execute(action, timeout)
            trace.append({"step": active, **asdict(receipt)})
            if receipt.status != "ok" or receipt.action_id != action.request_id:
                raise RuntimeError(receipt.message or "视觉回程未完成")
            return {"status": "ok", "motion_dispatched": True, "trace": trace,
                    "data": {"reached": receipt.evidence["reached"],
                             "terminal_pose": receipt.evidence["terminal_pose"]}}
        if stage == "preview":
            active = "preview"
            io._deadline = io.now_ns() + int(timeout*1e9)
            steps, frame = io.plan_wrist(recipe["drawer_open"], recipe["drawer_opening_m"], timeout)
            return {"status": "ok", "motion_dispatched": False, "frame": asdict(frame), "plan": steps}
        r, w = recipe["route_refs"], recipe["waypoint_refs"]
        params = {
            "open_drawer_release": {"target_opening_m": recipe["drawer_opening_m"]},
            "observe_high": {"route_ref": r.get("rise_to_observe"), "waypoint_ref": w.get("OBSERVE_HIGH")},
            "open_gripper": {"width_m": recipe["gripper_open_m"]},
            "retreat_to_start": {"route_ref": r.get("retreat_to_start"), "waypoint_ref": w.get("START")},
            "move_to_drop": {"route_ref": r.get("carry_to_drop"), "waypoint_ref": w.get("DROP")},
            "release_box": {"release_mode": "drop", "width_m": recipe["gripper_open_m"]},
            "return_home": {"route_ref": r.get("drop_to_home"), "waypoint_ref": w.get("HOME")},
        }
        if experiment == 2:
            known = recipe["known_box"]
            if io.live and (not known.get("evidence_ref")
                            or (stage != "dry" and known.get("placement_verified") is not True)):
                raise ValueError("Verify the measured test box placement this run")
            params.update(_targets(recipe, known["center_xy_m"]))
            params["close_gripper"] = {"width_m": recipe["gripper_close_m"]}
            params["dry_descend_return"] = params["descend"]
        for active in STAGES[experiment][stage]:
            required = (("dry_path_traced", "gripper_empty") if active == "dry_descend_return"
                        else io.fw.EVIDENCE[active])
            request = io.fw.Action(active, params.get(active, {}), required, uuid.uuid4().hex)
            dispatched = True
            receipt = io.execute(request, timeout)
            trace.append({"step": active, **asdict(receipt)})
            if receipt.status != "ok" or receipt.action_id != request.request_id:
                raise RuntimeError(receipt.message or "Action did not complete")
        return {"status": "ok", "motion_dispatched": dispatched, "trace": trace,
                "terminal_stage": active}
    except (Exception, KeyboardInterrupt) as exc:
        result = {"status": "unknown" if dispatched else "failed", "failed_step": active,
                  "message": str(exc), "motion_dispatched": dispatched, "trace": trace}
        if dispatched:
            result["stop"] = io.stop(str(exc))
        return result


def _digest(data):
    canonical = json.dumps(data, sort_keys=True, allow_nan=False, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _experiment_3_metadata(stage, recipe, routes):
    approval = recipe.get("experiment_approval", {})
    model = recipe.get("model_id")
    backend = recipe.get("perception_backend")
    weights = recipe.get("weights_sha256")
    if (recipe.get("flow") != {"rotate_for_drop": False, "close_drawer": False}
            or recipe.get("drawer_open", {}).get("mode") != "wrist_vision"
            or recipe.get("waypoint_refs", {}).get("OBSERVE_HIGH") != "P0"):
        raise ValueError("实验 3 需要腕部开屉、固定底盘、P0 观察点")
    if not isinstance(model, str) or not model.strip():
        raise ValueError("实验 3 缺少模型编号 model_id")
    if backend == "bench_hsv":
        if not model.startswith("bench_top_mask_") or weights is not None:
            raise ValueError("临时色块服务需要 bench_top_mask_ 模型编号，权重摘要留空")
    elif backend == "yolo":
        if model.startswith("bench_top_mask_") or not isinstance(weights, str) or not re.fullmatch("[0-9a-f]{64}", weights):
            raise ValueError("YOLO 需要正式模型编号和权重文件的 SHA-256 摘要")
    else:
        raise ValueError("perception_backend 必须是 bench_hsv 或 yolo")
    if (approval.get("experiment") != 3 or stage not in approval.get("stages", [])
            or not all(isinstance(approval.get(k), str) and approval[k].strip() for k in (
                "experiment_1_evidence_ref", "experiment_2_evidence_ref",
                "wrist_open_evidence_ref", "vision_validation_ref"))):
        raise ValueError("实验 3 需要实验 1、2、腕部开屉和本模型两视角定位的现场验收")
    metadata = {
        "setup_id": recipe["setup_id"],
        "perception_backend": backend,
        "model_id": model,
        "weights_sha256": weights,
        "routes_sha256": _digest(routes.data),
        "configuration_sha256": _digest({k: v for k, v in recipe.items() if k != "experiment_approval"}),
    }
    if backend == "yolo":
        _require_prior_report(approval.get("bench_full_report"), {
            "experiment": 3, "stage": "full", "setup_id": metadata["setup_id"],
            "perception_backend": "bench_hsv", "routes_sha256": metadata["routes_sha256"],
        }, "同一现场和路线的临时分割整链", {"box_in_bin": True})
    if stage == "full":
        _require_prior_report(approval.get("hover_report"), {
            "experiment": 3, "stage": "hover", **metadata,
        }, "当前模型和配置的真机悬停复拍", {
            "checkpoint": "verify_above_box", "terminal_pose": "ABOVE_BOX", "full_cycle": False,
        })
    return metadata


def _require_prior_report(path, expected, label, evidence):
    if not isinstance(path, str) or not path.strip():
        raise ValueError(f"缺少{label}报告")
    report = read_json(path)
    expected = {"status": "ok", "backend": "robot", **expected}
    if (not isinstance(report, dict)
            or any(report.get(key) != value for key, value in expected.items())
            or not isinstance(report.get("data"), dict)
            or any(report["data"].get(key) != value for key, value in evidence.items())):
        raise ValueError(f"{label}报告未通过或与当前配置不符")


def run_site(experiment, stage, *, recipe_path, routes_path, wrapper, opener=None,
             perception_url=None, report_path=None):
    """Call from the running shared Agent. An optional report is created once."""
    if wrapper is None:
        raise ValueError("A shared Agent wrapper must be injected")
    allowed = STAGES[experiment][stage]
    recipe, routes = read_json(recipe_path), RouteStore.load(routes_path)
    metadata = {"setup_id": recipe["setup_id"]}
    perception = None
    if experiment == 1 and stage in VISUAL_RETURN_STAGES:
        if recipe.get("perception_backend") != "bench_hsv" or not perception_url:
            raise ValueError("实验 1 需要现场临时视觉服务")
        perception = YoloHttpClient(perception_url, model_id=recipe["model_id"],
                                    perception_backend="bench_hsv")
    if experiment == 3:
        metadata = _experiment_3_metadata(stage, recipe, routes)
    report = Path(report_path).open("x", encoding="utf-8") if report_path is not None else None
    try:
        if experiment == 3:
            result = run_flow(recipe_path, routes_path, backend="robot", wrapper=wrapper,
                              opener=opener, perception_url=perception_url, hover_only=stage == "hover")
        else:
            io = BenchIO(SharedAgentLink(recipe, wrapper=wrapper), perception, routes, opener=opener, allowed=allowed)
            result = _run_bench(experiment, stage, recipe, io)
        result.update(metadata, backend="robot", experiment=experiment, stage=stage)
        if report is not None:
            json.dump(result, report, ensure_ascii=False, indent=2, allow_nan=False)
            report.write("\n")
        return result
    finally:
        if report is not None:
            report.close()


def mock(experiment, stage, *, faults=()):
    if experiment == 3:
        return run_flow(hover_only=stage == "hover", faults=faults, image_mock=True)
    recipe, routes = offline_recipe(), RouteStore.load(DEFAULT_ROUTES)
    steps = routes.route("drawer_open_release")
    routes.data["waypoints"]["HANDLE_CLEAR"] = steps[-1]["pose"]
    recipe["drawer_open"] = {"mode": "wrist_vision", "camera_id": "mock_wrist",
                             "calibration_id": recipe["calibration"]["id"]}
    recipe["known_box"] = {"center_xy_m": [0.52, 0.0]}
    class Planner:
        def plan(self, request):
            return steps
    rig = OfflineRig(recipe, routes)
    for name in faults:
        apply_fault(rig, name)
    if experiment == 1 and stage in VISUAL_RETURN_STAGES:
        recipe["waypoint_refs"]["P0"] = "P0"
        pose = routes.waypoint("OBSERVE_HIGH")
        for name, dx, dz in zip(RETURN_POINTS, (0, -.04, -.07, -.11), (0, -.03, -.06, -.10)):
            routes.data["waypoints"][name] = [pose[0]+dx, pose[1], pose[2]+dz, *pose[3:]]
        rig.tcp = routes.waypoint("P0")
        rig.drawer_opening = recipe["drawer_opening_m"]
    if experiment == 2:
        rig.drawer_opening = recipe["drawer_opening_m"]
        rig.tcp = routes.waypoint("HANDLE_CLEAR")
        if stage == "dry":
            rig.box["where"] = "removed"
    io = BenchIO(rig, ImageMockPerception(rig), routes, framework=load(), opener=Planner(), allowed=STAGES[experiment][stage])
    return _run_bench(experiment, stage, recipe, io)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=int, choices=STAGES)
    parser.add_argument("mode", choices=("plan", "mock"))
    parser.add_argument("--stage")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    stages = STAGES[args.experiment]
    stage = args.stage or next(iter(stages))
    if stage not in stages:
        parser.error(f"Use one of: {', '.join(stages)}")
    result = ({"status": "ok", "stage": stage, "actions": list(stages[stage]), "stages": list(stages)}
              if args.mode == "plan" else mock(args.experiment, stage))
    if args.mode == "plan" and args.experiment == 3:
        plan = load().plan_for(offline_recipe())
        result["actions"] = plan[:plan.index("verify_above_box")+1] if stage == "hover" else plan
    body = json.dumps({**result, "offline": True}, ensure_ascii=False, indent=2, allow_nan=False)
    if args.out:
        args.out.write_text(body+"\n", encoding="utf-8")
    print(body)
    return 0 if result["status"] == "ok" else 2


if __name__ == "__main__":
    raise SystemExit(main())
