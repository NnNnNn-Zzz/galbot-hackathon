"""`DemoIO` 的共享 Agent 实现：把 20 步状态机翻译成 SDK 级动作与证据。

设计上的三条硬规矩，都来自现场已经踩过的坑：

1. **不信命令回执，只信新读数。** 底盘 `set_base_pose` 返回 SUCCESS 却零位移
   （2026-11-07 实测），夹爪闭合成功也可能什么都没夹到。每个 action 结束后
   重新读里程计/夹爪/抽屉，证据只从读数算。
2. **不在适配器里重试。** 失败即停、锁存，等操作员。重发已失败的运动命令
   在现场是二次伤害，`sdk_wrapper` 的 `BASE_POSE_RETRIES` 不适用于本层。
3. **缺能力就在运动前拒。** P0 能力缺失在 preflight 抛错；P1 缺失只在用到的
   那一段失败，并且明确说"停在这里，不要宣称完成"。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from .framework import load as load_framework
from .link import LinkError, fresh, finite, norm_quat, required_capabilities
from .wrist_opener import WristOpenRequest

DRAWER_TOL_M = 0.012
TCP_TOL_M = 0.006
BASE_XY_TOL_M = 0.006
BASE_YAW_TOL_RAD = math.radians(0.6)
STILL_TCP_M = 0.0015


class ActionFailed(RuntimeError):
    """可判定的失败：动作没做成，且未完成。运行时据此锁存停止。"""


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class SharedAgentIO:
    """一次运行一个实例。不要跨 run 复用：内部锁存了停止状态。"""

    backend = "shared_agent"

    def __init__(self, link, perception, routes, *, framework=None, opener=None):
        self.fw = framework or load_framework()
        self.link = link
        self.perception = perception
        self.routes = routes
        self.opener = opener
        self.recipe: dict | None = None
        self._ready = False
        self._latched: str | None = None
        self._busy = False
        self._state_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._stop_result = None
        self._deadline = 0
        self._run_start_base: dict | None = None
        self._grasp_z: float | None = None
        self._drop_checked_ns: int | None = None
        self._approach: list[dict] = []
        self._record_approach = False
        self.stop_after: str | None = None
        self._log: list[dict] = []

    # ------------------------------------------------------------------ 契约
    def now_ns(self) -> int:
        return self.link.clock_ns()

    @property
    def live(self) -> bool:
        return self.link.kind == "shared_agent"

    def preflight(self, recipe: dict) -> dict:
        self._ready = False
        if self.live:
            from .runner import validate_site
            self.fw.validate_recipe(recipe, self.backend)
            validate_site(recipe, self.routes, self.fw)
        self.recipe = recipe
        caps = self.link.caps()
        rotate, _ = self.fw.flow_options(recipe)
        missing = [name for name in required_capabilities(rotate=rotate, live=self.live)
                   if not caps.get(name)]
        if missing:
            raise LinkError("共享 Agent 未提供 P0 能力：" + "、".join(missing))

        rig = self.link.rig_status()
        arm = self._arm()
        base = self._base()
        self._run_start_base = {"x": base["x"], "y": base["y"], "yaw": base["yaw"]}
        now = self.now_ns()
        limit = recipe["limits"]["max_frame_age_s"]

        gaps = self.routes.check(recipe)
        cal = recipe["calibration"]
        facts = {
            "shared_control_ready": rig.get("exclusive_right_arm") is True,
            "stop_channel_ready": rig.get("stop_channel") is True and caps.get("stop") is True,
            "telemetry_fresh": (fresh(arm.get("stamp_ns"), now, limit)
                                and fresh(base.get("stamp_ns"), now, limit)),
            "legs_head_locked_and_base_parked": (rig.get("legs_locked") is True
                                                 and rig.get("head_locked") is True
                                                 and rig.get("base_parked") is True),
            "tool_tcp_verified": (rig.get("tool_id") == recipe["tool_id"]
                                  and rig.get("tcp_frame") == recipe["tcp_frame"]),
            "camera_calibration_verified": (rig.get("camera_calibration_id") == cal["id"]),
            "routes_verified": not gaps,
            "observe_high_is_verified_max_z": self.routes.verified("observe_high_max_z"),
            "bin_fixed_verified": self.routes.verified("bin_fixed"),
            "chassis_pose_control_owned": rig.get("controller_owned") is True,
            "base_yaw_free_path_verified": self.routes.verified("base_yaw_free_path"),
        }
        if self.live:
            try:
                self._check_waypoint("HOME", arm)
                at_home = True
            except ActionFailed:
                at_home = False
            facts.update(
                start_pose_verified=at_home and rig.get("site_setup_id") == recipe["setup_id"]
                and rig.get("dock_pose_verified") is True,
                drawer_initial_state_verified=self._drawer()["opening_m"] <= 0.003
                and self._hold()["object"] is None
                and not arm["gripper"]["is_moving"]
                and arm["gripper"]["width"] >= recipe["gripper_open_m"] - 0.005,
                contact_monitor_ready=rig.get("force_monitor_ready") is True,
                perception_ready=self.perception.health() is True,
                stop_channel_ready=facts["stop_channel_ready"]
                and rig.get("independent_stop_ready") is True,
                telemetry_fresh=facts["telemetry_fresh"]
                and fresh(rig.get("stamp_ns"), self.now_ns(), limit),
            )
        facts["missing_capabilities"] = missing
        facts["route_gaps"] = gaps
        facts["backend_kind"] = getattr(self.link, "kind", "unknown")
        facts["tcp_z_m"] = arm["tcp"][2]
        opening = recipe.get("drawer_open", {})
        if opening.get("mode") == "wrist_vision":
            facts["wrist_opener_ready"] = callable(getattr(self.opener, "plan", None))
            if self.live:
                facts["wrist_opener_ready"] &= (
                    rig.get("wrist_calibration_id") == opening["calibration_id"])
        self._ready = all(facts.get(key) is True for key in self.fw.preflight_keys(recipe))
        return facts

    def capture(self, viewpoint: str, timeout_s: float) -> Any:
        if viewpoint not in ("OBSERVE_HIGH", "ABOVE_BOX"):
            raise ActionFailed(f"未知观察位 {viewpoint}")
        requested = self.now_ns()
        cal = self.recipe["calibration"]
        info = self.link.capture(cal["camera_id"], timeout_s)
        if info.get("calibration_id") != cal["id"]:
            raise ActionFailed("取图回报的标定 id 与配方不符")
        if list(info.get("size_wh") or []) != list(cal["image_size_wh"]):
            raise ActionFailed("取图尺寸与标定不符")
        stamp = info.get("stamp_ns")
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < requested:
            raise ActionFailed("取图时间戳不是本次请求之后的采样")
        return self.fw.Frame(info["frame_id"], info["camera_id"], stamp,
                             tuple(info["size_wh"]), info["image_ref"], cal["id"],
                             self.recipe["setup_id"])

    def segment(self, frame, timeout_s: float) -> dict:
        payload = self.perception.segment(frame, timeout_s) \
            if _takes_timeout(self.perception.segment) else self.perception.segment(frame)
        for key in ("frame_id", "camera_id", "capture_timestamp_ns", "image_size_wh"):
            value = payload.get(key)
            expected = getattr(frame, key)
            if key == "image_size_wh":
                value, expected = list(value or []), list(expected)
            if value != expected:
                raise ActionFailed(f"分割结果与帧不对应：{key}")
        if payload.get("model_id") != self.recipe["model_id"]:
            raise ActionFailed("分割结果 model_id 与配方不符")
        return payload

    def _action_enabled(self, name: str) -> bool:
        return name in self.fw.plan_for(self.recipe)

    def execute(self, action, timeout_s: float):
        with self._state_lock:
            if self._latched is not None:
                return self._receipt(action, "failed", f"已锁存停止（{self._latched}），不再下发动作")
            if self._busy:
                return self._receipt(action, "failed", "适配器忙：同一时刻只允许一个动作")
            if not self._ready:
                return self._receipt(action, "failed", "预检未通过")
            if not self._action_enabled(action.name):
                return self._receipt(action, "failed", "动作未获当前流程授权")
            if not finite(timeout_s) or timeout_s <= 0:
                return self._receipt(action, "failed", "动作时限非法")
            self._busy = True
            started = self.now_ns()
            self._deadline = started + int(timeout_s * 1e9)
            deadline = self._deadline
        timer = threading.Timer(timeout_s, self._watchdog, args=(action.name, deadline))
        timer.daemon = True
        timer.start()
        try:
            handler = getattr(self, "_do_" + action.name, None)
            if handler is None:
                raise ActionFailed(f"适配器未实现动作 {action.name}")
            before = self._snapshot()
            facts = handler(action.params or {}, timeout_s) or {}
            after = self._snapshot()
            self._check_deadline()
            if not self._unchanged(action.name, before, after):
                raise ActionFailed(f"{action.name} 期间底盘/抽屉发生了未授权变化")
            evidence = {"scene_unchanged": True,
                        "path_checked": True,
                        "arrived": facts.pop("arrived", None) is True,
                        "backend_kind": getattr(self.link, "kind", "unknown")}
            evidence.update(facts)
            for fact in action.required:
                if evidence.get(fact) is not True:
                    raise ActionFailed(f"{action.name} 缺少物证：{fact}")
            self._log.append({"action": action.name, "status": "ok",
                              "elapsed_ms": round((self.now_ns() - started) / 1e6, 1),
                              "evidence": _brief(evidence)})
            with self._state_lock:
                self._check_deadline()
                self._deadline = 0
                if self.stop_after == action.name:
                    self._latched = f"staged stop after {action.name}"
            return self._receipt(action, "ok", "", evidence)
        except ActionFailed as exc:
            self._log.append({"action": action.name, "status": "failed", "reason": str(exc)})
            return self._receipt(action, "failed", str(exc))
        except TimeoutError as exc:
            self.stop(str(exc))
            self._log.append({"action": action.name, "status": "unknown", "reason": str(exc)})
            return self._receipt(action, "unknown", f"未收到到位回执：{exc}")
        except (Exception, KeyboardInterrupt) as exc:
            self.stop(str(exc))
            self._log.append({"action": action.name, "status": "unknown",
                              "reason": f"{type(exc).__name__}: {exc}"})
            return self._receipt(action, "unknown", f"{type(exc).__name__}: {exc}")
        finally:
            timer.cancel()
            with self._state_lock:
                self._busy = False
                self._deadline = 0

    def _watchdog(self, action: str, deadline: int) -> None:
        with self._state_lock:
            if not self._busy or self._deadline != deadline:
                return
            self._latched = self._latched or f"{action} 超时"
        self.stop(self._latched)

    def stop(self, reason: str) -> dict:
        with self._state_lock:
            self._latched = self._latched or str(reason)
        # Do not wait behind another stop call. Its confirmation is not known yet.
        if not self._stop_lock.acquire(blocking=False):
            return {"confirmed": False, "reason": self._latched, "stop_in_progress": True}
        try:
            if self._stop_result is not None:
                return self._stop_result
            chains = {}
            for chain in ("right_arm", "base"):
                try:
                    chains[chain] = self.link.stop(chain)
                except Exception as exc:  # noqa: BLE001
                    chains[chain] = {"accepted": False, "completed": False, "error": str(exc)}
            now = self.now_ns()
            confirmed = all(isinstance(v, dict) and v.get("accepted") is True
                            and v.get("completed") is True and fresh(v.get("stamp_ns"), now, 1.0)
                            for v in chains.values())
            self._stop_result = {"confirmed": confirmed, "reason": self._latched, "chains": chains,
                                 "backend_kind": self.link.kind}
            return self._stop_result
        finally:
            self._stop_lock.release()

    # ------------------------------------------------------------------ 动作
    def _do_open_drawer_release(self, params: dict, timeout_s: float) -> dict:
        target = float(params["target_opening_m"])
        config = self.recipe.get("drawer_open", {})
        if config.get("mode") == "wrist_vision":
            self._open_with_wrist(config, target, timeout_s)
        else:
            self._replay(params["route_ref"], timeout_s, drawer_direction=1)
        opening = self._drawer()["opening_m"]
        if abs(opening - target) > DRAWER_TOL_M:
            raise ActionFailed(f"抽屉实测开度 {opening * 1000:.0f} mm，"
                               f"未达目标 {target * 1000:.0f} mm（不认 TCP 位移）")
        hold = self._hold()
        if hold.get("object") is not None:
            raise ActionFailed("松手后夹爪仍报告持有物体")
        handle = self.routes.waypoint(self.recipe["waypoint_refs"]["HANDLE_RELEASE"])
        tcp = self._tcp()
        clear = tcp[0] <= handle[0] - 0.02 and tcp[2] >= handle[2] + 0.04
        if not clear:
            raise ActionFailed("手臂尚未撤离抽屉把手区域")
        if self.live or config.get("mode") == "wrist_vision":
            self._check_waypoint("HANDLE_CLEAR")
        drawer = self._drawer()
        return {"drawer_open_verified": True, "handle_released": True,
                "arm_clear_of_drawer": True, "opening_m": opening,
                "opening_uncertainty_m": drawer["uncertainty_m"],
                "handle_clear_tcp": tcp, "setup_id": self.recipe["setup_id"],
                "opening_mode": config.get("mode", "taught"), "arrived": True}

    def _open_with_wrist(self, config: dict, target: float, timeout_s: float) -> None:
        if config.get("use_view_route"):
            self._replay(self.recipe["route_refs"]["wrist_view"], timeout_s)
        steps, _ = self.plan_wrist(config, target, timeout_s)
        self._replay_steps(steps, timeout_s, drawer_direction=1)

    def plan_wrist(self, config: dict, target: float, timeout_s: float):
        """Capture and validate a wrist plan at the current pose, without moving."""
        if not self._still():
            raise ActionFailed("腕部取图前手臂未停稳")
        requested = self.now_ns()
        info = self.link.capture(config["camera_id"], min(timeout_s, self._remaining_s()))
        stamp, size = info.get("stamp_ns"), info.get("size_wh")
        if (info.get("camera_id") != config["camera_id"]
                or info.get("calibration_id") != config["calibration_id"]
                or not fresh(stamp, self.now_ns(), self.recipe["limits"]["max_frame_age_s"])
                or stamp < requested
                or not isinstance(size, (list, tuple)) or len(size) != 2
                or not all(type(v) is int and v > 0 for v in size)
                or not all(isinstance(info.get(k), str) and info[k] for k in ("image_ref", "frame_id"))):
            raise ActionFailed("腕部图像不新鲜或与本轮相机标定不符")
        frame = self.fw.Frame(info["frame_id"], config["camera_id"], stamp, tuple(size),
                              info["image_ref"], config["calibration_id"], self.recipe["setup_id"])
        ref = self.recipe["waypoint_refs"]
        request = WristOpenRequest(self.recipe["setup_id"], frame, target, tuple(self._tcp()),
                                   tuple(self.routes.waypoint(ref["HANDLE_RELEASE"])),
                                   tuple(self.routes.waypoint(ref["HANDLE_CLEAR"])), self._deadline)
        steps = self.opener.plan(request)
        gaps = self.routes.open_plan_gaps(steps)
        if gaps:
            raise ActionFailed("腕部开屉方案未通过： " + "；".join(gaps))
        if not fresh(stamp, self.now_ns(), self.recipe["limits"]["max_frame_age_s"]):
            raise ActionFailed("腕部视觉计划生成过慢，图像已过期")
        self._check_pose(self._tcp(), request.start_tcp)
        last_move = next(step for step in reversed(steps) if step["op"] == "move_tcp")
        self._check_pose(last_move["pose"], request.handle_clear_tcp)
        return steps, frame

    def _do_observe_high(self, params: dict, timeout_s: float) -> dict:
        if "flow" in self.recipe:
            self._approach = [self._path_point(self._arm())]
            self._record_approach = True
        try:
            self._replay(params["route_ref"], timeout_s)
        finally:
            self._record_approach = False
        self._check_waypoint("OBSERVE_HIGH")
        if not self._still():
            raise ActionFailed("观察位未稳定，取图会带运动模糊")
        return {"arrived": True, "stationary": True,
                "joint_branch_verified": True}

    def _do_open_gripper(self, params: dict, timeout_s: float) -> dict:
        width = float(params["width_m"])
        self._grip(width, timeout_s)
        measured = self._arm()["gripper"]["width"]
        if measured < width - 0.005:
            raise ActionFailed(f"夹爪实测张开 {measured * 1000:.0f} mm 不足 {width * 1000:.0f} mm")
        return {"gap_verified": True, "arrived": True, "width_m": measured}

    def _do_align_above_box(self, params: dict, timeout_s: float) -> dict:
        pose = _pose7(params["tcp_pose"])
        self._record_approach = bool(self._approach)
        try:
            self._move_via(pose, timeout_s)
        finally:
            self._record_approach = False
        tcp = self._tcp()
        if math.dist(tcp[:3], pose[:3]) > TCP_TOL_M:
            raise ActionFailed("未到达盒上方对准位")
        if not self._still():
            raise ActionFailed("对准后未稳定")
        return {"arrived": True, "stationary": True, "joint_branch_verified": True,
                "tcp": tcp}

    def _do_descend(self, params: dict, timeout_s: float) -> dict:
        if not fresh(params.get("vision_timestamp_ns"), self.now_ns(),
                     self.recipe["limits"]["max_frame_age_s"]):
            raise ActionFailed("下探前复拍已过期")
        return self._descend_to_pose(params, timeout_s)

    def _descend_to_pose(self, params: dict, timeout_s: float) -> dict:
        pose = _pose7(params["tcp_pose"])
        if math.dist(self._tcp()[:2], pose[:2]) > TCP_TOL_M:
            raise ActionFailed("下探起点偏离已复拍的 XY")
        self._grasp_z = float(pose[2])
        self._move(pose, timeout_s)
        tcp = self._tcp()
        if abs(tcp[2] - pose[2]) > 0.004:
            raise ActionFailed(f"下探未到抓取高度：z={tcp[2]:.3f}")
        return {"arrived": True, "joint_branch_verified": True, "at_grasp_height": True,
                "tcp_z_m": tcp[2]}

    def _do_close_gripper(self, params: dict, timeout_s: float) -> dict:
        width = float(params["width_m"])
        self._grip(width, timeout_s)
        arm = self._arm()
        effort = float(arm["gripper"]["effort"])
        measured = float(arm["gripper"]["width"])
        need = 0.5 * float(self.recipe["limits"]["gripper_effort_n"])
        if effort < need:
            raise ActionFailed(f"夹持电流 {effort:.1f} N 低于 {need:.1f} N，未形成接触")
        expected = self.recipe["box_size_m"][1]
        if (arm["gripper"]["is_moving"] or measured <= width
                or abs(measured - expected) > self.recipe["limits"]["size_tolerance_m"]
                or self._hold()["object"] != "box"):
            raise ActionFailed("夹爪实测宽度或夹持物证与药盒不符")
        return {"arrived": True, "contact_verified": True,
                "grip_width_m": measured, "grip_effort_n": effort}

    def _do_short_lift(self, params: dict, timeout_s: float) -> dict:
        pose = _pose7(params["tcp_pose"])
        before = self._tcp()[2]
        if self._hold()["object"] != "box":
            raise ActionFailed("短抬前已失去药盒夹持")
        self._move(pose, timeout_s)
        lifted = self._tcp()[2] - before
        if lifted < 0.8 * float(self.recipe["short_lift_m"]):
            raise ActionFailed(f"短抬只升高 {lifted * 1000:.0f} mm")
        return {"arrived": True, "joint_branch_verified": True, "lift_m": lifted}

    def _do_verify_holding(self, params: dict, timeout_s: float) -> dict:
        hold = self._hold()
        if hold.get("object") != "box" or hold.get("box_lift_verified") is not True:
            raise ActionFailed("缺少药盒随夹爪抬升的独立证据")
        if self._grasp_z is None:
            raise ActionFailed("缺少抓取高度基准，无法核对抬升")
        lift = self._tcp()[2] - self._grasp_z
        if lift < 0.8 * float(self.recipe["short_lift_m"]):
            raise ActionFailed(f"抬升 {lift * 1000:.0f} mm 不足，盒可能仍在抽屉里")
        return {"arrived": True, "holding_verified": True, "box_lift_verified": True,
                "lift_m": lift, "hold_source": hold.get("source")}

    def _do_retreat_to_start(self, params: dict, timeout_s: float) -> dict:
        reversed_points = 0
        if "flow" in self.recipe:
            if len(self._approach) < 2:
                raise ActionFailed("缺少本次接近轨迹，无法原路退出")
            # First undo the vertical descent, then reverse the actual approach
            # waypoints. Every reverse leg is checked again with the box held.
            above = self._approach[-1]["tcp"]
            if math.dist(self._tcp()[:2], above[:2]) > TCP_TOL_M:
                raise ActionFailed("持盒抬升后偏离下探轴")
            for point in reversed(self._approach):
                if self._hold()["object"] != "box":
                    raise ActionFailed("原路退出时未确认持盒")
                self._move(point["tcp"], timeout_s, expected_joints=point["joints"])
                reversed_points += 1
        self._replay(params["route_ref"], timeout_s)
        self._check_waypoint("START")
        tcp = self._tcp()
        handle = self.routes.waypoint(self.recipe["waypoint_refs"]["HANDLE_RELEASE"])
        if not (tcp[0] <= handle[0] - 0.05 and tcp[2] >= handle[2] + 0.05):
            raise ActionFailed("持盒位仍未离开抽屉扫掠区")
        if self._hold().get("object") != "box":
            raise ActionFailed("返回途中失去夹持")
        return {"arrived": True, "at_start": True, "holding_verified": True,
                "payload_clear_of_drawer": True, "reversed_approach_points": reversed_points}

    def _do_rotate_to_drop(self, params: dict, timeout_s: float) -> dict:
        drop_yaw = float(params["yaw_rad"])
        if drop_yaw >= 0:
            raise ActionFailed("右转必须是负偏航（SDK 正值为左转）")
        if self._hold().get("object") != "box":
            raise ActionFailed("转体前未确认持盒")
        # 先转体，再在新坐标系里走到示教落点。反过来会把"落点"当成
        # 转动前的 base_link 坐标，转完就偏了。
        tcp_before = self._tcp()
        measured = self._rotate(params["yaw_segments_rad"], drop_yaw, timeout_s)
        tcp_turned = self._tcp()
        if math.dist(tcp_before[:3], tcp_turned[:3]) > 0.004:
            raise ActionFailed("底盘转动期间手臂位姿变化，扫掠包络失效")
        facts = self._do_move_to_drop(params, timeout_s)
        return {**facts, "swept_clear": True,
                "base_motion_locked": True, "yaw_measured_rad": measured}

    def _do_move_to_drop(self, params: dict, timeout_s: float) -> dict:
        self._replay(params["route_ref"], timeout_s)
        self._check_waypoint("DROP")
        if self._hold()["object"] != "box":
            raise ActionFailed("到桶上方时未确认持盒")
        return {"arrived": True, "holding_verified": True, "above_bin": True}

    def _do_release_box(self, params: dict, timeout_s: float) -> dict:
        if params.get("release_mode") != "drop":
            raise ActionFailed("只支持到点开爪落桶，不支持抛掷")
        self._check_waypoint("DROP")
        if self._hold()["object"] != "box":
            raise ActionFailed("释放前未确认持盒")
        if self.live:
            bin_state = self._bin()
            offset = bin_state.get("release_offset_m")
            if (bin_state.get("box_present") is not False
                    or bin_state.get("release_clearance_verified") is not True
                    or not finite(offset)
                    or not 0 <= offset <= self.recipe["base_motion"]["drop_offset_tolerance_m"]):
                raise ActionFailed("释放前未确认桶空、落点及投放净空")
        self._drop_checked_ns = self.now_ns()
        self._grip(float(params["width_m"]), timeout_s)
        hold = self._hold()
        width = self._arm()["gripper"]["width"]
        if hold.get("holding") is True:
            raise ActionFailed("开爪后仍报告持有物体")
        if width < float(params["width_m"]) - 0.005:
            raise ActionFailed("夹爪未张开到释放宽度")
        return {"arrived": True, "gripper_empty": True, "width_m": width}

    def _do_verify_drop(self, params: dict, timeout_s: float) -> dict:
        caps = self.link.caps()
        if not caps.get("bin_state"):
            raise ActionFailed("缺少 bin_state 能力：落点无法确认，停在桶前交操作员接管")
        info = self._bin()
        if (info.get("box_present") is not True or self._drop_checked_ns is None
                or info["stamp_ns"] < self._drop_checked_ns):
            raise ActionFailed("桶内未检测到药盒")
        distance = info.get("distance_to_center_m")
        if not finite(distance) or distance < 0:
            raise ActionFailed("桶内落点距离不可读")
        budget = float(self.recipe["base_motion"]["drop_offset_tolerance_m"])
        if distance > budget:
            raise ActionFailed(f"落点距桶心 {distance * 1000:.0f} mm，超出 {budget * 1000:.0f} mm")
        if self._hold().get("object") is not None:
            raise ActionFailed("落盒后夹爪仍有物体")
        return {"arrived": True, "box_in_bin": True, "gripper_empty": True,
                "drop_offset_m": distance}

    def _do_rotate_back(self, params: dict, timeout_s: float) -> dict:
        if self._hold().get("object") is not None:
            raise ActionFailed("回转前夹爪应为空")
        # Use the separately checked return sweep with the arm in the drop pose.
        self._rotate(params["yaw_segments_rad"], float(params["yaw_rad"]), timeout_s)
        base = self._base()
        start = self._run_start_base
        yaw_residual = _wrap(base["yaw"] - start["yaw"])
        shift = math.hypot(base["x"] - start["x"], base["y"] - start["y"])
        settings = self.recipe["base_motion"]
        if abs(yaw_residual) > settings["return_yaw_tolerance_rad"] or shift > settings["return_shift_tolerance_m"]:
            raise ActionFailed("回转残差超限，拒绝在偏移底盘上回放手臂路线")
        self._replay(params["route_ref"], timeout_s)
        return {"arrived": True, "base_returned": True, "gripper_empty": True,
                "yaw_measured_rad": yaw_residual, "base_shift_m": shift}

    def _do_regrip_handle(self, params: dict, timeout_s: float) -> dict:
        self._replay(params["route_ref"], timeout_s)
        target = self.routes.waypoint(params["waypoint_ref"])
        if math.dist(self._tcp()[:3], target[:3]) > 0.006:
            raise ActionFailed("未回到示教松手位")
        hold = self._hold()
        if hold.get("object") != "handle":
            raise ActionFailed("重夹后未报告夹到把手")
        return {"arrived": True, "handle_grasped": True, "at_release_pose": True}

    def _do_push_drawer_closed(self, params: dict, timeout_s: float) -> dict:
        opening_before = self._drawer()["opening_m"]
        self._replay(params["route_ref"], timeout_s, drawer_direction=-1)
        opening_after = self._drawer()["opening_m"]
        stroke = opening_before - opening_after
        if opening_after > DRAWER_TOL_M:
            raise ActionFailed(f"抽屉仍开 {opening_after * 1000:.0f} mm，未闭合")
        hold = self._hold()
        if hold.get("object") != "handle":
            raise ActionFailed("推回过程中失去把手夹持")
        return {"arrived": True, "drawer_closed": True, "stroke_verified": True,
                "grip_maintained": True, "stroke_m": stroke,
                "opening_before_m": opening_before, "opening_after_m": opening_after}

    def _do_release_handle(self, params: dict, timeout_s: float) -> dict:
        self._grip(float(self.recipe["gripper_open_m"]), timeout_s)
        if self._hold().get("object") is not None:
            raise ActionFailed("松爪后仍报告夹住把手")
        if params.get("route_ref"):
            self._replay(params["route_ref"], timeout_s)
        elif not self.live:
            tcp = self._tcp()
            self._move([tcp[0] - 0.04, tcp[1], tcp[2] + 0.06, *tcp[3:]], timeout_s)
        else:
            raise ActionFailed("缺少松开把手后的已验证撤离路线")
        after = self._tcp()
        handle = self.routes.waypoint(self.recipe["waypoint_refs"]["HANDLE_RELEASE"])
        closed_handle_x = handle[0] + float(self.recipe["drawer_opening_m"])
        if not (after[2] >= handle[2] + 0.03
                and after[0] <= closed_handle_x - 0.02):
            raise ActionFailed("撤手后手臂仍在把手扫掠区")
        return {"arrived": True, "handle_released": True, "arm_clear_of_drawer": True}

    def _do_return_home(self, params: dict, timeout_s: float) -> dict:
        self._replay(params["route_ref"], timeout_s)
        self._check_waypoint("HOME")
        if self._hold().get("object") is not None:
            raise ActionFailed("归位时夹爪非空")
        return {"arrived": True, "at_home": True, "gripper_empty": True}

    # ------------------------------------------------------------------ 复现
    def _replay(self, name: str, timeout_s: float, *, drawer_direction: int = 0) -> None:
        self._replay_steps(self.routes.route(name), timeout_s, drawer_direction=drawer_direction)

    def _replay_steps(self, steps, timeout_s: float, *, drawer_direction: int = 0) -> None:
        for step in steps:
            self._check_deadline()
            op = step["op"]
            if op == "move_tcp":
                self._move(step["pose"], timeout_s)
            elif op == "move_rel":
                current = self._tcp()
                pose = [current[0] + step["dx"], current[1] + step["dy"],
                        current[2] + step["dz"], *current[3:]]
                if drawer_direction:
                    if step["dx"] * drawer_direction >= 0:
                        raise ActionFailed("分段接触位移与抽屉运动方向不符")
                    if self._hold()["object"] != "handle":
                        raise ActionFailed("接触位移前未确认夹住把手")
                    opening_before = self._drawer()["opening_m"]
                self._move(pose, timeout_s)
                if drawer_direction:
                    expected = abs(current[0] - pose[0])
                    measured = self._drawer()["opening_m"]
                    moved = drawer_direction * (measured - opening_before)
                    if self._hold()["object"] != "handle":
                        raise ActionFailed("抽屉运动中失去把手夹持")
                    minimum = 0.002 if self.live else 0.7 * expected
                    if expected > 0.005 and moved < minimum:
                        raise ActionFailed(
                            f"抽屉行程未跟随夹持位移：期望 {expected * 1000:.0f} mm，"
                            f"实测 {moved * 1000:.0f} mm")
                    if measured > self.recipe["drawer_opening_m"] + DRAWER_TOL_M:
                        raise ActionFailed("抽屉超过目标开度范围")
            elif op == "grip":
                if step["width_m"] >= self.recipe["gripper_open_m"]:
                    raise ActionFailed("闭爪路线宽度不小于张爪宽度")
                self._grip(step["width_m"], timeout_s)
                if drawer_direction and self._hold()["object"] != "handle":
                    raise ActionFailed("闭爪后没有夹到把手")
            elif op == "release":
                if self.live and abs(step["width_m"] - self.recipe["gripper_open_m"]) > 1e-6:
                    raise ActionFailed("松爪路线宽度不符合已验收配方")
                if drawer_direction == 1 and self.fw.flow_options(self.recipe)[1]:
                    self._check_pose(self._tcp(), self.routes.waypoint(
                        self.recipe["waypoint_refs"]["HANDLE_RELEASE"]))
                if drawer_direction == 1 and abs(
                        self._drawer()["opening_m"] - self.recipe["drawer_opening_m"]) > DRAWER_TOL_M:
                    raise ActionFailed("抽屉未到目标开度，保留夹持并停止")
                self._grip(step["width_m"], timeout_s)
                if self._hold()["object"] is not None:
                    raise ActionFailed("松爪后仍有接触")
            elif op == "settle":
                time.sleep(min(float(step["ms"]) / 1000.0, self._remaining_s()))
            else:
                raise ActionFailed(f"路线 {name} 含未知操作 {op}")

    def _rotate(self, segments, total_rad: float, timeout_s: float) -> float:
        if not self.fw.flow_options(self.recipe)[0] or not self.routes.verified("base_yaw_free_path"):
            raise ActionFailed("底盘转体未获当前配方授权")
        segments = [float(s) for s in segments]
        if abs(sum(segments) - total_rad) > 1e-6:
            raise ActionFailed("转角分段之和与目标偏航不符")
        start = self._base()
        start_yaw = start["yaw"]
        start_tcp = self._tcp()
        payload = self._hold()["object"]
        commanded = 0.0
        tolerance = float(self.recipe["base_motion"]["yaw_tolerance_rad"])
        per_segment = float(self.recipe["base_motion"]["rotate_segment_timeout_s"])
        for index, segment in enumerate(segments, 1):
            result = self._call("set_base_pose_rel", 0.0, segment,
                                min(per_segment, self._remaining_s()))
            self._require_done(result, f"第 {index} 段底盘转角")
            if self.live and result.get("swept_clear") is not True:
                raise ActionFailed("缺少转体扫掠净空的执行证据")
            commanded += segment
            current = self._base()
            measured = _wrap(current["yaw"] - start_yaw)
            residual = _wrap(measured - commanded)
            if abs(residual) > tolerance:
                raise ActionFailed(
                    f"第 {index} 段转角残差 {math.degrees(residual):+.1f}°，"
                    f"超过 {math.degrees(tolerance):.1f}°")
            if self.live and (
                    math.hypot(current["x"] - start["x"], current["y"] - start["y"]) > BASE_XY_TOL_M
                    or math.dist(self._tcp()[:3], start_tcp[:3]) > STILL_TCP_M
                    or self._hold()["object"] != payload):
                raise ActionFailed("分段转体时底盘平移、手臂漂移或夹持改变")
        return _wrap(self._base()["yaw"] - self._run_start_base["yaw"])

    # ------------------------------------------------------------------ 原语
    def _move(self, pose, timeout_s: float, *, expected_joints=None) -> None:
        pose = _pose7(pose)
        distance = math.dist(pose[:3], self._tcp()[:3])
        budget = self._remaining_s()
        if budget <= 0:
            raise TimeoutError("动作超时")
        speed = float(self.recipe["limits"]["max_linear_speed_m_s"])
        need = distance / speed if speed > 0 else budget
        extra = {"expected_joints_rad": expected_joints} if self.live and expected_joints else {}
        payload = self._hold()["object"]
        result = self._call("move_tcp", pose, min(budget, timeout_s, max(need * 1.5, 1.0)),
                            straight=True, hold=payload or "none", **extra)
        self._require_done(result, "TCP 运动")
        if result.get("collision_checked") is not True:
            raise ActionFailed("运动未带碰撞检查结果，路径不予采信")
        if self.live:
            for fact in ("swept_clear", "force_monitored", "joint_branch_continuous"):
                if result.get(fact) is not True:
                    raise ActionFailed(f"轨迹执行缺少 {fact} 证据")
        state = self._arm()
        self._check_pose(state["tcp"], pose)
        if payload is not None and self._hold()["object"] != payload:
            raise ActionFailed("运动途中失去夹持")
        if self.live:
            self._check_joints(state, result.get("joint_goal_rad"))
            if expected_joints is not None:
                self._check_joints(state, expected_joints)
        if self._record_approach:
            self._approach.append(self._path_point(state, pose))

    def _move_via(self, pose, timeout_s: float) -> None:
        pose = _pose7(pose)
        current = self._tcp()
        if current[2] < pose[2]:
            self._move([*current[:2], pose[2], *current[3:]], timeout_s)
            current = self._tcp()
        if math.hypot(pose[0] - current[0], pose[1] - current[1]) > 0.02:
            self._move([pose[0], pose[1], current[2], *pose[3:]], timeout_s)
        self._move(pose, timeout_s)

    def _grip(self, width: float, timeout_s: float) -> dict:
        limits = self.recipe["limits"]
        result = self._call("set_gripper", float(width), float(limits["gripper_speed_m_s"]),
                            float(limits["gripper_effort_n"]), min(self._remaining_s(), 5.0))
        self._require_done(result, "夹爪动作")
        return result

    def _call(self, method: str, *args, **kwargs) -> dict:
        self._check_deadline()
        started = self.now_ns()
        result = getattr(self.link, method)(*args, **kwargs)
        self._check_deadline()
        if not isinstance(result, dict):
            raise LinkError(f"{method} 返回类型异常：{type(result).__name__}")
        if self.live and (not self._fresh_state(result) or result["stamp_ns"] < started):
            raise LinkError(f"{method} 回执早于本次请求或已经过期")
        return result

    @staticmethod
    def _require_done(result: dict, what: str) -> None:
        if result.get("accepted") is not True:
            raise ActionFailed(f"{what}未被接收：{result.get('status')}")
        if result.get("completed") is not True:
            raise TimeoutError(f"{what}未确认完成：{result.get('status')}")

    def _remaining_s(self) -> float:
        return max(0.0, (self._deadline - self.now_ns()) / 1e9)

    def _check_deadline(self) -> None:
        if self._latched is not None or self._remaining_s() <= 0:
            raise TimeoutError(self._latched or "动作超出时限")

    def _receipt(self, action, status: str, message: str, evidence: dict | None = None):
        return self.fw.Receipt(status, action.request_id, self.now_ns(),
                               evidence or {}, message)

    # ------------------------------------------------------------------ 读数
    def _fresh_state(self, state: dict) -> bool:
        return fresh(state.get("stamp_ns"), self.now_ns(),
                     self.recipe["limits"]["max_receipt_age_s"])

    @staticmethod
    def _path_point(state: dict, tcp=None) -> dict:
        return {"tcp": list(state["tcp"] if tcp is None else tcp),
                "joints": dict(zip(state["joint_names"], state["joints"]))}

    def _check_pose(self, actual, target) -> None:
        if math.dist(actual[:3], target[:3]) > TCP_TOL_M:
            raise ActionFailed("TCP 位置未到位")
        tolerance = self.recipe["limits"].get("tcp_angle_tolerance_rad", 0.05)
        dot = abs(sum(x*y for x, y in zip(norm_quat(actual[3:]), norm_quat(target[3:]))))
        if 2 * math.acos(min(1.0, dot)) > tolerance:
            raise ActionFailed("TCP 姿态未到位")

    def _check_joints(self, arm: dict, target) -> None:
        actual = dict(zip(arm["joint_names"], arm["joints"]))
        tol = self.recipe["limits"]["joint_tolerance_rad"]
        if (not isinstance(target, dict) or target.keys() != actual.keys()
                or not finite(*target.values())
                or any(abs(actual[name] - target[name]) > tol for name in actual)):
            raise ActionFailed("实测命名关节与规划或示教分支不符")

    def _check_waypoint(self, key: str, arm=None) -> None:
        ref = self.recipe["waypoint_refs"][key]
        arm = self._arm() if arm is None else arm
        self._check_pose(arm["tcp"], self.routes.waypoint(ref))
        if self.live:
            self._check_joints(arm, self.routes.joint_target(ref))

    def _arm(self) -> dict:
        state = self.link.arm_state()
        tcp = state.get("tcp")
        if not (isinstance(tcp, list) and len(tcp) == 7 and finite(*tcp)):
            raise LinkError("右臂 TCP 读数非法")
        if not self._fresh_state(state):
            raise LinkError("右臂读数过期")
        names, joints = state.get("joint_names"), state.get("joints")
        if (not isinstance(names, (list, tuple)) or len(names) != 7
                or not all(isinstance(n, str) and n for n in names) or len(set(names)) != 7
                or not isinstance(joints, (list, tuple)) or len(joints) != 7 or not finite(*joints)):
            raise LinkError("右臂命名关节读数非法")
        gripper = state.get("gripper")
        if (not isinstance(gripper, dict) or not finite(gripper.get("width"), gripper.get("effort"))
                or gripper["width"] < 0 or type(gripper.get("is_moving")) is not bool):
            raise LinkError("右夹爪状态非法")
        return state

    def _tcp(self) -> list[float]:
        return [float(v) for v in self._arm()["tcp"]]

    def _base(self) -> dict:
        state = self.link.base_pose()
        if not finite(state.get("x"), state.get("y"), state.get("yaw")):
            raise LinkError("里程计读数非法")
        if not fresh(state.get("stamp_ns"), self.now_ns(),
                     float(self.recipe["limits"]["max_receipt_age_s"])):
            raise LinkError("里程计读数过期")
        return state

    def _drawer(self) -> dict:
        state = self.link.drawer_state()
        if not finite(state.get("opening_m")) or state["opening_m"] < 0 or not self._fresh_state(state):
            raise LinkError("抽屉开度读数非法")
        uncertainty = state.get("uncertainty_m")
        if not finite(uncertainty) or not 0 <= uncertainty <= 0.003:
            raise LinkError("抽屉开度不确定度超限")
        return state

    def _hold(self) -> dict:
        state = self.link.hold_state()
        if (type(state.get("holding")) is not bool or not self._fresh_state(state)
                or state.get("object") not in (None, "box", "handle")
                or state["holding"] != (state.get("object") is not None)):
            raise LinkError("夹持状态不可判定（禁止当成成功）")
        return state

    def _bin(self) -> dict:
        state = self.link.bin_state()
        if not self._fresh_state(state) or type(state.get("box_present")) is not bool:
            raise LinkError("桶状态过期或不可判定")
        return state

    def _still(self) -> bool:
        first = self._arm()
        time.sleep(0.12)
        second = self._arm()
        first_joints = dict(zip(first["joint_names"], first["joints"]))
        second_joints = dict(zip(second["joint_names"], second["joints"]))
        return (second["stamp_ns"] > first["stamp_ns"]
                and first_joints.keys() == second_joints.keys()
                and math.dist(first["tcp"][:3], second["tcp"][:3]) <= STILL_TCP_M
                and max(abs(q - second_joints[name]) for name, q in first_joints.items()) <= 0.004)

    def _snapshot(self) -> dict:
        base = self._base()
        if not self.fw.flow_options(self.recipe)[0]:
            start = self._run_start_base
            if (math.hypot(base["x"] - start["x"], base["y"] - start["y"]) > BASE_XY_TOL_M
                    or abs(_wrap(base["yaw"] - start["yaw"])) > BASE_YAW_TOL_RAD):
                raise ActionFailed("底盘偏离本次标定停靠位")
        return {"base": base, "drawer": self._drawer()["opening_m"]}

    def _unchanged(self, action: str, before: dict, after: dict) -> bool:
        base_free = action in ("rotate_to_drop", "rotate_back")
        drawer_free = action in ("open_drawer_release", "push_drawer_closed")
        if not base_free:
            if (math.hypot(after["base"]["x"] - before["base"]["x"],
                           after["base"]["y"] - before["base"]["y"]) > BASE_XY_TOL_M
                    or abs(_wrap(after["base"]["yaw"] - before["base"]["yaw"])) > BASE_YAW_TOL_RAD):
                return False
        if not drawer_free and abs(after["drawer"] - before["drawer"]) > 0.001:
            return False
        return True

    # ------------------------------------------------------------------ 诊断
    def report(self) -> dict:
        return {"backend": self.backend,
                "link": getattr(self.link, "kind", "unknown"),
                "latched": self._latched, "actions": list(self._log)}


def _pose7(value) -> list[float]:
    """{'position_m': [...], 'orientation_xyzw': [...]} 或 7 维数组 → 7 维列表。"""
    if isinstance(value, dict):
        position = value.get("position_m")
        quat = value.get("orientation_xyzw")
        if not (isinstance(position, (list, tuple)) and len(position) == 3
                and isinstance(quat, (list, tuple)) and len(quat) == 4):
            raise ActionFailed("位姿字段不完整")
        pose = [float(v) for v in list(position) + list(quat)]
    elif isinstance(value, (list, tuple)) and len(value) == 7:
        pose = [float(v) for v in value]
    else:
        raise ActionFailed("位姿必须是 7 维或含 position_m/orientation_xyzw 的字典")
    if not finite(*pose):
        raise ActionFailed("位姿含非有限值")
    pose[3:] = norm_quat(pose[3:])
    return pose


def _takes_timeout(func) -> bool:
    import inspect
    try:
        return len(inspect.signature(func).parameters) >= 2
    except (TypeError, ValueError):
        return False


def _brief(evidence: dict) -> dict:
    keep = ("drawer_open_verified", "handle_released", "arm_clear_of_drawer", "stationary",
            "gap_verified", "contact_verified", "holding_verified", "box_lift_verified",
            "at_start", "yaw_measured_rad", "gripper_empty", "box_in_bin", "base_shift_m",
            "handle_grasped", "drawer_closed", "stroke_m", "at_home", "drop_offset_m",
            "grip_effort_n", "lift_m")
    return {k: v for k, v in evidence.items() if k in keep}
