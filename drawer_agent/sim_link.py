"""离线装置：用 SDK 形状的接口模拟底盘、右臂、夹爪、抽屉与桶。

用途只有一个 —— 证明**适配器本身**能把 20 步流程跑完：状态机、路线回放、
证据计算、转角里程计核验、推回行程测量都走真实代码，只有硬件读数来自这里。
它不是真机验收，也没有任何识别能力。

装置内的几何是构造值，仅保证自洽：抽屉把手在 x=0.90 m，拉出 0.27 m 后到
x=0.63 m；盒心在把手后 0.11 m；盒顶平面 z=0.84 m；桶心由示教 DROP 点反推
（转 −30° 后正好在桶心正上方），这与现场"桶摆在示教落点下方"的做法一致。

故障注入在链路层，不在状态机层：这样测的是适配器有没有真的读里程计、
真的量行程，而不是状态机有没有按剧本拒绝。
"""

from __future__ import annotations

import math
import time
from typing import Any

from .link import LinkError, finite, norm_quat, now_ns

HANDLE_X_CLOSED = 0.90
HANDLE_Z = 0.98
HANDLE_THICKNESS = 0.020
BOX_OFFSET_FROM_HANDLE = 0.11
BOX_TOP_Z = 0.84
BOX_SIZE = (0.085, 0.075)
DRAWER_MAX_OPENING = 0.30
MAX_STEP_M = 0.70
MAX_SPEED_M_S = 0.30

JOINT_NAMES = ("right_arm_joint_1", "right_arm_joint_2", "right_arm_joint_3",
               "right_arm_joint_4", "right_arm_joint_5", "right_arm_joint_6",
               "right_arm_joint_7")
NOMINAL_JOINTS = [0.0, 0.4, 0.0, 1.2, 0.0, 0.3, 0.0]


def _rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class OfflineRig:
    kind = "offline_rig"

    def __init__(self, recipe: dict, routes, *, tool_id: str = "offline_tool",
                 tcp_frame: str = "offline_tcp"):
        self.recipe = recipe
        self.routes = routes
        self.tool_id = tool_id
        self.tcp_frame = tcp_frame
        self.calibration_id = recipe["calibration"]["id"]

        self.base = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.tcp = [0.30, 0.0, 1.15, 1.0, 0.0, 0.0, 0.0]
        self.joints = list(NOMINAL_JOINTS)
        self.grip_width = 0.095
        self.grip_effort = 0.0
        self.payload = "none"            # none / handle / box
        self.drawer_opening = 0.0
        self.box: dict[str, Any] = {"where": "drawer", "x": None, "y": None}
        self.bin_world = self._bin_from_teach()
        self.frames = 0
        self.stops: list[str] = []
        self.calls: list[str] = []
        self.faults: dict[str, Any] = {}
        self._drop_z = 0.60

    # ------------------------------------------------------------------ 装置
    def _bin_from_teach(self) -> tuple[float, float]:
        drop = self.routes.waypoint("DROP")
        yaw = (float(self.recipe["base_motion"]["drop_yaw_rad"])
               if self.recipe["base_motion"]["allowed"] else 0.0)
        return _rot(yaw, drop[0], drop[1])

    def inject(self, name: str, **kwargs) -> None:
        self.faults[name] = kwargs or True

    # ------------------------------------------------------------------ 能力
    def caps(self) -> dict[str, bool]:
        return {name: True for name in (
            "rig_status", "arm_state", "move_tcp", "set_gripper", "base_pose",
            "set_base_pose_rel", "stop", "capture", "drawer_state", "hold_state",
            "bin_state", "move_joints")}

    def clock_ns(self) -> int:
        return now_ns()

    # ------------------------------------------------------------------ 读数
    def rig_status(self) -> dict:
        return {"legs_locked": True, "head_locked": True, "base_parked": True,
                "exclusive_right_arm": True, "controller_owned": True,
                "stop_channel": True, "tool_id": self.tool_id, "tcp_frame": self.tcp_frame,
                "camera_calibration_id": self.calibration_id, "stamp_ns": now_ns()}

    def base_pose(self) -> dict:
        return {"x": self.base["x"], "y": self.base["y"], "yaw": self.base["yaw"],
                "accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def arm_state(self) -> dict:
        self.calls.append("arm_state")
        return {"tcp": list(self.tcp), "joints": list(self.joints),
                "joint_names": list(JOINT_NAMES),
                "gripper": {"width": self.grip_width, "effort": self.grip_effort,
                            "is_moving": False},
                "accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def drawer_state(self) -> dict:
        self.calls.append("drawer_state")
        opening = self.drawer_opening
        if self.faults.get("drawer_jam"):
            opening = self.faults["drawer_jam"].get("reported_m", 0.0)
        return {"opening_m": opening, "uncertainty_m": 0.002,
                "accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def hold_state(self) -> dict:
        self.calls.append("hold_state")
        holding = self.payload != "none"
        return {"holding": holding, "object": None if self.payload == "none" else self.payload,
                "source": "gripper_width_effort",
                "box_lift_verified": self.payload == "box"
                and self.tcp[2] >= self.recipe["grasp_tcp_z_m"] + 0.8*self.recipe["short_lift_m"],
                "width_m": self.grip_width, "effort_n": self.grip_effort,
                "accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def bin_center(self) -> tuple[float, float]:
        shift = self.faults.get("bin_shift")
        if shift:
            return (self.bin_world[0] + float(shift.get("dx", 0.0)),
                    self.bin_world[1] + float(shift.get("dy", 0.0)))
        return self.bin_world

    def bin_state(self) -> dict:
        self.calls.append("bin_state")
        if self.box["where"] != "world":
            return {"box_present": False, "distance_to_center_m": None,
                    "accepted": True, "completed": True, "status": "SUCCESS",
                    "stamp_ns": now_ns()}
        center = self.bin_center()
        dx = self.box["x"] - center[0]
        dy = self.box["y"] - center[1]
        return {"box_present": True, "distance_to_center_m": math.hypot(dx, dy),
                "accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def capture(self, camera_id: str, timeout_s: float) -> dict:
        self.calls.append("capture")
        self.frames += 1
        stamp = now_ns()
        if self.faults.get("stale_frame"):
            stamp -= 2_000_000_000
        size = self.recipe["calibration"]["image_size_wh"]
        return {"image_ref": f"offline://frame/{self.frames}",
                "frame_id": f"offline-{self.frames}-{stamp}",
                "camera_id": camera_id, "stamp_ns": stamp,
                "size_wh": list(size), "calibration_id": self.calibration_id,
                "accepted": True, "completed": True, "status": "SUCCESS"}

    # ------------------------------------------------------------------ 动作
    def set_base_pose_rel(self, x_m: float, yaw_rad: float, timeout_s: float) -> dict:
        self.calls.append("set_base_pose_rel")
        if not finite(x_m, yaw_rad) or abs(x_m) > 0.5 or abs(yaw_rad) > math.pi:
            raise LinkError("底盘相对位移参数非法")
        applied_yaw = yaw_rad
        if self.faults.get("short_yaw"):
            applied_yaw *= float(self.faults["short_yaw"].get("fraction", 0.6))
        if self.faults.get("yaw_stuck"):
            applied_yaw = 0.0
        dx, dy = _rot(self.base["yaw"], x_m, 0.0)
        self.base["x"] += dx
        self.base["y"] += dy
        self.base["yaw"] += applied_yaw
        self._sync_payload()
        # 故意如实返回 SUCCESS：真机 2026-11-07 就是这样，命令成功但零位移。
        # 适配器只能靠里程计发现，这条路径必须在离线装置里也走通。
        return {"accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def move_tcp(self, pose: list[float], timeout_s: float, *, straight: bool = True,
                 hold: str = "none", expected_joints_rad=None) -> dict:
        self.calls.append("move_tcp")
        if not isinstance(pose, (list, tuple)) or len(pose) != 7 or not finite(*pose):
            raise LinkError("TCP 位姿必须是 7 维有限值")
        target = [float(v) for v in pose]
        target[3:] = norm_quat(target[3:])
        delta = math.dist(target[:3], self.tcp[:3])
        if delta > MAX_STEP_M:
            return {"accepted": False, "completed": False, "status": "STEP_TOO_LARGE",
                    "stamp_ns": now_ns()}
        if timeout_s <= 0 or delta / timeout_s > MAX_SPEED_M_S:
            return {"accepted": False, "completed": False, "status": "SPEED_LIMIT",
                    "stamp_ns": now_ns()}
        previous = list(self.tcp)
        self.tcp = target
        self._drag_drawer(previous)
        self._sync_payload()
        if self.faults.get("jam_step") and delta > 0.05:
            raise TimeoutError("离线装置注入：未收到到位回执")
        return {"accepted": True, "completed": True, "status": "SUCCESS",
                "collision_checked": True, "stamp_ns": now_ns()}

    def move_joints(self, positions: list[float], timeout_s: float) -> dict:
        self.calls.append("move_joints")
        if not isinstance(positions, (list, tuple)) or len(positions) != 7 or not finite(*positions):
            raise LinkError("关节目标必须是 7 个有限值")
        self.joints = [float(v) for v in positions]
        return {"accepted": True, "completed": True, "status": "SUCCESS",
                "stamp_ns": now_ns()}

    def set_gripper(self, width_m: float, speed_m_s: float, effort_n: float,
                    timeout_s: float) -> dict:
        self.calls.append("set_gripper")
        if not finite(width_m, speed_m_s, effort_n) or width_m < 0:
            raise LinkError("夹爪参数非法")
        # 判"合"还是"开"看目标宽度相对当前宽度，不能只看绝对值：
        # 夹盒用 65 mm、夹把手用 20 mm，两种都是闭合指令。
        closing = width_m < self.grip_width - 1e-6
        if closing:
            engaged = self._engage()
            if engaged:
                self.payload = engaged
                self.grip_width = HANDLE_THICKNESS if engaged == "handle" else BOX_SIZE[1]
                self.grip_effort = float(effort_n) * 0.9
            else:
                self.payload = "none"
                self.grip_width = max(0.0, width_m)
                self.grip_effort = 1.0
        else:
            self._let_go()
            self.grip_width = float(width_m)
            self.grip_effort = 0.0
        return {"accepted": True, "completed": True, "status": "SUCCESS",
                "width_m": self.grip_width, "effort_n": self.grip_effort,
                "stamp_ns": now_ns()}

    def stop(self, chain: str) -> dict:
        self.calls.append(f"stop:{chain}")
        self.stops.append(chain)
        return {"accepted": True, "completed": True, "status": "SUCCESS",
                "chain": chain, "stamp_ns": now_ns()}

    # ------------------------------------------------------------------ 内部
    def handle_pose(self) -> tuple[float, float, float]:
        return (HANDLE_X_CLOSED - self.drawer_opening, 0.0, HANDLE_Z)

    def box_center(self) -> tuple[float, float]:
        if self.box["where"] == "world":
            return self.box["x"], self.box["y"]
        if self.payload == "box":
            return self.tcp[0], self.tcp[1]
        return HANDLE_X_CLOSED - self.drawer_opening - BOX_OFFSET_FROM_HANDLE, 0.0

    def box_visible(self) -> bool:
        return (self.box["where"] == "drawer" and self.drawer_opening >= 0.10
                and self.payload != "box")

    def _engage(self) -> str | None:
        hx, hy, hz = self.handle_pose()
        if math.dist((self.tcp[0], self.tcp[1], self.tcp[2]), (hx, hy, hz)) <= 0.015:
            # 只在抽屉已拉开后才夹空：第一次夹把手必须成功，否则测的不是重夹。
            if self.faults.get("regrip_miss") and self.drawer_opening > 0.20:
                return None
            return "handle"
        cx, cy = self.box_center()
        if (self.payload == "none" and self.box["where"] == "drawer"
                and math.hypot(self.tcp[0] - cx, self.tcp[1] - cy) <= 0.020
                and abs(self.tcp[2] - self.recipe["grasp_tcp_z_m"]) <= 0.030):
            if self.faults.get("empty_grasp"):
                return None
            return "box"
        return None

    def _let_go(self) -> None:
        if self.payload == "box":
            wx, wy = _rot(self.base["yaw"], self.tcp[0], self.tcp[1])
            self.box = {"where": "world", "x": self.base["x"] + wx,
                        "y": self.base["y"] + wy}
        self.payload = "none"

    def _drag_drawer(self, previous: list[float]) -> None:
        if self.payload != "handle":
            return
        moved = previous[0] - self.tcp[0]
        opening = self.drawer_opening + moved
        if self.faults.get("stroke_short"):
            # 抽屉卡在离闭合 20 mm 处：推得动，但到底差一截。
            opening = max(0.020, opening)
        self.drawer_opening = max(0.0, min(DRAWER_MAX_OPENING, opening))

    def _sync_payload(self) -> None:
        if self.payload == "handle":
            # 夹着把手时，把手位置由抽屉行程决定，TCP 跟着走不算漂移。
            return


class OfflinePerception:
    """离线合成掩膜：把装置里的盒顶矩形反投影回原始像素。

    它不是检测器，也不产生任何识别能力。作用是在不接权重的情况下，
    让 `geometry.localize` 的真实投影与判定逻辑被完整执行一遍：
    合成像素 → H → base_link，中心误差可在测试里量化。
    """

    def __init__(self, rig: OfflineRig):
        self.rig = rig

    def segment(self, frame) -> dict:
        rig = self.rig
        base = {"schema": "drawer_box_seg/1", "frame_id": frame.frame_id,
                "camera_id": frame.camera_id,
                "capture_timestamp_ns": frame.capture_timestamp_ns,
                "image_size_wh": list(frame.image_size_wh),
                "model_id": rig.recipe["model_id"], "status": "no_target",
                "detections": []}
        if rig.faults.get("no_target") or not rig.box_visible():
            return base
        poly = self._pixels()
        if poly is None:
            return base
        base.update(status="ok", detections=[{
            "class_name": "medicine_box_top",
            "confidence": 0.97,
            "mask_polygon_xy_px": poly,
            "source": "offline_synthetic"}])
        return base

    def _pixels(self) -> list[list[float]] | None:
        h = [float(v) for v in self.rig.recipe["calibration"]["H_pixel_to_base_xy"]]
        inv = _inv3(h)
        cx, cy = self.rig.box_center()
        half_l, half_s = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
        corners = [(cx - half_l, cy - half_s), (cx + half_l, cy - half_s),
                   (cx + half_l, cy + half_s), (cx - half_l, cy + half_s)]
        out = []
        for x, y in corners:
            u = inv[0] * x + inv[1] * y + inv[2]
            v = inv[3] * x + inv[4] * y + inv[5]
            w = inv[6] * x + inv[7] * y + inv[8]
            if abs(w) < 1e-12:
                return None
            out.append([u / w, v / w])
        return out


class ImageMockPerception(OfflinePerception):
    """Render a fixture image, then run the same HSV mask code as the bench service."""

    def segment(self, frame):
        import cv2
        import numpy as np
        from types import SimpleNamespace
        from yolo_medbox.bench_mask import ColorSegmenter

        width, height = frame.image_size_wh
        image = np.zeros((height, width, 3), dtype=np.uint8)
        if self.rig.box_visible() and not self.rig.faults.get("no_target"):
            polygon = self._pixels()
            if polygon is not None:
                cv2.fillConvexPoly(image, np.rint(polygon).astype(np.int32), (0, 255, 0))
        # The image fixture shares the exact contour extraction with the service.
        segmenter = ColorSegmenter(None, SimpleNamespace(
            capture_dir=".", hsv_low=(50, 200, 200), hsv_high=(70, 255, 255),
            min_area_px=100, model_id=self.rig.recipe["model_id"]))
        return segmenter.predict_image(image, {"frame_id": frame.frame_id, "camera_id": frame.camera_id},
                                       frame.capture_timestamp_ns)


def _inv3(h: list[float]) -> list[float]:
    """3×3 单应矩阵求逆（行主序）。手写一次，省掉一个运行期依赖。"""
    a, b, c, d, e, f, g, i, j = h
    det = a * (e * j - f * i) - b * (d * j - f * g) + c * (d * i - e * g)
    if abs(det) < 1e-15:
        raise LinkError("单应矩阵奇异，无法反投影")
    return [(e * j - f * i) / det, (c * i - b * j) / det, (b * f - c * e) / det,
            (f * g - d * j) / det, (a * j - c * g) / det, (c * d - a * f) / det,
            (d * i - e * g) / det, (b * g - a * i) / det, (a * e - b * d) / det]


def clock_probe(rig: OfflineRig) -> float:
    """返回装置时钟与本地单调时钟的偏差（秒），用于自检打印。"""
    return (time.monotonic_ns() - rig.clock_ns()) / 1e9
