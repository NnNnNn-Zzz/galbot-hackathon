"""Deterministic, hardware-free implementation of :class:`RobotAdapter`."""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass

import numpy as np

from sdk_wrapper.dev.contracts import (
    ErrorCode,
    RunMode,
    TaskStatus,
    ToolResult,
    canceled,
    failed,
    ok,
    rejected,
    running,
)
from sdk_wrapper.dev.adapters.base import (
    ALONG_MAX_TRAVEL_M,
    ALONG_SAFE_Z,
    RobotAdapter,
    SUPPORTED_GESTURES,
    SUPPORTED_PLACES,
    quat_to_rot,
)
from sdk_wrapper.dev.adapters.synthetic_scene import SyntheticCameraMixin


@dataclass
class _MockTask:
    operation: str
    target: str
    polls_remaining: int
    status: TaskStatus = TaskStatus.RUNNING
    failure_message: str | None = None


class MockAdapter(SyntheticCameraMixin, RobotAdapter):
    """Simulate basic robot operations without importing or initializing an SDK.

    A running task advances after ``task_polls_until_succeeded`` status polls.
    ``fail_next_task`` injects one execution failure after task startup.
    ``simulate_unknown_task_state`` is provided to exercise uncertain-state
    handling without implying that the task has stopped.

    相机能力来自 ``SyntheticCameraMixin``：输出与真机同格式的 JPEG / 裸 16UC1 /
    CameraInfo / 外参，所以 ``--adapter mock`` 也能完整演练"看 + 动"。
    ``fail_rgb`` / ``fail_depth`` 可模拟传感器未就绪。
    """

    def __init__(
        self,
        *,
        task_polls_until_succeeded: int = 1,
        fail_next_task: bool = False,
        fail_rgb: bool = False,
        fail_depth: bool = False,
        scene: str = "objects",
    ) -> None:
        if (
            isinstance(task_polls_until_succeeded, bool)
            or not isinstance(task_polls_until_succeeded, int)
            or task_polls_until_succeeded < 0
        ):
            raise ValueError("task_polls_until_succeeded must be a non-negative integer")
        if not isinstance(fail_next_task, bool):
            raise TypeError("fail_next_task must be a bool")
        if not isinstance(fail_rgb, bool):
            raise TypeError("fail_rgb must be a bool")
        if not isinstance(fail_depth, bool):
            raise TypeError("fail_depth must be a bool")
        if scene not in ("objects", "cup_table"):
            raise ValueError("scene must be 'objects' or 'cup_table'")

        self._task_polls_until_succeeded = task_polls_until_succeeded
        self._fail_next_task = fail_next_task
        self._tasks: dict[str, _MockTask] = {}
        self.fail_rgb = fail_rgb
        self.fail_depth = fail_depth
        self.scene_name = scene
        # 深拷贝：否则会改到类属性 DEFAULT_EE_POSE，多个实例互相污染
        self._ee_pose: dict[str, list[float]] = {
            chain: list(pose) for chain, pose in self.DEFAULT_EE_POSE.items()
        }
        self._ee_moves: list[dict[str, object]] = []
        self._gripper_width: dict[str, float] = {
            "left_gripper": 0.11, "right_gripper": 0.11,
        }
        self._audio_calls: list[dict[str, object]] = []
        self._volume: float = 80.0
        self._base_moves: list[dict[str, object]] = []
        self._base_stops: int = 0
        self._base_pose: list[float] = [0.0, 0.0, 0.0]
        self._camera_seed = {
            "right_arm": list(self.get_sensor_extrinsic("RIGHT_ARM_CAMERA")[0]),
            "left_arm": list(self.get_sensor_extrinsic("LEFT_ARM_CAMERA")[0]),
        }

    @property
    def mode(self) -> RunMode:
        return RunMode.MOCK

    def get_robot_state(self) -> ToolResult:
        active_task_ids = [
            task_id
            for task_id, task in self._tasks.items()
            if task.status is TaskStatus.RUNNING
        ]
        unknown_task_ids = [
            task_id
            for task_id, task in self._tasks.items()
            if task.status is TaskStatus.UNKNOWN
        ]
        motion_state = (
            "unknown"
            if unknown_task_ids
            else "moving"
            if active_task_ids
            else "idle"
        )
        data = {
            "connection_state": "simulated",
            "motion_state": motion_state,
            "active_task_ids": active_task_ids,
            "unknown_task_ids": unknown_task_ids,
        }
        if unknown_task_ids:
            return ToolResult(
                status=TaskStatus.UNKNOWN,
                mode=self.mode,
                message="Mock robot motion state is uncertain; it must not be treated as stopped.",
                task_id=unknown_task_ids[0] if len(unknown_task_ids) == 1 else None,
                data=data,
                error_code=ErrorCode.STATE_UNCERTAIN,
            )
        return ok(
            "Mock robot state retrieved.",
            mode=self.mode,
            data=data,
        )

    def execute_gesture(self, gesture_id: str) -> ToolResult:
        invalid = self._validate_registered_id(
            gesture_id, "gesture_id", SUPPORTED_GESTURES
        )
        if invalid is not None:
            return invalid
        return self._start_task("gesture", gesture_id)

    def navigate_to_place(self, place_id: str) -> ToolResult:
        invalid = self._validate_registered_id(
            place_id, "place_id", SUPPORTED_PLACES
        )
        if invalid is not None:
            return invalid
        return self._start_task("navigation", place_id)

    def get_map_pose(self) -> ToolResult:
        return ok(
            "Mock map pose retrieved.",
            self.mode,
            data={"pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "frame": "map"},
        )

    def check_path_reachability(self, goal_pose, start_pose=None) -> ToolResult:
        goal = [float(v) for v in list(goal_pose)]
        if len(goal) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "goal_pose must contain 7 values.",
                mode=self.mode,
            )
        return ok(
            "Mock path is reachable.",
            self.mode,
            data={"reachable": True, "goal_pose": goal},
        )

    def navigate_to_pose(
        self,
        pose,
        *,
        frame="map",
        speed_ratio=0.5,
        enable_collision_check=True,
    ) -> ToolResult:
        target = [float(v) for v in list(pose)]
        if len(target) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose must contain 7 values.",
                mode=self.mode,
            )
        return self._start_task("navigation_pose", str(target))

    def get_task_status(self, task_id: str) -> ToolResult:
        invalid = self._validate_task_id(task_id)
        if invalid is not None:
            return invalid

        task = self._tasks.get(task_id)
        if task is None:
            return failed(
                ErrorCode.UNKNOWN_TASK,
                f"Unknown mock task: {task_id}.",
                mode=self.mode,
                task_id=task_id,
            )
        if task.status is TaskStatus.UNKNOWN:
            return ToolResult(
                status=TaskStatus.UNKNOWN,
                mode=self.mode,
                message="Mock task state is uncertain; it must not be treated as stopped.",
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
                error_code=ErrorCode.STATE_UNCERTAIN,
            )
        if task.status is TaskStatus.CANCELLED:
            return canceled(
                "Mock task was cancelled.",
                mode=self.mode,
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
            )
        if task.status is TaskStatus.FAILED:
            return failed(
                ErrorCode.SDK_ERROR,
                task.failure_message or "Mock task failed.",
                mode=self.mode,
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
            )
        if task.status is TaskStatus.SUCCEEDED:
            return ok(
                "Mock task succeeded.",
                mode=self.mode,
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
            )

        if task.failure_message is not None:
            task.status = TaskStatus.FAILED
            return failed(
                ErrorCode.SDK_ERROR,
                task.failure_message,
                mode=self.mode,
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
            )
        if task.polls_remaining:
            task.polls_remaining -= 1
            return running(
                "Mock task is still running.",
                mode=self.mode,
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
            )

        task.status = TaskStatus.SUCCEEDED
        return ok(
            "Mock task succeeded.",
            mode=self.mode,
            task_id=task_id,
            data={"operation": task.operation, "target": task.target},
        )

    def cancel_task(self, task_id: str) -> ToolResult:
        invalid = self._validate_task_id(task_id)
        if invalid is not None:
            return invalid

        task = self._tasks.get(task_id)
        if task is None:
            return failed(
                ErrorCode.UNKNOWN_TASK,
                f"Unknown mock task: {task_id}.",
                mode=self.mode,
                task_id=task_id,
            )
        if task.status is TaskStatus.UNKNOWN:
            return ToolResult(
                status=TaskStatus.UNKNOWN,
                mode=self.mode,
                message="Mock task state is uncertain; cancellation cannot be confirmed.",
                task_id=task_id,
                data={"operation": task.operation, "target": task.target},
                error_code=ErrorCode.STATE_UNCERTAIN,
            )
        if task.status is not TaskStatus.RUNNING:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock task {task_id} is not running.",
                mode=self.mode,
                data={"task_status": task.status.value},
            )

        task.status = TaskStatus.CANCELLED
        return canceled(
            "Mock task cancellation confirmed.",
            mode=self.mode,
            task_id=task_id,
            data={"operation": task.operation, "target": task.target},
        )

    def simulate_unknown_task_state(self, task_id: str) -> None:
        """Set a known task to an uncertain state for tests and offline demos."""
        task = self._tasks.get(task_id)
        if task is None:
            raise KeyError(f"Unknown mock task: {task_id}")
        task.status = TaskStatus.UNKNOWN

    # ------------------------------------------------------------------ #
    # 手臂运动原语（虚拟实现，用于彩排闭环）
    # ------------------------------------------------------------------ #
    #: 初始终端位姿（base_link 下 [x,y,z,qx,qy,qz,qw]）。左臂抬在身前。
    DEFAULT_EE_POSE: dict[str, list[float]] = {
        "left_arm": [0.45, 0.20, 0.85, 0.0, 0.0, 0.0, 1.0],
        "right_arm": [0.45, -0.20, 0.85, 0.0, 0.0, 0.0, 1.0],
    }

    def get_end_effector_pose(self, chain: str = "left_arm") -> ToolResult:
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链的位姿：{chain!r}",
                mode=self.mode,
                data={"supported_ids": sorted(self._ee_pose)},
            )
        return ok(
            f"Mock {chain} 末端位姿",
            mode=self.mode,
            data={"chain": chain, "pose": list(self._ee_pose[chain]),
                  "frame": "base_link", "unit": "m"},
        )

    def get_chain_frame_pose(
        self,
        chain: str = "right_arm",
        frame_id: str = "Camera",
        reference_frame: str = "base_link",
    ) -> ToolResult:
        if frame_id not in {"EndEffector", "Camera", "TCP"}:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 不支持坐标系帧：{frame_id!r}",
                mode=self.mode,
            )
        if frame_id == "Camera":
            if chain not in self._ee_pose:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"Mock 未记录该链的位姿：{chain!r}",
                    mode=self.mode,
                )
            from sdk_wrapper.dev.algorithms import geometry as geo

            initial_ee = geo.extrinsic_to_matrix(self.DEFAULT_EE_POSE[chain])
            ee_from_camera = np.linalg.inv(initial_ee) @ geo.extrinsic_to_matrix(
                self._camera_seed[chain]
            )
            current_camera = geo.extrinsic_to_matrix(self._ee_pose[chain]) @ ee_from_camera
            qx, qy, qz, qw = geo.matrix_to_quaternion(current_camera[:3, :3])
            pose = [
                float(current_camera[0, 3]),
                float(current_camera[1, 3]),
                float(current_camera[2, 3]),
                qx,
                qy,
                qz,
                qw,
            ]
            timestamp_ns = 0
        else:
            if chain not in self._ee_pose:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"Mock 未记录该链的位姿：{chain!r}",
                    mode=self.mode,
                )
            pose = list(self._ee_pose[chain])
            timestamp_ns = 0
        return ok(
            f"Mock {chain}/{frame_id} 位姿",
            mode=self.mode,
            data={
                "chain": chain,
                "frame_id": frame_id,
                "reference_frame": reference_frame,
                "pose": list(pose),
                "timestamp_ns": int(timestamp_ns),
                "unit": "m",
            },
        )

    def check_end_effector_reachable(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链：{chain!r}",
                mode=self.mode,
            )
        target = [float(v) for v in list(pose)]
        if len(target) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose 必须是 7 个数值",
                mode=self.mode,
            )
        return ok(
            f"Mock {chain} 目标位姿可解",
            mode=self.mode,
            data={
                "chain": chain,
                "target_pose": target,
                "reachable": True,
                "collision_check": bool(enable_collision_check),
            },
        )

    def move_ee_delta(
        self,
        chain: str,
        dx: float,
        dy: float,
        dz: float,
        *,
        straight_line: bool = True,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """虚拟平移：记录轨迹，不碰硬件。用于把"推/抓"的闭环逻辑先跑通。"""
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链的位姿：{chain!r}",
                mode=self.mode,
                data={"supported_ids": sorted(self._ee_pose)},
            )
        try:
            dx, dy, dz = float(dx), float(dy), float(dz)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "dx/dy/dz 必须是数值", mode=self.mode)
        if max(abs(dx), abs(dy), abs(dz)) > 0.5:
            # 单次末端平移超过 0.5 m 一律拒绝：真实机器上这是危险操作
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次末端平移 {[dx, dy, dz]} m 超过 0.5 m 上限",
                mode=self.mode,
            )

        pose = self._ee_pose[chain]
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        self._ee_moves.append(
            {"chain": chain, "delta": [dx, dy, dz], "straight_line": bool(straight_line)}
        )
        # 粗略物理：俯视场景里把最近的杯子推走，好让闭环复测看到距离真的变了
        self.push_in_scene(dx, dy)
        return ok(
            f"Mock {chain} 末端平移完成",
            mode=self.mode,
            data={"chain": chain, "delta": [dx, dy, dz], "target_pose": list(pose),
                  "straight_line": bool(straight_line), "motion_status": "SUCCESS"},
        )

    def move_along_gripper(
        self,
        chain: str,
        distance_m: float,
        *,
        actuate: str = "with_torso",
        auto_lift: bool = True,
        max_step_m: float = 0.05,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """虚拟沿夹爪方向平移：按姿态算轴后改写虚拟位姿，不碰硬件。

        校验与真机保持同一套（行程上限 / actuate 合法 / 单步限幅），
        免得编排逻辑在 mock 上跑通、换真机才突然被拒。抬升在 mock 里不模拟
        （没有碰撞概念），所以 ``lifted_m`` 恒为 0。
        """
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链的位姿：{chain!r}",
                mode=self.mode,
                data={"supported_ids": sorted(self._ee_pose)},
            )
        try:
            distance_m = float(distance_m)
            max_step_m = float(max_step_m)
        except (TypeError, ValueError):
            return rejected(
                ErrorCode.INVALID_ARGUMENT, "distance_m / max_step_m 必须是数值", mode=self.mode
            )
        if not math.isfinite(distance_m):
            return rejected(ErrorCode.INVALID_ARGUMENT, "distance_m 必须是有限数值", mode=self.mode)
        if abs(distance_m) < 0.005:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"distance_m={distance_m:.4f} 太小；单次至少 5 mm（真机在更小的步长上会卡住）",
                mode=self.mode,
            )
        if abs(distance_m) > ALONG_MAX_TRAVEL_M:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次沿夹爪行程 {distance_m:.3f} m 超过上限 {ALONG_MAX_TRAVEL_M:.2f} m",
                mode=self.mode,
                data={"max_travel_m": ALONG_MAX_TRAVEL_M},
            )
        if actuate not in ALONG_SAFE_Z:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"未知 actuate {actuate!r}",
                mode=self.mode,
                data={"legal_actuate": sorted(ALONG_SAFE_Z)},
            )
        if not (0.0 < max_step_m <= 0.05):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"max_step_m 必须在 (0, 0.05] 内，收到 {max_step_m}",
                mode=self.mode,
            )

        pose = self._ee_pose[chain]
        rot = quat_to_rot(pose[3:])
        axis = (rot[0][0], rot[1][0], rot[2][0])
        start = list(pose)
        for j in range(3):
            pose[j] += distance_m * axis[j]
        self._ee_moves.append(
            {"chain": chain, "along_gripper": distance_m, "axis": list(axis), "actuate": actuate}
        )
        self.push_in_scene(distance_m * axis[0], distance_m * axis[1])
        return ok(
            f"Mock {chain} 沿夹爪{'伸出' if distance_m > 0 else '后退'} {abs(distance_m):.3f} m",
            mode=self.mode,
            data={
                "chain": chain,
                "requested_m": distance_m,
                "executed_m": round(distance_m, 4),
                "actuate": actuate,
                "gripper_axis_base": [round(v, 5) for v in axis],
                "start_pose": [round(v, 5) for v in start],
                "end_pose": [round(v, 5) for v in pose],
                "lifted_m": 0.0,
                "steps": max(1, math.ceil(abs(distance_m) / max_step_m)),
                "motion_status": "SUCCESS",
            },
        )

    def rotate_ee(
        self,
        chain: str,
        droll_deg: float,
        dpitch_deg: float,
        dyaw_deg: float,
        *,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """虚拟旋转：只记录，不改位姿（mock 不管姿态）。"""
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链的位姿：{chain!r}",
                mode=self.mode,
                data={"supported_ids": sorted(self._ee_pose)},
            )
        delta = [float(droll_deg), float(dpitch_deg), float(dyaw_deg)]
        self._ee_moves.append({"chain": chain, "rotate_deg": delta})
        return ok(
            f"Mock {chain} 末端旋转完成",
            mode=self.mode,
            data={"chain": chain, "delta_deg": delta, "motion_status": "SUCCESS"},
        )

    def move_ee_to(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        straight_line: bool = True,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """虚拟绝对移动：直接改写虚拟末端位姿（不碰硬件、不改变杯子位置）。"""
        if chain not in self._ee_pose:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 未记录该链的位姿：{chain!r}",
                mode=self.mode,
                data={"supported_ids": sorted(self._ee_pose)},
            )
        target = [float(v) for v in list(pose)]
        if len(target) != 7:
            return rejected(ErrorCode.INVALID_ARGUMENT, "pose 必须是 7 个数值", mode=self.mode)
        self._ee_pose[chain] = target
        self._ee_moves.append({"chain": chain, "target": target, "absolute": True})
        return ok(
            f"Mock {chain} 已到指定位姿",
            mode=self.mode,
            data={"chain": chain, "target_pose": target, "motion_status": "SUCCESS"},
        )

    def set_gripper_width(
        self,
        end_effector: str = "left_gripper",
        width_m: float = 0.0,
        *,
        velocity_mps: float = 0.05,
        effort: float = 10.0,
        is_blocking: bool = True,
    ) -> ToolResult:
        if "gripper" not in end_effector:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Mock 只模拟夹爪：{end_effector!r}",
                mode=self.mode,
                data={"supported_ids": ["left_gripper", "right_gripper"]},
            )
        try:
            width_m = float(width_m)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "width_m 必须是数值", mode=self.mode)
        if not 0.0 <= width_m <= 0.11:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"夹爪开度 {width_m} m 超出 [0, 0.11] 合理范围",
                mode=self.mode,
            )
        self._gripper_width[end_effector] = width_m
        return ok(
            f"Mock {end_effector} 开度设为 {width_m:.3f} m",
            mode=self.mode,
            data={"end_effector": end_effector, "width_m": width_m, "control_status": "SUCCESS"},
        )

    #: 供测试/彩排断言"推了几次、夹爪闭到多少"
    # ══════════════════════════════════════════════════════════════════ #
    # 底盘直控（并入原 wrapper 能力）
    # ══════════════════════════════════════════════════════════════════ #
    def move_base(self, distance_m: float, yaw_rad: float = 0.0, *,
                  timeout_s: float = 20.0, retries: int = 1) -> ToolResult:
        """模拟底盘直控：累计位姿，便于断言"确实走了这么远"。"""
        try:
            distance_m = float(distance_m)
            yaw_rad = float(yaw_rad)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT,
                            "distance_m / yaw_rad 必须是数值", mode=self.mode)
        self._base_moves.append({"distance_m": distance_m, "yaw_rad": yaw_rad})
        yaw0 = self._base_pose[2]
        self._base_pose = [
            self._base_pose[0] + distance_m * math.cos(yaw0),
            self._base_pose[1] + distance_m * math.sin(yaw0),
            yaw0 + yaw_rad,
        ]
        return ok(
            f"底盘已移动：{distance_m * 100:.1f} cm / {math.degrees(yaw_rad):.1f}°",
            self.mode,
            data={"distance_m": distance_m, "yaw_rad": yaw_rad,
                  "control_status": "SUCCESS", "base_pose": list(self._base_pose)},
        )

    def stop_base(self) -> ToolResult:
        self._base_stops += 1
        return ok("底盘已停止", self.mode, data={"stopped": True})

    def read_sensor(self, sensor_name: str) -> ToolResult:
        return rejected(
            ErrorCode.NOT_SUPPORTED,
            f"Mock 不支持任意传感器 {sensor_name!r}；相机请用 right_arm / left_arm",
            mode=self.mode, data={"sensor": sensor_name},
        )

    @property
    def base_moves(self) -> list[dict[str, object]]:
        """已下发的底盘动作列表（断言用）。"""
        return list(self._base_moves)

    @property
    def base_stops(self) -> int:
        return self._base_stops

    @property
    def ee_moves(self) -> list[dict[str, object]]:
        return list(self._ee_moves)

    @property
    def gripper_widths(self) -> dict[str, float]:
        return dict(self._gripper_width)

    # ------------------------------------------------------------------ #
    # 语音（记录调用，供彩排断言"提醒播过了"）
    # ------------------------------------------------------------------ #
    def write_audio_stream_output(self, audio_chunk: bytes, stream_id: str = "") -> bool:
        self._audio_calls.append(
            {"stream": str(stream_id), "bytes": len(bytes(audio_chunk or b""))}
        )
        return True

    def set_volume(self, volume: float) -> bool:
        self._volume = float(volume)
        return True

    @property
    def audio_bytes_played(self) -> int:
        return sum(int(call["bytes"]) for call in self._audio_calls)

    @property
    def audio_streams(self) -> list[str]:
        return [str(call["stream"]) for call in self._audio_calls]

    def _start_task(self, operation: str, target: str) -> ToolResult:
        task_id = str(uuid.uuid4())
        failure_message = None
        if self._fail_next_task:
            failure_message = "Simulated mock execution failure."
            self._fail_next_task = False

        self._tasks[task_id] = _MockTask(
            operation=operation,
            target=target,
            polls_remaining=self._task_polls_until_succeeded,
            failure_message=failure_message,
        )
        return running(
            f"Mock {operation} task started.",
            mode=self.mode,
            task_id=task_id,
            data={"operation": operation, "target": target},
        )

    def _validate_registered_id(
        self, value: str, parameter: str, supported_ids: frozenset[str]
    ) -> ToolResult | None:
        if not isinstance(value, str) or not value.strip():
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"{parameter} must be a non-empty string.",
                mode=self.mode,
                data={"parameter": parameter},
            )
        if value not in supported_ids:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"Unsupported {parameter}: {value!r}.",
                mode=self.mode,
                data={"parameter": parameter, "supported_ids": sorted(supported_ids)},
            )
        return None

    def _validate_task_id(self, task_id: str) -> ToolResult | None:
        if not isinstance(task_id, str) or not task_id.strip():
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "task_id must be a non-empty string.",
                mode=self.mode,
                data={"parameter": "task_id"},
            )
        return None
