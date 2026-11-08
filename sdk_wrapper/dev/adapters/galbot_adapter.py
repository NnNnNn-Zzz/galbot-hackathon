"""真机适配器：把 GalbotSDK 绑到 ``RobotAdapter`` 上。

════════════════════════════════════════════════════════════════════════════
⚠️ 关键：官方 SDK 是**三个独立单例类**，不是一个大类
════════════════════════════════════════════════════════════════════════════
| 类 | 负责 | 本文件用法 |
|---|---|---|
| ``GalbotRobot``      | 相机、本体状态、夹爪、关节设置、shutdown | `robot.*` |
| ``GalbotMotion``     | 运动规划：`set_end_effector_pose` / `move_line` / `get_end_effector_pose_on_chain` | `motion.*` |
| ``GalbotNavigation`` | 导航：`set_navigation_target` / `get_navigation_target_status` / `stop_navigation` / `is_localized` | `nav.*` |

把导航方法写到 ``GalbotRobot`` 上是会 ``AttributeError`` 的 —— 这是本文件曾经的 bug。

════════════════════════════════════════════════════════════════════════════
已核实的 SDK 事实（GalbotSDK V1.10.0，g1/__init__.pyi + 官方示例）
════════════════════════════════════════════════════════════════════════════
GalbotMotion (953)
  init() -> bool                                                              :1343
  get_end_effector_pose_on_chain(chain, "EndEffector", "base_link") -> (MotionStatus, [7])  :1213
  set_end_effector_pose(pose[7], chain, "base_link", ...) -> MotionStatus      :1622
  move_line(waypoints, params: Parameter) -> (MotionStatus, dict)              :1584
  get_chain_joint_names(chain) -> list[str]
GalbotNavigation (1740)
  init() -> bool                                                              :1861
  is_localized() -> bool                                                      :1871
  set_navigation_target(Pose, frame, speed_ratio, enable_collision_check) -> TaskHandle :2072
  get_navigation_target_status(task_id) -> NavigationTaskSnapshot              :1850
  stop_navigation() -> (success, status_string)  ⚠️ 不代表已停                :2123
  navigate_to_goal(pose[7], ...) -> (success, status_string)                  :1944
GalbotRobot (2222)
  init(enable_sensor_set) -> bool                                             :2772
  get_rgb_data(cam, RgbOutputFormat.JPEG, True) -> dict                        :2650
  get_depth_data(cam) -> dict                                                 :2391
  get_camera_intrinsic(cam) -> dict（Python dict 用大写 K / D）                :2354
  get_sensor_extrinsic(cam, "base_link") -> ([7], ts)                         :2673
  get_device_information() / get_bms_information() / get_base_velocity() / is_running()
  set_gripper_command(G1JointGroup.left_gripper, width_m, vel, effort, blocking) :3064
  set_joint_positions(...)                                                    :3132
  request_shutdown() / wait_for_shutdown() / destroy()                :2839/3351/2259

⚠️ 官方示例（move_line.py）里给 ``move_line`` 传的是 ``Parameter()``（4210），
   而 ``is_relative_pose`` 在 ``PlannerConfig``（4445）上、``Parameter`` **没有**这个开关。
   所以**不要依赖"相对位姿"**：本文件的做法是"读当前位姿 → 加位移 → 下发绝对目标"，
   这正是官方 move_line.py / example6_pick_and_place.py 的做法。

════════════════════════════════════════════════════════════════════════════
未伪造的能力缺口
════════════════════════════════════════════════════════════════════════════
``execute_gesture``：SDK 没有高层"挥手/鼓掌"接口，只有 ``set_joint_positions`` /
``execute_joint_trajectory``。需现场示教关节角并登记到 ``config/gestures.json``；
在此之前返回 ``rejected + not_supported``。
"""

from __future__ import annotations

import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
import typing

from sdk_wrapper.dev.adapters.base import (
    ALONG_FLOOR_Z,
    ALONG_MAX_TRAVEL_M,
    ALONG_SAFE_Z,
    RobotAdapter,
    SUPPORTED_GESTURES,
    quat_to_rot,
)
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
    unknown,
)

LOG = logging.getLogger(__name__)

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
_DEFAULT_PLACES_PATH = os.path.join(_CONFIG_DIR, "places.json")

#: 初始化时启用的传感器。只开真正要用的，减少开销（g1/__init__.pyi:2779）
DEFAULT_SENSORS: tuple[str, ...] = (
    "LEFT_ARM_CAMERA",
    "RIGHT_ARM_CAMERA",
    "LEFT_ARM_DEPTH_CAMERA",
    "RIGHT_ARM_DEPTH_CAMERA",
    "HEAD_LEFT_CAMERA",
)

#: 判定"机器人在动"的速度阈值（m/s 或 rad/s）
_MOTION_EPS = 0.02

#: "with_torso" 让躯干协助手臂到达更高/更远的点（官方 move_line.py 用的就是这个）
_ACTUATE = "with_torso"

#: 沿夹爪方向的推荐执行高度（米），按参与的链分档。真机实测：
#: 纯手臂伸展时肘部下沉更多，z=0.834 时前臂扫掠会被判碰撞（第 4 步 FAULT），
#: 抬到 0.865 才走完全程；带躯干 0.85 就够。两档别混用同一个阈值。
#: 定义在 base 里，与 MockAdapter 共用，防止两边漂移。
_ALONG_SAFE_Z = ALONG_SAFE_Z

#: 沿夹爪方向单次调用的总行程上限（米）。超过请拆多次，避免一次走太远无法回退。
_ALONG_MAX_TRAVEL_M = ALONG_MAX_TRAVEL_M

#: 通用最低高度（米）。后退时用它做抬升门槛，见 base.ALONG_FLOOR_Z。
_ALONG_FLOOR_Z = ALONG_FLOOR_Z

#: NavigationTaskStatus -> (协议 status, error_code)  见 g1/__init__.pyi:4090-4133
_NAV_STATUS_MAP: dict[str, tuple[TaskStatus, ErrorCode | None]] = {
    "SUCCESS": (TaskStatus.SUCCEEDED, None),
    "RUNNING": (TaskStatus.RUNNING, None),
    "UNKNOWN": (TaskStatus.UNKNOWN, ErrorCode.STATE_UNCERTAIN),
    "FAILED": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
    "INTERRUPTED": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
    "OCCUPIED": (TaskStatus.FAILED, ErrorCode.SAFETY_LIMIT),
    "COLLISION": (TaskStatus.FAILED, ErrorCode.SAFETY_LIMIT),
    "CLOSE_TO_OBSTACLE": (TaskStatus.FAILED, ErrorCode.SAFETY_LIMIT),
}

#: MotionStatus -> (协议 status, error_code)  见 g1/__init__.pyi:4020
_MOTION_STATUS_MAP: dict[str, tuple[TaskStatus, ErrorCode | None]] = {
    "SUCCESS": (TaskStatus.SUCCEEDED, None),
    "IN_PROGRESS": (TaskStatus.RUNNING, None),
    "TIMEOUT": (TaskStatus.FAILED, ErrorCode.TIMEOUT),
    "INVALID_INPUT": (TaskStatus.REJECTED, ErrorCode.INVALID_ARGUMENT),
    "INIT_FAILED": (TaskStatus.FAILED, ErrorCode.NOT_READY),
    "COMM_DISCONNECTED": (TaskStatus.UNKNOWN, ErrorCode.STATE_UNCERTAIN),
    "FAULT": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
    "STOPPED_UNREACHED": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
    "UNSUPPORTED_FUNCRION": (TaskStatus.REJECTED, ErrorCode.NOT_SUPPORTED),
    "DATA_FETCH_FAILED": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
    "PUBLISH_FAIL": (TaskStatus.FAILED, ErrorCode.SDK_ERROR),
}


#: 底盘单次位移硬上限（米）。软限幅在 toolhub / wrapper 层（safety_config.json）。
_BASE_MAX_DISTANCE_M = 2.0

#: 底盘单次转向硬上限（度）。
_BASE_MAX_TURN_DEG = 180.0


def _sdk_bootstrap() -> str:
    """把 GalbotSDK 的 Python 包目录注入 sys.path（PYTHONPATH 已提供则跳过）。"""
    import glob
    import importlib.util
    import platform as _platform

    if importlib.util.find_spec("galbot_sdk") is not None:
        return ""
    sdk_root = os.environ.get("GALBOT_SDK_ROOT", os.path.expanduser("~/GalbotSDK"))
    arch = _platform.machine()
    candidates = sorted(glob.glob(
        os.path.join(sdk_root, "galbot_sdk", f"linux-{arch}-gcc*", "lib", "python")))
    if not candidates:
        return ""
    pkg_dir = candidates[0]
    if pkg_dir not in sys.path:
        sys.path.insert(0, pkg_dir)
    return pkg_dir


def _ensure_nav_service() -> None:
    """确保导航规划服务 service_navigation_plan 在跑。

    launcher.service 有时没启动它（实测场地机器人就没起），导致所有导航命令返回
    WAIT_INITIALIZED。检测缺失则自动拉起（仅 HPU 真机存在该二进制时生效）。
    """
    nav_svc = "/data/galbot/bin/service_navigation_plan"
    if not os.path.exists(nav_svc):
        return
    try:
        probe = subprocess.run(["pgrep", "-f", "service_navigation_plan"],
                               capture_output=True, timeout=5)
        if probe.returncode == 0:
            return
    except Exception:  # noqa: BLE001 - 探测失败就继续尝试拉起
        pass
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/data/galbot/lib:" + env.get("LD_LIBRARY_PATH", "")
    subprocess.Popen([nav_svc], env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    LOG.info("已启动 service_navigation_plan（launcher 未自动拉起，手动补启）")
    time.sleep(3.0)


def _enum_name(value: typing.Any) -> str:
    """把 pybind 枚举转成成员名。"""
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    text = str(value)
    return text.rsplit(".", 1)[-1] if "." in text else text


class GalbotAdapter(RobotAdapter):
    """真机适配器。

    双重闸门（协议 §1「默认禁用」）：
      * ``enable_real_control`` —— 允许 import / 初始化 SDK；
      * ``real_authorized``     —— 允许真正下发行走/动作指令。
    两者都 False 时任何调用都返回 ``rejected + permission_denied``，**不 import SDK**。
    """

    def __init__(
        self,
        *,
        enable_real_control: bool = False,
        real_authorized: bool = False,
        sensors: typing.Sequence[str] | None = None,
        places_path: str | os.PathLike[str] | None = None,
    ) -> None:
        if not isinstance(enable_real_control, bool):
            raise TypeError("enable_real_control must be a bool")
        if not isinstance(real_authorized, bool):
            raise TypeError("real_authorized must be a bool")

        self._enabled = enable_real_control
        self._authorized = real_authorized
        self._sensors: tuple[str, ...] = tuple(sensors) if sensors else DEFAULT_SENSORS
        self._places_path = os.fspath(places_path) if places_path else _DEFAULT_PLACES_PATH
        self._places: dict[str, dict[str, typing.Any]] | None = None

        self._robot: typing.Any = None
        self._motion: typing.Any = None
        self._navigation: typing.Any = None
        self._sdk: typing.Any = None
        self._lock = threading.RLock()
        self._chassis_ready = False
        self._nav_tasks: dict[str, dict[str, typing.Any]] = {}

    # ══════════════════════════════════════════════════════════════════ #
    # 生命周期
    # ══════════════════════════════════════════════════════════════════ #
    @property
    def mode(self) -> RunMode:
        return RunMode.REAL

    def _authorization_rejection(self) -> ToolResult | None:
        if not self._enabled or not self._authorized:
            return rejected(
                ErrorCode.PERMISSION_DENIED,
                "真机控制未授权：需要同时设置 enable_real_control=True 和 real_authorized=True"
                "（CLI 用 --allow-real-control）。未初始化 SDK。",
                mode=self.mode,
            )
        return None

    def _import_sdk(self) -> typing.Any:
        """懒加载 galbot_sdk.g1。找不到包目录时先 bootstrap 一次再试。"""
        if self._sdk is not None:
            return self._sdk
        try:
            import galbot_sdk.g1 as gm  # noqa: PLC0415 - 故意延迟到真机才 import
        except Exception as first_exc:  # noqa: BLE001
            pkg_dir = ""
            try:
                pkg_dir = _sdk_bootstrap()
                import galbot_sdk.g1 as gm  # noqa: PLC0415, F811
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError(
                    "无法导入 galbot_sdk（该模块是 Linux x86_64 ELF，Windows 上不可用）。\n"
                    f"  原始错误：{first_exc}\n"
                    f"  bootstrap 后仍失败：{exc}（注入路径：{pkg_dir or '未找到'}）\n"
                    "  检查：export PYTHONPATH=/opt/galbot/galbot_sdk/linux-x86_64-gcc940/lib/python\n"
                    "        export LD_LIBRARY_PATH=/opt/galbot/galbot_sdk/linux-x86_64-gcc940/lib"
                ) from exc
        self._sdk = gm
        return gm

    def _ensure_interfaces(
        self, *, need_nav: bool = True
    ) -> tuple[typing.Any, typing.Any, typing.Any]:
        """一次性建好三个单例：Motion → (Navigation) → Robot。

        顺序照官方 move_line.py（先 motion.init() 再 robot.init()）；
        官方 example6 还会 nav.init()。

        ``need_nav=False`` 时**跳过 Navigation** —— 底盘直控 / 机械臂 / 夹爪 / 相机
        都不需要导航服务。这是"底盘能单独动"的关键：导航后端
        ``service_navigation_plan`` 没起来时，不该把整机初始化一起拖死。
        真需要导航时会自动补建（见 ``_ensure_navigation``）。
        """
        with self._lock:
            if self._robot is not None:
                if need_nav:
                    self._ensure_navigation()
                return self._robot, self._motion, self._navigation

            gm = self._import_sdk()
            members = gm.SensorType.__members__
            bad = [name for name in self._sensors if name not in members]
            if bad:
                raise RuntimeError(f"未知的 SensorType 名 {bad}；可用名见 galbot_sdk.g1.SensorType")

            motion = gm.GalbotMotion()
            if not motion.init():
                raise RuntimeError("GalbotMotion.init() 返回 False")
            self._motion = motion

            navigation = self._ensure_navigation() if need_nav else None

            robot = gm.GalbotRobot()
            if not robot.init({members[name] for name in self._sensors}):
                raise RuntimeError("GalbotRobot.init() 返回 False（通信总线未建立）")

            self._robot, self._motion, self._navigation = robot, motion, navigation
            LOG.info("GalbotSDK 初始化完成（motion / navigation=%s / robot），传感器：%s",
                     navigation is not None, list(self._sensors))
            return robot, motion, navigation

    def close(self) -> None:
        """按 SDK 规定顺序释放：request_shutdown → wait_for_shutdown → destroy。

        官方示例只对 ``GalbotRobot`` 做这三步，``GalbotMotion`` / ``GalbotNavigation``
        没有独立 shutdown，因此这里保持与官方一致。
        """
        with self._lock:
            robot, self._robot = self._robot, None
            self._motion = None
            self._navigation = None
        if robot is None:
            return
        for name in ("request_shutdown", "wait_for_shutdown", "destroy"):
            func = getattr(robot, name, None)
            if not callable(func):
                continue
            try:
                func()
            except Exception as exc:  # noqa: BLE001 - 退出路径不该二次崩溃
                LOG.warning("GalbotRobot.%s() 失败：%s", name, exc)
        LOG.info("GalbotSDK 资源已释放")

    # ══════════════════════════════════════════════════════════════════ #
    # 相机：原样透传，不解码
    # ══════════════════════════════════════════════════════════════════ #
    def _sensor(self, sensor: str) -> typing.Any:
        gm = self._import_sdk()
        members = gm.SensorType.__members__
        if sensor not in members:
            raise ValueError(f"未知 SensorType {sensor!r}；可用：{sorted(members)}")
        return members[sensor]

    def get_rgb_data(self, sensor: str) -> typing.Any:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        fmt = self._import_sdk().RgbOutputFormat
        return robot.get_rgb_data(self._sensor(sensor), fmt.JPEG, True)

    def get_depth_data(self, sensor: str) -> typing.Any:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        return robot.get_depth_data(self._sensor(sensor))

    def get_camera_intrinsic(self, sensor: str) -> typing.Any:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        return robot.get_camera_intrinsic(self._sensor(sensor))

    def get_sensor_extrinsic(self, sensor: str, reference_frame: str = "base_link") -> typing.Any:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        return robot.get_sensor_extrinsic(self._sensor(sensor), reference_frame)

    # ══════════════════════════════════════════════════════════════════ #
    # 本体状态
    # ══════════════════════════════════════════════════════════════════ #
    def get_robot_state(self) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            robot, _motion, navigation = self._ensure_interfaces()
        except Exception as exc:  # noqa: BLE001
            return unknown(ErrorCode.STATE_UNCERTAIN, f"SDK 初始化失败：{exc}", mode=self.mode)

        try:
            device = robot.get_device_information() or {}
            bms = robot.get_bms_information() or {}
            velocity = robot.get_base_velocity() or {}
            alive = bool(robot.is_running())
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN, f"读取状态时抛错：{exc}；实际状态未知", mode=self.mode
            )

        localized: bool | None
        try:
            localized = bool(navigation.is_localized())
        except Exception:  # noqa: BLE001 - 定位查询失败不算致命
            localized = None

        linear = list(velocity.get("linear_velocity") or [0.0, 0.0, 0.0])
        angular = list(velocity.get("angular_velocity") or [0.0, 0.0, 0.0])
        speed = max([abs(float(v)) for v in linear + angular] or [0.0])
        with self._lock:
            active = [
                task_id
                for task_id, task in self._nav_tasks.items()
                if task["status"] is TaskStatus.RUNNING
            ]
        motion_state = "moving" if (speed > _MOTION_EPS or active) else "idle"

        data: dict[str, typing.Any] = {
            "robot_connected": alive,
            "connection_state": "connected" if alive else "disconnected",
            "model": device.get("model"),
            "serial_number": device.get("serial_number"),
            "firmware_version": device.get("firmware_version"),
            "battery_percent": bms.get("battery_level"),
            "charging": bms.get("charging_status"),
            "motion_state": motion_state,
            "base_speed": round(speed, 4),
            "localized": localized,
            "active_task_ids": active,
            "sensors_enabled": list(self._sensors),
        }
        if not alive:
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                "SDK 报告系统未在运行；机器人实际状态不可知，不得视为已停止",
                mode=self.mode,
                data=data,
            )
        return ok("真机状态读取成功", mode=self.mode, data=data)

    # ══════════════════════════════════════════════════════════════════ #
    # 动作（当前能力缺口，显式拒绝）
    # ══════════════════════════════════════════════════════════════════ #
    def execute_gesture(self, gesture_id: str) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        if gesture_id not in SUPPORTED_GESTURES:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"未登记的动作 {gesture_id!r}",
                mode=self.mode,
                data={"gesture_id": gesture_id, "supported_ids": sorted(SUPPORTED_GESTURES)},
            )
        return rejected(
            ErrorCode.NOT_SUPPORTED,
            "预设动作在真机上尚未标定：SDK 没有高层'鼓掌/挥手'接口，只有 set_joint_positions。"
            "需先示教关节角并登记到 config/gestures.json 再接进来。接口保留，但不伪造执行成功。",
            mode=self.mode,
            data={"gesture_id": gesture_id},
        )

    # ══════════════════════════════════════════════════════════════════ #
    # 手臂运动原语（推/抓 都靠这三个）
    # ══════════════════════════════════════════════════════════════════ #
    def get_end_effector_pose(self, chain: str = "left_arm") -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            _robot, motion, _nav = self._ensure_interfaces()
            status, pose = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link")
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN, f"读取末端位姿失败：{exc}", mode=self.mode,
                data={"chain": chain},
            )
        name = _enum_name(status)
        if name != "SUCCESS" or len(list(pose)) != 7:
            return failed(
                ErrorCode.SDK_ERROR,
                f"get_end_effector_pose_on_chain({chain}) 返回 {name}，位姿长度 {len(list(pose))}",
                mode=self.mode,
                data={"chain": chain, "motion_status": name},
            )
        return ok(
            f"{chain} 末端位姿已读取",
            mode=self.mode,
            data={"chain": chain, "pose": [float(v) for v in pose], "frame": "base_link",
                  "unit": "m"},
        )

    def get_chain_frame_pose(
        self,
        chain: str = "right_arm",
        frame_id: str = "Camera",
        reference_frame: str = "base_link",
    ) -> ToolResult:
        """Read an SDK semantic frame without commanding motion."""
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        if frame_id not in {"EndEffector", "Camera", "TCP"}:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"unsupported chain frame {frame_id!r}",
                mode=self.mode,
                data={"supported": ["EndEffector", "Camera", "TCP"]},
            )
        try:
            _robot, motion, _nav = self._ensure_interfaces()
            status, pose = motion.get_end_effector_pose_on_chain(
                chain, frame_id, reference_frame
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"读取 {chain}/{frame_id} 位姿失败：{exc}",
                mode=self.mode,
                data={"chain": chain, "frame_id": frame_id},
            )
        name = _enum_name(status)
        pose = [float(v) for v in list(pose)]
        data = {
            "chain": chain,
            "frame_id": frame_id,
            "reference_frame": reference_frame,
            "motion_status": name,
            "pose": pose,
            "unit": "m",
        }
        if name == "SUCCESS" and len(pose) == 7:
            return ok(f"{chain}/{frame_id} 位姿已读取", mode=self.mode, data=data)
        if name == "INVALID_INPUT":
            return rejected(
                ErrorCode.NOT_READY,
                f"{chain}/{frame_id} 尚不可用；TCP 需要先 attach_tool",
                mode=self.mode,
                data=data,
            )
        return failed(
            ErrorCode.SDK_ERROR,
            f"读取 {chain}/{frame_id} 位姿失败（{name}）",
            mode=self.mode,
            data=data,
        )

    def _execute_cartesian(
        self,
        chain: str,
        target: typing.Sequence[float],
        *,
        straight_line: bool,
        enable_collision_check: bool,
        timeout_s: float,
        actuate: str = _ACTUATE,
    ) -> typing.Any:
        """下发一个绝对末端目标，返回 SDK 的 MotionStatus（异常向上抛）。

        ``actuate`` 决定参与规划的链：``with_torso`` 让躯干协助手臂，
        ``with_chain_only`` 只用该臂（躯干零动作）。注意 assist_chains 只在
        带躯干时设 —— 两个开关方向不一致会让规划器把躯干也算进来。
        """
        gm = self._import_sdk()
        _robot, motion, _nav = self._ensure_interfaces()
        target = [float(v) for v in target]
        if straight_line:
            target_obj = gm.MotionPlanChainTarget()
            target_obj.chain_name = chain
            target_obj.mode = gm.MotionPlanTargetMode.kCartesian
            target_obj.cart.chain_name = chain
            target_obj.cart.frame_id = "EndEffector"
            target_obj.cart.reference_frame = "base_link"
            target_obj.cart.pose = gm.Pose(target)
            if actuate == "with_torso":
                target_obj.cart.assist_chains = {"torso"}

            params = gm.Parameter()
            params.set_direct_execute(True)
            params.set_blocking(True)
            params.set_timeout(float(timeout_s))
            params.set_check_collision(bool(enable_collision_check))
            params.set_reference_frame("base_link")
            params.set_actuate(actuate)
            status, _traj = motion.move_line([[target_obj]], params)
            return status
        target_obj = gm.MotionPlanChainTarget()
        target_obj.chain_name = chain
        target_obj.mode = gm.MotionPlanTargetMode.kCartesian
        target_obj.cart.chain_name = chain
        target_obj.cart.frame_id = "EndEffector"
        target_obj.cart.reference_frame = "base_link"
        target_obj.cart.pose = gm.Pose(target)
        target_obj.cart.assist_chains = {"torso"}

        params = gm.PlannerConfig()
        params.is_direct_execute = True
        params.is_blocking = True
        params.timeout_second = float(timeout_s)
        params.is_check_collision = bool(enable_collision_check)
        params.enable_env_collision_check = bool(enable_collision_check)
        params.reference_frame = "base_link"
        status, _traj = motion.motion_plan([[target_obj]], params)
        return status

    def check_end_effector_reachable(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        """Run a read-only collision-aware path plan for an EndEffector target."""
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            target = [float(v) for v in list(pose)]
        except (TypeError, ValueError):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose 必须是 7 个数值",
                mode=self.mode,
            )
        if len(target) != 7 or not all(math.isfinite(v) for v in target):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose 必须是 7 个有限数值",
                mode=self.mode,
                data={"pose_length": len(target)},
            )
        try:
            _robot, motion, _nav = self._ensure_interfaces()
            gm = self._import_sdk()
            target_obj = gm.MotionPlanChainTarget()
            target_obj.chain_name = chain
            target_obj.mode = gm.MotionPlanTargetMode.kCartesian
            target_obj.cart.chain_name = chain
            target_obj.cart.frame_id = "EndEffector"
            target_obj.cart.reference_frame = "base_link"
            target_obj.cart.pose = gm.Pose(target)
            target_obj.cart.assist_chains = {"torso"}

            params = gm.PlannerConfig()
            params.is_direct_execute = False
            params.is_blocking = True
            params.timeout_second = 15.0
            params.is_check_collision = bool(enable_collision_check)
            params.enable_env_collision_check = bool(enable_collision_check)
            params.reference_frame = "base_link"
            status, trajectory = motion.motion_plan([[target_obj]], params)
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"只读 IK 预检异常：{exc}",
                mode=self.mode,
                data={"chain": chain, "target_pose": target},
            )
        name = _enum_name(status)
        data = {
            "chain": chain,
            "target_pose": target,
            "reachable": name == "SUCCESS",
            "collision_check": bool(enable_collision_check),
            "environment_collision_check": bool(enable_collision_check),
            "motion_status": name,
            "trajectory_points": {
                key: len(value) for key, value in trajectory.items()
            },
        }
        if name == "SUCCESS":
            return ok(
                f"{chain} 目标存在无碰撞自由空间路径",
                mode=self.mode,
                data=data,
            )
        code = (
            ErrorCode.INVALID_ARGUMENT
            if name == "INVALID_INPUT"
            else ErrorCode.SAFETY_LIMIT
        )
        return rejected(
            code,
            f"{chain} 目标位姿不可达或未通过碰撞预检（{name}）",
            mode=self.mode,
            data=data,
        )

    def _motion_result(
        self,
        status: typing.Any,
        *,
        chain: str,
        data: dict[str, typing.Any],
        on_success: str,
    ) -> ToolResult:
        name = _enum_name(status)
        data = dict(data, motion_status=name)
        task_status, code = _MOTION_STATUS_MAP.get(name, (TaskStatus.FAILED, ErrorCode.SDK_ERROR))
        if task_status is TaskStatus.SUCCEEDED:
            return ok(on_success, mode=self.mode, data=data)
        if task_status is TaskStatus.RUNNING:
            return running(
                f"{chain} 仍在移动（{name}）", mode=self.mode,
                task_id=f"motion-{int(time.monotonic() * 1000)}", data=data,
            )
        if task_status is TaskStatus.UNKNOWN:
            return unknown(
                code or ErrorCode.STATE_UNCERTAIN, f"运动状态不可知（{name}）",
                mode=self.mode, data=data,
            )
        return failed(code or ErrorCode.SDK_ERROR, f"末端移动失败（{name}）", mode=self.mode, data=data)

    def move_ee_to(
        self,
        chain: str,
        pose: typing.Sequence[float],
        *,
        straight_line: bool = True,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """移动到 base_link 下的绝对位姿。用于"回位再复测"。"""
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            target = [float(v) for v in list(pose)]
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "pose 必须是 7 个数值", mode=self.mode)
        if len(target) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"pose 必须是 [x,y,z,qx,qy,qz,qw]，收到 {len(target)} 个",
                mode=self.mode,
            )
        try:
            status = self._execute_cartesian(
                chain, target,
                straight_line=straight_line,
                enable_collision_check=enable_collision_check,
                timeout_s=timeout_s,
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"下发绝对移动时抛错：{exc}；机器人是否已移动未知",
                mode=self.mode,
                data={"chain": chain, "target_pose": target},
            )
        return self._motion_result(
            status, chain=chain,
            data={"chain": chain, "target_pose": target, "straight_line": bool(straight_line)},
            on_success=f"{chain} 已到位姿 {[round(v, 3) for v in target[:3]]}",
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
        """末端平移 (dx,dy,dz) 米，姿态不变。

        用官方示例的写法：读当前位姿 → 加位移 → 下发**绝对**目标。
        不用 `is_relative_pose`（它在 PlannerConfig 上，而 move_line 示例传的是
        没有该开关的 Parameter，不能依赖）。
        """
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            dx, dy, dz = float(dx), float(dy), float(dz)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT, "dx/dy/dz 必须是数值", mode=self.mode)

        try:
            _robot, motion, _nav = self._ensure_interfaces()
            status, pose = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link")
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN, f"移动前读位姿失败：{exc}；未下发运动", mode=self.mode,
                data={"chain": chain},
            )
        name = _enum_name(status)
        pose = list(pose)
        if name != "SUCCESS" or len(pose) != 7:
            return failed(
                ErrorCode.SDK_ERROR,
                f"移动前读位姿失败（{name}）；未下发运动",
                mode=self.mode,
                data={"chain": chain, "motion_status": name},
            )

        target = [pose[0] + dx, pose[1] + dy, pose[2] + dz,
                  pose[3], pose[4], pose[5], pose[6]]
        try:
            status = self._execute_cartesian(
                chain, target,
                straight_line=straight_line,
                enable_collision_check=enable_collision_check,
                timeout_s=timeout_s,
            )
        except Exception as exc:  # noqa: BLE001
            # 运动指令抛异常 ≠ 机器人没动
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"下发末端移动时抛错：{exc}；机器人是否已移动未知",
                mode=self.mode,
                data={"chain": chain, "delta": [dx, dy, dz]},
            )
        return self._motion_result(
            status, chain=chain,
            data={
                "chain": chain,
                "delta": [dx, dy, dz],
                "target_pose": target,
                "straight_line": bool(straight_line),
            },
            on_success=f"{chain} 末端平移完成",
        )

    def move_along_gripper(
        self,
        chain: str,
        distance_m: float,
        *,
        actuate: str = _ACTUATE,
        auto_lift: bool = False,
        max_step_m: float = 0.05,
        enable_collision_check: bool = True,
        timeout_s: float = 10.0,
    ) -> ToolResult:
        """沿夹爪指向（EndEffector 的 +X）直线走 ``distance_m`` 米，姿态不变。

        实现要点：SDK 的参考系只有 base_link/map/world，``is_relative_pose``
        也拿不到工具系，所以方向必须自己算 —— 读当前位姿，用旋转矩阵**第一列**
        当基向量换成 base_link 位移，再用 move_line 下发绝对目标。

        ``actuate`` 分两档，安全高度也分两档：纯手臂伸展时肘部下沉更多，
        沿用带躯干的高度会在中途被判碰撞（实测 z=0.834 时第 4 步 FAULT）。
        """
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection

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
                f"distance_m={distance_m:.4f} 太小；单次至少 5 mm。"
                f"毫米级位移没有实际意义，而且极小步长会让规划器卡住不返回",
                mode=self.mode,
            )
        if abs(distance_m) > _ALONG_MAX_TRAVEL_M:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次沿夹爪行程 {distance_m:.3f} m 超过上限 {_ALONG_MAX_TRAVEL_M:.2f} m；"
                f"请拆成多次，每次走完复测位姿再继续",
                mode=self.mode,
                data={"max_travel_m": _ALONG_MAX_TRAVEL_M},
            )
        if actuate not in _ALONG_SAFE_Z:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"未知 actuate {actuate!r}",
                mode=self.mode,
                data={"legal_actuate": sorted(_ALONG_SAFE_Z)},
            )
        if not (0.0 < max_step_m <= 0.05):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"max_step_m 必须在 (0, 0.05] 内，收到 {max_step_m}",
                mode=self.mode,
            )

        safe_z = _ALONG_SAFE_Z[actuate]

        try:
            _robot, motion, _nav = self._ensure_interfaces()
            status, pose = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link")
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN, f"移动前读位姿失败：{exc}；未下发运动", mode=self.mode,
                data={"chain": chain},
            )
        name = _enum_name(status)
        pose = [float(v) for v in pose]
        if name != "SUCCESS" or len(pose) != 7:
            return failed(
                ErrorCode.SDK_ERROR, f"移动前读位姿失败（{name}）；未下发运动",
                self.mode, data={"chain": chain, "motion_status": name},
            )

        start_pose = list(pose)
        quat = pose[3:]
        rot = quat_to_rot(quat)
        axis = (rot[0][0], rot[1][0], rot[2][0])
        predicted_end_z = pose[2] + distance_m * axis[2]
        #: 安全高度容差（米）。安全高度本身是经验值，差几毫米没有意义；
        #: 但为这几毫米单独下发一次运动指令会让规划器在极小位移上卡住
        #: （实测：从 0.8484 补到 0.8500 的 1.6 mm 一步，move_line 不再返回）。
        along_z_tol = 0.005
        # 前进：路径整体在下沉，按模式的建议高度算。
        # 后退：路径是往上升的，只有起点可能贴桌，按通用下限算就行 ——
        # 否则会为了"抬高"下发一次纯臂抬升，而手臂已伸展时这个动作会 FAULT，
        # 白白挡掉一个本来完全安全的退出动作。
        if distance_m > 0:
            lowest_z, gate_z = min(pose[2], predicted_end_z), safe_z
        else:
            lowest_z, gate_z = pose[2], _ALONG_FLOOR_Z
        need_lift = lowest_z < gate_z - along_z_tol

        lifted = 0.0
        executed = 0.0
        steps_done = 0

        def _read() -> tuple[str, list[float]]:
            st, p = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link")
            return _enum_name(st), [float(v) for v in list(p)]

        def _step(target: list[float]) -> str | None:
            """下达一个绝对目标；成功返回 None，否则返回最后的状态名。

            实测 `FAULT` 很不可靠：同一目标 IK 带碰撞检查 SUCCESS、不带反而 FAULT，
            说明它是规划器/控制器的偶发状态，不是稳定的"到不了"。所以：
              1. 失败后**先重读位姿**——可能其实已经走到位了，只是状态码报错，
                 这种情况直接当成功，避免重复走一遍导致位移翻倍；
              2. 确实没到就重试，前两次仍走直线，最后一次改用绕行规划。
            """
            last = "UNKNOWN"
            for attempt in range(3):
                straight = attempt < 2
                try:
                    st = self._execute_cartesian(
                        chain, target,
                        straight_line=straight,
                        enable_collision_check=enable_collision_check,
                        timeout_s=timeout_s,
                        actuate=actuate,
                    )
                    name = _enum_name(st)
                except Exception as exc:  # noqa: BLE001 - 下发失败≠没动
                    LOG.warning("move_along_gripper 下发抛错（第%d次）：%s", attempt + 1, exc)
                    name = f"EXC:{type(exc).__name__}"
                if name == "SUCCESS":
                    return None

                st2, cur = _read()
                if st2 == "SUCCESS":
                    gap = math.sqrt(sum((cur[i] - target[i]) ** 2 for i in range(3)))
                    if gap < 0.005:
                        LOG.info(
                            "move_along_gripper 报 %s 但实际已到位（差 %.1f mm），按成功处理",
                            name, gap * 1000,
                        )
                        return None
                last = name
                if attempt < 2:
                    LOG.warning(
                        "move_along_gripper 第%d次失败（%s），%.1fs 后重试（%s）",
                        attempt + 1, name, 0.3, "直线" if attempt == 0 else "绕行",
                    )
                    time.sleep(0.3)
            return last

        # ---- 阶段 1：抬到该模式的建议高度（仅在 auto_lift=True 时）----
        # auto_lift=False = 纯沿夹爪平移：不抬、也不拒绝。
        # 高度门槛建立在"桌面高度已知 / 深度可信"的假设上，假设不成立时
        # 强行抬升只会给轨迹塞进一段竖直位移（实测每步被抬 6 cm，完全跑偏）。
        if need_lift and auto_lift:
            while True:
                st_name, cur = _read()
                if st_name != "SUCCESS":
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN, f"抬升中读位姿失败（{st_name}）", mode=self.mode,
                        data={"lifted_m": lifted},
                    )
                if cur[2] >= safe_z - along_z_tol:
                    break
                dz = min(max_step_m, safe_z - cur[2])
                # 余量过小就别单独发一次运动：极小位移会让规划器卡住（见上）
                if dz < along_z_tol:
                    break
                bad = _step([cur[0], cur[1], cur[2] + dz] + cur[3:])
                if bad is not None:
                    return failed(
                        ErrorCode.SDK_ERROR,
                        f"抬升到 z={cur[2] + dz:.3f} 失败（{bad}）；"
                        f"已抬 {lifted:.3f} m，尚未开始沿夹爪移动。"
                        f"纯手臂在已伸展的状态下抬升容易被拒 —— "
                        f"改用 actuate='with_torso'，或先手动把手臂抬到 {safe_z:.3f} 附近再调",
                        self.mode,
                        data={"chain": chain, "lifted_m": lifted, "actuate": actuate,
                              "motion_status": bad, "suggested_z": safe_z},
                    )
                lifted += dz

        # ---- 阶段 2：沿夹爪方向分步直线走 ----
        n_steps = max(1, math.ceil(abs(distance_m) / max_step_m))
        step_len = distance_m / n_steps
        for i in range(n_steps):
            st_name, cur = _read()
            if st_name != "SUCCESS":
                return unknown(
                    ErrorCode.STATE_UNCERTAIN,
                    f"第 {i + 1}/{n_steps} 步前读位姿失败（{st_name}）；"
                    f"已走 {executed:.3f} m，机器人位置未知",
                    mode=self.mode,
                    data={"executed_m": executed, "steps_done": steps_done},
                )
            cur_rot = quat_to_rot(cur[3:])
            ax = (cur_rot[0][0], cur_rot[1][0], cur_rot[2][0])
            target = [cur[j] + step_len * ax[j] for j in range(3)] + cur[3:]
            bad = _step(target)
            if bad is not None:
                return failed(
                    ErrorCode.SDK_ERROR,
                    f"第 {i + 1}/{n_steps} 步沿夹爪移动失败（{bad}）；"
                    f"已实际走 {executed:.3f} m / 目标 {distance_m:.3f} m",
                    self.mode,
                    data={"chain": chain, "executed_m": executed, "steps_done": steps_done,
                          "actuate": actuate, "motion_status": bad, "axis_base": list(ax)},
                )
            st_name, after = _read()
            if st_name == "SUCCESS":
                moved = [after[j] - cur[j] for j in range(3)]
                executed += sum(moved[j] * ax[j] for j in range(3))
                cur = after
            else:
                executed += step_len
            steps_done += 1

        st_name, end_pose = _read()
        return ok(
            f"{chain} 沿夹爪{'伸出' if distance_m > 0 else '后退'} {executed:.3f} m"
            f"（{actuate}{f'，先抬 {lifted:.3f} m' if lifted > 0 else ''}）",
            mode=self.mode,
            data={
                "chain": chain,
                "requested_m": distance_m,
                "executed_m": round(executed, 4),
                "actuate": actuate,
                "gripper_axis_base": [round(v, 5) for v in axis],
                "start_pose": [round(v, 5) for v in start_pose],
                "end_pose": [round(v, 5) for v in end_pose] if st_name == "SUCCESS" else None,
                "lifted_m": round(lifted, 4),
                "steps": steps_done,
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
        """末端姿态旋转（度），位置不变。旋转轴为 base_link 系（世界系）。

        与 move_ee_delta 同套路：读当前位姿 → 左乘增量旋转 → 下发绝对目标。
        给 VLM 直控用：夹爪朝向不对时不用移位置，纯转手腕对准。
        """
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            droll_deg, dpitch_deg, dyaw_deg = (
                float(droll_deg), float(dpitch_deg), float(dyaw_deg))
        except (TypeError, ValueError):
            return rejected(
                ErrorCode.INVALID_ARGUMENT, "角度必须是数值", mode=self.mode)

        import numpy as np

        try:
            _robot, motion, _nav = self._ensure_interfaces()
            status, pose = motion.get_end_effector_pose_on_chain(
                chain, "EndEffector", "base_link")
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN, f"旋转前读位姿失败：{exc}；未下发运动",
                mode=self.mode, data={"chain": chain})
        name = _enum_name(status)
        pose = list(pose)
        if name != "SUCCESS" or len(pose) != 7:
            return failed(
                ErrorCode.SDK_ERROR, f"旋转前读位姿失败（{name}）；未下发运动",
                mode=self.mode, data={"chain": chain, "motion_status": name})

        # 增量旋转（base_link 系，内旋 ZYX 顺序：先 yaw 再 pitch 再 roll）
        def _axis_angle_quat(axis, deg):
            half = math.radians(deg) / 2.0
            s = math.sin(half)
            return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)])

        def _quat_mul(a, b):
            ax, ay, az, aw = a
            bx, by, bz, bw = b
            return np.array([
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ])

        q_delta = _quat_mul(
            _quat_mul(_axis_angle_quat([0, 0, 1], dyaw_deg),
                      _axis_angle_quat([0, 1, 0], dpitch_deg)),
            _axis_angle_quat([1, 0, 0], droll_deg))
        q_cur = np.array([pose[3], pose[4], pose[5], pose[6]])
        # 世界系旋转 = 增量左乘当前姿态
        q_new = _quat_mul(q_delta, q_cur)
        q_new = q_new / np.linalg.norm(q_new)

        target = [pose[0], pose[1], pose[2],
                  float(q_new[0]), float(q_new[1]), float(q_new[2]), float(q_new[3])]
        try:
            status = self._execute_cartesian(
                chain, target,
                straight_line=False,  # 纯旋转不强制直线
                enable_collision_check=True,
                timeout_s=timeout_s,
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"下发末端旋转时抛错：{exc}；机器人是否已动未知",
                mode=self.mode,
                data={"chain": chain,
                      "delta_deg": [droll_deg, dpitch_deg, dyaw_deg]},
            )
        return self._motion_result(
            status, chain=chain,
            data={
                "chain": chain,
                "delta_deg": [droll_deg, dpitch_deg, dyaw_deg],
                "target_pose": target,
            },
            on_success=f"{chain} 末端旋转完成",
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
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            gm = self._import_sdk()
            # ⚠️ G1JointGroup 是 pybind11 类型，静态属性是**描述符**——
            # vars() 取到的是描述符对象而非字符串，isinstance(value,str) 会全部过滤掉
            # 导致白名单恒空、一切夹爪被拒（2026-11-07 真机实测 rejected/未知夹爪）。
            # 正确做法：用 getattr 取值（描述符 __get__ 返回真字符串）。
            # 候选夹爪固定为 G1 的两个 gripper 组（dexhand/suction_cup 不是 gripper）。
            candidate_grippers = ("left_gripper", "right_gripper")
            groups: dict[str, str] = {}
            for name in candidate_grippers:
                val = getattr(gm.G1JointGroup, name, None)
                if isinstance(val, str):
                    groups[name] = val
            if end_effector not in groups:
                return rejected(
                    ErrorCode.INVALID_ARGUMENT,
                    f"未知夹爪 {end_effector!r}",
                    mode=self.mode,
                    data={"supported_ids": sorted(groups.values())},
                )
            robot, _m, _n = self._ensure_interfaces(need_nav=False)
            # set_gripper_command(end_effector: str, ...) 要字符串；
            # groups[end_effector] 就是 getattr 取到的真字符串（如 "left_gripper"）。
            status = robot.set_gripper_command(
                groups[end_effector],
                float(width_m),
                float(velocity_mps),
                float(effort),
                bool(is_blocking),
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"设置夹爪时抛错：{exc}；夹爪实际状态未知",
                mode=self.mode,
                data={"end_effector": end_effector, "width_m": float(width_m)},
            )
        name = _enum_name(status)
        data = {"end_effector": end_effector, "width_m": float(width_m), "control_status": name}
        if name == "SUCCESS":
            return ok(f"{end_effector} 开度已设为 {width_m:.3f} m", mode=self.mode, data=data)
        return failed(ErrorCode.SDK_ERROR, f"设置夹爪失败（{name}）", mode=self.mode, data=data)

    # ══════════════════════════════════════════════════════════════════ #
    # 底盘：直控 set_base_pose（并入原 wrapper 能力，不需要地图/定位）
    # ══════════════════════════════════════════════════════════════════ #
    def _ensure_navigation(self) -> typing.Any:
        """懒建 GalbotNavigation（真需要导航时才拉起服务）。"""
        if self._navigation is not None:
            return self._navigation
        gm = self._import_sdk()
        _ensure_nav_service()
        navigation = gm.GalbotNavigation()
        if not navigation.init():
            raise RuntimeError(
                "GalbotNavigation.init() 返回 False（service_navigation_plan 未就绪？）"
            )
        self._navigation = navigation
        LOG.info("GalbotNavigation 初始化完成")
        return navigation

    def _ensure_chassis_pose_ctrl(self) -> str:
        """把底盘控制器切到 CHASSIS_POSE_CTRL —— set_base_pose 直控的前提。幂等。

        官方 SDK 切完控制器底盘会进初始化态，立即下令返回 WAIT_INITIALIZED，
        所以成功后等 2s（原 wrapper 真机实测的就绪窗口）。
        """
        with self._lock:
            if self._chassis_ready:
                return "SUCCESS"
            gm = self._import_sdk()
            robot, _m, _n = self._ensure_interfaces(need_nav=False)
            name = "UNKNOWN"
            for attempt in range(3):
                try:
                    name = _enum_name(
                        robot.switch_controller(gm.G1ControllerName.CHASSIS_POSE_CTRL)
                    )
                except Exception as exc:  # noqa: BLE001
                    name = f"EXCEPTION:{exc}"
                if name == "SUCCESS":
                    self._chassis_ready = True
                    LOG.info("底盘控制器已切换到 CHASSIS_POSE_CTRL")
                    time.sleep(2.0)
                    return name
                LOG.warning("切换 CHASSIS_POSE_CTRL 第 %d 次返回 %s", attempt + 1, name)
                time.sleep(1.0)
            return name

    def move_base(self, distance_m: float, yaw_rad: float = 0.0, *,
                  timeout_s: float = 30.0, retries: int = 1) -> ToolResult:
        """相对当前位姿前进/后退 + 原地转向（``set_base_pose`` frame_id='rel(0)'）。

        **不经过 GalbotNavigation**：不需要 localization / map / 规划服务，
        没建图的场地也能动。软限幅在 toolhub / wrapper 层做，这里只挡物理硬上限。
        """
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            distance_m = float(distance_m)
            yaw_rad = float(yaw_rad)
        except (TypeError, ValueError):
            return rejected(ErrorCode.INVALID_ARGUMENT,
                            "distance_m / yaw_rad 必须是数值", mode=self.mode)
        if abs(distance_m) > _BASE_MAX_DISTANCE_M:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次底盘位移 {distance_m:.2f} m 超过硬上限 {_BASE_MAX_DISTANCE_M} m",
                mode=self.mode, data={"distance_m": distance_m},
            )
        if abs(yaw_rad) > math.radians(_BASE_MAX_TURN_DEG):
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"单次转向 {math.degrees(yaw_rad):.1f}° 超过硬上限 {_BASE_MAX_TURN_DEG:.0f}°",
                mode=self.mode, data={"yaw_rad": yaw_rad},
            )

        ctrl = self._ensure_chassis_pose_ctrl()
        if ctrl != "SUCCESS":
            return failed(
                ErrorCode.NOT_READY,
                f"底盘控制器切换失败（{ctrl}），未下发运动指令",
                mode=self.mode, data={"control_status": ctrl},
            )

        try:
            robot, _m, _n = self._ensure_interfaces(need_nav=False)
        except Exception as exc:  # noqa: BLE001
            return unknown(ErrorCode.STATE_UNCERTAIN, f"SDK 初始化失败：{exc}", mode=self.mode)

        name = ""
        for attempt in range(max(0, int(retries)) + 1):
            try:
                res = robot.set_base_pose(
                    distance_m, 0.0, yaw_rad,
                    frame_id="rel(0)", is_blocking=True, timeout_s=float(timeout_s),
                )
            except TypeError:
                # 老版本 pybind 只吃位置参数
                try:
                    res = robot.set_base_pose(distance_m, 0.0, yaw_rad,
                                              "rel(0)", True, float(timeout_s))
                except Exception as exc:  # noqa: BLE001
                    return unknown(
                        ErrorCode.STATE_UNCERTAIN,
                        f"下发底盘运动时抛错：{exc}；机器人是否已移动未知",
                        mode=self.mode,
                    )
            except Exception as exc:  # noqa: BLE001
                return unknown(
                    ErrorCode.STATE_UNCERTAIN,
                    f"下发底盘运动时抛错：{exc}；机器人是否已移动未知",
                    mode=self.mode,
                )
            name = _enum_name(res)
            if name == "SUCCESS":
                break
            if attempt < int(retries):
                LOG.warning("set_base_pose 第 %d 次返回 %s，2s 后重试", attempt + 1, name)
                time.sleep(2.0)

        data = {"distance_m": distance_m, "yaw_rad": yaw_rad, "control_status": name}
        if name == "SUCCESS":
            return ok(
                f"底盘已移动：{distance_m * 100:.1f} cm / {math.degrees(yaw_rad):.1f}°",
                self.mode, data=data,
            )
        if name == "TIMEOUT":
            return failed(ErrorCode.TIMEOUT, f"底盘运动超时（{name}）", self.mode, data=data)
        return failed(ErrorCode.SDK_ERROR, f"底盘运动失败（{name}）", self.mode, data=data)

    def stop_base(self) -> ToolResult:
        """停底盘：先撤导航任务，再补一条零位移指令（真正压住底盘）。

        ⚠️ 只调 ``stop_navigation`` 在"导航没起过"的进程里等于空操作，
        所以底盘控制器就绪时再发一次 ``set_base_pose(0,0,0)`` 兜底。
        """
        nav_err = None
        nav_used = False
        if self._navigation is not None:
            nav_used = True
            try:
                self._navigation.stop_navigation()
            except Exception as exc:  # noqa: BLE001
                nav_err = str(exc)

        zero_status = "skipped(底盘未启用)"
        if self._chassis_ready:
            try:
                robot, _m, _n = self._ensure_interfaces(need_nav=False)
                try:
                    res = robot.set_base_pose(0.0, 0.0, 0.0, frame_id="rel(0)",
                                              is_blocking=True, timeout_s=5.0)
                except TypeError:
                    res = robot.set_base_pose(0.0, 0.0, 0.0, "rel(0)", True, 5.0)
                zero_status = _enum_name(res)
            except Exception as exc:  # noqa: BLE001
                zero_status = f"EXCEPTION:{exc}"

        data = {"stopped": True, "nav": "stopped" if nav_used else "not_used",
                "zero_cmd": zero_status}
        if nav_err:
            return unknown(ErrorCode.STATE_UNCERTAIN,
                           f"停止导航时抛错：{nav_err}", mode=self.mode, data=data)
        return ok("底盘已停止", self.mode, data=data)

    def read_sensor(self, sensor_name: str) -> ToolResult:
        """读任意传感器（相机请走 get_rgb_data / get_depth_data）。"""
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            robot, _m, _n = self._ensure_interfaces(need_nav=False)
        except Exception as exc:  # noqa: BLE001
            return unknown(ErrorCode.STATE_UNCERTAIN, f"SDK 初始化失败：{exc}", mode=self.mode)
        getter = getattr(robot, "get_sensor_data", None)
        if not callable(getter):
            return rejected(
                ErrorCode.NOT_SUPPORTED,
                f"SDK 没有 get_sensor_data，无法读 {sensor_name!r}；"
                f"相机请用 get_rgb_data / get_depth_data",
                mode=self.mode, data={"sensor": sensor_name},
            )
        try:
            value = getter(sensor_name)
        except Exception as exc:  # noqa: BLE001
            return failed(ErrorCode.SDK_ERROR, f"读传感器失败：{exc}", mode=self.mode,
                          data={"sensor": sensor_name})
        return ok(f"已读取传感器 {sensor_name}", self.mode,
                  data={"sensor": sensor_name, "value": value})

    # ══════════════════════════════════════════════════════════════════ #
    # 语音（播放预生成的 16 kHz / 16-bit / mono PCM）
    # ══════════════════════════════════════════════════════════════════ #
    def write_audio_stream_output(self, audio_chunk: bytes, stream_id: str = "") -> bool:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        return bool(robot.write_audio_stream_output(audio_chunk, stream_id))

    def set_volume(self, volume: float) -> bool:
        robot, _m, _n = self._ensure_interfaces(need_nav=False)
        return bool(robot.set_volume(float(volume)))

    # ══════════════════════════════════════════════════════════════════ #
    # 导航（→ GalbotNavigation，注意不是 GalbotRobot）
    # ══════════════════════════════════════════════════════════════════ #
    def _load_places(self) -> dict[str, dict[str, typing.Any]]:
        if self._places is not None:
            return self._places
        places: dict[str, dict[str, typing.Any]] = {}
        if os.path.exists(self._places_path):
            try:
                with open(self._places_path, encoding="utf-8") as fh:
                    payload = json.load(fh)
                raw = payload.get("places") or {}
                if isinstance(raw, dict):
                    places = {str(k): dict(v or {}) for k, v in raw.items()}
            except Exception as exc:  # noqa: BLE001
                LOG.warning("读取 %s 失败：%s", self._places_path, exc)
        else:
            LOG.warning("地点配置不存在：%s（所有导航都会被拒绝）", self._places_path)
        self._places = places
        return places

    def navigate_to_place(self, place_id: str) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection

        places = self._load_places()
        place = places.get(place_id)
        if place is None:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"未登记的地点 {place_id!r}",
                mode=self.mode,
                data={"place_id": place_id, "supported_ids": sorted(places)},
            )

        pose = place.get("pose")
        if not place.get("verified") or not pose or len(pose) != 7:
            return rejected(
                ErrorCode.NOT_READY,
                f"地点 {place_id} 已登记但**坐标未标定**（pose 为空或 verified=false）。"
                f"请现场记录目标位姿填入 {os.path.basename(self._places_path)} 后再试。"
                f"未下发任何运动指令。",
                mode=self.mode,
                data={"place_id": place_id},
            )

        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            if not bool(navigation.is_localized()):
                return rejected(
                    ErrorCode.NOT_READY,
                    "机器人当前未在地图中定位（is_localized()=False），无法导航；"
                    "请先完成重定位。未下发任何运动指令。",
                    mode=self.mode,
                    data={"place_id": place_id},
                )
            gm = self._import_sdk()
            handle = navigation.set_navigation_target(
                gm.Pose([float(v) for v in pose]),
                frame=str(place.get("frame") or "map"),
                speed_ratio=float(place.get("speed_ratio") or 1.0),
                enable_collision_check=bool(place.get("enable_collision_check", True)),
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"下发导航时抛错：{exc}；机器人是否开始移动未知",
                mode=self.mode,
                data={"place_id": place_id},
            )

        if not bool(getattr(handle, "request_sent", False)):
            return failed(
                ErrorCode.SDK_ERROR,
                f"导航请求未发出：{getattr(handle, 'msg', '') or '无返回信息'}",
                mode=self.mode,
                data={"place_id": place_id},
            )

        task_id = str(handle.task_id)
        with self._lock:
            self._nav_tasks[task_id] = {
                "target": place_id,
                "status": TaskStatus.RUNNING,
                "started": time.monotonic(),
            }
        return running(
            f"导航任务已下发（尚未到达）：{place_id}",
            mode=self.mode,
            task_id=task_id,
            data={"place_id": place_id, "frame": place.get("frame") or "map"},
        )

    def get_map_pose(self) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            if not bool(navigation.is_localized()):
                return rejected(
                    ErrorCode.NOT_READY,
                    "机器人尚未在 SLAM 地图中定位",
                    mode=self.mode,
                )
            pose = [float(v) for v in navigation.get_current_pose()]
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"读取地图位姿失败：{exc}",
                mode=self.mode,
            )
        if len(pose) != 7:
            return failed(
                ErrorCode.SDK_ERROR,
                f"地图位姿长度 {len(pose)}，应为 7",
                mode=self.mode,
            )
        return ok(
            "地图位姿已读取",
            self.mode,
            data={"pose": pose, "frame": "map", "unit": "m"},
        )

    def check_path_reachability(
        self,
        goal_pose: typing.Sequence[float],
        start_pose: typing.Sequence[float] | None = None,
    ) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        goal = [float(v) for v in list(goal_pose)]
        if len(goal) != 7:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "goal_pose 必须是 7 个数值",
                mode=self.mode,
            )
        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            if not bool(navigation.is_localized()):
                return rejected(
                    ErrorCode.NOT_READY,
                    "机器人尚未在 SLAM 地图中定位",
                    mode=self.mode,
                )
            start = (
                [float(v) for v in list(start_pose)]
                if start_pose is not None
                else [float(v) for v in navigation.get_current_pose()]
            )
            reachable = bool(navigation.check_path_reachability(goal, start))
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"路径可达性检查失败：{exc}",
                mode=self.mode,
            )
        if reachable:
            return ok(
                "SLAM 路径可达",
                self.mode,
                data={"reachable": True, "goal_pose": goal, "start_pose": start},
            )
        return rejected(
            ErrorCode.SAFETY_LIMIT,
            "SLAM 路径不可达或会碰撞",
            mode=self.mode,
            data={"reachable": False, "goal_pose": goal, "start_pose": start},
        )

    def navigate_to_pose(
        self,
        pose: typing.Sequence[float],
        *,
        frame: str = "map",
        speed_ratio: float = 0.5,
        enable_collision_check: bool = True,
    ) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        target = [float(v) for v in list(pose)]
        if len(target) != 7 or not all(math.isfinite(v) for v in target):
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                "pose 必须是 7 个有限数值",
                mode=self.mode,
            )
        if frame not in {"map", "base_link", "base"}:
            return rejected(
                ErrorCode.INVALID_ARGUMENT,
                f"不支持的导航 frame {frame!r}",
                mode=self.mode,
            )
        if not 0.05 <= float(speed_ratio) <= 1.0:
            return rejected(
                ErrorCode.SAFETY_LIMIT,
                f"speed_ratio 必须在 [0.05, 1.0]，收到 {speed_ratio}",
                mode=self.mode,
            )
        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            if not bool(navigation.is_localized()):
                return rejected(
                    ErrorCode.NOT_READY,
                    "机器人尚未在 SLAM 地图中定位",
                    mode=self.mode,
                )
            gm = self._import_sdk()
            handle = navigation.set_navigation_target(
                gm.Pose(target),
                frame=frame,
                speed_ratio=float(speed_ratio),
                enable_collision_check=bool(enable_collision_check),
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"下发 SLAM 导航时抛错：{exc}；机器人是否开始移动未知",
                mode=self.mode,
                data={"target_pose": target, "frame": frame},
            )
        if not bool(getattr(handle, "request_sent", False)):
            return failed(
                ErrorCode.SDK_ERROR,
                f"SLAM 导航请求未发出：{getattr(handle, 'msg', '') or '无返回信息'}",
                mode=self.mode,
            )
        task_id = str(handle.task_id)
        with self._lock:
            self._nav_tasks[task_id] = {
                "target": "dynamic_map_pose",
                "target_pose": target,
                "frame": frame,
                "status": TaskStatus.RUNNING,
                "started": time.monotonic(),
            }
        return running(
            "SLAM 导航已下发（尚未到达）",
            self.mode,
            task_id=task_id,
            data={"target_pose": target, "frame": frame},
        )

    def get_task_status(self, task_id: str) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        if not isinstance(task_id, str) or not task_id.strip():
            return rejected(ErrorCode.INVALID_ARGUMENT, "task_id 必须是非空字符串", mode=self.mode)

        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            snapshot = navigation.get_navigation_target_status(task_id)
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"查询任务状态时抛错：{exc}；实际执行状态未知",
                mode=self.mode,
                data={"task_id": task_id},
            )

        name = _enum_name(getattr(snapshot, "status", ""))
        status, code = _NAV_STATUS_MAP.get(name, (TaskStatus.UNKNOWN, ErrorCode.STATE_UNCERTAIN))
        message = str(getattr(snapshot, "msg", "") or f"导航状态：{name}")
        data = {"task_id": task_id, "sdk_status": name}

        with self._lock:
            if task_id in self._nav_tasks:
                self._nav_tasks[task_id]["status"] = status

        if status is TaskStatus.SUCCEEDED:
            return ok(f"已到达：{message}", mode=self.mode, task_id=task_id, data=data)
        if status is TaskStatus.RUNNING:
            return running(message, mode=self.mode, task_id=task_id, data=data)
        if status is TaskStatus.UNKNOWN:
            return unknown(code or ErrorCode.STATE_UNCERTAIN, message, mode=self.mode, data=data)
        return failed(
            code or ErrorCode.SDK_ERROR, message, mode=self.mode, data=data, task_id=task_id
        )

    def cancel_task(self, task_id: str) -> ToolResult:
        rejection = self._authorization_rejection()
        if rejection is not None:
            return rejection
        if not isinstance(task_id, str) or not task_id.strip():
            return rejected(ErrorCode.INVALID_ARGUMENT, "task_id 必须是非空字符串", mode=self.mode)

        try:
            _robot, _motion, navigation = self._ensure_interfaces()
            success, status_string = navigation.stop_navigation()
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"发送停止指令时抛错：{exc}；机器人是否已停未知",
                mode=self.mode,
                data={"task_id": task_id},
            )

        # ⚠️ stop_navigation 的 success 只代表"指令已发出"，不代表已经停下。
        # 必须再查一次任务状态，才能说 cancelled（协议 §2）。
        try:
            snapshot = navigation.get_navigation_target_status(task_id)
            name = _enum_name(getattr(snapshot, "status", ""))
            status, _code = _NAV_STATUS_MAP.get(
                name, (TaskStatus.UNKNOWN, ErrorCode.STATE_UNCERTAIN)
            )
        except Exception as exc:  # noqa: BLE001
            return unknown(
                ErrorCode.STATE_UNCERTAIN,
                f"停止指令已发出（sdk_success={success}, {status_string}），"
                f"但复核状态失败：{exc}；是否已停未知",
                mode=self.mode,
                data={"task_id": task_id},
            )

        with self._lock:
            if task_id in self._nav_tasks:
                self._nav_tasks[task_id]["status"] = status

        if status is not TaskStatus.RUNNING:
            return canceled(
                f"停止已确认（SDK 状态 {name}）",
                mode=self.mode,
                task_id=task_id,
                data={"task_id": task_id, "sdk_status": name, "sdk_success": bool(success)},
            )
        return unknown(
            ErrorCode.STATE_UNCERTAIN,
            f"停止指令已发出（sdk_success={success}, {status_string}）但任务仍为 RUNNING；"
            f"不得认为机器人已停止",
            mode=self.mode,
            data={"task_id": task_id, "sdk_status": name},
        )


__all__ = ["GalbotAdapter", "DEFAULT_SENSORS"]
