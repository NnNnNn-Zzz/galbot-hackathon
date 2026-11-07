"""dev 层统一工具入口（ToolHub）—— agent / 场景脚本唯一该碰的 API 面。

════════════════════════════════════════════════════════════════════════════
分层（合并后：SDK 单例只归 dev 层所有）
════════════════════════════════════════════════════════════════════════════
    agent/main.py · scenes/* · scripts/*
                  │  历史名字兼容层：sdk_wrapper/wrapper.py（薄壳，无 SDK）
                  ▼
          sdk_wrapper/dev/toolhub.py        ← 本文件：扁平函数 + dict 返回
                  │
     ┌────────────┼──────────────┬───────────────┬────────────┬──────────────┐
     ▼            ▼              ▼               ▼            ▼              ▼
 MotionTool  PerceptionTool  NavigationTool   AudioTool   CupScenario   adapter.move_base
     └────────────┴──────────────┴───────────────┴────────────┴──────────────┘
                  ▼
        GalbotAdapter / MockAdapter   ← 全进程**唯一**的 SDK 单例持有者

为什么要有这层
--------------
* 老 wrapper.py 自己 init 一套 galbot_sdk 单例，dev/GalbotAdapter 又 init 一套，
  同进程互踩（纪律④），于是"底盘在 wrapper、手臂/相机在 dev"两边割裂。
* 合并后只有 adapter 碰 galbot_sdk；本文件只调 adapter / 工具的方法，不 import SDK。
* 所有公开函数返回 **dict**（``{"success": bool, "status": ..., "message": ..., **data}``），
  因为要直接 JSON 化喂给大模型；dev 的 ``ToolResult`` 在这里统一转换。

单位约定（对外一律用这套，避免两套单位混用）
    * 距离：厘米（cm）      * 角度：度（deg）      * 夹爪开度：厘米（cm）
    * 位姿：米 + 四元数 [x,y,z,qx,qy,qz,qw]（跟 SDK 一致，不换算）
"""

from __future__ import annotations

import logging
import math
import os
import threading
import typing

LOG = logging.getLogger("sdk_wrapper.dev.toolhub")

__all__ = [
    # 生命周期
    "configure", "get_adapter", "describe", "shutdown", "tool_names",
    # 工具实例
    "motion", "perception", "navigation", "audio", "scenario", "result_to_dict",
    "power_strip_yolo", "charger_yolo", "calibration", "charger_search",
    # 底盘
    "move_base", "move_forward", "move_backward", "turn", "stop_base",
    # 相机 / 感知
    "capture_frame", "capture_right_wrist", "read_sensor",
    "list_cameras", "find_object", "inspect_table", "capture_scene",
    "detect_power_strip_head", "detect_charger_head",
    "segment_charger_top_wrist", "observe_charger_search",
    "advance_charger_search", "reset_charger_search",
    "check_right_wrist_rgbd_calibration", "check_right_arm_eye_in_hand",
    # 手臂 / 夹爪
    "get_ee_pose", "move_arm_to", "move_arm_delta",
    "set_gripper", "open_gripper", "close_gripper",
    # 场景
    "push_cup",
]

#: G1 夹爪物理开度上限（米）。dev D5 真机验证 0~0.12m 均 SUCCESS，保守取 0.085m。
GRIPPER_MAX_WIDTH_M = 0.085

#: 相机短名 → SDK SensorType 名。让大模型说 "右腕相机" 也能命中。
_CAMERA_ALIASES: dict[str, str] = {
    "right_arm": "RIGHT_ARM_CAMERA",
    "right_wrist": "RIGHT_ARM_CAMERA",
    "右腕": "RIGHT_ARM_CAMERA",
    "右腕相机": "RIGHT_ARM_CAMERA",
    "left_arm": "LEFT_ARM_CAMERA",
    "left_wrist": "LEFT_ARM_CAMERA",
    "左腕": "LEFT_ARM_CAMERA",
    "左腕相机": "LEFT_ARM_CAMERA",
    "right_arm_depth": "RIGHT_ARM_DEPTH_CAMERA",
    "left_arm_depth": "LEFT_ARM_DEPTH_CAMERA",
    "right_arm_camera": "RIGHT_ARM_CAMERA",
    "left_arm_camera": "LEFT_ARM_CAMERA",
    "right_arm_depth_camera": "RIGHT_ARM_DEPTH_CAMERA",
    "left_arm_depth_camera": "LEFT_ARM_DEPTH_CAMERA",
}

#: 深度相机与彩色相机的配对（抓取反投影要成对取）
_DEPTH_OF: dict[str, str] = {
    "RIGHT_ARM_CAMERA": "RIGHT_ARM_DEPTH_CAMERA",
    "LEFT_ARM_CAMERA": "LEFT_ARM_DEPTH_CAMERA",
}

_adapter: typing.Any = None
_mode_override: bool | None = None
_tools: dict[str, typing.Any] = {}
_lock = threading.RLock()


# ══════════════════════════════════════════════════════════════════════════ #
# 生命周期：建一个 adapter，全世界共用
# ══════════════════════════════════════════════════════════════════════════ #
def _env_mock() -> bool:
    return os.environ.get("GALBOT_MOCK", "0") == "1"


def _is_mock() -> bool:
    return _mode_override if _mode_override is not None else _env_mock()


def _build_adapter(
    *,
    sensors: typing.Sequence[str] | None = None,
    places_path: str | os.PathLike[str] | None = None,
) -> typing.Any:
    """建适配器。mock 模式给 MockAdapter（自带合成相机，场景用 cup_table）。"""
    from sdk_wrapper.dev.adapters import GalbotAdapter, MockAdapter

    if _is_mock():
        return MockAdapter(scene="cup_table")
    return GalbotAdapter(
        enable_real_control=True,
        real_authorized=True,
        sensors=sensors,
        places_path=places_path,
    )


def configure(
    *,
    mock: bool | None = None,
    sensors: typing.Sequence[str] | None = None,
    places_path: str | os.PathLike[str] | None = None,
    force: bool = False,
) -> dict:
    """配置运行模式。``mock=None`` 表示跟随环境变量 ``GALBOT_MOCK``。

    幂等：已建好的 adapter 不会被重建（除非 ``force=True`` 或换了模式）。
    """
    global _adapter, _mode_override
    with _lock:
        if mock is not None and mock != _is_mock():
            _shutdown_locked()
            _mode_override = bool(mock)
        if _adapter is not None and not force:
            return describe()
        if force:
            _shutdown_locked()
        if mock is not None:
            _mode_override = bool(mock)
        _adapter = _build_adapter(sensors=sensors, places_path=places_path)
        LOG.info("ToolHub 适配器已建立：%s", type(_adapter).__name__)
        return describe()


def get_adapter() -> typing.Any:
    global _adapter
    with _lock:
        if _adapter is None:
            _adapter = _build_adapter()
        return _adapter


def describe() -> dict:
    return {"success": True, "mode": "mock" if _is_mock() else "real",
            "adapter": type(get_adapter()).__name__}


def shutdown() -> None:
    """真机必须走 SDK 关机序列（request_shutdown → wait_for_shutdown → destroy）。"""
    with _lock:
        _shutdown_locked()


def _shutdown_locked() -> None:
    global _adapter
    _tools.clear()
    if _adapter is None:
        return
    closer = getattr(_adapter, "close", None)
    try:
        if callable(closer):
            closer()
    except Exception as exc:  # noqa: BLE001 - 退出路径不该二次崩溃
        LOG.warning("关闭适配器失败：%s", exc)
    finally:
        _adapter = None


def tool_names() -> list[str]:
    """给大模型看的"能调什么"清单（扁平接口名）。"""
    return [name for name in __all__ if name not in (
        "configure", "get_adapter", "describe", "shutdown", "tool_names",
        "motion", "perception", "navigation", "audio", "scenario", "result_to_dict",
        "power_strip_yolo", "charger_yolo", "calibration", "charger_search",
    )]


# ══════════════════════════════════════════════════════════════════════════ #
# 工具实例（懒建，共用同一个 adapter 与 safety）
# ══════════════════════════════════════════════════════════════════════════ #
def _safety() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.safety import RobotSafety

    key = "safety"
    if key not in _tools:
        _tools[key] = RobotSafety(get_adapter(), real_authorized=True)
    return _tools[key]


def motion() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.motion import MotionTool

    if "motion" not in _tools:
        _tools["motion"] = MotionTool(get_adapter(), safety=_safety())
    return _tools["motion"]


def perception() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.perception import PerceptionTool

    if "perception" not in _tools:
        _tools["perception"] = PerceptionTool(get_adapter())
    return _tools["perception"]


def navigation() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.navigation import NavigationTool

    if "navigation" not in _tools:
        _tools["navigation"] = NavigationTool(get_adapter(), safety=_safety())
    return _tools["navigation"]


def audio() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.audio import AudioTool

    if "audio" not in _tools:
        _tools["audio"] = AudioTool(get_adapter())
    return _tools["audio"]


def scenario() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.cup_scenario import CupScenario

    if "scenario" not in _tools:
        _tools["scenario"] = CupScenario(perception(), audio(), motion())
    return _tools["scenario"]


def power_strip_yolo() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.power_strip_yolo import PowerStripYoloTool

    if "power_strip_yolo" not in _tools:
        _tools["power_strip_yolo"] = PowerStripYoloTool(perception())
    return _tools["power_strip_yolo"]


def charger_yolo() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.charger_yolo import ChargerYoloTool

    if "charger_yolo" not in _tools:
        _tools["charger_yolo"] = ChargerYoloTool(perception())
    return _tools["charger_yolo"]


def calibration() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.calibration import CameraCalibrationTool

    if "calibration" not in _tools:
        _tools["calibration"] = CameraCalibrationTool(perception())
    return _tools["calibration"]


def charger_search() -> typing.Any:
    from sdk_wrapper.dev.robot_tools.charger_search import ChargerSearchTool

    if "charger_search" not in _tools:
        _tools["charger_search"] = ChargerSearchTool(
            perception(),
            strip_tool=power_strip_yolo(),
            charger_tool=charger_yolo(),
        )
    return _tools["charger_search"]


def result_to_dict(res: typing.Any) -> dict:
    """``ToolResult`` → 扁平 dict（``success`` 放最前，其余 data 摊平）。"""
    if isinstance(res, dict):
        return res
    if res is None:
        return {"success": False, "status": "failed", "message": "工具返回 None"}
    data = getattr(res, "data", None)
    payload: dict[str, typing.Any] = {}
    if isinstance(data, dict):
        payload.update(data)
    status = getattr(res, "status", None)
    status_name = getattr(status, "value", None) or str(status)
    payload["success"] = status_name == "succeeded"
    payload["status"] = status_name
    payload["message"] = getattr(res, "message", "") or ""
    err = getattr(res, "error_code", None)
    if err is not None:
        payload["error_code"] = getattr(err, "value", None) or str(err)
    task_id = getattr(res, "task_id", None)
    if task_id:
        payload["task_id"] = task_id
    return payload


# ══════════════════════════════════════════════════════════════════════════ #
# 底盘（直控 set_base_pose，不需要地图 / 定位）
# ══════════════════════════════════════════════════════════════════════════ #
def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        LOG.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


def _safety_cfg() -> dict:
    from sdk_wrapper.config import load_safety

    return load_safety()


def move_base(distance_m: float, yaw_rad: float = 0.0, *,
              timeout_s: float = 30.0) -> dict:
    """相对当前位姿走一段 + 转一个角（不经过导航规划，不需要地图）。"""
    return result_to_dict(get_adapter().move_base(distance_m, yaw_rad, timeout_s=timeout_s))


def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米，负值即后退）。自动限幅，阻塞执行。"""
    cfg = _safety_cfg()
    distance_m = _clamp(float(distance_cm) / 100.0, cfg["max_base_distance_m"], "distance")
    r = move_base(distance_m, 0.0)
    r["actual_distance_cm"] = round(distance_m * 100, 2)
    return r


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(float(distance_cm)))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    cfg = _safety_cfg()
    angle_deg = _clamp(float(angle_deg), cfg["max_turn_deg"], "turn_angle")
    r = move_base(0.0, math.radians(angle_deg))
    r["actual_angle_deg"] = round(angle_deg, 2)
    return r


def stop_base() -> dict:
    """立即停止底盘运动。"""
    return result_to_dict(get_adapter().stop_base())


# ══════════════════════════════════════════════════════════════════════════ #
# 相机 / 感知
# ══════════════════════════════════════════════════════════════════════════ #
def _resolve_camera(camera_id: typing.Any) -> str:
    raw = str(camera_id or "").strip()
    key = raw.lower()
    if key in _CAMERA_ALIASES:
        return _CAMERA_ALIASES[key]
    upper = raw.upper()
    if upper in ("RIGHT_ARM_CAMERA", "LEFT_ARM_CAMERA",
                 "RIGHT_ARM_DEPTH_CAMERA", "LEFT_ARM_DEPTH_CAMERA"):
        return upper
    raise ValueError(
        f"未知相机 {camera_id!r}；可用：right_arm / left_arm "
        f"（或直接写 SDK 名 RIGHT_ARM_CAMERA）"
    )


def capture_frame(camera_id: typing.Any = "right_arm",
                  save_path: str = "/tmp/frame.jpg",
                  depth_sensor: typing.Any = None) -> dict:
    """抓一帧彩色图 +（可选）深度 + 内参 + 外参，返回 base_link 反投影所需的全部原料。

    ⚠️ 深度与彩色是否同一光心未验证（cameras.json: unverified）。
       真机先比对 width/height，不一致则同一像素取深度会错位，需先做对齐。

    Returns:
        {"success", "image_path", "width", "height", "depth_m": np.ndarray,
         "intrinsic": {...}, "extrinsic": [7], "depth_color_same_res": bool, ...}
        ``depth_m`` 是 numpy 数组（米），只给场景脚本 deproject 用，不要喂大模型。
    """
    import cv2
    import numpy as np

    sensor = _resolve_camera(camera_id)
    depth_name = _resolve_camera(depth_sensor) if depth_sensor else _DEPTH_OF.get(sensor)
    adapter = get_adapter()

    rgb = adapter.get_rgb_data(sensor)
    if not isinstance(rgb, dict) or not rgb.get("data"):
        return {"success": False, "message": f"{sensor} 取图失败（无数据）"}

    img = cv2.imdecode(np.frombuffer(rgb["data"], np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return {"success": False, "message": f"{sensor} JPEG 解码失败"}
    os.makedirs(os.path.dirname(os.path.abspath(save_path)) or ".", exist_ok=True)
    cv2.imwrite(save_path, img)

    out: dict[str, typing.Any] = {
        "success": True,
        "camera_id": sensor,
        "image_path": save_path,
        "width": int(rgb.get("width") or img.shape[1]),
        "height": int(rgb.get("height") or img.shape[0]),
        "message": "ok",
    }

    try:
        intr = adapter.get_camera_intrinsic(sensor)
        out["intrinsic"] = dict(intr) if isinstance(intr, dict) else {"raw": intr}
    except Exception as exc:  # noqa: BLE001 - 无内参不致命，反投影时才需要
        out["intrinsic"] = None
        out["intrinsic_error"] = str(exc)

    try:
        ext_raw = adapter.get_sensor_extrinsic(sensor, "base_link")
        ext = ext_raw[0] if isinstance(ext_raw, (tuple, list)) and len(ext_raw) >= 1 else ext_raw
        out["extrinsic"] = [float(v) for v in list(ext)[:7]]
    except Exception as exc:  # noqa: BLE001
        out["extrinsic"] = None
        out["extrinsic_error"] = str(exc)

    if depth_name:
        try:
            depth = adapter.get_depth_data(depth_name)
            if isinstance(depth, dict) and depth.get("data"):
                scale = float(depth.get("depth_scale") or 0.001)
                dw, dh = int(depth["width"]), int(depth["height"])
                darr = (np.frombuffer(depth["data"], dtype=np.uint16)
                        .reshape(dh, dw).astype(np.float64) * scale)
                out["depth_m"] = darr
                out["depth_width"], out["depth_height"] = dw, dh
                same = (out["width"] == dw and out["height"] == dh)
                out["depth_color_same_res"] = same
                if not same:
                    out["message"] = "深度与彩色分辨率不一致，取深度前需对齐"
            else:
                out["depth_m"] = None
                out["message"] = f"{depth_name} 无深度数据"
        except Exception as exc:  # noqa: BLE001
            out["depth_m"] = None
            out["depth_error"] = str(exc)
    return out


def capture_right_wrist(save_path: str = "/tmp/grasp_rgb.jpg") -> dict:
    """右腕相机抓帧（抓杯子场景的默认视角）。"""
    return capture_frame("right_arm", save_path, depth_sensor="right_arm_depth")


def read_sensor(sensor_name: str) -> dict:
    """读传感器。相机名直接转抓帧；其它透传给 adapter。"""
    try:
        return capture_frame(sensor_name)
    except ValueError:
        pass
    adapter = get_adapter()
    reader = getattr(adapter, "read_sensor", None)
    if reader is None:
        raise NotImplementedError(f"传感器 '{sensor_name}' 接口尚未封装")
    return result_to_dict(reader(sensor_name))


def list_cameras() -> dict:
    return result_to_dict(perception().list_cameras())


def find_object(label: str, camera_id: typing.Any = None,
                max_distance_m: float = 1.5) -> dict:
    """按颜色/前景找目标，返回像素坐标 + 相机系 3D + base_link 3D。"""
    cam = _resolve_camera(camera_id) if camera_id else None
    return result_to_dict(perception().find_object(label, cam, max_distance_m))


def inspect_table(camera_id: typing.Any = None) -> dict:
    """判桌面：每个杯子到桌沿的距离 + 往哪推。需要 table_calib.json 标定。"""
    cam = _resolve_camera(camera_id) if camera_id else None
    return result_to_dict(perception().inspect_table(cam))


def capture_scene(camera_id: typing.Any = None, include_depth: bool = True,
                  note: str = "") -> dict:
    cam = _resolve_camera(camera_id) if camera_id else None
    return result_to_dict(perception().capture_scene(cam, include_depth, note))


def detect_power_strip_head() -> dict:
    return result_to_dict(power_strip_yolo().detect_power_strip_head())


def detect_charger_head() -> dict:
    return result_to_dict(charger_yolo().detect_charger_head())


def segment_charger_top_wrist() -> dict:
    return result_to_dict(charger_yolo().segment_charger_top_wrist())


def observe_charger_search() -> dict:
    return result_to_dict(charger_search().observe_charger_search())


def advance_charger_search(operator_confirmed: bool = False) -> dict:
    """Advance one bounded head-camera search/approach step."""
    return result_to_dict(
        charger_search().advance_charger_search(
            operator_confirmed=operator_confirmed
        )
    )


def reset_charger_search() -> dict:
    return result_to_dict(charger_search().reset_charger_search())


def check_right_wrist_rgbd_calibration() -> dict:
    return result_to_dict(calibration().check_right_wrist_rgbd_calibration())


def check_right_arm_eye_in_hand() -> dict:
    return result_to_dict(calibration().check_right_arm_eye_in_hand())


# ══════════════════════════════════════════════════════════════════════════ #
# 手臂 / 夹爪
# ══════════════════════════════════════════════════════════════════════════ #
def get_ee_pose(chain: str = "right_arm") -> dict:
    """读末端在 base_link 下的位姿 [x,y,z,qx,qy,qz,qw]（米 / 四元数）。"""
    return result_to_dict(motion().get_end_effector_pose(chain))


def move_arm_to(pose: typing.Sequence[float], chain: str = "right_arm",
                straight_line: bool = True) -> dict:
    """手臂到 base_link 下的绝对位姿（7 个数）。抓取前对准用。"""
    return result_to_dict(motion().move_ee_to(pose, chain=chain, straight_line=straight_line))


def move_arm_delta(dx: float, dy: float, dz: float, chain: str = "right_arm",
                   straight_line: bool = True) -> dict:
    """末端平移 (dx,dy,dz) 米，姿态不变。"""
    return result_to_dict(get_adapter().move_ee_delta(
        chain, float(dx), float(dy), float(dz), straight_line=straight_line))


def set_gripper(end_effector: str = "left_gripper", width_cm: float = 0.0,
                effort: float = 10.0) -> dict:
    """设定夹爪开度（厘米）。0=闭合，8.5=全开（约）。"""
    width_m = max(0.0, min(float(width_cm) / 100.0, GRIPPER_MAX_WIDTH_M))
    r = result_to_dict(motion().set_gripper(end_effector, width_m, effort=effort))
    r["actual_width_cm"] = round(width_m * 100, 2)
    return r


def open_gripper(end_effector: str = "left_gripper") -> dict:
    """夹爪张开到最大（物理上限内）。"""
    return set_gripper(end_effector, GRIPPER_MAX_WIDTH_M * 100)


def close_gripper(end_effector: str = "left_gripper", effort: float = 10.0) -> dict:
    """夹爪合拢（抓取 / 用侧面推）。"""
    return set_gripper(end_effector, 0.0, effort=effort)


# ══════════════════════════════════════════════════════════════════════════ #
# 场景
# ══════════════════════════════════════════════════════════════════════════ #
def push_cup() -> dict:
    """把不安全的杯子推离桌沿（闭环：判不安全 → 推 → 复测 → 直到达标）。"""
    return result_to_dict(scenario().run_cup_scenario())
