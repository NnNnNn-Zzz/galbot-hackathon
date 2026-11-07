"""统一封装层（薄壳）：能力全部落在 ``sdk_wrapper.dev``，本模块只做三件事。

    ┌──────────────────────────────────────────────────────────────┐
    │ 1. 名字兼容：保留全部历史函数名与参数（老脚本一行不用改）        │
    │ 2. 安全限幅：按 safety_config.json 截断距离/角度/位移           │
    │ 3. 单位换算：对外厘米/度 ↔ dev 层米/弧度                        │
    └──────────────────────────────────────────────────────────────┘

════════════════════════════════════════════════════════════════════════════
为什么要改成薄壳（这是本文件第 N 版）
════════════════════════════════════════════════════════════════════════════
老版本自己 ``GalbotRobot()`` / ``GalbotNavigation()`` / ``GalbotMotion()`` 各 init 一套，
dev 层 ``GalbotAdapter`` 又 init 一套 —— 同进程**两套 SDK 单例互踩**（纪律④），
于是能力被割裂成"底盘在 wrapper、手臂/相机/感知在 dev"两块，接新能力时要在两边重复实现。

现在改成：

    agent / 场景脚本 / 老脚本
              │   本文件（薄壳：限幅 + 单位换算 + dict 返回）
              ▼
    sdk_wrapper.dev.toolhub（dev 层统一工具入口）
              ▼
    GalbotAdapter / MockAdapter   ← 全进程**唯一**的 SDK 单例持有者

⚠️ 老代码若 import 了本模块的私有单例（``_get_robot`` / ``get_nav``），
   仍可用，但它们现在返回的是 **adapter 内部的那一份**，不再是另外 new 的。
   新代码请直接调本模块的函数，或 ``sdk_wrapper.dev.toolhub``。
"""

from __future__ import annotations

import logging
import math
import os

from sdk_wrapper.config import load_safety

logger = logging.getLogger("sdk_wrapper")

#: 兜底开关：环境变量 GALBOT_MOCK=1 与 --mock 等效（保留旧行为）
_mock_mode = os.environ.get("GALBOT_MOCK", "0") == "1"

#: 夹爪默认运动速度（米/秒）与力档位。沿用 dev adapter 真机验证值（D5 通过）。
GRIPPER_DEFAULT_VELOCITY_MPS = 0.05
GRIPPER_DEFAULT_EFFORT = 10.0

#: 手臂单次平移限幅（米）。
ARM_MAX_STEP_M = 0.10

#: 夹爪物理开度上限（米）。dev 层 D5 真机验证 0~0.12m 均 SUCCESS，保守取 0.085m。
GRIPPER_MAX_WIDTH_M = 0.085


# ---------------------------------------------------------------------------
# 内部：拿到 dev 层统一入口
# ---------------------------------------------------------------------------
def _hub():
    """懒加载 dev 层工具入口。

    懒加载是必须的：``sdk_wrapper/__init__`` → wrapper → dev.toolhub → wrapper
    如果在模块顶层 import 会形成循环。
    """
    from sdk_wrapper.dev import toolhub

    return toolhub


def set_mock_mode(enabled: bool):
    """由 agent/main.py 在启动时根据 --mock 参数调用。"""
    global _mock_mode
    _mock_mode = bool(enabled)
    _hub().configure(mock=bool(enabled))


# ---------------------------------------------------------------------------
# 单例入口（兼容保留）
# ---------------------------------------------------------------------------
def get_robot():
    """公开入口：共享机器人单例。

    真机返回 ``GalbotRobot``（adapter 内部那一份，已 init）；
    mock 返回 ``MockAdapter``（它自带 stop_base / set_gripper 等方法）。

    ⚠️ 场景脚本**禁止**自己 init()/destroy()，必须从这里拿。
    """
    hub = _hub()
    adapter = hub.get_adapter()
    if hub._is_mock():
        return adapter
    try:
        robot, _motion, _nav = adapter._ensure_interfaces(need_nav=False)
        return robot
    except Exception as exc:  # noqa: BLE001 - 兼容路径，拿不到就退回 adapter
        logger.warning("取 GalbotRobot 单例失败，退回 adapter：%s", exc)
        return adapter


def _get_robot():
    """历史私有名字，等价 ``get_robot()``（baseline.py / 老脚本仍在用）。"""
    return get_robot()


class _MockNavigation:
    """mock 下的导航替身：老代码会调 ``nav.stop_navigation()``。"""

    def __init__(self) -> None:
        self.stopped = 0

    def stop_navigation(self):
        self.stopped += 1
        return True, "SUCCESS"

    def is_localized(self) -> bool:
        return True


_mock_nav = _MockNavigation()


def get_nav():
    """公开入口：共享导航单例（真机是 adapter 的 GalbotNavigation）。

    ⚠️ 直接调 nav 没有 wrapper 层的自动限幅，请自觉遵守 safety_config.json。
    """
    hub = _hub()
    adapter = hub.get_adapter()
    if hub._is_mock():
        return _mock_nav
    _robot, _motion, nav = adapter._ensure_interfaces(need_nav=True)
    return nav


def _get_nav():
    """历史私有名字，等价 ``get_nav()``。"""
    return get_nav()


def shutdown():
    """真机 SDK 清理（程序退出时必须调用）。

    注意：SDK destroy() 后同进程无法再 init()，重启 agent 请重新起进程。
    """
    try:
        _hub().shutdown()
    except Exception as exc:  # noqa: BLE001 - 清理失败也要让退出流程继续
        logger.error("SDK 清理异常: %s", exc)


# ---------------------------------------------------------------------------
# 内部：限幅
# ---------------------------------------------------------------------------
def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        logger.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# 底盘运动接口（直控 set_base_pose，不需要地图/定位）
# ---------------------------------------------------------------------------
def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米，负值即后退）。自动限幅，阻塞执行。

    Returns:
        {"success": bool, "actual_distance_cm": float, "message": str}
    """
    cfg = load_safety()
    distance_m = _clamp(distance_cm / 100.0, cfg["max_base_distance_m"], "distance")
    r = _hub().move_base(distance_m, 0.0)
    r["actual_distance_cm"] = round(distance_m * 100, 2)
    return r


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(distance_cm))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    cfg = load_safety()
    angle_deg = _clamp(angle_deg, cfg["max_turn_deg"], "turn_angle")
    r = _hub().move_base(0.0, math.radians(angle_deg))
    r["actual_angle_deg"] = round(angle_deg, 2)
    return r


def stop_base() -> dict:
    """立即停止底盘运动（场景退出前建议调用）。"""
    return _hub().stop_base()


def read_sensor(sensor_name: str):
    """读取传感器（相机名会直接转成"抓一帧"）。"""
    return _hub().read_sensor(sensor_name)


# ---------------------------------------------------------------------------
# 夹爪接口
# ---------------------------------------------------------------------------
def set_gripper(end_effector: str = "left_gripper", width_cm: float = 0.0,
                velocity_mps: float = GRIPPER_DEFAULT_VELOCITY_MPS,
                effort: float = GRIPPER_DEFAULT_EFFORT) -> dict:
    """设置夹爪开度。单位：厘米（0=完全闭合，约 8.5cm=最大张开）。

    ``velocity_mps`` 仅为兼容旧签名保留（dev 层用固定速度 0.05 m/s）。
    """
    if abs(velocity_mps - GRIPPER_DEFAULT_VELOCITY_MPS) > 1e-9:
        logger.debug("set_gripper 的 velocity_mps 参数已忽略（dev 层固定 0.05 m/s）")
    return _hub().set_gripper(end_effector, float(width_cm), effort=float(effort))


def open_gripper(end_effector: str = "left_gripper") -> dict:
    """夹爪完全张开。"""
    return _hub().open_gripper(end_effector)


def close_gripper(end_effector: str = "left_gripper",
                  effort: float = GRIPPER_DEFAULT_EFFORT) -> dict:
    """夹爪合拢（抓取 / 用侧面推）。"""
    return _hub().close_gripper(end_effector, effort=float(effort))


# ---------------------------------------------------------------------------
# 手臂运动接口
# ---------------------------------------------------------------------------
def get_ee_pose(chain: str = "right_arm") -> dict:
    """读末端位姿 [x,y,z,qx,qy,qz,qw]（base_link，米/四元数）。"""
    return _hub().get_ee_pose(chain)


def move_arm_to(pose, chain: str = "right_arm", timeout_s: float = 15.0) -> dict:
    """手臂到 base_link 绝对位姿。pose=[x,y,z,qx,qy,qz,qw]（米/四元数）。"""
    pose = [float(v) for v in pose]
    if len(pose) != 7:
        return {"success": False, "message": f"pose 需 7 元，收到 {len(pose)}"}
    return _hub().move_arm_to(pose, chain=chain, straight_line=False)


def move_arm_delta(dx: float, dy: float, dz: float, chain: str = "right_arm",
                   timeout_s: float = 15.0) -> dict:
    """末端平移 (dx,dy,dz) 米（base_link 系），姿态不变。单轴限幅 ARM_MAX_STEP_M。"""
    dx = _clamp(dx, ARM_MAX_STEP_M, "arm_dx")
    dy = _clamp(dy, ARM_MAX_STEP_M, "arm_dy")
    dz = _clamp(dz, ARM_MAX_STEP_M, "arm_dz")
    return _hub().move_arm_delta(dx, dy, dz, chain=chain, straight_line=False)


# ---------------------------------------------------------------------------
# 感知接口
# ---------------------------------------------------------------------------
def capture_right_wrist(save_path: str = "/tmp/grasp_rgb.jpg") -> dict:
    """抓右腕相机 RGB + 深度 + 内参 + 外参，供反投影。

    ⚠️ 深度与彩色是否同一光心未验证（cameras.json: unverified）。
       真机先比对 width/height，不一致则同一像素取深度会错位，需先做对齐。

    Returns:
        {"success": bool, "image_path": str, "depth_m": np.ndarray,
         "intrinsic": dict, "extrinsic": [7], "width": int, "height": int, ...}
        depth_m 是 numpy 数组（米），仅供场景脚本内部 deproject 用，不给 LLM。
    """
    return _hub().capture_right_wrist(save_path)


def capture_frame(camera_id: str = "right_arm",
                  save_path: str = "/tmp/frame.jpg") -> dict:
    """抓任意相机一帧（right_arm / left_arm）。"""
    return _hub().capture_frame(camera_id, save_path)


def find_object(label: str, camera_id: str | None = None,
                max_distance_m: float = 1.5) -> dict:
    """按颜色/前景找目标（label 见 dev.algorithms.scene_analyzer）。"""
    return _hub().find_object(label, camera_id, max_distance_m)


def inspect_table(camera_id: str | None = None) -> dict:
    """判桌面杯子到桌沿的距离（需 table_calib.json 标定）。"""
    return _hub().inspect_table(camera_id)


def list_cameras() -> dict:
    """列出可用相机与能力（有无 RGB/深度）。"""
    return _hub().list_cameras()


def push_cup() -> dict:
    """把不安全的杯子推离桌沿（dev 层 CupScenario 闭环）。"""
    return _hub().push_cup()


# ---------------------------------------------------------------------------
# 场景层 P0 接口（2026-11-07 补全，药品场景专用，纯追加不改动上文）
#
# 全部是对官方 SDK 的薄封装，写法对照 sdk_wrapper/dev GalbotAdapter
# （该层已在场地真机 22/22 验证）。关键坑位提醒：
#   - 状态判定一律走 _status_name()，pybind 枚举 bool() 恒真，TIMEOUT 也会变 True；
#   - set_gripper_command 的夹爪名要传字符串（getattr 取 G1JointGroup 静态值），
#     不能传 pybind 枚举/描述符；
#   - move_line 需要包 MotionPlanChainTarget（照 dev._execute_cartesian）。
# mock 模式下全部返回成功假数据（真机路径才碰 SDK），与场景 MockIO 互不干扰
# （场景 mock 走 io.MockIO，不经过本模块）。
# ---------------------------------------------------------------------------

_motion = None
#: attach_tool 挂载过的链名，供 set_arm_pose / move_line_to 缺省使用
_attached_chain: str = "right_arm"


def _get_motion():
    """获取 GalbotMotion 单例（真机）。mock 模式返回字符串占位。"""
    global _motion
    if _motion is None:
        if _mock_mode:
            _motion = "mock"
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotMotion
            m = GalbotMotion()
            if not m.init():
                raise RuntimeError("GalbotMotion.init() 返回 False")
            _motion = m
            logger.info("Motion 实例已初始化")
    return _motion


def _mock_ok(message: str, **extra):
    """mock 模式统一成功返回（completion 恒 completed，与契约一致）。"""
    return {"success": True, "message": f"[MOCK] {message}",
            "completion": "completed", **extra}


def attach_tool(chain: str, tool: str) -> dict:
    """把末端工具（如 galbot_gripper）挂载到手臂链，后续位姿控制的前提。"""
    global _attached_chain
    if _mock_mode:
        _attached_chain = chain
        return _mock_ok(f"已挂载 {tool} 到 {chain}")
    status = _get_motion().attach_tool(chain, tool)
    name = _status_name(status)
    ok = name == "SUCCESS"
    if ok:
        _attached_chain = chain
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"attach_tool({chain},{tool}): {status}"}


def set_arm_pose(pose, timeout_s: float = 20.0, chain: str = "") -> dict:
    """移动到 base_link 下的绝对末端位姿 [x,y,z,qx,qy,qz,qw]（自由路径）。"""
    pose = [float(v) for v in pose]
    if len(pose) != 7:
        return {"success": False, "completion": "failed",
                "message": f"位姿应为 7 维，收到 {len(pose)} 维"}
    if _mock_mode:
        return _mock_ok("set_arm_pose", pose=pose)
    chain = chain or _attached_chain
    status = _get_motion().set_end_effector_pose(
        pose, chain, "base_link",
        enable_collision_check=True, is_blocking=True, timeout=float(timeout_s))
    name = _status_name(status)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"set_end_effector_pose: {status}"}


def move_line_to(pose, chain: str = "") -> dict:
    """末端直线逼近 base_link 下的绝对位姿（抓取最后一段必须用直线）。"""
    pose = [float(v) for v in pose]
    if len(pose) != 7:
        return {"success": False, "completion": "failed",
                "message": f"位姿应为 7 维，收到 {len(pose)} 维"}
    if _mock_mode:
        return _mock_ok("move_line_to", pose=pose)
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    chain = chain or _attached_chain
    target = gm.MotionPlanChainTarget()
    target.chain_name = chain
    target.mode = gm.MotionPlanTargetMode.kCartesian
    target.cart.chain_name = chain
    target.cart.frame_id = "EndEffector"
    target.cart.reference_frame = "base_link"
    target.cart.pose = gm.Pose(pose)
    target.cart.assist_chains = {"torso"}
    params = gm.Parameter()
    params.set_direct_execute(True)
    params.set_blocking(True)
    params.set_timeout(10.0)
    params.set_check_collision(True)
    params.set_reference_frame("base_link")
    params.set_actuate(True)
    status, _traj = _get_motion().move_line([[target]], params)
    name = _status_name(status)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"move_line: {status}"}


def set_joints(positions, names, speed_rad_s: float = 0.2) -> dict:
    """按关节名列表下关节角（弧度），用于收臂抱持等固定姿态。"""
    positions = [float(v) for v in positions]
    if names and len(names) != len(positions):
        return {"success": False, "completion": "failed",
                "message": f"关节名 {len(names)} 个、位置 {len(positions)} 个，数量不一致"}
    if _mock_mode:
        return _mock_ok("set_joints", joints=positions)
    robot = _get_robot()
    status = robot.set_joint_positions(
        positions, joint_names=list(names), is_blocking=True,
        speed_rad_s=float(speed_rad_s), timeout_s=15.0)
    name = _status_name(status)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"set_joint_positions: {status}"}


def set_gripper(width_m: float, side: str = "left",
                velocity_mps: float = 0.05, effort: float = 10.0) -> dict:
    """设置夹爪开度（米）。side: 'left' / 'right'。"""
    if _mock_mode:
        return _mock_ok(f"set_gripper {side} -> {width_m:.3f}m")
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    # 夹爪名必须传字符串；getattr 取 G1JointGroup 静态属性值（描述符 __get__），
    # 用 vars() 会拿到描述符对象导致白名单恒空（真机实测坑，见 dev 层注释）。
    gripper_name = getattr(gm.G1JointGroup, f"{side}_gripper", None)
    if not isinstance(gripper_name, str):
        return {"success": False, "completion": "failed",
                "message": f"未知夹爪侧 {side!r}，G1JointGroup 无 {side}_gripper"}
    robot = _get_robot()
    status = robot.set_gripper_command(
        gripper_name, float(width_m), float(velocity_mps), float(effort), True)
    name = _status_name(status)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"set_gripper_command: {status}"}


def get_odom() -> dict:
    """读底盘里程计：{"odom": {"position": [x,y,z], "orientation": [qx,qy,qz,qw]}}。"""
    if _mock_mode:
        return _mock_ok("get_odom",
                        odom={"position": [0.0, 0.0, 0.0],
                              "orientation": [0.0, 0.0, 0.0, 1.0]})
    robot = _get_robot()
    odom = robot.get_odom()  # 失败时 SDK 返回空 dict
    if not odom or "position" not in odom:
        return {"success": False, "completion": "failed",
                "message": "get_odom 返回空（里程计不可用）"}
    return {"success": True, "message": "ok", "odom": {
        "position": [float(v) for v in odom["position"]],
        "orientation": [float(v) for v in odom.get("orientation", [0, 0, 0, 1])]}}


def move_left(distance_m: float) -> dict:
    """沿机体左向平移（米，负值向右）。相对当前位姿，自动限幅。"""
    cfg = load_safety()
    distance_m = _clamp(float(distance_m), cfg["max_base_distance_m"], "distance")
    if _mock_mode:
        return _mock_ok(f"move_left {distance_m:.2f}m")
    robot = _get_robot()
    # set_base_pose(x, y, yaw, frame_id='rel(0)'=相对当前)：x 前、y 左
    res = robot.set_base_pose(0.0, distance_m, 0.0,
                              frame_id='rel(0)', is_blocking=True,
                              timeout_s=BASE_POSE_TIMEOUT_S)
    name = _status_name(res)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"set_base_pose: {res}",
            "actual_distance_m": distance_m}


def move_to_pose(x: float, y: float, yaw: float) -> dict:
    """走到底盘绝对位姿（odom 系，x/y 米、yaw 弧度）。走导航通道。"""
    if _mock_mode:
        return _mock_ok(f"move_to_pose ({x:.2f},{y:.2f},{yaw:.2f})")
    nav = _get_nav()
    qx, qy, qz, qw = _yaw_to_quaternion(float(yaw))
    goal = [float(x), float(y), 0.0, qx, qy, qz, qw]
    nav.move_straight_to(goal, is_blocking=False, timeout=30.0)
    name = _wait_nav_done(nav, timeout_s=60.0)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name, "completion": "completed" if ok else "failed",
            "message": "ok" if ok else f"move_straight_to 终态: {name}"}


def speak(text: str) -> dict:
    """播报文本。SDK 无 TTS，只能播预合成的 16kHz/16bit/mono PCM。

    素材映射：sdk_wrapper/assets/tts/<sha1(text)前16位>.pcm，
    用 scripts/make_pcm.py 预生成（Windows SAPI 或 Edge TTS 转 PCM）。
    缺素材时返回 success=False + played=False（场景层降级为仅日志可见，
    不阻塞流程）——绝不拿假成功冒充已播报。
    """
    import hashlib
    pcm_dir = os.path.join(os.path.dirname(__file__), "assets", "tts")
    pcm_path = os.path.join(pcm_dir, hashlib.sha1(text.encode("utf-8")).hexdigest()[:16] + ".pcm")
    if _mock_mode:
        return _mock_ok(f"speak「{text}」", played=True)
    if not os.path.exists(pcm_path):
        return {"success": False, "completion": "failed", "played": False,
                "message": f"PCM 素材缺失：{pcm_path}（先用 scripts/make_pcm.py 生成）"}
    robot = _get_robot()
    with open(pcm_path, "rb") as f:
        data = f.read()
    chunk = 4096
    ok = True
    for i in range(0, len(data), chunk):
        if not robot.write_audio_stream_output(data[i:i + chunk], ""):
            ok = False
            break
    return {"success": ok, "completion": "completed" if ok else "failed",
            "played": ok,
            "message": "ok" if ok else "write_audio_stream_output 中途返回 False"}


def get_ee_pose(chain: str = "") -> dict:
    """读手臂末端在 base_link 下的当前位姿 [x,y,z,qx,qy,qz,qw]。

    示教取数专用：手动/手柄把手臂摆到目标位后调用，把返回的 pose
    填进场景 tuning.py（PRE_GRASP_POSE / GRASP_POSE / LIFT_POSE）。
    """
    if _mock_mode:
        return _mock_ok("get_ee_pose",
                        pose=[0.40, -0.10, 0.65, 0.0, 0.0, 0.0, 1.0],
                        frame="base_link", unit="m")
    chain = chain or _attached_chain
    status, pose = _get_motion().get_end_effector_pose_on_chain(
        chain, "EndEffector", "base_link")
    name = _status_name(status)
    pose = [float(v) for v in pose]
    ok = name == "SUCCESS" and len(pose) == 7
    return {"success": ok, "status": name,
            "message": "ok" if ok else f"get_end_effector_pose_on_chain: {status}",
            "pose": pose, "frame": "base_link", "unit": "m"}


def capture_image(tag: str = "") -> dict:
    """拍一帧头部左目 RGB（JPEG），存到 logs/frames/，返回 frame_id=文件路径。

    供识别类策略（OCR/检测）取图用；基线固定位姿抓取不依赖它。
    """
    if _mock_mode:
        return _mock_ok("capture_image", frame_id=f"mock_frame_{tag or '0'}")
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    robot = _get_robot()
    frame = robot.get_rgb_data(gm.SensorType.HEAD_LEFT_CAMERA,
                               gm.RgbOutputFormat.JPEG, True)
    if not frame or "data" not in frame:
        return {"success": False, "completion": "failed",
                "message": "get_rgb_data 返回空（相机不可用或未订阅）"}
    out_dir = os.path.join(os.path.dirname(__file__), "..", "logs", "frames")
    os.makedirs(out_dir, exist_ok=True)
    frame_id = os.path.join(out_dir, f"frame_{int(time.time())}_{tag or 'shot'}.jpg")
    with open(frame_id, "wb") as f:
        f.write(frame["data"])
    return {"success": True, "message": "ok", "frame_id": frame_id}
