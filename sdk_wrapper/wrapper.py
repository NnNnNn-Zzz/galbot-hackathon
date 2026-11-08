"""SDK 封装层：提供带安全限幅的高层接口 + 共享单例入口。

安全策略：
- 高层接口（move_forward / turn / ...）按 safety_config.json 限幅（clamp），超限不报错、静默截断并记日志。
- galbot_sdk 全局单例由本模块管理；场景脚本**允许**自己组装 SDK 封装
  （import galbot_sdk），但实例必须从本模块的 get_robot()/get_nav() 拿，
  严禁自行 init()/destroy()（会打死主进程单例和急停监听）。
- 支持 MOCK 模式（无真机/无 SDK 时开发调试）：
    方式1: 命令行 `python agent/main.py --mock`
    方式2: 环境变量 `GALBOT_MOCK=1`

真机接口说明（Galbot SDK v1.10.0 / G1）：
- 底盘移动走 GalbotNavigation.move_straight_to(goal_pose, is_blocking, timeout)，
  goal_pose = [x, y, z, qx, qy, qz, qw]，相对当前 base_link（odom 系，单位米/四元数）。
- 用导航前必须先把控制器切到 CHASSIS_POSE_CTRL（本模块在 _get_nav() 里自动完成）。
- SDK 不在项目 venv 里，_sdk_bootstrap() 负责把 SDK 包目录注入 sys.path；
  原生依赖（libfastcdr 等）需先执行 `sudo ~/GalbotSDK/install.sh`，
  或 source scripts/env.sh 设置 LD_LIBRARY_PATH。
"""

from __future__ import annotations
import glob
import logging
import math
import os
import platform
import subprocess
import sys
import time

from sdk_wrapper.config import load_safety

logger = logging.getLogger("sdk_wrapper")

_robot = None
_nav = None
_mock_mode = os.environ.get("GALBOT_MOCK", "0") == "1"

#: 真机 init 时要启用的传感器（SensorType 成员名字符串，init 时再解析成枚举）。
#: SDK 规则（g1/__init__.pyi:2785 + 官方 example4 注释）：只有 init(enable_sensor_set)
#: 传入的传感器才能取到数据，没传的 get_rgb_data 一律返回空 dict。
#: 这里在默认集合基础上补齐头部双目（HEAD_LEFT / HEAD_RIGHT），
#: 供「双目识别药盒」等功能包步骤使用（2026-11-07）。
ENABLE_SENSOR_NAMES: tuple = (
    "HEAD_LEFT_CAMERA",
    "HEAD_RIGHT_CAMERA",
    "LEFT_ARM_CAMERA",
    "RIGHT_ARM_CAMERA",
    "LEFT_ARM_DEPTH_CAMERA",
    "RIGHT_ARM_DEPTH_CAMERA",
)


def set_mock_mode(enabled: bool):
    """由 agent/main.py 在启动时根据 --mock 参数调用。"""
    global _mock_mode
    _mock_mode = enabled


def get_robot():
    """公开入口：给场景自定义封装用的共享机器人单例。

    真机返回 GalbotRobot（已 init），mock 返回 MockRobot。
    场景禁止自己 GalbotRobot() init / destroy——必须从这里拿。
    """
    return _get_robot()


def get_nav():
    """公开入口：给场景自定义封装用的共享导航单例（已切 CHASSIS_POSE_CTRL）。

    注意：直接调 nav.move_straight_to 没有 wrapper 层的自动限幅，
    请自觉遵守 safety_config.json 的量级。
    """
    return _get_nav()


def _sdk_bootstrap() -> str:
    """把 GalbotSDK 的 Python 包目录注入 sys.path，返回包目录路径。

    环境变量 GALBOT_SDK_ROOT 可覆盖 SDK 位置（默认 ~/GalbotSDK）。
    如果 galbot_sdk 已可通过 PYTHONPATH 导入（如 HPU 机器人端部署：
    PYTHONPATH=~/g1_runtime），则直接跳过注入。
    注意：本函数只解决 Python 包可见性；原生依赖请提前用 install.sh 安装
    或 source scripts/env.sh / env_hpu.sh（LD_LIBRARY_PATH 须进程启动前生效）。
    """
    import importlib.util
    if importlib.util.find_spec('galbot_sdk') is not None:
        return ''  # 已可用（PYTHONPATH 提供）
    sdk_root = os.environ.get("GALBOT_SDK_ROOT", os.path.expanduser("~/GalbotSDK"))
    arch = platform.machine()  # x86_64 / aarch64
    candidates = sorted(glob.glob(
        os.path.join(sdk_root, "galbot_sdk", f"linux-{arch}-gcc*", "lib", "python")))
    if not candidates:
        raise ImportError(
            f"未找到 galbot_sdk 包目录（arch={arch}），"
            f"请检查 GALBOT_SDK_ROOT（当前={sdk_root}）")
    pkg_dir = candidates[0]
    if pkg_dir not in sys.path:
        sys.path.insert(0, pkg_dir)
    return pkg_dir


def _ensure_nav_service():
    """确保导航规划服务 service_navigation_plan 在跑。

    launcher.service 有时没启动它（实测场地机器人就没起），导致所有导航命令
    返回 WAIT_INITIALIZED。本函数检测缺失则自动拉起（仅 HPU 真机模式生效）。
    """
    nav_svc = "/data/galbot/bin/service_navigation_plan"
    if not os.path.exists(nav_svc):
        return  # 不在 HPU 上（PC 开发模式），跳过
    try:
        r = subprocess.run(["pgrep", "-f", "service_navigation_plan"],
                           capture_output=True, timeout=5)
        if r.returncode == 0:
            return  # 已在跑
    except Exception:
        pass
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/data/galbot/lib:" + env.get("LD_LIBRARY_PATH", "")
    subprocess.Popen([nav_svc], env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    logger.info("已启动 service_navigation_plan（launcher 未自动拉起，手动补启）")
    time.sleep(3)  # 等它初始化（日志显示 ~2s 进入主循环）


def _get_robot():
    """获取全局机器人单例。mock 模式下返回 MockRobot。"""
    global _robot
    if _robot is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockRobot
            _robot = MockRobot()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotRobot
            _robot = GalbotRobot()
        if _mock_mode:
            _robot.init()
        else:
            # 只启用声明过的传感器（官方口径：不传 = 默认集合，头部双目不在其中，
            # 这正是 capture_image 真机返回空的根因，见 vlm_grasp.md「真机上还缺什么」）
            from galbot_sdk.g1 import SensorType
            sensor_set = set()
            for name in ENABLE_SENSOR_NAMES:
                member = getattr(SensorType, name, None)
                if member is None:
                    logger.warning("SensorType.%s 不存在（SDK 版本差异？），跳过", name)
                else:
                    sensor_set.add(member)
            _robot.init(sensor_set)
            logger.info("已启用传感器: %s", sorted(ENABLE_SENSOR_NAMES))
        # 底盘运动需要先切到 CHASSIS_POSE_CTRL（仅真机）
        if not _mock_mode:
            from galbot_sdk.g1 import ControlStatus, G1ControllerName
            res = _robot.switch_controller(G1ControllerName.CHASSIS_POSE_CTRL)
            if res != ControlStatus.SUCCESS:
                logger.warning("切换底盘控制器失败: %s（继续，set_base_pose 可能不生效）", res)
            else:
                logger.info("底盘控制器已切换到 CHASSIS_POSE_CTRL")
        logger.info("机器人实例已初始化 (mock=%s)", _mock_mode)
    return _robot


def _get_nav():
    """获取全局导航单例，并确保底盘控制器已切到 CHASSIS_POSE_CTRL。"""
    global _nav
    if _nav is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockNavigation
            _nav = MockNavigation()
            _nav.init()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import (ControlStatus, G1ControllerName,
                                       GalbotNavigation)
            _ensure_nav_service()  # 导航后端没跑的话先拉起，否则 WAIT_INITIALIZED
            _nav = GalbotNavigation()
            _nav.init()
            robot = _get_robot()
            res = robot.switch_controller(G1ControllerName.CHASSIS_POSE_CTRL)
            if res != ControlStatus.SUCCESS:
                raise RuntimeError(f"切换底盘控制器失败: {res}")
            logger.info("底盘控制器已切换到 CHASSIS_POSE_CTRL")
            # 控制器切换后底盘进入初始化，立即下令会返回 WAIT_INITIALIZED，
            # 给 2 秒就绪窗口（实测足够；不够的话 _move_straight 还有重试兜底）
            time.sleep(2.0)
        logger.info("导航实例已初始化 (mock=%s)", _mock_mode)
    return _nav


def shutdown():
    """真机 SDK 清理（程序退出时必须调用）。mock 模式下仅记录日志。

    注意：SDK destroy() 后同进程无法再 init()，重启 agent 请重新起进程。
    """
    global _robot, _nav
    if _robot is None and _nav is None:
        return
    try:
        if _mock_mode:
            if _nav is not None:
                _nav.stop_navigation()
            logger.info("[MOCK] shutdown()")
        else:
            if _nav is not None:
                _nav.stop_navigation()
            if _robot is not None:
                _robot.request_shutdown()
                _robot.wait_for_shutdown()
                _robot.destroy()
            logger.info("SDK 资源已释放")
    except Exception as e:  # noqa: BLE001 - 清理失败也要让退出流程继续
        logger.error("SDK 清理异常: %s", e)
    finally:
        _robot = None
        _nav = None


def _clamp(value: float, limit: float, name: str) -> float:
    if abs(value) > limit:
        logger.warning("[安全限幅] %s 请求值 %.3f 超过上限 %.3f，已截断", name, value, limit)
    return max(-limit, min(limit, value))


#: set_base_pose 阻塞调用的超时（秒）。
#: 真机实测 15s 偏紧（2026-11-07 场地机器人返回 ControlStatus.TIMEOUT），放宽到 30。
BASE_POSE_TIMEOUT_S = 30.0

#: 非成功状态（TIMEOUT/WAIT_INITIALIZED 等）后的重试次数与间隔。
#: 控制器切换后底盘需要就绪窗口，首次下令偶发超时属正常。
BASE_POSE_RETRIES = 1
BASE_POSE_RETRY_WAIT_S = 2.0


def _status_name(res) -> str:
    """把 SDK 返回的 pybind 枚举转成成员名（'SUCCESS' / 'TIMEOUT' / ...）。

    ⚠️ 千万别用 bool(res) 判断——pybind 枚举成员恒为真，
    TIMEOUT/FAILED 也会被当成成功（真机实测踩过这个坑，2026-11-07）。
    """
    name = getattr(res, "name", None)
    if isinstance(name, str):
        return name
    text = str(res)
    return text.rsplit(".", 1)[-1] if "." in text else text


def _yaw_to_quaternion(yaw_rad: float) -> tuple:
    """偏航角（弧度）→ 四元数 (qx, qy, qz, qw)。"""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _wait_nav_done(nav, timeout_s: float):
    """轮询导航状态直到终态或超时。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            ns = nav.get_navigation_status()
            name = ns.name if hasattr(ns, "name") else str(ns)
        except Exception:
            name = "UNKNOWN"
        if name in ("SUCCESS", "FAILED", "INTERRUPTED",
                    "COLLISION", "CLOSE_TO_OBSTACLE", "OCCUPIED"):
            return name
        time.sleep(0.2)
    return "TIMEOUT"


def _move_straight(x_m: float, yaw_rad: float) -> dict:
    """相对当前位姿走直线 + 转角（用 GalbotRobot.set_base_pose 直接底盘控制）。

    不经过 GalbotNavigation / 导航规划服务，直接命令底盘控制器，
    不需要 localization / map / service_navigation_plan。

    Args:
        x_m:     沿当前朝向的位移（米），正值向前
        yaw_rad: 相对转角（弧度），正值左转
    """
    robot = _get_robot()
    # set_base_pose(x, y, yaw, frame_id='rel(0)'=相对当前位置, is_blocking=True)
    # 真实状态必须如实上报：只有 SUCCESS 才算 success，TIMEOUT 等一律如实透传。
    name = ""
    res = None
    for attempt in range(BASE_POSE_RETRIES + 1):
        res = robot.set_base_pose(x_m, 0.0, yaw_rad,
                                  frame_id='rel(0)', is_blocking=True,
                                  timeout_s=BASE_POSE_TIMEOUT_S)
        name = _status_name(res)
        if name == "SUCCESS":
            break
        if attempt < BASE_POSE_RETRIES:
            logger.warning("set_base_pose 第 %d 次返回 %s（x=%.2f yaw=%.2f），"
                           "%.1fs 后重试一次",
                           attempt + 1, name, x_m, yaw_rad, BASE_POSE_RETRY_WAIT_S)
            time.sleep(BASE_POSE_RETRY_WAIT_S)
    ok = name == "SUCCESS"
    return {"success": ok, "status": name,
            "message": "ok" if ok else f"set_base_pose: {res}"}


# ---------------------------------------------------------------------------
# 底盘运动接口
# ---------------------------------------------------------------------------

def move_forward(distance_cm: float) -> dict:
    """前进指定距离（厘米，负值即后退）。自动限幅，阻塞执行。

    Returns:
        {"success": bool, "actual_distance_cm": float, "message": str}
    """
    cfg = load_safety()
    distance_m = _clamp(distance_cm / 100.0, cfg["max_base_distance_m"], "distance")
    r = _move_straight(distance_m, 0.0)
    r["actual_distance_cm"] = distance_m * 100
    return r


def move_backward(distance_cm: float) -> dict:
    """后退指定距离（厘米）。"""
    return move_forward(-abs(distance_cm))


def turn(angle_deg: float) -> dict:
    """原地转向（度），正值左转、负值右转。自动限幅。"""
    cfg = load_safety()
    angle_deg = _clamp(angle_deg, cfg["max_turn_deg"], "turn_angle")
    r = _move_straight(0.0, math.radians(angle_deg))
    r["actual_angle_deg"] = angle_deg
    return r


def stop_base() -> dict:
    """立即停止底盘运动（场景退出前建议调用）。"""
    if _nav is not None and hasattr(_nav, "stop_navigation"):
        _nav.stop_navigation()
    else:
        _get_nav().stop_navigation()
    return {"success": True, "message": "base stopped"}


# ---------------------------------------------------------------------------
# 感知接口
# ---------------------------------------------------------------------------

def read_sensor(sensor_name: str):
    """读取传感器数据。

    Args:
        sensor_name: 传感器名称，如 'front_camera' / 'lidar' / 'imu'

    Returns:
        传感器数据（具体格式取决于传感器类型）

    TODO(负责人A): 按 Galbot SDK v1.10.0 实际接口补齐相机订阅、雷达读取等。
    真机可用接口参考：robot.get_imu_data(sensor_id) / get_depth_data(camera_id)。
    """
    robot = _get_robot()
    if hasattr(robot, "get_sensor_data"):
        return robot.get_sensor_data(sensor_name)
    raise NotImplementedError(f"传感器 '{sensor_name}' 接口尚未封装，请联系负责人 A")


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


def get_joint_positions(joint_groups=None, joint_names=None) -> dict:
    """读指定关节组的当前关节角（弧度）。**只读**，不发任何运动指令。

    示教采点专用：拖动手臂到位后读一次，把 positions 连同 names 一起存 json 回放。

    ⚠️ 两个必守细节（踩过）：
      1. joint_groups 必须显式给（如 ["right_arm"]），joint_names 传 []；
         无参调用返回 25 个错位数据，且不报错 —— 比报错更危险。
      2. 返回的 joint_names/positions 必须**成对保存**，回放时按名字下发，
         不能按索引猜，否则左右臂一混就撞。
    """
    groups = [str(g) for g in (joint_groups or ["right_arm"])]
    names = [str(n) for n in (joint_names or [])]
    if _mock_mode:
        mock_names = names or [f"{groups[0]}_joint{i}" for i in range(1, 8)]
        return _mock_ok("get_joint_positions",
                        joint_names=mock_names,
                        joint_positions=[-1.90, 1.46, 0.54, 1.96, 0.0, 0.4, 0.0])
    robot = _get_robot()
    try:
        pos = robot.get_joint_positions(joint_groups=groups, joint_names=names)
        nm = robot.get_joint_names(only_active_joint=True, joint_groups=groups) if not names else names
    except Exception as exc:  # noqa: BLE001 - 读关节失败要如实报，不许编数
        return {"success": False, "completion": "failed",
                "message": f"get_joint_positions 调用失败: {exc}"}
    if pos is None or len(pos) == 0:
        return {"success": False, "completion": "failed",
                "message": "get_joint_positions 返回空（关节组名写错或机器人未就绪）"}
    pos = [float(v) for v in pos]
    if len(nm) != len(pos):
        # 数量对不上就不猜对应关系，直接判不可信
        return {"success": False, "completion": "failed",
                "message": f"关节名 {len(nm)} 个、角度 {len(pos)} 个，数量不一致，拒绝返回"}
    return {"success": True, "message": "ok", "completion": "completed",
            "joint_names": [str(n) for n in nm], "joint_positions": pos}


def get_gripper_width(side: str = "right") -> dict:
    """读夹爪当前开度（米）。**只读**，不发动作。"""
    if _mock_mode:
        return _mock_ok("get_gripper_width", width=0.10, side=side)
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    gripper_name = getattr(gm.G1JointGroup, f"{side}_gripper", None)
    if not isinstance(gripper_name, str):
        return {"success": False, "completion": "failed",
                "message": f"未知夹爪侧 {side!r}，G1JointGroup 无 {side}_gripper"}
    robot = _get_robot()
    state = robot.get_gripper_state(gripper_name)
    width = getattr(state, "width", None)
    if width is None:
        return {"success": False, "completion": "failed",
                "message": "get_gripper_state 返回无 width 字段"}
    return {"success": True, "message": "ok", "completion": "completed",
            "width": float(width), "side": side}


#: capture_image 支持的相机名 → SensorType 成员名（头部双目 + 手臂，按需扩充）
CAMERA_MAP = {
    "head_left": "HEAD_LEFT_CAMERA",
    "head_right": "HEAD_RIGHT_CAMERA",
    "left_arm": "LEFT_ARM_CAMERA",
    "right_arm": "RIGHT_ARM_CAMERA",
}


def capture_image(tag: str = "", camera: str = "head_left") -> dict:
    """拍一帧 RGB（JPEG），存到 logs/frames/，返回 frame_id=文件路径。

    camera 支持 "head_left" / "head_right"（头部双目，识别药盒用）、
    "left_arm" / "right_arm"（腕部）。默认头部左目（兼容旧调用）。
    相机必须在 ENABLE_SENSOR_NAMES 里（init 时已启用），否则返回空。
    """
    member_name = CAMERA_MAP.get(camera)
    if member_name is None:
        return {"success": False, "completion": "failed",
                "message": f"未知相机 '{camera}'，可选: {sorted(CAMERA_MAP)}"}
    if _mock_mode:
        return _mock_ok("capture_image", frame_id=f"mock_frame_{camera}_{tag or '0'}")
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    robot = _get_robot()
    sensor = getattr(gm.SensorType, member_name, None)
    if sensor is None:
        return {"success": False, "completion": "failed",
                "message": f"SensorType.{member_name} 不存在"}
    frame = robot.get_rgb_data(sensor, gm.RgbOutputFormat.JPEG, True)
    if not frame or "data" not in frame:
        return {"success": False, "completion": "failed",
                "message": "get_rgb_data 返回空（相机不可用或未订阅）"}
    out_dir = os.path.join(os.path.dirname(__file__), "..", "logs", "frames")
    os.makedirs(out_dir, exist_ok=True)
    frame_id = os.path.join(out_dir, f"frame_{int(time.time())}_{tag or 'shot'}.jpg")
    with open(frame_id, "wb") as f:
        f.write(frame["data"])
    return {"success": True, "message": "ok", "frame_id": frame_id}
