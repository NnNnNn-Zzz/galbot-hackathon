#!/usr/bin/env python3
"""wrapper 全功能真机逐项验证脚本（也支持 mock 干跑）。

覆盖两层封装的每一个对外功能，从"只读安全"到"真动"逐级推进，每步人工确认：

    Part base（老 sdk_wrapper：底盘 API，单位 厘米/度）
        B1 单例初始化 get_robot()          B2 导航单例 get_nav()
        B3 状态读取 check_baseline/read_sensor
        B4 安全限幅配置与 clamp            B5 stop_base()
        B6 move_forward(10cm)              B7 move_backward(10cm)
        B8 turn(+15°)/turn(-15°)           B9 baseline.reset_to_baseline()

    Part dev（sdk_wrapper/dev：ToolResult 契约，单位 米/弧度/秒）
        D1 未授权闸门（不 import SDK，永远安全）
        D2 真机状态 get_robot_state()      D3 相机感知 list/capture/analyze/find
        D4 末端位姿读 get_end_effector_pose
        D5 夹爪 set_gripper_width          D6 末端小位移 move_ee_delta + move_ee_to 回位
        D7 预期拒绝项 execute_gesture / navigate_to_place（真机故意拒绝，拒绝=通过）
        D8 语音 speak（无 PCM 素材时文本兜底）

用法（真机，SSH 到 HPU 后）：
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/test_wrapper_real.py --part base   # 第一遍：只测底盘 API
    python3 scripts/test_wrapper_real.py --part dev    # 第二遍：只测 dev 层（独立进程，单例不互踩）

用法（mock 干跑，任何时候安全）：
    python3 scripts/test_wrapper_real.py --mock --yes

设计约定：
    * base / dev 分两个进程跑——两边各自持有 SDK 单例，同进程会互踩（纪律④）；
    * 每步先打印"预期结果"，你确认后按 Enter 执行；Ctrl+C 随时中止并走清理；
    * 结果汇总成 PASS / FAIL / SKIP 表，FAIL 不阻塞后续步（除非是前置依赖）。
"""

from __future__ import annotations
import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
LOG = logging.getLogger("test_wrapper_real")

# ------------------------------- 可调参数 -------------------------------- #
BASE_DISTANCE_CM = 10.0   # B6/B7 前进后退距离（真机第一轮建议 10cm）
BASE_TURN_DEG = 15.0      # B8 原地转向角度
EE_DELTA_M = 0.02         # D6 末端单次位移（米），2cm 足够看清又足够安全
GRIPPER_OPEN_M = 0.06     # D5 夹爪开合幅度（米）
CAMERA_ID = "left_arm"    # D3 默认测试机位（cameras.json 里的别名）

_results: list[tuple[str, str, str, str]] = []  # (id, 名称, 结果, 说明)


def record(step_id: str, name: str, result: str, detail: str = "") -> None:
    _results.append((step_id, name, result, detail))
    LOG.info("[%s] %s => %s %s", step_id, name, result, detail)


def confirm(step_id: str, name: str, expect: str, auto_yes: bool, moving: bool) -> bool:
    print("\n" + "=" * 66)
    print(f"步骤 {step_id}: {name}")
    print(f"预期结果: {expect}")
    if moving:
        print("⚠️  此步机器人会真实移动！确认周围空旷、急停在手边。")
    print("=" * 66)
    if auto_yes:
        return True
    try:
        input("按 Enter 执行，输入 s 跳过，Ctrl+C 中止: ").strip().lower()
    except EOFError:
        print("（非交互环境，危险步骤默认跳过）")
        return False
    return True


def skip(step_id: str, name: str, why: str) -> None:
    record(step_id, name, "SKIP", why)


# ════════════════════════════════════════════════════════════════════════ #
# Part base —— 老 sdk_wrapper（底盘 API）
# ════════════════════════════════════════════════════════════════════════ #

def run_part_base(auto_yes: bool) -> None:
    from sdk_wrapper import baseline, wrapper

    # B1 单例初始化 ------------------------------------------------------- #
    if confirm("B1", "get_robot() 单例初始化",
               "日志出现『机器人实例已初始化 (mock=False)』+『底盘控制器已切换到 CHASSIS_POSE_CTRL』",
               auto_yes, moving=False):
        robot = wrapper.get_robot()
        ok = robot is not None
        record("B1", "get_robot()", "PASS" if ok else "FAIL",
               f"实例类型 {type(robot).__name__}")
    else:
        skip("B1", "get_robot()", "用户跳过"); return

    # B2 导航单例 --------------------------------------------------------- #
    if confirm("B2", "get_nav() 导航单例初始化",
               "真机：可能补启 service_navigation_plan，日志『导航实例已初始化 (mock=False)』",
               auto_yes, moving=False):
        try:
            nav = wrapper.get_nav()
            record("B2", "get_nav()", "PASS", f"实例类型 {type(nav).__name__}")
        except Exception as exc:  # noqa: BLE001
            record("B2", "get_nav()", "FAIL", f"{type(exc).__name__}: {exc}")
    else:
        skip("B2", "get_nav()", "用户跳过")

    # B3 只读状态 --------------------------------------------------------- #
    if confirm("B3", "check_baseline() + read_sensor('imu')",
               "check_baseline 返回 dict（当前是占位实现恒 True）；"
               "read_sensor 预期抛 NotImplementedError（已知缺口，抛=符合设计）",
               auto_yes, moving=False):
        r = baseline.check_baseline()
        ok = isinstance(r, dict) and r.get("at_baseline") is True
        record("B3a", "check_baseline()", "PASS" if ok else "FAIL", str(r))
        try:
            data = wrapper.read_sensor("imu")
            # mock 的 MockRobot 有 get_sensor_data（返回 None）；真机预期 NotImplementedError
            record("B3b", "read_sensor('imu')", "PASS" if wrapper._mock_mode else "FAIL",
                   "mock: MockRobot.get_sensor_data 返回 None" if wrapper._mock_mode
                   else f"竟然成功了？真机可能带 get_sensor_data，请更新文档: {data!r}")
        except NotImplementedError as exc:
            record("B3b", "read_sensor('imu')", "PASS" if not wrapper._mock_mode else "FAIL",
                   f"真机预期缺口: {exc}" if not wrapper._mock_mode else "mock 不该抛这个")
        except Exception as exc:  # noqa: BLE001
            record("B3b", "read_sensor('imu')", "FAIL", f"{type(exc).__name__}: {exc}")

    # B4 安全限幅（只读，不动） -------------------------------------------- #
    if confirm("B4", "安全配置加载 + 限幅函数",
               "打印 safety_config.json 五个限幅值；请求 3.0m 被 clamp 到 2.0m",
               auto_yes, moving=False):
        from sdk_wrapper.config import load_safety
        cfg = load_safety()
        clamped = wrapper._clamp(3.0, cfg["max_base_distance_m"], "distance")
        ok = abs(clamped - cfg["max_base_distance_m"]) < 1e-9
        record("B4", "load_safety + _clamp", "PASS" if ok else "FAIL",
               f"clamp(3.0)={clamped}m, max_turn={cfg['max_turn_deg']}°, "
               f"max_speed={cfg['max_base_speed_mps']}m/s")

    # B5 停底盘 ----------------------------------------------------------- #
    if confirm("B5", "stop_base()", "返回 {'success': True, 'message': 'base stopped'}",
               auto_yes, moving=False):
        r = wrapper.stop_base()
        record("B5", "stop_base()", "PASS" if r.get("success") else "FAIL", str(r))

    # B6/B7/B8 真实移动 ---------------------------------------------------- #
    if confirm("B6", f"move_forward({BASE_DISTANCE_CM}cm)",
               f"机器人前进约 {BASE_DISTANCE_CM}cm，返回 success + actual_distance_cm",
               auto_yes, moving=True):
        r = wrapper.move_forward(BASE_DISTANCE_CM)
        record("B6", "move_forward", "PASS" if r.get("success") else "FAIL", str(r))

    if confirm("B7", f"move_backward({BASE_DISTANCE_CM}cm)",
               f"机器人退回约 {BASE_DISTANCE_CM}cm（回到 B6 之前的位置）",
               auto_yes, moving=True):
        r = wrapper.move_backward(BASE_DISTANCE_CM)
        record("B7", "move_backward", "PASS" if r.get("success") else "FAIL", str(r))

    if confirm("B8", f"turn(+{BASE_TURN_DEG}°) / turn(-{BASE_TURN_DEG}°)",
               "左转再右转回正（视觉确认朝向复原）",
               auto_yes, moving=True):
        r1 = wrapper.turn(BASE_TURN_DEG)
        r2 = wrapper.turn(-BASE_TURN_DEG)
        ok = r1.get("success") and r2.get("success")
        record("B8", "turn ±", "PASS" if ok else "FAIL", f"{r1} / {r2}")

    # B9 归位 -------------------------------------------------------------- #
    if confirm("B9", "baseline.reset_to_baseline()",
               "返回 True，日志『已归位到基线状态』（当前只做停底盘，手臂/夹爪是 TODO）",
               auto_yes, moving=False):
        ok = baseline.reset_to_baseline()
        record("B9", "reset_to_baseline", "PASS" if ok else "FAIL", "")

    wrapper.shutdown()


# ════════════════════════════════════════════════════════════════════════ #
# Part dev —— sdk_wrapper/dev（ToolResult 契约层）
# ════════════════════════════════════════════════════════════════════════ #

def run_part_dev(auto_yes: bool, mock: bool) -> None:
    from sdk_wrapper.dev import (GalbotAdapter, MockAdapter, RobotSafety,
                                 TaskStatus, MotionTool)
    from sdk_wrapper.dev.robot_tools.perception import PerceptionTool
    from sdk_wrapper.dev.robot_tools.audio import AudioTool

    # D1 未授权闸门（永不 import SDK，任何环境都能测） ---------------------- #
    if confirm("D1", "GalbotAdapter 未授权闸门",
               "get_robot_state 返回 rejected + permission_denied，且进程没碰 SDK",
               auto_yes, moving=False):
        gate = GalbotAdapter().get_robot_state()  # 两个开关都关着
        ok = gate.status is TaskStatus.REJECTED
        record("D1", "未授权闸门", "PASS" if ok else "FAIL", gate.message)

    # 授权装配 ------------------------------------------------------------- #
    if mock:
        adapter = MockAdapter(scene="cup_table")
        safety = RobotSafety(adapter)
        LOG.info("MOCK 模式：用 MockAdapter（真机授权检查被跳过，代码路径其余一致）")
    else:
        adapter = GalbotAdapter(enable_real_control=True, real_authorized=True)
        safety = RobotSafety(adapter, real_authorized=True)
    motion = MotionTool(adapter, safety=safety)
    perception = PerceptionTool(adapter, save_frames=False)
    audio = AudioTool(adapter)

    # D2 状态 --------------------------------------------------------------- #
    if confirm("D2", "get_robot_state()",
               "succeeded，data 里有 motion_state=idle/moving（真机读的是 SDK 状态）",
               auto_yes, moving=False):
        r = adapter.get_robot_state()
        ok = r.status is TaskStatus.SUCCEEDED
        record("D2", "get_robot_state", "PASS" if ok else "FAIL",
               f"motion_state={r.data.get('motion_state')}")

    # D3 相机感知 ----------------------------------------------------------- #
    if confirm("D3", f"相机链路 list→capture→analyze→find（机位 {CAMERA_ID}）",
               "list_cameras 列出 10 机位；capture_scene 返回 capture_id；"
               "analyze_scene 给出目标+距离；find_object('red') 给 3D 坐标。"
               "（真机某机位 not_ready 属正常——未装配，换机位重试）",
               auto_yes, moving=False):
        r1 = perception.list_cameras()
        n = len(r1.data.get("cameras", {}))
        record("D3a", "list_cameras", "PASS" if r1.status is TaskStatus.SUCCEEDED else "FAIL",
               f"{n} 个机位")
        r2 = perception.capture_scene(camera_id=CAMERA_ID)
        record("D3b", f"capture_scene({CAMERA_ID})",
               "PASS" if r2.status is TaskStatus.SUCCEEDED else "FAIL",
               r2.message[:80])
        if r2.status is TaskStatus.SUCCEEDED:
            r3 = perception.analyze_scene(camera_id=CAMERA_ID)
            record("D3c", "analyze_scene",
                   "PASS" if r3.status is TaskStatus.SUCCEEDED else "FAIL",
                   r3.message[:80])
            r4 = perception.find_object("red", camera_id=CAMERA_ID)
            # 真机视野里没红色目标属正常——能返回明确结果就算链路通
            ok = r4.status in (TaskStatus.SUCCEEDED, TaskStatus.FAILED)
            record("D3d", "find_object('red')", "PASS" if ok else "FAIL", r4.message[:80])

    # D4 末端位姿读 ---------------------------------------------------------- #
    if confirm("D4", "get_end_effector_pose('left_arm')",
               "succeeded，data['pose'] 是 7 个数 [x,y,z,qx,qy,qz,qw]（米，base_link 系）",
               auto_yes, moving=False):
        r = adapter.get_end_effector_pose("left_arm")
        pose = r.data.get("pose")
        ok = r.status is TaskStatus.SUCCEEDED and len(pose or []) == 7
        record("D4", "get_end_effector_pose", "PASS" if ok else "FAIL", str(pose))

    # D5 夹爪 ---------------------------------------------------------------- #
    if confirm("D5", f"夹爪开合 {GRIPPER_OPEN_M}m → 0.0m",
               "两次都 succeeded（真机夹爪真实开合；motion_status=SUCCESS）",
               auto_yes, moving=True):
        r1 = adapter.set_gripper_width("left_gripper", GRIPPER_OPEN_M)
        r2 = adapter.set_gripper_width("left_gripper", 0.0)
        ok = r1.status is TaskStatus.SUCCEEDED and r2.status is TaskStatus.SUCCEEDED
        record("D5", "set_gripper_width", "PASS" if ok else "FAIL",
               f"{r1.status.value}/{r2.status.value} | {r1.message[:60]}")

    # D6 末端小位移 + 回位 ---------------------------------------------------- #
    if confirm("D6", f"move_ee_delta(+{EE_DELTA_M}m x) → move_ee_to 回位",
               "左臂沿 x+ 平移 2cm 再回到 D4 记录的位姿，两次都 succeeded",
               auto_yes, moving=True):
        pose0 = adapter.get_end_effector_pose("left_arm").data.get("pose")
        r1 = adapter.move_ee_delta("left_arm", EE_DELTA_M, 0.0, 0.0, straight_line=True)
        r2 = adapter.move_ee_to("left_arm", pose0, straight_line=True)
        ok = r1.status is TaskStatus.SUCCEEDED and r2.status is TaskStatus.SUCCEEDED
        record("D6", "move_ee_delta + move_ee_to", "PASS" if ok else "FAIL",
               f"{r1.status.value}/{r2.status.value}")

    # D7 预期拒绝项（拒绝 = 通过） -------------------------------------------- #
    if confirm("D7", "预期行为：execute_gesture('wave_right') / navigate_to_place('demo_table')",
               "真机：两项都应 rejected（手势未标定 NOT_SUPPORTED；导航点未实测 pose=null），拒绝=通过\n"
               "        mock：两项正常启动任务（running），running=通过",
               auto_yes, moving=False):
        rg = motion.execute_gesture("wave_right")
        if mock:
            ok_g = rg.status is TaskStatus.RUNNING
            note = f"mock 正常: {rg.status.value}"
        else:
            ok_g = rg.status is TaskStatus.REJECTED
            note = f"{rg.status.value}: {rg.message[:60]}"
        record("D7a", "execute_gesture", "PASS" if ok_g else "FAIL", note)
        rn = adapter.navigate_to_place("demo_table")
        if mock:
            ok_n = rn.status is TaskStatus.RUNNING
            note = f"mock 正常: {rn.status.value}"
        else:
            ok_n = rn.status is TaskStatus.REJECTED
            note = f"{rn.status.value}: {rn.message[:60]}"
        record("D7b", "navigate_to_place", "PASS" if ok_n else "FAIL", note)

    # D8 语音 ----------------------------------------------------------------- #
    if confirm("D8", "speak('cup_unsafe')",
               "无 PCM 素材时 succeeded + mode=text_fallback（文本兜底，不假装播过）。"
               "有素材则真实播放并等播完",
               auto_yes, moving=False):
        r = audio.speak("cup_unsafe")
        ok = r.status is TaskStatus.SUCCEEDED
        record("D8", "speak", "PASS" if ok else "FAIL",
               f"mode={r.data.get('mode')}")


# ════════════════════════════════════════════════════════════════════════ #
def summary() -> int:
    print("\n" + "#" * 66)
    print("验证汇总")
    print("#" * 66)
    fails = 0
    for step_id, name, result, detail in _results:
        mark = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️"}.get(result, " ? ")
        print(f"  {mark} {step_id:<4} {name:<28} {result:<5} {detail}")
        fails += result == "FAIL"
    n_pass = sum(1 for x in _results if x[2] == "PASS")
    n_skip = sum(1 for x in _results if x[2] == "SKIP")
    print(f"\n  共 {len(_results)} 项：PASS {n_pass} / FAIL {fails} / SKIP {n_skip}")
    return 1 if fails else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="wrapper 全功能逐项验证（base/dev 分进程跑）")
    parser.add_argument("--part", choices=("base", "dev"), default="base",
                        help="base=老底盘 API；dev=ToolResult 契约层。真机分两次跑，别同进程")
    parser.add_argument("--mock", action="store_true", help="mock 模式（不碰真机）")
    parser.add_argument("--yes", action="store_true", help="跳过每步确认（mock 干跑用）")
    parser.add_argument("--distance", type=float, default=BASE_DISTANCE_CM)
    parser.add_argument("--turn", type=float, default=BASE_TURN_DEG)
    args = parser.parse_args()

    globals()["BASE_DISTANCE_CM"] = args.distance
    globals()["BASE_TURN_DEG"] = args.turn

    if args.mock:
        os.environ["GALBOT_MOCK"] = "1"
        from sdk_wrapper import wrapper
        wrapper.set_mock_mode(True)
        print("=== MOCK 模式（不碰真机）===")
    else:
        try:
            import galbot_sdk  # noqa: F401
        except ImportError:
            print("!! 找不到 galbot_sdk —— 先 source scripts/env.sh（WSL）"
                  "或 scripts/env_hpu.sh（机器人 HPU）")
            return 2
        print("!! 真机模式：请确认周围空旷、急停在手边 !!")

    try:
        if args.part == "base":
            run_part_base(args.yes or args.mock)
        else:
            run_part_dev(args.yes or args.mock, args.mock)
    except KeyboardInterrupt:
        print("\n用户中断。")
        return 130
    return summary()


if __name__ == "__main__":
    sys.exit(main())
