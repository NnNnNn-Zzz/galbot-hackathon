#!/usr/bin/env python3
"""底盘"完全不动 + TIMEOUT"分层诊断脚本。

背景（2026-11-07 真机实测）：set_base_pose 返回 ControlStatus.TIMEOUT，
机器人无任何位移。TIMEOUT 只说明"阻塞等待没等到完成"，不动的原因要分层查：

    第 1 层  控制器权限   switch_controller 结果 + get_active_controller('chassis')
                           —— 被示教器/其他程序占用时 WBCS 会拒绝
    第 2 层  底盘读数     get_base_velocity()（当下是否在动）+ get_odom()（起点）
    第 3 层  非阻塞下令   set_base_pose(is_blocking=False) 立即返回值
                           —— FAULT/WAIT 状态会立刻暴露，不用等 15s 超时
    第 4 层  位移实测      轮询 odom 5 秒，位移是否真的发生
    第 5 层  阻塞确认     正常流程重跑一次，前后 odom 差对比

结论对照表（脚本最后自动打印）：
    非阻塞立刻 FAULT            → 底盘权限被占（关掉占用方再跑）
    非阻塞 SUCCESS 但 odom 不变  → 底盘收到没执行：查物理急停 / 低电 / 机器人模式
    非阻塞 SUCCESS 且 odom 变了  → 底盘没毛病，只是阻塞确认超时（调大 timeout 即可）
    switch_controller 非 SUCCESS → 控制器切换被拒（优先级/占用问题）

用法（机器人 HPU 上）：
    cd ~/galbot_hackathon && source scripts/env_hpu.sh
    python3 scripts/diag_base_motion.py             # 全流程（含 10cm 移动测试）
    python3 scripts/diag_base_motion.py --no-move   # 只读诊断，绝不动（最安全，先跑这个）
"""

from __future__ import annotations
import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
LOG = logging.getLogger("diag_base")

# ------------------------------- 可调参数 -------------------------------- #
MOVE_X_M = 0.10        # 第 5 层移动测试的距离（米）
POLL_S = 5.0           # 第 4 层轮询时长（秒）
POLL_INTERVAL_S = 0.5  # 轮询间隔


def _name(res) -> str:
    """枚举 → 成员名。pybind 枚举禁止 bool() 判断（恒真，会踩坑）。"""
    n = getattr(res, "name", None)
    if isinstance(n, str):
        return n
    return str(res).rsplit(".", 1)[-1]


def _odom_xy(robot) -> tuple[float, float] | None:
    """读 odom 的 (x, y)，失败返回 None。"""
    try:
        o = robot.get_odom()
        pos = o.get("position")
        return (float(pos[0]), float(pos[1])) if pos else None
    except Exception as exc:  # noqa: BLE001
        LOG.warning("get_odom 失败: %s", exc)
        return None


def _dist(a, b) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def diag(no_move: bool) -> int:
    from sdk_wrapper import wrapper

    print("\n═══ 第 1 层：控制器权限 ═══")
    robot = wrapper.get_robot()   # 单例初始化（内部已切一次控制器）
    try:
        groups = robot.get_joint_group_names()
        print(f"  关节组: {list(groups)}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_joint_group_names 不可用: {exc}")
    # 显式再切一次并打印结果（wrapper 只写日志不返回）
    from galbot_sdk.g1 import G1ControllerName
    res = robot.switch_controller(G1ControllerName.CHASSIS_POSE_CTRL)
    print(f"  switch_controller(CHASSIS_POSE_CTRL) => {_name(res)}")
    try:
        active = robot.get_active_controller("chassis")
        print(f"  get_active_controller('chassis')   => {active}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_active_controller 不可用: {exc}")
    switch_ok = _name(res) == "SUCCESS"

    print("\n═══ 第 2 层：底盘读数（只读） ═══")
    try:
        print(f"  get_base_velocity() => {robot.get_base_velocity()}")
    except Exception as exc:  # noqa: BLE001
        print(f"  get_base_velocity 不可用: {exc}")
    start = _odom_xy(robot)
    print(f"  get_odom() 起点     => {start}")

    if no_move:
        print("\n--no-move 模式：只读诊断结束（不下发任何运动指令）")
        wrapper.shutdown()
        return 0

    input(f"\n⚠️  接下来底盘将尝试平移 {MOVE_X_M} m，确认周围安全后按 Enter...")

    print("\n═══ 第 3 层：非阻塞下令（立即返回，不等超时） ═══")
    res = robot.set_base_pose(MOVE_X_M, 0.0, 0.0,
                              frame_id="rel(0)", is_blocking=False)
    nb = _name(res)
    print(f"  set_base_pose(is_blocking=False) => {nb}")

    print(f"\n═══ 第 4 层：轮询 odom {POLL_S}s，看位移是否真实发生 ═══")
    max_disp = 0.0
    for i in range(int(POLL_S / POLL_INTERVAL_S)):
        time.sleep(POLL_INTERVAL_S)
        now = _odom_xy(robot)
        if start and now:
            d = _dist(start, now)
            max_disp = max(max_disp, d)
            print(f"  t={POLL_INTERVAL_S * (i + 1):.1f}s  odom=({now[0]:+.3f},{now[1]:+.3f})  位移 {d * 100:.1f} cm")
        else:
            print(f"  t={POLL_INTERVAL_S * (i + 1):.1f}s  odom 读取失败")
    moved = max_disp > 0.02  # 2cm 阈值：编码器噪声一般 <1cm

    print("\n═══ 第 5 层：阻塞流程复测（正常 wrapper 路径） ═══")
    before = _odom_xy(robot)
    r = wrapper.move_forward(MOVE_X_M * 100)
    print(f"  wrapper.move_forward => {r}")
    after = _odom_xy(robot)
    if before and after:
        print(f"  阻塞前后 odom 位移: {_dist(before, after) * 100:.1f} cm")
    wrapper.shutdown()

    # ───────────────────────── 结论 ───────────────────────── #
    print("\n" + "#" * 62)
    print("诊断结论")
    print("#" * 62)
    if not switch_ok:
        print("❗ 控制器切换失败（第 1 层）：WBCS 拒绝了 CHASSIS_POSE_CTRL。")
        print("   常见原因：示教器/官方 App/其他程序占着底盘权限。")
        print("   处理：关掉占用方，或按 SDK 文档走 acquire_controller 流程。")
    if nb == "FAULT":
        print("❗ 非阻塞下令返回 FAULT（第 3 层）：底盘控制器状态异常/无权限。")
    if nb == "SUCCESS" and not moved:
        print("❗ 指令下发成功但底盘没动（第 4 层 odom 零位移）：")
        print("   按顺序检查：① 物理急停按钮是否被按下（最常见！）")
        print("              ② 机器人是否处于维护/示教模式（App 里切回运行模式）")
        print("              ③ 电量是否过低（低电保护会拒动）")
        print("              ④ 试试 resume_from_emergency_stop() 解除软急停")
    if nb == "SUCCESS" and moved:
        print("✅ 底盘实际能动能走（第 4 层有位移）：问题只是阻塞确认超时。")
        print("   wrapper 已把超时放宽到 30s + 失败重试 1 次，重新部署后应显示 SUCCESS。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="底盘不动分层诊断")
    parser.add_argument("--no-move", action="store_true",
                        help="只读诊断（控制器/速度/odom），绝不下发运动指令")
    args = parser.parse_args()

    if os.environ.get("GALBOT_MOCK") == "1":
        print("本脚本只用于真机诊断，mock 没有意义。去掉 GALBOT_MOCK=1 再跑。")
        return 2
    try:
        import galbot_sdk  # noqa: F401
    except ImportError:
        print("找不到 galbot_sdk：先 source scripts/env_hpu.sh（HPU）或 scripts/env.sh（WSL）")
        return 2

    try:
        return diag(args.no_move)
    except KeyboardInterrupt:
        print("\n用户中断。")
        return 130


if __name__ == "__main__":
    sys.exit(main())
