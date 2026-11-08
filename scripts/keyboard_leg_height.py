#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G1 腰部高度（腿高机构）键盘遥控 —— 直上直下。

为什么用 set_leg_height
------------------------
G1 没有单独的「腰升」轴。整机竖直升降由 SDK 的 LEG_HEIGHT_CTRL 完成：
它协同解算前三条腿关节（leg_joint1/2/3），把 head_base_link 抬上抬下。
这正好对应「腰部直上直下」，而且不会像 leg_joint4/5 那样产生偏航/滚转副作用
（见 docs/TROUBLESHOOTING.md 中 2026-11-07 的实测记录）。

官方接口
--------
    GalbotRobot.set_leg_height(target_m, duration_s, blocking, timeout_s)
    GalbotRobot.get_transform("base_link", "head_base_link")   # 读实际高度

安全设计（很重要，别随意改小）
------------------------------
1. SDK 支持区间 0.96~1.54 m；本脚本默认再内缩到 1.00~1.50 m（--min/--max 可调，
   但永远不会超出 SDK 区间）。
2. 每一小步都「先读实测高度 → 再下发」，位置天然不累积漂移，贴边界自动停住。
3. 单步硬上限 2 cm，默认速度 3 cm/s。
4. 松手即停：终端只有 auto-repeat 字符流，用 keep-alive 时间窗判定「还在按」。
5. 空格 = 急停，q = 退出，Ctrl-C 同样安全退出。
6. ★ 已知坑：set_leg_height 可能返回 FAULT 但实际已到位。所以每步都以回读的
   head_base_link Z 为准，只有「状态异常 且 实测没到位」才停机报错。

用法
----
    source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 scripts/keyboard_leg_height.py --status   # 只读：打印当前高度后退出
    python3 scripts/keyboard_leg_height.py            # 交互：单键，不用回车
    python3 scripts/keyboard_leg_height.py --line     # 无 tty 兜底：每行一个字符

按键
----
    w / ↑     上升一步（按住连续上升）
    s / ↓     下降一步（按住连续下降）
    空格      急停（停在下发前的位置）
    p         打印一行完整状态
    r         回到启动时的高度
    q         退出

退出时如果偏离起始高度超过 2 cm，会问要不要回原位（默认回）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)


# --------------------------------------------------------------------------
# 常量：安全边界与节拍
# --------------------------------------------------------------------------

#: SDK 定义的支持区间（docstring of SquatTool），不是机械极限也不是平衡极限
SDK_HEIGHT_MIN = 0.96
SDK_HEIGHT_MAX = 1.54

#: 默认软件限位，两侧各留 4 cm 余量
DEFAULT_SOFT_MIN = 1.00
DEFAULT_SOFT_MAX = 1.50

#: 单步硬上限，沿用仓库既有的 MAX_STEP_M
MAX_STEP_M = 0.02

#: 默认速度（cm/s）与每步运动时长（s）：step = speed/100 * duration
DEFAULT_SPEED_CM_S = 3.0
DEFAULT_STEP_S = 0.25

#: 空闲（没按键）时的轮询周期
IDLE_POLL_S = 0.05

#: 认为「已到位」的容差
POSE_TOL_M = 0.005

#: 实测高度与目标偏差超过它就停机
DRIFT_ABORT_M = 0.03

#: 连续多少步没有实际位移就判卡死
STALL_STEPS = 5

#: 键盘 keep-alive：首次按下 / auto-repeat 续命窗口
FIRST_HOLD_S = 0.90
REPEAT_HOLD_S = 0.40
REPEAT_GAP_S = 0.60

#: 读取高度的具体链路（z 就是腰高）
TRANSFORM_FRAME = "head_base_link"

_PRINT_LOCK = threading.Lock()


def say(msg: str) -> None:
    """线程安全的单行输出。"""
    with _PRINT_LOCK:
        sys.stdout.write("\r\033[K" + msg + "\n")
        sys.stdout.flush()


def _enum_name(value) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    return str(value).rsplit(".", 1)[-1]


# --------------------------------------------------------------------------
# SDK 薄封装
# --------------------------------------------------------------------------


def open_robot():
    """取共享 SDK 单例（wrapper 内部已做重试与控制器切换）。"""
    from sdk_wrapper.wrapper import get_robot

    return get_robot()


def read_height(robot) -> float:
    """读 head_base_link 在 base_link 下的 Z，即当前「腰高」。"""
    pose, _ts = robot.get_transform(
        "base_link", TRANSFORM_FRAME, timestamp_ns=0, timeout_ms=500
    )
    if pose is None or len(pose) < 3:
        raise RuntimeError("get_transform 返回空，取不到 %s" % TRANSFORM_FRAME)
    height = float(pose[2])
    if not math.isfinite(height):
        raise RuntimeError("高度读数非有限值：%r" % (height,))
    return height


# --------------------------------------------------------------------------
# 共享状态
# --------------------------------------------------------------------------


class TeleopState(object):
    def __init__(self, home: float, soft_min: float, soft_max: float) -> None:
        self.lock = threading.RLock()
        self.home = home
        self.current = home
        self.soft_min = soft_min
        self.soft_max = soft_max
        self.direction = 0          # +1 上 / -1 下 / 0 停
        self.keep_alive_until = 0.0  # monotonic
        self.last_key_at = -1e9
        self.goto: Optional[float] = None
        self.estop = False
        self.stop = False
        self.error: Optional[str] = None
        self.steps = 0
        self.sdk_faults_tolerated = 0

    def snapshot(self):
        with self.lock:
            return {
                "current": self.current,
                "home": self.home,
                "direction": self.direction,
                "goto": self.goto,
                "estop": self.estop,
                "error": self.error,
                "steps": self.steps,
                "soft": (self.soft_min, self.soft_max),
            }


def _fail(st: TeleopState, message: str) -> None:
    with st.lock:
        st.error = message
        st.direction = 0
        st.goto = None
        st.keep_alive_until = 0.0
    say("[停止] " + message)


# --------------------------------------------------------------------------
# 运动线程
# --------------------------------------------------------------------------


def _move_one_step(robot, target: float, duration_s: float, timeout_s: float, st: TeleopState):
    """下发一步并回读。返回 (sdk_status_name, actual_height)。"""
    status = robot.set_leg_height(target, duration_s, True, timeout_s)
    name = _enum_name(status)
    try:
        actual = read_height(robot)
    except Exception as exc:  # 读不回来不代表没动，交给上层判 TIMEOUT
        say("[警告] 动作后读高度失败：%s: %s" % (type(exc).__name__, exc))
        actual = float("nan")
    return name, actual


def motion_loop(robot, st: TeleopState, *, step_m: float, duration_s: float,
                timeout_s: float) -> None:
    stall = 0
    while True:
        with st.lock:
            if st.stop or st.error is not None:
                return
            estop = st.estop
            goto = st.goto
            now = time.monotonic()
            direction = 0 if estop else st.direction
            if direction and now >= st.keep_alive_until:
                direction = 0
                st.direction = 0
        if not direction and goto is None:
            time.sleep(IDLE_POLL_S)
            continue

        try:
            before = read_height(robot)
        except Exception as exc:
            _fail(st, "读高度失败：%s: %s" % (type(exc).__name__, exc))
            return
        with st.lock:
            st.current = before

        # 计算这一步的目标
        if goto is not None:
            remaining = goto - before
            if abs(remaining) <= POSE_TOL_M:
                with st.lock:
                    st.goto = None
                say("[到位] 已回到 %.3f m" % before)
                continue
            direction = 1 if remaining > 0 else -1
            step = min(step_m, abs(remaining))
        else:
            step = step_m

        target = before + direction * step
        target = max(st.soft_min, min(st.soft_max, target))
        if abs(target - before) < 1e-4:
            edge = st.soft_max if direction > 0 else st.soft_min
            with st.lock:
                st.direction = 0
                st.keep_alive_until = 0.0
                st.goto = None
            say("[限位] 已到软件限位 %.3f m，停止下发" % edge)
            continue

        name, actual = _move_one_step(robot, target, duration_s, timeout_s, st)

        # ---- 判定：以实测位置为准，不只看返回值 ----
        if not math.isfinite(actual):
            _fail(st, "SDK 返回 %s，且无法回读高度，位置不确定，请现场确认后重跑" % name)
            return

        with st.lock:
            st.current = actual
            st.steps += 1

        err = actual - target
        if name == "SUCCESS":
            pass
        elif name == "FAULT" and abs(err) <= POSE_TOL_M:
            # 已知坑：返回 FAULT 但实际已到位，视为成功继续
            with st.lock:
                st.sdk_faults_tolerated += 1
                tolerated = st.sdk_faults_tolerated
            if tolerated in (1, 10, 50):
                say("[提示] SDK 返回 FAULT 但实测已到位（已知坑，累计 %d 次），继续" % tolerated)
        elif name == "TIMEOUT":
            _fail(st, "set_leg_height 超时，位置不确定（实测 %.3f m / 目标 %.3f m）。"
                      "已停止，请现场确认稳定后重跑" % (actual, target))
            return
        else:
            _fail(st, "set_leg_height 返回 %s（实测 %.3f m / 目标 %.3f m）。已停止"
                  % (name, actual, target))
            return

        if abs(err) > DRIFT_ABORT_M:
            _fail(st, "实测高度偏离目标 %.3f m（> %.3f），疑似被打断或受阻。已停止"
                  % (abs(err), DRIFT_ABORT_M))
            return

        # 卡死检测：连续多步几乎没有位移
        if abs(actual - before) < step_m * 0.15:
            stall += 1
            if stall >= STALL_STEPS:
                _fail(st, "连续 %d 步几乎无位移（实测 %.3f m），疑似受阻/到机械极限。已停止"
                      % (stall, actual))
                return
        else:
            stall = 0


# --------------------------------------------------------------------------
# 键盘
# --------------------------------------------------------------------------


class RawKeys(object):
    """cbreak 单键读取；保留 ISIG，Ctrl-C 仍能中断。"""

    def __init__(self, fd: int) -> None:
        self.fd = fd
        self.old = None

    def __enter__(self):
        import termios
        import tty

        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_exc):
        import termios

        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        return False

    def poll(self, timeout_s: float) -> bytes:
        import select

        try:
            ready, _, _ = select.select([self.fd], [], [], timeout_s)
        except (OSError, ValueError):
            return b""
        if not ready:
            return b""
        try:
            return os.read(self.fd, 64)
        except OSError:
            return b""


def parse_keys(buf: bytes) -> List[str]:
    """把原始字节流拆成逻辑按键名。"""
    keys: List[str] = []
    i = 0
    n = len(buf)
    while i < n:
        if buf[i:i + 1] == b"\x1b":
            if buf[i:i + 3] == b"\x1b[A":
                keys.append("up")
                i += 3
                continue
            if buf[i:i + 3] == b"\x1b[B":
                keys.append("down")
                i += 3
                continue
            if buf[i:i + 3] == b"\x1bOA":
                keys.append("up")
                i += 3
                continue
            if buf[i:i + 3] == b"\x1bOB":
                keys.append("down")
                i += 3
                continue
            keys.append("esc")
            i += 1
            continue
        keys.append(buf[i:i + 1].decode("utf-8", "replace"))
        i += 1
    return keys


def _set_direction(st: TeleopState, direction: int, now: float,
                   hold_s: Optional[float] = None) -> None:
    with st.lock:
        if hold_s is not None:
            st.keep_alive_until = now + hold_s
        else:
            gap = now - st.last_key_at
            st.keep_alive_until = now + (FIRST_HOLD_S if gap > REPEAT_GAP_S else REPEAT_HOLD_S)
        st.last_key_at = now
        st.estop = False
        st.goto = None
        st.direction = direction


def _status_line(st: TeleopState) -> str:
    snap = st.snapshot()
    if snap["error"]:
        arrow = "ERR"
    elif snap["estop"]:
        arrow = "急停"
    elif snap["goto"] is not None:
        arrow = "回位"
    elif snap["direction"] > 0:
        arrow = "▲ 上升"
    elif snap["direction"] < 0:
        arrow = "▼ 下降"
    else:
        arrow = "— 停止"
    lo, hi = snap["soft"]
    return "腰高 %.3f m  %s   软限位[%.2f, %.2f]  起始 %.3f  步数 %d" % (
        snap["current"], arrow, lo, hi, snap["home"], snap["steps"],
    )


def _print_full_status(st: TeleopState) -> None:
    snap = st.snapshot()
    lo, hi = snap["soft"]
    say("腰高 %.4f m | 起始 %.4f m | 软限位 [%.2f, %.2f] | SDK 区间 [%.2f, %.2f]"
        % (snap["current"], snap["home"], lo, hi, SDK_HEIGHT_MIN, SDK_HEIGHT_MAX))
    say("方向 %+d | 已下发步数 %d | 急停 %s | 错误 %s"
        % (snap["direction"], snap["steps"], snap["estop"], snap["error"] or "无"))


HINT = (
    "按键：w/↑ 上升　s/↓ 下降　空格 急停　p 状态　r 回起始　q 退出\n"
    "（按住 w/s 会连续动；松手后最多再走一步）"
)


def _interactive_loop(st: TeleopState, kb: RawKeys) -> None:
    while True:
        if st.stop or st.error is not None:
            return
        buf = kb.poll(0.02)
        now = time.monotonic()
        for key in parse_keys(buf):
            if key in ("w", "W", "up"):
                _set_direction(st, +1, now)
            elif key in ("s", "S", "down"):
                _set_direction(st, -1, now)
            elif key == " ":
                with st.lock:
                    st.estop = True
                    st.direction = 0
                    st.goto = None
                    st.keep_alive_until = 0.0
                say("[急停] 已停止下发（按 w/s 恢复）")
            elif key in ("r", "R"):
                with st.lock:
                    if st.goto is None:
                        st.goto = st.home
                        st.estop = False
                        st.direction = 0
                say("[回位] 正在回到起始 %.3f m" % st.home)
            elif key in ("p", "P"):
                _print_full_status(st)
            elif key in ("q", "Q", "esc"):
                with st.lock:
                    st.stop = True
                return
            elif key in ("\x03",):
                with st.lock:
                    st.stop = True
                return
        line = _status_line(st)
        with _PRINT_LOCK:
            sys.stdout.write("\r\033[K" + line)
            sys.stdout.flush()


# --------------------------------------------------------------------------
# 无 tty 兜底：逐行命令
# --------------------------------------------------------------------------


def run_line_mode(robot, st: TeleopState, *, step_m: float, duration_s: float,
                  timeout_s: float) -> None:
    worker = threading.Thread(
        target=motion_loop, args=(robot, st),
        kwargs=dict(step_m=step_m, duration_s=duration_s,
                    timeout_s=timeout_s),
        daemon=True, name="leg-motion",
    )
    worker.start()
    say("逐行模式：每行输入一个字符命令（w=上一步 s=下一步 stop=急停 r=回原位 p=状态 q=退出）")
    try:
        while True:
            if st.stop or st.error is not None:
                break
            try:
                cmd = os.sys.stdin.readline()
            except KeyboardInterrupt:
                break
            if cmd == "":
                break
            cmd = cmd.strip()
            if not cmd:
                continue
            now = time.monotonic()
            if cmd in ("w", "W", "up"):
                _set_direction(st, +1, now, hold_s=duration_s * 0.9)
            elif cmd in ("s", "S", "down"):
                _set_direction(st, -1, now, hold_s=duration_s * 0.9)
            elif cmd in ("stop", " "):
                with st.lock:
                    st.estop = True
                    st.direction = 0
                    st.goto = None
                    st.keep_alive_until = 0.0
                say("[急停] 已停止下发")
            elif cmd in ("r", "R"):
                with st.lock:
                    st.goto = st.home
                    st.estop = False
                say("[回位] 正在回到起始 %.3f m" % st.home)
            elif cmd in ("p", "P", "status"):
                _print_full_status(st)
            elif cmd in ("q", "Q", "quit", "exit"):
                with st.lock:
                    st.stop = True
                break
            else:
                say("未知命令：%r（用 w / s / stop / r / p / q）" % cmd)
    finally:
        with st.lock:
            st.stop = True
        worker.join(timeout=3.0)


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="G1 腰部高度（直上直下）键盘遥控",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED_CM_S,
                    help="升降速度 cm/s（单步硬上限仍为 2 cm）")
    ap.add_argument("--duration", type=float, default=DEFAULT_STEP_S,
                    help="每步的运动时长 s；步长 = 速度 x 时长")
    ap.add_argument("--min", dest="soft_min", type=float, default=DEFAULT_SOFT_MIN,
                    help="软件下限 m（不会低于 SDK 下限）")
    ap.add_argument("--max", dest="soft_max", type=float, default=DEFAULT_SOFT_MAX,
                    help="软件上限 m（不会高于 SDK 上限）")
    ap.add_argument("--status", action="store_true", help="只读：打印当前高度后退出")
    ap.add_argument("--line", action="store_true", help="强制逐行输入模式")
    ap.add_argument("--yes", action="store_true", help="跳过启动确认")
    ap.add_argument("--no-restore", action="store_true", help="退出时不提示回原位")
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if not args.status and not args.line and not sys.stdin.isatty():
        say("当前 stdin 不是交互终端，自动切到逐行模式（--line）。")
        args.line = True

    soft_min = max(SDK_HEIGHT_MIN, float(args.soft_min))
    soft_max = min(SDK_HEIGHT_MAX, float(args.soft_max))
    if not (SDK_HEIGHT_MIN <= soft_min < soft_max <= SDK_HEIGHT_MAX):
        say("限位非法：需要 %.2f <= min < max <= %.2f，收到 [%.2f, %.2f]"
            % (SDK_HEIGHT_MIN, SDK_HEIGHT_MAX, soft_min, soft_max))
        return 2

    duration_s = max(0.10, float(args.duration))
    step_m = float(args.speed) / 100.0 * duration_s
    if step_m > MAX_STEP_M:
        say("速度 %.1f cm/s x %.2f s 超出单步上限，步长被压到 %.0f mm"
            % (float(args.speed), duration_s, MAX_STEP_M * 1000))
        step_m = MAX_STEP_M
    step_m = max(0.002, step_m)
    timeout_s = max(3.0, duration_s * 10)

    say("正在初始化 SDK（首次约需数秒）…")
    try:
        robot = open_robot()
    except Exception as exc:
        say("初始化失败：%s: %s" % (type(exc).__name__, exc))
        return 1

    try:
        home = read_height(robot)
    except Exception as exc:
        say("读取当前高度失败：%s: %s" % (type(exc).__name__, exc))
        return 1

    say("当前腰高 %.4f m（链路 base_link <- %s）" % (home, TRANSFORM_FRAME))

    if not (SDK_HEIGHT_MIN - 0.02 <= home <= SDK_HEIGHT_MAX + 0.02):
        say("当前高度 %.4f 超出 SDK 支持区间 [%.2f, %.2f]，拒绝动作"
            % (home, SDK_HEIGHT_MIN, SDK_HEIGHT_MAX))
        return 1

    if args.status:
        say("软限位 [%.2f, %.2f]；速度 %.1f cm/s，单步 %.1f mm，单步时长 %.2f s"
            % (soft_min, soft_max, step_m / duration_s * 100.0,
               step_m * 1000.0, duration_s))
        return 0

    say("计划：速度 %.1f cm/s，单步 %.1f mm，单步时长 %.2f s，软限位 [%.2f, %.2f] m"
        % (step_m / duration_s * 100.0, step_m * 1000.0, duration_s,
           soft_min, soft_max))
    say("安全提醒：确认周围 2 m 空旷、急停在手边、XCU 处于正常模式（ABNORMAL 会拒动）。")
    if not args.yes:
        try:
            answer = input("确认开始键盘升降控制？(输入 y 继续)：").strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = ""
        if answer != "y":
            say("已取消，未下发任何运动命令。")
            return 0

    st = TeleopState(home=home, soft_min=soft_min, soft_max=soft_max)

    try:
        if args.line:
            run_line_mode(robot, st, step_m=step_m, duration_s=duration_s,
                          timeout_s=timeout_s)
        else:
            worker = threading.Thread(
                target=motion_loop, args=(robot, st),
                kwargs=dict(step_m=step_m, duration_s=duration_s,
                            timeout_s=timeout_s),
                daemon=True, name="leg-motion",
            )
            worker.start()
            say(HINT)
            try:
                with RawKeys(sys.stdin.fileno()) as kb:
                    _interactive_loop(st, kb)
            finally:
                with st.lock:
                    st.stop = True
                worker.join(timeout=5.0)
    except KeyboardInterrupt:
        say("收到 Ctrl-C，已停止下发。")
        with st.lock:
            st.stop = True
    except Exception as exc:
        say("运行异常：%s: %s" % (type(exc).__name__, exc))
        with st.lock:
            st.stop = True
        return 1
    finally:
        with _PRINT_LOCK:
            sys.stdout.write("\n")
            sys.stdout.flush()

    snap = st.snapshot()
    say("结束。最终腰高 %.4f m，共下发 %d 步。" % (snap["current"], snap["steps"]))
    if snap["error"]:
        say("本次运行出现过错误：%s" % snap["error"])

    # ---- 退出前回原位 ----
    if args.no_restore:
        return 0
    try:
        cur = read_height(robot)
    except Exception:
        return 0
    if abs(cur - home) <= 0.02:
        return 0
    try:
        answer = input("当前 %.3f m 偏离起始 %.3f m，回原位？(Enter/y 回，k 保持)："
                       % (cur, home)).strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = "k"
    if answer == "k":
        say("保持当前位置 %.4f m。" % cur)
        return 0

    restore = TeleopState(home=home, soft_min=soft_min, soft_max=soft_max)
    restore.current = cur
    with restore.lock:
        restore.goto = home
    worker = threading.Thread(
        target=motion_loop, args=(robot, restore),
        kwargs=dict(step_m=step_m, duration_s=duration_s,
                    timeout_s=timeout_s),
        daemon=True, name="leg-restore",
    )
    worker.start()
    deadline = time.monotonic() + 120.0
    while time.monotonic() < deadline:
        with restore.lock:
            done = restore.goto is None or restore.error is not None
        if done:
            break
        time.sleep(0.1)
    with restore.lock:
        restore.stop = True
    worker.join(timeout=5.0)
    if restore.error:
        say("[回位失败] %s" % restore.error)
    try:
        final = read_height(robot)
        say("回原位结束，当前 %.4f m（目标 %.4f m）。" % (final, home))
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
