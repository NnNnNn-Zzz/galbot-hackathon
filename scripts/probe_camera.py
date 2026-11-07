#!/usr/bin/env python3
"""相机诊断探针 —— 只读，不动臂、不走底盘、不抓取。

它要回答的问题
--------------
我们的 capture_image 拿不到图（get_rgb_data 返回空）。可能是两个原因：

  假设 A：init() 没传 enable_sensor_set，默认集合里没有 HEAD_LEFT_CAMERA
  假设 B：init() 之后没等够时间，相机还没出第一帧

本脚本用四个阶段把这两个假设分开，每个阶段单独判定，不做任何推断。

【重要】SDK 的 destroy() 之后同进程无法再 init()，
所以本脚本按 SDK 的会话模型组织：**一次只 init 一次**，
用 `--phase` 参数让你分多次运行，或者接受单进程只测一个组合。

推荐用法（HPU 上，source scripts/env_hpu.sh 之后）：

    # 第一轮：模拟现状（不传集合，不预热）——预期拿不到图
    python3 scripts/probe_camera.py --mode current

    # 第二轮：只加预热（不传集合，等 5 秒）——用来区分假设 A/B
    python3 scripts/probe_camera.py --mode warmup

    # 第三轮：显式订阅（传集合，等 5 秒）——官方写法
    python3 scripts/probe_camera.py --mode explicit

三轮结果对照表：

    current   warmup   explicit   |  结论
    -------------------------------------------
    空        空       有图       |  假设 A 成立：必须显式订阅
    空        有图     有图       |  假设 B 成立：只需要加 sleep
    有图      有图     有图       |  两个都不是，问题在别处（见下文）
    空        空       空         |  相机硬件/驱动问题，别改代码

    「有图」= 拿到 data 且字节数 > 0

退出码：0 = 本模式拿得到图；1 = 拿不到；2 = init 失败（环境问题）
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _say(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


def _probe_once(mode: str, warmup_s: float, camera_name: str) -> int:
    """跑一个模式：init 一次，取图一次，destroy，然后退出。"""
    try:
        from sdk_wrapper.wrapper import _sdk_bootstrap
        _sdk_bootstrap()
        import galbot_sdk.g1 as gm
    except Exception as exc:  # noqa: BLE001
        _say({"phase": mode, "fatal": f"导入 SDK 失败：{exc}",
              "action": "先 source scripts/env_hpu.sh，并确认 galbot_sdk 在 PYTHONPATH"})
        return 2

    cam = getattr(gm.SensorType, camera_name, None)
    if cam is None:
        _say({"phase": mode, "fatal": f"SensorType 里没有 {camera_name}"})
        return 2

    robot = gm.GalbotRobot()

    if mode == "current":
        _say({"phase": mode, "init_args": "无参数（复现现状）"})
        ok = robot.init()
    else:
        # warmup 与 explicit 都用同样的集合，区别只在是否真的验证订阅
        sensors = {cam}
        _say({"phase": mode, "init_args": f"enable_sensor_set={{{camera_name}}}"})
        ok = robot.init(sensors)

    _say({"phase": mode, "init_ok": bool(ok)})
    if not ok:
        _say({"phase": mode, "fatal": "init 返回 False",
              "action": "先解决初始化问题（SDK/网络/embosa），相机是次要的"})
        try:
            robot.destroy()
        except Exception:  # noqa: BLE001
            pass
        return 2

    if warmup_s > 0:
        _say({"phase": mode, "warmup_s": warmup_s,
              "note": "官方 get_camera_data.py 固定 sleep(5)"})
        time.sleep(warmup_s)

    got = False
    size = 0
    # 取三次，每次间隔 1 秒，看是否只是首帧没出
    for attempt in range(1, 4):
        try:
            frame = robot.get_rgb_data(cam, gm.RgbOutputFormat.JPEG, True)
        except Exception as exc:  # noqa: BLE001
            _say({"phase": mode, "attempt": attempt, "exception": str(exc)[:200]})
            frame = None
        if frame and "data" in frame:
            size = len(frame["data"])
            header = frame.get("header", "")
            got = True
            _say({"phase": mode, "attempt": attempt, "result": "有图",
                  "bytes": size, "header": str(header)[:120]})
            break
        _say({"phase": mode, "attempt": attempt, "result": "空",
              "frame_is_none": frame is None,
              "keys": list(frame.keys()) if isinstance(frame, dict) else None})
        time.sleep(1.0)

    try:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
    except Exception as exc:  # noqa: BLE001
        _say({"phase": mode, "cleanup_warning": str(exc)[:200]})

    _say({"phase": mode, "verdict": "有图" if got else "空", "bytes": size})
    return 0 if got else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="相机诊断探针（只读）")
    ap.add_argument("--mode", choices=["current", "warmup", "explicit"],
                    default="current",
                    help="current=复现现状；warmup=加睡眠；explicit=显式订阅+睡眠")
    ap.add_argument("--warmup", type=float, default=5.0,
                    help="预热秒数（current 模式强制为 0）")
    ap.add_argument("--camera", default="HEAD_LEFT_CAMERA",
                    help="要测的相机，默认 HEAD_LEFT_CAMERA")
    args = ap.parse_args()

    warmup = 0.0 if args.mode == "current" else args.warmup
    _say({"probe": "camera", "mode": args.mode, "camera": args.camera,
          "warmup_s": warmup,
          "safety": "本脚本只调 init / get_rgb_data / destroy，"
                    "不发任何运动指令，不动臂、不走底盘"})

    return _probe_once(args.mode, warmup, args.camera)


if __name__ == "__main__":
    raise SystemExit(main())
