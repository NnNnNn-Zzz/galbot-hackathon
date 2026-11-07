#!/usr/bin/env python3
"""dev 层能力分项自检 —— 真机上一次只验一项，出事好定位。

用法（机器人上，项目根目录）：

    source scripts/env_hpu.sh

    python3 scripts/selftest_dev_layer.py status    # 只看，不动（默认子命令）
    python3 scripts/selftest_dev_layer.py base      # 底盘：前进10cm → 后退10cm → 左转15° → 右转15°
    python3 scripts/selftest_dev_layer.py gripper   # 夹爪：张开 → 合拢 → 张开 → 合拢（左臂）
    python3 scripts/selftest_dev_layer.py arm       # 手臂：读位姿 → +2cm → 回原位
    python3 scripts/selftest_dev_layer.py camera    # 右腕相机：抓一帧存 /tmp
    python3 scripts/selftest_dev_layer.py table     # 桌面检测（需 table_calib.json 标定）
    python3 scripts/selftest_dev_layer.py all       # 依次跑上面全部（约 40s，机器人会动！）

加 --mock 可在没有机器人的地方干跑（校验代码链路，不碰硬件）：

    python3 scripts/selftest_dev_layer.py all --mock

⚠️ 真机跑之前：周围 2m 空旷、急停在手边、XCU 里 WORKING_MODE 是正常态。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sdk_wrapper import wrapper          # noqa: E402
from sdk_wrapper.dev import toolhub      # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def step(name: str, fn, *, want_success: bool = True, brief: str = ""):
    """执行一步并记录结果。异常不中断后续步骤（真机排障要一次看全）。"""
    t0 = time.time()
    try:
        r = fn()
        ok = isinstance(r, dict) and (r.get("success") is True) == want_success
        detail = brief or _brief(r)
        RESULTS.append((name, ok, detail))
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {name:<32s} {time.time() - t0:5.2f}s  {detail}")
        return r
    except Exception as exc:  # noqa: BLE001
        RESULTS.append((name, False, f"异常：{exc}"))
        print(f"  [FAIL] {name:<32s} 抛异常：{exc}")
        traceback.print_exc(limit=2)
        return None


def _brief(r) -> str:
    if not isinstance(r, dict):
        return str(r)[:120]
    keys = ("message", "control_status", "actual_distance_cm", "actual_angle_deg",
            "actual_width_cm", "image_path", "status", "error_code")
    picked = {k: r[k] for k in keys if k in r}
    return str(picked)[:160]


# ────────────────────────────────────────────────────────────────────────── #
def cmd_status():
    print("\n[0] 环境与适配器")
    info = toolhub.describe()
    print(f"  模式：{info['mode']}   适配器：{info['adapter']}")
    step("list_cameras()", wrapper.list_cameras, brief="看相机清单")
    step("get_ee_pose(right_arm)", lambda: wrapper.get_ee_pose("right_arm"))


def cmd_base():
    print("\n[1] 底盘直控（set_base_pose，不需要地图）")
    print("    ⚠️ 机器人会移动 ±10cm / 转 ±15°，请确认周围空旷")
    step("move_forward(10cm)", lambda: wrapper.move_forward(10))
    step("move_backward(10cm)", lambda: wrapper.move_backward(10))
    step("turn(+15°) 左转", lambda: wrapper.turn(15))
    step("turn(-15°) 右转回正", lambda: wrapper.turn(-15))
    step("stop_base()", wrapper.stop_base)


def cmd_gripper():
    print("\n[2] 夹爪（左臂，开合两次）")
    for i in (1, 2):
        step(f"open_gripper 第{i}次", lambda: wrapper.open_gripper("left_gripper"))
        step(f"close_gripper 第{i}次", lambda: wrapper.close_gripper("left_gripper"))
    step("open_gripper 收尾（松开）", lambda: wrapper.open_gripper("left_gripper"))


def cmd_arm():
    print("\n[3] 手臂末端（读位姿 → +2cm → 回原位）")
    print("    ⚠️ 手臂会平移 2cm，注意别碰周围")
    p = step("get_ee_pose(right_arm)", lambda: wrapper.get_ee_pose("right_arm"))
    if not p or not p.get("success"):
        print("  → 读不到位姿，跳过平移（先解决位姿读取）")
        return
    pose = p["pose"]
    print(f"    当前位姿：[{', '.join(f'{v:.3f}' for v in pose)}]")
    step("move_arm_delta(+2cm x)", lambda: wrapper.move_arm_delta(0.02, 0.0, 0.0))
    step("move_arm_to(回原位姿)", lambda: wrapper.move_arm_to(pose))


def cmd_camera():
    print("\n[4] 右腕相机（抓一帧）")
    r = step("capture_right_wrist()", lambda: wrapper.capture_right_wrist("/tmp/selftest_rgb.jpg"))
    if r and r.get("success"):
        print(f"    图片：{r.get('image_path')}  {r.get('width')}x{r.get('height')}")
        same = r.get("depth_color_same_res")
        if same is False:
            print("    ⚠️ 深度与彩色分辨率不一致 → 同一像素取深度会错位，需先对齐")
        if r.get("depth_m") is None:
            print("    ⚠️ 没取到深度，抓取反投影会失败")


def cmd_table():
    print("\n[5] 桌面检测（依赖 sdk_wrapper/dev/config/table_calib.json）")
    step("inspect_table()", wrapper.inspect_table)


def cmd_all():
    cmd_status()
    cmd_base()
    cmd_gripper()
    cmd_arm()
    cmd_camera()
    cmd_table()


COMMANDS = {
    "status": cmd_status,
    "base": cmd_base,
    "gripper": cmd_gripper,
    "arm": cmd_arm,
    "camera": cmd_camera,
    "table": cmd_table,
    "all": cmd_all,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="dev 层能力分项自检")
    parser.add_argument("command", nargs="?", default="status", choices=sorted(COMMANDS))
    parser.add_argument("--mock", action="store_true", help="Mock 模式（不碰硬件）")
    args = parser.parse_args()

    if args.mock:
        os.environ["GALBOT_MOCK"] = "1"
        wrapper.set_mock_mode(True)
    info = toolhub.describe()
    print("=" * 74)
    print(f"dev 层自检  |  模式={info['mode']}  适配器={info['adapter']}  子命令={args.command}")
    print("=" * 74)
    if info["mode"] == "real" and args.command in ("base", "gripper", "arm", "all"):
        print("\n⚠️  真机模式：下面这些步骤会让机器人真实动作。")
        print("    确认周围 2m 空旷、急停在手边。3 秒后开始……\n")
        time.sleep(3)

    COMMANDS[args.command]()

    total = len(RESULTS)
    passed = sum(1 for _n, ok, _d in RESULTS if ok)
    print("\n" + "=" * 74)
    print(f"汇总：{passed}/{total} 通过")
    failed = [n for n, ok, _d in RESULTS if not ok]
    if failed:
        print("失败项：")
        for name in failed:
            print(f"  - {name}")
    print("=" * 74)
    wrapper.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
