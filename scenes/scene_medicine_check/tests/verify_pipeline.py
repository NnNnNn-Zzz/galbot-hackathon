"""离线验证：在不接硬件的 mock 后端上跑通整条链路，并覆盖各类异常分支。

跑法（项目根目录）：
    python3 scenes/scene_medicine_check/tests/verify_pipeline.py
不需要任何第三方依赖，不需要 pytest。

覆盖的场景：
  1. 正常流程（7 步全 ok，抓到药盒，归位成功）
  2. 抓取失败（夹爪动不了）→ failed，仍执行归位
  3. 左移超时 → failed + TIMEOUT，仍执行归位
  4. 起始里程计读不到 → 非致命，流程照常走完
  5. 逼近抓取时状态不可判定 → unknown，立即全停，**不执行归位**
  6. stage 抛异常 → runner 兜住，failed，仍执行归位
  7. 升级版（过期判定）→ 选中并抓起真正过期的那盒
  8. 升级版但桌面上没有过期药 → failed + NO_TARGET
  9. 升级版但日期全读不出 → unknown（不瞎猜成"没过期"）
"""

from __future__ import annotations

import json
import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from scenes.scene_medicine_check import baseline_flow as bf  # noqa: E402

logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

RESULTS = []


def check(no: int, name: str, cond: bool, detail: str = "") -> None:
    RESULTS.append((no, name, bool(cond), detail))
    mark = "PASS" if cond else "FAIL"
    print(f"[{mark}] {no}. {name}" + (f"  —— {detail}" if detail else ""))


def step_of(res: dict, step_id: str) -> dict:
    for t in (res.get("details") or {}).get("trace", []):
        if t.get("step") == step_id:
            return t
    return {}


def run(params: dict) -> dict:
    p = dict(params)
    p.setdefault("io", "mock")
    p.setdefault("pause_s", 0)
    return bf.run(p)


def main() -> int:
    # ---------------------------------------------------------------- 1 正常
    r = run({})
    d = r.get("details") or {}
    check(1, "正常流程 7 步全 ok，抓到并投出，归位成功",
          r.get("success") is True
          and d.get("steps_ok") == 7
          and d.get("picked") is True
          and d.get("home_arm_ok") is True and d.get("home_base_ok") is True,
          f"success={r.get('success')} steps_ok={d.get('steps_ok')} picked={d.get('picked')} "
          f"feedback={r.get('feedback')}")
    print("     播报文本：", d.get("announce_text"))

    # ---------------------------------------------------------------- 2 抓取失败
    r = run({"mock_faults": [{"method": "set_gripper", "behavior": "fail_once",
                              "message": "夹爪电机堵转"}]})
    home = step_of(r, "home")
    check(2, "抓取失败 → failed，且仍执行归位",
          r.get("success") is False
          and (r.get("details") or {}).get("trace")
          and step_of(r, "grasp").get("status") == "failed"
          and home.get("status") != "skipped",
          f"failed_step={(r.get('details') or {}).get('trace') and step_of(r, 'grasp').get('message')} "
          f"home={home.get('status')}")

    # ---------------------------------------------------------------- 3 超时
    r = run({"mock_faults": [{"method": "move_left", "behavior": "timeout", "delay_s": 0.1}]})
    shf = step_of(r, "shift_left")
    home = step_of(r, "home")
    check(3, "左移超时 → failed + TIMEOUT，且仍执行归位",
          r.get("success") is False and shf.get("code") == "TIMEOUT" and home.get("status") != "skipped",
          f"shift_left.status={shf.get('status')} code={shf.get('code')} home={home.get('status')}")

    # ---------------------------------------------------------------- 4 非致命
    r = run({"mock_faults": [{"method": "get_odom", "behavior": "unknown",
                              "message": "里程计无回执"}]})
    pre = step_of(r, "preflight")
    check(4, "起始里程计读不到 → 非致命，流程照常完成",
          r.get("success") is True and pre.get("status") == "failed"
          and step_of(r, "home").get("status") != "skipped",
          f"preflight={pre.get('status')} success={r.get('success')}")

    # ---------------------------------------------------------------- 5 unknown
    r = run({"mock_faults": [{"method": "move_line_to", "behavior": "unknown",
                              "message": "视觉与位姿读数矛盾"}]})
    d = r.get("details") or {}
    home = step_of(r, "home")
    check(5, "抓取逼近时不可判定 → unknown 立即全停，不执行归位",
          r.get("success") is False and d.get("stopped_without_safety") is True
          and home.get("status") == "skipped",
          f"home={home.get('status')} feedback={r.get('feedback')}")

    # ---------------------------------------------------------------- 6 抛异常
    # turn 接口现在只被 turn_bin（右转 30°）使用，异常注入打在这里
    r = run({"mock_faults": [{"method": "turn", "behavior": "raise", "message": "驱动掉线"}]})
    home = step_of(r, "home")
    check(6, "stage 抛异常 → runner 兜住，failed，且仍执行归位",
          r.get("success") is False and step_of(r, "turn_bin").get("status") == "failed"
          and home.get("status") != "skipped",
          f"turn_bin={step_of(r, 'turn_bin').get('message')} home={home.get('status')}")

    # ---------------------------------------------------------------- 7 升级版
    r = run({"pipeline": "medicine.expiry_demo"})
    d = r.get("details") or {}
    check(7, "升级版：选中并抓起真正过期的那盒（最左 box_left, 2026-08-01）",
          r.get("success") is True and d.get("target") == "box_left" and d.get("picked") is True,
          f"success={r.get('success')} target={d.get('target')} verdict={d.get('verdict')}")

    # ---------------------------------------------------------------- 8 无过期药
    no_expired = [
        {"id": "a", "label": "white", "ee_pose": [0.40, 0.30, 0.65], "expiry_date": "2099-01-01"},
        {"id": "b", "label": "white", "ee_pose": [0.40, 0.10, 0.65], "expiry_date": "2099-02-01"},
    ]
    r = run({"pipeline": "medicine.expiry_demo", "objects": no_expired})
    check(8, "升级版：桌面没有过期药 → failed + NO_TARGET",
          r.get("success") is False and step_of(r, "grasp").get("code") == "NO_TARGET",
          f"msg={step_of(r, 'grasp').get('message')}")

    # ---------------------------------------------------------------- 9 读不出日期
    unreadable = [
        {"id": "a", "label": "white", "ee_pose": [0.40, 0.30, 0.65], "expiry_date": ""},
        {"id": "b", "label": "white", "ee_pose": [0.40, 0.10, 0.65], "expiry_date": ""},
    ]
    r = run({"pipeline": "medicine.expiry_demo", "objects": unreadable})
    d = r.get("details") or {}
    check(9, "升级版：日期全读不出 → unknown，不瞎猜成「没过期」",
          r.get("success") is False and step_of(r, "grasp").get("status") == "unknown"
          and d.get("stopped_without_safety") is True,
          f"msg={step_of(r, 'grasp').get('message')}")

    # ---------------------------------------------------------------- 汇总
    bad = [x for x in RESULTS if not x[2]]
    print("\n" + "=" * 72)
    print(f"合计 {len(RESULTS)} 项，通过 {len(RESULTS) - len(bad)} 项，失败 {len(bad)} 项")
    if bad:
        print("失败项：", [(x[0], x[1]) for x in bad])
    print("=" * 72)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
