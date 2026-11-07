#!/usr/bin/env python3
"""探针：连读两次右臂末端位姿，判断机器人是否正被其他控制源操作。

两次读数差异 < 1mm 视为静止（可发实验）；差异大说明有人/有程序在动臂，
按协作纪律应停止发指令并报告。
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sdk_wrapper import wrapper  # noqa: E402


def main() -> int:
    p1 = wrapper.get_ee_pose("right_arm")
    print(json.dumps({"t1_ok": p1.get("success"), "t1": p1.get("pose")}), flush=True)
    time.sleep(3.0)
    p2 = wrapper.get_ee_pose("right_arm")
    print(json.dumps({"t2_ok": p2.get("success"), "t2": p2.get("pose")}), flush=True)
    if p1.get("success") and p2.get("success"):
        a, b = p1["pose"], p2["pose"]
        drift = max(abs(a[i] - b[i]) for i in range(3))
        still = drift < 0.001
        print(json.dumps({"drift_m": round(drift, 6),
                          "arm_still": still,
                          "verdict": "静止，可发实验" if still else "臂在动！疑似有其他控制源，停手"},
                         ensure_ascii=False), flush=True)
        return 0 if still else 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
