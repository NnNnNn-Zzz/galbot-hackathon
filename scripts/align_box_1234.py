#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""【腰高 1.234 m 专用】绿块视觉对准 —— 标准位置写死，不受其它高度档案影响。

对应参考图：211d89c3...jpg（1280x720）
    中心 (686.34, 481.38)   倾角 +0.78°（相对图像竖直）
    长边 205.91 px / 短边 180.70 px   面积占比 3.85%

约束与通用版一致：只动手臂关节、夹爪指向不变、末端高度不变。

用法
----
    source ~/galbot_hackathon/scripts/env_hpu.sh
    python3 scripts/align_box_1234.py --dry-run     # 只看差距
    python3 scripts/align_box_1234.py               # 闭环对齐

腰高不是 1.234±0.02 m 时会直接拒绝（标准位置和腰高是一一对应的）。
确知风险要强行跑，加 --force。
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from align_green_box import main as _main  # noqa: E402

#: 本程序锁定的标准档案
PROFILE = "1234"
WAIST_M = 1.2337
REF_CENTER = (686.34, 481.38)
REF_TILT_DEG = 0.78


if __name__ == "__main__":
    sys.exit(_main(["--profile", PROFILE] + sys.argv[1:]))
