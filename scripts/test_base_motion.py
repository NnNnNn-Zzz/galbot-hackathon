#!/usr/bin/env python3
"""底盘直连冒烟测试：绕过 LLM，直接验证 wrapper → SDK → 机器人 链路。

用途：真机验证的【第一步】——先证明不动大脑、只动腿是通的，
再去调 agent 决策链路。出问题时分层定位：本脚本挂 = SDK/连接问题；
本脚本过、agent 挂 = LLM/提示词问题。

用法：
    # 1. mock 模式（任何时候都安全，先跑这个）
    python3 scripts/test_base_motion.py --mock

    # 2. 真机模式（确认机器人周围 2m 空旷、急停在手边）
    source scripts/env.sh
    python3 scripts/test_base_motion.py --distance 10   # 前进 10cm 再退回来

    # 自定义动作
    python3 scripts/test_base_motion.py --distance 10 --turn 0      # 只前进
    python3 scripts/test_base_motion.py --distance 0 --turn 30      # 只左转 30°
"""

from __future__ import annotations
import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("test_base_motion")

from sdk_wrapper import wrapper  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="G1 底盘直连冒烟测试")
    parser.add_argument("--mock", action="store_true", help="mock 模式（不碰真机）")
    parser.add_argument("--distance", type=float, default=10.0,
                        help="前进距离（厘米），会自动退回原位；0=不动")
    parser.add_argument("--turn", type=float, default=0.0,
                        help="附加原地转向（度），正值左转；0=不转")
    args = parser.parse_args()

    if args.mock or os.environ.get("GALBOT_MOCK") == "1":
        wrapper.set_mock_mode(True)
        logger.info("=== MOCK 模式 ===")
    else:
        print("=" * 60)
        print("!! 真机模式：机器人即将真实移动 !!")
        print(f"   计划动作：前进 {args.distance}cm 再退回，转向 {args.turn}°")
        print("   请确认：周围空旷、急停按钮在手边")
        print("=" * 60)
        input("确认无误后按 Enter 开始，Ctrl+C 取消...")

    try:
        if args.distance > 0:
            r = wrapper.move_forward(args.distance)
            logger.info("前进结果: %s", r)
            if not r["success"]:
                logger.error("前进失败，中止测试"); return 1

        if args.turn != 0:
            r = wrapper.turn(args.turn)
            logger.info("转向结果: %s", r)
            if not r["success"]:
                logger.error("转向失败，中止测试（前进部分已完成）")
                return 1

        if args.distance > 0:
            r = wrapper.move_backward(args.distance)
            logger.info("退回结果: %s", r)
            if not r["success"]:
                logger.warning("退回失败，机器人不在原位，请人工确认")

        logger.info("=== 冒烟测试通过：wrapper → SDK → 机器人 链路正常 ===")
        return 0
    except KeyboardInterrupt:
        logger.warning("用户中断，执行急停归位")
        return 130
    finally:
        wrapper.stop_base()
        wrapper.shutdown()


if __name__ == "__main__":
    sys.exit(main())
