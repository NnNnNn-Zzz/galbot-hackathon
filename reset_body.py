"""直接控制机器人回到基准身体状态（独立工具脚本，不改封装层）。

基线状态定义 = 底盘停止 + 手臂折叠安全位 + 夹爪释放。
当前实现（安全可做）：
    1. 停底盘            wrapper.stop_base()
    2. 双夹爪释放        wrapper.open_gripper(left/right)
      —— 张开到 ~8.5cm，dev 层 D5 真机验证过的安全开度
跳过：
    3. 手臂折叠位        FOLD_POSE 尚未由负责人 A 示教填入 baseline.py
      —— 乱给手臂目标位姿有撞自己 / 奇异点风险，故不动手。

用法：
    GALBOT_MOCK=1 uv run python reset_body.py     # mock 验证调用链
    uv run python reset_body.py                    # 真机（需 G1 上电、controller 正常）

退出码 0 = 全部成功；非 0 = 有步骤失败（见日志）。
"""

from __future__ import annotations

import logging
import os
import sys

# 保证项目根目录在 sys.path（与 agent/main.py 同款处理）
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from sdk_wrapper import wrapper  # noqa: E402
from sdk_wrapper.dev import toolhub  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
logger = logging.getLogger("reset_body")


def reset_body() -> dict:
    """停底盘 + 双夹爪释放。返回各步结果。"""
    # 建立适配器（mock 跟随 GALBOT_MOCK 环境变量；真机自动 init+switch_controller）
    cfg = toolhub.configure()
    mode = cfg.get("mode", "unknown")
    logger.info("运行模式: %s (adapter=%s)", mode, cfg.get("adapter"))

    results: dict = {}

    # 1. 底盘停止 —— baseline.reset_to_baseline() 当前唯一已实现项
    try:
        results["stop_base"] = wrapper.stop_base()
    except Exception as exc:  # noqa: BLE001 - 单步失败不阻断后续
        results["stop_base"] = {"success": False, "error": str(exc)}
    logger.info("stop_base -> %s", results["stop_base"])

    # 2. 双夹爪释放（张开到安全最大宽度，dev 层固定 0.05 m/s）
    for ee in ("left_gripper", "right_gripper"):
        key = f"open_{ee}"
        try:
            results[key] = wrapper.open_gripper(ee)
        except Exception as exc:  # noqa: BLE001
            results[key] = {"success": False, "error": str(exc)}
        logger.info("%s -> %s", key, results[key])

    # 3. 手臂折叠位 —— 跳过（FOLD_POSE 待负责人 A 示教填入 baseline.py）
    results["fold_arm"] = {
        "success": False,
        "skipped": True,
        "reason": "FOLD_POSE 未定义，跳过——需负责人 A 示教折叠位姿后填入 baseline.py",
    }
    logger.warning("手臂折叠位未实现，跳过（待负责人 A 示教 FOLD_POSE）")

    return results


def _all_ok(results: dict) -> bool:
    """stop_base + 双夹爪三步都 success 即视为归位成功（手臂跳过不算失败）。"""
    must = ("stop_base", "open_left_gripper", "open_right_gripper")
    return all(
        bool(results.get(k, {}).get("success"))
        for k in must
    )


if __name__ == "__main__":
    out = reset_body()
    print("\n=== 归位结果 ===")
    for k, v in out.items():
        print(f"  {k}: {v}")
    ok = _all_ok(out)
    print(f"\n归位（停底盘+双夹爪释放）: {'成功' if ok else '失败'}")
    print("注意：手臂折叠位未实现，整体身体归位尚未完成——待负责人 A 示教 FOLD_POSE。")
    sys.exit(0 if ok else 1)
