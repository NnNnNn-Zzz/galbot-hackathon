"""基线版入口（table 模式）：只做三件事 —— 读配置、跑管线、拼返回。

真正的拆分在 pipeline/ 目录：
    pipeline/configs/*.json   决定跑哪些步骤、以什么顺序
    pipeline/stages/          内置基线动作块（每块一个文件）
    pipeline/policies/        队友的替换实现（自动发现，不用改任何 __init__）
    pipeline/runner.py        唯一的控制流（ok / failed / unknown 三种终态）
    pipeline/io.py            硬件访问层（wrapper / mock 两个后端，只在这里切换）
    pipeline/tuning.py        现场标定常量（原文件顶部的"需要补充的数据"）

对外契约不变：run(params) -> dict，字段仍是 success / feedback / suggested_adjustment / details。
scene.py 的 table 分支照旧 `baseline_flow.run(params)`，零改动。

params（全部可选）：
    pipeline        配置名，默认 medicine.baseline
    io              后端 "wrapper" | "mock"，默认取环境变量 MED_IO，再默认 wrapper
    item_name       播报用药品名
    pause_s         播报后停顿秒数
    shift_m         左移距离，米（覆盖配置里的 distance_m；侧移不转身）
    bin_turn_deg    到桌前转向角度（负=右转）
    objects         mock 后端的桌面物体（不传则用内置三盒默认表）
    dry_run         True 时只打印编排，不动硬件
"""

import json
import logging
import os
import sys

if __package__ is None:
    # 允许直接 `python3 scenes/scene_medicine_check/baseline_flow.py` 跑 dry_run：
    # 脚本直接执行时 sys.path 不含项目根，scenes/sdk_wrapper 都会 import 失败。
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from sdk_wrapper import baseline                                    # noqa: E402
from scenes.scene_medicine_check.pipeline import runner as pipeline_runner   # noqa: E402
from scenes.scene_medicine_check.pipeline import tuning              # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context   # noqa: E402
from scenes.scene_medicine_check.pipeline.io import build_io         # noqa: E402

logger = logging.getLogger("scene.medicine.baseline")

DEFAULT_PIPELINE = "medicine.baseline"

# ===========================================================================
# 向后兼容：常量搬到 pipeline/tuning.py 了，这里保持同名可 import
# ===========================================================================
LEG_JOINT_NAMES = tuning.LEG_JOINT_NAMES
LEG_JOINT_POSITIONS = tuning.LEG_JOINT_POSITIONS
UPPER_JOINT_NAMES = tuning.UPPER_JOINT_NAMES
UPPER_JOINT_POSITIONS = tuning.UPPER_JOINT_POSITIONS
CARRY_JOINT_NAMES = tuning.CARRY_JOINT_NAMES
CARRY_JOINT_POSITIONS = tuning.CARRY_JOINT_POSITIONS
PRE_GRASP_POSE = tuning.PRE_GRASP_POSE
GRASP_POSE = tuning.GRASP_POSE
LIFT_POSE = tuning.LIFT_POSE
PRE_DROP_POSE = tuning.PRE_DROP_POSE
DROP_POSE = tuning.DROP_POSE
GRIPPER_SIDE = tuning.GRIPPER_SIDE
GRIPPER_OPEN_M = tuning.GRIPPER_OPEN_M
GRIPPER_CLOSE_M = tuning.GRIPPER_CLOSE_M
GRIPPER_VELOCITY_MPS = tuning.GRIPPER_VELOCITY_MPS
GRIPPER_EFFORT_N = tuning.GRIPPER_EFFORT_N
TURN_AROUND_DEG = tuning.TURN_AROUND_DEG
SHIFT_LEFT_M = tuning.SHIFT_LEFT_M
DRIVE_DISTANCE_CM = tuning.DRIVE_DISTANCE_CM
TURN_RIGHT_TO_BIN_DEG = tuning.TURN_RIGHT_TO_BIN_DEG
ITEM_NAME = tuning.ITEM_NAME
ANNOUNCE_TEMPLATE = tuning.ANNOUNCE_TEMPLATE
ANNOUNCE_PAUSE_S = tuning.ANNOUNCE_PAUSE_S
MOVE_TIMEOUT_S = tuning.MOVE_TIMEOUT_S
ARM_SPEED_RAD_S = tuning.ARM_SPEED_RAD_S
_split_distance = tuning.split_distance


# ===========================================================================
# 编排查看
# ===========================================================================

def plan(params: dict = None) -> list:
    """不动机器人，只列出动作编排。现场对流程、评审看设计时用这个。

    返回 [(步骤 id, 说明, 实现类), ...]
    """
    p = params or {}
    cfg = pipeline_runner.load_config(str(p.get("pipeline") or DEFAULT_PIPELINE))
    return pipeline_runner.describe(cfg)


# ===========================================================================
# 结果整形
# ===========================================================================

def _shape(res: dict, ctx: Context, io, cfg_name: str) -> dict:
    """把 runner 的结构化结果翻译成场景契约的四字段 dict。"""
    trace = res.get("trace", [])
    home = next((t for t in trace if t.get("step") == "home"), None)
    home_data = (home or {}).get("data") or {}

    details = {
        "trace": trace,
        "pipeline": cfg_name,
        "io_backend": getattr(io, "backend", None),
        "item": ctx.params.get("item_name") or tuning.ITEM_NAME,
        "announce_text": ctx.flags.get("announce.text"),
        "steps_ok": res.get("counts", {}).get("ok", 0),
        "steps_skipped": res.get("counts", {}).get("skipped", 0),
        "home_arm_ok": home_data.get("arm_ok"),
        "home_base_ok": home_data.get("base_ok"),
    }
    if res.get("io_missing"):
        details["io_missing"] = res["io_missing"]
    if ctx.target:
        details["target"] = ctx.target.get("id")
    if ctx.verdict:
        details["verdict"] = ctx.verdict
    if ctx.picked is not None:
        details["picked"] = ctx.picked

    status = res.get("status")
    step, msg = res.get("failed_step"), res.get("failed_stage")

    if status == "ok":
        return {
            "success": True,
            "feedback": (f"基线清理完成：「{details['item']}」已从桌面取出、播报、"
                         f"送到另一张桌子并投入垃圾桶，机器人已回初始位姿"),
            "suggested_adjustment": None,
            "details": details,
        }

    if status == "unknown":
        details["stopped_without_safety"] = True
        return {
            "success": False,
            "feedback": f"步骤 {step} 状态不可判定（{msg}），已立即停止，未继续执行任何动作",
            "suggested_adjustment": ("这是 unknown 而非 failed：机器人当前状态未知，"
                                     "未自动归位。请人工确认底盘/手臂/夹爪状态后再决定是否复位重跑"),
            "details": details,
        }

    return {
        "success": False,
        "feedback": f"基线流程中断于步骤 {step}：{msg}",
        "suggested_adjustment": ("检查该步骤依赖的封装层接口与示教位姿；"
                                 "已执行部分见 details.trace，机器人已尝试归位"),
        "details": details,
    }


def _safe_reset(io=None) -> None:
    """硬性规范：无论成败都必须归位。归位本身失败也不能把主流程结果冲掉。

    mock 后端不走 sdk_wrapper.baseline.reset_to_baseline：那条路内部 _get_robot()
    依赖 GALBOT_MOCK 环境变量，与我们的 MED_IO 是两套开关，硬调会去 import galbot_sdk。
    mock 下改用 io.stop_base() 模拟"停底盘"，等价语义、不触达任何硬件。
    """
    try:
        if io is not None and getattr(io, "backend", "") == "mock":
            stop = getattr(io, "stop_base", None)
            if callable(stop):
                stop()
            logger.info("mock 后端：已模拟归位（底盘停，不触达硬件）")
            return
        baseline.reset_to_baseline()
    except Exception as exc:  # noqa: BLE001 - 归位失败只记日志
        logger.error("归位调用失败：%s", exc)


# ===========================================================================
# 入口
# ===========================================================================

def run(params: dict) -> dict:
    """场景入口，签名固定（docs/DEVELOPMENT_RULES.md 4.1）。"""
    p = dict(params or {})
    cfg_name = str(p.get("pipeline") or DEFAULT_PIPELINE)

    # ---- dry_run：只打印编排 ----
    if p.get("dry_run"):
        try:
            rows = plan(p)
        except Exception as exc:  # noqa: BLE001
            logger.exception("读取管线配置失败")
            return {"success": False, "feedback": f"读取管线配置失败：{exc}",
                    "suggested_adjustment": f"检查 pipeline/configs/{cfg_name}.json 是否存在且合法",
                    "details": {}}
        for step_id, text, impl in rows:
            print(f"[{step_id}] {text}\n      ↳ {impl}")
        return {"success": True,
                "feedback": f"dry_run：仅输出动作编排（{cfg_name}，共 {len(rows)} 步）",
                "suggested_adjustment": None,
                "details": {"pipeline": cfg_name, "steps": len(rows)}}

    # ---- 实跑 ----
    ctx = Context(params=p)
    io = None
    try:
        cfg = pipeline_runner.load_config(cfg_name)
        io = build_io(str(p.get("io") or ""), objects=p.get("objects"),
                      faults=p.get("mock_faults"))
        logger.info("管线 %s，IO 后端 %s，共 %d 步", cfg_name, io.backend, len(cfg["steps"]))
        res = pipeline_runner.run_pipeline(ctx, io, cfg)
    except Exception as exc:  # noqa: BLE001 - 场景异常不能拖垮主进程
        logger.exception("基线流程异常")
        return {"success": False, "feedback": f"基线流程异常: {exc}",
                "suggested_adjustment": "检查封装层接口状态，确认急停未按下，归位后重试",
                "details": {"trace": ctx.trace}}
    finally:
        _safe_reset(io)

    return _shape(res, ctx, io, cfg_name)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    print(json.dumps(run({"dry_run": True}), ensure_ascii=False, indent=2))
