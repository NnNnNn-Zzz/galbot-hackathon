"""顶层 Agent 主循环。

用法：
    # mock 模式（无真机调试，必走的第一步）
    uv run python agent/main.py --mock

    # 真机模式
    export LLM_API_KEY="sk-..."
    export LLM_BASE_URL="https://api.deepseek.com/v1"   # 可选
    export LLM_MODEL="deepseek-chat"                     # 可选
    uv run python agent/main.py

    # 带一次性任务启动（进入循环后第一轮就带着该任务决策）
    uv run python agent/main.py --task "往前走50cm"

    # 运行中随时下新任务：往项目根目录 task.txt 写入任务文本即可，
    # agent 下一轮读到后会执行并清空该文件
    echo "左转90度" > task.txt

架构说明：
    1. 启动时加载 prompt.txt + registry/scenes.yaml，拼出 system prompt。
    2. 每轮：观察（含用户任务）→ LLM 决策 → 执行（调封装层或场景）→ 写 task_log。
    3. 场景返回后做基线检查（记录不阻塞）。
    4. Agent 架构不固定，后续按比赛实际改本文件即可；LLM 提供方在 llm_client.py 换。
"""

import argparse
import logging
import os
import sys
import time

# 保证项目根目录在 sys.path（场景脚本 import sdk_wrapper 需要）
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from agent import memory, scene_loader  # noqa: E402
from agent.llm_client import create_llm_client  # noqa: E402
from sdk_wrapper import baseline, wrapper  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("agent.main")

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# 运行中下达任务的通道：往这个文件写入任务文本，agent 下一轮读取并清空
_TASK_FILE = os.path.join(_ROOT, "task.txt")


def poll_user_task() -> str | None:
    """读取 task.txt 里的用户任务。读到即清空文件，防止重复执行。"""
    if not os.path.exists(_TASK_FILE):
        return None
    try:
        with open(_TASK_FILE, "r", encoding="utf-8") as f:
            content = f.read().strip()
        if content:
            with open(_TASK_FILE, "w", encoding="utf-8") as f:
                f.write("")
            logger.info("收到用户任务: %s", content)
            return content
    except OSError as e:
        logger.warning("读取任务文件失败: %s", e)
    return None


def build_system_prompt() -> str:
    """用户提示词 + 可用能力清单 → system prompt。"""
    with open(os.path.join(_ROOT, "agent", "prompt.txt"), "r", encoding="utf-8") as f:
        user_prompt = f.read()
    scenes_desc = scene_loader.registry_summary()
    return f"""{user_prompt}

【你可以调用的封装硬件接口（sdk_wrapper）】
- move_forward(distance_cm): 前进指定厘米数（自动限幅）
- move_backward(distance_cm): 后退
- turn(angle_deg): 转向，正值左转
- stop_base(): 停止底盘
- read_sensor(sensor_name): 读传感器
- reset_to_baseline(): 回到基线状态

【你可以调用的场景脚本】
{scenes_desc}

【输出格式】严格输出 JSON：
{{"action": "call_scene"|"call_wrapper"|"wait"|"finish",
  "target": "场景名或函数名",
  "params": {{}},
  "reason": "决策理由"}}"""


def observe(user_task: str | None = None, one_shot: bool = False) -> dict:
    """采集当前观察（多模态：文本 + 相机帧路径列表）。

    返回格式固定为 {"text": str, "images": [图片路径, ...]}，
    llm_client 会把 images 里的本地图片按多模态消息格式发给模型；
    images 为空时自动退化为纯文本，纯文本模型也能跑。

    one_shot=True（--task 一次性任务模式）时，无新任务的轮次提示 LLM
    finish/wait 而不是默认巡逻——防止验证任务跑完后机器人自主乱动。

    TODO(负责人A): 接入 G1 相机后，在这里抓帧存成临时 jpg 放进 images，
    例如 wrapper.read_sensor('front_camera') → 存 /tmp/agent_frame.jpg。
    """
    if user_task:
        text = f"【用户下达的任务】{user_task}。请规划并执行该任务，完成后输出 finish。"
    elif one_shot:
        text = ("此前下达的任务已处理。若任务已完成，请输出 finish 结束程序；"
                "若还在执行中可继续调用接口；无事可做就 wait。"
                "禁止自主巡逻或执行其他场景。")
    else:
        text = "暂无感知输入（mock/待接入感知），请按提示词默认策略行动。"
    return {
        "text": text,
        "images": [],  # 接入相机后填本地图片路径，如 ["/tmp/agent_frame.jpg"]
    }


def execute_decision(decision: dict) -> dict:
    """执行 LLM 决策。"""
    action = decision.get("action")
    target = decision.get("target")
    params = decision.get("params") or {}

    if action == "call_scene":
        result = scene_loader.run_scene(target, params)
        # 场景后基线检查：记录不阻塞
        check = baseline.check_baseline()
        if not check.get("at_baseline"):
            memory.log("unsafe_exit", {"scene": target, "detail": check.get("detail")})
            baseline.reset_to_baseline()
        return result
    elif action == "call_wrapper":
        fn = getattr(wrapper, target, None)
        if fn is None or target.startswith("_"):
            return {"success": False, "feedback": f"封装层无此函数: {target}"}
        try:
            r = fn(**params)
            return {"success": True, "feedback": str(r)}
        except TypeError as e:
            return {"success": False, "feedback": f"参数错误: {e}"}
    elif action in ("wait", "finish", None):
        return {"success": True, "feedback": action or "wait"}
    return {"success": False, "feedback": f"未知 action: {action}"}


def main():
    parser = argparse.ArgumentParser(description="G1 顶层 Agent")
    parser.add_argument("--mock", action="store_true", help="mock 模式（无真机调试）")
    parser.add_argument("--interval", type=float, default=5.0, help="决策间隔秒数")
    parser.add_argument("--max-loops", type=int, default=0, help="最大循环数，0=不限")
    parser.add_argument("--task", type=str, default=None,
                        help="启动时下达的一次性任务，如 \"往前走50cm\"")
    args = parser.parse_args()

    if args.mock:
        wrapper.set_mock_mode(True)
        logger.info("=== MOCK 模式 ===")

    llm = create_llm_client()
    system_prompt = build_system_prompt()
    logger.info("Agent 已启动，决策间隔 %.1fs", args.interval)
    if args.task:
        logger.info("启动任务: %s", args.task)

    pending_task = args.task  # 首轮注入的一次性任务
    loop = 0
    try:
        while True:
            loop += 1
            if args.max_loops and loop > args.max_loops:
                break
            # 任务优先级：启动参数 > task.txt 文件
            task = pending_task or poll_user_task()
            pending_task = None
            observation = observe(task, one_shot=args.task is not None)
            try:
                decision = llm.decide(system_prompt, observation,
                                      memory.read_history(10))
            except Exception as e:  # noqa: BLE001
                logger.error("LLM 决策失败: %s", e)
                time.sleep(args.interval)
                continue

            logger.info("决策 #%d: %s", loop, decision)
            memory.log("decision", {"loop": loop, **decision})

            result = execute_decision(decision)
            logger.info("结果: %s", result)
            memory.log("result", {"loop": loop, **result})

            if decision.get("action") == "finish":
                logger.info("LLM 决定结束任务")
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，正在归位...")
    finally:
        baseline.reset_to_baseline()
        wrapper.shutdown()  # 真机模式必须走 SDK 关机序列
        logger.info("Agent 退出")


if __name__ == "__main__":
    main()
