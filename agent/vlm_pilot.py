"""VLM 直控模式：多模态大模型看着相机画面，一步步直接开机器人。

与 main.py 的区别
-----------------
main.py 是"Agent 决策 → 调场景/封装函数"，场景内部是写死的流程。
本模式把循环压平：**每个控制周期抓 3 路相机 → 模型看完整画面 →
直接输出一个底层动作**（手臂平移/夹爪/底盘），立即执行，再抓下一帧。
模型对任务的规划、纠错全靠它自己看画面，我们不给它任何指导和反馈，
只把它自己过去发出的动作历史带给它（防止失忆重复动作）。

用法：
    # mock 调试（无 API Key 时用内置脚本假模型跑通管线）
    uv run python agent/vlm_pilot.py --mock --max-steps 3

    # 真机 / 真模型
    export LLM_API_KEY="sk-..."
    export LLM_BASE_URL="https://api.deepseek.com/v1"   # 视提供方而定
    export LLM_VLM_MODEL="deepseek-vl2"                  # 多模态模型名
    uv run python agent/vlm_pilot.py --task "把桌上的水杯夹起来"

每步产物落盘到 sdk_wrapper/dev/runs/vlm_pilot/<时间戳>/：
    step_0001_head.jpg / step_0001_left_wrist.jpg / step_0001_right_wrist.jpg
    decisions.jsonl     （每行：模型原始决策 + 执行结果 + 图片路径）
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk_wrapper import baseline, wrapper  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("agent.vlm_pilot")

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# ---- 可调参数 ------------------------------------------------------------ #
#: 三路相机：内部名 → (toolhub 相机名, 保存文件名, 给模型看的中文标签)
CAMERAS = {
    "head":        ("head_left", "head.jpg",        "头部相机（全局视野，看桌面整体）"),
    "left_wrist":  ("left_arm",  "left_wrist.jpg",  "左腕相机（眼在手上，近距离对准目标）"),
    "right_wrist": ("right_arm", "right_wrist.jpg", "右腕相机（另一侧视角）"),
}
#: 默认用左臂抓取（左腕相机已摆好位置）；想换右臂改这里 + --arm right_arm
DEFAULT_ARM = "left_arm"
GRIPPER_OF = {"left_arm": "left_gripper", "right_arm": "right_gripper"}

#: 单张图超过该字节就不发（保护带宽/token）
MAX_IMAGE_BYTES = int(os.environ.get("LLM_MAX_IMAGE_BYTES", str(4 * 1024 * 1024)))


# ---- system prompt：只给规则，不给任务指导 ------------------------------- #
def build_system_prompt(task: str, arm: str) -> str:
    gripper = GRIPPER_OF[arm]
    return f"""你正在直接控制一台 Galbot G1 人形机器人。每个控制周期你会收到 3 张相机画面
（按顺序：头部相机、左腕相机、右腕相机），然后你必须只输出一个动作，立即执行。

【任务】{task}

【可用动作】严格输出一个 JSON，action 只能是以下之一：
1. {{"action": "move_arm", "params": {{"dx": 0.0, "dy": 0.0, "dz": 0.0}}, "reason": "..."}}
   手臂末端平移，单位米，base_link 坐标系（x 向前，y 向左，z 向上）。
   单轴每步最大 0.10m，建议单步 0.02~0.05m，小步慢调。当前控制的是 {arm}。
2. {{"action": "open_gripper", "params": {{}}, "reason": "..."}}  张开 {gripper}
3. {{"action": "close_gripper", "params": {{}}, "reason": "..."}} 闭合 {gripper}
4. {{"action": "move_base", "params": {{"distance_cm": 0}}, "reason": "..."}}
   底盘前进（正）/后退（负），厘米，小步。
5. {{"action": "turn", "params": {{"angle_deg": 0}}, "reason": "..."}}
   原地转向，正左转负右转，度。
6. {{"action": "wait", "params": {{}}, "reason": "..."}}  等一拍（等画面稳定）
7. {{"action": "finish", "params": {{}}, "reason": "..."}} 任务完成，结束

【规则】
- 每步只输出一个 JSON，不要输出任何其他文字。
- 看画面自己判断下一步；夹取前先张爪，对准后再闭爪。
- 看不清目标时可以先移动手臂/底盘换个视角。
- 硬件限制：手臂不能跨过身体中线（y≈0），目标在中线另一侧时先用 turn 转底盘让目标对正你这条手臂。
- 确认杯子已被夹起离开桌面后输出 finish。"""


# ---- 模型调用（OpenAI vision 兼容，零依赖） ------------------------------ #
def _encode_image(path: str) -> dict | None:
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        return None
    if len(data) > MAX_IMAGE_BYTES:
        logger.warning("图片 %s 超 %d 字节，跳过", path, MAX_IMAGE_BYTES)
        return None
    b64 = base64.b64encode(data).decode("ascii")
    return {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}


def vlm_decide(system_prompt: str, frames: list[tuple[str, str]],
               action_history: list[dict]) -> dict:
    """把 3 路画面 + 自己过去的动作历史发给 VLM，返回一个动作 JSON。

    frames: [(图片路径, 中文标签), ...]。只带动作历史，不带任何成败反馈——
    模型自己看新画面判断上一动有没有奏效。
    """
    api_key = os.environ.get("LLM_API_KEY", "")
    if not api_key:
        raise RuntimeError("未设 LLM_API_KEY")
    base_url = os.environ.get("LLM_BASE_URL", "https://api.deepseek.com/v1")
    model = os.environ.get("LLM_VLM_MODEL",
                           os.environ.get("LLM_MODEL", "deepseek-vl2"))

    hist_text = "（暂无）" if not action_history else "\n".join(
        f"  第{i+1}步: {h['action']} {json.dumps(h.get('params', {}), ensure_ascii=False)}"
        for i, h in enumerate(action_history[-15:]))  # 只带最近15步，防爆token
    text = (f"以下是机器人当前的 {len(frames)} 路相机画面。\n"
            f"你过去发出的动作（仅供参考，结果请自己从画面判断）：\n{hist_text}\n"
            f"请输出下一个动作 JSON。")

    content = [{"type": "text", "text": text}]
    for path, label in frames:
        block = _encode_image(path)
        if block:
            content.append({"type": "text", "text": f"【{label}】"})
            content.append(block)

    body = json.dumps({
        "model": model,
        "messages": [{"role": "system", "content": system_prompt},
                     {"role": "user", "content": content}],
        "response_format": {"type": "json_object"},
        "temperature": 0.2,
    }).encode("utf-8")
    req = urllib.request.Request(
        f"{base_url}/chat/completions", data=body,
        headers={"Content-Type": "application/json",
                 "Authorization": f"Bearer {api_key}"})
    with urllib.request.urlopen(req, timeout=90) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    raw = data["choices"][0]["message"]["content"]
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        start = raw.find("{")
        if start < 0:
            raise
        obj, _ = json.JSONDecoder().raw_decode(raw[start:])
        return obj


class ScriptedClient:
    """mock 联调用：没 API Key 时的假模型，按固定剧本出动作，验证管线。"""

    SCRIPT = [
        {"action": "open_gripper", "params": {}, "reason": "[剧本] 先张爪"},
        {"action": "move_arm", "params": {"dx": 0.05, "dy": 0.0, "dz": -0.03},
         "reason": "[剧本] 向前下方伸"},
        {"action": "close_gripper", "params": {}, "reason": "[剧本] 闭爪"},
        {"action": "move_arm", "params": {"dx": 0.0, "dy": 0.0, "dz": 0.08},
         "reason": "[剧本] 抬起"},
        {"action": "finish", "params": {}, "reason": "[剧本] 完成"},
    ]

    def __init__(self):
        self._i = 0

    def decide(self, *_args) -> dict:
        d = self.SCRIPT[min(self._i, len(self.SCRIPT) - 1)]
        self._i += 1
        return dict(d)


# ---- 动作执行（全走 wrapper，安全限幅天然生效） --------------------------- #
def execute(decision: dict, arm: str) -> dict:
    action = decision.get("action")
    params = decision.get("params") or {}
    gripper = GRIPPER_OF[arm]
    try:
        if action == "move_arm":
            return wrapper.move_arm_delta(
                float(params.get("dx", 0)), float(params.get("dy", 0)),
                float(params.get("dz", 0)), chain=arm)
        if action == "open_gripper":
            return wrapper.open_gripper(gripper)
        if action == "close_gripper":
            return wrapper.close_gripper(gripper)
        if action == "move_base":
            return wrapper.move_forward(float(params.get("distance_cm", 0)))
        if action == "turn":
            return wrapper.turn(float(params.get("angle_deg", 0)))
        if action in ("wait", "finish"):
            return {"success": True, "message": action}
        return {"success": False, "message": f"未知动作: {action}"}
    except Exception as e:  # noqa: BLE001
        return {"success": False, "message": f"执行异常: {e}"}


# ---- 主循环 --------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="G1 VLM 直控模式")
    parser.add_argument("--mock", action="store_true", help="mock 模式")
    parser.add_argument("--task", default="把桌上的水杯夹起来", help="任务描述")
    parser.add_argument("--arm", default=DEFAULT_ARM,
                        choices=["left_arm", "right_arm"], help="用哪条臂抓取")
    parser.add_argument("--interval", type=float, default=1.0, help="步间间隔秒")
    parser.add_argument("--max-steps", type=int, default=40, help="最大步数保险丝")
    args = parser.parse_args()

    if args.mock:
        wrapper.set_mock_mode(True)
        logger.info("=== MOCK 模式 ===")

    run_dir = os.path.join(_ROOT, "sdk_wrapper", "dev", "runs", "vlm_pilot",
                           time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "decisions.jsonl")
    logger.info("运行目录: %s", run_dir)

    use_script = args.mock and not os.environ.get("LLM_API_KEY")
    scripted = ScriptedClient() if use_script else None
    if use_script:
        logger.warning("无 LLM_API_KEY，用内置剧本假模型验证管线")
    system_prompt = build_system_prompt(args.task, args.arm)
    action_history: list[dict] = []
    consecutive_faults = 0

    logger.info("任务: %s | 手臂: %s | 相机: %s", args.task, args.arm,
                ", ".join(CAMERAS))
    try:
        for step in range(1, args.max_steps + 1):
            # 1) 抓 3 路画面（失败的相机跳过并在文本里说明，不中断）
            frames: list[tuple[str, str]] = []
            failed: list[str] = []
            for key, (cam_id, fname, label) in CAMERAS.items():
                path = os.path.join(run_dir, f"step_{step:04d}_{fname}")
                r = wrapper.capture_frame(cam_id, save_path=path)
                if r.get("success"):
                    frames.append((path, label))
                else:
                    failed.append(f"{key}({r.get('message', '?')})")
                    logger.warning("相机 %s 取图失败: %s", key, r.get("message"))
            if not frames:
                logger.error("所有相机取图失败，终止")
                break

            # 2) 模型决策
            try:
                decision = (scripted.decide() if scripted
                            else vlm_decide(system_prompt, frames, action_history))
            except Exception as e:  # noqa: BLE001
                logger.error("第 %d 步模型决策失败: %s", step, e)
                time.sleep(args.interval)
                continue
            action_history.append({"action": decision.get("action"),
                                   "params": decision.get("params") or {}})

            # 3) 展示 + 执行 + 落盘（只记录，不把成败反馈给模型）
            print(f"\n[step {step:03d}] 决策: {json.dumps(decision, ensure_ascii=False)}")
            if failed:
                print(f"          （取图失败已跳过: {'; '.join(failed)}）")
            result = execute(decision, args.arm)
            print(f"          执行: {result.get('success')} - {result.get('message', '')}")

            # 安全熔断：运动连续 FAULT 说明在够不可达区域/碰撞边缘，
            # 模型收不到失败反馈会一直重复同一动作（00:12 真机实测连撞 17 次）。
            # 安全红线「出 FAULT 立刻停，不要连续重试」由这里兜底。
            if result.get("status") == "failed" or result.get("error_code"):
                consecutive_faults += 1
                if consecutive_faults >= 3:
                    logger.error("连续 %d 次 FAULT，熔断停机（模型在无反馈空转）",
                                 consecutive_faults)
                    break
            else:
                consecutive_faults = 0
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({
                    "step": step, "decision": decision, "result": {
                        k: v for k, v in result.items() if k != "depth_m"},
                    "images": [p for p, _ in frames],
                }, ensure_ascii=False) + "\n")

            if decision.get("action") == "finish":
                logger.info("模型判断任务完成，共 %d 步", step)
                break
            time.sleep(args.interval)
        else:
            logger.warning("达到最大步数 %d，强制停止", args.max_steps)
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，停止")
    finally:
        wrapper.stop_base()
        baseline.reset_to_baseline()
        wrapper.shutdown()
        logger.info("退出。决策记录: %s", log_path)


if __name__ == "__main__":
    main()
