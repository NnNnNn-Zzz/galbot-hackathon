"""agent/app.py —— 多模态 Agent 主程序（B 负责）。

用法（工作目录必须是 galbot_dev/）：
    python -m agent.app --list-tools                       # 看注册了哪些工具
    python -m agent.app --demo                             # 离线演示（无需网络/真机）
    python -m agent.app --once "你看到了什么"               # 单轮
    python -m agent.app                                    # 交互式 REPL

接真模型（任何 OpenAI 兼容端点）：
    set OPENAI_API_KEY=sk-xxx
    python -m agent.app --model qwen-vl-max --base-url https://dashscope.aliyuncs.com/compatible-mode/v1 --vision

接真机（A 的适配器就位后）：
    python -m agent.app --adapter real

铁律（协议 §1/§5）：
    * `--adapter real` 导入失败就**报错退出**，绝不静默退回 mock；
    * 每个 Tool 的返回都要带 mode，模型必须把 mock 结果说成 mock；
    * 导航/动作类工具即使返回 succeeded，也必须以 A 的任务状态为准（本层只转发）。
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import json
import logging
import os
import sys
import typing

# 允许 `python agent/app.py` 与 `python -m agent.app` 两种跑法
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from agent.llm_client import LLMError, build_client, image_data_url  # noqa: E402
from agent.offline_demo_adapter import OfflineSceneAdapter  # noqa: E402
from agent.tool_registry import ToolRegistry, build_default_registry  # noqa: E402
from contracts import RunMode, to_json  # noqa: E402
from frame_store import FrameStore  # noqa: E402
from robot_tools.perception import PerceptionTool  # noqa: E402

LOG = logging.getLogger("agent")

SYSTEM_PROMPT = """你是银河通用 Galbot G1 人形机器人的智能体。你通过工具感知和行动。

必须遵守：
1. 只能用已注册的工具表达"动作"。不要假装自己长了眼睛或腿。
2. 距离单位一律是米(m)，角度是弧度(rad)，时间秒(s)。图像坐标 (u,v) = (列, 行)。
3. 每个工具返回都含 `mode` 与 `status`：
   - mode=mock 表示"模拟数据"。你必须明确告诉用户这是模拟结果，不能说成真机。
   - status=succeeded 才算真的成功；running 只是任务已启动，必须用任务查询工具确认到达；
     rejected/failed/unknown 要如实转述 error_code，不要粉饰。
4. 不要编造没读到的数据。工具说"没找到/不可用"就照实说，并给出下一步建议。
5. 坐标必须带坐标系。相机系坐标要注明 coordinate_frame。
6. 回答简洁，先说结论再说依据。中文回答。
"""


# ═══════════════════════════════════════════════════════════════════════════ #
# 适配器装配（A 的 adapters/ 就位后自动可用，否则用 B 的离线合成场景）
# ═══════════════════════════════════════════════════════════════════════════ #
_ADAPTER_TARGETS: dict[str, tuple[str, tuple[str, ...]]] = {
    "mock": ("adapters.mock_adapter", ("MockAdapter", "MockRobotAdapter", "Adapter")),
    "real": ("adapters.galbot_adapter", ("GalbotAdapter", "GalbotRobotAdapter", "RealAdapter", "Adapter")),
}


class AdapterNotAvailable(RuntimeError):
    pass


#: 感知层需要的相机能力；缺任何一个，视觉类工具都会返回 not_supported
_CAMERA_METHODS = (
    "get_rgb_data",
    "get_depth_data",
    "get_camera_intrinsic",
    "get_sensor_extrinsic",
)


def _call_with_supported_kwargs(target: typing.Any, kwargs: dict[str, typing.Any]) -> typing.Any:
    """只把 target 真正接受的关键字参数传进去，签名不符时不至于直接崩。"""
    if not kwargs:
        return target()
    try:
        accepted_names = inspect.signature(target).parameters
    except (TypeError, ValueError):
        return target()
    return target(**{k: v for k, v in kwargs.items() if k in accepted_names})


def load_adapter(kind: str, *, allow_real_control: bool = False) -> typing.Any:
    """按名字加载 adapter。mock/real 找不到就**报错**，绝不降级到离线替身。

    ``allow_real_control`` 只在 kind="real" 时生效，且必须由操作者显式开启：
    它会同时打开 adapter 的 ``enable_real_control`` 和 ``real_authorized``
    （A 的 GalbotAdapter 默认两个都是 False，所以不显式授权就永远拒绝执行）。
    """
    if kind == "offline":
        return OfflineSceneAdapter()

    module_name, class_names = _ADAPTER_TARGETS[kind]
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise AdapterNotAvailable(
            f"找不到 {module_name}（{exc}）。\n"
            f"  · 要用 B 的离线合成场景：--adapter offline\n"
            f"  · 要用真机：请让 A 先完成 {module_name}"
        ) from exc

    kwargs: dict[str, typing.Any] = {}
    if kind == "real" and allow_real_control:
        kwargs = {"enable_real_control": True, "real_authorized": True}

    for factory_name in ("build_adapter", "create_adapter", "build", "create"):
        factory = getattr(module, factory_name, None)
        if callable(factory):
            return _call_with_supported_kwargs(factory, kwargs)

    for class_name in class_names:
        cls = getattr(module, class_name, None)
        if cls is None:
            continue
        try:
            return _call_with_supported_kwargs(cls, kwargs)
        except TypeError as exc:
            raise AdapterNotAvailable(
                f"{module_name}.{class_name}() 需要构造参数（{exc}）；"
                f"请提供一个无参工厂 build_adapter()，便于 app.py 装配"
            ) from exc
    raise AdapterNotAvailable(
        f"{module_name} 里没有找到 {class_names} 或 build_adapter()"
    )


def _release_adapter(adapter: typing.Any) -> None:
    """按 SDK 规定顺序释放资源。

    优先用 ``adapter.close()``（A 的实现应把三步都做在里头）；没有 close 就退回
    逐个调用 ``request_shutdown() -> wait_for_shutdown() -> destroy()``。
    任何一步失败都只警告，不再抛异常 —— 进程退出路径上不该二次崩溃。
    """
    if adapter is None:
        return
    close = getattr(adapter, "close", None)
    if callable(close):
        try:
            close()
        except Exception as exc:  # noqa: BLE001
            print(f"[警告] adapter.close() 失败：{exc}", file=sys.stderr)
        return
    for name in ("request_shutdown", "wait_for_shutdown", "destroy"):
        func = getattr(adapter, name, None)
        if not callable(func):
            continue
        try:
            func()
        except Exception as exc:  # noqa: BLE001
            print(f"[警告] adapter.{name}() 失败：{exc}", file=sys.stderr)


def check_camera_capability(adapter: typing.Any, kind: str) -> list[str]:
    """adapter 缺相机方法时给出明确提示，避免演示时被误认为"项目坏了"。"""
    missing = [name for name in _CAMERA_METHODS if not hasattr(adapter, name)]
    if missing:
        print(
            f"[提示] adapter={kind} 未实现相机能力（缺 {', '.join(missing)}）：\n"
            f"       capture_scene / analyze_scene / find_object / measure_distance 会返回 not_supported。\n"
            f"       要完整演示感知请用 --adapter offline，或让 A 在 adapters 里补齐这 4 个方法。",
            file=sys.stderr,
        )
    return missing


# ═══════════════════════════════════════════════════════════════════════════ #
# Agent
# ═══════════════════════════════════════════════════════════════════════════ #
class AgentApp:
    def __init__(
        self,
        registry: ToolRegistry,
        client: typing.Any,
        *,
        frames: FrameStore | None = None,
        max_turns: int = 6,
        vision: bool = False,
        verbose: bool = False,
    ) -> None:
        self.registry = registry
        self.client = client
        self.frames = frames
        self.max_turns = int(max_turns)
        self.vision = bool(vision)
        self.verbose = bool(getattr(logging, "DEBUG", 10) == logging.getLogger().level or verbose)
        self.messages: list[dict[str, typing.Any]] = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {
                "role": "system",
                "content": (
                    f"当前执行模式：{registry.mode.value}。"
                    + ("这是模拟数据，任何结论都必须注明[模拟]。" if registry.mode is RunMode.MOCK else "这是真机数据。")
                ),
            },
        ]
        self._last_rgb_path: str | None = None
        # 让离线规则引擎知道到底有哪些工具可用（用于诚实地解释"能力缺口"）
        if hasattr(client, "tool_names"):
            client.tool_names = set(registry.names)

    # ---- 主循环 ----
    def ask(self, text: str) -> str:
        self.messages.append({"role": "user", "content": text})
        for turn in range(self.max_turns + 1):
            try:
                response = self.client.complete(self.messages, self.registry.schemas())
            except LLMError as exc:
                return f"[模型调用失败] {exc}"

            if not response.wants_tools:
                self.messages.append({"role": "assistant", "content": response.content})
                return response.content or "（模型没有返回内容）"

            self.messages.append(
                {
                    "role": "assistant",
                    "content": response.content or None,
                    "tool_calls": [call.as_message_part() for call in response.tool_calls],
                }
            )
            for call in response.tool_calls:
                result = self.registry.dispatch(call.name, call.arguments)
                payload = result.to_dict()
                if self.verbose:
                    LOG.info("tool=%s args=%s -> %s", call.name, call.arguments, payload.get("status"))
                self._note_frame(payload)
                self.messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call.id,
                        "name": call.name,
                        "content": to_json(payload),
                    }
                )
        return "（达到最大工具轮数，已停止；请缩小问题范围）"

    # ---- 视觉注入 ----
    def _note_frame(self, payload: typing.Mapping[str, typing.Any]) -> None:
        data = payload.get("data") or {}
        capture_id = data.get("capture_id")
        if not capture_id or self.frames is None:
            return
        try:
            record = self.frames.record(str(capture_id))
        except Exception:  # noqa: BLE001
            return
        if record.rgb_path:
            self._last_rgb_path = record.rgb_path

    def maybe_attach_image(self) -> None:
        """把最近一帧图示给支持视觉的模型。失败不影响主流程。"""
        if not (self.vision and self._last_rgb_path):
            return
        try:
            url = image_data_url(self._last_rgb_path)
        except Exception as exc:  # noqa: BLE001
            LOG.warning("图像注入失败：%s", exc)
            return
        self.messages.append(
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "这是上一步抓到的画面，请结合它回答。"},
                    {"type": "image_url", "image_url": {"url": url}},
                ],
            }
        )

    # ---- 视觉增强版 ask ----
    def ask_with_image(self, text: str) -> str:
        answer = self.ask(text)
        if self.vision and self._last_rgb_path:
            self.maybe_attach_image()
            follow = self.ask("结合这张图，再确认一次你的结论。")
            return answer + "\n\n[视觉复核] " + follow
        return answer

    def reset(self) -> None:
        self.messages = self.messages[:2]


# ═══════════════════════════════════════════════════════════════════════════ #
# CLI
# ═══════════════════════════════════════════════════════════════════════════ #
# 演示问题集：按"实际注册了哪些工具"决定演示哪几问
# ═══════════════════════════════════════════════════════════════════════════ #
DEMO_VISION_QUESTIONS = (
    "有哪些相机可用？",
    "你看到了什么？",
    "红色方块在哪，离我多远？",
    "最近的前景物体是什么？",
    "像素 (230, 300) 处的距离是多少？",
)

DEMO_EXECUTION_QUESTIONS = (
    "机器人现在状态怎么样？",
    "挥个手",
    "任务完成了吗？",
    "取消任务",
    "去 demo_table",   # 已登记地点 -> running -> 可轮询
    "去前台",          # 未登记地点 -> 应被 rejected，演示 fail-closed
)


def demo_questions(registry: ToolRegistry) -> tuple[str, ...]:
    """A 的执行类工具没注册时（如 offline 模式），只演示感知部分。"""
    if registry.get("get_robot_state") is None:
        return DEMO_VISION_QUESTIONS
    return DEMO_VISION_QUESTIONS + DEMO_EXECUTION_QUESTIONS


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m agent.app",
        description="Galbot G1 多模态 Agent（感知层实机/Mock 均可）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--adapter", choices=("offline", "mock", "real"), default="offline",
        help="offline=B 的合成场景(默认); mock=A 的 adapters/mock_adapter; real=A 的真机适配器",
    )
    parser.add_argument("--model", default=None, help="模型名，如 qwen-vl-max / deepseek-chat / gpt-4o-mini")
    parser.add_argument("--base-url", default=None, help="OpenAI 兼容 base_url")
    parser.add_argument("--api-key", default=None, help="默认读环境变量 OPENAI_API_KEY")
    parser.add_argument("--vision", action="store_true", help="把抓到的图也发给模型（需要多模态模型）")
    parser.add_argument(
        "--allow-real-control",
        action="store_true",
        help="显式授权驱动真机（默认关闭）。开启后才把 enable_real_control/real_authorized 传给 A",
    )
    parser.add_argument("--max-turns", type=int, default=6, help="单个问题的最大工具轮数")
    parser.add_argument("--once", default=None, metavar="TEXT", help="只问一个问题就退出")
    parser.add_argument("--demo", action="store_true", help="跑一遍固定的演示问题集")
    parser.add_argument("--list-tools", action="store_true", help="打印已注册工具后退出")
    parser.add_argument("--frames-dir", default=os.path.join(_ROOT, "runs", "frames"))
    parser.add_argument("--no-save-frames", action="store_true", help="不落盘（省磁盘）")
    parser.add_argument("--config", default=None, help="cameras.json 路径")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    try:
        adapter = load_adapter(args.adapter, allow_real_control=args.allow_real_control)
    except AdapterNotAvailable as exc:
        print(f"[装配失败] {exc}", file=sys.stderr)
        return 2

    if args.allow_real_control and args.adapter == "real":
        print(
            "[警告] 已开启 --allow-real-control：执行类工具会真实驱动机器人。"
            "软件授权不能替代物理急停与现场监护。",
            file=sys.stderr,
        )
    check_camera_capability(adapter, args.adapter)

    try:
        return _run(args, adapter)
    except KeyboardInterrupt:
        print("\n（已中断，正在释放资源…）", file=sys.stderr)
        return 130
    finally:
        # SDK 要求 request_shutdown() -> wait_for_shutdown() -> destroy() 才能干净退出
        # （g1/__init__.pyi:2839 / 3351 / 2259）。缺一步都可能留下未停止的中间件订阅，
        # 导致下次启动连不上机器人 —— 现场最难排查的那类故障。
        _release_adapter(adapter)


def _run(args: argparse.Namespace, adapter: typing.Any) -> int:
    """真正的运行段；资源释放在 main() 的 finally 里统一做。"""
    frames = FrameStore(args.frames_dir)
    perception_kwargs: dict[str, typing.Any] = {"frame_store": frames, "save_frames": not args.no_save_frames}
    if args.config:
        perception_kwargs["config_path"] = args.config
    perception = PerceptionTool(adapter, **perception_kwargs)

    registry = build_default_registry(
        perception=perception,
        adapter=adapter,
        mode=perception.mode,
        real_authorized=args.allow_real_control,
    )

    if args.list_tools:
        manifest = registry.manifest()
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
        by_owner: dict[str, int] = {}
        for item in manifest:
            by_owner[item["owner"]] = by_owner.get(item["owner"], 0) + 1
        summary = "，".join(f"{owner}={count}" for owner, count in sorted(by_owner.items()))
        print(f"# 共 {len(manifest)} 个工具：{summary}", file=sys.stderr)
        return 0

    client = build_client(
        model=args.model, base_url=args.base_url, api_key=args.api_key, supports_vision=args.vision
    )
    app = AgentApp(
        registry, client, frames=frames, max_turns=args.max_turns, vision=args.vision, verbose=args.verbose
    )
    banner = (
        f"adapter={args.adapter}  mode={perception.mode.value}  "
        f"llm={getattr(client, 'model', getattr(client, 'name', type(client).__name__))}  "
        f"tools={len(registry.names)}"
    )
    print(banner)
    if args.once:
        print(app.ask_with_image(args.once))
        return 0
    if args.demo:
        for question in demo_questions(registry):
            print(f"\n你> {question}")
            print(f"机器人> {app.ask(question)}")
        return 0
    return repl(app)


def repl(app: AgentApp) -> int:
    print("输入问题，Ctrl-C / 空行退出。")
    while True:
        try:
            text = input("\n你> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not text:
            return 0
        if text in {"exit", "quit", ":q"}:
            return 0
        if text in {"reset", ":r"}:
            app.reset()
            print("（上下文已清空）")
            continue
        try:
            print(f"机器人> {app.ask_with_image(text)}")
        except KeyboardInterrupt:
            print("\n（已中断）")


if __name__ == "__main__":
    raise SystemExit(main())
