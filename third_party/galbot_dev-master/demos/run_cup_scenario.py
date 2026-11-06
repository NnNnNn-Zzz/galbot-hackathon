"""彩排：在 Mock 上跑一遍"把不安全杯子推离桌沿"，验证闭环逻辑。

用法（在 galbot_dev/ 下）：
    python -m demos.run_cup_scenario          # 闭环，推到达标
    python -m demos.run_cup_scenario --stuck  # 模拟推不动（杯子被粘住）
    python -m demos.run_cup_scenario --bad-direction   # 模拟推反方向（联锁生效）

它**只用工具**，不碰 SDK，所以和在真机上跑的代码路径完全一致。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import typing

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from adapters import MockAdapter  # noqa: E402
from agent.tool_registry import build_default_registry  # noqa: E402
from frame_store import FrameStore  # noqa: E402
from robot_tools.perception import PerceptionTool  # noqa: E402


class StuckAdapter(MockAdapter):
    """推不动：末端动了，但杯子不跟着动（桌面很滑/杯子很轻）。"""

    def push_in_scene(self, dx: float, dy: float) -> None:  # noqa: D102
        return None


class WrongDirectionAdapter(MockAdapter):
    """方向反了：杯子被推得更靠近桌沿。"""

    def push_in_scene(self, dx: float, dy: float) -> None:  # noqa: D102
        return super().push_in_scene(-dx, -dy)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="推杯子场景彩排（Mock）")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--stuck", action="store_true", help="模拟推不动")
    group.add_argument("--bad-direction", action="store_true", help="模拟推反方向")
    parser.add_argument("--frames-dir", default=os.path.join(_ROOT, "runs", "frames"))
    args = parser.parse_args(argv)

    adapter_cls: type[MockAdapter] = MockAdapter
    label = "正常"
    if args.stuck:
        adapter_cls, label = StuckAdapter, "推不动（杯子不动）"
    elif args.bad_direction:
        adapter_cls, label = WrongDirectionAdapter, "方向反了（d 变小）"

    adapter = adapter_cls(scene="cup_table")
    frames = FrameStore(args.frames_dir)
    perception = PerceptionTool(adapter, frame_store=frames)
    registry = build_default_registry(perception=perception, adapter=adapter, mode=perception.mode)

    print(f"adapter={adapter_cls.__name__}（{label}）  tools={len(registry.names)}")
    print(f"已注册：{', '.join(registry.names)}\n")

    result = registry.dispatch("run_cup_scenario", {})
    payload = result.to_dict()
    data = payload["data"]

    print(f"status = {payload['status']}  error_code = {payload['error_code']}")
    print(f"message: {payload['message']}\n")
    for step in data.get("trace", []):
        print("  ·", json.dumps(step, ensure_ascii=False))
    if data.get("warnings"):
        print("\nwarnings:")
        for warning in data["warnings"]:
            print("  !", warning)
    if data.get("celebration"):
        print(f"\n庆祝动作：{json.dumps(data['celebration'], ensure_ascii=False)}")
    print(f"\n初始 d = {data.get('initial_d_m')} m → 最终 d = {data.get('final_d_m')} m "
          f"（目标 {data.get('target_m')} m，推了 {data.get('pushes')} 次，结论 {data.get('outcome')}）")
    print(f"\n语音播放：{adapter.audio_streams}  共 {adapter.audio_bytes_played} 字节")
    return 0 if payload["status"] == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
