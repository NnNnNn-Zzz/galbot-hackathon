"""agent/ —— B 的"大脑"：多模态模型 + Tool 注册 + 对话调度。

    tool_registry.py        工具 schema 生成与分发（OpenAI function-calling 格式）
    llm_client.py           OpenAI 兼容 HTTP 客户端 + 离线规则引擎
    offline_demo_adapter.py 无机器人时的合成场景适配器（**离线演示用，不是 A 的交付物**）
    app.py                  CLI 入口：REPL / 单轮 / 演示

依赖方向：agent -> robot_tools -> algorithms，绝不反向。
"""

from __future__ import annotations

__all__ = ["tool_registry", "llm_client", "app"]
