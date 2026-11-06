"""队友的插拔区：每个 .py 在这里会被自动发现，无需改本文件或 __init__.py。

只要满足三条边界（详见 docs/plugin_guide.md）：
  1. 不 import galbot_sdk、不 import sdk_wrapper、不 import adapters；
  2. 所有硬件动作只经 run(ctx, io) 的 io 参数发起；
  3. 返回 StageResult（用 ok / failed / unknown 构造），不要返回裸 dict。
"""
