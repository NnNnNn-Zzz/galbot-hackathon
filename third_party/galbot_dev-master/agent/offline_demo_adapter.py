"""agent/offline_demo_adapter.py —— 无机器人时的**纯相机**替身。

⚠️ 这不是 A 的交付物。合成场景的实现已经搬到
   ``adapters/synthetic_scene.py``（与 ``MockAdapter`` 共用同一份代码），
   本文件只保留一层薄壳，供 ``--adapter offline`` 使用：

       offline   —— 只有眼睛（没有执行能力），用来单独验证感知链路
       mock      —— 眼睛 + 执行（``adapters.mock_adapter.MockAdapter``）

两者的相机 payload 完全一致，且都与真机格式相同（JPEG / 裸 16UC1 / CameraInfo / 外参），
所以 Mock 和真机走的是同一段解码代码，不会"模拟通过、真机炸"。
"""

from __future__ import annotations

from adapters.synthetic_scene import (  # noqa: F401 - 兼容旧的 import 路径
    CX,
    CY,
    DEPTH_SCALE,
    FX,
    FY,
    HEIGHT,
    SCENE_OBJECTS,
    WIDTH,
    SyntheticCameraMixin,
    render_scene,
)

__all__ = [
    "OfflineSceneAdapter",
    "render_scene",
    "SyntheticCameraMixin",
    "WIDTH",
    "HEIGHT",
    "FX",
    "FY",
    "CX",
    "CY",
    "DEPTH_SCALE",
    "SCENE_OBJECTS",
]


class OfflineSceneAdapter(SyntheticCameraMixin):
    """只有相机能力的适配器；``mode`` 显式标成 mock，禁止冒充真机。"""

    def __init__(
        self,
        *,
        fail_rgb: bool = False,
        fail_depth: bool = False,
        depth_as_png: bool = False,
    ) -> None:
        self.fail_rgb = bool(fail_rgb)
        self.fail_depth = bool(fail_depth)
        self.depth_as_png = bool(depth_as_png)

    mode = "mock"
