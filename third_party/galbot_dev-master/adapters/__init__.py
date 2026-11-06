"""Robot execution adapters.

    base.py            抽象契约（执行 + 原始相机 payload）
    mock_adapter.py    无硬件：状态/任务 + 合成相机
    galbot_adapter.py  真机：透传 GalbotSDK（懒加载，Windows 上 import 不会崩）
    synthetic_scene.py 合成场景，供 mock 与 agent/offline_demo_adapter 共用
"""

from adapters.base import RobotAdapter
from adapters.mock_adapter import MockAdapter

#: GalbotAdapter 故意不在这里 import：它在真机上才会懒加载 galbot_sdk，
#: 但模块本身依赖 contracts，提前 import 没有坏处；放这里只是为了显式。
from adapters.galbot_adapter import GalbotAdapter

__all__ = ["RobotAdapter", "MockAdapter", "GalbotAdapter"]
