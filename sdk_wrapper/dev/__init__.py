"""sdk_wrapper/dev —— 并入的 galbot_dev SDK 栈（并存分层）。

来源：https://github.com/JingyiOuyang/galbot_dev @ 167b147
并入方式：原样保留其「contracts + adapters + robot_tools + algorithms」四层，
仅把扁平导入改写为 sdk_wrapper.dev.* 包导入，逻辑零改动。

与现有封装的关系（并存，不互相替代）：
  * sdk_wrapper.wrapper：底盘移动原语（move_forward/turn/stop_base/read_sensor），
    单位是**厘米/度**，超限静默截断 —— 老场景继续用，不要动。
  * sdk_wrapper.dev：ToolResult 契约 + 任务生命周期（task_id/超时/取消）+
    导航与动作白名单 + 末端笛卡尔原语（push/gripper）+ 感知层 + 语音，
    单位是**米/弧度/秒**，失败显式返回 error_code —— 新场景优先用。

⚠️ 两套单位制不同，混用前先换算（1 m = 100 cm；π rad = 180°）。

真机注意：
  * adapters.galbot_adapter 懒加载 galbot_sdk，real 模式必须
    RobotSafety(adapter, real_authorized=True) 显式授权，否则一律 rejected；
  * config/places.json 里 pose 为 null 或 verified=false 的目标点，
    真机导航会被拒绝（Mock 不受影响）；
  * 语音素材 assets/audio/*.pcm 需赛前合成，缺失时 speak() 走文本兜底。

详见 docs/ 下两份指南（motion_navigation_adapter.md / perception_sdk_guide.md）。
"""

from sdk_wrapper.dev.contracts import (  # noqa: F401
    ErrorCode,
    RunMode,
    TaskStatus,
    ToolResult,
    ToolError,
    to_json,
)
from sdk_wrapper.dev.adapters import GalbotAdapter, MockAdapter, RobotAdapter  # noqa: F401
from sdk_wrapper.dev.robot_tools.motion import MotionTool  # noqa: F401
from sdk_wrapper.dev.robot_tools.navigation import NavigationTool  # noqa: F401
from sdk_wrapper.dev.robot_tools.safety import RobotSafety  # noqa: F401
from sdk_wrapper.dev.robot_tools.stereo_depth import StereoDepthTool  # noqa: F401
from sdk_wrapper.dev.robot_tools.squat import SquatTool  # noqa: F401
from sdk_wrapper.dev.robot_tools.torso_lean import TorsoLeanTool  # noqa: F401
from sdk_wrapper.dev.robot_tools.head import HeadLookTool  # noqa: F401

__all__ = [
    "ErrorCode",
    "RunMode",
    "TaskStatus",
    "ToolResult",
    "ToolError",
    "to_json",
    "RobotAdapter",
    "MockAdapter",
    "GalbotAdapter",
    "MotionTool",
    "NavigationTool",
    "RobotSafety",
    "StereoDepthTool",
    "SquatTool",
    "TorsoLeanTool",
    "HeadLookTool",
]
