"""共享 Agent 适配器：把 drawer_demo 的 20 步流程接到 sdk_wrapper 上。

用法（在 galbot-hackathon 仓库根目录）：

    python -m drawer_agent.cli caps
    python -m drawer_agent.cli run --backend offline
    python -m drawer_agent.cli run --backend offline --fault short_yaw
    python -m drawer_agent.cli selftest
    python -m drawer_agent.cli routes --recipe ... --routes ...

真机后端需要 `sdk_wrapper` 补齐 `drawer_agent/shared_link.py::NEEDED` 里的方法；
缺失项由 `caps` 直接列出，不会带着缺口下发动作。
"""

from .io import ActionFailed, SharedAgentIO
from .link import NEEDED_HINT, REQUIRED, LinkError, MissingCapability
from .perception import SCHEMA as SEG_SCHEMA, YoloHttpClient
from .routes import RouteStore
from .runner import DEFAULT_RECIPE, DEFAULT_ROUTES, FAULTS, build, run_flow
from .shared_link import NEEDED, SharedAgentLink
from .sim_link import OfflinePerception, OfflineRig

__all__ = [
    "SharedAgentIO", "ActionFailed", "LinkError", "MissingCapability",
    "SharedAgentLink", "OfflineRig", "OfflinePerception",
    "RouteStore", "YoloHttpClient", "SEG_SCHEMA", "REQUIRED", "NEEDED", "NEEDED_HINT",
    "build", "run_flow", "FAULTS", "DEFAULT_RECIPE", "DEFAULT_ROUTES",
]
