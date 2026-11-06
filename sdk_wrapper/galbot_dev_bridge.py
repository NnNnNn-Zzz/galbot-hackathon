"""Use the unmodified galbot_dev tool layer with this process's SDK singletons.

The original project lives in third_party/galbot_dev-master. Its Real adapter
owns separate GalbotRobot/GalbotNavigation instances, so callers in this
repository must create it through this bridge instead.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

from sdk_wrapper import wrapper

SOURCE_ROOT = Path(__file__).resolve().parent.parent / "third_party" / "galbot_dev-master"


def _load_source():
    """Expose the original project's absolute imports without editing its files."""
    path = str(SOURCE_ROOT)
    if not SOURCE_ROOT.is_dir():
        raise ImportError(f"galbot_dev source is missing: {SOURCE_ROOT}")
    if path not in sys.path:
        sys.path.insert(0, path)
    from adapters.galbot_adapter import GalbotAdapter
    from adapters.mock_adapter import MockAdapter
    from robot_tools.audio import AudioTool
    from robot_tools.motion import MotionTool
    from robot_tools.navigation import NavigationTool
    from robot_tools.perception import PerceptionTool
    from robot_tools.safety import RobotSafety
    return GalbotAdapter, MockAdapter, AudioTool, MotionTool, NavigationTool, PerceptionTool, RobotSafety


def create_toolkit(*, mock: bool = True, real_authorized: bool = False):
    """Return adapter, safety, motion, navigation, perception and audio tools.

    Real mode uses the existing sdk_wrapper instances and must be explicitly
    authorized. All tools share one safety controller. No SDK is initialized
    merely by constructing the toolkit.
    """
    (GalbotAdapter, MockAdapter, AudioTool, MotionTool, NavigationTool,
     PerceptionTool, RobotSafety) = _load_source()

    if mock:
        adapter = MockAdapter()
    else:
        class SharedGalbotAdapter(GalbotAdapter):
            def _import_sdk(self):
                if self._sdk is None:
                    wrapper._sdk_bootstrap()
                    import galbot_sdk.g1 as gm
                    self._sdk = gm
                return self._sdk

            def _ensure_interfaces(self):
                with self._lock:
                    if self._robot is None:
                        # The source adapter's own constructors would create
                        # duplicate robot/nav instances in the Agent process.
                        self._motion = wrapper.get_motion()
                        self._navigation = wrapper.get_nav()
                        self._robot = wrapper.get_robot()
                    return self._robot, self._motion, self._navigation

            def close(self):
                # The Agent alone owns the SDK shutdown sequence.
                with self._lock:
                    self._robot = self._motion = self._navigation = None

        adapter = SharedGalbotAdapter(
            enable_real_control=real_authorized,
            real_authorized=real_authorized,
        )

    safety = RobotSafety(adapter, real_authorized=real_authorized)
    return SimpleNamespace(
        adapter=adapter,
        safety=safety,
        motion=MotionTool(adapter, safety=safety),
        navigation=NavigationTool(adapter, safety=safety),
        perception=PerceptionTool(adapter),
        audio=AudioTool(adapter),
    )
