# Motion and navigation adapter handoff

This document covers the execution-side modules in stages A–E. The existing
perception and Agent files are intentionally unchanged.

## Local mock usage

Run the end-to-end demo from the repository root:

```powershell
py -3.10 -m demos.mock_robot_demo
```

Run all unit tests:

```powershell
py -3.10 -m unittest discover -s tests -v
```

The demo processes a simulated instruction, reads mock state, executes the
allowlisted `wave_right` gesture, queries its task ID to completion, navigates
to the allowlisted `demo_table`, and queries that task to completion. Every
result is a `contracts.ToolResult` with `mode="mock"`.

## Calling the execution tools

Construct one adapter and one shared safety controller, then pass that
controller to both tools so motion conflicts and task IDs are tracked together:

```python
from adapters import MockAdapter
from robot_tools.motion import MotionTool
from robot_tools.navigation import NavigationTool
from robot_tools.safety import RobotSafety

adapter = MockAdapter()
safety = RobotSafety(adapter)
motion = MotionTool(adapter, safety=safety)
navigation = NavigationTool(adapter, safety=safety)

gesture = motion.execute_gesture("wave_right")
if gesture.status.value == "running":
    result = navigation.get_task_status(gesture.task_id)
```

`MotionTool.execute_gesture(gesture_id)` exposes only registered gestures;
arbitrary joint angles, velocities, and torques are not accepted.
`NavigationTool` provides `navigate_to_place(place_id)`,
`get_task_status(task_id)`, and `cancel_task(task_id)`. A task reported as
`running` always includes a queryable task ID. Calls that share a
`RobotSafety` instance reject conflicting motion. Task timeouts request
cancellation; without confirmed cancellation, the result is `unknown`, not
`succeeded` or stopped.
The timeout is checked when task status is polled; a blocking adapter/SDK call
cannot be interrupted by this synchronous Python layer and must be bounded by
the future adapter implementation.

`RobotSafety` denies Real mode unless trusted application code explicitly
enables it with `real_authorized=True`. The Real Adapter has a separate,
default-off `enable_real_control` gate; neither flag enables the unimplemented
SDK calls. This software policy is not a substitute for hardware emergency
stops, drive limits, or controller protections.

## Real SDK status and unverified API

`adapters/galbot_adapter.py` is intentionally a fail-closed placeholder:
constructing it does not import GalbotSDK, initialize a robot, or connect to
hardware. Calls are rejected by default. Even if real control is explicitly
enabled and authorized, the placeholder raises `NotImplementedError`.
There is no automatic Mock fallback.

The repository's SDK guide documents **GalbotSDK 1.10.0**, not 1.7.0. The
checked-in `src/main.cpp` shows a C++ initialization and joint-state probe, but
does not establish the corresponding Python SDK surface. Consequently, the
following GalbotSDK 1.7.0 details remain unverified and must be checked against
that exact version's installed source, type stubs, and official examples before
implementing real control:

- Python package/module names, supported G1 classes, and robot construction.
- Initialization signature, required configuration, readiness checks, and
  shutdown/resource-release sequence.
- Safe gesture/action catalog and the official action-execution API.
- Navigation interface, place/goal representation, task ID format, and
  terminal/running state mapping.
- Task-status query, cancellation semantics, and timeout/error behavior.
- Robot-state fields needed to confirm idle state and detect conflicting tasks.

Until those checks are complete, no real SDK methods or arguments are assumed.
