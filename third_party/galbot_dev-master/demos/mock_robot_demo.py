"""Run a hardware-free end-to-end motion and navigation demonstration."""

from __future__ import annotations

import json

from adapters import MockAdapter
from contracts import TaskStatus
from robot_tools.motion import MotionTool
from robot_tools.navigation import NavigationTool
from robot_tools.safety import RobotSafety


def main() -> int:
    adapter = MockAdapter(task_polls_until_succeeded=0)
    safety = RobotSafety(adapter)
    motion = MotionTool(adapter, safety=safety)
    navigation = NavigationTool(adapter, safety=safety)

    print("用户指令（模拟）：执行 wave_right，然后前往 demo_table。")
    print("机器人状态：")
    print(json.dumps(adapter.get_robot_state().to_dict(), ensure_ascii=False, indent=2))

    gesture = motion.execute_gesture("wave_right")
    print("动作 Tool：")
    print(json.dumps(gesture.to_dict(), ensure_ascii=False, indent=2))
    if gesture.status is not TaskStatus.RUNNING or gesture.task_id is None:
        return 1

    gesture_status = navigation.get_task_status(gesture.task_id)
    print("动作任务查询：")
    print(json.dumps(gesture_status.to_dict(), ensure_ascii=False, indent=2))
    if gesture_status.status is not TaskStatus.SUCCEEDED:
        return 1

    destination = navigation.navigate_to_place("demo_table")
    print("导航 Tool：")
    print(json.dumps(destination.to_dict(), ensure_ascii=False, indent=2))
    if destination.status is not TaskStatus.RUNNING or destination.task_id is None:
        return 1

    destination_status = navigation.get_task_status(destination.task_id)
    print("导航任务查询：")
    print(json.dumps(destination_status.to_dict(), ensure_ascii=False, indent=2))
    return 0 if destination_status.status is TaskStatus.SUCCEEDED else 1


if __name__ == "__main__":
    raise SystemExit(main())
