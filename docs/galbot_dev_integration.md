# galbot_dev 工具层接入

`third_party/galbot_dev-master/` 是 `~/Downloads/galbot_dev-master` 的原样源码快照：
45 个源码、配置、文档、测试和示例资源文件逐字节一致。仅排除了 Python 缓存
（`__pycache__`、`.pyc`）和个人 IDE 配置（`.idea`）。原始 `agent/`、`adapters/`、
`robot_tools/`、`algorithms/`、`config/`、`tests/`、`demos/`、`docs/`、`assets/`、
`runs/`、`scripts/`、`src/` 均保留，没有修改原文件。

这套工具提供相机感知、末端位姿与运动、夹爪、预录 PCM 音频、登记地点导航、
任务状态与安全调度；其预设挥手动作和示例导航地点仍需真机示教或标定，
原实现会拒绝未标定的动作和地点。

## 在本仓库使用

```python
from sdk_wrapper.galbot_dev_bridge import create_toolkit

tools = create_toolkit(mock=True)
print(tools.perception.list_cameras().to_dict())
print(tools.motion.execute_gesture("wave_right").to_dict())
```

真机进程中由可信的调用方显式创建：

```python
tools = create_toolkit(mock=False, real_authorized=True)
```

桥接层让参考项目的 `GalbotAdapter` 复用 `sdk_wrapper.get_robot()`、
`get_nav()` 和 `get_motion()`；关闭 adapter 不会销毁 Agent 持有的 SDK 实例。
同一个 toolkit 的运动与导航工具共享 `RobotSafety`。构建 toolkit 不会立即
初始化 SDK。不要在当前 Agent 进程里直接实例化参考项目原版 `GalbotAdapter`，
它会自行创建和销毁机器人实例。

参考项目保留了自己的 `agent/`，尚未替换当前仓库的场景调度器；新能力可通过
`create_toolkit()` 在场景或后续 Agent 接入代码中调用。真机接口尚未在设备上验证。

## 验证

```bash
python3 -m unittest discover -s tests -v
PYTHONPATH=third_party/galbot_dev-master python3 -m unittest discover -s third_party/galbot_dev-master/tests -v
```
