# 相机订阅修复方案（待确认后执行）

> 这份文档只描述改动，**代码尚未写进 `sdk_wrapper/wrapper.py`**。
> 按你的规矩：动 `sdk_wrapper/` 这件事要先问过你。
> 你说「照这个改」我就动手；你说「先不改」我就只留文档。

---

## 一、根因（有官方证据，不是猜的）

### 现状代码

`sdk_wrapper/wrapper.py:115-137` 的 `_get_robot()`：

```python
def _get_robot():
    global _robot
    if _robot is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockRobot
            _robot = MockRobot()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotRobot
            _robot = GalbotRobot()
        _robot.init()          # <-- 问题在这里：没有传 enable_sensor_set
        ...
```

### 官方签名（`galbot_sdk.ppy` 第 2772 行，SDK 1.10.0）

```python
def init(
    self,
    enable_sensor_set: collections.abc.Set[SensorType] = ...,
    enable_sync_mode: bool = False,
) -> bool:
    """
    Only sensors in enable_sensor_set are initialized.
    ...
    enable_sensor_set (set[SensorType]): Set of sensors to enable.
                                         Empty set uses default sensors.
    """
```

### 官方示例的实际写法（`examples/g1/python/tutorials/example6_pick_and_place.py:556-562`）

```python
# Enable only the RGB and depth cameras used for target detection.
enable_sensor_set = {
    SensorType.LEFT_ARM_CAMERA,
    SensorType.LEFT_ARM_DEPTH_CAMERA,
}
if robot.init(enable_sensor_set):
```

`examples/g1/python/galbot_robot/get_camera_data.py` 同样是：

```python
enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, SensorType.LEFT_ARM_DEPTH_CAMERA, ...}
robot.init(enable_sensor_set)
print("Initialization succeeded")
time.sleep(5)                    # <-- 官方固定预热 5 秒
rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA, RgbOutputFormat.JPEG, True)
```

### 结论

1. **`init()` 不传集合时，默认集合里未必有你想要的那路相机。** 官方所有相机相关示例都是显式点名所需相机的，一个都不例外。文档「Empty set uses default sensors」只说「会用默认的」，没说「默认里有头部左目」。
2. **少了 `sleep(5)` 预热。** 官方 `get_camera_data.py` 在 init 成功后固定睡 5 秒才取图。我们的 `capture_image` 是 init 后立刻取，这个时间差完全可能让首帧拿不到。
3. 这两点叠加，表现就是：`get_rgb_data` 返回 `None` 或空 dict -> 我们的代码判 `"data" not in frame` -> 报「相机不可用或未订阅」。

---

## 二、要改的三处

### 改动 1：`_get_robot()` 显式订阅相机

```python
def _get_robot():
    """获取全局机器人单例。mock 模式下返回 MockRobot。"""
    global _robot
    if _robot is None:
        if _mock_mode:
            from sdk_wrapper.mock_robot import MockRobot
            _robot = MockRobot()
            _robot.init()
        else:
            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotRobot, SensorType
            _robot = GalbotRobot()
            # 显式订阅所需传感器：官方示例一律这样做，不依赖默认集合。
            # 头部左目用于识别类策略取图；手臂相机留给后续 VLM/OCR 扩展。
            sensors = {SensorType.HEAD_LEFT_CAMERA}
            ok = _robot.init(sensors)
            if not ok:
                logger.error("GalbotRobot.init(enable_sensor_set=%s) 返回 False", sensors)
            # 相机数据就绪需要时间：官方 get_camera_data.py 固定 sleep(5)
            time.sleep(5.0)
            # 底盘运动需要先切到 CHASSIS_POSE_CTRL
            from galbot_sdk.g1 import ControlStatus, G1ControllerName
            res = _robot.switch_controller(G1ControllerName.CHASSIS_POSE_CTRL)
            ...
        logger.info("机器人实例已初始化 (mock=%s)", _mock_mode)
    return _robot
```

关键点：
- `SensorType` 在 `from galbot_sdk.g1 import ...` 里一起导入
- **只订头部左目**，不是全订 —— 官方文档明说「specify only required sensors to reduce overhead」，全订会拖慢 init（当前无真机 init 已要 127 秒）
- `sleep(5.0)` 加在 init 之后、切控制器之前

### 改动 2：`capture_image()` 加一次重试

```python
def capture_image(tag: str = "", retries: int = 1) -> dict:
    if _mock_mode:
        return _mock_ok("capture_image", frame_id=f"mock_frame_{tag or '0'}")
    _sdk_bootstrap()
    import galbot_sdk.g1 as gm
    robot = _get_robot()

    frame = None
    for attempt in range(retries + 1):
        frame = robot.get_rgb_data(gm.SensorType.HEAD_LEFT_CAMERA,
                                   gm.RgbOutputFormat.JPEG, True)
        if frame and "data" in frame:
            break
        if attempt < retries:
            logger.warning("取图第 %d 次为空，等 1s 重试", attempt + 1)
            time.sleep(1.0)

    if not frame or "data" not in frame:
        return {"success": False, "completion": "failed",
                "message": "get_rgb_data 连续返回空（相机不可用或未订阅）"}
    ...
```

为什么要重试：`get_rgb_data(..., once=True)` 是阻塞等**新帧**。若相机刚起来还没出帧，第一次就会空。给一次 1 秒后的重试能救掉大部分偶发。

### 改动 3：`init()` 的 127 秒问题（可选）

现在的 `_robot.init()` 在无真机时要等约 **127 秒**超时。加了 `sleep(5)` 会变成约 132 秒。如果你觉得太慢，可以考虑设环境变量 `MED_SKIP_SENSOR_PREHEAT=1` 跳过预热——但**不建议**，因为上真机后这 5 秒是必要的。

---

## 三、改完之后怎么验证

**不要直接跑实验 2/3。** 先跑探针：

```bash
# 在 HPU 上
source scripts/env_hpu.sh
python3 scripts/probe_camera.py
```

`probe_camera.py` 应该输出类似：

```
[1] init() 不传集合 -> 检查默认能出哪些相机
  HEAD_LEFT_CAMERA: 空 / 有数据(12345 bytes)
[2] init({HEAD_LEFT_CAMERA}) -> 检查显式订阅
  HEAD_LEFT_CAMERA: 有数据(12345 bytes)
```

**判读方式：**

| 现象 | 含义 | 怎么办 |
|---|---|---|
| 两种都出图 | 根因不是订阅，是别的问题（可能是 `sleep` 不够） | 只加 `sleep(5)`，不显式订阅 |
| 只有 [2] 出图 | **根因确认**：默认集合不含头部相机 | 按本方案改 |
| 两种都空 | 相机硬件/驱动问题 | 别改代码，报给赛事对接人 |
| init 本身返回 False | SDK/网络问题 | 先解决 init，相机是次要的 |

---

## 四、风险与影响面

| 项 | 评估 |
|---|---|
| 影响哪些场景 | 只影响 `capture_image`。实验 1/2 是盲抓，**不依赖相机**，改了也不影响它们 |
| 会不会拖慢启动 | 会，+5 秒。但这是官方要求的预热 |
| 会不会影响队友 | `sdk_wrapper/` 是共享封装层，改动会影响所有场景。**改前要跟队友说一声** |
| 回滚成本 | 低。三处改动都很局部，`git revert` 即可 |
| mock 模式 | 不受影响，`_mock_mode` 分支不变 |

---

## 五、我的建议

**建议先只跑 `probe_camera.py`，拿到实测结果再决定改不改。**

理由：现在「默认集合不含 HEAD_LEFT_CAMERA」只是**强推断**，不是实测结论。虽然官方示例全都显式传集合，但文档确实写了「Empty set uses default sensors」。万一默认集合里有头部相机，那根因就变成 `sleep` 不够，改法完全不同（只加 sleep，不动订阅）。

探针跑一次 2 分钟，能省掉一次错误的改动。
