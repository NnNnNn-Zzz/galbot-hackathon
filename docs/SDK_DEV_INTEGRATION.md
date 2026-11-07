# galbot_dev SDK 并入说明

来源仓库：https://github.com/JingyiOuyang/galbot_dev @ `167b147`（feat: 优化合作代码，添加推杯子场景赛前准备）

## 这是什么

galbot_dev 的 SDK 封装栈已**通过全部离线测试**（21 单元测试 + 3 个场景 demo + 感知全分支），
以「并存分层」方式并入本项目，位置：`sdk_wrapper/dev/`。

```
sdk_wrapper/dev/
  contracts.py        # ToolResult 契约：status/mode/message/task_id/data/error_code 六键
  frame_store.py      # 抓帧登记（capture_id / rgb_ref / depth_ref）
  adapters/           # RobotAdapter 抽象 + MockAdapter（无硬件）+ GalbotAdapter（真机，懒加载 SDK）
  robot_tools/        # 暴露给大模型的工具层：motion / navigation / safety / perception / audio / cup_scenario
  algorithms/         # 颜色检测 / 几何 / 场景分析 / 桌面单应 / 地名解析
  config/             # 白名单：cameras.json / places.json / scenarios.json / table_calib.json
  assets/audio/       # 语音 PCM 素材（赛前合成，缺失时文本兜底）
  tests/              # 21 个单元测试（unittest，无 pytest 依赖）
  docs/               # 原仓库两份指南
```

## 与现有 sdk_wrapper 的关系（并存，不互相替代）

| | 现有 `sdk_wrapper`（wrapper.py） | 新并入 `sdk_wrapper.dev` |
|---|---|---|
| 能力 | 底盘移动：move_forward/backward/turn/stop_base/read_sensor | 任务生命周期（task_id/超时/取消）、导航与动作白名单、末端笛卡尔原语（推/夹爪/位姿）、感知（RGB-D/反投影/桌面）、语音 |
| 单位 | **厘米、度** | **米、弧度、秒** |
| 失败语义 | 超限静默截断 | 显式 ToolResult + error_code，unknown ≠ 已停止 |
| 使用方 | 老场景（scene_patrol / scene_greet） | 新场景优先用 |

⚠️ **两套单位制不同，混用前必须换算**（1 m = 100 cm；π rad = 180°）。
原样保留 galbot_dev 的四层结构，仅把扁平导入改写为 `sdk_wrapper.dev.*` 包导入，逻辑零改动。

## 快速上手

```python
from sdk_wrapper.dev import MockAdapter, MotionTool, NavigationTool, RobotSafety, TaskStatus

adapter = MockAdapter()                  # 真机换 GalbotAdapter() + real_authorized=True
safety = RobotSafety(adapter)
motion, nav = MotionTool(adapter, safety=safety), NavigationTool(adapter, safety=safety)

task = motion.execute_gesture("wave_right")     # 白名单动作，返回 running + task_id
print(nav.get_task_status(task.task_id).status) # 轮询直到 succeeded / failed
```

感知（Mock 与真机走同一套解码代码）：

```python
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool
p = PerceptionTool(adapter)          # 默认读 sdk_wrapper/dev/config/cameras.json
print(p.analyze_scene().to_dict())   # 结构化场景：目标 + 像素 + 3D
print(p.find_object("red").to_dict())
```

头部双目深度（真机 G1 SDK `FOUNDATION_STEREO`，只读）：

```python
from sdk_wrapper.dev import StereoDepthTool

result = StereoDepthTool().capture_stereo_depth(timeout_s=8.0)
print(result.to_dict())
# 成功时：np.load(result.data["depth_ref"], allow_pickle=False) 读取原始二维数组
```

工具名为 `capture_stereo_depth`，返回通用六字段 `ToolResult`。`depth_ref` 是保存在
`sdk_wrapper/dev/runs/stereo_depth/` 下的原始 `.npy`，`preview_ref` 是伪彩色预览；
结果还包含尺寸、传感器名、时间戳、有效像素比例和原始值范围。SDK 将图放在
`DetectionResult.instance_mask`，但未说明该值的物理单位，因此 `unit` 固定为
`sdk_raw_unverified`，不可直接当米使用，也不与臂部深度相机的米制数据混用。
首次调用会初始化模型并等待约 10 秒，后续调用仅运行单次推理。

G1 腿部三关节逐键控制（`leg_joint1/2/3`，每条命令只动一个关节）：

```bash
source scripts/env_hpu.sh
python3 scripts/keyboard_leg_joints.py
```

运行后先记录三个关节的会话起始值，单关节相对起始偏移限 ±0.05 rad；`j/k`
每次 0.005 rad，`J/K` 每次 0.01 rad。脚本逐键选择关节和方向，不允许一次
命令多个关节，并显示实际关节变化、身高变化、躯干姿态变化及未命令腿关节漂移。
这三条关节属于高度协同机构，单动会改变支撑和身高，必须由现场人员监督。

推杯子闭环（判不安全 → 语音 → 小步推 → 回位复测）：

```python
from sdk_wrapper.dev.robot_tools.cup_scenario import CupScenario
from sdk_wrapper.dev.robot_tools.audio import AudioTool
scenario = CupScenario(PerceptionTool(adapter), AudioTool(adapter), motion)
print(scenario.run_cup_scenario().message)
```

## 验证命令

```bash
cd ~/galbot_hackathon
python3 -m unittest discover -s sdk_wrapper/dev/tests -v   # 21 个单元测试
python3 agent/main.py --mock --max-loops 3                 # 现有架构回归
```

## 真机注意事项

- `GalbotAdapter` 懒加载 `galbot_sdk`；real 模式必须 `RobotSafety(adapter, real_authorized=True)` 显式授权，否则一律 `rejected + permission_denied`。
- `config/places.json` 中 `pose=null` 或 `verified=false` 的目标点，真机导航会被拒绝（Mock 不受影响）——**先在真机实测填位姿**。
- 语音素材 `assets/audio/*.pcm` 需赛前用 Windows `System.Speech` 合成（命令见 `robot_tools/audio.py` 文件头注释）；缺失时 `speak()` 自动文本兜底，不会静默失败。
- 感知层依赖 opencv（已在 pyproject.toml：opencv-python>=4.8）。galbot_dev 建议 `<5`（真机 SDK 自带 4.x）；本机 WSL 为 cv2 5.0.0，全部测试已通过，但真机对齐时以真机 4.x 为准。
- 原仓库 `config/gestures.json` 未提交，动作白名单回落到内置兜底 `{wave_right}`；要加动作请新建该文件（格式见 adapters/base.py `_load_registered_ids`）。
