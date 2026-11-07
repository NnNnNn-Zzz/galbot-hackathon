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

G1 直立下蹲（使用官方 `GalbotRobot.set_leg_height()`，逐步且需要现场确认）：

```bash
source scripts/env_hpu.sh
python3 scripts/supervised_squat.py
```

SDK 定义的 `head_base_link` 高度范围为 0.96–1.54 m，封装每步最多下降/上升
2 cm，并要求操作者对每一步逐项确认。0.96 m 是 SDK 支持下限，不等同于真机
验证过的机械极限或平衡极限。全程用 `get_transform("base_link", "head_base_link")`
读回实际高度；完成恢复后可按官方示例切回 `leg_pvt_ctrl`。流程不会关闭共享
SDK 单例，真机启动必须在有现场监督者和可触达急停的环境中进行。

G1 前倾关节不能用 `torso` 链的 `leg_joint4/5`：2026-11-07 监督测试确认
`leg_joint4` 产生偏航，`leg_joint5` 动作时头部俯仰/偏航基本不变而滚转变化。
`TorsoLeanTool` 因此继续禁用这两个关节的动作，仅保留只读状态查询。

同日 `leg_joint3 +0.002 rad` 和 `+0.010 rad` 两次受控微步均产生主导俯仰
（头部 pitch 分别约 `+0.119°`、`+0.571°`，X 分别前移约 `1.26 mm`、
`6.00 mm`），说明 `leg_joint3` 是前倾候选。但它属于
`LEG_HEIGHT_CTRL` 使用的前三条腿关节，单动会耦合身高与支撑位置；在实现和
真机验证专用单步工具前，不应假定更大的角度范围或平衡安全边界。

G1 头部键盘控制（只动 `head_joint2` 俯仰）：

```bash
source scripts/env_hpu.sh
python3 scripts/keyboard_head_control.py
```

`HeadLookTool` 每步最多 0.02 rad、速度 0.05 rad/s，测试阶段软件范围暂限
-0.25–0.40 rad；每步复读 `head_joint2`、实际头部 pitch 和未命令的
`head_joint1` 漂移。头部姿态必须从会随头关节运动的
`head_end_effector_mount_link` 读取；`head_base_link` 位于头关节之前，不能用
来判断低头方向。脚本中 `j/k` 以 0.01 rad 低头/抬头，`J/K` 以 0.02 rad
低头/抬头，`r` 逐步回起始，`q` 退出。正负方向会由实际头部末端姿态验证。

G1 专用前倾工具（只动 `leg_joint3`，增加量=前倾）：

```bash
source scripts/env_hpu.sh
python3 scripts/keyboard_forward_lean.py
```

`ForwardLeanTool.lean_forward_step()` 只给 `leg_joint3` 正值目标，
`lean_backward_step()` 只给负值目标；单步最多 0.01 rad、速度 0.03 rad/s。
每步同时验证目标关节变化方向、头部 X 前移、头部 pitch 方向和其它腿关节漂移。
脚本以启动时的 `leg_joint3` 为基准，本次偏移限 ±0.05 rad；`f/F` 前倾
0.005/0.01 rad，`b/B` 回正，`r` 恢复起始，`q` 退出。

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
