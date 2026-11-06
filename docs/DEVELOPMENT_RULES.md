# G1 黑客松合作开发规范（DEVELOPMENT RULES）

> 所有人开工前必须读完本文档并在群里确认。违反红线造成的损失（撞机、环境崩坏）由责任人承担。

## 〇、四条核心纪律（最重要）

1. **场景开发者可以自由组装 SDK、包装成自己想要的封装**：在自己文件夹里 `import galbot_sdk` 或组合 `sdk_wrapper` 均可，接口形态自己定，无需 A 审批。硬约束只有三条：①机器人/导航实例必须从 `sdk_wrapper.get_robot()` / `get_nav()` 拿共享单例，**禁止自己 `GalbotRobot()` init，禁止 destroy/shutdown**；②不得写 `safety_config.json`；③归位纪律不变。`sdk_wrapper/` 目录本身与统一依赖仍由负责人 A 维护。
2. **场景必须归位。** 场景返回后，机器人必须处于基线状态（底盘停、手臂折叠、夹爪释放）。
3. **安全参数不可被场景运行时覆盖。** `safety_config.json` 只能人工改。
4. **所有场景与 Agent 共享同一个 Python 进程和环境。** 禁止场景内 `uv init`、禁止起子进程跑机器人控制。

## 一、为什么不能拆独立进程/环境

Galbot SDK 的控制器、相机订阅、急停监听是**机器人级单例资源**，不能随场景子进程退出而安全释放。场景隔离的是**代码文件夹和负责人**，不是运行时环境。

如果某个场景的算法依赖确实与主环境冲突（如模型库与 SDK 的 numpy ABI 不兼容），**不要拆场景环境**，而是把算法隔离为独立**感知服务进程**（本地 socket / HTTP），只传结构化结果，机器人控制链路始终留在 Agent 主进程。

## 二、主进程基线服务：不可碰清单

比赛开始后，负责人 A 启动的基线服务维持机器人基本运行。所有场景脚本**严禁**干扰：

| 不可碰对象 | 作用 | 禁止行为 |
|---|---|---|
| `galbot_chassis_service` | 底盘控制服务 | 不能 kill、重启、绕过封装层直发原始指令 |
| `singorix` 进程 | 运动规划与控制核心 | 不能停止、不能修改其配置 |
| `GalbotRobot` 全局单例 | SDK 机器人实例 | 场景不能重复 `init()` 或 `destroy()` |
| 急停监听线程 | 监控急停按钮 | 不能屏蔽、不能覆盖回调 |
| Agent 主循环进程 | 调度器本身 | 场景不能 `sys.exit()`、不能 `os.kill` 父进程 |

场景执行前后 Agent 会检查上述服务存活。异常则标记 `unsafe_exit` 并写入 task_log。

## 三、SDK 封装层规范

- `sdk_wrapper` 提供带安全限幅的高层接口（`move_forward` / `turn` / `read_sensor` / ...），**推荐优先使用**。
- **场景开发者可以自己组装 SDK 包装成自己想要的封装**：在自己文件夹里随便 `import galbot_sdk` 原生接口或组合 `sdk_wrapper`，函数名、参数、返回格式全部自己定，不需要向 A 申请。
- 直接用 SDK 原生接口时的三条硬约束：
  1. 机器人/导航实例必须用 `sdk_wrapper.get_robot()` / `sdk_wrapper.get_nav()` 拿共享单例，**禁止自己 `GalbotRobot()` init，禁止 destroy/shutdown**（会打死主进程单例和急停监听）。
  2. 若切换了控制器（`switch_controller`），场景 `finally` 归位前必须停运动、恢复底盘控制。
  3. `safety_config.json` 不得写入。注意：自动限幅截断只发生在 `sdk_wrapper` 高层函数里，直接调 SDK 原生接口**没有自动限幅**，请自觉遵守限幅量级（速度 ≤0.3 m/s、单次距离 ≤2m、转向 ≤180°）。
- `safety_config.json` 仅负责人 A 可改；比赛期间临时放宽须在群里说明原因和时间，赛后恢复。
- 需要修改/新增 `sdk_wrapper` 底层封装函数（目录本身仍仅 A 改）→ 群里发："场景 X 需要 `函数名(参数)`，返回 `...`"，A 确认后实施。

## 四、场景脚本规范

### 4.1 入口约定

```python
def run(params: dict) -> dict:
    ...
    return {
        "success": True / False,
        "feedback": "一句话描述结果（Agent 会拿去做下一步决策）",
        "suggested_adjustment": None  # 失败时给 Agent 的调整建议
    }
```

### 4.2 归位规范（硬性）

> 每个场景脚本在执行结束前（**无论成功或异常**），必须确保机器人回到基线状态：底盘停止运动、手臂处于折叠安全位、夹爪释放。
>
> 实现方式自行决定：可以在 `run()` 末尾调用 `sdk_wrapper.baseline.reset_to_baseline()`，推荐放在 `finally` 块里。关键要求：**场景返回后，机器人不能处于遗留的运动状态或异常姿态。**
>
> 如果异常导致提前退出，归位逻辑同样必须执行。必须先用 mock 封装层（`GALBOT_MOCK=1`）测试正常路径和异常路径。

### 4.3 注册

在 `registry/scenes.yaml` **只追加自己的条目**（name / path / when_to_call / expected_feedback / recovery_on_failure / expected_exit_state）。这份注册表会原样进顶层 Agent 的 prompt，写清楚 Agent 才会正确调用。

### 4.4 场景间零调用

场景之间禁止互相 import、禁止共享全局状态。需要协作时通过 Agent 调度或 task_log 传递信息。

### 4.5 场景内自定义封装（可选但推荐）

- 建议把场景专用的接口函数收在自己文件夹的单独模块里（如 `api.py` / `actions.py`），`scene.py` 只留编排逻辑。
- `import galbot_sdk` 和 `import sdk_wrapper` 都可以，实例从 `sdk_wrapper.get_robot()` / `get_nav()` 拿（硬约束见第三节）；命名建议语义化动词（如 `wave_hand` / `go_to_charger`），避免与底层 `move_forward` 等撞名造成阅读混淆。
- 直接调 SDK 原生接口的代码在 mock 模式下不可测（MockRobot 只实现最小接口），建议自定义封装里对 mock 做兼容或标注"仅真机"。
- 自定义封装**跨场景不可复用**（场景间零调用原则不变）。如果某个封装被多个场景需要 → 告诉 A，由 A 下沉到 `sdk_wrapper` 成为公共函数。

## 五、依赖管理

- **不强制 uv**：环境统一用 WSL 系统 `python3`（Ubuntu 22.04 自带 3.10），依赖直接 `pip` 安装。
- 所有依赖记录在根目录 `pyproject.toml`（作为清单），由负责人 A 执行 `python3 -m pip install <pkg>` 安装并同步更新清单。
- 场景文件夹**禁止**出现 `pyproject.toml` / `.venv` / `requirements.txt`（避免各装各的）。
- 需要新依赖 → 群里发：“场景 X 需要 `包名==版本`，用途是……”，A 评估后统一安装并提交 pyproject.toml。
- 版本冲突时协商统一版本，不拆环境（同进程原则，见第一节）。

## 六、修改权限

| 区域 | 谁可以改 | 规则 |
|---|---|---|
| `pyproject.toml` | 仅负责人 A | 其他人提需求，A 统一 pip install 并更新清单 |
| `sdk_wrapper/` | 仅负责人 A | 提需求，A 评估后统一加 |
| `agent/` | 仅负责人 A | 场景开发者不改调度逻辑 |
| `scenes/自己文件夹/` | 该场景负责人 | 自由修改，可自己组装 SDK 封装（实例从 sdk_wrapper 单例拿） |
| `registry/scenes.yaml` | 该场景负责人 | 只追加自己的条目 |
| `docs/` | 任何人 | 修改后群里通知 |

## 七、补充建议（比赛实战）

1. **场景切换前轻量状态检查**：Agent 会对比上一场景的 `expected_exit_state`，偏差大先归位再进入。
2. **归位验证"记录不阻塞"**：场景返回后状态异常只写 task_log 标记，不强制拦截，避免误杀正常场景。
3. **安全参数变更留痕**：改 `safety_config.json` 必须群里说明。

## 八、赛前检查清单

- [ ] SDK v1.10.0 已克隆到 WSL，机器人环境版本确认为 V1.18
- [ ] `install_python_deps.sh` 已执行
- [ ] 系统 python3 已装 `pyproject.toml` 清单里的依赖（`python3 -m pip install pyyaml numpy opencv-python`）
- [ ] 封装层用 mock 测试通过
- [ ] Agent 调度器 mock 模式跑通完整流程
- [ ] 每个场景负责人用 mock 测试自己的 `run()` 正常路径 + 异常路径 + 归位
- [ ] 真机到手后：`move_forward(50)` 卷尺验证误差 ±5cm；`reset_to_baseline()` 验证归位
