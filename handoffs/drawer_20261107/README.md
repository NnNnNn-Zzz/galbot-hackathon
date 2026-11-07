# 抽屉示教交接 · 2026-11-07

本包交接两组已采数据：三个静止位置 P0/P1/P2，以及早先一次拉屉过程。另附只读采集工具、抽屉几何与回放代码、离线测试和失败原因证据。没有把这些数据标成完整连续轨迹或已验收的自动操作流程。

先读 [简短交接](HANDOFF_BRIEF.md)，了解当前进度和下一步。

## 接手先看

1. [数据与质量说明](DATA.md)：三个点各是什么、数据在哪、哪些字段不可直接用于回放。
2. [失败原因与待修问题](FAILURES.md)：最近一次复现被拒绝的明确原因，以及其他失败事件。
3. `medicine_frontend/tools/capture_waypoint.py`：整理后的通用只读采集入口，替代本地逐点复制的临时脚本。
4. `medicine_frontend/data/medicine_dataset/teaching/drawer_pull_20261107_122158/demonstration.json`：夹持、八段拉屉、停止事件、状态与画面索引。
5. `SHA256SUMS.json`：包内文件完整性清单；可运行 `python verify_package.py`。

## 三个位置

| 点 | 含义 | 状态采样 | 数据目录（相对于 `medicine_frontend/data/medicine_dataset/teaching/`） |
|---|---|---:|---|
| P0 | 夹取前准备；用户确认补拍时保持同一位置 | 50 帧 | `grasp_capture_20261107_194545/` |
| P1 | 从夹取前向原始位置退回的第一个关键点 | 20 帧 | `pregrasp_to_initial_20261107_200135_P1/` |
| P2 | 同方向第二个关键点 | 20 帧 | `pregrasp_to_initial_20261107_200135_P2/` |

三个点原采集包含 25 个关节的位置/速度/力矩/设备时间戳、左右夹爪状态、四路 RGB 和两路腕部深度。本次交接按用户要求只保留一张正视参考照片，保留全部数值状态、深度和原相机元数据。用户没有明确把 P2 指认为原始终点，因此终点仍标记为未确认。不要把 P0 误称为原始位置，也不要把新采的三个点直接视为早先拉屉示教的同一坐标基准。

## 使用代码

以下命令在本目录运行。本地主机推荐 Python 3.10+；几何/感知测试需要 numpy、Pillow，SDK 读取子程序使用机器人现有 SDK 1.10。包内保留了原有工作区相对层级，内部 `galbot-hackathon/` 只是场景适配器和配方快照，不是第二个 Git 仓库。

```bash
python verify_package.py
python -m medicine_frontend.tools.capture_waypoint --help
python -m medicine_frontend.tools.replay_drawer_pull plan
python -m medicine_frontend.tools.regrip_and_close_drawer plan
python -m unittest medicine_frontend.test_replay_drawer_pull medicine_frontend.test_regrip_and_close_drawer medicine_frontend.test_drawer_regrip_close
```

下一次采点前，通过进程环境提供 `GALBOT_SSH_TARGET`（已授权 SSH 别名或 user@host）与 `GALBOT_CAM_CODE` / `GALBOT_CAM_TOKEN`，不要把凭据写进源码。SSH 使用主机已有 key/agent 和 known_hosts；本工具不自动接受新主机指纹，不内置密码。相机地址通过 `GALBOT_CAM_BASE` 配置，默认场地旧地址，使用前核实。

```bash
python -m medicine_frontend.tools.capture_waypoint --session return_demo_YYYYMMDD_HHMMSS --point P3
```

先由现场人员移动并停稳，再触发采集。新工具通过 SSH 标准输入执行 SDK 读取，不部署或修改机器人文件，不发送运动/夹爪指令。默认机器人已有运行环境为 `/home/galbot/galbot-sdk-web-console/.venv/bin/python`；环境不同应先核实调整。20 帧采样间隔设为 0.1 秒，实际频率看时间戳。画面与关节并发启动但不硬同步，缺项返回非零退出码；重采请使用新批次，保留之前的部分记录。

连续采集入口 `record_drawer_stream.py`（需要当前 `GALBOT_CONSOLE_CODE`）与 `record_drawer_cameras.py` 仍保留，用于下一次完整动作开始前启动。相机访问码和控制台访问码不是同一个。

## 代码状态

| 类别 | 文件 | 状态 |
|---|---|---|
| 可用采集基础 | `dataset.py`、`tools/collect.py`、`tools/record_drawer_*.py` | 既有实现；保留原始质量标记 |
| 本轮整理 | `tools/capture_waypoint.py` | 复用 P1/P2 已用的 SDK 读取代码；通用参数入口仅离线验证，未再次运行真机 |
| 拉屉参考 | `tools/replay_drawer_pull.py`、场景 `drawer_replay.py` | 分段回放与门控；没有验收为可无人值守运行 |
| 关屉开发参考 | `drawer_regrip_close.py`、`tools/regrip_and_close_drawer.py` | 保留算法、测试和配方；有已知缺陷，见 FAILURES.md |
| 历史依赖 | `tools/close_drawer_second_probe.py`、`tools/pull_drawer_arm_step.py` 等 | 为导入依赖和追溯而保留，含旧会话硬编码；不要直接运行历史 main |

本次仅整理、打包和离线验证，未修复运动控制缺陷、未执行真机动作、未修改现场 SDK。回放前需完成底盘姿态门控、力流独立超时保护和推回位移计算修复，并由现场人员验证。代码默认 plan 模式不运动；不能因为存在 `--execute` 就视为已批准自动执行。

## 本地清理范围

交接包和文件哈希验证通过后，删除明确失败/阻塞会话的大体积本地原始目录、对应数据库索引、逐点临时脚本、缓存和重复导出。失败原因所需 JSON 留在 `evidence/`，清理清单留在 `CLEANUP.json`。成功的三个点、拉屉原始记录、其他同学代码与未提交前端改动保留。

## 本次图像精简与尺寸保留

用户要求只保留一张正视照片：`reference/front_view.jpg`（原始光照，未修图，仅作形状参考，不作颜色或尺度标定）。其他 RGB 已从交付包移除；原 JSON 内的路径、哈希保留用于追溯，明确省略清单见 `ASSET_POLICY.json`，不是完整图像集。原始深度数值保留质量标记，不视为已确认尺寸。

把手模型与尺寸表位于 `medicine_frontend/data/handle/`：用户确认把手外长 170 mm；CAD 外廓 170×70×25 mm、名义开口 150×50 mm，其余为模型值，非全部实物实测。目标行程已由 250 mm 追加到 300 mm；TCP 约 301.14 mm 位移不能当作独立测得的抽屉实际行程。柜体长宽高及夹指尺寸没有可靠记录，不补猜。
