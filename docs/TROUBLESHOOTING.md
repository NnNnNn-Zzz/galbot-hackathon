# G1 真机调试经验文档

## 抽屉示教复现被关节配置门控拒绝（2026-11-07）

- **现象**：手部 TCP 看似回到把手处，但 `replay_drawer_pull.prepare` 返回 `blocked`，没有下发拉屉命令。
- **确认原因**：`drawer_replay_20261107_190843/prepare.json` 唯一问题为 `right-arm joints differ from recorded alignment`。与配方相比 TCP 位置差约 0.036 mm，但右臂 joint3、joint5 分别差 +0.315664、−0.317870 rad，超过逐关节 0.12 rad 门槛。相同末端位置不保证相同关节解。
- **诊断**：对照 prepare.json 的 `state.right_arm_joints_rad` 与配方 `alignment_right_arm_joints_rad`，计算每关节差值；同时检查 `issues` 和 `motion_commands_sent`。不要把准备阶段拒绝误判为拉力不足。
- **处理**：采集实际关节配置关键点，核对初始/终点和基座条件，再生成、验证点间路径；或在当前已验证姿态上重新示教。不要仅放宽门槛。
- **验证结果**：已从日志确认拒绝原因；本轮已有 P0/P1/P2 三个静止点，但原始终点尚未被现场明确确认，点间回放未做真机验收。完整证据和其他失败分类见 [抽屉交接失败总结](../handoffs/drawer_20261107/FAILURES.md)。

## 抽屉控制器切换前力基线超时（2026-11-07）

- **现象**：`drawer_replay_20261107_185938/controller_switch.json` 报 `Right wrist force baseline did not arrive`。
- **确认原因**：力监控未在就绪期限内返回可用基线，`submission=not_started`；前后均为 `right_arm_pvt_ctrl_teleop`。最终日志有 299 个样本，不证明命令前基线已及时就绪。
- **诊断/处理**：分开记录 SSH 连接、SDK 初始化、首帧和基线完成时间，核对设备时间戳及新鲜度；只有明确耗时来源后才调整等待，不跳过保护。SSH/SDK 启动慢目前为解释假设，不等同传感器故障。
- **验证结果**：日志证明保护阻止了切换；该次未发送运动，尚无此原因修复后的独立真机复测结论。

> 每次任务开始前先 `git pull` 拉最新版本文档；调试中遇到新坑请追加到对应章节并 `git push`。

## 速查：机器人怎么连、怎么跑

```bash
# 1. SSH 到机器人（场地 WiFi 同网段，IP 可能变，看路由器 DHCP 分配）
ssh galbot@<机器人IP>      # 密码 gb@2023

# 2. 环境
cd ~/galbot_hackathon && source scripts/env_hpu.sh

# 3. 冒烟（前进10cm再退回）
python3 scripts/test_base_motion.py --distance 10

# 4. agent 全链路
LLM_API_KEY="sk-..." LLM_BASE_URL="https://api.deepseek.com/v1" LLM_MODEL="deepseek-chat" \
python3 agent/main.py --task "往前走50cm"

# 5. 运行中追加任务（另开终端）
echo "左转90度" > ~/galbot_hackathon/task.txt
```

改完代码在 WSL 一键重新部署：
```bash
bash scripts/deploy_hpu.sh [机器人IP]   # 默认 192.168.166.7
```

## 速查：错误状态 → 原因 → 修法

| 错误 | 原因 | 修法 |
|---|---|---|
| `robot.init()` 返回 False（40s 超时） | SDK 连不上机器人服务 | 检查 SSH 是否能登上机器人；机器人是否开机 |
| `WAIT_INITIALIZED` | `service_navigation_plan` 没跑 | wrapper 已自动启动（`_ensure_nav_service`）；如果还不行手动 `LD_LIBRARY_PATH=/data/galbot/lib /data/galbot/bin/service_navigation_plan &` |
| `move_straight_to: FAILED` | 导航规划服务缺 localization_server，位姿规划不可用 | **不要用 `nav.move_straight_to`**，用 `robot.set_base_pose`（wrapper 已改好） |
| `navigate_with_velocity: FAILED` | 同上，导航服务不完整 | 同上，用 `robot.set_base_pose` |
| `TIMEOUT` | `is_blocking=True` 在部分固件上不报告到达 | wrapper 已改为 `set_base_pose(is_blocking=True)` 直接控制 |
| `set_base_pose` 返回 `TIMEOUT` 且底盘零位移 | 整机进入 `ABNORMAL` 模式，运动被整机保护拒绝（2026-11-07 真机排查） | 在 XCU 运行 `/data/bin/sys_tool`，输入 `q` 检查 `mode_type`；再查 `/userdata/log/mission_manager/mission_manager.INFO` 中的 `abnormal_service`。恢复异常服务或重启机器人，确认回到 `WORKING_MODE` 后重试 |
| `ImportError: libgalbot_sdk.so` | 没设 LD_LIBRARY_PATH | `source scripts/env_hpu.sh` |
| `OpenSSL version mismatch` (WSL ssh) | `.bashrc` 自动加载 SDK setup.sh 污染了 LD_LIBRARY_PATH | 已注释掉 ~/.bashrc 第 121-123 行 |
| Python 3.8 `TypeError: X | None` | HPU 是 Python 3.8，不支持 3.10 联合类型注解 | 所有 .py 已加 `from __future__ import annotations` |
| `ModuleNotFoundError: galbot_sdk` (HPU) | HPU 没装 Python SDK 包 | `deploy_hpu.sh` 会自动推送；手动：从 WSL rsync `~/GalbotSDK/galbot_sdk/linux-aarch64-gcc940/lib/python/galbot_sdk`（只留 cp38）到 HPU `~/g1_runtime/` |
| Web Console 网页和三维模型能打开，但机器人状态不更新 | 网页静态资源可独立运行；`--live` 只表示尝试初始化 SDK，不能证明 PC 后端已连到机器人 | 按下文“四层验收”检查 SDK 初始化、`connected`、设备时间戳持续推进和机器人 SN；先核对 PC—HPU—XCU 有线网络及同一 Python 环境中的 SDK 导入 |

## 架构决策记录

### 底盘运动：用 `GalbotRobot.set_base_pose`，不用 `GalbotNavigation`

**踩坑过程（按时间线）：**

1. `nav.move_straight_to(goal, is_blocking=True)` → `WAIT_INITIALIZED`
   - 原因：`/data/galbot/bin/service_navigation_plan` 没在跑（launcher.service 没启动它）
   - 修法：手动启动 + wrapper 加 `_ensure_nav_service()` 自动拉起

2. 启动 service_navigation_plan 后 → `move_straight_to` 返回 `FAILED`
   - 原因：导航规划服务依赖 `localization_server`（没跑、没地图），位姿规划不可用
   - `nav.is_localized()` 返回 False，`nav.relocalize()` 10 次也没用（没地图）

3. 改用 `nav.navigate_with_velocity(vx, vy, vyaw, duration)` → 命令被接受(`True SUCCESS`)，但状态轮询立刻 `FAILED`
   - 同根原因：navigate_with_velocity 也经过导航服务，服务不完整就失败
   - 换 `CHASSIS_TWIST_CTRL` 控制器也一样

4. **最终方案**：`robot.set_base_pose(x, y, yaw, frame_id='rel(0)', is_blocking=True)`
   - 这是 `GalbotRobot` 上的直接底盘控制方法，**不经过 GalbotNavigation / 导航规划服务**
   - 不需要 localization / map / service_navigation_plan
   - 实测返回 `ControlStatus.SUCCESS`，机器人真实移动
   - wrapper._move_straight 已改用此接口

**结论：G1 底盘控制走 `GalbotRobot.set_base_pose`，不走 `GalbotNavigation`。** 除非你需要全局路径规划（navigate_to_goal），才需要完整的导航服务（localization_server + map_server + service_navigation_plan）。

### 整机 `ABNORMAL` 模式会拒绝底盘运动

2026-11-07 真机排查中，机器人能连接，但 `set_base_pose` 返回 `TIMEOUT` 且底盘零位移。检查 XCU 的 `sys_tool`（进入后输入 `q`）发现 `mode_type: ABNORMAL`，并在 `mission_manager.INFO` 的 `abnormal_service` 记录中定位到异常服务。恢复服务或重启机器人后，确认 `mode_type: WORKING_MODE`，再执行底盘测试；该次重启后底盘恢复运动。

直控底盘不依赖地图或定位，但整机异常模式仍会阻止运动。若机器人能运动、只有全局导航失败，再检查 HPU 上的定位、地图和导航服务。真机测试步骤见 [REAL_ROBOT_TEST_GUIDE.md](REAL_ROBOT_TEST_GUIDE.md)。

### 头部双目相机有 RGB、却读不到深度（2026-11-07）

- **现象**：`HEAD_LEFT_CAMERA` / `HEAD_RIGHT_CAMERA` 的普通 RGB 接口可用，但 `PerceptionTool` 的相机配置没有对应的 `depth_sensor`，无法从该路径取得双目深度。
- **确认原因**：G1 SDK 的双目推理属于 `GalbotPerception` 的 `FOUNDATION_STEREO` 模块，结果放在 `DetectionResult.instance_mask`；它不通过 `GalbotRobot.get_depth_data` 返回。SDK 中的 `LIGHT_STEREO` 标为当前版本不支持。
- **诊断**：在 HPU 执行 `source scripts/env_hpu.sh` 后，调用 `GalbotPerception().init({PerceptionModule.FOUNDATION_STEREO})`，等待约 10 秒，再依次检查 `run_once(...)`、`wait_for_new_result(..., timeout_s=8)`、`get_latest_result(...)` 是否成功，以及 `np.asarray(result.instance_mask).shape/dtype`。若在 PC 上 `init` 返回 `False`，先在 HPU 做同样检查，避免将 PC 到机器人服务的连通问题误判为没有双目能力。
- **修复**：使用 `StereoDepthTool.capture_stereo_depth()` 直接调该 SDK 模块，保存原始 `.npy` 与伪彩色预览。SDK 未明确原始深度数值的物理单位，暂不换算成米。
- **验证**：2026-11-07 在 HPU 真机完成封装调用，返回 `succeeded/real`，图像为 1280×960、`uint16`，有效像素比例 1.0，原始值范围 384–7798；预览能区分桌面、物体和背景。此结果验证深度图获取，不证明米制距离精度。

### 部署架构：全机器人端（HPU），不在 PC 上跑

- Agent + SDK 都跑在机器人 HPU 上（hostname `galbot-echo`，aarch64，Ubuntu 20.04，Python 3.8）
- HPU 有外网（DeepSeek 直连，不需要代理）
- PC（WSL）只做代码编辑和部署（`deploy_hpu.sh` 一键 rsync）
- SDK 原生库用机器人自带的 `/data/galbot/lib`（版本与固件配套，勿动）
- SDK Python 包从 WSL 推送到 HPU `~/g1_runtime/`（aarch64 cp38 版）

### 为什么不在 PC 上跑 SDK

- WSL 是 mirrored 网络模式 + Clash TUN，HTTPS 出口基本不通
- PC 没有 192.168.1.99 网卡（官方网线直连拓扑要求）
- embosa 发现服务（11888）在 PC 侧从来没通
- 机器人本机跑 SDK 走本地回环，以上问题全部消失

### Web Console 状态同步：页面可访问不等于实机已连接

**适用范围**：独立项目 `galbot-sdk-web-console` 的 PC 部署；本仓库的 Agent 仍按上节在 HPU 运行。来源：用户提供的 `robot-state-sync-runbook.md`（2026-11-07 收录）。原文记录的 PC 只有无线地址、缺少 SDK 官方 PC 部署要求的 `192.168.1.x/24` 有线连接；未执行 Embosa 三端配置，**尚未完成 PC 实机状态同步验收**。因此以下是排查与部署路径，不是已验证的修复结果。

**现象与原因**：`127.0.0.1:8080` 网页、静态机器人模型、机器人 SSH、独立相机截图服务均可工作，但它们不证明“机器人传感器/控制服务 → HPU/XCU → SDK → 控制台后端 → `/api/events` → 浏览器”这条状态链路已连通。`--live` 只尝试初始化 SDK；`live_mode=true` 也不能代替 `connected=true`。

**诊断顺序**：

1. 在运行控制台的 **PC** 上确认有线网口、路由和源地址：`ip -4 addr`、`ip route get <HPU有线IP>`，再从 PC、HPU、XCU 三端检查互通。仅能连机器人无线 IP 不满足 PC SDK 的三端有线网络条件。
2. 在启动控制台的同一个 shell 和 Python/`uv` 环境中确认 `import galbot_sdk.g1` 成功；查看启动日志有无 `Galbot SDK runtime initialization failed`。机器人 SSH 终端能导入 SDK，不代表 PC 控制台进程能导入。
3. 登录控制台后逐层验收：`/api/capabilities.live_mode=true`；`/api/robot/state.connected=true`；`/api/events` WebSocket 持续收到 `navigation_state`、`joint_state`，且关节数据的**设备** `timestamp_ns` 向前推进；最后用 `get_device_information().serial_number` 对照目标机器人 SN。事件外层时间戳只说明控制台在发消息，不代表设备有新采样。
4. 若 API 返回 `401`，先确认登录的是本次控制台启动生成的访问码；相机截图服务与控制台的访问码不是同一个。若 `connected=true` 但画面冻结，检查 WebSocket 断线、`service_error` 和设备时间戳。

**处理路径与验证边界**：采用 PC 官方 SDK 部署时，先完成三端网络和 SDK 环境配置，再在维护时段运行官方 `configure_embosa_ip.sh`；该脚本会修改 PC、HPU、XCU 配置并重启服务，应先保存原配置。随后启动控制台 `--live`，只有上述四层验收全通过才能称为状态已同步。若无法建立三端有线网络，另行实现兼容 HPU Python 3.8 的只读状态桥；现有相机截图服务不是状态桥。两条路径在来源文档中都未完成实机同步验收。

## HPU 环境详情

| 项目 | 值 |
|---|---|
| hostname | galbot-echo |
| 架构 | aarch64 (ARM) |
| OS | Ubuntu 20.04.6 LTS |
| Python | 3.8.10（**不支持 3.10 语法**，所有 .py 已加 future annotations） |
| SDK 原生库 | `/data/galbot/lib`（libgalbot_sdk.so.1.10.0 + fastcdr 全套，机器人自带） |
| SDK Python 包 | `~/g1_runtime/galbot_sdk/`（从 WSL 推送，cp38 版） |
| 磁盘 | 1.8T NVMe，无压力 |
| 外网 | 有（DeepSeek API 直连返回 401=连通） |
| SSH | galbot@IP / gb@2023 |
| 固件 | GBS_1.18.2.0.rc68 |
| 序列号 | R001ABDD22AA0480 |

## 安全检查清单（每次真机前过一遍）

- [ ] 急停按钮已释放（弹起来，不是按下去的）
- [ ] 机器人周围 2m 空旷
- [ ] 机器人不在充电座上（充电时可能限制运动）
- [ ] Ctrl+C 能随时触发归位关机（wrapper.shutdown 自动执行）

## 导航服务（如果将来需要全局路径规划）

目前底盘移动用 `robot.set_base_pose`（直接控制），不需要导航服务。
如果将来需要 `nav.navigate_to_goal`（全局路径规划），需要启动以下服务：

```bash
# 在 HPU 上（需要地图文件）
LD_LIBRARY_PATH=/data/galbot/lib /data/galbot/bin/service_navigation_plan &
LD_LIBRARY_PATH=/data/galbot/lib /data/galbot/bin/localization_server &
```

`service_navigation_plan` 日志里如果出现 `定位话题 update timeout, 来源localization_server`，说明 localization_server 没跑。

## 文件索引

| 文件 | 用途 |
|---|---|
| `scripts/env_hpu.sh` | HPU 端环境变量（PYTHONPATH + LD_LIBRARY_PATH） |
| `scripts/env.sh` | PC/WSL 端环境变量（已基本不用，HPU 部署后） |
| `scripts/deploy_hpu.sh` | WSL→HPU 一键部署（rsync 代码 + SDK + mock 自检） |
| `scripts/test_base_motion.py` | 底盘直连冒烟测试（前进Ncm再退回） |
| `sdk_wrapper/wrapper.py` | 封装层（`set_base_pose` 直接控制 + 安全限幅 + 自动启动导航服务） |
| `safety_config.json` | 安全参数（单次最远2m / 限速0.3m/s / 最大转向180°） |
