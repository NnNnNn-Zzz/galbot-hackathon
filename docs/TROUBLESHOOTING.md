# G1 真机调试经验文档

> 每次任务开始前先 `git pull` 拉最新版本文档；调试中遇到新坑请追加到对应章节并 `git push`。

## 速查：机器人怎么连、怎么跑

```bash
# 1. SSH 到机器人（场地 WiFi 同网段，IP 可能变，看路由器 DHCP 分配）
ssh galbot@<机器人IP>      # 密码由现场管理员提供

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
| `OpenSSL version mismatch. Built against 30000020, you have 30300000`（WSL `ssh`） | 当前 shell 的 `LD_LIBRARY_PATH` 先加载了 SDK 自带的 `libcrypto.so.3`，与系统 `ssh` 的 OpenSSL ABI 不匹配；不是机器人 SSH 服务或密码错误 | 只给 SSH 命令使用系统库：`export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu && ssh galbot@<HPU_IP>`；也可用 `env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ssh galbot@<HPU_IP>` 避免影响同一终端里的 SDK 命令。确认诊断：`LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ldd "$(command -v ssh)" \| grep libcrypto` 应指向 `/usr/lib/x86_64-linux-gnu/libcrypto.so.3`。若不再需要 SDK，清除或修正 `.bashrc` 中自动加载 SDK 环境变量的行。2026-11-07 在 WSL 实测该修复后可登录 `galbot@192.168.166.7`。 |
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
- **修复**：使用 `StereoDepthTool.capture_stereo_depth()` 直接调该 SDK 模块，保存原始 `.npy` 与伪彩色预览。2026-11-08 用头部外参和地面平面完成尺度验证后，应用层换算固定为 `depth_m = raw_value * 0.001`。
- **验证**：2026-11-07 在 HPU 真机完成封装调用，返回 `succeeded/real`，图像为 1280×960、`uint16`，有效像素比例 1.0，原始值范围 384–7798；预览能区分桌面、物体和背景。2026-11-08 复测得到头左内参 `K` 为 `fx=486.2493, fy=486.2607, cx=652.1300, cy=480.4339`、`distortion_model=kannala_brandt`，并通过 `get_sensor_extrinsic(HEAD_LEFT_CAMERA, "base_link")` 反投影。按毫米假设变换到 `base_link` 后，地面 RANSAC 平面 `z=-0.0021 m`、法向 `(0.0868, -0.0323, 0.9957)`，42,809/86,016 个采样点落在 3 cm 平面内；其他候选尺度会得到数十米到数千米的地面高度，因此确认 FOUNDATION_STEREO 原始深度单位是毫米。RGB 高 992、双目图高 960，反投影时 `fy/cy` 必须乘 `960/992`，并对 Kannala-Brandt 鱼眼畸变先去畸变。

  2026-11-08 正式 charger 链路复测时，`StereoDepthTool` 报 `ImportError: cannot import name '_sdk_bootstrap' from 'sdk_wrapper.wrapper'`。确认原因是 wrapper 重构后 bootstrap 已移动到 `sdk_wrapper.dev.adapters.galbot_adapter`，工具仍从旧位置导入。将导入改为 adapter 的 `_sdk_bootstrap()` 后，真机头部检测得到有效的 `target_3d` 和 `target_safety`；当前低置信提案因距离、平面法向等安全门槛被正确拒绝，没有进入底盘或腕部运动。

### G1 躯干轴向：`leg_joint4/5` 不当前倾，`leg_joint3` 可产生俯仰（2026-11-07）

- **现象**：监督脚本最初期望用 `torso` 链的 `leg_joint4`、`leg_joint5` 实现前倾，但真机中 `leg_joint4` 产生偏航，`leg_joint5` 也没有产生俯仰；现场观察到侧向运动，`leg_joint5` 微步时头部滚转约变化 1.14°，俯仰和偏航基本不变。
- **确认原因**：G1 的 `set_leg_height()` 使用 `LEG_HEIGHT_CTRL` 协同解算前三个腿关节，`leg_joint4/5` 不属于该高度协同段，也没有可用的单关节前倾轴。改用 `leg_joint3` 做 0.002 rad 受控微步后，头部俯仰变化明显，确认它是前倾候选。
- **诊断**：在 HPU 的 `source scripts/env_hpu.sh` 环境读取全部五条腿关节状态，再记录 `get_transform("base_link", "head_base_link")` 的四元数并计算 roll/pitch/yaw。单步只命令 `leg_joint3`，比较动作前后的目标关节、其它关节漂移、头部 X/Z 位移和姿态变化；只看 X/Y 位移会把偏航或滚转误判为前倾。
- **决策与修复**：`TorsoLeanTool` 仍禁用 `leg_joint4/5` 的单关节动作；前倾控制应改用 `leg_joint3` 的受控小步。因为 `leg_joint3` 属于高度协同段，单动它也会耦合身高和支撑位置，不能直接套用之前的纯躯干关节范围。
- **验证结果与边界**：2026-11-07 先单步 `+0.002 rad`，实际 `leg_joint3` 变化 `+0.001977 rad`；头部俯仰变化 `+0.002083 rad`（约 +0.119°），滚转变化 `+0.000068 rad`、偏航变化 `+0.000755 rad`；头部 X 前移约 `+1.258 mm`，高度变化约 `-0.053 mm`。随后监督执行更大一步 `+0.010 rad`，实际关节变化 `+0.009983 rad`；头部俯仰变化 `+0.009971 rad`（约 +0.571°）、滚转和偏航均只有约 0.01° 量级，X 前移约 `+6.00 mm`，高度变化约 `+0.04 mm`。目标位置在 `leg_joint3≈0.262107 rad` 保持；相隔 2 秒复读时 `leg_joint3` 漂移为 0，头部位置漂移小于 `0.04 mm`。随后把五条腿关节一次性恢复到实验前基线，返回 `SUCCESS`，各腿关节目标误差最大约 `0.000084 rad`；头部/双臂未被命令，最大自然漂移约 `0.00046 rad`。恢复后 2 秒内全部活动关节最大漂移约 `0.000049 rad`，无关节超过 `0.0001 rad`。这确认正方向小步可产生前倾且可保持和恢复，但不证明更大角度、长期承载或重心安全极限。

### `set_leg_height` 返回 `FAULT` 但已到达目标（2026-11-07）

- **现象**：把腿高从 `1.051637 m` 提升到 1.42 m 时，`set_leg_height(1.42, 6.0, True, 20.0)` 返回 `ControlStatus.FAULT`，但随后读取 `head_base_link` 的 Z 为 `1.419966 m`，距目标仅 0.034 mm。
- **确认结果与原因边界**：已确认“返回 FAULT 时目标仍可能已经到达”，因此不能仅凭返回枚举判断是否执行成功。本次 FAULT 的具体触发原因尚未定位，可能是阻塞式任务完成状态的上报差异，尚属假设。
- **诊断**：动作后立即读取 `get_transform("base_link", "head_base_link")`，比较 Z 与目标，并连续读取一次确认时间戳更新和位置稳定；不要因为 FAULT 就盲目重发同一个大位移命令。
- **处理与验证**：本次没有重发命令，位置读数与目标误差小于 0.1 mm，随后正常执行 SDK shutdown。后续封装应把“SDK 状态”和“实际目标位姿”分开报告；只有位置复读也异常时才进入故障处理。

### `8092` 相机桥进程存活但请求队列堵满、页面超时（2026-11-08）

- **现象**：`http://192.168.166.7:8092/` 的腕部及健康接口请求超时，`galbot-camera-robot.service` 仍显示 `active (running)`；`ss -ltnp` 显示 `8092` 的 `Recv-Q/Send-Q` 都为 `5`，达到 `ThreadingHTTPServer` 默认 backlog。日志同时出现 `BrokenPipeError`。
- **原因边界**：已确认存在请求队列耗尽，重启服务后恢复；尚不能确认队列为何未及时排空，可能是浏览器持续轮询叠加长时间未结束的请求。该现象发生在接入腕部 YOLO 叠加之前，不能归因于模型推断。
- **诊断**：在 HPU 执行 `ss -ltnp | grep 8092`，检查监听进程是否存活以及队列是否为 `5/5`；再查 `systemctl --user status galbot-camera-robot.service` 和 `/home/galbot/galbot-camera-shots/systemd.log`。不要仅因进程存活就判断服务正常。
- **修复**：先备份 `/home/galbot/galbot-camera-shots/robot_server.py` 与对应的 user unit，再执行 `systemctl --user daemon-reload && systemctl --user restart galbot-camera-robot.service`。该操作只重启只读相机桥，不发送机器人运动或夹爪命令。
- **验证**：2026-11-08 重启后队列恢复为 `0/5`，主页返回 HTTP 200，认证后的 `/health` 以及左右腕 `/frame/*` 均返回 JPEG；腕部实例分割状态为 `ready`。后续若再次出现 `5/5`，应同时检查前端轮询和单个请求是否阻塞，不能无限重启掩盖问题。

  `8092` 服务与正式 ToolHub 使用不同的权重配置入口。正式配置切到 `charger_whole` 后，页面仍可能继续加载旧 `charger_seg_clean_v2`，因为服务进程只在启动时读取自己的默认权重。诊断时先读取认证后的 `/api/info.segmentation.weights` 和 `strategy`，再与 `sdk_wrapper/dev/config/charger_yolo.json` 对照；如果 phase 不一致，需要更新 `/home/galbot/galbot-camera-shots/robot_server.py` 的默认模型并重启服务。2026-11-08 已将页面服务改为复用 `WholeRoiTopSurfaceBackend`，`/api/info` 验证为 `charger_seg_whole_final -> charger_seg_clean_v2`，策略 `whole_roi_top_surface`。

### 夹爪碰到物体后继续收窄会返回 `TIMEOUT`（2026-11-08）

- **现象**：右手夹爪夹住约 2 cm 宽的充电器时，先后请求 1.8 cm 和 1.6 cm 开度，`set_gripper` 均返回 `control_status: TIMEOUT`、`success: false`；现场确认 1.6 cm 指令后夹持已足够紧。返回结果只保留请求值，没有实际夹爪开度反馈。
- **原因边界**：已确认返回 `TIMEOUT` 不代表夹爪没有动作或没有夹住物体；具体触发机制仍是**假设**，可能是两指接触物体后无法到达更小目标宽度，阻塞式控制等待超时。本次没有测量电机堵转状态，不能把 `TIMEOUT` 单独当作硬件故障证据。
- **诊断**：每次只下发一个小开度变化，立即目视或手动确认两指是否接触且物体稳定；若已稳定就停止，不要继续逐级收窄或在 `TIMEOUT` 后盲目重发。需要量化夹持时，应另加力/位置反馈，不能把请求开度当成实测开口。
- **处理与验证**：把“物体限位导致的 `TIMEOUT`”作为停止条件，记录请求开度并标记实际开度未知；程序化抓取必须通过抬起后重新检测目标是否随夹爪移动来确认抓取，不能仅凭 `TIMEOUT` 或“闭合指令已下发”报告成功。2026-11-08 本次以 1.6 cm 请求停下并由现场确认夹持足够紧；未做负载、长期夹持或实际开口尺寸验证。

### 腕部 charger pre-grasp 返回 `FAULT`：目标超出右臂 IK 可达域（2026-11-08）

- **现象**：腕部 YOLO-seg 检出 charger 后，`move_to_wrist_charger_pregrasp(operator_confirmed=True)` 返回 `failed/sdk_error`，消息为“末端移动失败（FAULT）”；机器人没有执行该臂目标。
- **确认原因**：`service_motion_plan` 在 `01:32:33` 对 `right_arm` 目标 `[0.8988262, -0.4200548, 0.9301692]` 连续报 `TRAC-IK solver failed to find a solution`，随后记录 `SERVICE_STATUS_EXECUTE_FAILED`。把同一位置替换成当前末端姿态仍返回 `FAULT`，而把目标姿态放到当前末端位置则返回 `SUCCESS`，确认主因是**目标平移超出右臂可达域**，不是通信、急停或碰撞服务本身。
- **附带问题**：首次 YOLO 框落在 charger 窄边/侧面，三维尺寸仅约 `3.6×2.4 cm`，表面法向偏离 base_link 竖直方向约 `60.9°`。末端轴映射也曾错误地假设 local `+Z` 是接近轴；只读挂载 `galbot_gripper` 后确认 TCP 沿 local `+X` 偏置约 `0.146 m`，相机光轴也近似 local `+X`，因此接近轴应为 local `+X`。
- **诊断**：

  ```bash
  # HPU 上 source scripts/env_hpu.sh 后，对同一目标做只读 IK：
  python3 - <<'PY'
  from sdk_wrapper.dev.adapters.galbot_adapter import GalbotAdapter, _enum_name
  ad = GalbotAdapter(enable_real_control=True, real_authorized=True)
  _, motion, _ = ad._ensure_interfaces()
  target = [0.8988262, -0.4200548, 0.9301692,
            0.7178771, 0.4775492, -0.1712226, -0.4767411]
  status, joints = motion.inverse_kinematics(
      target, ["right_arm"], target_frame="EndEffector",
      reference_frame="base_link", enable_collision_check=False)
  print(_enum_name(status), joints)
  PY

  grep -n "TRAC-IK solver failed\|SERVICE_STATUS_EXECUTE_FAILED" \
    /userdata/log/service_motion_plan/Info_*.log | tail -40
  ```
- **修复**：末端姿态改为 local `+X=接近轴`、local `Y=两指闭合轴`、local `Z=夹爪长轴`；`plan` 阶段先调用只读 `inverse_kinematics`，无解时直接返回 `rejected/safety_limit`，不再把命令送进运动服务。腕部精定位同时增加物理长/短边最小值（`0.02/0.015 m`）和表面法向相对竖直最大倾角门槛；当前校准值为 `50°`。
- **验证**：2026-11-08 HPU 单测通过。修正轴映射后，首个位姿的旋转量从约 `167°` 降到约 `93°`，但只读 IK 仍返回 `FAULT`，工具在运动前明确拒绝。调整机器人位置后再次观测到完整顶面：中心 `[0.60777, -0.052781, 0.735523]`、长/短边 `3.02/2.37 cm`、平面 RMS `1.38 mm`、法向倾角 `3.98°`，证明该充电器本身是小型目标，最初 `4×6 cm` 门槛过严。最终门槛改为 `2×1.5 cm` 并新增小尺寸合法样本回归。更新后的目标 `[0.7025586, -0.034594, 0.8552943]` 仍因距离稍远而无 IK 解；虚拟底盘前移约 `0.12 m` 后即可求解，因此继续测试时应先小幅靠近再重新 `plan`。

### charger 上表面接近水平却被判成法向倒置（2026-11-08）

- **现象**：D405 点云估计的充电器上表面中心为 `[0.571627, -0.27607, 0.737347]`，三维尺寸和平面残差都正常，但 `surface_normal_base.z=-0.9983`，因此被 `surface_normal_tilt_above_maximum` 拒绝。
- **确认原因**：平面 PCA 求出的法向先按 `z>=0` 翻到上方向，随后又经过通用的“首非零轴取正”规范化；当平面的 x 分量略负时，该步骤把整个法向再次翻转，造成 `z<0`。这是确定性的坐标方向 bug，不是深度或机器人姿态问题。
- **诊断**：读取 `surface.surface_normal_base`，对接近水平的工作台面应满足 `z≈+1`；若 `x/y` 很小而 `z≈-1`，检查是否在朝上修正之后又执行了首轴符号规范化。用保存的 `charger_mask_bool.npy`、`depth_m.npy`、原内参和当前深度相机外参离线重算即可复现。
- **修复**：平面法向只做单位化和 `z>=0` 修正，不再调用首非零轴 canonicalization；长边轴仍可独立规范化。新增小负倾角回归测试，确保 `-10°` 的合法表面仍得到向上法向。
- **验证**：2026-11-08 对同一保存帧重算后，法向变为 `[-0.02009, 0.05441, 0.99832]`，倾角 `3.32°`，观测通过精定位门槛；使用当前 `EndEffector` 对生成的 pre-grasp pose 做只读 IK 返回 `SUCCESS`。后续实时帧重新检测到 charger 后即可继续 `plan`/`move`。

### `charger_seg_clean` / `clean_v2` 在真实腕部视角无可靠输出（2026-11-08）

- **现象**：实时腕部 `detect` 在 charger 清晰可见、距离约 `0.50 m` 时返回 `not_ready`；原图 1280×720 中 charger 约 `64×87 px`，并非出框或被夹爪遮挡。换成 `charger_seg_clean_v2` 后，当前右腕的 100W 白色充电器仍为 0 框。
- **确认原因**：模型本身已正确加载为 `YOLO11-seg / charger`，但 clean 系列对当前腕部视角和充电器表征的召回退化。2026-11-08 对同一当前右腕原图、固定 `640/iou=0.7`、把阈值降到 `0.001` 后，`yolo11n_seg_charger_v2` 最高分为 `0.593493`，`charger_seg_clean` 为 `0.092665`，`charger_seg_clean_v2` 只有 `0.030783`；只裁出充电器附近区域后 clean_v2 最高仍只有约 `0.0207`。因此不是阈值、输入裁切、目标太小或推理链路问题，而是新训练丢失了旧权重拥有的目标表征。
- **诊断**：直接对同一保存帧运行新权重和旧 v2 权重，固定 `iou=0.7`、`conf=0.05`：

  | 权重 / 输入 | 当前 charger 框 | 结果 |
  |---|---:|---|
  | `charger_seg_clean`, 640 | 无 | 低于 0.05 |
  | `charger_seg_clean`, 1280 | `0.156` | 低于生产阈值 0.30 |
  | `yolo11n_seg_charger_v2`, 640 | `0.432` | 可通过 0.30 |

  该对比证明相机、D405、预处理和 YOLO 调用链均在工作，差异来自模型对该角度/距离的训练覆盖。

  `clean_v2` 的离线结果也需要谨慎解读：数据集只有 54 张训练图（50 张正样本、4 张负样本）和 13 张验证图（12 张正样本、1 张负样本），且 `split.json` 是按帧随机抽取，不是按连续拍摄会话分组。640 输入下 `conf=0.20` 时验证集为 12/12，但 `conf=0.25` 时漏掉最高分为 `0.2168` 的 `charger_0026`；改到 960 输入反而只有 7/12，因此不能用“提高 imgsz”或只看随机验证集 mAP 解决真实场景召回。
- **处理方案**：当前演示若要求立即检测，应恢复 `yolo11n_seg_charger_v2` 或做双模型/旧模型召回兜底；继续使用 clean 权重时，必须把当前腕部失败帧、不同距离、不同滚动角、插头在视野左右两侧的样本与旧数据中的有效正样本混合训练，并按拍摄会话分组验证。不要只把全局 `confidence` 降到一个很低值：它会同时引入背景误检。模型替换后必须对当前真实腕部帧做固定阈值扫描，并同时检查负样本误检，不能只看单一验证集 mAP。
- **后续修复（2026-11-08）**：已改为 `charger_whole` 整体模型先行划定 ROI，再在 ROI 内调用 clean 上表面模型；旧模型漏检或上表面几何不可用时，保留整体掩码并随机采样有效深度点，每个采样点仍须通过局部平面 RMS、法向朝上、深度有效率、触边和 IK 检查。真实右腕冒烟中整体模型置信度 `0.496`，旧上表面模型无可接受结果，回退采样得到 `uv=[658,496]` 且深度有效率 `92.5%`。这解决“有整体目标却因旧上表面漏检直接失败”的问题。
- **验证**：2026-11-08 用当前右腕原图和 clean 验证集完成旧/新权重对照、阈值扫描、640/960 输入对照及目标裁剪复核；clean_v2 没有达到当前生产门槛，旧 v2 在同一 640 输入下有约 `0.593` 的有效目标框。

### charger `move` 在确认后重新规划失败（2026-11-08）

- **现象**：操作员输入 `MOVE` 后，程序报“运动前重新规划失败：腕部已识别 charger，但该观测不满足精定位门槛”；机械臂未移动。现场看到 YOLO 置信度约 `0.84`，容易误判为无理由拒绝。
- **确认原因**：旧交互顺序先要求输入 `MOVE`，之后才重新拍照并规划。`08:39` 的新观测置信度为 `0.8396`，但 D405 上表面法向为 `[-0.730,-0.111,0.674]`、倾斜 `47.6°`，被当时 `30°` 的最大倾斜门槛拒绝。置信度只表示分割类别可信，不代表三维表面适合顶抓。
- **验证**：对同一保存观测直接生成 pre-grasp，并做左臂碰撞感知只读 IK，返回 `SUCCESS`、目标旋转约 `34.8°`，说明左臂可达性和运动规划不是阻碍；拒绝来自过严的表面倾角门槛。
- **修复**：最大表面倾角从 `30°` 调整为 `50°`，仍低于此前约 `60.9°` 的侧面误检；`--stage move` 改为先重新观测、生成位姿并做 IK 预检，全部通过后才要求输入 `MOVE`。通过预检的位姿在 15 秒内直接复用，不会再因为输入确认后重新拍照导致第二次拒绝；超时后才重新规划。预检失败时直接退出，失败输出会打印嵌套观测及具体拒绝原因。
- **验证结果与边界**：2026-11-08 本地 105 项单测通过；同一 `08:39` 观测放开到 `50°` 后几何合格，左臂碰撞感知 IK 有解。复核时实时画面已没有 charger，`move` 在预检阶段直接退出，未要求 `MOVE`，也未下发真机动作。此前一次 `moving` 拒绝是动作前状态门槛，连续复读后机器人已回到 `idle`，不是规划失败。

### 腕部 charger 测试成功但打印 `success=None`、退出码 1 且没有观测详情（2026-11-08）

- **现象**：真机 `--stage detect` 的工具结果实际为 `status=succeeded`，但脚本打印 `success=None`、进程退出码 1，并且不显示 `surface`、检测阶段或后备采样坐标；容易被误判为识别失败。
- **确认原因**：`ToolResult.to_dict()` 按统一协议把业务字段放在 `data` 下，且顶层没有 `success`，状态字段只有 `status`。`test_charger_wrist_pregrasp.py` 仍按旧扁平结构读取 `result["surface"]` / `result["observation"]`，并用不存在的 `result["success"]` 决定退出码。
- **诊断**：对工具返回值执行 `print(result.to_dict().keys())`，会得到固定的 `status/mode/message/task_id/data/error_code`；`status == "succeeded"` 时业务观测在 `result["data"]`，顶层 `success` 必然不存在。
- **修复**：现场脚本统一从 `data` 读取观测、位姿和动作结果，用 `status == "succeeded"` 判断成功；新增脚本输出回归测试。修复后 `detect/plan/move` 的退出码与工具协议一致，并会打印 `target_mode`、`whole_fallback_sampling` 和采样像素。
- **验证**：新增回归测试后本地全量 116 项单测通过。真机右腕已运行到整体掩码随机采样路径；重新部署修正版脚本后只识别阶段退出 0 并显示采样详情。

### 腕部冷启动首帧偶发 `not_ready`，紧接着独立取图可正常识别（2026-11-08）

- **现象**：同一场景连续执行右腕 `detect`，第一次识别成功并生成目标；随后一次 `plan` 和一次 `detect` 都在“没有候选”处返回 `not_ready`。立即用 `capture_right_wrist()` 取 Current RGB，画面中 charger 清晰可见；同一张图离线运行 `charger_whole` 得到最高置信度 `0.9088`。
- **原因状态**：**假设**为右腕相机数据流在部分冷启动会话中尚未发布，既可能发生在新进程初始化后的首帧，也可能持续到整个会话；现有证据排除了模型权重、类别和当前场景遮挡，但尚未定位到 SDK 共享内存、相机服务或硬件连接中的具体一层，因此不把原因写成已确认。
- **诊断**：失败时同时保存 `capture_id`、RGB 和检测报告；若紧接着的独立取图清晰且离线模型高置信度命中，检查进程启动后的前几帧，而不是立即下调生产置信度。
- **修复**：腕部精定位把空图重试和检测重试分开。空 payload 最多尝试 20 次、间隔 `0.5 s`（约 10 秒冷启动预算）；只有拿到有效 RGB 后才计入 3 次“无候选”检测尝试，因此正常无目标不会等待 10 秒。每次重新取 RGB-D 和运行两级模型，结果记录在 `capture_attempt` / `capture_attempt_reports`；脚本在真机退出时注册 `toolhub.shutdown()`，确保下一次进程能重新初始化相机。头部交接仍只取一帧，避免改变任务时序。
- **验证**：新增“模型首帧空”“adapter 首帧空 payload”和“有图但连续无目标只重试 3 次”三类回归测试；本地全量 118 项单测通过。部署后连续两次真机右腕 `detect` 均成功；随后 `plan` 也成功产出 pre-grasp 位姿并通过只读 IK/碰撞预检，未下发运动。之后再次冷启动时出现连续 20 次空 payload，约 10 秒重试仍未恢复；同一时刻直接调用 `capture_right_wrist()` 也返回 `False`，说明该次失败已超出本工具的单进程重试范围，更接近相机数据源或传感器服务状态问题，原因仍待现场确认。成功会话中的三次观测均为 `capture_attempt=1`。

### `collision_check=True` 不等于机械臂会绕开桌子（2026-11-08）

- **现象**：`move_ee_to(straight_line=False)` 带有碰撞检查参数，但仍可能选择贴近桌面的不良轨迹；现场表现为夹爪容易撞桌。
- **确认原因**：旧实现走 `set_end_effector_pose()`，服务日志为 `MpsTask handling on traj plan`，本质是目标关节轨迹规划，不是采样式自由空间路径搜索。`collision_check` 只能拒绝与**已加载碰撞环境**相交的轨迹，不会自动把实时相机看到的桌子加入碰撞世界。
- **诊断**：检查 `service_motion_plan` 日志中的 planner 类型。`traj plan` 表示直接轨迹规划；低层采样式避障应显示 `Start Unified Motion Planning` 和 `MpsTask handling on motion plan`。SDK 文档也明确：GalbotMotion 不自动导入实时感知，环境障碍必须通过 `add_obstacle()` / `attach_target_object()` 加载。
- **修复**：`straight_line=False` 的绝对末端移动改为低层 `motion_plan([[MotionPlanChainTarget]], PlannerConfig)`，同时启用 `is_check_collision` 和 `enable_env_collision_check`，并允许 torso 协助；只读预检使用同一 planner 但设置 `is_direct_execute=False`，确认有完整路径后才执行。
- **验证**：2026-11-08 在 HPU 对当前左臂 EndEffector 位姿做只读路径预检，返回 `SUCCESS`；服务日志从 `MpsTask handling on traj plan` 变为 `MpsTask handling on motion plan`。本地 105 项单测通过，未执行额外真机动作。
- **边界**：这只保证在“已加载的碰撞环境”中搜索安全路径，不会凭空知道桌子几何。2026-11-08 在 HPU 查询 `GalbotMotion.get_built_obstacles_list()`，实际仅返回 `['ground']`，没有桌面或目标物体。因此当前 planner 会避开地面和机器人自身，但无法可靠避开未加载的桌子；仍需通过底盘观察位、提高预抓取高度，或使用 `add_obstacle()` 显式加载桌面 box/点云/深度图后再规划。

### 头部关节已动但姿态校验不变：测错 frame（2026-11-07）

- **现象**：`HeadLookTool` 命令 `head_joint2` 增加 0.01 rad，SDK 返回 `SUCCESS`，实际关节变化约 0.00713 rad；但工具读取 `head_base_link` 的姿态几乎不变，因此错误返回“实际俯仰方向与预期不符”。
- **确认原因**：`head_base_link` 位于头部关节之前，是头部安装基座，不随 `head_joint1/2` 转动；用它做头部动作验收会得到假阴性。真正随头部运动的是 `head_end_effector_mount_link`。
- **诊断**：同时读取 `head_joint2` 与 `head_base_link`、`head_link1`、`head_link2`、`head_end_effector_mount_link` 的 `get_transform("base_link", frame)`。若关节位置已更新而候选 frame 的姿态时间戳/数值不更新，逐层检查 frame 是否位于关节之后。
- **修复**：`HeadLookTool` 改用 `head_end_effector_mount_link` 计算头部 roll/pitch/yaw 并验证低头方向；关节位置反馈和控制目标不变。
- **验证结果与边界**：修正后 PC 端 6 个头部单元测试通过；截至记录时尚未重新发送真机头部动作，需用下一次单步键盘输入验证实际低头方向。

### 头部键盘脚本动作成功后因日志格式化崩溃（2026-11-07）

- **现象**：键盘脚本收到 `j` 后，头部工具返回成功，机器人也实际移动；随后脚本在打印 `OK ...` 时抛出 `ValueError: Unknown format code 'f' for object of type 'str'` 并退出。
- **确认原因**：日志字符串混用了 `%s` 和 `str.format()`。未替换的 `%s` 留在字符串中，而第一个 `{:.4f}` 实际收到的是字符串 `head_joint2`，因此按浮点格式输出时报错。动作本身和 SDK 状态正常。
- **诊断**：出现该 traceback 后先只读 `get_joint_states([], ["head_joint1", "head_joint2"])` 并读取 `head_end_effector_mount_link` 姿态，确认动作是否完成，不能因为脚本退出就立即重复动作。
- **修复**：把成功日志封装为 `format_step_success()`，只用 `str.format()` 并固定参数顺序；新增格式化回归测试。脚本异常时仍会把未完成动作视为停止条件。
- **验证**：2026-11-07 该次 `head_joint2` 已从约 0.0499 rad 到 0.059885 rad，头部末端 pitch 约 2.976°，按 `j` 后并未回退。修复后 PC 与 HPU 的 7 个头部测试全部通过；部署修正版后未重复发送动作。

### 部署架构：全机器人端（HPU），不在 PC 上跑

- Agent + SDK 都跑在机器人 HPU 上（hostname `galbot-echo`，aarch64，Ubuntu 20.04，Python 3.8）
- HPU 有外网（DeepSeek 直连，不需要代理）
- PC（WSL）只做代码编辑和部署（`deploy_hpu.sh` 一键 rsync）
- SDK 原生库用机器人自带的 `/data/galbot/lib`（版本与固件配套，勿动）
- SDK Python 包从 WSL 推送到 HPU `~/g1_runtime/`（aarch64 cp38 版）

### HPU 上的 YOLO11 依赖出现 TLS、C3k2 或 torchvision NMS 错误（2026-11-08）

- **现象**：Python 3.8 下先报 `import torch` 缺少 `libopenblas.so.0`；改用系统 NumPy 自带 BLAS 后又出现 `zgemm_`/`cblas_caxpy64_` 未定义。旧版 Ultralytics 加载 YOLO11 权重时报 `Can't get attribute 'C3k2'`。换成新 Ultralytics 后，NVIDIA 定制 PyTorch 与通用 torchvision 在推理时分别报 `Couldn't load custom C++ ops`、`cannot allocate memory in static TLS block` 或直接 segfault。
- **确认原因**：HPU 的 `torch 2.1.0a0+...nv23.06` 是 NVIDIA aarch64 定制构建，不能直接搭配 PyPI 通用 torchvision 的 C++ 扩展；YOLO11 权重需要包含 `C3k2` 的新版 Ultralytics。仅“torch 能 import”或“YOLO 权重能 load”不代表推理链路可用，必须在实际 `predict()` 完成 NMS 和掩码解码后才算通过。
- **诊断**：

  ```bash
  cd ~/galbot_charger_yolo
  export PYTHONPATH="$HOME/galbot_charger_yolo:$HOME/charger-yolo-runtime:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="$HOME/charger-yolo-libs:${LD_LIBRARY_PATH:-}"
  python3 - <<'PY'
  import os
  import cv2, torch
  from ultralytics import YOLO
  print(cv2.__version__, torch.__version__)
  weights = os.path.expanduser("~/yoloseg/runs/yolo11n_seg_charger_v2/weights/best.pt")
  model = YOLO(weights)
  print(model.task, model.names)
  PY
  ```

  再对一张已知 charger 验证图执行一次完整 `predict()`，检查能返回 bbox 和与输入同尺寸的 mask。
- **修复**：使用独立目录 `~/charger-yolo-runtime` 安装 `ultralytics==8.4.203`、`opencv-python-headless==4.10.0.84` 和通用 torchvision Python 包；保留机器人 NVIDIA PyTorch，不覆盖其 CUDA 构建。`sdk_wrapper/dev/algorithms/charger_yolo.py` 在启动时探测 `torchvision.ops.nms`，只有该 C++ 算子不可用时才替换为纯 PyTorch NMS；正常环境不会启用后备。
- **验证**：2026-11-08 在 `galbot-echo` 用真实 `yolo11n_seg_charger_v2/weights/best.pt` 对 1920×1098 验证图完成端到端推理，检出 1 个 charger（confidence 0.7708），返回 `1098×1920` 布尔掩码，`runtime_workaround=torchvision_nms_python_fallback`。随后真机右腕 `detect` 阶段可完成相机取图与推理，并在当前画面无 charger 时明确返回 `not_ready`，未下发运动。
- **边界**：该后备只覆盖 Ultralytics 当前使用的 NMS；若未来模型加入依赖其他 torchvision C++ 算子的后处理，需要重新验证，不能把“导入成功”当作兼容。

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

### 机器人地图管理平台 `:8000` 消失（2026-11-08 已恢复）

**先区分两个地图页面**：

| 地址 | 提供者 | 排查方向 |
|---|---|---|
| `http://<HPU_IP>:8000/` | 机器人 HPU 的 `galbot_map`，标题为“机器人地图管理平台” | 机器人在线状态、`launcher.service`、`hpu_launcher` 子进程 |
| `http://127.0.0.1:8080/#robot` | 开发电脑上的 Galbot SDK Web Console | 本机控制台进程和本地静态 PCD |

两者互相独立。机器人 `:8000` 打不开不代表本机 `:8080` 的静态地图损坏；本机没有地图也不会影响机器人自带平台。

**本次已确认原因与时间线**：地图目录和文件仍存在，`/var/maps/cur` 仍指向当前“新建地图”，并非地图被删除或 `galbot_map` 单独崩溃。2026-11-07 23:57:01，父进程 `hpu_launcher` 记录 `The proc is over!!!` 后整组服务退出，网页、SSH、地图平台和相机页面同时断开；机器人约 23:58 重启，23:58:44 新 launcher 拉起服务树。新 `galbot_map` 在 23:58:47–23:59:07 因 `gateway_service:9902` 尚未就绪持续重连，随后页面恢复。结论：**平台随 `hpu_launcher` 管理的整组服务重启而暂时消失；重启完成后由 launcher 恢复正常。**

**服务依赖**：

```text
launcher.service
└── hpu_launcher
    ├── gateway_service          # 内部网关，9902
    ├── localization_server      # 定位
    ├── galbot_map               # 地图管理平台，8000
    ├── map_manager_node
    ├── aphropm_map_server
    └── service_navigation_plan
```

机器人关机、重启、维护升级或 `launcher.service` 重启都会让 `:8000` 一起消失。不要给 `galbot_map` 另建绕过官方服务树的常驻 unit，也不要在 launcher 已运行时手动重复启动同名进程。

**开发电脑快速判断**：

```bash
ping -c 3 192.168.166.7
for port in 22 8000 18765 9902; do
  timeout 2 bash -c "</dev/tcp/192.168.166.7/$port" >/dev/null 2>&1 \
    && echo "$port open" || echo "$port closed"
done
```

正常状态为 22、8000、18765、9902 均打开。若机器人能 ping 通但业务端口全关，优先怀疑机器人正在重启，不要先删除地图或修改网络配置。

**HPU 诊断**：

```bash
hostname
uptime
systemctl status launcher.service --no-pager
ps -eo pid,ppid,lstart,stat,comm,args | grep -E \
  'hpu_launcher|gateway_service|localization_server|galbot_map|map_manager_node|aphropm_map_server'
readlink -f /var/maps/cur
ls -lh /var/maps/cur/global_cloud_cleaned.pcd
```

`uptime` 很短表示仍在启动；`launcher.service` 非 `active` 时平台不会正常；`hpu_launcher` 存在但 `galbot_map` 长时间缺失时查 launcher 日志，不要手工重复拉起。地图目录、PCD 和地图配置都可读时，不要为了恢复网页而删图或重新建图。

**日志定位**：

```bash
tail -n 120 /userdata/log/galbot_map/galbot_map.log
find /userdata/log/hpu_launcher -maxdepth 1 -type f -printf '%TY-%Tm-%Td %TH:%TM %p\n' \
  | sort -r | head -10
```

| 日志 | 含义 | 处理 |
|---|---|---|
| `dial tcp 127.0.0.1:9902: connection refused` | 内部网关尚未就绪 | 启动顺序问题，等待并刷新；持续超过 1 分钟再查 gateway |
| `write: broken pipe` / WebSocket `close 1006` | 浏览器或 Wi-Fi 断开 | 刷新页面，不能据此判定服务崩溃 |
| `process nbr exceeds max nbr` | 重复启动受 launcher 管理的进程 | 停止手动启动并回到 launcher 检查 |
| `The proc is over!!!` | `hpu_launcher` 生命周期结束 | 机器人可能正在重启或关机 |

**恢复顺序**：

1. 确认机器人是否正在重启；开机未完成时不做服务操作。
2. 等待 `launcher.service` 完整启动，通常需要数十秒。
3. 重新打开 `http://<HPU_IP>:8000/` 并强制刷新。
4. 检查 8000 和 9902 均已监听；页面能开但数据为空时优先等待 9902。
5. IP 变化时从路由器租约或机器人屏幕获取新地址。
6. 只有维护人员明确授权时才重启机器人或 `launcher.service`。不要单独杀 `galbot_map`、gateway 或 localization 再手工拼回服务树。

**验收标准**：以下条件必须同时满足。

```bash
systemctl is-active launcher.service
ps -eo comm,args | grep -E 'hpu_launcher|gateway_service|galbot_map' | grep -v grep
ss -ltnp | grep -E ':(8000|18765|9902)\b'
curl -sS http://127.0.0.1:8000/ | grep -o '<title>[^<]*'
```

预期为 launcher `active`，三个核心进程存在，8000/18765/9902 均有监听，页面标题为“机器人地图管理平台”；开发电脑访问机器人 `:8000` 返回 HTTP 200。

**开发机 `:8080` 静态地图**：控制台依次尝试 `GALBOT_STATIC_MAP`、HPU 的 `/var/maps/cur/global_cloud_cleaned.pcd`、项目内 `assets/maps/current/global_cloud_cleaned.pcd`。本次已在开发机保留当前 PCD 副本；机器人地图更新后应重新同步并计算新哈希，不能把旧文件大小或 SHA-256 当作永久固定值。

**避免复发**：固定机器人 DHCP 租约；保持 `launcher.service` enabled；不绕过官方服务树；重启后等待 8000、18765、9902 全部就绪；修改地图/定位/网络前记录 `/var/maps/cur` 和进程列表；WebSocket 断开先刷新，不把 `broken pipe` 当成平台崩溃证据；开发机保留当前 PCD，便于机器人重启期间仍能展示场景。

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
