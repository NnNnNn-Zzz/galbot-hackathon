# 真机逐项跑通指南（wrapper 全功能验证）

> **真机验证状态：22/22 全通过（2026-11-07，机器人 galbot-echo / SN R001ABDD22AA0480）**
> 配套脚本：`scripts/test_wrapper_real.py`（每步人工确认，mock 22/22 预验证 + 真机 22/22 实测）。
> 两套封装（老 `sdk_wrapper` 底盘 API + 新 `sdk_wrapper/dev` ToolResult 层）均已真机跑通，
> **后续场景/Agent 直接调用接口即可控制机器人，无需再改封装层代码。**

## 〇、功能总表

### Part base —— 老 `sdk_wrapper`（底盘 API，单位：厘米 / 度）

| # | 函数 | 干什么 | 真机实测 |
|---|---|---|---|
| B1 | `get_robot()` | SDK 单例初始化 + 自动切 CHASSIS_POSE_CTRL 控制器 | ✅ 通过（GalbotRobot 实例） |
| B2 | `get_nav()` | 导航单例（自动补启 service_navigation_plan） | ✅ 通过（GalbotNavigation 实例） |
| B3 | `check_baseline()` / `read_sensor(name)` | 状态检查 / 传感器读取 | ✅ 通过（check_baseline 返回 dict；read_sensor 预期 NotImplementedError，已知占位缺口） |
| B4 | `load_safety()` + 限幅 | 读 safety_config.json，超限静默截断 | ✅ 通过（clamp(3.0)=2.0m） |
| B5 | `stop_base()` | 立即停底盘 | ✅ 通过 |
| B6/B7 | `move_forward(cm)` / `move_backward(cm)` | 直线走（底层 `set_base_pose`，不需要地图） | ✅ 通过（10cm 前进/后退，odom 实测 9.2cm，SUCCESS） |
| B8 | `turn(deg)` | 原地转（+ 左 / − 右） | ✅ 通过（±15°，SUCCESS） |
| B9 | `baseline.reset_to_baseline()` | 归位（当前只停底盘，手臂/夹爪是 TODO） | ✅ 通过（停底盘部分；手臂/夹爪 TODO） |

### Part dev —— `sdk_wrapper/dev`（ToolResult 契约层，单位：米 / 弧度 / 秒）

| # | 函数 | 干什么 | 真机实测 |
|---|---|---|---|
| D1 | `GalbotAdapter()` 未授权 | 双闸门（`enable_real_control` + `real_authorized`），不授权连 SDK 都不 import | ✅ 通过（rejected + permission_denied，零风险） |
| D2 | `get_robot_state()` | 结构化状态快照（motion_state=idle/moving/unknown） | ✅ 通过（motion_state=idle） |
| D3 | `list_cameras / capture_scene / analyze_scene / find_object` | 相机抓帧 → 解码 → 3D 反投影 → 目标检索 | ✅ 通过（real 抓帧 640×360，analyze 检测 8 目标） |
| D4 | `get_end_effector_pose(chain)` | 读末端位姿 [x,y,z,qx,qy,qz,qw]（base_link，米） | ✅ 通过（7 元组位姿读取正常） |
| D5 | `set_gripper_width(ee, width_m)` | 夹爪开度 0–0.12 m | ✅ 通过（0.06/0.0/0.12 均 SUCCESS，夹爪真实开合） |
| D6 | `move_ee_delta / move_ee_to` | 末端直线平移 / 绝对位姿（单次上限 0.5 m） | ✅ 通过（2cm 平移 + 回位，succeeded） |
| D7a | `execute_gesture('wave_right')` | 预设动作 | ✅ 通过（**预期 rejected**：手势未标定 NOT_SUPPORTED，拒绝=设计正确） |
| D7b | `navigate_to_place('demo_table')` | 定点导航 | ✅ 通过（**预期 rejected**：places.json pose=null 未实测，拒绝=设计正确） |
| D8 | `speak(prompt_id)` | 语音（无 PCM 素材时文本兜底） | ✅ 通过（mode=text_fallback，不假装播过；PCM 素材合成后即真出声） |

> D7 两项"跑通"的标准 = **正确地拒绝**。这是设计的安全行为，返回 succeeded 反而说明出了问题。
> D8 的文本兜底也是设计行为；要真出声需先合成 PCM 素材（命令在 `sdk_wrapper/dev/robot_tools/audio.py` 文件头）。

## 一、上真机前的准备（顺序执行）

```bash
# 1. WSL 里先 mock 干跑一遍，确认脚本本身是好的（任何时候都安全）
cd ~/galbot_hackathon
python3 scripts/test_wrapper_real.py --mock --yes --part base
python3 scripts/test_wrapper_real.py --mock --yes --part dev

# 2. 部署最新代码到机器人 HPU（IP 变了就传参）
bash scripts/deploy_hpu.sh            # 默认 192.168.166.7

# 3. SSH 上机器人
ssh galbot@192.168.166.7
cd ~/galbot_hackathon && source scripts/env_hpu.sh
```

**安全检查（每轮都做）**：
- 机器人周围 **2 米空旷**，无桌角/线缆/台阶
- 急停按钮在手边，另一人陪护；**背部红色急停按钮确认已旋开弹出**（自锁，按下会锁定全部运动）
- 机器人电量足够（低电跑一半断电很危险）
- G1 站立姿态稳定（如果是坐姿先扶起来）

**整机工作模式检查（2026-11-07 踩坑后新增，必做）**：

机器人"能连上但完全不动"时，先查整机模式，别怀疑代码：
```bash
ssh galbot@<HPU_IP>            # 密码 gb@2023
ssh root@192.168.1.66         # 从 HPU 跳 XCU，密码 12345678
cd /data/bin && ./sys_tool    # XCU 系统工具
# 交互界面里输入 q → 应显示 mode_type: WORKING_MODE
```
- 若显示 `ABNORMAL`：查 `/userdata/log/mission_manager/mission_manager.INFO` 搜 `abnormal_service` 定位异常服务，恢复后再跑（重启机器人可清掉大部分异常模式）。
- **直控底盘（set_base_pose）不需要建图/定位**，但 ABNORMAL 模式会一刀切拒动——这是整机保护，与走哪条控制路径无关。
- 只有**导航**（GalbotNavigation / navigate_to_place）才需要定位 + 地图 + service_navigation_plan。
- 排查分工记忆：**能连不动 → 查 XCU（singorix/底盘/模式）；能动但地图/避障异常 → 查 HPU（localization/navigation）**。

## 二、Part base 实测（第一遍，只测底盘 API）

```bash
python3 scripts/test_wrapper_real.py --part base
```

每步会先打印预期结果，Enter 执行 / 输 s 跳过 / Ctrl+C 中止。推荐流程：

| 步 | 判定标准 | 不过怎么办 |
|---|---|---|
| B1 | 日志『机器人实例已初始化 (mock=False)』+『底盘控制器已切换到 CHASSIS_POSE_CTRL』 | 挂在 import：`source scripts/env_hpu.sh` 没做；挂在 init：机器人开机状态/网络 |
| B2 | 日志『导航实例已初始化』，可能先看到补启 service_navigation_plan | WAIT_INITIALIZED 反复出现 → 导航服务没起来，看下方排障表 |
| B3 | check_baseline 返回 dict；read_sensor 抛 NotImplementedError = **通过**（缺口已文档化） | — |
| B4 | clamp(3.0)=2.0m，限幅值与 safety_config.json 一致 | — |
| B5 | `{'success': True}` | — |
| B6/B7 | 前进 ~10cm 再退回，肉眼核对位移量级对 | 只动一半：正常（里程计误差）；完全不动：见排障表 |
| B8 | 左转 15° 再转回，朝向复原 | — |
| B9 | 返回 True | — |

## 三、Part dev 实测（第二遍，独立进程！）

```bash
python3 scripts/test_wrapper_real.py --part dev
```

> ⚠️ **必须和 base 分开跑**：两套封装各自持有 SDK 单例，同进程会互踩（纪律④）。

| 步 | 判定标准 | 不过怎么办 |
|---|---|---|
| D1 | `rejected + permission_denied`（零风险，不碰 SDK） | 挂了说明闸门代码被改坏 |
| D2 | `succeeded`，motion_state=idle | 报 unknown：先看机器人实际状态，unknown ≠ 已停止 |
| D3 | capture 返回 capture_id；analyze 给目标+距离。**not_ready = 该机位未装配**，换 `--camera` 重试（cameras.json 有 10 个机位） | 全部机位 not_ready：相机服务没起 |
| D4 | pose 7 个数，量级 ~0.4-0.6 m | — |
| D5 | 夹爪真实开合两次 | — |
| D6 | 左臂沿 x+ 平 2cm 再回原位 | 动作前确认手臂活动范围内没人和物 |
| D7 | **两项都 rejected = 通过** | 返回 succeeded 要立刻查代码 |
| D8 | 无素材时 mode=text_fallback | 想真出声：先按 `robot_tools/audio.py` 文件头的 PowerShell 命令合成 PCM 放进 `sdk_wrapper/dev/assets/audio/` |

跑完 base + dev 全 PASS，wrapper 封装就算真机全量跑通了。

## 四、排障速查

| 症状 | 原因 | 处理 |
|---|---|---|
| set_base_pose TIMEOUT 且机器人**零位移**（2026-11-07 实测） | 整机进入 ABNORMAL 模式（如同伴调试定位服务触发 localization 异常），整机拒动保护 | XCU `sys_tool`→`q` 查模式；查 mission_manager.INFO 的 abnormal_service；重启机器人恢复 WORKING_MODE 后重试。**不是代码问题** |
| B6-B8 `status=TIMEOUT` 但 `success=True`（旧版脚本输出） | 旧版 wrapper 用 `bool(res)` 判断，pybind 枚举恒真，把 TIMEOUT 当成了成功（2026-11-07 真机实测踩坑，已修复：如实上报 + 30s 超时 + 重试 1 次） | `git pull` 后重新部署再跑；修复后 TIMEOUT 会如实显示 FAIL，同时观察机器人是否实际移动了 |
| `ModuleNotFoundError: galbot_sdk` | 没source环境 | `source scripts/env_hpu.sh`（HPU）/ `source scripts/env.sh`（WSL） |
| 导航一直 `WAIT_INITIALIZED` | service_navigation_plan 没起 | 脚本会自动补启 + 等 2s；仍不行手动 `pgrep -f service_navigation_plan` 检查 |
| `切换底盘控制器失败` | 机器人控制器被别的程序占用（如示教器） | 关掉占用方，重启测试进程 |
| set_base_pose 返回非 SUCCESS 但机器人没动 | 控制器切换后未就绪 | wrapper 已内置 2s 等待 + 重试；持续失败重启进程 |
| 程序异常退出后机器人还"卡" | SDK 单例没清理 | 重跑 `wrapper.shutdown()` 或直接重启进程（destroy 后同进程不能再 init） |
| 相机 not_ready | 该机位未装配 | 属正常，换机位 |

## 五、已知缺口（负责人 A 的 TODO，测通≠补完）

1. `read_sensor()`：只留了接口，真机抛 NotImplementedError。相机数据走 dev 层 PerceptionTool。
2. `baseline.reset_to_baseline()`：只停底盘；手臂折叠位、夹爪释放未接。
3. `execute_gesture`：需先示教关节角，登记到 `sdk_wrapper/dev/config/gestures.json`。
4. `navigate_to_place`：需真机实测各目标点位姿，填进 `sdk_wrapper/dev/config/places.json` 并把 `verified` 改 true。
5. 语音 PCM 素材未合成（合成命令在 `sdk_wrapper/dev/robot_tools/audio.py` 文件头）。
