# 合并流程：腰回正 → 腰高 → run_sequence → 张爪后退 → 抬手程序

一条命令跑完五个原本分散的程序。合并只固定**调用顺序和前置条件**，每个子脚本的运动逻辑一字未改。

## 一、五步在干什么

| 步 | 动作 | 脚本 | 会不会动机器人 |
|---|---|---|---|
| ① | 腰部回正：`leg_joint4` 偏航 / `leg_joint5` 侧倾 → 0° | `scripts/set_waist_upright.py` | 会（≤4°/步，0.05 rad/s） |
| ② | 腰高设到抬手程序规定的值（默认 1.3904 m） | `scripts/set_waist_height.py` | 会（回读闭环，最多 3 次补发） |
| ③ | **run_sequence 全序列** | `~/run_sequence.py` | 会（整套 8 小步） |
| ④ | 夹爪张到最大（12 cm）→ 沿夹爪朝向后退 15 cm | `scripts/gripper_open_retreat.py` | 会 |
| ⑤ | 抬手程序：右臂 7 关节分步回放（默认倒序） | `scripts/replay_joint_waypoints.py` | 会 |

### 第③步 run_sequence 内部做什么

| # | 动作 |
|---|---|
| 1 | 右夹爪开到 4 cm |
| 2 | 末端转到 `Rx(-90°)`：夹爪水平向前 + 内旋 90°（位置不变） |
| 3 | 视觉标定：以 `~/ref_handle.jpg` 为基准，只平移把腕部画面对齐（姿态全程锁死） |
| 4 | 人工确认（编排器已自动带 `--yes` 跳过） |
| 5 | 沿夹爪前进 5 cm × 5 次 |
| 6 | 夹爪开到 2 cm |
| 7 | 沿反方向**连续拉出 27 cm**（一次调用） |
| 8 | 夹爪开到 6 cm |

它**依赖同目录的 `wrist_align.py` 和基准图 `~/ref_handle.jpg`**，所以只能放 home 跑，不适合挪进 `scripts/`。编排器启动时会检查这两个依赖。

### 为什么必须是这个顺序

- **先回正、再设腰高**：回正会改变 `head_base_link` 的高度读数，反过来设完高度会被回正动作带偏。
- **第③步摆姿态，第④步才有意义**：run_sequence 第 2 小步会把夹爪轴摆成纯 `[1,0,0]`（水平向前），第 7 小步沿这个轴拉 27 cm。第④步紧接着沿**同一个轴**再退 15 cm，所以是**接近纯水平的退**。如果跳过第③步直接跑第④步、而当前姿态竖直，那"后退 15 cm"就变成"上抬 15 cm"（详见第五节）。
- **抬手程序最后**：它只下发 `right_arm_joint1..7`，不动腰。采样序列是在「腰高 1.3904 m」前提下录的，前面必须把腰摆回去，否则同样的关节角末端位置对不上。

### 为什么是分进程而不是一个 Python 进程

每个子脚本都自建 `gm.GalbotMotion()` / `gm.GalbotRobot()`，跑完在 `finally` 里 `request_shutdown / wait_for_shutdown / destroy`。**SDK 单例 destroy 之后在同一进程里重新 init 会失败** —— 2026-11-07 架构合并时踩过双单例同进程互踩（wrapper 与 dev adapter 各持一套），跑一次崩一次。

所以编排器用 `subprocess` 逐条串行：每步开一个干净进程，跑完销毁。好处是这些脚本各自真机验证过的行为原样保留，不重新发明一遍运动逻辑。

## 二、怎么跑

```bash
# 0. 每次新开 ssh 都要先过这两句
ssh g1
cd ~/galbot_hackathon_dev
source ~/galbot_hackathon/scripts/env_hpu.sh

# 1. 先空跑一遍（只读，机器人不动；第③步会跳过，见下）
python3 -u scripts/run_merged_program.py --dry-run

# 2. 真跑（交互终端会等你确认）
python3 -u scripts/run_merged_program.py

# 3. 无人值守 / 从 Windows ssh 一条命令跑（必须带 --yes）
python3 -u scripts/run_merged_program.py --yes

# 4. 腰已就位，只跑后三步
python3 -u scripts/run_merged_program.py --skip 1,2

# 5. 只跑第③步（单独调 run_sequence）
python3 -u scripts/run_merged_program.py --skip 1,2,4,5 --yes
```

从 Windows 侧一条命令跑（免密已配好）：

```bash
wsl -d Ubuntu-22.04-GalbotG1 -e bash -c 'ssh g1 "cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh >/dev/null 2>&1; python3 -u scripts/run_merged_program.py --dry-run"'
```

**注意：`--dry-run` 不覆盖第③步。** `run_sequence.py` 没有 dry-run 模式，一跑就会动，所以空跑时整步跳过并在输出里标注。这意味着 dry-run 只能验证 ①②④⑤ 四条链路，第③步要真跑才知道。

## 三、参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--dry-run` | 关 | 空跑（跳过第③步，其余四步不下发运动指令） |
| `--yes` | 关 | 免确认；**非交互环境（ssh）真跑必须带** |
| `--waist-height` | 自动 | 显式指定腰高（米）；不给就从序列文件读 |
| `--seq` | `~/joint_records/right_arm_sequence.json` | 抬手程序的序列文件（腰高和回放点都从这里来） |
| `--retreat` | `0.15` | 第④步后退距离（米），上限 0.30 |
| `--open-cm` | `12.0` | 第④步夹爪目标开度（cm），顶到物理限位约 10.x cm |
| `--sequence-script` | `~/run_sequence.py` | 第③步脚本路径 |
| `--sequence-args` | 空 | 给第③步的额外参数，如 `"--pull 0.20 --fwd 4"` |
| `--order` | 空（倒序） | 第⑤步回放顺序，如 `seq01,seq02` |
| `--skip` | 空 | 跳过步骤，如 `1,2` |
| `--log` | 空 | 全过程输出同时落盘，如 `--log ~/run_$(date +%m%d_%H%M).log` |
| `--force` | 关 | 不打印"别的项目脚本在跑"的提示 |
| `-v` | 关 | 打印 PYTHONPATH / LD_LIBRARY_PATH |

`run_sequence` 自己的参数（`--step` / `--fwd` / `--pull` / `--ref` / `--servo-iter` / `--max-tilt` / `--max-roll-dev`）通过 `--sequence-args` 传，不用改它本身。

## 四、腰高是从哪来的

优先级：

1. `--waist-height` 显式给值
2. `~/joint_records/right_arm_sequence.json` **首点**（seq01）的 `waist.height_m`
   —— 首点既是录制起点，也是倒序回放的终点，所以是"抬手程序规定的腰高"
3. `~/joint_records/waist_baseline.json` 的 `height_m`

序列内各点腰高不一致时会打提醒。当前实测：`seq01~seq07 = 1.3904`，`seq08 = 1.3952`。

## 五、当前真机状态（2026-11-08 13:5x dry-run 实测）

| 项 | 值 |
|---|---|
| `leg_joint4` 偏航 | **+2.039°**（需回正，1 步 2.04° 就到位） |
| `leg_joint5` 侧倾 | −0.198°（已在容差 0.8° 内，不动） |
| 腰高 | 1.3902 m，目标 1.3904 m，差 **+0.2 mm**（容差 1.5 mm，不会动） |
| 末端 xyz | `[0.5434, −0.2077, 1.2338]` |
| 夹爪轴（**第③步之前**） | `[0.098, 0.063, −0.993]`，偏离水平 **83.3°（几乎竖直朝下）** |
| 夹爪开度 | 12.00 cm（已全开） |
| 抬手程序 | 8 段需 23+19+16+14+12+11+5+1 ≈ **101 步** |

### ⚠ 三点必须知道

**1. dry-run 里第④步会警告"上抬 14.9 cm"，那是第③步还没跑的状态。**
现在夹爪轴竖直朝下（z = −0.993），"沿轴后退"就是往上抬。真跑时 run_sequence 第 2 小步会先把姿态摆成 `Rx(-90°)`，夹爪轴变水平，第④步就成了**纯水平后退**。dry-run 跳过第③步，看到的警告反映的是"假如不跑第③步直接退"的情形 —— 那正是 `--skip 1,2,3` 时的真实风险，别那么跳。

**2. 抬手程序真跑约 101 步。**
单步 ≤8°、0.15 rad/s、每步回读，全程大概几分钟。`seq08` 目标与当前位置差 **182.67°**，第一段最长（23 步），别以为卡住了。

**3. 第③步是整套慢动作。**
视觉标定最多 12 次迭代、前进 5 次、拉出 27 cm（内部 6 步，历史上 6 步里有 3 步首次 FAULT 会自动重试）。整体几分钟量级，比另外四步加起来都长。

## 六、失败了怎么办

编排器 fail-fast：某步返回码非 0 就停，不再动后续步骤，并打印该步的排查建议。

| 步 | 常见现象 | 处理 |
|---|---|---|
| ① | 报 FAULT 且回读没到位 | 先 `probe_waist.py` 只读看一眼；确认不是 XCU `ABNORMAL` 模式（会一刀切拒动） |
| ② | `set_leg_height` 假 FAULT | 脚本已按回读 Z 判定 + 3 次补发；若差几毫米不动，属 G-22 类 |
| ③ | 视觉标定不收敛（返回 3） | 会打警告继续跑，属既有设计。想更贴就 `--sequence-args "--servo-iter 20"` |
| ③ | 硬失败退出（返回 1） | 多是画面里找不到红把手 —— 先用 `jog_ee.py` 把把手摆进视野 |
| ③ | 拉出段中途 FAULT | 历史常态，adapter 自动重试（2 次直线 + 1 次绕行）。6 步里 3 步重试属正常 |
| ④ | 张爪 TIMEOUT | 顶到物理限位，正常。实测宽度 <8 cm 才是真没张开（夹住了东西） |
| ④ | 后退失败 | 重试 3 次仍失败看打印的起点/目标/当前，多半姿态在 run_sequence 之后又漂了 |
| ⑤ | 回读与命令差 >3° | 脚本主动停。别强推，先手动确认手臂无干涉 |
| 任意 | SDK init 失败 | 有别的进程占着 SDK。`pgrep -af python3` 找出来关掉（尤其 `keyboard_leg_height.py` / `gripper_ctrl.py` / `robot_side/server.py`） |

## 七、文件清单

| 文件 | 作用 |
|---|---|
| `scripts/run_merged_program.py` | **总入口**（纯编排，不含运动逻辑） |
| `scripts/gripper_open_retreat.py` | 第④步实体动作 |
| `~/run_sequence.py` + `~/wrist_align.py` + `~/ref_handle.jpg` | 第③步（既有脚本，未改动） |
| `scripts/set_waist_upright.py` | 第①步（顺带修了 dry-run 模拟不打三遍） |
| `scripts/set_waist_height.py` | 第②步 |
| `scripts/replay_joint_waypoints.py` | 第⑤步「抬手程序」 |
| `scripts/record_right_arm_joints.py` | 配套：只读采集（扩序列用） |
| `scripts/probe_waist.py` | 配套：腰部只读探测 |
| `scripts/probe_pull.py` | 配套：沿轴后退 IK 只读预检（本流程已不用，留着调试） |

`scripts/` 下的都在 `~/galbot_hackathon`（git，origin = `github.com/NnNnNn-Zzz/galbot-hackathon`），
部署副本在 `~/galbot_hackathon_dev/scripts/`。第③步的三个文件只在机器人 home，不在 git 里。

---

# 第二段：绿块两级对准 + 夹紧 + 抬回 1.327 m

`scripts/run_second_stage.py`，跑在第一段五步全部完成**之后**。

## 七步

| 步 | 动作 | 脚本 |
|---|---|---|
| ① | 腰高 → **1.3270 m** | `set_waist_height.py --target 1.3270` |
| ② | 绿块视觉对准 @1.327（`--force`） | `align_box_1327.py` |
| ③ | 腰高 → **1.2337 m**（竖直下降） | `set_waist_height.py --target 1.2337` |
| ④ | 绿块视觉对准 @1.234（`--force`） | `align_box_1234.py` |
| ⑤ | 腰高 → **1.125 m**（继续竖直下降） | `set_waist_height.py --target 1.125` |
| ⑥ | 夹爪抓紧到 **7 cm** | `gripper_ctrl.py --seq 7 --effort 10` |
| ⑦ | 腰高 → **1.3270 m**（竖直上升） | `set_waist_height.py --target 1.3270` |

腰高轨迹：`当前 → 1.327 → 1.234 → 1.125 → 1.327`

## 三个设计决定

**1. 第①步先把腰高设到 1.327，再跑 align_box_1327。**
两个 align 脚本的标准档案（画面中心/长边/倾角）各自在对应腰高采的：
1.327 档是 `中心(681.54, 383.25) 倾角+1.10°`，1.234 档是 `中心(686.34, 481.38) 倾角+0.78°`。
腰高不对时"绿块该在画面哪个位置"本身就变了，在别的腰高强跑等于让手臂追一个不存在的构图。
所以是「先到位、再对准」；`--force` 只是容差保险，不是"随便什么高度都能对"。

**2. 升降用 `set_waist_height.py`，不用 `keyboard_leg_height.py`。**
后者是键盘遥控（靠字符流判定"还在按"），没法在编排里定到某个高度。
`set_waist_height.py` 走的是同一个 `set_leg_height`（腰部直上直下，不产生偏航/滚转副作用），
但带「回读闭环 + 最多 3 次补发」，正好能"降到 1.234 附近"。

**3. 升降时长按行程放大。**
1.3904 → 1.2337 是 157 mm，SDK 默认 3 s 就是 5.2 cm/s。编排默认给 `--duration 5`（≈3.1 cm/s）更稳。

## 怎么跑

```bash
cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh

python3 -u scripts/run_second_stage.py --dry-run    # 七步全空跑
python3 -u scripts/run_second_stage.py --yes        # 真跑（ssh 必须带 --yes）
python3 -u scripts/run_second_stage.py --yes --skip 1,2
python3 -u scripts/run_second_stage.py --yes --gripper-cm 6.5 --duration 6
python3 -u scripts/run_second_stage.py --yes --no-force-align   # 不给 align 加 --force
```

**和第一段不同：七步全都支持空跑**（`set_waist_height` / `align_green_box` 有 `--dry-run`，
`gripper_ctrl` 有 `--dry`），所以 `--dry-run` 能完整验证整条链。

## 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--h-align-hi` | `1.3270` | 高档对准腰高（align_box_1327 的标准） |
| `--h-align-lo` | `1.2337` | 低档对准腰高（align_box_1234 的标准） |
| `--h-grip` | `1.125` | 夹爪动作高度 |
| `--gripper-cm` | `7.0` | 夹爪目标开度 |
| `--gripper-effort` | `10.0` | 夹爪力度 N·m |
| `--duration` | `5.0` | 每次升降时长（秒） |
| `--no-force-align` | 关 | 不给 align 加 `--force` |
| `--skip` / `--dry-run` / `--yes` / `--force` / `--log` / `-v` | — | 同第一段 |

## 失败排查

| 步 | 现象 | 处理 |
|---|---|---|
| ①③⑤⑦ | 假 FAULT | 脚本按回读 Z 判定 + 3 次补发，看最终高度即可 |
| ①③⑤⑦ | 高度没变且状态异常 | 可能被安全限制拒绝，`probe_waist.py` 只读确认 |
| ②④ | 返回 3 | 多为「腰高不符 / 画面里没找到绿块」，先 `--dry-run` 看差距 |
| ②④ | 腰高到位仍返回 3 | 检查相机是否被挡住、绿块是否还在画面里 |
| ⑥ | TIMEOUT | 多半是夹住绿块到不了 7 cm，看它打印的实测宽度；抓空会合到最小间距 |

⚠ `align_box_*` 只看画面构图，**不保证物理夹持位置**；第⑥步夹紧才是真实接触。
如果夹完发现位置不对，要调的是对齐容差而不是夹爪参数。
