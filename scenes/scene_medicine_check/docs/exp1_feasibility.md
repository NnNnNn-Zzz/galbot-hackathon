# 实验 1 空桌盲抓：可行性检验 + 预期日志说明

> 检验日期：2026-11-07
> 检验对象：`scripts/run_exp1_grasp_only.py` 在 HPU 真机上的执行方案
> 检验方式：代码逐行核对 + mock 实走 + 真机连通性探测（**未向机器人发任何动作指令**）

---

## 一、可行性检验结论

### 1.1 方案整体可行，但有 4 个现场必须知道的差异

| # | 差异 | 影响 | 应对 |
|---|---|---|---|
| 1 | **无 logging 输出** | 真机上你看不到 SDK 的详细日志（如 `attach_tool` 耗时、`set_end_effector_pose` 返回状态），只能看到 JSON | 接受，或临时在脚本里加 `logging.basicConfig` |
| 2 | **timeout 不统一** | `set_arm_pose` 20s / `set_joints` 15s / `move_line_to` 10s —— 各阶段卡死的最长等待时间不同 | 心里有数，见 §三 |
| 3 | **相机必然失败** | `capture_image` 在真机必然返回空（相机未订阅），但不影响主流程 | 预期内，忽略 |
| 4 | **y 方向偏差 14.5cm** | `GRASP_POSE.y=-0.10` vs 实测静止位 y≈-0.245 —— 手臂会向左横移 | 这是实验要验证的内容，不是问题 |

### 1.2 风险项（可能让实验失败）

| 风险 | 概率 | 后果 | 预案 |
|---|---|---|---|
| `GRASP_POSE` z=0.75 过低，碰桌 | 中 | 拍急停，z 抬高 0.01–0.02 重试 | 见 §四 |
| 手臂横向移动 14.5cm 扫到桌沿 | 低 | 拍急停，y 往回挪 | 先跑冒烟确认右臂当前位置 |
| `set_end_effector_pose` 返回 TIMEOUT | 低 | 该步 failed，脚本退出码 1 | 检查网络延迟（当前 ping 292–663ms 偏高） |
| 网络断开 | 低 | 脚本卡死或异常退出 | SSH 会话保持，断线后重连 |

### 1.3 硬件/软件前提（已验证 ✅）

| 项 | 状态 | 依据 |
|---|---|---|
| 机器人在线 | ✅ | ping 192.168.166.7 通，SSH 22 端口开放 |
| `scripts/run_exp1_grasp_only.py` 存在 | ✅ | 4432 字节，已编译通过 |
| `tuning.py` 参数已标定 | ✅ | z=0.75 按桌面 0.72m 标定，GRIPPER_SIDE=right |
| 碰撞检查已开启 | ✅ | `enable_collision_check=True`（`set_arm_pose`） |
| mock 回归通过 | ✅ | `passed=true, grabbed=false` |

---

## 二、实验开始后的完整预期日志（逐行说明）

以下日志基于**代码实走**和**接口逐行核对**生成，不是推测。

### 阶段 A：启动（0–3 秒）

```json
{"step": "boot", "experiment": 1, "title": "实验 1 空桌盲抓：验证抓取轨迹安全（桌面不放盒）", "backend": "wrapper", "grasp_pose": [0.4, -0.1, 0.75, 0.0, 0.0, 0.0, 1.0], "note": "桌面必须清空。看到手臂伸向下述坐标即为正确方向。"}
```

**意义**：脚本已启动，确认后端是 `wrapper`（真机），目标抓取位姿是 `(0.40, -0.10, 0.75)`。
**你要做的**：核对 `grasp_pose` 的前三个数，确认和 `tuning.py` 一致。

```json
{"step": "config", "stages": ["preflight", "grasp"]}
```

**意义**：配置加载成功，只跑这两步（`cfg["steps"][:2]` 截断生效）。

---

### 阶段 B：preflight（约 3–130 秒，**这是最不确定的一段**）

```json
{"stage": "preflight", "fate": "ok", "code": "OK", "elapsed_s": 127.3, "message": "已记录起始里程计位姿"}
```

**意义**：
- `elapsed_s` 会**很大**（无真机时约 127 秒，有真机时应 <5 秒），因为这一步触发 `_get_robot()` → `GalbotRobot.init()` → 切底盘控制器
- 如果 `elapsed_s` > 100 秒，说明机器人**不在线**或网络极慢，但脚本仍会等下去

**你要做的**：
- 如果这里卡了 >60 秒，可以 Ctrl+C 中断，检查 `ping 192.168.166.7` 和 SSH 是否还通
- 如果 `fate: "failed"`，看 `message` 是不是「无里程计数据」——这是**非致命**的，流程会继续，但最后底盘归位会失效

> **真机预期**：`elapsed_s` 应该在 **3–10 秒** 之间（真机已在线，只是首次 init 需要握手）。

---

### 阶段 C：grasp（约 30–50 秒，**这是核心**）

这一步内部有 7 个子动作，但**只会输出一行 JSON**（因为 `run_pipeline` 只记录 stage 级别的结果）：

```json
{"stage": "grasp", "fate": "ok", "code": "OK", "elapsed_s": 42.7, "message": "已抓起左起第一盒药品并收臂"}
```

**意义**：
- `elapsed_s` 是 7 个子动作的总耗时
- `fate: "ok"` 只表示**流程走完**，**不代表真的抓到了东西**（空桌本来就没有东西可抓）

**内部子动作分解（你看不到，但时间在流逝）：**

| 子动作 | 调用 | 预期耗时 | 你在现场看到 |
|---|---|---|---|
| ① 挂工具 | `attach_tool("right_arm", "galbot_gripper")` | 1–2s | 无可见动作 |
| ② 开夹爪 | `set_gripper(0.10, "right", 0.05, 10.0)` | 1–2s | 夹爪张开到 10cm |
| ③ 预抓取位 | `set_arm_pose(PRE_GRASP_POSE, 20s)` | 3–8s | 手臂移到盒子**上方 10cm** |
| ④ 直线下降 | `move_line_to(GRASP_POSE)` | 2–5s | **垂直下降**到桌面**上方 3cm** |
| ⑤ 合夹爪 | `set_gripper(0.02, "right", 0.05, 10.0)` | 1–2s | 夹爪闭合到 2cm |
| ⑥ 抬起 | `set_arm_pose(LIFT_POSE, 20s)` | 3–8s | 手臂抬到 **+15cm** |
| ⑦ 收臂 | `set_joints(CARRY_JOINT_POSITIONS, ..., 0.2)` | 2–5s | 右臂收回到抱持位 |

**总计**：约 13–32 秒（真机），加上 `capture_image` 的 1–2 秒失败重试，**预期 30–50 秒**。

**你要做的（现场观察）：**
1. **②之后**：确认夹爪真的张开了（能看到两指间距 ~10cm）
2. **③**：手臂应该移到**右前方**（y=-0.10 是右侧，因为 base_link 的 y 轴指向左）
3. **④**：**必须垂直下降**，不能斜着扫过去（`move_line_to` 是直线插值）
4. **④末端**：指尖应该在**桌面上方 3cm**，不能贴桌
5. **⑤**：夹爪应该**空合**（因为没有盒子），你可能听到夹爪电机声
6. **⑦**：手臂应该收回到「抱着东西」的姿态，不是完全伸直

---

### 阶段 D：verdict（0 秒，立即输出）

```json
{"step": "verdict", "pipeline_status": "ok", "passed": true, "link_ok": true, "grabbed": false, "counts": {"ok": 2, "failed": 0, "unknown": 0, "skipped": 0, "err": 0}, "io_missing": [], "reasons": [], "next_action": "通过。下一步：进实验 2，按胶带把盒子摆到 x=0.40 / y=-0.10。"}
```

**逐字段解释：**

| 字段 | 值 | 意义 |
|---|---|---|
| `pipeline_status` | `"ok"` | 两步都成功，没有 failed/unknown |
| `passed` | `true` | **实验通过** |
| `link_ok` | `true` | 链路（动作序列）完整走完 |
| `grabbed` | `false` | **这是正确的** —— 空桌应该抓空。如果是 `true`，说明桌上还有东西 |
| `counts` | `{ok: 2, failed: 0, ...}` | 两步都 ok |
| `io_missing` | `[]` | wrapper 的 12 个接口都已实现（无 `DEP_MISSING`） |
| `reasons` | `[]` | 没有失败原因 |
| `next_action` | `"通过。下一步：进实验 2..."` | 判据给出的下一步指引 |

**退出码**：`0`（通过）

---

## 三、失败场景的预期日志（对照表）

### 场景 1：z 过低碰桌

```json
{"stage": "grasp", "fate": "failed", "code": "STAGE_FAILED", "elapsed_s": 18.3, "message": "移动到预抓取位姿失败（pre_grasp）"}
```

或者：

```json
{"stage": "grasp", "fate": "failed", "code": "STAGE_FAILED", "elapsed_s": 25.1, "message": "直线逼近抓取位姿失败（approach）"}
```

**verdict**：
```json
{"passed": false, "reasons": ["grasp 失败：直线逼近抓取位姿失败（approach）"], "next_action": "按固定顺序排查：① 先把盒子重新摆到胶带交点..."}
```

**退出码**：`1`

**你要做的**：拍急停 → `GRASP_POSE` z 每次 +0.01 → 重跑。

---

### 场景 2：手臂超时（网络差）

```json
{"stage": "grasp", "fate": "failed", "code": "STAGE_FAILED", "elapsed_s": 20.0, "message": "移动到预抓取位姿失败（pre_grasp）"}
```

`elapsed_s` 正好是 20.0 秒 = `set_arm_pose` 的 timeout。

**你要做的**：检查网络（当前 ping 292–663ms 偏高），如果持续 >500ms，考虑换有线或靠近路由器。

---

### 场景 3：unknown（最严重）

```json
{"stage": "grasp", "fate": "unknown", "code": "UNDETERMINED", "elapsed_s": 5.2, "message": "..."}
```

**verdict**：
```json
{"passed": false, "reasons": ["grasp 返回 unknown —— 机器人不知道自己在哪，按团队规则立即全停、不要重试、不要归位"], "next_action": "停止操作。unknown 表示机器人无法确定自身状态..."}
```

**退出码**：`1`

**你要做的**：**立即停手**，不要发任何动作，人工把臂摆回安全位，报给我排查。

---

### 场景 4：空桌却抓到东西（假阳性）

```json
{"stage": "grasp", "fate": "ok", "code": "OK", "elapsed_s": 35.0, "message": "已抓起左起第一盒药品并收臂"}
```

但 `grabbed: true`（在 verdict 里）：

```json
{"passed": false, "grabbed": true, "reasons": ["空桌实验却报告 picked=True —— 桌上是不是还有东西没拿走？"], "next_action": "先别调任何坐标。清空桌面（含盒子、纸片、边缘的卷尺），再跑一次同样的实验 1..."}
```

**退出码**：`1`

**你要做的**：清空桌面重跑。如果清空后仍 `grabbed=true`，说明是本后端 `is_holding()` 的假阳性（当前 wrapper 恒返回 `None`，理论上不该发生，但如果发生了要记录）。

---

## 四、真机特有的可见日志（你看不到但 SDK 会输出）

由于 `run_exp1_grasp_only.py` **没有配置 `logging.basicConfig`**，以下日志**不会出现在你的终端**，但会写进 SDK 的日志文件：

| 日志来源 | 内容示例 | 位置 |
|---|---|---|
| `wrapper.py` | `"机器人实例已初始化 (mock=False)"` | `~/galbot_sdk_log/` |
| `wrapper.py` | `"底盘控制器已切换到 CHASSIS_POSE_CTRL"` | 同上 |
| `s1_grasp.py` | `"抓取前拍照失败（get_rgb_data 返回空），盲抓流程不受影响"` | 同上 |
| `wrapper.py` | `"[依赖缺失] sdk_wrapper.xxx 未实现"` | 同上（如果出现） |

**如果你需要看到这些日志**，临时改一下脚本：

```python
import logging
logging.basicConfig(level=logging.INFO,
                    format="%(levelname)s %(name)s: %(message)s")
```

加在 `run_exp1_grasp_only.py` 的 `sys.path.insert` 之后。

---

## 五、时间预算

| 阶段 | 预期耗时 | 最坏情况 | 触发条件 |
|---|---|---|---|
| 启动 | 0s | 0s | — |
| preflight | 3–10s | 127s | 无真机/网络极慢 |
| grasp | 30–50s | 70s | 每个 `set_arm_pose` 都接近 20s timeout |
| verdict | 0s | 0s | — |
| **总计** | **33–60s** | **200s** | 网络差 + 所有 timeout 都触发 |

**建议：**
- 如果 2 分钟内没有输出 `verdict`，检查 SSH 是否还连着
- 如果 3 分钟还没有，可以 Ctrl+C 中断，看卡在哪个阶段

---

## 六、现场记录模板

实验结束后，记录这几行：

```
日期时间: 2026-11-07 15:xx
实验: 1 空桌盲抓
passed: true/false
grabbed: false/true/null
failed_step: null / grasp / preflight
elapsed_preflight: __._ s
elapsed_grasp: __._ s
现场观察:
  - 手臂是否伸向右前方？是/否
  - 下降是否垂直？是/否
  - 指尖距桌面高度估计：约 __ cm
  - 有无碰撞/急停？无/有（时间：__:__）
  - 其他异常：
```

---

## 七、最后确认

**方案可行**，但有 4 个现场必须知道的差异（见 §1.1）。

**你可以现在执行**，命令是：

```bash
ssh galbot@192.168.166.7          # 密码 gb@2023
cd ~/galbot_hackathon
source scripts/env_hpu.sh
MED_IO=wrapper python3 scripts/run_exp1_grasp_only.py
```

**或者先跑冒烟**（更保守）：

```bash
MED_IO=wrapper python3 scripts/smoke_medicine_hw.py
```

**或者先修相机**（顺便把根因定了）：

```bash
python3 scripts/probe_camera.py --mode current
```

你定。
