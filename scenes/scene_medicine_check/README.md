# scene_medicine_check - 药品管理（过期药品识别与清理）

- **负责人**：卓杭、韩桐欣
- **触发时机**：主 Agent 收到「检查药箱 / 清理过期药」指令时调用
- **依赖**：`sdk_wrapper`（硬件）+ 独立感知服务（drawer 模式：YOLO 检测 + 手写日期识别）

## 两种环境

| mode | 环境 | 流程 |
|---|---|---|
| `drawer`（默认） | 桌上固定小抽屉，内放 999 与维生素各一件 | 对位 → 拉开抽屉 → 逐件取出 → 拍照识别 → 判定 → 放回/丢弃 → 推回抽屉 |
| `table`（基线） | 药品直接摆在桌面固定位置 | 抓取最左一盒 → 播报 → 转身 180° → 直行 3m → 右转 30° → 投放 → 归位 |

比赛现场抽屉做不出来时用 `table`，行为等价于基线描述。
**table 模式不用视觉、不读日期**：日期识别是可插拔的升级项（见下节）。

---

## table 基线：可插拔管线

```
scene.py (mode=table)
   └─ baseline_flow.run(params)          ← 对外契约不变
        └─ pipeline/runner.py            ← 唯一控制流（ok / failed / unknown）
             ├─ pipeline/configs/*.json  ← 步骤顺序与参数，改这里不改代码
             ├─ pipeline/stages/         ← 内置基线动作块，每块一个文件
             ├─ pipeline/policies/       ← 队友的替换实现，自动发现
             └─ pipeline/io.py           ← 硬件访问层（wrapper / mock，只在这里切换）
```

### 文件清单

| 文件 | 作用 |
|---|---|
| `baseline_flow.py` | 入口壳：读配置、跑管线、拼返回；`plan()` 打印编排 |
| `pipeline/contracts.py` | 冻结契约：`StageResult` / `Context` / `Stage` |
| `pipeline/runner.py` | 控制流：按配置执行、记 trace、判 unknown |
| `pipeline/registry.py` | 注册表：`@register_stage(name)`，自动发现 stages 与 policies |
| `pipeline/io.py` | `WrapperIO`（转发 sdk_wrapper，缺失软降级）/ `MockIO`（纯内存 + 故障注入） |
| `pipeline/tuning.py` | 现场标定常量（位姿、距离、角度、关节初值） |
| `pipeline/stages/s0..s6` | preflight / grasp / announce / turn / drive / drop / return_home |
| `pipeline/policies/expiry_demo.py` | 示例升级版：读日期 → 判过期 → 取最左 → 夹取 |
| `tests/verify_pipeline.py` | 9 项离线回归（正常 + 5 类异常 + 3 种升级版情形） |

### 需要的参数

```python
run({
    "mode": "table",            # drawer | table
    "io": "mock",               # wrapper（默认）| mock；也可用环境变量 MED_IO
    "pipeline": "medicine.baseline",   # 换配置名 = 换实现，不改代码
    "item_name": "999 感冒灵",   # 播报用药品名
    "pause_s": 5.0,             # 播报后停顿（场景描述里的「停顿五秒」）
    "turn_deg": 180.0,          # 转身角度
    "drive_cm": 300.0,          # 直行距离
    "bin_turn_deg": -30.0,      # 到桌前转向（负=右转）
    "dry_run": False,           # True 只打印编排
})
```

### 返回值

```python
{
  "success": False,
  "feedback": "基线流程中断于步骤 grasp：夹爪张开失败（open_gripper）：夹爪堵转",
  "suggested_adjustment": "检查该步骤依赖的封装层接口与示教位姿；…",
  "details": {
      "trace": [...],            # 每步 status / code / message / elapsed_s
      "pipeline": "medicine.baseline",
      "io_backend": "mock",
      "steps_ok": 2, "steps_skipped": 5,
      "home_arm_ok": True, "home_base_ok": True,
      "io_missing": ["set_gripper", ...]    # wrapper 后端才会出现
  }
}
```

`success=False` 有两种语义，靠 `details.stopped_without_safety` 区分：
`failed` 会归位；`unknown` 立即全停、**不归位**（因为此时不知道机器人在哪，发运动指令不安全）。

### 三条硬规则

1. 失败不静默：每步独立记账，失败即停后续普通步骤。
2. 归位无条件：`must_run` 的 `return_home` 在前序失败后仍执行；`finally` 里再做硬件级兜底。
3. 依赖缺失软降级：封装层缺接口只记 warning + `DEP_MISSING`，不崩、不阻塞。

---

## 队友怎么加"识别日期"（三步，不改 runner）

见 [`docs/plugin_guide.md`](../../docs/plugin_guide.md)。摘要：

1. `pipeline/policies/` 下新建 `.py`，用 `@register_stage("你的名字")` 装饰，实现 `run(ctx, io)`；
2. 复制一份 `pipeline/configs/*.json`，只改 `grasp` 那一步的 `stage`；
3. `run({"pipeline": "你的配置名"})`。

参考实现：`pipeline/policies/expiry_demo.py`，其中 `read_date()` 就是要替换的那一个函数。

---

## 测试

```bash
cd <仓库根>

# 离线全量回归（无需硬件、无需依赖）
python3 scenes/scene_medicine_check/tests/verify_pipeline.py

# 只看动作编排
python3 scenes/scene_medicine_check/baseline_flow.py

# 升级版（过期判定）
python3 -c "
import sys; sys.path.insert(0,'.')
from scenes.scene_medicine_check import baseline_flow as bf
print(bf.run({'pipeline':'medicine.expiry_demo','io':'mock','pause_s':0})['feedback'])
"
```

---

## ⚠️ 上真机前必须补齐的封装层接口（归负责人 A）

本场景所有硬件动作走 `pipeline/io.py`，缺接口只记 warning 不崩，但**缺一个就少一段能力**。
完整清单与官方签名对照见仓库根 `to朱宁11.6.md`。

| 接口 | 用途 | 缺失后果 |
|---|---|---|
| `attach_tool(chain, tool)` | 挂夹爪 | 无法做末端位姿控制 |
| `set_arm_pose(pose, timeout)` | 预抓/抬起/投放 | 手臂无法按位姿动 |
| `move_line_to(pose)` | 直线逼近 | 只能点位移动，抓取易撞 |
| `set_gripper(width, side, …)` | 夹爪开合 | 抓不住也放不开 |
| `set_joints(pos, names, speed)` | 收臂/归位 | 抱持与复位不可用 |
| `get_odom()` | 记起点/回原点 | 无法闭环回原位 |
| `move_to_pose(x, y, yaw)` | 回原点 | 同上 |
| `speak(text)` | 播报 | 退化为日志，现场听不到 |
| `capture_image(tag)` | 取图 | 无法做任何视觉识别 |
| `grasp` / `release` / `place` | drawer 模式抓放 | drawer 模式不可用 |

另外两处已知缺陷：
- `move_backward` 内部被 `abs()` 吞掉负号 → **后退变前进**（drawer 模式拉抽屉会往前撞）。
- `baseline.reset_to_baseline()` 目前只做 `stop_base()`，手臂回折叠位仍是 TODO。

## drawer 模式的感知服务契约（需独立实现）

场景进程**不做任何模型推理**，只调 HTTP：

```
POST {perception_url}/detect
  body: {"image": "/tmp/xxx.jpg", "task": "medicine"}
  resp: {"items": [
          {"name": "999", "expiry_text": "2025-08-31",
           "bbox": [x, y, w, h], "confidence": 0.93}
        ]}
```

YOLO 负责定位药盒，手写日期识别（OCR/VLM）负责把 `expiry_text` 读出来。服务不在时场景返回空结果、不崩溃。

## 日期解析规则（`parse_expiry`）

| 标签写法 | 解析结果 |
|---|---|
| `2025-08-31` / `2025/8/31` / `2025年8月31日` | 直接用 |
| `2027年3月`（只有年月） | 该月最后一天（2027-03-31） |
| `8.15`（只有月日） | 默认当年；若算出来比今天晚超过 30 天，视为去年 |
| 认不出来 | `None` → `manual_check`，**不瞎猜** |

到期当天算有效（`expiry < today` 才过期）。
