# 场景管线接入指南（队友向）

对象：要在这套基线上加"识别日期 → 比对 → 找到过期的那盒 → 夹取"这类升级的同学。
目标：**你只写一个 .py + 复制一份 json，不改 runner、不改别人的 stage、不改 sdk_wrapper。**

---

## 一、三步接入

**第 1 步**：在 `scenes/scene_medicine_check/pipeline/policies/` 下新建 `my_stage.py`。
这个目录下的模块会被自动发现，**不需要改 `__init__.py`，不需要注册到任何列表**。

```python
from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok, unknown
from ..registry import register_stage


@register_stage("my_grasp")          # ← 这个名字就是配置里要写的 stage 值
class MyGrasp(Stage):
    stage_id = "grasp"

    def run(self, ctx: Context, io) -> StageResult:
        targets = list(ctx.objects) or list(io.fixture("objects") or [])
        if not targets:
            return unknown("视野内没有目标", code=Code.NO_TARGET)

        for obj in targets:
            text = my_ocr(io.capture_image(tag=str(obj["id"])))   # ← 你自己的识别
            ctx.expiry[str(obj["id"])] = text                     # "YYYY-MM-DD" 或 None
        ...
        return ok("已抓起过期药品", target=obj["id"])
```

**第 2 步**：复制 `pipeline/configs/medicine.baseline.json` 为 `medicine.mine.json`，
只改你要替换那一步的 `stage`：

```json
{ "id": "grasp", "stage": "my_grasp", "params": { "prefer": "leftmost" } }
```

**第 3 步**：调用时把配置名传进去，其余不变：

```python
run({"pipeline": "medicine.mine"})
```

参考实现：`pipeline/policies/expiry_demo.py`（读日期 → 判过期 → 取最左 → 夹取）。
它的 `read_date()` 就是留给你的替换点。

---

## 二、冻结契约（改动需协商）

`Context` 是 stage 之间**唯一**的数据通道。字段与归属：

| 字段 | 类型 | 谁生产 | 谁消费 |
|---|---|---|---|
| `ctx.params` | dict | 场景入口 | 所有 stage（**只读**） |
| `ctx.objects` | list[dict] | 感知 stage | 你的选目标逻辑 |
| `ctx.expiry` | `{obj_id: "YYYY-MM-DD" \| None}` | 读日期 stage | 判定 stage |
| `ctx.verdict` | `{obj_id: "expired"\|"valid"\|"unknown"}` | 判定 stage | 选目标 / 播报 |
| `ctx.target` | dict | 选目标 stage | 夹取 stage |
| `ctx.picked` | `bool \| None` | 夹取 stage | 汇报（**None = 无确证手段，不许写 True**） |
| `ctx.start_odom` | dict | preflight | return_home |
| `ctx.flags` | dict | 各 stage 暂存 | 键名必须带自身前缀，如 `"expiry.ocr_ms"` |
| `ctx.trace` | list | runner 写 | 只读 |

规则：**只能追加可选字段，不改已有字段语义，不改 `run(ctx, io)` 签名。**

---

## 三、三种返回语义（本项目最重要的一条）

| 返回 | 含义 | runner 后续行为 |
|---|---|---|
| `ok(...)` | 本步达成 | 继续下一步 |
| `failed(...)` | **已知**失败 | 停止后续普通步骤，但仍执行 `must_run` 的归位步骤 |
| `unknown(...)` | **不可判定** | 立即全停，**连归位也不发** |

为什么 unknown 不归位：归位本身是一次底盘 + 手臂运动，前提是"知道自己在哪"。
unknown 的定义恰恰是"不知道"（读数互相矛盾、视觉说不清），此时再发运动指令可能撞人撞物。
所以 unknown 只允许在场景 `finally` 里做硬件级安全停止，不做任何规划运动。

代码层面别自己 `if not ret["success"]: return failed(...)` —— 那会把 unknown 降级成 failed。
**用 `outcome(ret, "前缀")`**：它按 `ret["code"]` 自动分流，`UNDETERMINED` 走 unknown，其余走 failed
（含 `TIMEOUT`：超时后里程计通常还在，可以闭环回原位，所以归 failed）。

---

## 四、三条边界

1. **不许 `import galbot_sdk`、`import sdk_wrapper`、`import adapters`**。硬件只走 `io` 参数。
2. **只允许调 `io` 的白名单方法**：
   `get_odom` `attach_tool` `set_arm_pose` `move_line_to` `set_joints` `set_gripper`
   `turn` `move_forward` `move_to_pose` `speak` `capture_image` `is_holding` `fixture`
3. **不吞异常**：预期内失败用返回值表达，预期外让异常冒泡（runner 会兜住并记 trace）。

`io` 所有方法统一返回 `{"success": bool, "message": str, "code": str|缺省}`。

---

## 五、配置字段

| 字段 | 必填 | 含义 |
|---|---|---|
| `id` | ✅ | 步骤唯一名，出现在 trace 里 |
| `stage` | ✅ | 注册名（`@register_stage` 的字符串） |
| `params` | | 传给该 stage 构造函数的参数 |
| `must_run` | | 前序失败后仍要执行（归位类） |
| `non_fatal` | | 本步失败不中止流程（自检类） |

---

## 六、离线自测（不接硬件）

```bash
cd <仓库根>

# 全量回归：9 个场景（正常 / 抓取失败 / 超时 / 非致命 / unknown / 抛异常 / 升级版×3）
python3 scenes/scene_medicine_check/tests/verify_pipeline.py

# 只看编排
python3 scenes/scene_medicine_check/baseline_flow.py
```

故障注入走 `params`，不需要改代码：

```python
run({"io": "mock",
     "mock_faults": [{"method": "set_gripper", "behavior": "fail_once", "message": "夹爪堵转"}]})
```

可用 behavior：`fail` / `fail_once` / `timeout`（配 `delay_s`）/ `unknown` / `raise`。

> `io.fixture(...)` 只在 mock 下有数据，真机恒为 `None`。
> 用它读到"日期文本"只说明流程通了，**不代表你有识别能力**——真机必须自己接 OCR/VLM。

---

## 七、上真机怎么切

**只改一处**：`run({"io": "wrapper"})`，或设环境变量 `MED_IO=wrapper`（默认就是它）。
stage 一行都不用动。

前提是 `sdk_wrapper/wrapper.py` 的 13 个接口已补齐（清单见仓库根 `to朱宁11.6.md`）。
没补齐也能跑，只是缺的接口会返回 `DEP_MISSING` 并记进 `details.io_missing`。

---

## 八、未验证点（只能上真机确认）

| # | 事项 | 现状 |
|---|---|---|
| V1 | wrapper 13 个接口 | 未实现 → wrapper 后端只能跑到"依赖缺失"降级 |
| V2 | `move_backward` 方向 | 实测逻辑 bug（`abs()` 吞负号），drawer 模式后退会变前进 |
| V3 | 抓取/投放/归位位姿 | 全是官方 example6 占位值，必须现场示教 |
| V4 | 夹爪是否真夹住 | 真机无位置反馈时 `picked=null`，只靠"闭合指令成功"推断 |
| V5 | 左移 0.5m 的横移精度与可行性 | mock 是理想积分；侧移要求底盘支持横向运动（全向轮），真机若不支持需改回「转90°→前进→转回」 |
| V6 | 语音播报 | 需要预生成 16kHz/16bit/mono 裸 PCM，SDK 无 TTS |
| V7 | `reset_to_baseline` | 目前只 `stop_base()`，手臂回折叠位仍是 TODO；mock 下不验证它 |
