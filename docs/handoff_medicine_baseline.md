# 药品基线 · 队友交接文档

> 面向：负责「抽屉操作、日期识别、过期目标选择，再接回原有抓取与后续流程」的队友。
> 基线已确认：mock 全链路验证通过（9/9，见 §7）。你只管写你那一块，**现有接口保持不变**。

## 1. 代码位置与运行方式

| 项 | 值 |
|---|---|
| 仓库 | `NnNnNn-Zzz/galbot-hackathon` |
| 分支 | `feat/scene-medicine-check`（main 的纯追加，可快进合并） |
| 基线入口 | `scenes/scene_medicine_check/baseline_flow.py` 的 `run(params)` |
| 管线实现 | `scenes/scene_medicine_check/pipeline/`（runner / contracts / io / stages / policies / configs） |
| 你的接入模板 | `pipeline/policies/expiry_demo.py`（复制改注册名即用） |
| 你的接入说明 | `docs/plugin_guide.md` |

```bash
cd galbot-hackathon
# 1) 只看编排（不发任何"动作"，打印 8 步计划）
python3 scenes/scene_medicine_check/baseline_flow.py
# 2) mock 全链路（纯内存仿真，不接任何硬件）
MED_IO=mock python3 -c "from scenes.scene_medicine_check import baseline_flow as bf; \
import json; print(json.dumps(bf.run({'io':'mock','pause_s':0}), ensure_ascii=False, indent=1))"
# 3) 回归验证（正常/失败/超时/unknown/异常/升级版共 9 项）
python3 scenes/scene_medicine_check/tests/verify_pipeline.py
# 4) 升级版（日期识别+判过期）演示
MED_IO=mock python3 -c "from scenes.scene_medicine_check import baseline_flow as bf; \
print(bf.run({'io':'mock','pipeline':'medicine.expiry_demo','pause_s':0})['feedback'])"
```

上真机只切一处：`run({"io": "wrapper"})`（默认即是），**stage 一行不动**。

## 2. 你的任务范围与接回方式

| 你负责的 | 接入点 | 做完后 |
|---|---|---|
| 日期识别（OCR/VLM） | 实现 `read_date(io, obj)`（模板在 `policies/expiry_demo.py`） | 读不出返回 `None`，**绝不用猜测顶替** |
| 过期比对 + 选目标 | 同一个 policy 类里（示例已给：按"最左优先"选第一盒过期药） | 无过期 → `failed(NO_TARGET)`；全读不出 → `unknown` |
| 过期目标夹取 | 复用示例里的动作序列（attach→开爪→预抓→逼近→闭爪→抬起→抱持） | 产出 `ctx.target` + `ctx.picked` |
| 抽屉操作（新能力） | 新建 `pipeline/stages/` 或 `policies/` 下一个 stage，`@register_stage("...")` 注册，配置文件里插到 `preflight` 之后 | 遵守同一契约 |
| **接回后续流程** | **零改动**：播报→左移→右转→投放→归位由 runner 按配置自动接在你 stage 后面 | — |

切换方式 = 只改配置 JSON 里对应步骤的 `"stage"` 字段，runner 和其他 stage 不动。

## 3. 冻结契约（最重要的三条）

1. 你的 stage 只通过 `io`（白名单接口，§4-§6）访问硬件。**禁止** `import galbot_sdk` / `import sdk_wrapper` / 直连 `adapters`。
2. 返回 `StageResult` 三态：
   - `ok` —— 本步达成，继续；
   - `failed` —— **已知失败**（会执行归位后终止）；
   - `unknown` —— **不可判定**（立即终止，**连归位都不发**——不知道自己在哪还动，比不动危险）。
3. 不吞异常：预期内失败用 `failed()`，预期外让异常冒泡（runner 会兜底记 trace 并归位）。

## 4. 感知接口（objects 元素字段）

`ctx.objects` 是 `list[dict]`，每个元素（你要求的三个要素已显式成字段）：

| 字段 | 类型 | 语义 |
|---|---|---|
| `id` | str | **实例编号**，稳定唯一（如 `"box_left"`），`ctx.expiry` / `ctx.verdict` 的键就是它 |
| `image_ref` | str | **图像关联**：本目标 `bbox` 所在的帧标识，与 `io.capture_image()` 返回的 `frame_id` 对应 |
| `bbox` | list[int] | 该帧内像素框 `[x, y, w, h]`（左上角原点） |
| `position_m` | list[float] | **三维位置 `[x, y, z]`，单位米，坐标系见 `frame` 字段** |
| `frame` | str | 坐标系，当前恒为 `"base_link"`（x 向前、**y 向左**、z 向上；"最左" = y 最大） |
| `unit` | str | 单位，恒为 `"m"` |
| `ee_pose` | list[float] | `position_m` 的**历史别名**，语义相同；新代码请用 `position_m` |
| `label` | str | 类别标签（如 `"white"`） |
| `score` | float | 检测置信度 |
| `expiry_date` | str | ⚠️ **仅 mock 离线固定数据**。真机没有这列——日期靠你的 `read_date()` 读 |

当前 objects 的来源：mock 由 `io.fixture("objects")` 提供（内置三盒默认表，或 `run({"objects": [...]})` 传入）。真机的感知 stage（检测+反投影）尚未落地，届时产出**必须填齐同样字段**。

## 5. 右手动作接口（含"已完成/仅下发"语义）

全部返回 `dict`，公共四键：`success` / `message` / `code` / **`completion`**。

| 调用 | 作用 | mock 返回示例 |
|---|---|---|
| `io.attach_tool(chain, tool)` | 绑定末端工具（位姿控制前提） | `{"success": True, ...}` |
| `io.set_arm_pose(pose7, timeout_s)` | 手臂到指定位姿（7 维：[x,y,z,qx,qy,qz,qw]，米+四元数 scalar-last，base_link） | `{"success": True, "pose": [...], "completion": "completed"}` |
| `io.move_line_to(pose7)` | **直线**逼近（抓取/投放用） | 同上 |
| `io.set_joints(positions, names, speed)` | 关节角控制（抱持位用） | `{"success": True, "n": 21, "completion": "completed"}` |

**`completion` 语义（你点名要的）**：`"completed"` = 返回时动作**已完成**；`"dispatched"` = 仅**已下发**，完成需另行确认。**mock 下全部同步执行，恒为 `"completed"`**；真机（wrapper）由各接口按 SDK `is_blocking` 如实填写，wrapper 落地前真机语义**未定**——你的代码判断进度时请以 `completion` 字段为准，不要假设。

## 6. 抓取与播报接口

```python
# 夹爪：width_m 单位米（G1 量程 0~0.11m；收放用 0.10 ↔ 0.02）
ret = io.set_gripper(0.02, side="left", velocity_mps=0.05, effort=10.0)
# → {"success": True, "width_m": 0.02, "holding": "box_left", "completion": "completed"}
#   holding 是 mock 的物理推演；真机没有夹爪反馈通道：
io.is_holding()   # → True / False / None。真机恒 None = 无确证手段
                  #   → ctx.picked 只能写 None，禁止写成 True 糊过去

# 播报：
ret = io.speak("该药品已过期，帮你清理掉")
# → {"success": True, "played": True, "bytes": N, "completion": "completed"}
```

真机播报走 PCM 音频流（16 kHz/16-bit/mono 裸 PCM，经 `write_audio_stream_output` 分块下发），文本→PCM 用 `assets/audio/make_pcm.py`（galbot_dev 仓）离线生成，**不能运行时拼变量**（如药名），要么用"该药品"这类固定文案，要么每个词各生成一条。

## 7. 已验证与待补数据

**已验证（mock，9/9 通过）**：正常 7 步全流程；抓取失败仍归位；左移超时归位；起始里程计读不到非致命；不可判定立即全停（不发归位）；stage 抛异常 runner 兜底归位；升级版选中过期盒/无过期/日期全读不出。

**待补（真机前必须落实，详见 `to朱宁11.6.md`）**：

| 缺口 | 影响 |
|---|---|
| wrapper 13 个接口未实现（attach_tool/set_arm_pose/move_line_to/set_gripper/set_joints/get_odom/move_to_pose/move_left/speak/capture_image 等） | 真机后端会软降级 `DEP_MISSING`；补齐前只能 mock |
| 抓取/投放/归位位姿全是官方 example6 占位值 | **必须现场示教实测替换**（`pipeline/tuning.py`） |
| `places` / 左移 0.5 m 的实际里程 | mock 不校验，真机要标定；**侧移要求底盘支持横向运动，真机待确认** |
| PCM 语音素材未生成 | 真机播报会静音降级 |
| 真机 `completion` 语义 | wrapper 实现时按 `is_blocking` 填 |

## 8. 一句话流程图

```
S0 preflight → S1 grasp(你的：读日期+判过期+选目标+夹取) → S2 announce
→ S3 shift_left(左移0.5m) → turn(右转30°) → drop → return_home → 反馈
任何一步 unknown → 立即全停（不发归位）；failed → 归位后终止
```
