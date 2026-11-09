# VLM 校验版抓取（v2 思路落地）

## 一句话

盲抓版是「按剧本走完就下爪」；VLM 版在**下爪前加一道照片质检**——
先问多模态大模型「盒子在不在点上、偏多少」，通过才抓，不通过就小幅微调再问。

## 流程图

```
attach_tool → 开爪 → 预抓取位（盒上方10cm）
                          ↓
                    ┌── 拍一张照片 ──┐
                    ↓                │
              VLM 判断（JSON）        │
                    ↓                │
        ┌───────────┴───────────┐    │
   通过 │                  不通过 │    │
        ↓                       ↓    │
   直线逼近→合爪         按建议修正 ≤3cm
   →抬起→收臂                 │        │
        ↓                     └─ 重拍 ┘（最多 3 轮）
     抓到                   仍不行 → 停爪 failed
```

## 文件

| 文件 | 作用 |
|---|---|
| `pipeline/perception/vlm_check.py` | 校验核心：模型调用抽象、JSON 解析、**限幅**、静态桩 |
| `pipeline/policies/vlm_grasp.py` | 新 policy `grasp_vlm`，动作序列与盲抓版一致，中间插校验门 |
| `pipeline/configs/medicine.vlm.json` | 配置，`grasp` 步换成 `grasp_vlm` |
| `tests/verify_vlm_grasp.py` | 10 项专项回归（全部 mock，不联网） |

## 怎么用

### 1. 配模型（三选一，都是 OpenAI 兼容协议）

```bash
export VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
export VLM_API_KEY=sk-xxxx
export VLM_MODEL=qwen-vl-max
```

支持的例子：通义千问 VL（dashscope 兼容模式）、豆包 vision、DeepSeek-VL、
GPT-4o、自建 vLLM。**没有第三方依赖**，用的是 Python 标准库 urllib。

### 2. 跑

```bash
# mock（不联网，用 StaticVLM 桩）
MED_IO=mock python3 -c "from scenes.scene_medicine_check import baseline_flow; \
   baseline_flow.run({'pipeline': 'medicine.vlm'})"

# 真机（HPU 上）
MED_IO=wrapper python3 -c "...{'pipeline': 'medicine.vlm', 'io': 'wrapper'}"
```

### 3. 回归

```bash
python3 scenes/scene_medicine_check/tests/verify_vlm_grasp.py   # 10 项
python3 scenes/scene_medicine_check/tests/verify_pipeline.py    # 原 9 项不受影响
```

## 安全红线（代码里逐条落实）

| 红线 | 落实位置 |
|---|---|
| VLM 只有建议权，无执行权 | offset 只是**改位姿参数**，仍走原来的 `set_arm_pose` + 碰撞检查 |
| 修正量必须限幅 | `clamp_correction()`：单次 ≤3cm，累计 ≤6cm，超了拒绝 |
| 修正次数有上限 | `MAX_VERIFY_ROUNDS = 3`，超了 `failed` + `SAFETY_ABORT` |
| 模型不可用 → 停爪 | `verdict.ok=False` → `unknown`，**绝不硬抓** |
| 没有盒子 → 不抓空 | `box_present=False` → `failed` + `NO_TARGET` |
| 置信度过低 → 不算通过 | `confidence < 0.6` → 强制 `graspable=False` |
| 不编造 | 未配 API Key → 明确返回"未配置"，不返回假通过 |

## 和盲抓版的关系

- **两个 policy 并存**，改配置里一行 `"stage"` 即可切换，随时可回退；
- 盲抓版 `grasp_first_box` 一行没动，9 项基线回归照旧全绿；
- VLM 版复用同样的动作序列，所以真机上「臂怎么动」完全一致，
  唯一区别是多了一步拍照 + 判断 + 可能的 2~3cm 微调。

## 真机上还缺什么

1. **相机订阅**：`capture_image` 目前真机返回空（`get_rgb_data 返回空（相机不可用或未订阅）`）——
   VLM 版依赖拍照，这一项必须先修，才能上真机；
2. **VLM API 从 HPU 出网**：官方排障文档说 HPU 有外网（DeepSeek 直连），
   但通义/豆包是否可达需要实测；
3. **真实位姿**：x/y 仍是占位值，VLM 能纠正几厘米的偏差，
   但纠正不了"盒子根本不在视野里"——摆放仍要大致对准。
