# scene_medicine_check - 药品管理（过期药品识别与清理）

- **负责人**：卓杭、韩桐欣
- **触发时机**：主 Agent 收到「检查药箱 / 清理过期药」指令时调用
- **依赖**：`sdk_wrapper`（硬件）+ 独立感知服务（YOLO 检测 + 手写日期识别）

## 两种环境

| mode | 环境 | 流程 |
|---|---|---|
| `drawer`（默认） | 桌上固定小抽屉，内放 999 与维生素各一件 | 对位 → 拉开抽屉 → 逐件取出 → 拍照识别 → 判定 → 放回/丢弃 → 推回抽屉 |
| `table`（基线兜底） | 药品直接摆在桌面固定位置 | 抓取 → 读日期 → 判定 → 播报 → 丢弃/放回，跳过抽屉动作 |

比赛现场抽屉做不出来时用 `table`，行为等价于基线描述。

## 参数

```python
run({
    "mode": "drawer",           # drawer | table
    "confirm_wait_s": 5.0,      # 播报后停顿等人类回应（场景描述里的「停顿五秒」）
    "max_items": 4,             # 本次最多检查件数，防止无限循环
    "perception_url": "http://127.0.0.1:8010",   # 也可用环境变量 MEDICINE_PERCEPTION_URL
})
```

## 返回值

```python
{
  "success": True,
  "feedback": "药检完成：共检查 2 件，清理过期 1 件，放回 1 件，待人工复核 0 件",
  "suggested_adjustment": None,
  "details": [
    {"name": "999 感冒灵", "expiry": "2025-08-31", "action": "discarded", "days_overdue": 432},
    {"name": "维生素", "expiry": "2027-12-31", "action": "kept"}
  ]
}
```

`action` 三种取值：`discarded`（过期丢弃）/ `kept`（未过期放回）/ `manual_check`（日期读不出来，放回等人工）。

## 感知服务契约（需独立实现）

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

## 日期解析规则（写在代码里，避免口头扯皮）

| 标签写法 | 解析结果 |
|---|---|
| `2025-08-31` / `2025/8/31` / `2025年8月31日` | 直接用 |
| `2027年3月`（只有年月） | 该月最后一天（2027-03-31） |
| `8.15`（只有月日） | 默认当年；若算出来比今天晚超过 30 天，视为去年 |
| 认不出来 | `None` → `manual_check`，**不瞎猜** |

到期当天算有效（`expiry < today` 才过期）。

## ⚠️ 上真机前必须由负责人 A 补齐的封装层接口

本场景全部通过 `_call()` 软调用，接口缺失只记 warning 不崩，但目前**以下函数在 `sdk_wrapper/wrapper.py` 里还不存在**，缺一个就少一段能力：

| 接口 | 用途 | 缺失后果 |
|---|---|---|
| `capture_image(tag)` → `{"path": ...}` | 腕部/头部相机取图 | 无法拍照，全程跳过视觉判定 |
| `grasp(bbox)` / `release()` / `place(bbox)` | 夹爪抓取/释放/放回 | 无法取放药品 |
| `move_forward(cm)` / `move_backward(cm)` | 抽屉拉开与推回 | drawer 模式不可用 |
| `speak(text)` | 语音播报 | 退化为日志输出，现场听不到声音 |

另外 `baseline.reset_to_baseline()` 目前只做 `stop_base()`（手臂折叠是 TODO），
「场景退出必须归位」这条铁律本场景依赖它，需要 A 补齐后才真正生效。

## 测试

```bash
# 无真机、无感知服务时，验证日期解析与骨架能跑通
GALBOT_MOCK=1 uv run python -c "
from scenes.scene_medicine_check.scene import parse_expiry, is_expired
from datetime import date
for t in [\"2025-08-31\", \"2027年3月\", \"8.15\", \"看不清\"]:
    d = parse_expiry(t, date(2026, 11, 6))
    print(t, d, is_expired(d, date(2026, 11, 6)))
"
```
