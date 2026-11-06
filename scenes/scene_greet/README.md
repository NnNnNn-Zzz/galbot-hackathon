# scene_greet - 打招呼场景

- **负责人**：（填名字）
- **触发时机**：检测到有人接近（距离 < 2m）
- **行为**：向人挥手/点头示意
- **依赖**：仅 `sdk_wrapper`（如需要视觉依赖，向负责人 A 申请）

## 测试

```bash
# 在项目根目录，mock 模式下单测本场景
GALBOT_MOCK=1 uv run python -c "from scenes.scene_greet.scene import run; print(run({}))"
```
