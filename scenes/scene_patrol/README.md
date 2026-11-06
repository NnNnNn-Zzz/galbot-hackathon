# scene_patrol - 巡逻场景

- **负责人**：（填名字）
- **触发时机**：无其他任务时周期性调用
- **行为**：沿 `PATROL_ROUTE` 预设路线巡逻一圈，路线参数在 `scene.py` 顶部可调
- **依赖**：仅 `sdk_wrapper`

## 测试

```bash
GALBOT_MOCK=1 uv run python -c "from scenes.scene_patrol.scene import run; print(run({}))"
```
