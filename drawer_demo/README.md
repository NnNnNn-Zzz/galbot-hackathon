# 抽屉 Demo 状态机

首版固定底盘，完成拉屉、单盒视觉抓取、持盒原路退出和投桶。腕部传统视觉开屉与固定相机 YOLO 定位是两个独立的接口。前者只负责把手与开屉，后者在已验证的最高观察位接手。

```text
HOME → 对准把手 → 拉开 → 松手撤到 HANDLE_CLEAR
     → OBSERVE_HIGH → YOLO 定位 → 盒上方复拍 → 垂直下探
     → 夹持和短抬确认 → 逆序返回 → START → DROP → 入桶确认 → HOME
```

## 运行与配方

从 Junction 根目录运行。

```powershell
python -m medicine_frontend.drawer_demo plan
python -m medicine_frontend.drawer_demo mock
python -m medicine_frontend.drawer_demo mock --extended
python -m medicine_frontend.drawer_demo selftest
python -m unittest medicine_frontend.drawer_demo.test_demo
```

`site_recipe.template.json` 是待填模板。 `flow.rotate_for_drop` 和 `flow.close_drawer` 分别控制扩展，默认均为 false。旧版缺少 `flow` 的 v2 离线配方仍代表完整转体加关屉流程。现场入口要求显式配置。

`runtime.py` 负责顺序、时效和证据门槛，`geometry.py` 消费 `drawer_box_seg/1` 的原尺寸像素掩膜并投影到盒顶平面。标定和图像必须采用同一畸变校正约定。YOLO 只修正 XY，朝向超限时停止。复拍通过后冻结 XY，抓取 Z 来自现场工具和盒顶测量。

`galbot-hackathon/drawer_agent/README.md` 说明腕部算法接口、原路返回和共享封装要求。 `yolo_medbox/serve.py` 提供同机推理服务。权重由视觉队员交付，`--capture-dir` 为 Agent 写入原尺寸图像的共享目录。

```powershell
python -m yolo_medbox.serve --model <已验收分割权重> --model-id <与现场配方一致> --capture-dir <同机图像目录>
```

状态机的 Mock 和适配器的离线装置用于检查动作顺序与故障行为。上线前仍须核验现场点位、两机位图像、标定、真实持盒回程、停止通道和投放证据。
