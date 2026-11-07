# 抽屉取药分层实验

本场景接入 NNN 的场景注册表与 Agent 调度。Agent 使用 `scene_drawer_experiment` 运行 Mock。现场执行由当前共享 Agent 进程调用 `drawer_agent.experiments.run_site`，并注入已验收的现场封装、配方、路线和感知服务。

| 层级 | 顺序 | 验证结果 |
|---|---|---|
| 1 视觉确认与示教回程 | `vision` 在 P0 取新帧并投影定位。 `retreat` 从 P0 重新取帧，验证后空爪按 P1、P2、`INITIAL_CARRY` 顺序走。 | 视觉失败时留在 P0。通过时记录首帧与各点实测 TCP。 |
| 2 已知盒位 | `hover → dry → carry → drop` | 依次核对盒上方位置、空爪下探和回程、持盒原路退出、入桶归位。 |
| 3 整链 | 腕部开屉先用独立的 `1 preview/open` 验收。临时分割 `hover → full` 后，训练模型再各跑一次 `hover → full`。 | 从闭屉 HOME 经开屉、P0 定位、复拍、抓取、撤离到投放和归位。 |

第一层由视觉确认当前场景、由示教点约束回程。Mock 会渲染彩色测试盒，并走与临时分割服务相同的 HSV 提取及坐标投影代码。真机以真实新帧替换合成图像。该设计只在动作需要目标信息时取图，避免给已示教的回程引入多次推理和不必要的视觉修正。

P0、P1、P2 原始站定数据分别为 50、20、20 帧，右臂七轴各帧齐全。所见样本内最大关节跨度约为 0.000145 rad。原清单没有初始端点、点间连续轨迹或反向执行验收。暂将「初始的报识姿态」理解为独立的初始抱持位 `INITIAL_CARRY`。现场应复核其语义，录入四点当前工具的 TCP、终点七轴关节和扫掠净空。模板在 `drawer_agent/data/`，其中四点的 TCP 全部留空。空爪验证通过后，持盒回程仍单独验收。

## 验证命令

```powershell
python -m drawer_agent.experiments 1 mock --stage retreat
python -m drawer_agent.experiments 2 mock --stage drop
python -m drawer_agent.experiments 3 mock --stage full
python -m unittest drawer_demo.test_demo drawer_agent.test_adapter drawer_agent.test_experiment_stages scenes.scene_drawer_experiment.test_scene
```

Agent 调用参数如下。

```json
{"action":"call_scene","target":"scene_drawer_experiment","params":{"backend":"mock","experiment":1,"stage":"retreat"}}
```

真机阶段需要实际相机与标定、姿态与开度读数、力和夹持证据、停止控制，以及第三层的两视角位置核验。第三层配方的 `experiment_approval` 应记录 `experiment_1_evidence_ref`、`experiment_2_evidence_ref`、`wrist_open_evidence_ref` 和 `vision_validation_ref`。每次悬停记录 `hover_report`，换成正式 YOLO 时还要引用临时分割的 `bench_full_report`，并核对权重 SHA-256。

当前主线提供 `capture_frame(camera_id, save_path)`，与实验要求的超时、相机标定 ID、分辨率、帧号和统一时钟回执仍有接口差异。其他运动和停止能力也需要按共享适配契约落实。Mock 所测的是调用链、视觉门槛和动作顺序，现场可达性、受力和耗时由分段真机验收确认。
