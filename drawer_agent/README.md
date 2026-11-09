# 抽屉 Demo 接口与联调

首版在固定底盘下拉屉、取一盒药、原路持盒退出、在固定桶上方开爪投放，再回到右臂 HOME。转体和关屉是独立扩展，由现场配方中的 `flow.rotate_for_drop` 和 `flow.close_drawer` 控制。两项默认关闭。

## 腕部视觉开屉接口

队友负责用腕部 RGB 和传统视觉计算把手位置，输出从当前停稳位接近把手、夹持、分段拉出、松手并撤离的计划。将实现注入 `run_flow(..., opener=planner)`。接口定义在 `wrist_opener.py`，调用方法为 `planner.plan(request) -> list[dict]`。算法只计算计划，实际取图、运动、夹爪、停止和结果验收由共享适配层负责。

现场配方用以下字段选择这段。配置保存在现场配方，不随代码提交已验收的标记。

```json
{
  "drawer_open": {
    "mode": "wrist_vision",
    "camera_id": "<本轮腕部相机 ID>",
    "calibration_id": "<本轮腕部标定 ID>",
    "use_view_route": false
  }
}
```

默认的 `mode: "taught"` 使用已验收的示教开屉路线。若起始位看不到把手，先示教一段只移动手臂的 `route_refs.wrist_view`，再打开 `use_view_route`。适配器会走到取图位停稳，再用最新腕部图像调用算法。

| 输入 | 含义 |
|---|---|
| `request.frame` | 腕部图像路径、帧号、相机 ID、标定 ID、原始宽高和单调时钟采集时间 |
| `setup_id`、`target_opening_m` | 本轮现场编号和抽屉实际目标开度 |
| `start_tcp`、`handle_release_tcp`、`handle_clear_tcp` | base_link 下的当前、把手满开参考和松手撤离位，单位米，四元数顺序 xyzw |
| `deadline_ns` | 整段开屉的单调时钟截止时间 |

算法输出沿用 `drawer_routes/1` 的动作字段。依次提供 `move_tcp` 的 `pose`，`grip` 的 `width_m`，若干 `move_rel` 的 `dx/dy/dz`，`release` 的 `width_m`，最后用 `move_tcp` 到 `HANDLE_CLEAR`。允许插入短暂 `settle`。当前现场约定滑轨向机器人拉出为 base_link 负 X。这个方向需要随停靠位验收。每个相对运动向量限长 60 mm。无法可靠定位时抛错，由主流程停止。

开屉完成的交接证据写入 `open_drawer_release` 的回执。字段包含 `setup_id`、`opening_m`、`opening_uncertainty_m`、`drawer_open_verified`、`handle_released`、`arm_clear_of_drawer` 和 `handle_clear_tcp`。本轮目标是抽屉实际开度 270 mm，独立测量不确定度须在 3 mm 以内。旧记录约 301 mm 是 TCP 位移。

这是单帧规划接口。队友若采用持续视觉伺服，先明确需要重新取图的阶段，再扩展规划协议。共享适配层继续负责控制权和动作证据。

## 接 YOLO 与原路返回

开屉通过后，右臂由 `HANDLE_CLEAR` 走已核验的抬升路线，在最大 Z 的 `OBSERVE_HIGH` 停稳。此时切换到固定观察相机，拍新帧，经 `POST /segment` 取药盒顶面掩膜。盒顶平面标定给出 XY，盒子上方复拍通过后再沿同一 XY 下探、闭爪、短抬并验盒。

夹取成功后，先沿下探轴回到盒上方，再逆序经过本次记录的 TCP 接近点，回到开屉撤离位。路径包括从 `HANDLE_CLEAR` 到最高点及其后视觉对准的运动。现场回程须重新规划和验证持盒扫掠净空，并核对原命名关节分支。随后走 `retreat_to_start` 到持盒撤离位，再走 `carry_to_drop` 到桶上方。释放前确认桶空和投放净空，释放后核验入桶。最后按 `drop_to_home` 走右臂归位路线。

## 本地离线验证

从仓库根目录运行。框架包 `drawer_demo/` 可与 `drawer_agent/` 同放根目录，或通过 `DRAWER_DEMO_ROOT` 指向包含 `drawer_demo/` 的目录。

```powershell
python -m drawer_agent.cli run --trace
python -m drawer_agent.cli run --extended
python -m drawer_agent.cli selftest
python -m unittest drawer_agent.test_adapter
```

默认运行固定底盘版本。 `--extended` 选原 v2 合成装置，保留转体和关屉故障回归。两种离线装置的坐标和读数都是构造值。

## 现场接入条件

在团队 Agent 同进程注入共享封装。正式调用为 `run_flow(recipe_path, routes_path, backend="robot", wrapper=shared_wrapper, perception_url="http://127.0.0.1:8010", opener=planner)`。现有团队场景入口和注册表需要在停止、人工接管及归位行为对齐后接入。CLI 仅保留真机只读预检，腕部策略需从 Python 调用方注入。

- 配方和路线需分别带 `source_kind: "site"`、`commissioned: true`、`evidence_ref`，两者 `setup_id` 一致。配方必须有显式 `flow`。 `teach` 生成的是候选数据，需逐段核验。路线须包含 HOME、OBSERVE_HIGH、START、DROP、HANDLE_CLEAR 的 `waypoint_joints_rad`。验收项包括 `loaded_reverse_verified`。
- 共享封装提供 `link.NEEDED_HINT` 中当前流程需要的方法。现场起点读数需报告 `site_setup_id`、`dock_pose_verified`、`force_monitor_ready`、`independent_stop_ready`。使用腕部视觉时同时提供 `wrist_calibration_id`。HOME 的 TCP、关节、开爪及闭屉状态均需核对。
- `set_arm_pose_base` 接收速度、接触力上限、持有物类型和可选 `expected_joints_rad`，须在执行前落实轨迹检查并在执行期间监测。回执提供 `collision_checked`、`swept_clear`、`force_monitored`、`joint_branch_continuous` 和 `joint_goal_rad`。适配器会再次读回实际位置、姿态及命名关节。
- `get_hold_state` 返回持有物类型及独立的 `box_lift_verified`。 `get_bin_state` 在投放前返回桶是否为空、`release_clearance_verified`、`release_offset_m`，投放后返回 `box_present`、`distance_to_center_m`。所有读数和回执使用同一个单调时钟。
- 看门狗可在动作线程阻塞时请求停止。现场仍须验证 `stop_controller` 能独立中断运动并确认停稳。停止确认要求收到新鲜的已接收和已完成回执，随后禁止动作继续。失去状态后人工接管。
- 启用转体时分别核验持盒去程和空爪回程扫掠。启用关屉时提供独立示教的推回路线、松把手撤离路线及 `push_route_verified`。旧示教转换器生成的倒序推回只是候选。
