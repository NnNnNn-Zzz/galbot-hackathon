# 充电器视觉抓取：Tool 层设计

> 状态：头部米制 strip 目标、SLAM 观察位规划、导航期间目标追踪、头部 charger 3D 交接、腕部相机垂直摆位和腕部视觉 pre-grasp 位姿下发已实现；双深度精定位、下降、闭爪和抬升尚未授权执行。

## 1. 目标与安全边界

目标是夹住充电器上表面的两条长边。术语固定为：

- **长边轴**：上表面最长方向。
- **夹爪长轴**：夹爪本体 / 两指接触面的纵向，应与长边轴平行，使两指接触充电器的两条长边。
- **闭合轴**：两指相向运动方向，应与长边轴正交，跨过充电器短边尺寸。
- **接近轴**：上表面法向；抓取时沿 `-normal` 下降。

任何坐标必须先转到 `base_link` 再和手臂可达区、底盘观察位比较。相机光学轴、夹爪接近轴和夹爪闭合轴不能互相替代，必须使用标定后的手眼 / 工具轴变换。

## 2. 分层与 Tool 边界

```text
大模型
  -> ChargerSearchTool.observe_charger_search()
  -> ChargerSearchTool.advance_charger_search(operator_confirmed=True)
       -> NavigationTool.navigate_to_pose()
       -> ChargerSearchTool.monitor_observation_navigation()
       -> PowerStripYoloTool.detect_power_strip_head()
       -> ChargerYoloTool.detect_charger_head()
  -> ChargerSearchTool.align_wrist_from_head()
  -> ChargerYoloTool.segment_charger_top_wrist()
  -> ChargerGraspTool.plan_wrist_charger_pregrasp()
  -> ChargerGraspTool.move_to_wrist_charger_pregrasp(operator_confirmed=True)
  -> 后续下降 / 闭爪 / 抬升
       -> PerceptionTool / StereoDepthTool    （视觉与深度）
       -> MotionTool / 底盘安全封装            （停稳、分段移动、夹爪）
       -> 纯算法 power_strip_yolo / charger_yolo / geometry / calibration
                                             （无 SDK、无运动）
```

大模型只接收框、质量标志、拒绝原因、表面轴和结构化位置；模型权重路径、原始张量和任意动作位姿不暴露给大模型。真机动作仍必须经过 `RobotSafety` 和显式授权闸门。

## 3. 阶段 A：头部粗定位、3D 目标与 SLAM 观察位

1. 读取头左 RGB，运行轻量 YOLO 检测 `power_strip` 整体框。它与近距 `charger` 模型是两个独立模型。
2. 同时取得头部双目深度。检测框缩放到深度分辨率，不能假设两者分辨率相同。
3. 对框内深度检查：
   - 有效像素比例；
   - 深度中位数 / 离散度；
   - 多帧中心、尺寸和深度一致性；
   - 目标是否触边或被前景挡住。
4. **测距稳定**时，才把框内深度点反投影到 `base_link`，估计目标方向、观察距离和底盘观察位。
5. **测距不稳定**时只允许原地方向修正或停车复审，不再用前进式视觉伺服冒充导航。
6. 米制目标通过安全检查后，在当前 `base_link` 中生成观察点，再与 SLAM 当前地图位姿合成 `map` 目标；一次交给 `NavigationTool.navigate_to_pose()`。
7. RGB 与深度时间戳偏差超过门槛时不能把分割掩码直接套到深度上。

视觉状态机固定为：

```text
find_power_strip
  -> approach_power_strip
  -> navigating_to_observation
  -> find_charger
  -> approach_charger
  -> lock_base_for_wrist
  -> wrist_view_aligned
```

`advance_charger_search()` 启动一次 SLAM 导航任务；上层随后重复调用 `monitor_observation_navigation(task_id)`。每次监控都运行同一 `power_strip` YOLO tracker，连续 3 次丢失 track 就取消导航并停底盘。

双目框内统计有效比例、10/90 分位相对离散度和 RGB/双目时间戳差。2026-11-08 已通过 `base_link` 地面平面验证 `raw_to_m=0.001`：毫米反投影后地面 `z=-0.0021 m`、法向 `(0.0868,-0.0323,0.9957)`。目标点还要通过距离、高度、平面 RMS 和表面法向安全检查，再生成 SLAM 观察位。

## 4. 阶段 B：腕部观察位与切换门槛

头部 charger 上表面完整、短边尺寸足够、未触边且有米制 3D 目标后，连续 3 帧稳定才进入 `lock_base_for_wrist`。底盘锁定后，`align_wrist_from_head()` 根据目标中心、表面法向和长边轴规划腕部相机位姿：相机位于中心沿法向 `0.30 m` 处，光轴指向 `-normal`，图像 x 轴对齐长边。再使用官方手眼 TF 将相机目标转换为 `EndEffector` 目标，由现有 `MotionTool.move_ee_to()` 执行并复测相机位姿。随后至少连续 3 帧满足以下条件才允许切换 D405 精定位：

1. YOLO-seg 检测到上表面掩码，置信度稳定。
2. 掩码和检测框不接触图像边缘，上表面没有明显裁切。
3. 掩码内 D405 深度有效率达标，且表面不是由多个深度的离群点拼成。
4. 右腕 RGB 与 D405 深度运行时内外参通过一致性检查，时间戳偏差在允许范围内，并完成一次实时图像边缘对齐验收。
5. 掩码短边像素数达标；同时三维短边宽度要大于定位误差预算。只看像素面积不足，细长、倾斜或部分裁切的区域可能面积很大却无法定义闭合轴。
6. 表面中心、法向和长边轴稳定；变换到 `base_link` 后位于已标定的右臂可达区。
7. 相机到目标距离满足 D405 近距量程，并保留预抓取空间。

任一帧不满足则保持底盘锁定，调整腕部观察姿态或小幅平移底盘观察，不进入下降。

## 5. 阶段 C：双深度交叉验证

保持底盘和手臂静止，分别取得右腕 RGB-D 与头部双目结果，统一变换到 `base_link`：

| 校验项 | 建议初始门槛 | 作用 |
|---|---:|---|
| 上表面中心 XY 差 | 待实测标定 | 防止腕部误分割或头部落到相邻物体 |
| 上表面中心 Z / 法向投影差 | 待实测标定 | 防止深度单位或坐标系错误 |
| 表面法向夹角 | 待实测标定 | 保证垂直接近 |
| 长边方向夹角 | 待实测标定 | 保证夹爪与长边平行 |
| 长边 / 短边尺寸比例 | 待实测标定 | 排除裁切、透视或错误区域 |

只有两路证据一致时才生成抓取帧。无法交叉验证时保留检测结果，但 `eligible_for_precision=false`。

## 6. 阶段 D：预抓取、轴心修正与安全下降

1. 用 D405 掩码深度拟合上表面平面，得到表面中心 `C`、向上法向 `N`、长边轴 `L`、闭合轴 `S=cross(N,L)`。
2. 预抓取点 `P_pre = C + N * safe_standoff`。`safe_standoff` 必须根据夹指长度、腕部相机视野和实测定位误差确定，默认 0.12 m 只能作为初值。
3. 用当前末端位姿和相机外参求手眼关系，生成接近轴 `-N`、夹爪长轴 `L`、闭合轴 `S` 的目标姿态；不能直接把相机图像轴当夹爪轴。
4. 先到预抓取点，再在腕部 RGB 中检查掩码中心、长边角度和边界余量，小步修正 XY 与 yaw。
5. 重新取深度，计算指尖到表面的剩余下降距离。按每次不超过 5 mm 的直线小步下降，每步重新检查目标完整性和深度；禁止一次用远处深度直接下降到底。
6. 到达抓取高度后闭合夹爪，闭合宽度以实测短边尺寸和夹爪标定为准。当前 G1 没有可靠夹持反馈，闭合成功不能等同于抓住。
7. 沿 `+N` 小步抬起，重新检测；目标随夹爪移动且表面稳定后才报告抓取成功，否则进入恢复流程。

## 7. 当前已实现（第一阶段）

- 头部 `power_strip` 检测工具 `PowerStripYoloTool.detect_power_strip_head()`：当前选择 `/home/mr/yoloseg/runs/yolo11n_detect/weights/best.pt`，单类 `power_strip`；训练集 35 张 / 验证集 9 张，验证 `mAP50(B)=0.995`、`mAP50-95(B)=0.581`。输出只到方向/靠近候选，`grasp_coordinate` 明确为 null。
- `PowerStripYoloTool.track_power_strip_head()` 使用 YOLO track ID 保持同一个 strip 目标；目标 ID 连续丢失会阻止继续导航。
- `ChargerSearchTool` 串联“米制 strip 目标 -> 安全检查 -> SLAM 观察位 -> 导航期间 track -> 头部 charger 3D -> 腕部相机垂直摆位”。
- `ChargerYoloTool.detect_charger_head()` 同时使用 charger 分割模型和头部双目，输出 `target_3d`、表面法向/长边和安全检查；没有米制尺度或 3D 不安全时禁止腕部交接。
- 头部 charger 交接门槛包含完整掩码、触边状态、掩码面积和短边像素，不只看面积。
- 自动发现 `/home/mr/yoloseg/runs` 下时间最新的 `best.pt`，验证 `task=segment` 和类别 `charger` 后才加载。
- 当前选中更新的 `/home/mr/yoloseg/runs/yolo11n_seg_charger_v2/weights/best.pt`：YOLO11n-seg、单类 `charger`、输入 640、100 个训练 epoch。数据为 39 张训练 / 10 张验证，验证集 `mAP50(B)=0.707`、`mAP50-95(B)=0.555`。v2 验证集与 v1 不同，指标不能直接横向比较，只能作为初版权重。
- YOLO-seg 输出归一化为完整分辨率布尔掩码。
- 计算掩码面积、短边像素、填充率、触边状态、长边方向。
- 计算掩码内深度有效率、深度范围、上表面平面、法向、表面中心、长 / 短边三维尺寸。
- 输出夹爪长轴、夹爪闭合轴和预抓取点，但**不下发动作**。
- 大而裁切、大而细、深度无效三类样本均被单测拒绝。
- 已用真实权重在 5 张验证图上完成端到端推理；检测成功帧的掩码与标注上表面轮廓一致。
- `CameraCalibrationTool` 可只读比较右腕 RGB/D405 内参与外参；`check_right_wrist_rgbd_calibration()` 参数不一致时直接拒绝精抓取。
- `GalbotAdapter.get_chain_frame_pose()` 可读取 `EndEffector`、`Camera`、`TCP` 语义帧；`check_right_arm_eye_in_hand()` 会交叉验证运动学 `Camera` 帧和官方相机外参。
- `ChargerGraspTool` 复用腕部 YOLO-seg 结果生成 base_link 下的绝对 pre-grasp 位姿；只有显式现场确认才调用现有 `MotionTool.move_ee_to()`，到位后回读 `EndEffector` 位姿并检查平移/旋转误差。该阶段不下落、不闭爪，也不声称抓取成功。
- 初始末端轴约定为 local `+x=夹爪长轴`、local `+y=闭合轴反向`、local `+z=朝目标表面的接近轴`；长轴正负两个等价朝向中会选择更接近当前末端姿态的一个。该约定仍需用一次短距离监督动作验收。

### 7.1 现场分阶段测试

真机先只生成位姿，确认输出的坐标、方向、可达性和周围空间后，再执行动作：

```bash
# 1) 仅腕部 YOLO-seg + D405 几何
python3 scripts/test_charger_wrist_pregrasp.py --stage detect

# 2) 生成绝对 pre-grasp 位姿，不发运动
python3 scripts/test_charger_wrist_pregrasp.py --stage plan

# 3) 重新观测、显示计划、现场输入 MOVE 后才下发
python3 scripts/test_charger_wrist_pregrasp.py --stage move
```

对应 ToolHub 入口为 `segment_charger_top_wrist()`、`plan_wrist_charger_pregrasp()` 和 `move_to_wrist_charger_pregrasp(operator_confirmed=True)`。

## 8. 官方手眼标定与工具坐标

官方运行时链路已确认：

1. HPU 启动 `/data/galbot/bin/eyehand_calib_publish`，它发布相机到末端/机器人 TF 的手眼标定。
2. XCU 配置文件为 `/userdata/config/robot_config/eyehand_calib.toml`；当前 `right_arm` 的 D405 条目 `calib_status=calib`，`from_link=right_arm_end_effector_mount_link`，`to_link=right_arm_camera_color_optical_frame`。
3. `GalbotRobot.get_sensor_extrinsic(SensorType.RIGHT_ARM_CAMERA)` 或 `get_transform("base_link", "right_arm_camera_color_optical_frame")` 返回 `base_link <- camera` 实时变换。
4. `GalbotMotion.get_end_effector_pose_on_chain(chain, frame_id)` 支持语义帧 `EndEffector`、`Camera`、`TCP`；TCP 需要先 `attach_tool(chain, "galbot_gripper")`。
5. `set_end_effector_pose()` 默认控制法兰；抓取时应 attach 工具并通过 `Parameter.set_tool_pose(True)` 控制夹爪 TCP。

本机当前运动规划服务 `service_motion_plan` 未运行，`GalbotMotion.init()` 返回 False，因此暂时不能在线查询 `Camera`/`TCP` 位姿；相机外参和手眼配置文件仍可读取。

当前右臂 D405 彩色与深度运行时结果：

- 内参 `K` 和畸变参数完全一致；
- 640×360 分辨率一致；
- `get_sensor_extrinsic` 的时间戳和 `base_link <- camera` 外参完全一致；
- 深度 header 使用 `right_arm_camera_depth_optical_frame`，但该深度 frame 不在 TF 中；计算时使用颜色光心 frame `right_arm_camera_color_optical_frame`。

这只证明官方标定值一致，仍建议做一次实际 RGB-D 边缘对齐验收，防止配置正确但安装机械偏移。

## 9. 待现场标定

- 充电器真实长 / 短边尺寸，用于单目比例与双目尺度交叉检查。
- 右腕 RGB 与 D405 深度的实时图像边缘对齐验收（配置值已一致）。
- 启动 `service_motion_plan` 后验收运动学 `Camera`/`TCP` 帧与 `eyehand_calib_publish` 一致。
- 夹爪两指闭合轴、指面长度、最大开度、接触余量。
- 右臂可达区、底盘观察点到充电器的安全间距。
- 真实位姿误差、D405 距离误差、YOLO 中心误差，用于确定“目标尺寸足够”的数值门槛。
