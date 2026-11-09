# 充电器视觉抓取：Tool 层设计

> 状态：头部米制 strip 目标、SLAM 观察位规划、导航期间目标追踪、头部 charger 3D 交接、`charger_whole` ROI、上表面/整体掩码回退、约 5 cm 斜上方二次观测、全开夹爪、有界下探、闭爪和初步抬升已实现；抓取是否真正成功仍需抬升后的视觉复核。

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

## 6. 阶段 D：斜上方观察、轴向修正与安全夹取

1. 用 D405 掩码深度拟合上表面平面，得到表面中心 `C`、向上法向 `N`、长边轴 `L`、闭合轴 `S=cross(N,L)`。
2. 不直接把表面中心当末端目标。先生成夹爪 TCP 距中心约 `0.05 m`、相对法向倾斜约 `35°` 的斜上方观察位 `P_obs`；相机光轴沿接近轴继续指向 `C`，使腕部相机仍能看到目标。当前 TCP 相对 `EndEffector` 沿接近轴约有 `0.146 m` 偏置，下发前必须换算成法兰位姿，不能把 TCP 的 5 cm 直接当 EndEffector 的 5 cm。
3. 按斜向接近轴和上表面长边轴生成目标姿态，移动完成后把夹爪张开到 G1 当前量程上限约 `0.11 m`。
4. 在斜上方位置重新获取 RGB-D，必须重新通过完整可见、深度有效率、表面平面和 IK 门槛；若目标中心相对旧位姿偏差超过容差，则拒绝直接下探。
5. 从二次观测计算短距离下探位，保持当前斜向接近轴，在表面上方保留安全间隙。按不超过约 `10 mm` 的绝对位姿步长分段下探，每段仍经过碰撞检查。
6. 闭合宽度根据本次实测短边确定，并限制在 `12–25 mm`；`TIMEOUT` 可能表示夹到物体，但只记录为 `contact_limited`，不能据此宣称抓取成功。
7. 沿接近轴反向小步抬起约 `30 mm`，记录 `grasp_state=closed_and_lifted_unverified`；必须在抬起后重新识别并确认目标随夹爪移动，才能报告 `grasp_completed=true`。

### 6.1 `charger_whole` ROI 与上表面回退

腕部精定位采用两级模型：

```text
charger_whole 分割
  -> 完整充电器掩码与 ROI
  -> ROI 内运行旧 charger 上表面分割
  -> 成功：以上表面掩码拟合平面、中心和夹爪轴
  -> 失败：保留 charger_whole 掩码，随机采样局部目标
```

整体模型始终先运行，不会因为旧上表面模型漏检而丢失目标。旧模型在裁剪 ROI
内的掩码必须与整体掩码有足够包含关系，并回投到原图分辨率。若旧模型找不到
上表面，或找到了但三维几何不满足门槛，`whole_fallback` 会对整体掩码做以下
处理后生成候选取点：

1. 排除整体掩码边缘带，避免把物体轮廓外缘作为目标；
2. 在有效 D405 深度像素中做可复现的随机采样；
3. 对每个采样点拟合局部平面，只接受点数足够、RMS 低且法向朝上的候选；
4. 从通过门槛的候选里随机选一个，使用整体掩码平面估计长轴和尺寸；
5. 仍执行深度有效率、触边、标定、坐标系和 IK 安全检查后才生成末端位姿。

回退结果的 `target_mode=whole_mask_random_sample`，并在
`whole_fallback_sampling` 中记录随机种子、尝试次数、拒绝计数和选中的像素。
随机采样不等同于旧上表面精定位，但提供了明确、可审计的退化路径，不再因旧
模型漏检直接拒绝整个任务。

### 6.2 分支稳定的“先平移、后旋转”控制器

笛卡尔目标同时包含较大平移和旋转时，运动服务可能为中间位姿选择远端 IK
分支，规划虽成功，执行阶段仍可能因腿部或另一条手臂碰撞而 `FAULT`。正式
工具链因此统一使用 `PoseTransitionController`：

1. 平移阶段锁定任务开始时测得的四元数，只改变位置，每步最多 `2 cm`；
2. 位置误差进入 `2 mm` 后进入旋转阶段，锁定该时刻的位置，每步最多 `7.5°`；
3. 每一步都从最新实测位姿重新计算下一目标，不累计规划误差；
4. 平移阶段累计姿态漂移超过 `min(rotation_tolerance, 2°)`、旋转阶段位置
   漂移超过 `max(position_tolerance, 5 mm)` 时立即停止；
5. 单步实际变化超过“请求步长 + 小容差”、成功但无实际前进、或超过最大
   步数时停止，防止 IK 分支跳变和重复命令；
6. 所有步骤固定使用 `with_chain_only`，同时保留环境碰撞、自碰撞和 WBC
   轨迹碰撞监控。

该控制器只决定如何把同一个绝对目标拆成安全子步，不修改目标位姿，也不
关闭任何安全监控。外部障碍或右臂/躯干与路径发生真实碰撞时仍会拒绝执行，
但不会再由逐步四元数漂移诱发远端 IK 分支。

## 7. 当前已实现（第一阶段）

- 已训练五折 `charger_whole` 分割模型，选中 Fold 4（Mask mAP50 `0.9886`、Mask mAP50-95 `0.6800`），用于完整充电器检测和 ROI 划分。
- 腕部流程已实现“整体 ROI -> 旧上表面模型 -> 整体掩码随机采样回退”；两种路径共用现有 D405 几何、坐标变换、末端位姿规划和运动安全闸门。
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
- 已用右腕相机与 `galbot_gripper` TCP 只读查询确认末端轴：TCP 沿 local `+X` 偏置约 `0.146 m`，相机光轴也近似 local `+X`，因此 local `+X` 是接近轴；local `Y` 是两指闭合轴，local `Z` 是夹爪长轴。两个绕接近轴的等价 180° 朝向中会选择更接近当前末端姿态的一个。
- pre-grasp 下发前增加只读 IK/碰撞预检；不可达时 `plan` 阶段直接返回 `safety_limit`，不会再把无 IK 解的位姿交给运动服务。

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
