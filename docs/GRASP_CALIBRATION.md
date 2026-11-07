# scene_grasp_cup 真机标定清单

> 抓取场景 mock 编排已验证通过。真机跑通前，以下参数/环节必须现场标定。
> 全部集中在 `scenes/scene_grasp_cup/scene.py` 顶部的可标定参数区。

## 必标定（不标定抓不到）

| 参数 | 位置 | 怎么标 |
|---|---|---|
| `OBSERVE_POSE` | scene.py | 右臂抬到一个让右腕相机能俯视到桌面杯子的位姿。手动示教/挪手臂到桌面正上方，读 `get_ee_pose('right_arm')` 记下来填入 |
| `GRASP_ORIENTATION` | scene.py | 抓取时末端朝向（四元数）。夹爪要竖直朝下对准杯子。先真机示教一个"朝下"姿态读四元数填入 |
| 深度-彩色对齐 | cameras.json 标 unverified | capture 后比对 rgb 和 depth 的 width/height；不一致则 deproject 里已按比例映射像素，但**光心不同仍会错位**，需实测确认 |

## 要调精度（影响成功率）

| 环节 | 风险 | 对策 |
|---|---|---|
| VLM 像素精度 | deepseek-flash/qwen-vl-max 给的 (u,v) 有误差 | 让 VLM 给杯子**中心**而非边缘；反投影前可在 bbox 内取深度中位数更稳 |
| 深度值 | 杯壁反光/透明可能深度无效 | scene 已判 z<=0.05 拒绝；换个角度/距离重拍 |
| 预抓取高度 `PRE_GRASP_LIFT_M` | 太高碰不到，太低撞杯子 | 默认 0.12m，按杯子高度调 |
| 下沉量 `GRASP_SINK_M` | 夹爪要伸到杯子中部 | 默认 0.02m，按杯子高度调 |

## 建议的真机调试顺序

1. **先单测取图**：`wrapper.capture_right_wrist()` 确认真机出图、深度有效、内外参正常
2. **再单测 VLM**：把取的图发给 VLM，看它给的像素坐标准不准
3. **单测反投影**：对一个已知位置的物体，反投影出的 3D 坐标和卷尺实测比对
4. **单测手臂**：move_arm_to 到一个安全位姿，确认 GalbotMotion 在 CHASSIS_POSE_CTRL 下也能动手臂（单例共存验证）
5. **最后全链路**：跑 scene_grasp_cup

## 单例共存待验证（重要）

wrapper 的 `_get_robot()` 切了 CHASSIS_POSE_CTRL（底盘），现在又要用 GalbotMotion（手臂）。
两者共用 SDK 进程，**真机要确认：切了底盘控制器后，手臂运动是否仍正常**。
若手臂不动，可能要切换控制器模式或调整 init 顺序（官方是 motion 先 init）。
