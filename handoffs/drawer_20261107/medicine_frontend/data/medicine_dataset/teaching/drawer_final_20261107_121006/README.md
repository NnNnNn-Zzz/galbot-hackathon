# 抽屉把手最终示教采集

本批次 `drawer_final_20261107_121006` 是唯一保留的当前抽屉对位记录。四路 RGB 和双腕深度与 SDK 状态在同一次并发采集中获取。用户先前确认右夹爪对位正确；本次右腕图中把手仍位于两指之间。夹爪尚未闭合，没有抓稳或拉屉结果。

| 量 | 最终读数 |
|---|---|
| 底盘位置 map，米 | (-1.148284, 0.037167, 0.008464) |
| 底盘 yaw | -88.7087°；完整四元数见 teaching_record.json |
| 底盘状态 | connected=true，localized=true，导航状态 `UNKNOWN` |
| 右手 EndEffector 位置 base_link，米 | (0.751722, -0.238316, 0.822836) |
| 右手 EndEffector 四元数 xyzw | (-0.699969, 0.042836, 0.014368, 0.712742) |
| 右夹爪反馈开度 | 50.004 mm，is_moving=false |
| 右夹爪 effort | 2.914286（SDK 原值，物理力单位未核定） |
| 数值关节状态 | 共 25 个，右臂 7 轴如下 |

| 右臂关节 | 位置 rad |
|---|---|
| right_arm_joint1 | -0.235968500 |
| right_arm_joint2 | 1.427249908 |
| right_arm_joint3 | 0.137818173 |
| right_arm_joint4 | 0.438260883 |
| right_arm_joint5 | -1.154704571 |
| right_arm_joint6 | -0.074782118 |
| right_arm_joint7 | 0.271609873 |

| 相机 | RGB 设备时间（北京时间） | RGB-D 主机采集时间差 | 质量标记 |
|---|---|---|---|
| head_right | 2026-11-07T12:10:07.493+08:00 | 无深度 | 无 |
| head_left | 2026-11-07T12:10:07.927+08:00 | 无深度 | 无 |
| wrist_right | 2026-11-07T12:10:08.040+08:00 | -31.7 ms | clock_offset_detected, joint_state_not_numeric, geometry_calibration_unverified |
| wrist_left | 2026-11-07T12:10:08.884+08:00 | 701.7 ms | joint_state_not_numeric, geometry_calibration_unverified |

SDK 遥测窗口为 2026-11-07T12:10:06.817+08:00 至 2026-11-07T12:10:08.858+08:00，底盘/关节/夹爪和末端查询各有独立时间戳，详见原始 `sdk_snapshot.json`。各设备时钟可能有偏差；并发采集不等于硬件同步。右腕 RGB-D 主机采集时间差为 -31.7 ms，左腕为 701.7 ms。右腕存在时钟偏差告警，RGB-D 几何配准尚未验证。

完整原始数据索引在 `teaching_record.json`，图像和深度在 `raw/wrist/drawer_final_20261107_121006`、`raw/head/drawer_final_20261107_121006`，SDK 原始状态在 `sdk_snapshot.json`。SQLite 的 captures、frames、state_snapshots 均已登记。本次未下发夹爪、手臂或底盘动作。

当前数据可以用于后续对位分析；地图原点/版本、工具变换及夹持验证仍缺，**不能直接当作可回放抓取轨迹**。
