# SDK 踩坑清单（活文档，每次调错必须追加）

> **纪律**：任何一次 SDK 调用异常，5 分钟内按下面的模板追加一条到本文件，并在文末「更新记录」加一行。
> 不写 = 下次必然重踩。写完 commit（`git commit -m "docs(gotchas): <一句话>"`）。
>
> 平台：galbot-echo / aarch64 / Python 3.8.10 / GalbotSDK V1.10.0 / SN R001ABDD22AA0480

## 追加模板

```markdown
### G-编号 一句话现象
- **报错原文**：`<粘贴真实报错/返回>`
- **触发条件**：什么姿势、什么参数、什么调用顺序
- **真因**：一句话本质原因（不是表面现象）
- **正确做法**：可直接照做的修正
- **涉及接口**：`xxx.yyy()`
- **发现日期 / 验证**：YYYY-MM-DD，真机 / mock
```

---

## 一、初始化与环境

### G-01 `GalbotRobot.init() 返回 False（通信总线未建立）`
- **触发条件**：`sensors=("BASE_LIDAR", "HEAD_LIDAR")` 这类雷达传感器名传进 init
- **真因**：SDK 的 `SensorType` 里**列了** `BASE_LIDAR / CHASSIS_LIDAR / HEAD_LIDAR / BACK_LIDAR`，
  但这台的 SDK 配置没启用雷达；`LidarData` 类型虽存在（字段是 PointCloud2 那套
  `data/fields/point_step/width/height/is_dense`），却读不到数据
- **正确做法**：init 只传相机类传感器（`*_CAMERA` / `*_DEPTH_CAMERA`）。
  要用雷达得先让厂商/配置把对应 sensor enable，否则 `init` 直接失败
- **涉及接口**：`GalbotRobot.init(enable_sensor_set)`
- **发现日期**：2026-11-07，真机

### G-02 `GalbotMotion.init() 返回 False` / 各种 `init` 莫名失败
- **真因**：**同一时间有两个进程碰 SDK**。SDK 是进程内单例，抢占会失败
- **正确做法**：跑之前 `pgrep -af python3` 确认没有 `agent/main.py`、别的测试脚本在跑；
  一个进程退出后再起下一个
- **发现日期**：2026-11-07，真机

### G-03 `无法导入 galbot_sdk`
- **触发条件**：没 `source scripts/env_hpu.sh`，或 `PYTHONPATH` 没含 `~/g1_runtime`
- **正确做法**：先 `source scripts/env_hpu.sh`（设 `PYTHONPATH` + `LD_LIBRARY_PATH=/data/galbot/lib`）；
  或设 `GALBOT_SDK_ROOT` 让 adapter 的 `_sdk_bootstrap()` 自己找包目录
- **涉及接口**：`GalbotAdapter._import_sdk()`
- **发现日期**：2026-11-07

### G-04 相机拿到了但 `data` 是空的
- **真因**：相机传感器**没在 init 的 sensor 集合里**。`SensorType` 里有名字 ≠ init 时被启用
- **正确做法**：init 时把要用的相机都列进去，例如
  `sensors=("LEFT_ARM_CAMERA","RIGHT_ARM_CAMERA","LEFT_ARM_DEPTH_CAMERA","RIGHT_ARM_DEPTH_CAMERA","HEAD_LEFT_CAMERA","HEAD_RIGHT_CAMERA")`
- **涉及接口**：`toolhub.configure(sensors=[...], force=True)`
- **发现日期**：2026-11-07，真机（头部相机一开始就是这个问题）

---

## 二、底盘

### G-05 `set_base_pose` 返回 `WAIT_INITIALIZED`，或明明成功却没动
- **真因**：`switch_controller(CHASSIS_POSE_CTRL)` 之后底盘进入初始化态，**立即下令不生效**
- **正确做法**：切完控制器等 **2 秒**再下令；失败重试间隔也留够 **2 秒**
  （1 秒实测不够，会连续失败）
- **涉及接口**：`GalbotRobot.switch_controller()` / `set_base_pose(..., frame_id="rel(0)")`
- **发现日期**：2026-11-07，真机

### G-06 pybind 枚举 `bool()` 恒为真，`TIMEOUT` 被当成成功
- **真因**：`ControlStatus` / `MotionStatus` 是 pybind11 枚举，`bool(枚举)` 不看值
- **正确做法**：一律用 `_enum_name(res)` / `_status_name(res)` 取成员名再比 `== "SUCCESS"`
- **发现日期**：2026-11-07（早期 wrapper 因此把失败当成功）

---

## 三、手臂

### G-07 `move_ee_to / move_ee_delta` 返回 `FAULT`，但位姿看起来完全正常
- **触发条件**（三种，按概率排序）：
  1. **末端太低横向移动**：z≈0.73（桌面 0.709）时夹爪手指已贴桌面，横向一动就判碰撞
  2. **跨身体中线**：右臂向 +y 到 `y≈-0.14`、左臂向 -y 到 `y≈+0.18` 就到限位
  3. **瞬时故障**：同一 pose 下偶尔失败，下一次就好
- **正确做法**：
  1. 先在 **z ≥ 0.85** 的高度完成横向对位，最后再 `down` 降下来夹
  2. 用 `turn` 转底盘让目标落到手臂自己的工作空间，别硬够
  3. 失败后改 `move_arm_to(绝对位姿)` 重试一次，实测常能成功
- **涉及接口**：`set_end_effector_pose` / `move_line`（底层都走 `_execute_cartesian`）
- **发现日期**：2026-11-07，真机（连踩三次）

### G-08 手臂挡住头部相机
- **真因**：头部相机在 1.465 m 高、俯视；手臂伸到杯子跟前时正好遮住目标
- **正确做法**：贴近后 VLM 找不到目标属于正常现象，切到"只有腕部图"的收尾分支，
  或在头部图上标注夹爪 FK 投影作为参照
- **发现日期**：2026-11-07，真机

---

## 四、感知

### G-09 深度值离谱（例：44,060,000 m）
- **真因**：`depth_scale` 是**除数**不是乘数。真机 payload：`{"format":"16UC1","depth_scale":10000}`
  → `米 = raw / depth_scale`；写成 `raw * scale` 就放大一万倍
- **正确做法**：用 dev 层现成的 `perception.decode_depth_payload(payload)`（16UC1 / compressedDepth PNG 都吃），
  **不要手搓解码**
- **涉及接口**：`robot.get_depth_data()` / `toolhub.capture_frame()`
- **发现日期**：2026-11-07，真机

### G-10 玻璃杯/透明物的深度完全不可信
- **现象**：同一个杯子，同一像素附近，深度在 0.33~0.44 m 之间乱跳
- **正确做法**：只信任**不透明表面**（桌面）的深度。求杯子 3D 用
  「像素射线 ∩ 桌面平面 z=0.709」，不要直接取杯身像素的深度
- **发现日期**：2026-11-07，真机

### G-11 VLM 返回的像素框越界（992 高的图给到 y=1130）
- **真因**：模型对绝对像素坐标不稳定（缩放/letterbox 都会影响）
- **正确做法**：让它输出 **0~1 归一化坐标**，尺寸由代码乘回去，并做越界校验
- **发现日期**：2026-11-07，真机

### G-12 DeepSeek 视觉返回空 content
- **真因**：请求少了 `response_format: {"type": "json_object"}`
- **正确做法**：请求体必须带这个字段；`temperature=0`
- **发现日期**：2026-11-07，真机

### G-13 射线求交算出 -4.7 m 这种荒唐坐标
- **真因**：旋转写成了转置。`sum(R[i][j]*d[i] for i in range(3))` 实际是 `Rᵀ·d`
- **正确做法**：`d_base = [sum(R[r][c]*d_cam[c] for c in range(3)) for r in range(3)]`
- **发现日期**：2026-11-07，真机

### G-14 夹爪报「未知夹爪」，一切夹爪调用被拒
- **真因**：`G1JointGroup` 是 pybind11 类型，静态属性是**描述符**；`vars()` 取到描述符对象而不是字符串，
  白名单被过滤成空
- **正确做法**：必须 `getattr(gm.G1JointGroup, name)` 取值
- **涉及接口**：`GalbotRobot.set_gripper_command()`
- **发现日期**：2026-11-07，真机（D5 阶段）

---

## 五、工程

### G-15 Python 3.8 上的语法坑
- **真因**：机器人是 3.8.10；`dict[str, int]` / `X | None` 在**运行时**求值的地方会报错
- **正确做法**：文件头一律 `from __future__ import annotations`；不要在 `isinstance` 等运行期位置用新语法
- **发现日期**：2026-11-07

### G-16 部署把队友的改动冲掉
- **真因**：机器人 `~/galbot_hackathon` 是团队共用目录，队友直接在上面改过 `wrapper.py`（607 行，药品 P0 接口），
  且目录**没有 .git**；`rsync --delete` 会静默覆盖
- **正确做法**：实验只推 `~/galbot_hackathon_dev`；动共用目录前先 `cp -a` 备份到 `~` 下；rsync 带 `--exclude ".bak_*"`
- **发现日期**：2026-11-07

---

### G-17 整机莫名进入数采模式（DATA_COLLECTION_MODE），所有运动接口失效
- **报错原文**：
  `I20261107 23:53:47 mode_arbitration.cpp:881] [Iteration]: current_mode_instance_ = DATA_COLLECTION_MODE`
  `E20261107 23:58:48 machine_mode.cpp:298] [isProcessReady]: process galbot_vtn is not running`
  现象侧：`get_ee_pose` 报 `GalbotMotion.init() 返回 False`；`set_base_pose` 返回 `TIMEOUT` 且底盘零位移
- **触发条件**：有人跑过官方场景切换脚本 `/data/tools/service_display/mode_change_select.sh` 并选了 `data_collection`
  （root 的 `~/.bash_history` 末尾能看到），之后**重启整机**才生效 —— 所以典型表现是「隔了一段时间突然变数采」
- **真因**：`/userdata/user_config/mode_service_config_default.toml` 的 `[start_up_mode]` 被整体替换成数采场景：
  `auto_enter_mode_type = "DATA_COLLECTION_MODE"`、`apply_scene = "data_collection"`，且全文 `para_dir`
  从 `/userdata/user_config/base` 改为 `.../data_collection`。mission_manager **每次启动按这个值自动进模式**，
  不是运行时谁手滑切的，所以重启也回不到 WORKING
- **正确做法**：
  1. 诊断：`ssh root@192.168.1.66` → `head -5 /userdata/user_config/mode_service_config_default.toml`
     看 `auto_enter_mode_type`；对照出厂默认 `/data/config/default/mode_service_config_default.toml`（应为 `WORKING_MODE` / `base`）
  2. 切回：`cd /data/tools/service_display && ./mode_change_select.sh --mode base`
     （不带 `--force` 会 8 秒后自动重启整机；带 `--force` 只改配置 + 同步 HPU、不重启，需自己重启）
  3. **必须重启才生效**。重启后校验：`grep "current_mode_instance_ = " /userdata/log/mission_manager/mission_manager.INFO | tail -1` → `WORKING_MODE`
- **涉及接口**：整机模式（mission_manager / XCU），所有运动类 SDK 调用都受影响
- **发现日期 / 验证**：2026-11-07，真机（SN R001ABDD22AA0480）

### G-18 切回 WORKING_MODE 后 `singorix_wbcs_main` 起来又崩，`GalbotMotion.init()` 仍失败
- **报错原文**：
  ```
  E controller_manager.cpp:819] [WBCS][CONTROLLER][SHUTDOWN_REJECTED] action=register, device=right_arm, process_exit=1
  E main.cpp:462] Init fail
  W wbcs_manager.cpp:898] [WBCS][SERVICE_STATE][ABNORMAL] source=embosa_exit_callback, reason=shutdown_requested_before_embosa_shutdown
  ```
  mission_manager 侧：`handle_child_exits_safely]: Erased process: singorix_wbcs_main, pid: 5818`，之后**不会自动重拉**
- **触发条件**：模式切换后的整机重启 / `launcher` 重启过程中，wbcs 正在注册右臂控制器
  （日志上一条还是 `galbot_compliance_base_controller.cpp:114] right_arm init successful`）
- **真因**：初始化中途 Embosa 收到退出回调（`shutdown_requested_before_embosa_shutdown`），controller_manager
  进入 shutdown gate 后拒绝后续 `register`，wbcs 主动 `Init fail` 退出；此时 `relaunch_flag=true` 也**不会**重拉。
  属启动时序问题，不是配置写错、也不是代码问题
- **正确做法**：`ssh root@192.168.1.66` → `systemctl restart launcher`，等 ~45s，
  `ps -ef | grep singorix_wbcs_main` 应看到 `/data/bin/singorix_wbcs_main /userdata/user_config/base`；
  再从 HPU 跑 `python3 scripts/selftest_dev_layer.py status`，`get_ee_pose` 应为 PASS
- **涉及接口**：`GalbotMotion.init()` 及全部运动接口
- **发现日期 / 验证**：2026-11-07，真机

---


### G-19 `capture_frame("head_left")` 静默失败，头部相机取不到图
- **报错原文**：无报错；`capture_frame` 返回 `{"success": False, "message": "HEAD_LEFT_CAMERA 取图失败（无数据）"}`，
  调用方若做了跳过兜底则完全无感（vlm_pilot 跑了 28 步头部相机一张图都没有）
- **触发条件**：用 toolhub/adapter 默认配置取头部相机
- **真因**：`GalbotAdapter.DEFAULT_SENSORS` 只列了四个臂部相机（左右 RGB + 左右深度），
  `robot.init(enable_sensor_set)` 只开启集合内的传感器，头部相机根本没被启用
- **正确做法**：`DEFAULT_SENSORS` 加入 `HEAD_LEFT_CAMERA`（已修，commit f4f85ae）；
  自定义 sensors 时记得把头/腕相机都列全
- **涉及接口**：`GalbotRobot.init(enable_sensor_set)` / `toolhub.capture_frame()`
- **发现日期**：2026-11-08，真机

---


### G-20 极小位移的 `move_line` 永不返回（进程挂死，无报错）
- **报错原文**：无。调用不返回、不抛错、`timeout=10` 也不生效；
  进程只能 `kill`，表现是"脚本卡住、连 print 都没输出"
- **触发条件**：抬到安全高度时"补最后一小步"——当前 z=0.8484，目标 z=0.8500，
  **下发 1.6 mm 的直线运动**。同一进程先前 5 cm 的步都正常
- **真因**：位移量小到规划器认为无需规划，`blocking=True` 的执行回调却不返回，
  于是同步等待永远不结束。不是崩溃，是死等
- **正确做法**：给所有自动补偿的小步设容差和下界 ——
  ① 判断"是否需要抬"时留 5 mm 容差（`abs(z-safe_z)>0.005` 才算不达标）；
  ② 剩余量 <5 mm 直接跳过，不下发；
  ③ 对外接口拒绝 `|distance| < 5 mm` 的请求（`move_along_gripper` 已按此实现）。
  另：跑长脚本一律 `python3 -u`，否则输出被缓冲，卡住时完全看不到进行到哪
- **涉及接口**：`GalbotMotion.move_line()`
- **发现日期**：2026-11-08，真机

---


### G-21 `inverse_kinematics` 参数类型：传 `Pose` 报错、漏姿态静默 INVALID_INPUT
- **报错原文**：
  `TypeError: inverse_kinematics(): incompatible function arguments. ... target_pose: collections.abc.Sequence[typing.SupportsFloat]`
  / 以及**不报错**的一版：返回 `MotionStatus.INVALID_INPUT`（31 个点全失败）
- **触发条件**：① `target_pose=gm.Pose([...])`；② 只传 `[x,y,z]` 三个数、漏掉四元数
- **真因**：`inverse_kinematics` 要**纯 list 的 7 个数**（xyz + qx,qy,qz,qw），
  和 `move_line` 的 `t.cart.pose = gm.Pose([...7])` 正好相反。漏姿态时长度不足，
  它不抛异常、只返回 `INVALID_INPUT`，极易误判成"目标不可达"
- **正确做法**：IK 一律 `motion.inverse_kinematics(list7, [chain], "EndEffector", "base_link")`；
  看到 `INVALID_INPUT` 先数参数长度，别急着怀疑几何。
  `initial_joint_positions={}` 和 `gm.Parameter()` 作为位置参数补上是**可以**的
- **涉及接口**：`GalbotMotion.inverse_kinematics()` / `inverse_kinematics_by_state()`
- **发现日期**：2026-11-08，真机


---

### G-22 `move_ee_to` 对 3~5 mm 位移返回 SUCCESS 但机器人一动不动；平移+旋转组合走直线会 FAULT
- **报错原文**：① 小步：**无报错**，`data.motion_status == "SUCCESS"`，但前后
  `get_end_effector_pose` 完全一致；② 组合步：
  `末端移动失败（FAULT）`，可同一目标 `check_end_effector_reachable` 却返回 `SUCCESS`
- **触发条件**：① `move_ee_to` 纯平移 3 / 4 / 5 mm；② `move_ee_to(straight_line=True)`
  同时改位置（dx≈-4 mm, dy≈+10 mm）与姿态（绕末端局部 +X 转 5°）
- **真因**：① 位移低于运动服务最小分辨率，规划器认为"已在目标"，直接回 SUCCESS 而不下发。
  比 G-20 的死等更危险——**它假装成功**，脚本会以为动了继续往下走；
  ② `straight_line=True` 的直线规划在有姿态变化时解不出路径 → FAULT，
  与 IK 无关（IK 预检是通的，别去调几何）
- **正确做法**：① 每步要么跳过、要么 ≥6 mm。实测：6 mm→6.24 mm（准）、
  7/8/10/12 mm 均准，**5/4/3 mm 全部 0.0 mm 纹丝不动**；
  下发后必须**回读位姿确认真的动了**，只看返回值会被骗；
  ② 平移+旋转同时下发一律 `straight_line=False`；能拆成两步就拆开
- **涉及接口**：`GalbotAdapter.move_ee_to()` / `_execute_cartesian()` / `check_end_effector_reachable()`
- **发现日期**：2026-11-08，真机（`scripts/align_green_box.py` 视觉伺服调试中）

---

## 更新记录

| 日期 | 条目 | 一句话 |
|---|---|---|
| 2026-11-07 | G-01 ~ G-16 | 首版：把当天真机踩过的 16 个坑全部落盘 |
| 2026-11-07 | G-17 | 机器人莫名进数采模式：真因是 `mode_change_select.sh` 把 `auto_enter_mode_type` 改成 DATA_COLLECTION，重启才生效 |
| 2026-11-07 | G-18 | 切回 WORKING 后 wbcs 因 Embosa exit callback 崩掉且不自动重拉，`systemctl restart launcher` 修 |
| 2026-11-08 | G-19 | 头部相机静默失败：DEFAULT_SENSORS 没列 HEAD_LEFT_CAMERA，init 只开声明过的传感器 |
| 2026-11-08 | G-20 | 1.6 mm 的 move_line 会死等不返回：小步要有容差和下界，脚本一律 `python3 -u` |
| 2026-11-08 | G-21 | IK 要纯 list 7 元，传 `Pose` 报 TypeError、漏四元数静默 INVALID_INPUT |
| 2026-11-08 | G-22 | move_ee_to 对 <6mm 位移回 SUCCESS 但不动（假成功）；平移+旋转组合走直线 FAULT |
