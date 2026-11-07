# 位姿示教操作流程（药品场景 · 右臂）

目的：把 `pipeline/tuning.py` 里 3 个 TODO 占位位姿（`PRE_GRASP_POSE` /
`GRASP_POSE` / `LIFT_POSE`）换成真机实测值。**这是上真机后唯一必须现场做的事**，
不做的话抓取一定抓空（占位值是拍脑袋的）。

## 坐标系常识（先记住再动手）

- 全部位姿在 `base_link` 系：**x 向前、y 向左、z 向上**，单位米；
  姿态是四元数 `[qx,qy,qz,qw]`，共 7 个数：`[x,y,z,qx,qy,qz,qw]`。
- 右臂作业区 **y < 0**（机器人右手侧）。4 个药盒从左到右 = y 从大到小。
- 「左起第 4 个盒子」= y 最小（最靠右）的那个点。

## 前置条件

1. 机器人在场、已上电，PC 与机器人同 LAN（参考 MEMORY 里的网段表）。
2. PC 侧环境：`source ~/galbot/galbot_sdk/linux-x86_64-gcc940/setup.sh`。
3. 场地安全：桌面清空只留药盒，急停按钮位置确认，一人盯机一人操作。

## 步骤 0：冒烟（必做，5 分钟）

```bash
cd ~/projects/galbot-hackathon
MED_IO=wrapper python3 scripts/smoke_medicine_hw.py
```

预期输出每步 `"success": true`，肉眼看到夹爪开→合一次。
把整段日志发给 Agent 复核（S1-S4 全 true、S5 允许降级）。
**冒烟不过，禁止进入示教。**

## 步骤 1：摆臂取数（每个位姿重复一遍）

对 3 个位姿各做一轮：

1. **摆位**：用示教手柄/零力拖动（场地提供的方式）把右臂末端挪到目标位：
   - `PRE_GRASP_POSE`：目标盒正上方约 10~15 cm，夹爪朝下、张开不碰盒；
   - `GRASP_POSE`：下探到夹爪包住盒体中部的位置（闭合即夹住的高度）；
   - `LIFT_POSE`：抓住后竖直抬起约 15 cm 的安全高度（不碰邻盒）。
2. **读数**：保持手臂不动，PC 上跑：

   ```bash
   python3 -c "
   import os; os.environ['MED_IO']='wrapper'
   from sdk_wrapper import wrapper
   import json
   print(json.dumps(wrapper.get_ee_pose('right_arm'), ensure_ascii=False, indent=2))
   "
   ```

3. **记录**：把返回的 `pose` 7 个数原样抄下（别四舍五入）。
4. **复检**：再读一次，两次差异应 < 1 mm；差异大说明臂没稳住或被碰了，重摆。

## 步骤 2：回填 tuning.py

把 3 组数填进 `pipeline/tuning.py`，删掉对应 TODO 注释：

```python
PRE_GRASP_POSE = [x, y, z, qx, qy, qz, qw]  # 示教于 2026-11-XX，左起第 4 盒上方
GRASP_POSE     = [...]                       # 同盒，夹取位
LIFT_POSE      = [...]                       # 抬起位
```

要抓**多个盒位**时：每个盒位重复步骤 1 各采一组 GRASP_POSE，
存成 `GRASP_POSES = {"box_1": [...], ..., "box_4": [...]}`，policy 按目标选位。
（基线 `grasp_first_box` 只认单个 GRASP_POSE，多位姿是策略层的活。）

## 步骤 3：空抓验证（实验 1）

1. 桌面上只放 1 个空盒在示教点；
2. 只跑 s0+s1（抓取阶段），肉眼确认：预抓取→直线逼近→合爪→抬起→收臂，
   盒子离桌即成功；
3. 失败后**只调 GRASP_POSE 的 x/z**（前后/高度），每次改 ≤ 1 cm，
   不要同时改两个轴，改完重示教复检。

## 注意事项

- `move_line_to`（逼近段）带碰撞检查，若反复失败先怀疑 GRASP_POSE 下探过深
  戳到桌面，把 z 抬 5 mm 再试。
- 示教时夹爪保持**张开**（0.08 m），避免刮碰；闭合动作由代码执行。
- 所有示教数据当场抄进调参记录（哪盒、哪位姿、日期），别只留在终端里。
