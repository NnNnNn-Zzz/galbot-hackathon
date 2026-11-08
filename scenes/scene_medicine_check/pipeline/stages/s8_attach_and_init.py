"""步骤 2：挂夹爪 + 回固定初始位。

为什么必须"先回初始位"再干别的（实验 1 踩过的真 bug）：
  第一次真机跑，`pre_grasp` 直接返回 MotionStatus.FAULT。根因不是位姿算错，
  而是手臂停在上一次遗留的奇怪姿态，规划器从那个姿态出发找不到可行路径。
  官方 example6 的做法是：每次动作序列开头先回一个**固定已知姿态**，
  把状态归零。加上这一步之后实验 1 一次通过。

本 stage 就是把这个"归零"动作独立出来，便于单独复跑（scripts/run_init_pose_only.py
走的是同一条逻辑）。回位目标取自示教点 P1_init 的关节角，而不是 tuning 里写死的数组 ——
换桌/换人重示教后不用改代码。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict

from .. import outcome
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage
from ..teach import _safe_read, load_point as _load_point

logger = logging.getLogger("scene.medicine.stage.attach_and_init")




@register_stage("attach_and_init")
class AttachAndInit(Stage):
    stage_id = "attach"

    def describe(self) -> str:
        pt = self.params.get("init_point", "P1_init")
        return f"挂夹爪 → 回初始位（示教点 {pt}）"

    def run(self, ctx: Context, io) -> StageResult:
        chain = str(self.params.get("chain", "right_arm"))
        tool = str(self.params.get("tool", "galbot_gripper"))
        teach_file = str(self.params.get("teach_file",
                                        "pipeline/configs/teach_drawer.json"))
        init_pt = str(self.params.get("init_point", "P1_init"))
        speed = float(self.params.get("speed_rad_s", 0.2))
        settle_s = float(self.params.get("settle_s", 0.5))

        ret = io.attach_tool(chain, tool)
        if not ret.get("success"):
            return outcome(ret, "attach_tool 失败，无法做末端位姿控制")

        rec = _load_point(teach_file, init_pt)
        names = rec.get("joint_names")
        pos = rec.get("joint_positions")
        if not names or not pos:
            # 没示教也要能动：退回 tuning 里的官方 example6 初始位
            from .. import tuning
            logger.warning("取不到示教点 %s 的关节角，退回 tuning 里的固定初始位", init_pt)
            names, pos = tuning.CARRY_JOINT_NAMES, tuning.CARRY_JOINT_POSITIONS

        ret = io.set_joints(pos, names, speed)
        if not ret.get("success"):
            return outcome(ret, "回初始位失败（规划器可能从异常姿态出发，见实验1 FAULT 记录）")
        if settle_s > 0:
            time.sleep(settle_s)

        chk = _safe_read(io, "get_joint_positions", joint_groups=[chain], joint_names=names)
        dev = None
        if chk.get("success") and len(chk.get("joint_positions") or []) == len(pos):
            dev = max(abs(a - b) for a, b in zip(chk["joint_positions"], pos))
            logger.info("回初始位后最大偏差 %.2f°", dev * 57.2958)

        return ok(f"已挂 {tool} 并回初始位", chain=chain, init_point=init_pt,
                  dev_rad=dev)
