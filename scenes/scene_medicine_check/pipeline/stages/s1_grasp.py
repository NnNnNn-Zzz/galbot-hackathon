"""步骤 1（基线）：右手抓取桌面左起第一个药盒（右臂作业区 y<0，见 tuning.py）。

官方参考 tutorials/example6_pick_and_place.py：
    张开夹爪 → 预抓取位姿 → 直线逼近抓取位姿 → 闭合夹爪 → 抬起 → 收臂抱持

这是**可替换点 #1**。队友要做「识别日期 → 比对 → 找到过期的那盒 → 夹取」时，
不要改本文件，新建 policies/xxx.py 注册一个新名字，然后在配置里换 stage 名即可。
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Dict, List, Tuple

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.stage.grasp")


@register_stage("grasp_first_box")
class GraspFirstBox(Stage):
    """固定位姿抓取，不识别、不选目标 —— 基线用。"""

    stage_id = "grasp"

    def describe(self) -> str:
        return "抓取桌面左起第一个药盒（固定位姿，不识别）"

    def run(self, ctx: Context, io) -> StageResult:
        # 抓取前先拍一帧留证（非致命）：盲抓版本用于事后复盘「盒子到底在不在点上」，
        # 后续 VLM 版本在同一个取图点换成「识别校验通过才下爪」。
        shot = io.capture_image(tag="pre_grasp")
        ctx.flags["grasp.pre_image"] = shot.get("frame_id")
        if not shot.get("success"):
            logger.warning("抓取前拍照失败（%s），盲抓流程不受影响", shot.get("message"))

        chain = self.params.get("chain", "right_arm")
        tool = self.params.get("tool", "galbot_gripper")
        pre = self.params.get("pre_grasp_pose", tuning.PRE_GRASP_POSE)
        grasp = self.params.get("grasp_pose", tuning.GRASP_POSE)
        lift = self.params.get("lift_pose", tuning.LIFT_POSE)

        plan: List[Tuple[str, Callable[[], Dict[str, Any]], str]] = [
            ("attach_tool", lambda: io.attach_tool(chain, tool),
             "attach_tool 失败，无法做末端位姿控制"),
            ("open_gripper", lambda: io.set_gripper(
                tuning.GRIPPER_OPEN_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N), "夹爪张开失败"),
            ("pre_grasp", lambda: io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S),
             "移动到预抓取位姿失败"),
            ("approach", lambda: io.move_line_to(grasp), "直线逼近抓取位姿失败"),
            ("close_gripper", lambda: io.set_gripper(
                tuning.GRIPPER_CLOSE_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N), "夹爪闭合失败"),
            ("lift", lambda: io.set_arm_pose(lift, tuning.MOVE_TIMEOUT_S), "抬起失败"),
        ]

        for name, fn, err in plan:
            ret = fn()
            if not ret.get("success"):
                return outcome(ret, f"{err}（{name}）", sub_step=name)

        # 抓着东西走之前先把胳膊收回抱持位，避免转身时扫到桌面
        ret = io.set_joints(tuning.CARRY_JOINT_POSITIONS, tuning.CARRY_JOINT_NAMES,
                            tuning.ARM_SPEED_RAD_S)
        if not ret.get("success"):
            logger.warning("收臂失败（%s），仍继续，但转身时有碰撞风险", ret.get("message"))

        # 是否有东西在夹爪里：只有能读到夹爪状态的硬件才说得清，读不到就是 None
        ctx.picked = io.is_holding()
        if ctx.picked is None:
            logger.warning("本后端无夹爪反馈，夹取成功与否无确证手段（picked=unknown）")

        return ok("已抓起左起第一盒药品并收臂",
                  picked=ctx.picked, verified=ctx.picked is not None)
