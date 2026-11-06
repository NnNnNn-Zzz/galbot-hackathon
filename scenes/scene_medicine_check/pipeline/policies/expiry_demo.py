"""【示例·插拔区】识别日期 → 与今天比对 → 找过期的那盒 → 夹取。

这份文件是给队友看的**模板**，不是最终实现：真正要写的是 `read_date()`，
其余部分（判过期、按最左优先选、夹取动作序列）可以直接复用。

替换方式（不动 runner、不动其他 stage）：
    1. 复制本文件为新名字，改类名与 @register_stage("...") 的注册名；
    2. 实现 read_date()：真机上用 io.capture_image() 拍照，再自己接 OCR/VLM 解析日期；
    3. 复制 configs/medicine.expiry_demo.json，把 grasp 那一步的 "stage" 改成你的注册名；
    4. run({"pipeline": "你的配置名"})。

诚实性约定：读不出日期就返回 None → 判 unknown，**绝不用今天的日期顶替**，
也不把"没读出来"当成"没过期"。
"""

from __future__ import annotations

import datetime
import logging
import re
from typing import Any, Dict, List, Optional

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok, unknown
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.policy.expiry")

_DATE_RE = re.compile(r"(\d{4})\s*[-/.年]\s*(\d{1,2})\s*[-/.月]\s*(\d{1,2})")


def parse_date(text: str) -> Optional[str]:
    """从任意文本里抠出第一个日期，规范化成 YYYY-MM-DD；抠不到返回 None。"""
    if not text:
        return None
    m = _DATE_RE.search(str(text))
    if not m:
        return None
    y, mo, d = (int(g) for g in m.groups())
    try:
        return datetime.date(y, mo, d).isoformat()
    except ValueError:
        logger.warning("日期非法，丢弃：%s", text)
        return None


def _leftmost_key(obj: Dict[str, Any]) -> float:
    """最左 = base 系 y 最大（G1 base_link 的 y 轴指向机器人左侧）。"""
    pose = obj.get("base_m") or obj.get("ee_pose") or obj.get("pose") or [0, 0, 0]
    try:
        return float(pose[1])
    except (TypeError, IndexError, ValueError):
        return -1e9


@register_stage("grasp_expired_box")
class GraspExpiredBox(Stage):
    """找到最左的过期药盒并抓起来。"""

    stage_id = "grasp"

    def describe(self) -> str:
        return "读日期 → 判过期 → 取最左的过期盒 → 夹取（示例实现，OCR 待补）"

    # ------------------------------------------------------------------ 读日期
    def read_date(self, io, obj: Dict[str, Any]) -> Optional[str]:
        """单个目标的日期读取。**这是队友要替换的那一个函数。**

        真机写法示意（不要照抄，取决于你选的 OCR/VLM）：
            shot = io.capture_image(tag=f"label_{obj['id']}")
            if not shot.get("success"):
                return None
            text = your_ocr(shot["path"])          # ← 你自己的识别
            return parse_date(text)

        示例实现读的是 mock 的固定数据，真机上 fixture() 恒为 None，
        因此会老老实实返回 None（= 读不到），不会假装读到。
        """
        table = io.fixture("label_text")
        if not table:
            logger.warning("当前后端没有日期固定数据，且未接 OCR —— 本实现无法读日期")
            return None
        return parse_date(table.get(str(obj.get("id")), ""))

    # ------------------------------------------------------------------ 抓取
    def _grasp_at(self, ctx: Context, io, target: Dict[str, Any]) -> StageResult:
        chain = self.params.get("chain", "left_arm")
        tool = self.params.get("tool", "galbot_gripper")
        pose = target.get("ee_pose") or target.get("pose")
        if not pose or len(pose) < 3:
            return failed(f"目标 {target.get('id')} 没有可用位姿，不能抓", code=Code.NO_TARGET)

        pre = [float(pose[0]), float(pose[1]), float(pose[2]) + 0.05, 0.0, 0.0, 0.0, 1.0]
        grasp = [float(pose[0]), float(pose[1]), float(pose[2]), 0.0, 0.0, 0.0, 1.0]
        lift = [float(pose[0]), float(pose[1]), float(pose[2]) + 0.06, 0.0, 0.0, 0.0, 1.0]

        plan = [
            ("attach_tool", lambda: io.attach_tool(chain, tool)),
            ("open_gripper", lambda: io.set_gripper(
                tuning.GRIPPER_OPEN_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)),
            ("pre_grasp", lambda: io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S)),
            ("approach", lambda: io.move_line_to(grasp)),
            ("close_gripper", lambda: io.set_gripper(
                tuning.GRIPPER_CLOSE_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)),
            ("lift", lambda: io.set_arm_pose(lift, tuning.MOVE_TIMEOUT_S)),
        ]
        for name, fn in plan:
            ret = fn()
            if not ret.get("success"):
                return outcome(ret, f"夹取失败于 {name}", sub_step=name)

        ret = io.set_joints(tuning.CARRY_JOINT_POSITIONS, tuning.CARRY_JOINT_NAMES,
                            tuning.ARM_SPEED_RAD_S)
        if not ret.get("success"):
            logger.warning("收臂失败（%s），仍继续", ret.get("message"))
        return ok(f"已夹起 {target.get('id')}")

    # ------------------------------------------------------------------ 主流程
    def run(self, ctx: Context, io) -> StageResult:
        objects: List[Dict[str, Any]] = list(ctx.objects) or list(io.fixture("objects") or [])
        if not objects:
            return unknown("视野内没有任何目标物体，无法判定过期情况", code=Code.NO_TARGET)

        today = datetime.date.today()
        expired: List[Dict[str, Any]] = []
        unreadable = 0

        for obj in objects:
            oid = str(obj.get("id"))
            date_str = self.read_date(io, obj)
            ctx.expiry[oid] = date_str
            if date_str is None:
                ctx.verdict[oid] = "unknown"
                unreadable += 1
                continue
            is_expired = datetime.date.fromisoformat(date_str) < today
            ctx.verdict[oid] = "expired" if is_expired else "valid"
            if is_expired:
                expired.append(obj)

        logger.info("判定结果：%s（读不出 %d 个）", ctx.verdict, unreadable)

        if not expired:
            # 一个过期都没找到，但只要有读不出的，就不能下"全都没过期"的结论
            if unreadable:
                return unknown(f"{unreadable} 个目标日期读不出，无法确认是否还有过期药品",
                               code=Code.UNDETERMINED, verdict=ctx.verdict)
            return failed("桌面上没有过期药品，无需清理", code=Code.NO_TARGET, verdict=ctx.verdict)

        prefer = self.params.get("prefer", "leftmost")
        if prefer != "leftmost":
            return failed(f"不支持的 prefer 策略：{prefer}", code=Code.STAGE_FAILED)
        target = sorted(expired, key=_leftmost_key, reverse=True)[0]
        ctx.target = target
        logger.info("选中过期目标 %s（日期 %s）", target.get("id"), ctx.expiry.get(str(target.get("id"))))

        res = self._grasp_at(ctx, io, target)
        if not res.ok:
            return res
        ctx.picked = io.is_holding()
        return ok(f"已抓起过期药品 {target.get('id')}",
                  target=target.get("id"), date=ctx.expiry.get(str(target.get("id"))),
                  picked=ctx.picked, unreadable=unreadable)
