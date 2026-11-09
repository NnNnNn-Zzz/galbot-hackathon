"""【VLM 校验版抓取】下爪前先用多模态大模型看一眼，确认盒子在点上再抓。

与 `s1_grasp.GraspFirstBox`（盲抓）的区别，只在**下爪前多一道校验门**：

    盲抓版：  attach → 开爪 → 预抓取 → 直线逼近 → 合爪 → 抬起 → 收臂
    VLM 版：  attach → 开爪 → 预抓取 → 【拍照 → VLM 判断】→ ?
                                            ├─ 通过 → 直线逼近 → 合爪 → 抬起 → 收臂
                                            └─ 不通过 → 按建议微调位姿 → 重拍复验（最多 3 轮）
                                                          └─ 仍不行 → failed（不再下爪，走归位）

安全红线：
  1. VLM 只有**建议权**：偏移量必经 `clamp_correction()` 限幅（单次 ≤3cm、累计 ≤6cm）；
  2. 修正次数硬上限（默认 3 轮），超了直接 failed，不无限重试；
  3. VLM 未配置 / 超时 / 返回乱码 / 置信度低 → 一律**停爪**，绝不当成"可以抓"；
  4. `box_present=False` → 返回 failed(NO_TARGET)，**不抓空**；
  5. 每次修正都记日志（给了什么建议、实际改了多少），便于复盘。

替换方式（不动 runner）：
    复制 configs/medicine.baseline.json 为 medicine.vlm.json，
    把 grasp 那一步的 "stage" 改成 "grasp_vlm" 即可。

    python3 -c "from scenes.scene_medicine_check import baseline_flow; \
                baseline_flow.run({'pipeline': 'medicine.vlm', 'io': 'wrapper'})"
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Dict, List, Tuple

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok, unknown
from ..perception import vlm_check
from ..registry import register_stage

logger = logging.getLogger("scene.medicine.policy.vlm_grasp")

#: 最多允许几轮「拍照→判断→修正」
MAX_VERIFY_ROUNDS = 3


@register_stage("grasp_vlm")
class VLMGrasp(Stage):
    """固定位姿 + VLM 校验门：确认盒子在点上才下爪。"""

    stage_id = "grasp"

    def describe(self) -> str:
        return ("预抓取后拍照交多模态大模型校验，通过才下爪"
                f"（最多 {self.params.get('max_rounds', MAX_VERIFY_ROUNDS)} 轮修正）")

    # ------------------------------------------------------------------ 内部
    def _backend(self, io) -> vlm_check.VLMBackend:
        """后端选择：显式注入 > 显式参数 > mock 后端 > 环境变量配置。

        `io._vlm_override` 是测试/调试用的注入点（测试脚本直接塞假后端）。
        走 mock IO 时默认 StaticVLM，保证回归测试不联网。
        """
        override = getattr(io, "_vlm_override", None)
        if override is not None:
            return override
        kind = str(self.params.get("vlm_backend") or "")
        if not kind and getattr(io, "backend", "") == "mock":
            kind = "static"
        return vlm_check.build_backend(kind)

    def _verify_once(self, ctx: Context, io, backend, tag: str) -> Tuple[Any, str]:
        """拍一张 + 问一次模型，返回 (verdict, image_ref)。"""
        shot = io.capture_image(tag=tag)
        image_ref = shot.get("frame_id")
        if not shot.get("success"):
            return (vlm_check.VLMVerdict(
                ok=False,
                message=f"拍照失败，无法校验：{shot.get('message', '')}"),
                image_ref)
        verdict = backend.judge(str(image_ref))
        return verdict, image_ref

    # -------------------------------------------------------------------- run
    def run(self, ctx: Context, io) -> StageResult:
        chain = self.params.get("chain", "right_arm")
        tool = self.params.get("tool", "galbot_gripper")
        pre = list(self.params.get("pre_grasp_pose", tuning.PRE_GRASP_POSE))
        grasp = list(self.params.get("grasp_pose", tuning.GRASP_POSE))
        lift = list(self.params.get("lift_pose", tuning.LIFT_POSE))
        max_rounds = int(self.params.get("max_rounds", MAX_VERIFY_ROUNDS))

        plan: List[Tuple[str, Callable[[], Dict[str, Any]], str]] = [
            ("attach_tool", lambda: io.attach_tool(chain, tool),
             "attach_tool 失败，无法做末端位姿控制"),
            ("open_gripper", lambda: io.set_gripper(
                tuning.GRIPPER_OPEN_M, tuning.GRIPPER_SIDE,
                tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N), "夹爪张开失败"),
            ("pre_grasp", lambda: io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S),
             "移动到预抓取位姿失败"),
        ]
        for name, fn, err in plan:
            ret = fn()
            if not ret.get("success"):
                return outcome(ret, f"{err}（{name}）", sub_step=name)

        # ══════════════════ VLM 校验门（本文件的全部价值所在）══════════════════
        backend = self._backend(io)
        total_fix = 0.0
        verdict = None
        for rnd in range(1, max_rounds + 1):
            verdict, image_ref = self._verify_once(ctx, io, backend, f"vlm_r{rnd}")
            logger.info("[VLM 第%d轮] 图=%s 判断=%s", rnd, image_ref,
                        verdict.to_dict())

            if not verdict.ok:
                # 模型不可用/拍照失败：保守停爪，绝不硬抓
                return unknown(f"视觉校验不可用（{verdict.message}），停爪不抓",
                               code=Code.NOT_VERIFIED,
                               rounds=rnd, image=image_ref)

            if not verdict.box_present:
                return failed(f"夹爪下方没有检测到药盒（{verdict.reason}），不再下爪",
                              code=Code.NO_TARGET, rounds=rnd, image=image_ref)

            if verdict.graspable and verdict.confidence >= vlm_check.MIN_CONFIDENCE:
                logger.info("[VLM 第%d轮] 通过，可以下爪（置信度 %.2f）",
                            rnd, verdict.confidence)
                break

            # 不通过：按建议小幅修正，然后重拍
            if rnd >= max_rounds:
                return failed(
                    f"校验 {max_rounds} 轮仍未通过（{verdict.message or verdict.reason}），"
                    f"累计修正 {total_fix * 100:.1f}cm，停止下爪",
                    code=Code.SAFETY_ABORT, rounds=rnd,
                    last_offsets=[verdict.offset_x_cm, verdict.offset_y_cm])

            dx_cm = verdict.offset_x_cm or 0.0
            dy_cm = verdict.offset_y_cm or 0.0
            dx_m, dy_m, note = vlm_check.clamp_correction(
                dx_cm / 100.0, dy_cm / 100.0, total_fix)
            if dx_m == 0.0 and dy_m == 0.0 and note:
                return failed(f"修正量被拒绝（{note}），停止下爪",
                              code=Code.SAFETY_ABORT, rounds=rnd)
            if note:
                logger.warning("[VLM 第%d轮] %s", rnd, note)

            # base_link 系：x 前、y 左。VLM 的 offset 与之一致（prompt 已定义）
            pre[0] += dx_m
            pre[1] += dy_m
            grasp[0] += dx_m
            grasp[1] += dy_m
            lift[0] += dx_m
            lift[1] += dy_m
            total_fix += max(abs(dx_m), abs(dy_m))
            logger.warning("[VLM 第%d轮] 未通过，按建议微调 dx=%.1fcm dy=%.1fcm（累计 %.1fcm）",
                           rnd, dx_m * 100, dy_m * 100, total_fix * 100)

            ret = io.set_arm_pose(pre, tuning.MOVE_TIMEOUT_S)
            if not ret.get("success"):
                return outcome(ret, "按校验建议微调位姿失败", sub_step="vlm_adjust")

        # ══════════════════════ 校验通过，正式下爪 ══════════════════════
        ctx.flags["grasp.vlm_verdict"] = verdict.to_dict()
        ctx.flags["grasp.vlm_correction_m"] = round(total_fix, 4)

        ret = io.move_line_to(grasp)
        if not ret.get("success"):
            return outcome(ret, "直线逼近抓取位姿失败", sub_step="approach")

        ret = io.set_gripper(tuning.GRIPPER_CLOSE_M, tuning.GRIPPER_SIDE,
                             tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "夹爪闭合失败", sub_step="close_gripper")

        ret = io.set_arm_pose(lift, tuning.MOVE_TIMEOUT_S)
        if not ret.get("success"):
            return outcome(ret, "抬起失败", sub_step="lift")

        ret = io.set_joints(tuning.CARRY_JOINT_POSITIONS, tuning.CARRY_JOINT_NAMES,
                            tuning.ARM_SPEED_RAD_S)
        if not ret.get("success"):
            logger.warning("收臂失败（%s），仍继续，但转身时有碰撞风险", ret.get("message"))

        ctx.picked = io.is_holding()
        if ctx.picked is None:
            logger.warning("本后端无夹爪反馈，夹取成功与否无确证手段（picked=unknown）")

        return ok("VLM 校验通过，已抓起药盒并收臂",
                  picked=ctx.picked,
                  vlm_verdict=verdict.to_dict() if verdict else None,
                  vlm_correction_m=round(total_fix, 4))
