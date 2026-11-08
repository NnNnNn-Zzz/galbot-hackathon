"""步骤 9：抽屉内竖直下探 28 cm → 夹取 → 上探回升。

为什么必须用「分段步进」而不是一条 move_line_to 走完 28 cm：
  1. `sdk_wrapper.move_line_to` 内部把 `params.set_timeout(10.0)` **写死**，
     28 cm 在 10 s 内走不完就会被判超时（不是失败原因，是时间不够）。
  2. 一次性长直线只要中途有人/物进工作区，碰撞检查只能在起点判，
     中途插不进任何"停"的机会。
  分段后：每段 2–4 cm，每段之间有 sleep，任何一段失败都能立刻停下并原路退回。

为什么用「位姿增量」而不是「绝对目标点」：
  下探段的 x/y 必须严格不动（箱内净空按 10 cm 算，横偏 2 cm 就可能蹭壁）。
  所以起点用示教点 P3_dive_ref 实测位姿，之后每步只在 z 上减 delta，
  x/y/四元数照抄起点 —— 从结构上消除累积漂移。

分段策略（depth_cm = 28，step_cm = 4 → 7 段）：
    0 cm ─────────────────────────────────────► 起点（示教点3）
     ├─4 cm 段  每段 move_line_to + sleep
     ├─8 cm 段
     ├─ ...
     ▼ 28 cm                                    终点（药盒处，仍高于桌面）
    闭爪 → 读宽度 → 判是否夹到 → 上探（反向步进）→ 回起点

⚠️ 未标定的量：本 stage 依赖"z 轴正方向确为世界向上"。若现场发现 z 反了，
   第一次只跑到 4 cm 就停（配置里 max_segments=1），别硬来。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage
from ..teach import _safe_read, load_point as _load_point

logger = logging.getLogger("scene.medicine.stage.drawer_dive")

DEG = 57.29577951308232




@register_stage("drawer_dive")
class DrawerDive(Stage):
    """从示教点竖直下探 N 厘米、夹取、再原路回升。"""

    stage_id = "drawer_dive"

    def describe(self) -> str:
        depth = float(self.params.get("depth_cm", 28.0))
        step = float(self.params.get("step_cm", 4.0))
        n = self.params.get("max_segments")
        tail = f"，最多 {n} 段" if n else ""
        return f"从示教点竖直下探 {depth:.0f} cm（每段 {step:.0f} cm{tail}）→ 闭爪 → 回升"

    # ------------------------------------------------------------ 子步骤
    def _step_to(self, io, pose: List[float], label: str) -> Dict[str, Any]:
        ret = io.move_line_to(pose)
        logger.info("%s → x=%.3f y=%.3f z=%.3f : %s",
                    label, pose[0], pose[1], pose[2],
                    "ok" if ret.get("success") else ret.get("message"))
        return ret

    def run(self, ctx: Context, io) -> StageResult:
        # ---------------- 参数 ----------------
        teach_file = str(self.params.get("file", "pipeline/configs/teach_drawer.json"))
        ref_point = str(self.params.get("ref_point", "P3_dive_ref"))
        depth_cm = float(self.params.get("depth_cm", 28.0))
        step_cm = float(self.params.get("step_cm", 4.0))
        max_seg = self.params.get("max_segments")
        settle_s = float(self.params.get("settle_s", 0.6))
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))
        close_m = float(self.params.get("close_m", 0.075))
        open_m = float(self.params.get("open_m", tuning.GRIPPER_OPEN_M))
        hold_s = float(self.params.get("hold_s", 0.5))
        ascend = bool(self.params.get("ascend", True))
        expect_box = self.params.get("expect_box_cm")     # 期望夹到的高度，仅日志用
        #: 只读预演：算完深度/闸门/分段数就返回，**一条运动指令都不发**。
        #: `run_exp.py --plan` 会把这个开关打进所有 drawer_* / exp_replay_chain 的 params。
        dry = bool(self.params.get("dry_run_no_motion") or self.params.get("dry_run"))

        if depth_cm <= 0 or step_cm <= 0:
            return failed(f"深度/步长必须为正（depth={depth_cm}, step={step_cm}）",
                          code=Code.STAGE_FAILED)

        # ---------------- 起点：示教点实测位姿 ----------------
        ref = _load_point(teach_file, ref_point)
        base_pose = ref.get("pose")
        if not base_pose or len(base_pose) != 7:
            return failed(
                f"取不到示教点 {ref_point} 的末端位姿（{teach_file}）。"
                f"下探必须有实测起点，不接受硬编码猜测 —— 请先跑 teach_record",
                code=Code.DEP_MISSING, file=teach_file, point=ref_point)

        # 真机优先用**当前实测**位姿做起点，示教点只作参照比对
        cur = _safe_read(io, "get_ee_pose", chain=str(self.params.get("chain", "right_arm")))
        if cur.get("success") and len(cur.get("pose") or []) == 7:
            start = [float(v) for v in cur["pose"]]
            dev = max(abs(a - b) for a, b in zip(start[:3], base_pose[:3]))
            logger.info("下探起点 = 当前实测位姿；与示教点 %s 位置差 %.1f mm",
                        ref_point, dev * 1000.0)
            if dev > float(self.params.get("ref_tolerance_m", 0.02)):
                logger.warning("当前位置离示教点 %.1f mm，偏差偏大，建议先回位再下探",
                               dev * 1000.0)
        else:
            start = [float(v) for v in base_pose]
            logger.warning("读不到当前末端位姿（%s），退回用示教点 %s 作起点 —— "
                           "这是次优选择，实际起点可能已经偏了", cur.get("message"), ref_point)

        z0 = start[2]
        # 安全闸门：不能探到桌面以下（桌面实测 0.72 m）
        table_z = float(self.params.get("table_z", tuning.TABLE_Z_M))
        clearance = float(self.params.get("min_clearance_m", 0.02))
        z_end = z0 - depth_cm / 100.0
        if z_end < table_z + clearance:
            return failed(
                f"下探终点 z={z_end:.3f} m 会低于桌面 {table_z:.2f} m + 安全余量 "
                f"{clearance*100:.0f} cm，拒绝执行。请抬高示教点 {ref_point} 或减小深度",
                code=Code.SAFETY_ABORT, z0=z0, z_end=z_end, table_z=table_z)

        ctx.flags["dive.start_pose"] = start
        ctx.flags["dive.z0"] = z0
        ctx.flags["dive.z_end_planned"] = z_end

        # ---------------- 只读预演：算完就返回 ----------------
        if dry:
            n = int(round(depth_cm / step_cm))
            return ok(
                f"[预演] 从 z={z0:.3f} 下探 {depth_cm:.0f} cm 到 z={z_end:.3f}"
                f"（分 {n} 段，每段 {step_cm:.0f} cm），终点距地面 {z_end - table_z:.3f} m，"
                f"未发任何运动指令",
                dry_run=True, z0=round(z0, 4), z_end=round(z_end, 4),
                segments=n, table_z=table_z, clearance_m=round(z_end - table_z, 4),
                ref_point=ref_point, motion_dispatched=False)

        # ---------------- 张开夹爪（下探前必须是张开状态） ----------------
        ret = io.set_gripper(open_m, side, tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "下探前张开夹爪失败")

        # ---------------- 分段下探 ----------------
        n_seg_planned = int(round(depth_cm / step_cm))
        n_seg = n_seg_planned if max_seg is None else min(n_seg_planned, int(max_seg))
        step_actual = depth_cm / n_seg_planned          # 保持总深度准，步数按计划
        log: List[Dict[str, Any]] = []
        reached = 0

        for i in range(1, n_seg + 1):
            z = z0 - step_actual * i / 100.0
            target = [start[0], start[1], z, start[3], start[4], start[5], start[6]]
            ret = self._step_to(io, target, f"下探第 {i}/{n_seg} 段")
            row = {"seg": i, "z": round(z, 4), "ok": bool(ret.get("success"))}
            if not ret.get("success"):
                row["message"] = ret.get("message")
                log.append(row)
                ctx.flags["dive.log"] = log
                # 已经进去了一段就不要再盲目动，直接报失败让上层决定
                prev_z = log[-2]["z"] if len(log) > 1 else z0
                return outcome(ret, f"下探第 {i} 段失败（当前约 z={prev_z:.3f} m）")
            log.append(row)
            reached = i
            if settle_s > 0:
                time.sleep(settle_s)      # 让机械臂抖动收敛，再走下一段

        ctx.flags["dive.log"] = log
        ctx.flags["dive.z_reached"] = z0 - step_actual * reached / 100.0
        logger.info("下探完成：%d 段，实际到达 z=%.3f m（计划 %.3f）",
                    reached, ctx.flags["dive.z_reached"], z_end)

        # ---------------- 闭爪 ----------------
        ret = io.set_gripper(close_m, side, tuning.GRIPPER_VELOCITY_MPS, tuning.GRIPPER_EFFORT_N)
        if not ret.get("success"):
            return outcome(ret, "闭爪失败，药盒未夹到")
        if hold_s > 0:
            time.sleep(hold_s)

        # ---------------- 判夹取：只有读得到宽度才算有确证 ----------------
        width = None
        gw = _safe_read(io, "get_gripper_width", side=side)
        if gw.get("success"):
            width = gw.get("width")
            ctx.flags["dive.gripper_width"] = width
            logger.info("闭爪后宽度 %.3f m", float(width))
        else:
            logger.warning("读不到夹爪宽度（%s），夹取与否无确证", gw.get("message"))

        # 宽度判据：夹到东西时张角 ≈ 物体厚度；空夹会一路走到 close_m 附近。
        # margin 用 +（严于指令值）：真机夹爪有柔性，空夹常停在 close_m + 1~2 mm，
        # margin 取负会把空夹误判成夹到 —— 这里偏保守。
        picked = None
        if width is not None:
            picked = float(width) > close_m + float(self.params.get("width_margin_m", 0.003))
        ctx.picked = picked

        # ---------------- 上探回升 ----------------
        ascended_to = None
        if ascend:
            for i in range(reached, 0, -1):
                z = z0 - step_actual * (i - 1) / 100.0
                target = [start[0], start[1], z, start[3], start[4], start[5], start[6]]
                ret = self._step_to(io, target, f"回升第 {reached - i + 1}/{reached} 段")
                if not ret.get("success"):
                    # 拿着药盒回升失败：这是 failed 不是 unknown —— 位姿可读，能归位
                    return outcome(ret, f"回升第 {i} 段失败（药盒可能还在夹爪里）")
                if settle_s > 0:
                    time.sleep(settle_s)
            ascended_to = z0

        data = {
            "z0": round(z0, 4),
            "z_reached": round(ctx.flags["dive.z_reached"], 4),
            "z_end_planned": round(z_end, 4),
            "segments": reached,
            "depth_cm_actual": round(depth_cm * reached / n_seg_planned, 2),
            "gripper_width": width,
            "picked": picked,
            "ascended_to": ascended_to,
            "expect_box_cm": expect_box,
            "log": log,
        }

        if picked is False:
            # 空夹：位置可能在竖直方向偏了，属于"知道失败了"，可以归位
            return failed(f"下探 {data['depth_cm_actual']:.0f} cm 后夹爪只合到 "
                          f"{float(width):.3f} m，没夹到东西（可能竖直偏了或夹爪撞壁）",
                          code=Code.NO_TARGET, **data)
        if picked is None:
            return ok(f"下探 {data['depth_cm_actual']:.0f} cm 完成并回升，但无夹爪宽度反馈，"
                      f"夹取与否不可判定", verified=False, **data)

        return ok(f"下探 {data['depth_cm_actual']:.0f} cm → 夹住（宽 {float(width):.3f} m）"
                  f"→ 已回升", verified=True, **data)
