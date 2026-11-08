"""抽屉场景 · 实验 2 专用 stage（2026-11-08 新增）。

实验 2 要回答的问题
------------------
实验 1 只验证了「示教数据能不能把空爪摆到 P0」。实验 2 接在 P0 之后，验证：
  ① 从 P0 **竖直下探进抽屉内腔**能不能夹到药盒（构型是否可行、夹爪有没有真的碰到东西）；
  ② 夹住之后**原路回撤**到 P0、再走 P1 → P2 → 抱持位，**带着盒子**能不能走通；
  ③ 全程视觉是否给了可用判断（没有就如实记"未确认"，绝不冒充通过）。

为什么新开一个文件而不是塞进 s11
------------------------------
s11 里已有 exp_arm_snapshot / exp_replay_chain / exp_verify_chain 三个通用件，实验 2 直接复用
它们；只有下面三件事是实验 2 独有的语义，单独成文件便于对照「实验 1 通过」与
「实验 2 通过」两套判据，也避免 s11 继续膨胀（它已经 900+ 行）。

三个新 stage
------------
  exp_gate_start    起始位硬门：必须已在指定示教点上，否则**零运动**拒绝
  exp_vision_check  抓取前视觉识别/质检：只拍照 + 只判定，不动手臂
  exp_grip          夹爪开/合/读：把「命令宽度」与「实际宽度」分开记账，回读才是判据

为什么不复用 grasp_first_box / grasp_vlm
---------------------------------------
那两个是**基线药品流程**的桌面抓取段（固定笛卡尔位姿 + set_arm_pose + move_line_to）。
实验 2 的抓取在抽屉内腔，位姿只能来自离线 IK 的关节序列（真机上同一末端位姿有多支 IK 解，
交给规划器会随机选分支、贴限位的直接 FAULT）；而且它必须「夹住 → 原路撤回」，
回撤路径已被实验 1 验证过。两套语义硬塞成一个 stage，两边都会说不清。
按项目既有约定：不改老文件，新建 stage 注册新名字，配置里换名字即可。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok, unknown
from ..registry import register_stage
from ..teach import _safe_read, load_point
from .s11_exp_stages import DEG, _snapshot

logger = logging.getLogger("scene.medicine.stage.exp2")


@register_stage("exp_gate_start")
class ExpGateStart(Stage):
    """起始位硬门：当前构型必须**就是**指定示教点，否则零运动拒绝。

    实验 2 的起点是实验 1 的终点 P0。手臂可能被人拖过、被队友遥操作占过
    （真机实测：08:19 在位 → 08:40 偏 74°），所以不能"假设它在 P0"——必须实测比对。
    不在点上就一条运动指令都不发，由人摆位后再跑。

    参数：
      at              必须是哪个示教点（如 "P0_measured"）
      file            示教文件
      chain           默认 right_arm
      tolerance_deg   允许的最大单关节偏差（默认 2.0°）
      gripper_side    仅用于记录夹爪开度
    """

    stage_id = "exp_gate_start"

    def describe(self) -> str:
        at = self.params.get("at", "?")
        tol = float(self.params.get("tolerance_deg", 2.0))
        return f"起始位硬门：必须已在「{at}」±{tol:.1f}° 内，否则零运动"

    def run(self, ctx: Context, io) -> StageResult:
        at = str(self.params.get("at", ""))
        path = str(self.params.get("file", tuning.TEACH_FILE))
        chain = str(self.params.get("chain", "right_arm"))
        tol = float(self.params.get("tolerance_deg", 2.0))
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))

        if not at:
            return failed("exp_gate_start 缺 at 参数：不知道应该在哪一点",
                          code=Code.STAGE_FAILED)

        try:
            rec = load_point(path, at)
        except Exception as exc:  # noqa: BLE001 - 示教文件坏了要如实报，不能吞
            return failed(f"读不到示教点 {at}：{exc}", code=Code.DEP_MISSING, at=at)
        tgt = [float(v) for v in (rec.get("joint_positions") or [])]
        if len(tgt) != 7:
            return failed(f"示教点 {at} 的关节角不完整（{len(tgt)} 个）—— 请重采",
                          code=Code.DEP_MISSING, at=at)

        snap = _snapshot(io, chain)
        snap["gripper_side"] = side
        if not snap["joints_ok"]:
            return unknown(
                f"读不到当前关节角（{snap.get('message', '无反馈')}），"
                f"无法确认手臂是否已在 {at} —— 一条运动指令都不发",
                code=Code.UNDETERMINED, at=at)
        got = snap["joint_positions"]
        if len(got) != len(tgt):
            return failed(f"当前反馈 {len(got)} 关节，示教点 {len(tgt)} 关节，不能按索引比对",
                          code=Code.NOT_VERIFIED, at=at)

        delta = [abs(a - b) * DEG for a, b in zip(got, tgt)]
        dmax = max(delta)
        i_max = delta.index(dmax)
        names = snap["joint_names"]
        joint_max = str(names[i_max]) if i_max < len(names) else f"j{i_max + 1}"
        info: Dict[str, Any] = {
            "at": at,
            "max_delta_deg": round(dmax, 3),
            "max_delta_joint": joint_max,
            "deltas_deg": {str(n): round(d, 2) for n, d in zip(names, delta)},
            "current_deg": [round(v * DEG, 2) for v in got],
            "target_deg": [round(v * DEG, 2) for v in tgt],
            "tolerance_deg": tol,
            "gripper_width_m": snap.get("gripper_width"),
            "motion_dispatched": False,
        }
        ctx.flags["exp.gate"] = info

        if dmax > tol:
            return failed(
                f"起始位不对：当前不在 {at}，最差是 {joint_max} 差 {dmax:.2f}°"
                f"（容差 {tol:.1f}°）。零运动退出。"
                f"先 scripts/home_to_carry.py --yes 回正，再跑实验 1 到达 P0 后才能跑本实验。",
                code=Code.SAFETY_ABORT, **info)
        return ok(f"起始位确认：已在 {at}，最大偏差 {dmax:.2f}°（容差 {tol:.1f}°）",
                  passed=True, **info)


@register_stage("exp_vision_check")
class ExpVisionCheck(Stage):
    """抓取前的视觉识别 / 质检。**只读**：拍一张、问一句、记结论，不动手臂。

    三种策略（配置 `policy`）：
      advisory（默认）—— 判定只记录不拦路：模型没配 / 拍不到 / 置信度低都照常往下走，
                        但结论里 `verified` 一律 False，绝不冒充"视觉确认通过"；
      gate            —— 判定不通过就停爪（fail closed）。最安全，但没配 Key 时一跑就卡在这；
      off             —— 不拍照不判定，只留一条"视觉未启用"的记录。

    为什么默认 advisory 而不是 gate：
      现场视觉后端还没定（VLM Key 未配、YOLO 权重未给）。默认 gate 会让实验 2 在第一步
      就停死，连"臂能不能下探夹住"这个纯运动学问题都验证不了。默认 advisory 能把两件事
      分开：运动链是否成立（本实验要答的）↔ 视觉是否可靠（下一轮再答）。

    参数：
      camera, tag     拍哪路相机、帧标签
      policy          advisory | gate | off
      backend         ""（按环境变量） | openai | static
      static_verdict  仅 mock 用：直接给一个假判定，用来回归 gate 分支
    """

    stage_id = "exp_vision_check"

    def describe(self) -> str:
        pol = str(self.params.get("policy", "advisory"))
        cam = str(self.params.get("camera", "head_left"))
        return f"视觉识别/质检（{cam}，策略={pol}）：只拍照不下爪"

    # ---------------------------------------------------------------- 内部
    def _backend(self):
        from ..perception import vlm_check  # 局部 import：不让感知层成为硬依赖

        raw = self.params.get("static_verdict")
        if isinstance(raw, dict) and raw:
            return vlm_check.StaticVLM(vlm_check.VLMVerdict(
                ok=bool(raw.get("ok", True)),
                message=str(raw.get("message", "static")),
                box_present=bool(raw.get("box_present", True)),
                graspable=bool(raw.get("graspable", True)),
                offset_x_cm=raw.get("offset_x_cm"),
                offset_y_cm=raw.get("offset_y_cm"),
                confidence=float(raw.get("confidence", 0.95)),
                reason=str(raw.get("reason", "static backend")),
            )), "static"
        return vlm_check.build_backend(str(self.params.get("backend", ""))), "vlm"

    def run(self, ctx: Context, io) -> StageResult:
        policy = str(self.params.get("policy", "advisory")).lower()
        camera = str(self.params.get("camera", "head_left"))
        tag = str(self.params.get("tag", "exp2_grasp"))

        if policy == "off":
            ctx.flags["vision"] = {"enabled": False, "policy": policy, "verified": False}
            return ok("视觉未启用（policy=off）：不拍照不判定，抓取按示教位姿执行",
                      enabled=False, verified=False)

        # ---------------- 1. 拍照（只读）----------------
        try:
            shot = io.capture_image(tag=tag, camera=camera)
        except Exception as exc:  # noqa: BLE001 - 相机炸了不许拖垮运动链
            shot = {"success": False, "message": f"capture_image 抛异常：{exc}"}
        img = {
            "success": bool(shot.get("success")),
            "path": shot.get("path"),
            "frame_id": shot.get("frame_id"),
            "camera": camera,
            "mock": bool(shot.get("mock")),
            "message": shot.get("message", ""),
        }
        if not img["success"]:
            ctx.flags["vision"] = {"enabled": True, "policy": policy, "image": img,
                                   "verified": False,
                                   "reason": f"拍照失败：{img['message']}"}
            if policy == "gate":
                return failed(f"视觉门：拍照失败（{img['message']}），按 gate 策略停爪",
                              code=Code.NOT_VERIFIED, image=img)
            logger.warning("拍照失败（%s），advisory 策略继续，视觉结论记为不可判",
                           img["message"])
            return ok("视觉质检：拍照失败，结论不可判（advisory 继续）",
                      enabled=True, verified=False, image=img)

        # ---------------- 2. 判定 ----------------
        try:
            backend, source = self._backend()
            from ..perception import vlm_check
            prompt = str(self.params.get("prompt") or vlm_check.DEFAULT_PROMPT)
            verdict = backend.judge(str(img["path"]), prompt)
            vd = verdict.to_dict()
            vd["source"] = source
        except Exception as exc:  # noqa: BLE001 - 视觉后端任何异常都不许拖垮运动链
            vd = {"ok": False, "message": f"视觉后端异常：{exc}", "box_present": False,
                  "graspable": False, "confidence": 0.0, "reason": "", "source": "error"}
            logger.error("视觉后端异常：%s", exc)

        # 只有「模型给出可用判定 + 说有盒 + 说能夹 + 置信度够」才算视觉确认通过。
        # 缺任何一项都记 verified=False —— 不允许把"说不清"写成"通过"。
        confirmed = bool(vd.get("ok") and vd.get("box_present")
                         and vd.get("graspable")
                         and float(vd.get("confidence") or 0.0) >= 0.6)
        if vd.get("source") == "static":
            # 静态桩是测试替身，永远不算真视觉确认
            confirmed = False
            vd["note"] = "static 桩：仅用于回归 gate 分支，不构成视觉确认"

        ctx.flags["vision"] = {"enabled": True, "policy": policy, "image": img,
                               "verdict": vd, "verified": confirmed,
                               "reason": vd.get("message", "")}

        head = (f"视觉质检：box_present={vd.get('box_present')} "
                f"graspable={vd.get('graspable')} "
                f"conf={float(vd.get('confidence') or 0.0):.2f}（{vd.get('source')}）")
        # ⚠️ 不能把 vd 直接展开成关键字参数：vd 里有 "message" 键，会与 ok()/failed()
        # 的第一个形参同名而抛 TypeError（2026-11-08 mock 首次跑就踩了）。整包塞进 verdict。
        tail = {"enabled": True, "verified": confirmed, "verdict": vd,
                "image": img}
        if confirmed:
            return ok(head + " → 视觉确认通过", **tail)
        if policy == "gate":
            return failed(head + f" → gate 策略停爪：{vd.get('message', '')}",
                          code=Code.NO_TARGET, **tail)
        logger.warning("%s → advisory 继续，但视觉结论记为未确认", head)
        return ok(head + " → 未确认（advisory 继续，抓取按示教位姿执行）", **tail)


@register_stage("exp_grip")
class ExpGrip(Stage):
    """夹爪开/合/读：把「命令宽度」和「实际宽度」分开记账。

    为什么必须有这个 stage（而不是在别处顺手 set_gripper）：
      `set_gripper(w)` 在真机上只是"往 w 合"，合到一半被物体顶住就停在**物体厚度**上。
      于是「命令 2 cm、回读 7.5 cm」恰恰是**夹到了东西**的证据；而「命令 7.5 cm、
      回读 7.5 cm」在"空爪合到 7.5"和"夹住 7.5 cm 的盒子"之间**无法区分**。
      这就是实验 2 合爪指令取 0.02 m 而不是 0.075 m 的原因：让回读宽度成为判据。
      （真机已实测过同一行为：命令 0.02 / effort 10 N，实际停在 24.3 mm。）

      用户要的**可观测结果**仍然是「咬合维持 7.5 cm」——那是回读值，由盒子自身宽度决定，
      所以落在 expect_min_m/expect_max_m 区间里才算通过。

    参数：
      action          open | close | set | check
      width_m         set/close 的指令宽度（open 默认 tuning.GRIPPER_OPEN_M）
      side, velocity_mps, effort
      expect_min_m / expect_max_m   期望回读区间（超出判 NOT_VERIFIED）
      hard_floor_m    硬下限：回读低于它判「夹空」(NO_TARGET)
      label           证据标签，写进 ctx.flags[f"grip.{label}"]
    """

    stage_id = "exp_grip"

    def describe(self) -> str:
        act = str(self.params.get("action", "check"))
        w = self.params.get("width_m")
        tail = f" → {float(w) * 100:.1f} cm" if w is not None else ""
        return f"夹爪 {act}{tail}（命令与实际分开记录）"

    def run(self, ctx: Context, io) -> StageResult:
        act = str(self.params.get("action", "check")).lower()
        side = str(self.params.get("side", tuning.GRIPPER_SIDE))
        vel = float(self.params.get("velocity_mps", tuning.GRIPPER_VELOCITY_MPS))
        effort = float(self.params.get("effort", tuning.GRIPPER_EFFORT_N))
        label = str(self.params.get("label", act))

        cmd: Optional[float] = None
        if act == "open":
            cmd = float(self.params.get("width_m", tuning.GRIPPER_OPEN_M))
        elif act in ("close", "set"):
            if self.params.get("width_m") is None:
                return failed(f"exp_grip action={act} 缺 width_m", code=Code.STAGE_FAILED)
            cmd = float(self.params["width_m"])
        elif act != "check":
            return failed(f"未知 action={act!r}（只支持 open|close|set|check）",
                          code=Code.STAGE_FAILED)

        # ---------------- 1. 下发（check 不下发）----------------
        if cmd is not None:
            try:
                ret = io.set_gripper(cmd, side, vel, effort)
            except Exception as exc:  # noqa: BLE001
                return failed(f"夹爪 {act} 调用抛异常：{exc}", code=Code.STAGE_FAILED)
            if not ret.get("success"):
                return outcome(ret, f"夹爪 {act} 到 {cmd * 100:.1f} cm 失败",
                               sub_step=f"grip_{act}")

        # ---------------- 2. 回读（唯一客观证据）----------------
        read = _safe_read(io, "get_gripper_width", side=side)
        width = read.get("width") if read.get("success") else None
        rec: Dict[str, Any] = {
            "action": act, "side": side, "commanded_m": cmd,
            "width_m": (round(float(width), 4) if width is not None else None),
            "read_ok": width is not None,
            "expect_min_m": self.params.get("expect_min_m"),
            "expect_max_m": self.params.get("expect_max_m"),
            "hard_floor_m": self.params.get("hard_floor_m"),
        }
        ctx.flags[f"grip.{label}"] = rec

        if width is None:
            return unknown(
                f"夹爪 {act} 已下发，但读不到夹爪宽度"
                f"（{read.get('message', '无反馈')}）—— 夹没夹到不可判定，不许当成功",
                code=Code.UNDETERMINED, **rec)

        width = float(width)
        floor = self.params.get("hard_floor_m")
        emin = self.params.get("expect_min_m")
        emax = self.params.get("expect_max_m")
        head = (f"夹爪 {act}：命令 {cmd * 100:.2f} cm，实际 {width * 100:.2f} cm"
                if cmd is not None else f"夹爪读数：{width * 100:.2f} cm")

        if floor is not None and width < float(floor):
            ctx.picked = False
            return failed(head + f" → 低于硬下限 {float(floor) * 100:.1f} cm，判定夹空"
                                 f"（药盒不在爪心）",
                          code=Code.NO_TARGET, picked=False, **rec)
        if emin is not None and width < float(emin):
            ctx.picked = False
            return failed(head + f" → 低于期望下限 {float(emin) * 100:.1f} cm",
                          code=Code.NOT_VERIFIED, picked=False, **rec)
        if emax is not None and width > float(emax):
            ctx.picked = False
            return failed(head + f" → 高于期望上限 {float(emax) * 100:.1f} cm"
                                 f"（没夹住或爪没合到位）",
                          code=Code.NOT_VERIFIED, picked=False, **rec)

        if act in ("close", "set"):
            ctx.picked = True
        return ok(head, picked=ctx.picked, **rec)
