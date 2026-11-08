"""抽屉场景 · 分层实验专用 stage（2026-11-08 新增）。

三层实验设计（YOLO 未介入前能验证到的最深程度）：

  实验 1「示教链回放」  —— 只验证**示教数据本身能不能把手臂摆到指定构型**。
                           链条（方向已于 2026-11-08 纠正）= CARRY_home → P2 → P1 → P0，
                           即真实作业里「拉完抽屉后回收」那条路的正向复现，跑到最高点 P0 即停。
                           动作全部是关节角回放（关节空间，箱外自由空间），
                           **不含任何接触、不夹任何东西、全程悬空**。
                           通过条件 = 每点回放后关节偏差 < 容差 + 链条完整跑完。
                           ⚠️ 起点不是配置里写死的，而是**运行时读到的当前关节角**：
                           必须先跑 scripts/home_to_carry.py --yes 回正，否则第一段就可能超跨度。
                           ⚠️ 旧的 CARRY_home → P0 → P1 → P2 方向已废弃：CARRY(−108.9°) 一步到
                           P0(+69.3°) 是单段 j1 178.2°，末端直线 y 几乎不变，t≈0.3 即进入抽屉
                           占位区，会刮到桌上的抽屉（真机 09:04 于旧链第 3 点返回 ControlStatus.FAULT）。

  实验 2「抽屉开合」    —— 接在实验 1 之后，验证**接触类动作**：
                           对位把手 → 闭爪 → 沿 -x 直线后拉 → 张爪退出。
                           核心未知量是「10 N 夹持力能否拉开滑轨静摩擦」。

  实验 3「箱内 + 投放」  —— 接在实验 2 之后，验证剩下的两段：
                           竖直下探夹药盒 / 回升 / 投放 / 归位。

为什么实验 1 单独成层（而不是混进实验 2）：
  拉抽屉失败有三类原因 —— ① 臂没站对（关节/构型问题）② 站对了但夹不住
  ③ 夹住了但拉不动。混在一起跑，失败了分不清是哪一类。
  实验 1 先把 ① 排除掉，实验 2 只需面对 ②③。

实验 1 收尾为什么停在 P0、而不是回抱持位：
  用户 2026-11-08 09:52 明确「完成一个周期停止在 P0 退出实验」。
  P0 是视觉下探前的准备位（最高点），停在这里正好接上后续感知/下探段；
  把「回抱持位」拆成独立的 scripts/home_to_carry.py，不再混在本链里 ——
  这样回正失败与示教链失败可以分开判定，不会互相掩盖。

本文件新增三个 stage，都不改 runner / contracts / io 的契约：
  exp_arm_snapshot   只读：记录当前关节角、末端位姿、离限位余量（每步之间的"证据")
  exp_replay_chain   关节回放一串示教点，逐点复核偏差，并把证据汇总进 ctx.flags
  exp_verify_chain   只读断言：检查上一步/整条链留下的证据是否满足通过条件
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Dict, List, Optional

from .. import outcome, tuning
from ..contracts import Code, Context, Stage, StageResult, failed, ok, unknown
from ..registry import register_stage
from ..teach import _safe_read, load_point, load_teach, resolve_teach_path

logger = logging.getLogger("scene.medicine.stage.exp")

DEG = 57.29577951308232

#: 右臂 7 关节限位（度）。与 URDF 一致，见 _urdf/drawer_fk.py / MEMORY §7.3。
ARM_JOINT_LIMITS_DEG: Dict[str, tuple] = {
    "right_arm_joint1": (-172.1, 172.1),
    "right_arm_joint2": (-92.1, 92.1),
    "right_arm_joint3": (-167.1, 167.1),
    "right_arm_joint4": (17.2, 147.1),     # ⚠️ 不能伸直
    "right_arm_joint5": (-167.1, 167.1),
    "right_arm_joint6": (-42.1, 47.1),     # ⚠️ 幅宽只有 89.2°
    "right_arm_joint7": (-88.1, 88.1),
}

#: 余量低于这个值就报警（度）。j7@P0 = 4.5°、j4@ALIGN = 7.9° 都在这条线附近。
LIMIT_WARN_DEG = 5.0
#: 余量低于这个值判 failed（度）。不允许在贴限位的姿态上发运动指令。
LIMIT_FAIL_DEG = 2.0

#: 单段关节跨度超过这个值就判「危险」——必须拆中间点或改走笛卡尔。
#: 依据：队友实测的八段拉屉每段只敢走 3~5 cm，关节增量都在 25° 以内；
#: 实验 3 的 `probe_stability.py` 也证明大跨度下发后规划器会随机选 IK 分支。
SPAN_DANGER_DEG = 90.0
#: 跨度超过这个值只警告（60~90 之间属于「能走但要盯着」）。
SPAN_WARN_DEG = 45.0


def _span_risk(max_delta_deg: float, sum_delta_deg: float) -> str:
    """把一段跨度翻译成风险等级。返回 ok / warn / danger。"""
    if max_delta_deg >= SPAN_DANGER_DEG or sum_delta_deg >= 200.0:
        return "danger"
    if max_delta_deg >= SPAN_WARN_DEG or sum_delta_deg >= 120.0:
        return "warn"
    return "ok"


def _margin_deg(name: str, rad: float) -> Optional[float]:
    """某关节离最近限位的余量（度）。未知关节返回 None。"""
    lim = ARM_JOINT_LIMITS_DEG.get(name)
    if lim is None:
        return None
    d = rad * DEG
    return min(d - lim[0], lim[1] - d)


def _snapshot(io, chain: str) -> Dict[str, Any]:
    """读一次三件套（只读）。任一失败只标记，不抛。"""
    rec: Dict[str, Any] = {}
    jt = _safe_read(io, "get_joint_positions", joint_groups=[chain], joint_names=[])
    rec["joints_ok"] = bool(jt.get("success"))
    names = jt.get("joint_names") or []
    pos = jt.get("joint_positions") or []
    rec["joint_names"] = list(names)
    rec["joint_positions"] = [float(v) for v in pos] if rec["joints_ok"] else []
    if rec["joints_ok"] and names and len(names) == len(pos):
        margins = {}
        for n, p in zip(names, pos):
            m = _margin_deg(str(n), float(p))
            if m is not None:
                margins[str(n)] = round(m, 2)
        rec["limit_margin_deg"] = margins
        if margins:
            tight = min(margins, key=lambda k: margins[k])
            rec["tightest_joint"] = tight
            rec["tightest_margin_deg"] = margins[tight]

    ee = _safe_read(io, "get_ee_pose", chain=chain)
    rec["ee_ok"] = bool(ee.get("success")) and len(ee.get("pose") or []) == 7
    rec["ee_pose"] = [float(v) for v in (ee.get("pose") or [])] if rec["ee_ok"] else None

    gw = _safe_read(io, "get_gripper_width", side=str(rec.get("gripper_side", "right")))
    rec["gripper_ok"] = bool(gw.get("success"))
    rec["gripper_width"] = gw.get("width") if rec["gripper_ok"] else None
    return rec


@register_stage("exp_arm_snapshot")
class ExpArmSnapshot(Stage):
    """只读：记录当前手臂状态（关节角 + 末端位姿 + 限位余量）。

    一条运动指令都不发。用于在每个回放点之后留一份"证据"，
    失败了能回答"当时手臂到底在哪、哪个关节快贴限位了"。

    参数：
      chain       默认 right_arm
      label       证据标签（默认用 step id 语义，配置里显式给）
      fail_on_tight  余量 < LIMIT_FAIL_DEG 时是否判 failed（默认 True）
    """

    stage_id = "exp_snapshot"

    def describe(self) -> str:
        label = self.params.get("label", "?")
        return f"只读快照「{label}」：关节角 + 末端位姿 + 限位余量"

    def run(self, ctx: Context, io) -> StageResult:
        chain = str(self.params.get("chain", "right_arm"))
        label = str(self.params.get("label", self.stage_id))
        fail_on_tight = bool(self.params.get("fail_on_tight", True))

        snap = _snapshot(io, chain)
        snap["label"] = label
        snap["gripper_side"] = str(self.params.get("gripper_side", "right"))
        ctx.flags.setdefault("exp.snapshots", {})[label] = snap

        if not snap["joints_ok"]:
            return failed(f"快照「{label}」：读不到关节角，无法确证手臂状态",
                          code=Code.NOT_VERIFIED, label=label)

        tight_m = snap.get("tightest_margin_deg")
        msg = f"快照「{label}」：{len(snap['joint_positions'])} 关节已读"
        if snap["ee_ok"]:
            p = snap["ee_pose"]
            msg += f"，末端 ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"
        if tight_m is not None:
            msg += f"，最紧 {snap['tightest_joint']} 余量 {tight_m:.1f}°"
            if tight_m < LIMIT_FAIL_DEG and fail_on_tight:
                return failed(
                    f"快照「{label}」：{snap['tightest_joint']} 距限位只剩 {tight_m:.1f}°，"
                    f"不许在这个姿态上再发运动指令",
                    code=Code.SAFETY_ABORT, label=label,
                    tightest_joint=snap["tightest_joint"], margin_deg=tight_m)
            if tight_m < LIMIT_WARN_DEG:
                logger.warning("快照「%s」：%s 距限位只剩 %.1f°（低于 %.1f° 告警线）",
                               label, snap["tightest_joint"], tight_m, LIMIT_WARN_DEG)

        return ok(msg, label=label, ee_ok=snap["ee_ok"],
                  tightest_joint=snap.get("tightest_joint"),
                  tightest_margin_deg=tight_m)


@register_stage("exp_replay_chain")
class ExpReplayChain(Stage):
    """关节回放一串示教点，逐点复核，把证据汇总进 ctx.flags。

    与已有的 `teach_replay_joints`（单点）的区别：
      · 一次回放多个点，中间可以插 settle 与逐点复核；
      · 逐点记录「回放前偏差 / 回放后偏差 / 限位余量 / 实际末端位姿」，
        形成一条可核对的链；
      · 支持 `dry_run_no_motion=True`：只读当前状态并统计"各段要动多少度"，
        一条指令都不发 —— 这是**离线预演**，用来回答"这条链会不会一次跨太大"。

    ⚠️ 为什么用关节回放而不是笛卡尔：
      示教记录的是"人摆出来的姿态"，本质就是一串关节角。按关节回放复现性最好，
      不会因为同一末端位姿的多种关节解（肘上/肘下）跑到另一个构型去。
      代价：关节插值不保证末端走直线 → 只用于**箱外自由空间**。

    参数：
      file            示教文件
      points          要依次回放的点名列表
      speed_rad_s     关节速度上限
      tolerance_deg   每点回放后允许的最大关节偏差（度），默认 3.0
      settle_s        每点到位后等待
      dry_run_no_motion  只读预演（默认 False）
      stop_on_fail    任一点失败即停（默认 True；False 则继续跑完并全记下来）
    """

    stage_id = "exp_replay_chain"

    def describe(self) -> str:
        pts = self.params.get("points") or []
        dry = bool(self.params.get("dry_run_no_motion", False))
        head = "只读预演" if dry else "关节回放"
        return f"{head} {len(pts)} 个示教点：{' → '.join(map(str, pts)) or '(空)'}"

    # ------------------------------------------------------------ 主流程
    def run(self, ctx: Context, io) -> StageResult:
        chain = str(self.params.get("chain", "right_arm"))
        path = str(self.params.get("file", tuning.TEACH_FILE))
        points: List[str] = [str(x) for x in (self.params.get("points") or [])]
        speed = float(self.params.get("speed_rad_s", tuning.ARM_SPEED_RAD_S))
        tol_deg = float(self.params.get("tolerance_deg", 3.0))
        settle_s = float(self.params.get("settle_s", 0.5))
        dry = bool(self.params.get("dry_run_no_motion", False))
        stop_on_fail = bool(self.params.get("stop_on_fail", True))
        side = str(self.params.get("gripper_side", tuning.GRIPPER_SIDE))

        if not points:
            return failed("points 为空 —— 不知道要回放什么", code=Code.STAGE_FAILED)

        # ---------------- require_mock：真机禁止执行的分支 ----------------
        # 有些步在 mock 里跑得通、也是链路完整性的必要一环，但在真机上会把大跨度
        # 关节指令真发出去。典型：旧实验 1 的 leg2_jump_to_p2（P1→P2 跨度 170°）——
        # set_joints 走 2.97 rad / 0.15 rad·s⁻¹ = 19.8 s > timeout 15 s，
        # 而**超时不会让机器人停下来**。这类步一律打 require_mock，真机直接拒绝。
        if bool(self.params.get("require_mock", False)):
            backend = type(io).__name__
            if backend != "MockIO":
                return failed(
                    f"拒绝执行 {' → '.join(points)}：这一步标了 require_mock"
                    f"（它会下发超大跨度关节指令，真机上 15 s 超时后机器人不会停），"
                    f"当前 IO = {backend}。真机请改用 exp1_real_carry_p2_p1_p0.json。",
                    code=Code.SAFETY_ABORT, backend=backend, points=points)

        real = resolve_teach_path(path)
        teach = load_teach(path)
        if not teach:
            return failed(f"示教文件读不到或为空：{real}", code=Code.DEP_MISSING, file=real)

        # ---------------- 预检：所有点必须齐全，缺一个就不开跑 ----------------
        missing = [p for p in points if not (teach.get("points") or {}).get(p)]
        if missing:
            have = sorted((teach.get("points") or {}).keys())
            return failed(f"示教文件缺点位 {missing}（已有 {have}）",
                          code=Code.NO_TARGET, file=real, missing=missing, have=have)
        bad_joints = [p for p in points
                      if not (load_point(path, p).get("joint_names")
                              and load_point(path, p).get("joint_positions"))]
        if bad_joints:
            return failed(f"点位 {bad_joints} 的关节角不完整，不能回放 —— 请重采",
                          code=Code.DEP_MISSING, file=real, bad=bad_joints)

        names_ref = load_point(path, points[0]).get("joint_names")

        # ---------------- 起点 ----------------
        start = _snapshot(io, chain)
        start["gripper_side"] = side
        if not start["joints_ok"]:
            return failed(f"读不到当前关节角（{start.get('message', '无反馈')}），"
                          f"不知道手臂在哪就不能回放", code=Code.UNDETERMINED)

        trace: List[Dict[str, Any]] = []
        prev_pos = list(start["joint_positions"])
        # 起点标签：手臂实际从「运行时读到的当前关节角」出发，配置里的 start_point 只是
        # **名义**上一点（回正到位时应与该点吻合）。两者必须分开记 —— 否则日志里会出现
        # "START → VIA_P2_P1_1" 这种与实际不符的段（2026-11-08 实验 1 真机就踩了）。
        declared_start = str(self.params.get("start_point") or "") or None
        prev_name = declared_start or "RUNTIME_START"
        chain_t0 = time.time()
        chain_started_at = time.strftime("%Y-%m-%dT%H:%M:%S")

        if dry:
            # ---------------- 只读预演：只算跨度，不发指令 ----------------
            for pt in points:
                rec = load_point(path, pt)
                tgt = [float(v) for v in rec["joint_positions"]]
                if len(tgt) != len(prev_pos):
                    return failed(f"{pt} 的关节数 {len(tgt)} 与当前 {len(prev_pos)} 不符",
                                  code=Code.STAGE_FAILED, point=pt)
                delta = [abs(a - b) * DEG for a, b in zip(tgt, prev_pos)]
                margins = {}
                for n, v in zip(rec["joint_names"], tgt):
                    m = _margin_deg(str(n), v)
                    if m is not None:
                        margins[str(n)] = round(m, 2)
                tight = min(margins, key=lambda k: margins[k]) if margins else None
                row = {
                    "from": prev_name, "to": pt,
                    "max_delta_deg": round(max(delta), 2),
                    "max_delta_joint": str(rec["joint_names"][delta.index(max(delta))]),
                    "sum_delta_deg": round(sum(delta), 2),
                    "risk": _span_risk(max(delta), sum(delta)),
                    "target_limit_margin_deg": margins,
                    "target_tightest": tight,
                    "target_tightest_deg": margins.get(tight) if tight else None,
                    "motion_dispatched": False,
                }
                trace.append(row)
                lvl = logging.WARNING if row["risk"] != "ok" else logging.INFO
                logger.log(lvl, "[预演] %s → %s：最大 %s %.1f°（合计 %.1f°）风险=%s，%s 余量 %.1f°",
                           prev_name, pt, row["max_delta_joint"], row["max_delta_deg"],
                           row["sum_delta_deg"], row["risk"], tight,
                           row["target_tightest_deg"] if row["target_tightest_deg"] is not None else -1)
                prev_pos = tgt
                prev_name = pt

            ctx.flags["exp.dry_run"] = trace
            worst = max(trace, key=lambda r: r["max_delta_deg"])
            return ok(
                f"只读预演完成：{len(points)} 段，最大单段跨度 "
                f"{worst['max_delta_deg']:.1f}°（{worst['to']} 的 {worst['max_delta_joint']}），"
                f"未发任何运动指令",
                dry_run=True, segments=len(trace), worst=worst, trace=trace)
        # ---------------- 真回放 ----------------
        done: List[str] = []
        for i, pt in enumerate(points):
            rec = load_point(path, pt)
            names = list(rec["joint_names"])
            tgt = [float(v) for v in rec["joint_positions"]]
            if names != names_ref:
                return failed(f"{pt} 的关节名顺序与 {points[0]} 不一致，不能按索引比对",
                              code=Code.STAGE_FAILED, point=pt,
                              expect=names_ref, got=names)

            pre_delta = [abs(a - b) * DEG for a, b in zip(tgt, prev_pos)]
            risk = _span_risk(max(pre_delta), sum(pre_delta))
            if risk == "danger":
                # 单段跨度 ≥90° 或合计 ≥200°：真机上规划器极可能选到另一支 IK 解
                # （FAILURES.md §1 实测：TCP 差 0.036 mm、joint3 差 18° 就被拒）。
                # 这里**不发指令**，如实报出来让人拆中间点。
                if bool(self.params.get("block_danger_span", True)):
                    ctx.flags["exp.chain"] = trace
                    names_bad = names[pre_delta.index(max(pre_delta))]
                    # 说清楚是哪个门槛被踩了 —— 只写 90° 会在"合计超 200°"时误导人
                    hit = []
                    if max(pre_delta) >= SPAN_DANGER_DEG:
                        hit.append(f"单关节 {names_bad} 要转 {max(pre_delta):.1f}°"
                                   f" ≥ {SPAN_DANGER_DEG:.0f}°")
                    if sum(pre_delta) >= 200.0:
                        hit.append(f"7 关节合计要转 {sum(pre_delta):.1f}°"
                                   f" ≥ 200°（多关节同时大角度）")
                    return failed(
                        f"拒绝回放 {prev_name} → {pt}：{'；'.join(hit)}。"
                        f"真机上规划器大概率选到另一支 IK 解然后 FAULT。"
                        f"请在中间补一个示教点（每段压到 ≤40°），"
                        f"或改走 move_line 笛卡尔直线。"
                        f"（要强行跑就设 block_danger_span=false）",
                        code=Code.SAFETY_ABORT, seg_from=prev_name, seg_to=pt,
                        max_delta_deg=round(max(pre_delta), 2), joint=names_bad,
                        sum_delta_deg=round(sum(pre_delta), 2),
                        done=done, trace=trace)
                logger.error("跨度危险但 block_danger_span=false，照发：%s → %s %.1f°",
                             prev_name, pt, max(pre_delta))
            elif risk == "warn":
                logger.warning("跨度偏大：%s → %s 最大 %.1f°（合计 %.1f°），真机上要盯着",
                               prev_name, pt, max(pre_delta), sum(pre_delta))

            t_dispatch = time.time()
            ret = io.set_joints(tgt, names, speed)
            if not ret.get("success"):
                res = outcome(ret, f"回放到 {pt} 失败")
                res.data.update(point=pt, done=done, trace=trace)
                ctx.flags["exp.chain"] = trace
                return res
            if settle_s > 0:
                time.sleep(settle_s)

            after = _snapshot(io, chain)
            after["gripper_side"] = side
            if not after["joints_ok"]:
                row = {"to": pt, "verified": False, "reason": "回放后读不到关节角"}
                trace.append(row)
                ctx.flags["exp.chain"] = trace
                return unknown(f"回放到 {pt} 后读不到关节角，无法确证到位",
                               code=Code.UNDETERMINED, point=pt, done=done)

            got = after["joint_positions"]
            if len(got) != len(tgt):
                row = {"to": pt, "verified": False, "reason": f"维度不符 {len(got)}≠{len(tgt)}"}
                trace.append(row)
                ctx.flags["exp.chain"] = trace
                return failed(f"回放到 {pt} 后反馈维度 {len(got)} 与目标 {len(tgt)} 不符",
                              code=Code.NOT_VERIFIED, point=pt, done=done)

            err = [abs(a - b) * DEG for a, b in zip(got, tgt)]
            err_max = max(err)
            margins = {}
            for n, v in zip(names, got):
                m = _margin_deg(str(n), v)
                if m is not None:
                    margins[str(n)] = round(m, 2)
            tight = min(margins, key=lambda k: margins[k]) if margins else None

            row = {
                "from": prev_name, "to": pt,
                # 第 0 行（i=0）的 from 是配置声明的名义起点，不是实测上一点；
                # 之后的 from 都是上一条已回读校验过的点。
                "from_measured": i > 0,
                "pre_delta_deg": round(max(pre_delta), 2),
                "pre_sum_deg": round(sum(pre_delta), 2),
                "span_risk": risk,
                "err_deg": round(err_max, 3),
                "err_joint": names[err.index(err_max)],
                "tolerance_deg": tol_deg,
                "verified": err_max <= tol_deg,
                "limit_margin_deg": margins,
                "tightest_joint": tight,
                "tightest_margin_deg": margins.get(tight) if tight else None,
                "ee_pose": after["ee_pose"],
                "motion_dispatched": True,
                # 从下发 set_joints 到回读快照完成（含 settle）的实测墙钟时间
                "elapsed_s": round(time.time() - t_dispatch, 3),
                "settle_s": settle_s,
            }
            trace.append(row)
            logger.info("回放 %s：偏差 %.2f°（容差 %.1f°），%s 余量 %.1f°",
                        pt, err_max, tol_deg, tight, row["tightest_margin_deg"] or -1)

            # 贴限位判 failed：不允许在余量 < 2° 的姿态上继续往下走
            if row["tightest_margin_deg"] is not None and row["tightest_margin_deg"] < LIMIT_FAIL_DEG:
                ctx.flags["exp.chain"] = trace
                return failed(
                    f"回放到 {pt} 后 {tight} 距限位只剩 {row['tightest_margin_deg']:.1f}°，"
                    f"继续发指令可能撞限位",
                    code=Code.SAFETY_ABORT, point=pt, done=done, trace=trace)

            if err_max > tol_deg:
                ctx.flags["exp.chain"] = trace
                res = failed(
                    f"回放到 {pt} 后 {row['err_joint']} 偏差 {err_max:.1f}° 超过容差 {tol_deg:.1f}°"
                    f"（可能被限位/碰撞检查拦下，或这一步本身就是不可达构型）",
                    code=Code.NOT_VERIFIED, point=pt, done=done, trace=trace)
                if stop_on_fail:
                    return res
                logger.error("回放 %s 偏差超容差，stop_on_fail=False，继续", pt)

            done.append(pt)
            prev_pos = got
            prev_name = pt

        ctx.flags["exp.chain"] = trace
        ctx.flags["exp.chain_done"] = done
        # 累积版：一个实验里可能有多条独立的 exp_replay_chain（分段），
        # 后一条不该覆盖前一条的证据，否则 verify 只看得到最后一段。
        ctx.flags.setdefault("exp.chain_all", []).extend(trace)
        ctx.flags.setdefault("exp.chain_done_all", []).extend(done)
        verified = sum(1 for r in trace if r.get("verified"))
        total_s = round(time.time() - chain_t0, 3)
        motion_s = round(sum(float(r.get("elapsed_s") or 0.0) for r in trace), 3)
        logger.info("链路耗时：总 %.2f s（其中下发+回读 %d 段共 %.2f s，其余为日志/规划开销）",
                    total_s, len(trace), motion_s)
        return ok(f"回放完成：{verified}/{len(points)} 点通过（容差 {tol_deg:.1f}°，{total_s:.1f} s）",
                  points=points, done=done, verified=verified,
                  total=len(points), trace=trace,
                  total_s=total_s, motion_s=motion_s, started_at=chain_started_at)


@register_stage("exp_verify_chain")
class ExpVerifyChain(Stage):
    """只读断言：检查前面留下的一条/多条证据是否满足实验的通过条件。

    这条 stage 存在的意义：**把"实验通过"的定义写成代码**，而不是让人看日志。
    抓抽屉失败的三类原因（臂没站对 / 夹不住 / 拉不动）必须能靠机器判出来，
    否则每次上真机都是在赌。

    参数：
      require_points    必须出现在 exp.chain_done 里的点（一个都不能少）
      max_err_deg       所有已回放点的最大允许偏差
      min_limit_margin_deg  所有已回放点的最小限位余量要求（默认同 LIMIT_WARN_DEG）
      require_keys      ctx.flags 里必须存在的键（如 "pull.pulled_m")
      require_pull      实验 2 用：必须有一条成功的拉抽屉证据（pull.log + pull.pulled_m）
      min_pulled_cm     实验 2 用：累计后拉至少多少 cm 才算通过（默认 28.0）
      require_dive      实验 3 用：必须有一条成功的下探证据（dive.log + dive.z_reached）
      min_dive_cm       实验 3 用：实际下探深度至少多少 cm 才算通过（默认 26.0）
      require_picked    实验 3 用：必须确证夹到了东西（ctx.picked is True）
      require_drop      实验 3 用：必须有一条投放证据（drop.done）
    """

    stage_id = "exp_verify"

    def describe(self) -> str:
        bits = []
        pts = self.params.get("require_points") or []
        if pts:
            bits.append(f"{len(pts)} 个点全部回放成功")
        if self.params.get("require_pull"):
            bits.append(f"后拉 ≥ {self.params.get('min_pulled_cm', 28.0):.0f} cm")
        if self.params.get("require_dive"):
            bits.append(f"下探 ≥ {self.params.get('min_dive_cm', 26.0):.0f} cm")
        if self.params.get("require_picked"):
            bits.append("夹爪确证夹到")
        if not bits:
            bits.append("证据完整性")
        return "只读断言：" + "、".join(bits)

    # ------------------------------------------------------------ 接触类证据
    def _check_pull(self, ctx: Context) -> Optional[StageResult]:
        """实验 2 的判据：抽屉到底拉出来多少、拉的是不是直线。"""
        log = list(ctx.flags.get("pull.log") or [])
        pulled_m = ctx.flags.get("pull.pulled_m")
        bite = ctx.flags.get("pull.bite_pose")
        end = ctx.flags.get("pull.end_pose")
        min_cm = float(self.params.get("min_pulled_cm", 28.0))
        eff_cm = float(self.params.get("min_pull_efficiency", 0.90))

        if not log:
            return failed("实验未通过：没有后拉证据（pull.log 为空）—— "
                          "drawer_pull 没跑过，或第一步就失败了",
                          code=Code.NOT_VERIFIED)
        bad = [r for r in log if not r.get("ok")]
        if bad:
            return failed(f"实验未通过：后拉有 {len(bad)} 段失败（第 "
                          f"{[r['seg'] for r in bad]} 段）",
                          code=Code.NOT_VERIFIED, failed_segments=[r["seg"] for r in bad])
        if pulled_m is None:
            return failed("实验未通过：pull.pulled_m 缺失 —— 无法确证实际拉出多少",
                          code=Code.NOT_VERIFIED)
        pulled_cm = float(pulled_m) * 100.0
        if pulled_cm < min_cm:
            return failed(f"实验未通过：只拉到 {pulled_cm:.1f} cm，要求 ≥ {min_cm:.0f} cm。"
                          f"大概率是 10 N 夹持力不足或抽屉卡在行程末端",
                          code=Code.NOT_VERIFIED, pulled_cm=round(pulled_cm, 2))

        # 直线性：末端实际位移必须与指令位移方向一致（都是 -x），且横/竖向漂移小
        drift_msg = ""
        if bite and end:
            dx = float(end[0]) - float(bite[0])
            dy = abs(float(end[1]) - float(bite[1]))
            dz = abs(float(end[2]) - float(bite[2]))
            drift = max(dy, dz)
            drift_msg = f"，横/竖漂移 {dy*100:.1f}/{dz*100:.1f} cm"
            if dx > -pulled_cm / 100.0 * eff_cm:
                return failed(
                    f"实验未通过：指令拉了 {pulled_cm:.1f} cm，但末端实际只移了 "
                    f"{-dx*100:.1f} cm（效率 <{eff_cm*100:.0f}%）。夹爪可能打滑了",
                    code=Code.NOT_VERIFIED, dx_cm=round(-dx * 100, 2),
                    drift_cm=round(drift * 100, 2))
            if drift > float(self.params.get("max_drift_cm", 2.0)):
                return failed(
                    f"实验未通过：后拉时末端横/竖漂移 {drift*100:.1f} cm 超过 "
                    f"{float(self.params.get('max_drift_cm', 2.0))*100:.0f} cm，"
                    f"说明拉的不是直线（抽屉可能被拉歪）",
                    code=Code.NOT_VERIFIED, drift_cm=round(drift * 100, 2))

        gw = ctx.flags.get("pull.grip_width")
        gw_msg = ""
        if gw is not None:
            gw_msg = f"，咬住宽 {float(gw)*100:.1f} cm"
        return ok(f"实验通过：后拉 {pulled_cm:.1f} cm（{len(log)} 段全成功）{gw_msg}{drift_msg}",
                  passed=True, pulled_cm=round(pulled_cm, 2), segments=len(log),
                  grip_width=gw, drift_cm=(round(max(abs(float(end[1]) - float(bite[1])),
                                                    abs(float(end[2]) - float(bite[2]))) * 100, 2)
                                           if bite and end else None))

    # ------------------------------------------------------------ 下探类证据
    def _check_dive(self, ctx: Context) -> Optional[StageResult]:
        """实验 3 的判据：下探到底了没有、夹到了没有、回升了没有。"""
        log = list(ctx.flags.get("dive.log") or [])
        z_reached = ctx.flags.get("dive.z_reached")
        z0 = ctx.flags.get("dive.z0")
        min_cm = float(self.params.get("min_dive_cm", 26.0))

        if not log:
            return failed("实验未通过：没有下探证据（dive.log 为空）—— "
                          "drawer_dive 没跑过，或第一步就失败了",
                          code=Code.NOT_VERIFIED)
        bad = [r for r in log if not r.get("ok")]
        if bad:
            return failed(f"实验未通过：下探有 {len(bad)} 段失败（第 "
                          f"{[r['seg'] for r in bad]} 段）",
                          code=Code.NOT_VERIFIED, failed_segments=[r["seg"] for r in bad])
        if z_reached is None or z0 is None:
            return failed("实验未通过：dive.z_reached / dive.z0 缺失 —— 无法确证实际深度",
                          code=Code.NOT_VERIFIED)
        depth_cm = (float(z0) - float(z_reached)) * 100.0
        if depth_cm < min_cm:
            return failed(f"实验未通过：只下探了 {depth_cm:.1f} cm，要求 ≥ {min_cm:.0f} cm",
                          code=Code.NOT_VERIFIED, depth_cm=round(depth_cm, 2))

        if self.params.get("require_picked"):
            if ctx.picked is None:
                return failed(
                    f"实验未通过：下探 {depth_cm:.1f} cm 完成，但读不到夹爪宽度 —— "
                    f"夹没夹到不可判定。不允许把『不可判定』当成通过",
                    code=Code.NOT_VERIFIED, depth_cm=round(depth_cm, 2))
            if ctx.picked is not True:
                return failed(
                    f"实验未通过：下探 {depth_cm:.1f} cm 后夹爪是空的"
                    f"（宽度 {ctx.flags.get('dive.gripper_width')}），"
                    f"可能竖直方向偏了或夹爪撞壁",
                    code=Code.NO_TARGET, depth_cm=round(depth_cm, 2))

        msg = f"实验通过：下探 {depth_cm:.1f} cm（{len(log)} 段全成功）"
        if ctx.picked:
            msg += "，已确证夹到"
        return ok(msg, passed=True, depth_cm=round(depth_cm, 2), segments=len(log),
                  picked=ctx.picked)

    # ------------------------------------------------------------ 投放类证据
    def _check_drop(self, ctx: Context) -> Optional[StageResult]:
        """实验 3 的后半段：投放到底发生了没有。

        `teach_drop` 会写 drop.done（True = is_holding() 明确返回 False）。
        没有 is_holding 反馈时为 None —— 那就只能报"不可判定"，不许猜成成功。
        """
        if "drop.done" not in ctx.flags and "drop.point" not in ctx.flags:
            return failed("实验未通过：没有投放证据（drop.* 为空）—— teach_drop 没跑过",
                          code=Code.NOT_VERIFIED)
        done = ctx.flags.get("drop.done")
        gw = ctx.flags.get("drop.gripper_width")
        pt = ctx.flags.get("drop.point", "?")
        if done is None:
            return ok(f"投放已在 {pt} 执行，但无 is_holding 反馈，"
                      f"『是否真的放下』不可判定（夹爪宽 {gw}）",
                      passed=None, drop_point=pt, gripper_width=gw, verified=False)
        if done is not True:
            return failed(f"实验未通过：在 {pt} 张爪后 is_holding 仍为 True —— "
                          f"药盒没掉下去（可能夹爪卡住或盒粘在爪上）",
                          code=Code.STAGE_FAILED, drop_point=pt)
        return ok(f"实验通过：已在 {pt} 投放，is_holding=False 确证已松手"
                  f"（夹爪宽 {gw}）", passed=True, drop_point=pt, gripper_width=gw)

    def run(self, ctx: Context, io) -> StageResult:
        need = [str(x) for x in (self.params.get("require_points") or [])]
        max_err = float(self.params.get("max_err_deg", 3.0))
        min_margin = float(self.params.get("min_limit_margin_deg", LIMIT_WARN_DEG))
        require_keys = [str(k) for k in (self.params.get("require_keys") or [])]
        want_pull = bool(self.params.get("require_pull", False))
        want_dive = bool(self.params.get("require_dive", False))
        want_drop = bool(self.params.get("require_drop", False))

        # ---- 预演分支：只有 exp.dry_run 就按"预演口径"给结论，不判实验通过/失败 ----
        dry = list(ctx.flags.get("exp.dry_run") or [])
        if dry and not ctx.flags.get("exp.chain"):
            worst = max(dry, key=lambda r: r["max_delta_deg"]) if dry else {}
            tightest = min((r.get("target_tightest_deg") for r in dry
                            if r.get("target_tightest_deg") is not None), default=None)
            big = [r for r in dry if r["max_delta_deg"] > float(
                self.params.get("warn_delta_deg", 120.0))]
            msg = (f"[预演] {len(dry)} 段，最大单段跨度 {worst.get('max_delta_deg', 0):.1f}°"
                   f"（{worst.get('to', '?')} 的 {worst.get('max_delta_joint', '?')}），"
                   f"目标点最小限位余量 {tightest:.1f}°" if tightest is not None
                   else f"[预演] {len(dry)} 段，最大跨度 {worst.get('max_delta_deg', 0):.1f}°")
            if big:
                msg += f"；⚠️ {len(big)} 段跨度 > {float(self.params.get('warn_delta_deg', 120.0)):.0f}°，真机建议拆分"
            return ok(msg, dry_run=True, passed=None, segments=len(dry),
                      worst=worst, big_segments=[r["to"] for r in big],
                      min_target_margin_deg=tightest)

        # 优先用累积证据（多段链）；退回单段证据（老配置）
        all_chain = list(ctx.flags.get("exp.chain_all") or [])
        chain = all_chain if all_chain else list(ctx.flags.get("exp.chain") or [])
        all_done = list(ctx.flags.get("exp.chain_done_all") or [])
        done = all_done if all_done else list(ctx.flags.get("exp.chain_done") or [])

        # ---- 接触类证据（实验 2）：不走 exp.chain，只看 pull.* ----
        if want_pull or (not chain and (ctx.flags.get("pull.log") or want_pull)):
            res = self._check_pull(ctx)
            if res is not None:
                return res

        # ---- 下探类证据（实验 3）：只看 dive.* ----
        if want_dive or (not chain and (ctx.flags.get("dive.log") or want_dive)):
            res = self._check_dive(ctx)
            if res is not None:
                return res

        # ---- 投放类证据（实验 3 后半段）：只看 drop.* ----
        if want_drop:
            res = self._check_drop(ctx)
            if res is not None:
                return res

        if not chain:
            return failed("没有任何回放证据（exp.chain 为空）—— 前面没跑过 exp_replay_chain",
                          code=Code.NOT_VERIFIED)

        miss = [p for p in need if p not in done]
        if miss:
            return failed(f"实验未通过：{miss} 没有成功回放（已完成 {done}）",
                          code=Code.NOT_VERIFIED, missing=miss, done=done)

        over = [r for r in chain if r.get("err_deg", 0) > max_err]
        if over:
            worst = max(over, key=lambda r: r["err_deg"])
            return failed(
                f"实验未通过：{len(over)} 个点偏差超 {max_err:.1f}°，最差是 {worst['to']} "
                f"的 {worst['err_joint']} 差 {worst['err_deg']:.2f}°",
                code=Code.NOT_VERIFIED, over=[r["to"] for r in over])

        # 跨度检查：哪怕回放成功了，跨度太大的段也不该算"这条链可用"
        # allow_wide_edges：配置里显式声明的「已知不可行边」，只做取证、不计入通过判据。
        # 真机上这些边会被 SAFETY_ABORT 拦下（或已人工跳过），留它们在 chain 里是为了留证据。
        allow = set()
        for e in (self.params.get("allow_wide_edges") or []):
            if isinstance(e, str) and "->" in e:
                a, b = e.split("->", 1)
                allow.add((a.strip(), b.strip()))
        max_span = float(self.params.get("max_span_deg", SPAN_DANGER_DEG))
        wide = [r for r in chain
                if r.get("pre_delta_deg", 0) >= max_span
                and (str(r.get("from")), str(r.get("to"))) not in allow]
        if wide:
            w = max(wide, key=lambda r: r["pre_delta_deg"])
            return failed(
                f"实验未通过：{len(wide)} 段的关节跨度 ≥ {max_span:.0f}°（最宽 "
                f"{w['from']}→{w['to']} {w['pre_delta_deg']:.1f}°）。这种跨度真机上"
                f"规划器会随机选 IK 分支，必须拆中间点。",
                code=Code.NOT_VERIFIED, wide=[f"{r['from']}→{r['to']}" for r in wide])

        tight = [r for r in chain
                 if r.get("tightest_margin_deg") is not None
                 and r["tightest_margin_deg"] < min_margin]
        if tight:
            worst = min(tight, key=lambda r: r["tightest_margin_deg"])
            # 只警告不判失败：余量小本身不是错，但必须让人知道哪个点骑在限位上
            logger.warning("限位余量告警：%s 的 %s 只剩 %.1f°",
                           worst["to"], worst["tightest_joint"], worst["tightest_margin_deg"])

        lack = [k for k in require_keys if k not in ctx.flags]
        if lack:
            return failed(f"实验未通过：缺少证据键 {lack}", code=Code.NOT_VERIFIED, missing=lack)

        max_seen = max(r.get("err_deg", 0) for r in chain)
        min_margin_seen = min(
            (r["tightest_margin_deg"] for r in chain
             if r.get("tightest_margin_deg") is not None), default=None)
        # 报"多少个点到位"用 done 的真实长度（require_points 为空时 len(need)=0 会写出
        # "0 个点全部到位"这种自相矛盾的话）
        n_ok = len(done) if done else sum(1 for r in chain if r.get("verified"))
        head = f"实验通过：{n_ok} 个点到位"
        if need:
            head = f"实验通过：{len(need)} 个指定点全部到位（链上共 {n_ok} 点）"
        tail = f"，最大偏差 {max_seen:.2f}°（容差 {max_err:.1f}°）"
        if min_margin_seen is not None:
            tail += f"，最小限位余量 {min_margin_seen:.1f}°"
        if wide_skipped := [r for r in chain
                            if r.get("pre_delta_deg", 0) >= max_span]:
            tail += f"；另有 {len(wide_skipped)} 段已知不可行边仅作取证（未计入）"
        return ok(
            head + tail,
            passed=True, points=need, done=done, max_err_deg=round(max_seen, 3),
            min_limit_margin_deg=min_margin_seen,
            allowed_wide_edges=[f"{a}->{b}" for a, b in sorted(allow)],
            warn_tight=[r["to"] for r in tight])
