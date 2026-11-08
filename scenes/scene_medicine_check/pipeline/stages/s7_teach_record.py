"""步骤 7：示教采点 —— 把手臂摆到位，读一次当前位姿/关节角，存成 json。

**本 stage 全程只读，一条运动指令都不发。** 手臂由人（或手柄/拖动模式）摆到位，
这里只负责在按回车的那一刻记录端口数据，范式照抄官方 day1/05。

为什么要同时记「末端位姿」和「关节角」两套？
  - 末端位姿（pose）用于**下探段**：箱内要走直线，只能用笛卡尔。
  - 关节角（joint_positions）用于**拉抽屉段**：示教轨迹本质是一串关节角，
    直接回放最稳，不会因为同一个末端位姿有多种关节解而跑歪
    （队友实测踩过：TCP 差 0.036 mm，joint3 差 18°）。

产出的 json 结构：
  {
    "version": 1,
    "created": "2026-11-07 22:10:00",
    "chain": "right_arm",
    "gripper_side": "right",
    "points": {
      "P1_init":   {"pose": [7], "joint_names": [7], "joint_positions": [7],
                    "gripper_width": 0.10, "note": "初始位",
                    "ee_pose_ok": true, "joints_ok": true},
      ...
    }
  }

配置用法：
  {"id": "teach", "stage": "teach_record",
   "params": {"out": "pipeline/configs/teach_drawer.json",
              "points": ["P1_init", "P2_grasp", "P3_pull_done", "P3_dive_ref", "P4_drop"]}}

交互约定：终端里按 **回车** 记录当前点，输入 `s` 跳过当前点，输入 `q` 中止。
非交互环境（stdin 不是 tty，比如被脚本调用）会直接失败，不会卡住等输入。
"""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from typing import Any, Dict, List

from .. import outcome
from ..contracts import Code, Context, Stage, StageResult, failed, ok
from ..registry import register_stage
from ..teach import _safe_read, load_teach, resolve_teach_path, write_teach

logger = logging.getLogger("scene.medicine.stage.teach_record")


@register_stage("teach_record")
class TeachRecord(Stage):
    """交互式采集示教点位（只读）。"""

    stage_id = "teach_record"

    #: 默认点位顺序 —— 与抽屉流程的 8 段动作一一对应
    DEFAULT_POINTS: List[str] = [
        "P1_init",       # 段1：初始位（手臂竖直下垂的固定位）
        "P2_grasp",      # 段2：夹爪咬住抽屉把手的位姿
        "P3_pull_done",  # 段4：抽屉拉到位的末端位姿（拉完）
        "P3_dive_ref",   # 段5 点3：盲下探的起点（28 cm 之旅的上端）
        "P4_drop",       # 段8 点4：垃圾桶上方的投放点
    ]

    def describe(self) -> str:
        pts = self.params.get("points", self.DEFAULT_POINTS)
        out = self.params.get("out", "pipeline/configs/teach_drawer.json")
        return f"示教采 {len(pts)} 个点（纯只读）→ {os.path.basename(str(out))}"

    # ------------------------------------------------------------ 采点
    def _ask(self, name: str, idx: int, total: int) -> str:
        prompt = (f"\n[{idx}/{total}] 请把手臂摆到「{name}」位，摆好后按回车记录"
                  f"（s=跳过，q=中止）：")
        sys.stderr.write(prompt)
        sys.stderr.flush()
        return sys.stdin.readline().strip().lower()

    def _snapshot(self, io, chain: str, side: str) -> Dict[str, Any]:
        """读一次三件套。任一失败只标记，不抛 —— 缺什么就如实记 None。"""
        rec: Dict[str, Any] = {}

        ee = _safe_read(io, "get_ee_pose", chain=chain)
        rec["ee_pose_ok"] = bool(ee.get("success"))
        rec["pose"] = ee.get("pose") if rec["ee_pose_ok"] else None
        if not rec["ee_pose_ok"]:
            rec["pose_err"] = ee.get("message")

        jt = _safe_read(io, "get_joint_positions", joint_groups=[chain], joint_names=[])
        rec["joints_ok"] = bool(jt.get("success"))
        rec["joint_names"] = jt.get("joint_names") if rec["joints_ok"] else None
        rec["joint_positions"] = jt.get("joint_positions") if rec["joints_ok"] else None
        if not rec["joints_ok"]:
            rec["joints_err"] = jt.get("message")

        gw = _safe_read(io, "get_gripper_width", side=side)
        rec["gripper_ok"] = bool(gw.get("success"))
        rec["gripper_width"] = gw.get("width") if rec["gripper_ok"] else None

        return rec

    def run(self, ctx: Context, io) -> StageResult:
        chain = str(self.params.get("chain", "right_arm"))
        side = str(self.params.get("gripper_side", "right"))
        out_path = str(self.params.get("out", "pipeline/configs/teach_drawer.json"))
        points = list(self.params.get("points") or self.DEFAULT_POINTS)
        notes = dict(self.params.get("notes") or {})

        # 非交互环境直接失败：宁可报错，也不要在真机上等一个永远不来的回车
        interactive = bool(self.params.get("interactive", sys.stdin.isatty()))
        if not interactive:
            return failed(
                "示教采点需要交互终端（stdin 不是 tty）。请直接在 HPU 终端里跑，"
                "或先手工把点位写进 json",
                code=Code.STAGE_FAILED, interactive=False)

        out_path = resolve_teach_path(out_path)

        logger.info("示教采点开始：%d 个点 → %s", len(points), out_path)
        collected: Dict[str, Any] = {}
        skipped: List[str] = []

        for i, name in enumerate(points, start=1):
            try:
                ans = self._ask(name, i, len(points))
            except (EOFError, KeyboardInterrupt):
                logger.warning("终端输入中断，已采 %d 个点后退出", len(collected))
                break

            if ans == "q":
                logger.warning("用户中止采集")
                break
            if ans == "s":
                skipped.append(name)
                logger.info("跳过 %s", name)
                continue

            rec = self._snapshot(io, chain, side)
            rec["note"] = notes.get(name, "")
            collected[name] = rec
            logger.info("已记录 %s：pose_ok=%s joints_ok=%s grip=%.3f",
                        name, rec["ee_pose_ok"], rec["joints_ok"],
                        rec.get("gripper_width") or -1.0)

        if not collected:
            return failed("一个点都没采到", code=Code.STAGE_FAILED,
                          skipped=skipped)

        payload = {
            "version": 1,
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "chain": chain,
            "gripper_side": side,
            "frame": "base_link",
            "unit": "m / rad",
            "points": collected,
            "skipped": skipped,
            "warning": "pose 与 joint_positions 均来自真机只读接口，未经二次核验；"
                       "现场换桌/换人重摆后必须重采。",
        }

        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        out_path = write_teach(out_path, payload)

        ctx.flags["teach.file"] = out_path
        ctx.flags["teach.points"] = sorted(collected)

        bad = [k for k, v in collected.items() if not (v["ee_pose_ok"] and v["joints_ok"])]
        msg = f"已采 {len(collected)} 个点 → {out_path}"
        if bad:
            msg += f"；其中 {bad} 有字段缺失（见 json 里的 *_ok 标记）"
        if skipped:
            msg += f"；跳过 {skipped}"

        return ok(msg, file=out_path, points=sorted(collected),
                  incomplete=bad, skipped=skipped)


@register_stage("teach_replay_joints")
class TeachReplayJoints(Stage):
    """按示教 json 回放某个点的关节角（关节空间，不走笛卡尔）。

    为什么拉抽屉段用这个而不是 move_line_to：
      示教记录的是"人摆出来的姿态"，本质就是一串关节角。按关节回放复现性最好，
      不会因为同一末端位姿的多种关节解（肘上/肘下）而跑到另一个构型去。

    代价：关节插值不保证末端走直线。所以**只用于箱外的自由空间移动**，
    箱内下探必须走 draw_dive 的笛卡尔步进。
    """

    stage_id = "teach_replay_joints"

    def describe(self) -> str:
        pt = self.params.get("point", "?")
        f = os.path.basename(str(self.params.get("file", "teach_drawer.json")))
        return f"按关节回放到示教点 {pt}（来自 {f}）"

    def run(self, ctx: Context, io) -> StageResult:
        pt = str(self.params.get("point", ""))
        path = str(self.params.get("file", "pipeline/configs/teach_drawer.json"))
        speed = float(self.params.get("speed_rad_s", 0.2))
        tol = float(self.params.get("tolerance_rad", 0.05))

        path = resolve_teach_path(str(self.params.get("file", "pipeline/configs/teach_drawer.json")))
        if not os.path.exists(path):
            return failed(f"示教文件不存在：{path}", code=Code.DEP_MISSING, file=path)
        teach = load_teach(path)
        if not teach:
            return failed(f"示教文件为空或解析失败：{path}", code=Code.STAGE_FAILED, file=path)

        rec = (teach.get("points") or {}).get(pt)
        if not rec:
            return failed(f"示教文件里没有点位 {pt}（已有 {(teach.get('points') or {}).keys()}）",
                          code=Code.NO_TARGET, file=path)

        names = rec.get("joint_names")
        pos = rec.get("joint_positions")
        if not names or not pos or len(names) != len(pos):
            return failed(f"点位 {pt} 的关节角不完整（joints_ok={rec.get('joints_ok')}），"
                          f"不能回放 —— 请重采该点",
                          code=Code.DEP_MISSING, point=pt)

        # 回放前先读当前角，超过 tol 只告警不阻止：这一步本身就是一次"去某处"的运动
        cur = _safe_read(io, "get_joint_positions", joint_groups=[str(teach.get("chain", "right_arm"))], joint_names=names)
        cur_pos = cur.get("joint_positions") if cur.get("success") else None
        max_dev = None
        if cur_pos and len(cur_pos) == len(pos):
            max_dev = max(abs(a - b) for a, b in zip(cur_pos, pos))
            logger.info("回放 %s：与当前角最大偏差 %.1f°", pt, max_dev * 57.2958)

        ret = io.set_joints(pos, names, speed)
        if not ret.get("success"):
            return outcome(ret, f"回放到 {pt} 失败")

        # 回放后复核：真机上"指令 SUCCESS"不等于"到位"，读一次再判
        chk = _safe_read(io, "get_joint_positions", joint_groups=[str(teach.get("chain", "right_arm"))], joint_names=names)
        if not chk.get("success"):
            logger.warning("回放后无法复核关节角（%s），按未确证处理", chk.get("message"))
            return ok(f"已下发回放 {pt}，但缺关节反馈无法确证到位",
                      point=pt, verified=False, pre_dev_rad=max_dev)

        got = chk.get("joint_positions") or []
        if len(got) != len(pos):
            return ok(f"已回放 {pt}，但反馈维度不符，无法确证", point=pt, verified=False)
        err = max(abs(a - b) for a, b in zip(got, pos))
        deg = err * 57.2958
        ctx.flags[f"replay.{pt}.err_deg"] = deg

        if err > tol:
            return failed(f"回放到 {pt} 后偏差 {deg:.1f}° 超过容差 {tol*57.2958:.1f}°"
                          f"（可能被限位/碰撞检查拦下）",
                          code=Code.NOT_VERIFIED, point=pt, err_deg=deg)
        return ok(f"已回放到 {pt}，最大偏差 {deg:.2f}°", point=pt, err_deg=deg,
                  verified=True, pre_dev_rad=max_dev)
