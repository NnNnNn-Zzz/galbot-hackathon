"""_gen_exp2_config.py —— 生成实验 2 的配置（`exp2_visual_grasp_return.json`）。

为什么用脚本生成而不是手写 JSON
------------------------------
实验 2 的回程是**实验 1 那条 37 段链的反向**：
    P0 →（7 段，实验 1 leg3 倒序）→ P1
      →（12 段，实验 1 leg2 倒序）→ P2
      →（18 段，实验 1 leg1 倒序）→ 抱持位 CARRY_home
37 个点名手抄一遍必然出错，而且以后实验 1 的 VIA 点一改就会悄悄不一致。
这里直接从 `exp1_real_carry_p2_p1_p0.json` 读点、倒序、生成，保证两条链**结构上是同一条路**，
"回程走的就是去程"这件事变成代码保证，而不是注释承诺。

用法
----
    python tools/drawer_calib/_gen_exp2_config.py           # 只打印摘要
    python tools/drawer_calib/_gen_exp2_config.py --write   # 写 configs/exp2_visual_grasp_return.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CFG_DIR = os.path.join(REPO, "scenes", "scene_medicine_check", "pipeline", "configs")

EXP1 = os.path.join(CFG_DIR, "exp1_real_carry_p2_p1_p0.json")
OUT = os.path.join(CFG_DIR, "exp2_visual_grasp_return.json")

TEACH = "pipeline/configs/teach_drawer_real.json"

#: 下探段（由 tools/drawer_calib/_plan_exp2_dive.py 生成）
DIVE_N = 10
DIVE_PREFIX = "VIA_P0_DIVE"

#: 合爪指令宽度（m）。刻意小于盒厚 7.5 cm —— 详见 s12_exp2_stages.ExpGrip 的注释。
CLOSE_CMD_M = 0.02
#: 期望的回读咬合宽度区间（m）：用户要求「咬合维持 7.5 cm 的夹爪距离」
HOLD_MIN_M, HOLD_MAX_M = 0.060, 0.090
#: 回读低于它就判「夹空」
HOLD_FLOOR_M = 0.055


def _leg_points(cfg: dict, step_id: str) -> list:
    for s in cfg["steps"]:
        if s.get("id") == step_id:
            return [str(p) for p in (s.get("params") or {}).get("points") or []]
    raise KeyError(f"exp1 配置里没有 {step_id}")


def build() -> dict:
    with open(EXP1, "r", encoding="utf-8") as fh:
        e1 = json.load(fh)
    leg1 = _leg_points(e1, "leg1_carry_to_p2")   # CARRY → P2（18 段）
    leg2 = _leg_points(e1, "leg2_p2_to_p1")      # P2 → P1（12 段）
    leg3 = _leg_points(e1, "leg3_p1_to_p0")      # P1 → P0（7 段）

    # 回程 = 各段倒序；端点由后一段的终点补上。
    # 注意：实验 1 的每条 leg 的 points 里**最后一项已经是那一段的终点**（P2/P1/P0），
    # 倒序后它会跑到第一位，成为"当前就在此地"的空转段 —— 必须丢掉，否则日志里会多出
    # 一条 from==to 的假动作（真机上还真会下发一次 set_joints）。
    back1 = list(reversed(leg3))[1:] + ["P1_measured"]     # P0 → P1（7 段）
    back2 = list(reversed(leg2))[1:] + ["P2_measured"]     # P1 → P2（12 段）
    back3 = list(reversed(leg1))[1:] + ["CARRY_home"]      # P2 → 抱持位（18 段）
    n_back = len(back1) + len(back2) + len(back3)

    dive_down = [f"{DIVE_PREFIX}_{i}" for i in range(1, DIVE_N + 1)]
    # 回撤：从最底点出发，倒序回放上面那些构型。**丢掉 reversed 的第一项**
    # （它就是当前所在的 VIA_P0_DIVE_10），否则第一段是 from==to 的空转指令。
    dive_up = list(reversed(dive_down))[1:] + ["P0_measured"]

    common = {
        "chain": "right_arm",
        "speed_rad_s": 0.15,
        "tolerance_deg": 3.0,
        "settle_s": 0.4,
        "gripper_side": "right",
        "stop_on_fail": True,
        "block_danger_span": True,
        "require_fresh_joints": False,
        "file": TEACH,
    }

    def replay(sid: str, points: list, start_point: str, comment: str) -> dict:
        return {
            "id": sid,
            "stage": "exp_replay_chain",
            "params": dict(common, points=points, start_point=start_point, _comment=comment),
        }

    def snap(sid: str, label: str) -> dict:
        return {"id": sid, "stage": "exp_arm_snapshot",
                "params": {"label": label, "chain": "right_arm",
                           "gripper_side": "right", "fail_on_tight": False}}

    def grip(sid: str, action: str, label: str, width=None, **extra) -> dict:
        p = {"action": action, "side": "right", "label": label}
        if width is not None:
            p["width_m"] = width
        p.update(extra)
        return {"id": sid, "stage": "exp_grip", "params": p}

    steps = [
        {"id": "preflight", "stage": "preflight", "non_fatal": True},
        snap("snap_start", "START"),

        # ---- 0. 起始位硬门：必须已经在 P0，否则零运动 ----
        {"id": "gate_at_p0", "stage": "exp_gate_start",
         "params": {"at": "P0_measured", "file": TEACH, "chain": "right_arm",
                    "tolerance_deg": 2.0, "gripper_side": "right",
                    "_comment": "实验 1 的终点是 P0。手臂可能被拖过/被遥操作占过，"
                                "所以实测比对；不在 P0 就一条运动指令都不发。"}},

        # ---- 1. 视觉识别（只读）----
        {"id": "vision_check", "stage": "exp_vision_check",
         "params": {"camera": "head_left", "tag": "exp2_grasp",
                    "policy": "advisory", "backend": "",
                    "static_verdict": {
                        "ok": True, "box_present": True, "graspable": True,
                        "offset_x_cm": 0.0, "offset_y_cm": 0.0, "confidence": 0.9,
                        "reason": "mock 静态桩：仅用于把 gate 分支也跑一遍，"
                                  "真实运行请在配置里删掉 static_verdict 或配 VLM Key"},
                    "_comment": "默认 advisory：视觉只记录不拦路，结论里 verified=False。"
                                "要视觉把关就把 policy 改成 gate。"}},

        # ---- 2. 张爪（全开进腔，避免爪子蹭到抽屉壁）----
        grip("open_gripper", "open", "open_before_dive", width=0.10),

        # ---- 3. 竖直下探 42 cm ----
        replay("dive_to_grasp", dive_down, "P0_measured",
               "P0 竖直下探 10 段 × 4.2 cm，TCP z 1.2448 → 0.8248。"
               "x/y 锁死、姿态锁死为 P0 姿态，全部离线 IK（单段 ≤5.2°，最小限位余量 8.9°）。"),
        snap("snap_at_grasp", "AT_GRASP"),

        # ---- 4. 合爪 ----
        grip("grasp_close", "close", "grasp", width=CLOSE_CMD_M,
             effort=10.0,
             expect_min_m=HOLD_MIN_M, expect_max_m=HOLD_MAX_M,
             hard_floor_m=HOLD_FLOOR_M),

        # ---- 5. 原路回撤到 P0 ----
        replay("ascend_to_p0", dive_up, dive_down[-1],
               "倒序回放同一批构型（10 段）+ 末段回 P0_measured。"
               "构型集合与下行完全相同，故跨度/限位余量与下行一致。"),
        snap("snap_back_p0", "AT_P0_HOLDING"),

        # ---- 6. 咬合维持校验（回读 7.5 cm 才算夹住并带着走）----
        grip("hold_at_p0", "check", "hold_at_p0",
             expect_min_m=HOLD_MIN_M, expect_max_m=HOLD_MAX_M,
             hard_floor_m=HOLD_FLOOR_M),

        # ---- 7. 带载回程：P0 → P1 → P2 → 抱持位 ----
        replay("leg1_p0_to_p1", back1, "P0_measured",
               "实验 1 leg3 倒序 7 段。起点 j7 余量只有 4.4°（P0 固有），第一步就离开它。"),
        snap("snap_at_p1", "AT_P1"),
        replay("leg2_p1_to_p2", back2, "P1_measured",
               "实验 1 leg2 倒序 12 段（关节 lerp；端点 j3 差 170° 属两支 IK 解）。"),
        snap("snap_at_p2", "AT_P2"),
        replay("leg3_p2_to_carry", back3, "P2_measured",
               "实验 1 leg1 倒序 18 段，末端直线退回抱持位。"),
        snap("snap_at_carry", "AT_CARRY"),
        grip("hold_at_carry", "check", "hold_at_carry",
             expect_min_m=HOLD_MIN_M, expect_max_m=HOLD_MAX_M,
             hard_floor_m=HOLD_FLOOR_M),

        # ---- 8. 只读断言 ----
        {"id": "verify_exp2", "stage": "exp_verify_chain",
         "params": {
             "require_points": ["VIA_P0_DIVE_10", "P0_measured", "P1_measured",
                                "P2_measured", "CARRY_home"],
             "max_err_deg": 3.0,
             "min_limit_margin_deg": 2.0,
             "max_span_deg": 90.0,
             "allow_wide_edges": [],
             "require_keys": ["exp.gate", "vision", "grip.grasp",
                              "grip.hold_at_p0", "grip.hold_at_carry"],
             "_comment": "所有回放点到位 + 无 ≥90° 宽边 + 五类证据键齐全。"}},
    ]

    return {
        "name": "drawer.exp2_visual_grasp_return",
        "description": (
            "实验 2 · 真机版：从 P0 起，视觉识别 → 竖直下探 42 cm 夹取抽屉内药盒 → "
            "原路回撤到 P0（夹爪保持咬合 7.5 cm）→ P1 → P2 → 抱持位。"
            f"去程下探 10 段 + 回撤 10 段 + 回程 {n_back} 段，共 57 段；全程 0 底盘运动、0 左臂运动。"),
        "version": 1,
        "experiment": {
            "layer": 2,
            "start_point": "P0_measured",
            "mock_scene": "drawer_p0",
            "goal": (
                "验证『P0 竖直下探能进抽屉内腔并夹住药盒，再带着盒子原路撤回抱持位』"
                "这条完整作业链，为接入 YOLO/视觉定位后的自动化打好运动学基础。"),
            "scope": (
                "右臂 + 右夹爪。不含底盘转向、不含投放、不含左臂。"
                "视觉段只做识别与存证，不驱动位姿修正（无可靠的手眼标定）。"),
            "prerequisite": (
                "实验 1 已通过，且手臂当前就停在 P0（±2°，由 gate_at_p0 硬门判定）。"),
            "direction_rationale": (
                "去程 CARRY→P2→P1→P0 已由实验 1 在真机验证（37/37 段）。"
                "本实验到此为止不重走去程，直接接 P0 往下；回程是实验 1 那条链的严格倒序，"
                "构型集合完全相同，所以跨度与限位余量也被实验 1 一并验证过了。"),
            "pass_criteria": [
                "起始位硬门通过：起跑时已在 P0_measured ≤2.0°",
                "下探 10 段 + 回撤 10 段 + 回程 37 段，共 57 段全部到位，每段关节偏差 ≤3.0°",
                "每段跨度 max < 90°（本链最大 15.0°，来自实验 1 的 IK 分支尾差）",
                "合爪后回读宽度落在 0.060–0.090 m（用户要求咬合维持 7.5 cm），且 ≥0.055 m 硬下限",
                "回撤到 P0 后、以及回到抱持位后，回读宽度仍在该区间（= 盒子全程没掉）",
                "收尾停在抱持位 CARRY_home，不产生任何后续运动指令",
            ],
            "expected_outcome": (
                "总状态 ok；手臂回到抱持位；夹爪回读约 7.5 cm；视觉若未配后端则记 verified=False。"),
            "measured_spans_deg": {
                "P0_measured->VIA_P0_DIVE_1": 5.1,
                "下探段单段最大（离线 IK）": 5.15,
                "VIA_P0_DIVE_10->P0_measured（回撤末段）": 5.15,
                "P0_measured->VIA_P1_P0_6": 13.3,
                "P1->P2 关节lerp 单段": 14.2,
                "P2->CARRY 单段最大": 15.0,
            },
            "known_constraints": {
                "下探深度是配置值，不是视觉给的": (
                    "0.42 m 是按『药盒立在内腔底面上、TCP 抓盒腰』估的。"
                    "现场必须量内腔底面高度与药盒高度，用 "
                    "`tools/drawer_calib/_plan_exp2_dive.py --depth <m> --steps 10 --write` 重生成点位。"),
                "抽屉顶板是否挡竖直通道未实测": (
                    "占位盒按 z 上限 1.05 m 估。若外壳顶更高，竖直下探会在 VIA_P0_DIVE_5 前后撞顶，"
                    "表现为该段 SUCCESS 但回读偏差超容差，或规划器直接 FAULT。"),
                "带载回程净空靠估": (
                    "TCP 在抽屉占位盒内腔上方时，最小竖向余量 8.37 cm（VIA_P1_P0_4，z=1.134 vs 盒顶 1.05）。"
                    "盒子若从 TCP 下垂 >8 cm 就会刮到盒顶。盒子实际外廓未量。"),
                "P0 不能上抬": (
                    "想先抬 5 cm 再回程是**不可行**的：离线 IK 显示上抬 3 cm 就把 j7 用到限位"
                    "（余量 0.0°），因为 P0 本身 j7 只剩 4.4°。所以回程严格按原路，不加上抬。"),
            },
            "site_assumption": (
                "沿用用户 2026-11-08 09:52 的净空场地假设：不做连杆级/环境级碰撞检测，"
                "抽屉占位盒只作 TCP 参考。下探段**主动进入**占位盒范围（那是伸进抽屉），属预期。"),
            "exit_policy": (
                "跑完一个周期停在抱持位 CARRY_home，夹着药盒不动；不自动张爪、不自动投放、"
                "不自动归位。后续投放由实验 3 或人工决定。"),
            "failure_taxonomy": {
                "SAFETY_ABORT": "起始位不在 P0（gate）/ 某段跨度 ≥90° / 某点贴限位 <2°",
                "NO_TARGET": "合爪后回读宽度 <0.055 m = 夹空，药盒不在爪心（下探深度或 xy 偏了）",
                "NOT_VERIFIED": "回读宽度不在 0.060–0.090 m / 读不到夹爪宽度 / 视觉 gate 不通过",
                "UNDETERMINED": "读不到关节角或夹爪宽度 —— 不猜、也不再发动作",
            },
        },
        "notes": [
            "⚠️ VIA_P0_DIVE_* 全部是离线 IK（tools/drawer_calib/_plan_exp2_dive.py），零实测。",
            "⚠️ 执行前必须已停在 P0：先 scripts/home_to_carry.py --yes 回正，再跑实验 1 到达 P0。"
            "gate_at_p0 会在偏差 >2° 时零运动退出。",
            "⚠️ 合爪指令是 0.02 m（不是 0.075 m）。真机上夹爪合到盒厚就停住，回读 0.075 才是"
            "『咬住了』的证据；命令 0.075 时空爪与夹住无法区分。可观测结果仍是咬合 7.5 cm。",
            "⚠️ 视觉默认 advisory：没配 VLM Key 时结论记 verified=False，但**不拦路**。"
            "配置里带了 static_verdict，只有在 mock 上才会生效（真机上会被忽略）。",
            "⚠️ 本实验不做视觉位姿修正：腕部 RGB-D 的 depth_alignment=unverified、头部 depth_sensor=null，"
            "没有可信的手眼标定，任何「按视觉纠偏」都是赌。",
            "速度统一 0.15 rad/s，手放急停。",
        ],
        "steps": steps,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="生成实验 2 配置")
    ap.add_argument("--write", action="store_true", help="写进 configs/")
    args = ap.parse_args()
    cfg = build()
    n_replay = sum(1 for s in cfg["steps"] if s["stage"] == "exp_replay_chain")
    n_pts = sum(len(s["params"]["points"]) for s in cfg["steps"]
                if s["stage"] == "exp_replay_chain")
    print(f"steps={len(cfg['steps'])}  回放步={n_replay}  回放点合计={n_pts}")
    for s in cfg["steps"]:
        if s["stage"] == "exp_replay_chain":
            p = s["params"]
            print(f"  {s['id']:<20} {len(p['points']):>2} 段  "
                  f"起点={p['start_point']:<16} 首→末={p['points'][0]} → {p['points'][-1]}")
    if not args.write:
        print("（未写文件。要写入加 --write）")
        return 0
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(cfg, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    print(f"✅ 已写入 {os.path.relpath(OUT, REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
