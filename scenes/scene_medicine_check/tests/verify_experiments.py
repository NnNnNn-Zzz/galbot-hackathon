"""二次验证：抽屉三实验（示教链 / 抽屉开合 / 下探投放）的离线回归。

跑法（项目根目录）：
    python3 scenes/scene_medicine_check/tests/verify_experiments.py
不需要 pytest，不需要硬件。

为什么要单独一个文件（而不是塞进 verify_pipeline.py）：
  verify_pipeline.py 验的是**药品基线主线**（7 步流程 + 过期判定）。
  这里验的是**抽屉场景的三层实验**，跑的是 configs/exp1..3_*.json 这三份配置，
  走的是 runner + exp_* stage，不碰 baseline_flow。两套东西互不干扰，
  合在一起跑反而会让"9 项全过"这个基线结论变得含糊。

覆盖的场景（42 项）：
  A. 正常路径（4 项）
     1. 实验 1 全绿，链条跑到 P0
     2. 实验 1 的 P0→P1→P2 方向断言（端点到达顺序不得倒置）
     3. 实验 2（拉抽屉，旧 L2）全绿，后拉 30 cm 有确证
     4. 实验 3 全绿，下探 + 夹到 + 投放三段都有确证
  B. 安全拦截（3 项）
     5. 关掉 block_danger_span 也拦不住 —— 跨度超 90° 时 planner 层必须拒绝
     6. 限位余量 < 2° 的点 → SAFETY_ABORT，不发后续指令
     7. 示教文件缺点位 → NO_TARGET（不是 unknown，不许瞎猜）
  C. 三终态语义（4 项）
     8. set_joints=unknown → UNDETERMINED，后续步骤全 skipped，**不执行归位**
     9. set_joints=fail → STAGE_FAILED，**执行归位**（must_run 步）
    10. 夹爪读不到宽度 → 夹没夹到不可判定，verify 不允许判成 passed
    11. is_holding 为 None → 投放"不可判定"，不许当成功
  D. 契约一致性（3 项）
    12. 四份配置用的 stage 全部已注册
    13. 四份配置引用的示教点全部存在
    14. 配置文件里所有 known_impossible_edges 都被 allow_wide_edges 覆盖
  E. 真机可执行性（3 项）／F. 上机前准备（7 项）／G. 方向纠正一致性（8 项）
    15〜32. 见各项标题
  H. 实验 2「视觉识别 + 箱内夹取 + 带载回程」（10 项，2026-11-08 新增）
    33. 19 步全 ok，verify_exp2 判通过（57 段链）
    34. 动作顺序 下探→回撤→P1→P2→抱持位，端点到达顺序不得倒置
    35. 起始位不在 P0 → SAFETY_ABORT 且零 set_joints
    36. 爪心没盒子 → grasp_close 必须 NO_TARGET（不许把 set_gripper 的 SUCCESS 当夹到）
    37. 夹爪宽度读不到 → open_gripper 即 unknown，整体全停
    38. 视觉 gate 判定不通过 → 停爪，零 set_joints / 零 set_gripper
    39. 视觉 advisory（未配 VLM）→ 照走但 verified=False
    40. 回程 = 实验 1 的严格倒序（18/12/7 段逐点相等）
    41. 下探 10 点：限位内、标未实测、z 单调降、总深 42 cm、落点对齐 mock 盒位
    42. 只动右臂：零底盘指令、左臂从未被写入、正好 57 段
"""

from __future__ import annotations

import json
import logging
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT)

from scenes.scene_medicine_check.pipeline import io as pio  # noqa: E402
from scenes.scene_medicine_check.pipeline import registry, runner  # noqa: E402
from scenes.scene_medicine_check.pipeline.contracts import Context  # noqa: E402

logging.basicConfig(level=logging.ERROR, format="%(levelname)s %(name)s: %(message)s")

CFG_DIR = os.path.join(ROOT, "scenes", "scene_medicine_check", "pipeline", "configs")
EXPS = {
    "1": "exp1_teach_chain.json",
    "2": "exp2_drawer_open.json",
    "3": "exp3_dive_drop.json",
    "1a": "exp1_real_carry_p2_p1_p0.json",
    #: 实验 2（新编号）：P0 视觉识别 → 箱内夹取 → 带载回程到抱持位。
    #: 与 "2"（拉抽屉，旧 L2，编号已被本实验占用）是两份不同配置，别混。
    "2v": "exp2_visual_grasp_return.json",
}
TEACH = os.path.join(CFG_DIR, "teach_drawer_real.json")

RESULTS = []


def check(no: int, name: str, cond: bool, detail: str = "") -> None:
    RESULTS.append((no, name, bool(cond), detail))
    mark = "PASS" if cond else "FAIL"
    print(f"[{mark}] {no:>2}. {name}" + (f"\n         {detail}" if detail else ""))


def load(exp: str) -> dict:
    with open(os.path.join(CFG_DIR, EXPS[exp]), "r", encoding="utf-8") as fh:
        return json.load(fh)


def teach_points() -> dict:
    with open(TEACH, "r", encoding="utf-8") as fh:
        return json.load(fh).get("points") or {}


def step(res: dict, step_id: str) -> dict:
    for t in res.get("trace") or []:
        if t.get("step") == step_id:
            return t
    return {}


def run(exp: str, cfg: dict | None = None, faults: dict | list | None = None,
        objects=None, start: str | None = None, teach_file: str = "teach_drawer_real.json",
        want_io: bool = False):
    """跑一份实验配置，返回 runner 的结果字典。全程 mock，零硬件。

    faults 支持两种写法：
      {"set_joints": "unknown"}                       —— 单次注入
      [{"method": "...", "behavior": "...", "times": 5}] —— 完整写法（可指定次数）

    want_io=True 时返回 (结果, 后端)，后端对象上能看 `.calls`（下发过哪些指令）
    —— 用于断言"这一步一条运动指令都没发"。
    """
    if isinstance(faults, dict):
        faults = [{"method": k, "behavior": v} for k, v in faults.items()]
    cfg = cfg or load(exp)
    exp_meta = cfg.get("experiment") or {}
    kwargs = {}
    scene = str(exp_meta.get("mock_scene") or "")
    if scene:
        kwargs["objects"] = pio.SCENES[scene]()
    st = start or str(exp_meta.get("start_point") or "")
    if st:
        from scenes.scene_medicine_check.pipeline.teach import load_point
        # 起步姿态永远从**真示教文件**取（临时故障文件里可能没有这个点）
        rec = load_point("pipeline/configs/teach_drawer_real.json", st)
        kwargs["start_joints"] = [float(v) for v in rec["joint_positions"]]
        if rec.get("pose"):
            kwargs["start_pose"] = [float(v) for v in rec["pose"]]
    if objects is not None:
        kwargs["objects"] = objects
    kwargs["teach_file"] = teach_file
    bio = pio.build_io("mock", faults=faults or [], **kwargs)
    res = runner.run_pipeline(Context(params={}), bio, cfg)
    return (res, bio) if want_io else res


def statuses(res: dict) -> dict:
    return {t.get("step"): t.get("status") for t in (res.get("trace") or [])}


# ============================================================== A 正常路径
def part_a() -> None:
    r1 = run("1")
    st1 = statuses(r1)
    check(1, "实验 1 全绿：抱持位 → P2 → P1 → P0 全 10 步 ok，最终停在 P0",
          r1.get("status") == "ok"
          and all(st1.get(k) == "ok" for k in st1)
          and st1.get("snap_end") == "ok"
          and st1.get("snap_at_p0") == "ok",
          f"status={r1.get('status')} counts={r1.get('counts')} "
          f"非 ok 步={[k for k, v in st1.items() if v != 'ok']}")

    # 2. 方向断言：链条必须按 CARRY → P2 → P1 → P0 走，且不能出现反向边
    tr = r1.get("trace") or []
    order = []
    for x in tr:
        if str(x.get("stage")) != "exp_replay_chain":
            continue
        for row in (x.get("data") or {}).get("trace") or []:
            to = str(row.get("to") or "")
            if to in ("P2_measured", "P1_measured", "P0_measured"):
                order.append(to)
    want = ["P2_measured", "P1_measured", "P0_measured"]
    check(2, "实验 1 方向正确：端点到达顺序必须是 P2 → P1 → P0（P0 是最高点、最后到）",
          order == want, f"实际顺序={order}，应为={want}")


    r2 = run("2")
    st2 = statuses(r2)
    v2 = step(r2, "verify_exp2")
    check(3, "实验 2 全绿：咬住 + 后拉 30 cm + 前推退爪 + verify 判通过",
          r2.get("status") == "ok" and v2.get("status") == "ok"
          and all(v == "ok" for k, v in st2.items() if k != "preflight"),
          f"status={r2.get('status')} verify={v2.get('message', '')[:70]}")

    r3 = run("3")
    st3 = statuses(r3)
    vd = step(r3, "verify_dive")
    v3 = step(r3, "verify_exp3")
    check(4, "实验 3 全绿：下探夹取 / 投放 / 归位 三段全 ok，两条 verify 都通过",
          r3.get("status") == "ok" and vd.get("status") == "ok" and v3.get("status") == "ok"
          and st3.get("home") == "ok",
          f"status={r3.get('status')} verify_dive={(vd.get('message') or '')[:50]} "
          f"verify_exp3={(v3.get('message') or '')[:50]}")


# ============================================================== B 安全拦截
def _mutate(cfg: dict, step_id: str, **kv) -> dict:
    for s in cfg["steps"]:
        if s.get("id") == step_id:
            s.setdefault("params", {}).update(kv)
    return cfg


def _drop_step(cfg: dict, step_id: str) -> dict:
    cfg["steps"] = [s for s in cfg["steps"] if s.get("id") != step_id]
    return cfg


def part_b() -> None:
    # 5. 跨度拦截：把 leg2 的中间点全删掉 → P2→P1 直连 170.0°@j3 / 合计 522.3° → danger
    cfg = load("1")
    _mutate(cfg, "leg2_p2_to_p1", points=["P1_measured"], block_danger_span=True)
    r = run("1", cfg=cfg)
    s = step(r, "leg2_p2_to_p1")
    check(5, "跨度拦截：P2→P1 直连（170.0°/合计 522.3°，两支 IK 解）必须被 SAFETY_ABORT 拒绝",
          s.get("status") == "failed" and str(s.get("code")) == "SAFETY_ABORT",
          f"status={s.get('status')} code={s.get('code')} msg={(s.get('message') or '')[:90]}")

    # 6. 限位余量：硬造一个 j6 贴限位的点（−41.5°，限位下限 −42.1°，余量 0.6° < 2° 失败线）
    import copy
    pts = teach_points()
    evil = copy.deepcopy(pts["CARRY_home"])
    names = list(evil["joint_names"])
    evil["joint_positions"] = list(evil["joint_positions"])
    evil["joint_positions"][names.index("right_arm_joint6")] = -41.5 * math.pi / 180.0
    tmp = "_verify_exps_evil_teach.json"
    tmp_abs = os.path.join(CFG_DIR, tmp)
    with open(tmp_abs, "w", encoding="utf-8") as fh:
        json.dump({"points": {"EVIL_TIGHT": evil}}, fh, ensure_ascii=False)
    cfg = {
        "name": "verify.exp_tight",
        "steps": [
            {"id": "preflight", "stage": "preflight", "non_fatal": True},
            {"id": "tight", "stage": "exp_replay_chain",
             "params": {"file": f"pipeline/configs/{tmp}", "points": ["EVIL_TIGHT"],
                        "chain": "right_arm", "speed_rad_s": 0.15, "tolerance_deg": 3.0,
                        "settle_s": 0.0, "gripper_side": "right", "stop_on_fail": True,
                        "block_danger_span": True}},
        ],
    }
    try:
        r = run("1", cfg=cfg, start="CARRY_home", teach_file=tmp)
        s = step(r, "tight")
        okk = (s.get("status") == "failed" and str(s.get("code")) == "SAFETY_ABORT"
               and "限位" in str(s.get("message") or ""))
        detail = f"status={s.get('status')} code={s.get('code')} msg={(s.get('message') or '')[:110]}"
    finally:
        os.remove(tmp_abs)
    check(6, "限位余量：j6=−41.5°（余量 0.6° < 2° 失败线）→ SAFETY_ABORT", okk, detail)

    # 7. 缺点位 → NO_TARGET
    cfg = load("1")
    _mutate(cfg, "leg2_p2_to_p1", points=["P2_measured", "NO_SUCH_POINT"])
    r = run("1", cfg=cfg)
    s = step(r, "leg2_p2_to_p1")
    check(7, "示教文件缺点位 → NO_TARGET（不是 unknown，不许瞎猜）",
          s.get("status") == "failed" and str(s.get("code")) == "NO_TARGET",
          f"status={s.get('status')} code={s.get('code')} msg={(s.get('message') or '')[:90]}")


# ============================================================== C 三终态
def part_c() -> None:
    # 8. unknown → 全停，不归位
    r = run("2", faults=[{"method": "set_joints", "behavior": "unknown", "times": 99}])
    st = statuses(r)
    later = [k for k in st if st[k] == "ok" and k not in ("preflight", "snap_start")]
    check(8, "set_joints=unknown → UNDETERMINED，后续全 skipped，不执行归位",
          r.get("status") == "unknown"
          and step(r, "to_p0").get("status") == "unknown"
          and all(v == "skipped" for k, v in st.items()
                  if k not in ("preflight", "snap_start", "to_p0"))
          and not later,
          f"status={r.get('status')} to_p0={st.get('to_p0')} "
          f"aborted_without_safety={r.get('aborted_without_safety')}")

    # 9. failed → 已知失败，must_run 归位照跑
    r = run("3", faults=[{"method": "set_joints", "behavior": "fail", "times": 99}])
    st = statuses(r)
    check(9, "set_joints=fail → STAGE_FAILED，归位（must_run）照常执行",
          r.get("status") == "failed"
          and step(r, "to_dive_ref").get("status") == "failed"
          and st.get("home") not in (None, "skipped"),
          f"status={r.get('status')} to_dive_ref={st.get('to_dive_ref')} home={st.get('home')}")

    # 10. 夹爪宽度读不到 → picked=None，verify 不许判 passed
    r = run("3", faults=[{"method": "get_gripper_width", "behavior": "unknown", "times": 99}])
    vd = step(r, "verify_dive")
    check(10, "夹爪宽度读不到 → verify_dive 必须 failed（不许把「不可判定」当通过）",
          vd.get("status") == "failed",
          f"verify_dive={vd.get('status')} code={vd.get('code')} "
          f"msg={(vd.get('message') or '')[:100]}")

    # 11. is_holding 缺失 → 投放不可判定，但不算失败（信息不足 ≠ 失败）
    r = run("3", faults=[{"method": "is_holding", "behavior": "unknown", "times": 99}])
    v3 = step(r, "verify_exp3")
    data = v3.get("data") or {}
    check(11, "is_holding 读不到 → verify_exp3 报「不可判定」（passed=None），不判成成功也不判失败",
          v3.get("status") == "ok" and data.get("passed") is None,
          f"verify_exp3={v3.get('status')} passed={data.get('passed')} "
          f"msg={(v3.get('message') or '')[:100]}")


# ============================================================== D 契约
def part_d() -> None:
    used, unknown_stage = [], []
    for exp in EXPS:
        cfg = load(exp)
        for s in cfg["steps"]:
            sid = s.get("stage")
            used.append(sid)
            if not registry.get_stage(sid):
                unknown_stage.append((exp, sid))
    check(12, f"三份配置引用的 {len(set(used))} 个 stage 全部已注册",
          not unknown_stage, f"未注册：{unknown_stage}" if unknown_stage else "全部命中")

    pts = teach_points()
    missing = []
    for exp in EXPS:
        cfg = load(exp)
        for s in cfg["steps"]:
            for p in (s.get("params") or {}).get("points") or []:
                if p not in pts:
                    missing.append((exp, s.get("id"), p))
    check(13, f"三份配置引用的示教点全部存在于 teach_drawer_real.json（共 {len(pts)} 点）",
          not missing, f"缺失：{missing}" if missing else "全部命中")

    gap = []
    for exp in EXPS:
        cfg = load(exp)
        known = set((cfg.get("experiment") or {}).get("known_impossible_edges") or {})
        allowed = set()
        for s in cfg["steps"]:
            for e in (s.get("params") or {}).get("allow_wide_edges") or []:
                a, b = str(e).split("->")
                allowed.add((a.strip(), b.strip()))
        for edge in known:
            a, b = [x.strip() for x in str(edge).split("->")]
            if not any(x[0] in (a, "START") and x[1] == b for x in allowed) and a != "P1_measured":
                gap.append((exp, edge))
    check(14, "配置里声明的「已知不可行边」都有对应的 allow_wide_edges 豁免（否则实验永远判不通过）",
          not gap, f"未豁免：{gap}" if gap else "全部覆盖")


def part_e() -> None:
    """E 真机可执行性（入口 ramp / require_mock / 真机版配置）。"""

    # 15. require_mock：在非 MockIO 后端上必须拒绝执行、零运动
    import importlib
    from scenes.scene_medicine_check.pipeline import io as pio
    from scenes.scene_medicine_check.pipeline.contracts import Context
    s11 = importlib.import_module(
        "scenes.scene_medicine_check.pipeline.stages.s11_exp_stages")

    class WrapperIO(pio.MockIO):          # 类名冒充真机后端
        pass
    WrapperIO.__name__ = "WrapperIO"

    st = s11.ExpReplayChain({
        "points": ["P2_measured"], "chain": "right_arm",
        "block_danger_span": False, "require_mock": True,
        "file": "pipeline/configs/teach_drawer_real.json"})
    r = st.run(Context(params={}), WrapperIO())
    check(15, "require_mock：非 MockIO 后端上拒绝执行 170° 关节指令（真机超时不会停机）",
          r.status == "failed" and str(r.code).endswith("SAFETY_ABORT"),
          f"status={r.status} code={r.code} msg={r.message[:70]}")

    # 16. 真机版 exp1：mock 全绿，最终停在 P0（最高点）
    rr = run("1a")
    tr = rr.get("trace") or []
    at_p0 = any(x.get("stage") == "exp_arm_snapshot"
                and (x.get("data") or {}).get("label") == "HOLD_AT_P0"
                and x.get("status") == "ok" for x in tr)
    n_fail = sum(1 for x in tr if x.get("status") == "failed")
    check(16, "真机版 exp1（抱持位→P2→P1→P0）：mock 全绿、零 failed、最终停在 P0",
          rr.get("status") == "ok" and at_p0 and n_fail == 0,
          f"status={rr.get('status')} 到P0={at_p0} failed={n_fail}（应为 0）步数={len(tr)}（应为 10）")

    # 17. 真机版 exp1 里每一次真正下发的关节指令跨度都必须 < 90°
    #     只统计 motion_dispatched=True 的行；被 SAFETY_ABORT 拦下的行是 False，不算。
    #     注：trace 里每步的数据在 data 键下，链明细在 data["trace"]；不是 detail。
    wide, n_motion, worst = [], 0, 0.0
    for x in tr:
        if str(x.get("stage")) != "exp_replay_chain":
            continue
        for row in (x.get("data") or {}).get("trace") or []:
            if not row.get("motion_dispatched"):
                continue
            n_motion += 1
            pre = float(row.get("pre_delta_deg") or 0.0)
            worst = max(worst, pre)
            if pre >= 90.0:
                wide.append((x.get("step"), row.get("to"), round(pre, 1)))
    check(17, "真机版 exp1 每一次真正下发的关节指令跨度都 < 90°，且单段 ≤20°",
          (not wide) and n_motion == 37 and worst <= 20.0,
          f"共下发 {n_motion} 条关节指令（应为 37 = 18 + 12 + 7），"
          f"最大跨度 {worst:.1f}°；" + (f"超限：{wide}" if wide else "全部 < 90°"))


def part_f() -> None:
    """F 上机前准备（回正 / 抱持位基准）。这四项是"说开始就能动"的前提。"""

    import importlib
    import math
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    H = importlib.import_module("home_to_carry")

    #: 2026-11-08 08:40 真机只读实测的**偏离**构型（有人拖过手臂）
    DISP_R = [-124.4, 73.4, 36.2, 132.4, -74.1, -5.1, 14.8]
    DISP_L = [100.5, -83.2, -73.0, -132.6, -33.5, 25.9, 10.9]

    def _mk_io(faults=None):
        io_ = pio.build_io("mock", faults=faults or [])
        io_.arm_joints["right_arm"] = [math.radians(v) for v in DISP_R]
        io_.arm_joints["left_arm"] = [math.radians(v) for v in DISP_L]
        io_.joints = io_.arm_joints["right_arm"]
        return io_

    def _run(argv, io_):
        old_argv, old_build = sys.argv, H.build_io
        H.build_io = lambda backend="": io_
        sys.argv = ["home_to_carry.py"] + argv
        try:
            return H.main()
        finally:
            sys.argv, H.build_io = old_argv, old_build

    # 18. --check / --plan 必须零运动
    ok18, det18 = True, []
    for mode in ("--check", "--plan"):
        io_ = _mk_io()
        rc = _run([mode], io_)
        n = io_.calls.count("set_joints")
        det18.append(f"{mode} rc={rc} set_joints={n}")
        ok18 = ok18 and rc == 0 and n == 0
    check(18, "回正脚本 --check/--plan 只读：零 set_joints 调用",
          ok18, "；".join(det18))

    # 19. 从真机那个偏离构型回正 → 到位、分段合规
    io_ = _mk_io()
    rc = _run(["--yes", "--settle", "0"], io_)
    r_end = [round(math.degrees(v), 2) for v in io_.arm_joints.get("right_arm", [])]
    l_end = [round(math.degrees(v), 2) for v in io_.arm_joints.get("left_arm", [])]
    want_r = [round(math.degrees(v), 2) for v in H.ARM_TARGET["right_arm"]]
    want_l = [round(math.degrees(v), 2) for v in H.ARM_TARGET["left_arm"]]
    # 段数：右臂跨度 74.1°、左臂 48.8°，MAX_STEP 默认 10° → 8 段 / 5 段
    n_calls = io_.calls.count("set_joints")
    check(19, "回正：偏离构型 → 双臂到位，8+5=13 段，单段 ≤10°",
          rc == 0 and r_end == want_r and l_end == want_l and n_calls == 13,
          f"rc={rc} 右={r_end} 左={l_end} set_joints={n_calls}（应 13）")

    # 20. 读不到当前关节角 → 绝不动
    io_ = _mk_io([{"method": "get_joint_positions", "behavior": "unknown", "times": 99}])
    rc = _run(["--yes", "--settle", "0"], io_)
    n = io_.calls.count("set_joints")
    check(20, "回正：读不到当前关节角 → 退出码 1 且零运动（不知从哪出发就不许动）",
          rc == 1 and n == 0, f"rc={rc} set_joints={n}")

    # 21. 第一段下发失败 → 立刻停，不重试、不自动归位
    io_ = _mk_io([{"method": "set_joints", "behavior": "fail", "times": 99}])
    rc = _run(["--yes", "--settle", "0"], io_)
    n = io_.calls.count("set_joints")
    check(21, "回正：set_joints 失败 → 退出码 1 且只试了 1 次（不重试、不自动归位）",
          rc == 1 and n == 1, f"rc={rc} set_joints={n}（应 1）")

    # 22. 抱持位基准一致：INIT_POSE 必须与 CARRY_home 同值，入口首段从抱持位出发
    pts = teach_points()
    init_q = [round(float(v), 6) for v in pts["INIT_POSE"]["joint_positions"]]
    carry_q = [round(float(v), 6) for v in pts["CARRY_home"]["joint_positions"]]
    e1 = pts.get("VIA_CARRY_P2_1", {}).get("joint_positions") or []
    span1 = max(abs(float(a) - float(b)) * 180.0 / math.pi
                for a, b in zip(carry_q, e1)) if e1 else 999.0
    check(22, "抱持位基准唯一：INIT_POSE == CARRY_home，且入口首段 ≤15°",
          init_q == carry_q and span1 <= 15.0,
          f"INIT==CARRY {init_q == carry_q}；CARRY→VIA_CARRY_P2_1 最大 {span1:.1f}°")

    # 23. 不给 --yes 必须拒绝执行（零运动）
    io_ = _mk_io()
    rc = _run([], io_)
    n = io_.calls.count("set_joints")
    check(23, "回正：不给 --yes 拒绝执行（rc=2、零 set_joints）",
          rc == 2 and n == 0, f"rc={rc} set_joints={n}")

    # 24. 中途失败后直接重跑能接上（现场不用手工恢复）
    io_a = _mk_io([{"method": "set_joints", "behavior": "fail", "times": 1}])
    rc_a = _run(["--yes", "--settle", "0"], io_a)
    io_b = _mk_io()
    io_b.arm_joints["right_arm"] = list(io_a.arm_joints["right_arm"])
    io_b.arm_joints["left_arm"] = list(io_a.arm_joints["left_arm"])
    rc_b = _run(["--yes", "--settle", "0"], io_b)
    r_end = [round(math.degrees(v), 2) for v in io_b.arm_joints["right_arm"]]
    l_end = [round(math.degrees(v), 2) for v in io_b.arm_joints["left_arm"]]
    check(24, "回正：第 1 段失败后直接重跑，能从当前位接上并精确到位",
          rc_a == 1 and rc_b == 0 and r_end == want_r and l_end == want_l,
          f"首次 rc={rc_a} 重跑 rc={rc_b} 右={r_end}")


def part_g() -> None:
    """G 方向纠正后的静态一致性（旧反向点位不得残留、新链路自洽）。"""
    pts = teach_points()

    # 25. 方向错误的旧点位必须彻底清除
    stale = [k for k in pts if k.startswith(("JNT_ENTRY_", "JNT_P0_P1_"))]
    check(25, "旧的反向点位（JNT_ENTRY_* / JNT_P0_P1_*）已从示教文件彻底清除",
          not stale, f"仍残留 {len(stale)} 个：{sorted(stale)[:6]}")

    # 26. 全仓不得再**引用**已删除的点位或配置
    #     口径：.py 里不许出现旧前缀（那是代码引用）；.json 只查已删除的文件名
    #     —— provenance/note 里的历史说明文字是有意保留的，不算引用。
    bad_ref = []
    # tools/drawer_calib 是离线标定工具，其 STALE_PREFIXES 字面量用于**删除**旧点，
    # 属正当用途，不算"引用已删除点位"。
    skip_dirs = (os.sep + "logs", os.sep + ".git", os.sep + "__pycache__", os.sep + "_urdf",
                 os.sep + "tools" + os.sep + "drawer_calib")
    for dirpath, _dn, fns in os.walk(ROOT):
        if any(seg in dirpath for seg in skip_dirs):
            continue
        for fn in fns:
            if not fn.endswith((".py", ".json")):
                continue
            fp = os.path.join(dirpath, fn)
            if fn in ("verify_experiments.py", "teach_drawer_real.json"):
                continue
            try:
                txt = open(fp, "r", encoding="utf-8").read()
            except (UnicodeDecodeError, OSError):
                continue
            pats = ("exp1a_real_to_p1",)
            if fn.endswith(".py"):
                pats += ("JNT_ENTRY_", "JNT_P0_P1_")
            for pat in pats:
                if pat in txt:
                    bad_ref.append((os.path.relpath(fp, ROOT), pat))
    check(26, "全仓代码与配置中无对已删除点位 / 配置的引用（provenance 说明文字不计）",
          not bad_ref, f"残留：{bad_ref}" if bad_ref else "无残留")

    # 27. 实验 1 与实验 2 的接近段必须走同一条路径（不许各写一份分叉）
    p1 = (load("1")["steps"][2].get("params") or {}).get("points") or []
    e2 = load("2")
    to_p0 = next((s for s in e2["steps"] if s.get("id") == "to_p0"), {})
    p2 = (to_p0.get("params") or {}).get("points") or []
    check(27, "实验 2 的接近段与实验 1 共用同一条 CARRY→P2→P1→P0 路径（37 段）",
          len(p1) == 18 and len(p2) == 37 and p2[:18] == p1,
          f"实验1 leg1={len(p1)} 段，实验2 to_p0={len(p2)} 段，前缀一致={p2[:18] == p1}")

    # 28. 分段步 id 与顺序：必须是 leg1→P2、leg2→P1、leg3→P0
    ids = [s.get("id") for s in load("1a")["steps"] if s.get("stage") == "exp_replay_chain"]
    check(28, "真机版分段顺序：leg1_carry_to_p2 → leg2_p2_to_p1 → leg3_p1_to_p0",
          ids == ["leg1_carry_to_p2", "leg2_p2_to_p1", "leg3_p1_to_p0"], f"实际={ids}")

    # 29. 所有 VIA 中间点：7 关节齐全、在限位内、且明确标了未实测
    LIM = {
        "right_arm_joint1": (-172.1, 172.1), "right_arm_joint2": (-92.1, 92.1),
        "right_arm_joint3": (-167.1, 167.1), "right_arm_joint4": (17.2, 147.1),
        "right_arm_joint5": (-167.1, 167.1), "right_arm_joint6": (-42.1, 47.1),
        "right_arm_joint7": (-88.1, 88.1),
    }
    prob = []
    via = {k: v for k, v in pts.items() if k.startswith("VIA_")}
    for name, p in via.items():
        q = p.get("joint_positions") or []
        if len(q) != 7:
            prob.append((name, f"只有 {len(q)} 关节"))
            continue
        for n, val in zip(p.get("joint_names") or [], q):
            lo, hi = LIM.get(n, (-9e9, 9e9))
            d = float(val) * 180.0 / math.pi
            if not (lo < d < hi):
                prob.append((name, f"{n}={d:.1f}° 越限 [{lo},{hi}]"))
        if p.get("measured") is not False:
            prob.append((name, "measured 未标 False"))
    check(29, f"全部 {len(via)} 个 VIA 中间点：7 关节齐全、限位内、且明确标注未实测",
          not prob, f"问题点：{prob[:5]}" if prob else f"{len(via)} 点全部合规")

    # 30. 实验 1 必须是空爪：全程不下发夹爪指令
    st1 = statuses(run("1"))
    cfg1 = load("1a")
    grip = [s.get("id") for s in cfg1["steps"]
            if s.get("stage") in ("grasp_close", "gripper_close", "set_gripper")]
    check(30, "实验 1 是零接触空爪实验：配置里不含任何夹爪开合步",
          not grip and st1.get("leg3_p1_to_p0") == "ok",
          f"夹爪步={grip} leg3={st1.get('leg3_p1_to_p0')}")

    # ---- 31/32：trace 可观测性（2026-11-08 真机暴露的两个日志缺陷）----
    # 31. 每段都得有实测耗时，链级得有 total_s —— 否则事后只能靠文件 mtime 反推
    r31 = run("1")
    miss_elapsed, bad_total, n_rows, tot = [], [], 0, []
    for t in r31.get("trace") or []:
        if t.get("stage") != "exp_replay_chain":
            continue
        dd = t.get("data") or {}
        rows = dd.get("trace") or []
        n_rows += len(rows)
        miss_elapsed += [f"{t.get('step')}/{r.get('to')}"
                         for r in rows if not isinstance(r.get("elapsed_s"), (int, float))]
        # total_s 必须存在，且不小于各行 elapsed 之和（否则说明计时口径错）
        ts = dd.get("total_s")
        mot = dd.get("motion_s")
        if not isinstance(ts, (int, float)) or ts <= 0:
            bad_total.append((t.get("step"), ts))
        elif isinstance(mot, (int, float)) and mot > ts + 0.2:
            bad_total.append((t.get("step"), f"motion_s {mot} > total_s {ts}"))
        else:
            tot.append(ts)
    check(31, "每条回放行都带实测 elapsed_s，链级带 total_s / motion_s / started_at",
          (not miss_elapsed) and (not bad_total) and n_rows > 0,
          f"共 {n_rows} 行；缺 elapsed 的 {len(miss_elapsed)} 行 {miss_elapsed[:3]}；"
          f"total_s 异常 {bad_total[:2]}；各段 total_s={[round(v, 2) for v in tot]}")

    # 32. 每段首行的 from 必须写成配置声明的名义起点，且标明它没被实测过
    #     （真机 09:58 的 trace 里 leg2/leg3 首行都写成 "START"，与事实不符）
    cfg32 = {s["id"]: (s.get("params") or {}) for s in load("1")["steps"]}
    bad_from = []
    for t in r31.get("trace") or []:
        if t.get("stage") != "exp_replay_chain":
            continue
        sid = t.get("step")
        want = (cfg32.get(sid) or {}).get("start_point")
        rows = (t.get("data") or {}).get("trace") or []
        if not rows:
            continue
        if not want:
            bad_from.append((sid, "配置未声明 start_point"))
            continue
        if rows[0].get("from") != want:
            bad_from.append((sid, f"首行 from={rows[0].get('from')!r} 应为 {want!r}"))
        if rows[0].get("from_measured") is not False:
            bad_from.append((sid, "首行 from_measured 应为 False"))
        for prev, cur in zip(rows, rows[1:]):
            if cur.get("from") != prev.get("to") or cur.get("from_measured") is not True:
                bad_from.append((sid, f"{cur.get('to')} 的 from 链断裂"))
                break
    check(32, "trace 的 from 字段：首行=声明的名义起点并标 from_measured=False，其后逐点衔接",
          not bad_from, f"问题：{bad_from[:4]}" if bad_from else "三段首行起点均与配置一致")


def part_h() -> None:
    """H 实验 2（P0 视觉识别 → 箱内下探夹取 → 带载回程到抱持位）。

    这一组的重点是「不许把说不清当成通过」：
      · 起始位不在 P0 → 零运动拒绝，不是"照着走"；
      · 爪心没盒子 → 必须 NO_TARGET，不能因为 set_gripper 返回 SUCCESS 就算夹到；
      · 读不到夹爪宽度 → 必须 unknown，不能当成功；
      · 视觉不通过 → gate 策略必须停爪；advisory 下必须记 verified=False。
    """
    pts = teach_points()

    # 33. 正常路径全绿
    r33 = run("2v")
    st33 = statuses(r33)
    v33 = step(r33, "verify_exp2")
    bad33 = [k for k, v in st33.items() if v != "ok"]
    check(33, "实验 2 全绿：19 步全部 ok，verify_exp2 判通过（5 个指定点 + 57 段链）",
          r33.get("status") == "ok" and v33.get("status") == "ok" and not bad33
          and (v33.get("data") or {}).get("passed") is True,
          f"status={r33.get('status')} counts={r33.get('counts')} 非 ok 步={bad33} "
          f"verify={(v33.get('message') or '')[:80]}")

    # 34. 动作顺序：必须 下探 → 回撤 → P1 → P2 → 抱持位，不许倒过来
    ids34 = [s.get("id") for s in load("2v")["steps"] if s.get("stage") == "exp_replay_chain"]
    want34 = ["dive_to_grasp", "ascend_to_p0", "leg1_p0_to_p1",
              "leg2_p1_to_p2", "leg3_p2_to_carry"]
    reached = []
    for t in r33.get("trace") or []:
        if t.get("stage") != "exp_replay_chain":
            continue
        for row in (t.get("data") or {}).get("trace") or []:
            if str(row.get("to")) in ("P1_measured", "P2_measured", "CARRY_home"):
                reached.append(str(row.get("to")))
    check(34, "实验 2 动作顺序：下探→回撤→P1→P2→抱持位，端点到达顺序不得倒置",
          ids34 == want34 and reached == ["P1_measured", "P2_measured", "CARRY_home"],
          f"回放步={ids34}；端点到达顺序={reached}")

    # 35. 起始位硬门：人把它放在 P1 却下"开始实验 2" → 必须零运动拒绝
    r35, io35 = run("2v", start="P1_measured", want_io=True)
    g35 = step(r35, "gate_at_p0")
    n35 = io35.calls.count("set_joints")
    check(35, "起始位硬门：起点是 P1 时 gate_at_p0 必须 SAFETY_ABORT，且零 set_joints",
          g35.get("status") == "failed" and str(g35.get("code")) == "SAFETY_ABORT" and n35 == 0
          and not io35.calls.count("set_gripper"),
          f"gate={g35.get('status')}/{g35.get('code')} set_joints={n35}（应 0）"
          f" msg={(g35.get('message') or '')[:80]}")

    # 36. 夹空：爪心没有盒子 → 回读掉到指令值 0.02 → 必须 NO_TARGET
    r36 = run("2v", objects=[])
    g36 = step(r36, "grasp_close")
    check(36, "夹空判据：爪心没盒子时 grasp_close 必须 NO_TARGET（set_gripper 返 SUCCESS 不等于夹到）",
          g36.get("status") == "failed" and str(g36.get("code")) == "NO_TARGET"
          and r36.get("status") == "failed",
          f"grasp_close={g36.get('status')}/{g36.get('code')} "
          f"msg={(g36.get('message') or '')[:90]}")

    # 37. 读不到夹爪宽度 → 第一个要用宽度的步就 unknown，整体立即全停
    #     注意：卡住的不是 grasp_close 而是更早的 open_gripper —— 因为 exp_grip 每次都会回读
    #     宽度，读不到就报"不可判定"。这是**刻意的**：夹爪完全读不到时，
    #     "有没有夹到"这个问题全程无法回答，早停比带着未知往下走到合爪合理。
    r37 = run("2v", faults=[{"method": "get_gripper_width", "behavior": "unknown",
                             "times": 99}])
    g37 = step(r37, "open_gripper")
    check(37, "夹爪宽度读不到 → open_gripper 即 unknown，整体立即全停（不重试、不归位）",
          g37.get("status") == "unknown" and r37.get("status") == "unknown"
          and step(r37, "grasp_close").get("status") == "skipped"
          and r37.get("aborted_without_safety") is True,
          f"open_gripper={g37.get('status')} grasp_close="
          f"{step(r37, 'grasp_close').get('status')} 整体={r37.get('status')} "
          f"aborted_without_safety={r37.get('aborted_without_safety')}")

    # 38. 视觉 gate：判定"没盒子"时必须停爪，且一条运动/夹爪指令都不发
    cfg38 = load("2v")
    for s in cfg38["steps"]:
        if s.get("id") == "vision_check":
            s["params"]["policy"] = "gate"
            s["params"]["static_verdict"] = {
                "ok": True, "box_present": False, "graspable": False,
                "confidence": 0.5, "reason": "回归用：故意判不通过"}
    r38, io38 = run("2v", cfg=cfg38, want_io=True)
    v38 = step(r38, "vision_check")
    n38 = io38.calls.count("set_joints") + io38.calls.count("set_gripper")
    check(38, "视觉 gate 策略：判定不通过 → vision_check 停爪，零 set_joints / 零 set_gripper",
          v38.get("status") == "failed" and n38 == 0,
          f"vision_check={v38.get('status')} 运动/夹爪指令数={n38}（应 0） "
          f"msg={(v38.get('message') or '')[:80]}")

    # 39. 视觉 advisory（无 VLM Key）：不拦路，但必须记 verified=False
    cfg39 = load("2v")
    for s in cfg39["steps"]:
        if s.get("id") == "vision_check":
            s["params"].pop("static_verdict", None)
    r39 = run("2v", cfg=cfg39)
    vd39 = (step(r39, "vision_check").get("data") or {})
    inner39 = vd39.get("verdict") or {}
    check(39, "视觉 advisory（未配 VLM）：流程照走，但结论必须 verified=False，不许冒充视觉确认",
          r39.get("status") == "ok" and vd39.get("verified") is False
          and inner39.get("ok") is False and "未配置" in str(inner39.get("message", "")),
          f"整体={r39.get('status')} step.verified={vd39.get('verified')} "
          f"verdict.ok={inner39.get('ok')} msg={str(inner39.get('message'))[:60]}")

    # 40. 回程必须严格是实验 1 那条链的倒序（同一条路，不许另写一份）
    e1 = load("1a")
    legs1 = {s["id"]: (s.get("params") or {}).get("points") or [] for s in e1["steps"]}
    cfg2 = {s["id"]: (s.get("params") or {}).get("points") or [] for s in load("2v")["steps"]}
    pair = []
    for back, fwd in (("leg3_p2_to_carry", "leg1_carry_to_p2"),
                      ("leg2_p1_to_p2", "leg2_p2_to_p1"),
                      ("leg1_p0_to_p1", "leg3_p1_to_p0")):
        a, b = cfg2[back], list(reversed(legs1[fwd]))[1:]
        pair.append((back, len(a), len(b), a[:-1] == b))
    check(40, "回程 = 实验 1 的严格倒序（18/12/7 段，逐点相等；只把末点换成下一段起点）",
          all(x[3] and x[1] == x[2] + 1 for x in pair),
          "；".join(f"{n}: {la} vs {lb} 一致={okk}" for n, la, lb, okk in pair))

    # 41. 下探点位必须是「7 关节 + 限位内 + 标未实测 + z 单调下降」
    LIM = {
        "right_arm_joint1": (-172.1, 172.1), "right_arm_joint2": (-92.1, 92.1),
        "right_arm_joint3": (-167.1, 167.1), "right_arm_joint4": (17.2, 147.1),
        "right_arm_joint5": (-167.1, 167.1), "right_arm_joint6": (-42.1, 47.1),
        "right_arm_joint7": (-88.1, 88.1),
    }
    prob41, zs, spans41 = [], [], []
    prev_q = [float(v) for v in pts["P0_measured"]["joint_positions"]]
    for i in range(1, 11):
        name = f"VIA_P0_DIVE_{i}"
        p = pts.get(name)
        if not p:
            prob41.append((name, "缺失"))
            continue
        q = [float(v) for v in p["joint_positions"]]
        if len(q) != 7:
            prob41.append((name, f"{len(q)} 关节"))
            continue
        if p.get("measured") is not False:
            prob41.append((name, "measured 未标 False"))
        for n, val in zip(p.get("joint_names") or [], q):
            lo, hi = LIM.get(n, (-9e9, 9e9))
            d = val * 180.0 / math.pi
            if not (lo < d < hi):
                prob41.append((name, f"{n}={d:.1f}° 越限"))
        spans41.append(max(abs(a - b) * 180.0 / math.pi for a, b in zip(q, prev_q)))
        prev_q = q
        zs.append(float(p["pose"][2]))
    mono = all(a > b for a, b in zip(zs, zs[1:]))
    box_z = float(pio.SCENES["drawer_p0"]()[0]["ee_pose"][2])
    p0_z = float(pts["P0_measured"]["pose"][2])
    depth = p0_z - zs[-1]
    check(41, "下探 10 点：7 关节齐全、限位内、标未实测、z 单调下降、总深 42 cm 且落点与 mock 盒位一致（±1 cm）",
          not prob41 and mono and abs(zs[-1] - box_z) < 0.01
          and abs(depth - 0.42) < 0.01 and max(spans41) < 15.0,
          f"问题={prob41[:4] or '无'}；z={[round(z, 3) for z in zs]}；"
          f"P0 z={p0_z:.4f} 落点 z={zs[-1]:.4f} 总深={depth * 100:.1f}cm；"
          f"最大单段={max(spans41) if spans41 else -1:.2f}°；mock 盒 z={box_z:.4f}")

    # 42. 只动右臂：无底盘指令、无左臂指令、正好 57 段关节回放
    r42, io42 = run("2v", want_io=True)
    base42 = [c for c in io42.calls
              if c in ("turn", "move_forward", "move_left", "move_to_pose", "set_base_pose")]
    n42 = io42.calls.count("set_joints")
    left42 = "left_arm" in io42.arm_joints
    check(42, "实验 2 只动右臂：零底盘指令、左臂从未被写入、正好 57 段关节回放",
          not base42 and not left42 and n42 == 57,
          f"底盘指令={base42 or '无'} 左臂被写={left42} set_joints={n42}（应 57）")


def main() -> int:
    print("=" * 76)
    print("抽屉三实验 · 二次验证（全程 mock，零硬件指令）")
    print("=" * 76)
    for fn, title in ((part_a, "A 正常路径"), (part_b, "B 安全拦截"),
                      (part_c, "C 三终态语义"), (part_d, "D 契约一致性"),
                      (part_e, "E 真机可执行性"), (part_f, "F 上机前准备"),
                      (part_g, "G 方向纠正一致性"),
                      (part_h, "H 实验 2（视觉夹取 + 带载回程）")):
        print(f"\n---- {title} ----")
        fn()
    bad = [x for x in RESULTS if not x[2]]
    print("\n" + "=" * 76)
    print(f"合计 {len(RESULTS)} 项，通过 {len(RESULTS) - len(bad)} 项，失败 {len(bad)} 项")
    if bad:
        print("失败项：", [(x[0], x[1]) for x in bad])
    print("=" * 76)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
