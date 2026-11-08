# -*- coding: utf-8 -*-
"""实验 2 上机准备的**只读**体检 —— 一条运动指令都不发。

为什么实验 2 比实验 1 更需要这个脚本
----------------------------------
实验 1 的失败模式是"臂摆不过去"；实验 2 多了四种会直接损失硬件/时间的情况：

  ① **起点不对**：实验 2 的起点是 P0，而实验 1 结束后手臂停在 P0、但人随时可能拖走它
     （2026-11-08 实测：08:19 在位 → 08:40 偏 74°）。起点不对时 gate_at_p0 会零运动退出，
     但那时你已经占了真机窗口 —— 不如上机前先量出来。
  ② **夹爪读不到宽度**：实验 2 的"夹住了没有"完全靠回读宽度判定。这个通路断了，
     整条链在第一步 open_gripper 就会 unknown 全停。
  ③ **下探深度是配置值**：42 cm 是估的（内腔底面 + 药盒高度都没量过）。
     下探过深会顶内腔底面，过浅夹空。脚本把当前配置的落点 z 与"桌面高度"对照打出来。
  ④ **相机通路**：视觉段虽然默认 advisory 不拦路，但要提前知道能不能拍到，
     否则真机上才发现"根本没图"，整段视觉就是空的。

本脚本只调**只读接口**（get_joint_positions / get_ee_pose / get_odom /
get_gripper_width / is_holding / get_rgb_data），**不发** set_joints / set_gripper /
move_* / turn / speak。相机帧默认不拉（--camera 才拉一帧）。

用法（HPU 上，仓库根目录）：
    source scripts/env_hpu.sh
    python3 scripts/prep_exp2_readonly.py                # 只读查 + 打印上机方案
    python3 scripts/prep_exp2_readonly.py --json out.json
    python3 scripts/prep_exp2_readonly.py --skip-hw       # 完全不连 SDK，只查文件
    python3 scripts/prep_exp2_readonly.py --camera        # 额外拉一帧验证相机通路

退出码：0 = 全 PASS（可能有 WARN）；1 = 有 FAIL，不许上机。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

CONFIG = "exp2_visual_grasp_return"
TEACH = "pipeline/configs/teach_drawer_real.json"
DIVE_N = 10

rows = []


def add(level, key, msg, data=None):
    rows.append({"level": level, "item": key, "message": msg,
                 "data": data if data is not None else {}})
    print(f"[{level:<4}] {key:<26} {msg}", flush=True)


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- A 代码与数据
def check_files():
    from scenes.scene_medicine_check.pipeline import registry, runner
    from scenes.scene_medicine_check.pipeline.teach import load_teach, resolve_teach_path

    try:
        cfg = runner.load_config(CONFIG)
    except Exception as exc:  # noqa: BLE001
        add("FAIL", "配置可读", f"读不到 {CONFIG}：{exc}")
        return None
    steps = cfg["steps"]
    add("PASS", "配置可读", f"{CONFIG} v{cfg.get('version')}，{len(steps)} 步："
                            f"{' → '.join(s['id'] for s in steps)}")

    registry.load_all()
    have = set(registry.list_stages())
    missing = [s["stage"] for s in steps if s["stage"] not in have]
    add("FAIL" if missing else "PASS", "stage 已注册",
        f"缺 {missing}（代码版本不对？）" if missing
        else f"{len(steps)} 步的 stage 全部已注册（含 exp_gate_start / exp_vision_check / exp_grip）")

    real = resolve_teach_path(TEACH)
    teach = load_teach(TEACH)
    if not teach:
        add("FAIL", "示教文件", f"读不到 {real}")
        return cfg
    pts = teach.get("points") or {}

    # 1) 配置引用的点必须全在
    need = []
    for s in steps:
        need += list((s.get("params") or {}).get("points") or [])
    lack = sorted({p for p in need if p not in pts})
    add("FAIL" if lack else "PASS", "链条点位齐备",
        f"缺点 {lack[:6]}{'…' if len(lack) > 6 else ''}" if lack
        else f"配置引用 {len(set(need))} 个点，全部存在（示教文件共 {len(pts)} 点）")

    # 2) 下探点位数量
    dives = [f"VIA_P0_DIVE_{i}" for i in range(1, DIVE_N + 1)]
    bad = [d for d in dives if d not in pts]
    add("FAIL" if bad else "PASS", "下探点位数量",
        f"缺 {bad}" if bad else f"VIA_P0_DIVE_1..{DIVE_N} 共 {len(dives)} 点齐全")

    # 3) 下探路径几何：z 单调、总深、落点
    if not bad:
        zs = [float(pts[d]["pose"][2]) for d in dives]
        p0z = float(pts["P0_measured"]["pose"][2])
        depth = p0z - zs[-1]
        mono = all(a > b for a, b in zip(zs, zs[1:]))
        table_z = 0.72
        add("PASS" if mono else "FAIL", "下探 z 单调",
            f"P0 z={p0z:.4f} → 落点 z={zs[-1]:.4f}，总深 {depth * 100:.1f} cm"
            + ("" if mono else "；❌ 出现不单调，点位生成错了"))
        # 与桌面高对照：落点必须在桌面之上，否则等于把爪子捅到桌子里
        margin_cm = (zs[-1] - table_z) * 100
        lvl = "FAIL" if margin_cm <= 0 else ("WARN" if margin_cm < 5.0 else "PASS")
        add(lvl, "落点 vs 桌面高度",
            f"落点 z={zs[-1]:.4f}，桌面 z={table_z:.2f} → 高出桌面 {margin_cm:.1f} cm"
            + ("；❌ 落点在桌面以下，必须减小 --depth 重生成点位" if margin_cm <= 0 else "")
            + ("；⚠️ 余量偏小，现场量内腔底面后再定" if 0 < margin_cm < 5 else ""))

    # 4) 夹爪判据参数：合爪指令必须明显小于期望咬合宽度，否则回读无区分度
    g = next((s for s in steps if s.get("id") == "grasp_close"), {})
    gp = g.get("params") or {}
    cmd = gp.get("width_m")
    emin, emax = gp.get("expect_min_m"), gp.get("expect_max_m")
    floor = gp.get("hard_floor_m")
    ok_cfg = (cmd is not None and emin and emax and floor
              and float(cmd) < float(floor) < float(emin) < float(emax))
    add("PASS" if ok_cfg else "FAIL", "夹爪判据参数",
        f"合爪命令 {cmd} m < 硬下限 {floor} < 期望区间 [{emin}, {emax}]"
        if ok_cfg else
        f"参数不成序（cmd={cmd} floor={floor} expect=[{emin},{emax}]）—— "
        f"命令宽度必须小于硬下限，回读宽度才有区分度")

    # 5) 示教来源与标定
    prov = teach.get("provenance") or {}
    n_meas = sum(1 for k in ("P0_measured", "P1_measured", "P2_measured") if k in prov)
    add("PASS" if n_meas == 3 else "FAIL", "示教来源",
        "P0/P1/P2 三个实测点 provenance 齐全" if n_meas == 3
        else f"只找到 {n_meas}/3 个实测点 provenance，可能不是真机回读版")
    add("PASS" if teach.get("calibration") else "WARN", "末端标定",
        "标定段存在（姿态经 R_corr 修正后 ~2°）" if teach.get("calibration")
        else "示教文件没有 calibration 段 —— 末端位姿只能当估计值")

    return cfg


def check_static_margins():
    """离线核对 P0 + 下探 10 点的静态限位余量（下探段是实验 2 最贴限位的地方）。"""
    from scenes.scene_medicine_check.pipeline.stages.s11_exp_stages import _margin_deg
    from scenes.scene_medicine_check.pipeline.teach import load_teach

    pts = load_teach(TEACH).get("points") or {}
    names = ["P0_measured"] + [f"VIA_P0_DIVE_{i}" for i in range(1, DIVE_N + 1)]
    tight, fail = [], []
    for n in names:
        rec = pts.get(n) or {}
        ms = {}
        for jn, v in zip(rec.get("joint_names") or [], rec.get("joint_positions") or []):
            m = _margin_deg(str(jn), float(v))
            if m is not None:
                ms[str(jn)] = round(m, 2)
        if not ms:
            fail.append((n, "无关节角"))
            continue
        j = min(ms, key=lambda k: ms[k])
        tight.append((n, j, ms[j]))
        if ms[j] < 2.0:
            fail.append((n, f"{j} 余量 {ms[j]:.1f}°"))
    if not tight:
        add("FAIL", "静态限位余量", "一个点都算不出余量")
        return
    worst = min(tight, key=lambda t: t[2])
    lvl = "FAIL" if worst[2] < 2.0 else ("WARN" if worst[2] < 5.0 else "PASS")
    add(lvl, "静态限位余量",
        f"下探链最紧 {worst[0]} 的 {worst[1]} 只剩 {worst[2]:.1f}°"
        f"（P0 的 j7 固有 4.5° 会打 WARNING，属预期）", {"worst": worst, "all": tight})
    if fail:
        add("FAIL", "贴限位点位", f"{fail}")


# ---------------------------------------------------------------- B 真机只读
def check_hw():
    from scenes.scene_medicine_check.pipeline.io import build_io
    from scenes.scene_medicine_check.pipeline.stages.s11_exp_stages import _margin_deg
    from scenes.scene_medicine_check.pipeline.teach import load_point

    io = build_io("wrapper")

    # 2.1 关节角
    try:
        ret = io.get_joint_positions(joint_groups=["right_arm"], joint_names=[])
    except Exception as exc:  # noqa: BLE001
        add("FAIL", "读右臂关节角", f"抛异常：{exc}")
        return
    names = list(ret.get("joint_names") or [f"right_arm_joint{i}" for i in range(1, 8)])
    pos = ret.get("joint_positions") or []
    if not ret.get("success") or len(pos) != 7:
        add("FAIL", "读右臂关节角",
            f"读不到 7 个：success={ret.get('success')} len={len(pos)} "
            f"msg={ret.get('message')}（第一条指令就会 UNDETERMINED 全停）")
        return
    add("PASS", "读右臂关节角", f"7 轴已读：{[round(math.degrees(float(v)), 1) for v in pos]}")

    # 2.2 ★ 离 P0 多远 —— 实验 2 能不能开跑的唯一硬前提
    p0 = [float(v) for v in load_point(TEACH, "P0_measured")["joint_positions"]]
    d = [abs(float(a) - b) * 180.0 / math.pi for a, b in zip(pos, p0)]
    dmax = max(d)
    imax = d.index(dmax)
    jn = names[imax] if imax < len(names) else f"j{imax + 1}"
    lvl = "PASS" if dmax <= 2.0 else ("WARN" if dmax <= 10.0 else "FAIL")
    add(lvl, "离 P0 的距离",
        f"最大偏差 {dmax:.2f}°（{jn}，容差 2.0°）"
        + ("" if dmax <= 2.0 else
           "；⚠️ 不在 P0 —— 先 home_to_carry --yes 回正，再跑实验 1 到达 P0"
           if dmax <= 10.0 else
           "；❌ 完全不在 P0，直接跑实验 2 会被 gate_at_p0 零运动拒绝"),
        {"deltas_deg": [round(x, 2) for x in d], "max": round(dmax, 3)})

    # 2.3 限位余量
    ms = {}
    for n, v in zip(names, pos):
        m = _margin_deg(n, float(v))
        if m is not None:
            ms[n] = round(m, 2)
    if ms:
        j = min(ms, key=lambda k: ms[k])
        lvl = "FAIL" if ms[j] < 2.0 else ("WARN" if ms[j] < 5.0 else "PASS")
        add(lvl, "当前限位余量", f"最紧 {j} = {ms[j]:.1f}°"
            + ("（已在 failed 线内，先让臂离开限位）" if ms[j] < 2.0 else ""), ms)

    # 2.4 ★ 夹爪宽度 —— 实验 2 的判据全靠它
    gw = io.get_gripper_width(side="right")
    if gw.get("success") and gw.get("width") is not None:
        w = float(gw["width"])
        add("PASS", "读右爪宽度",
            f"{w * 1000:.1f} mm（实验 2 靠回读宽度判『夹住了没有』，此路必须通；"
            f"期望合爪后落在 60–90 mm）")
    else:
        add("FAIL", "读右爪宽度",
            f"读不到：{gw.get('message')} —— 实验 2 会在 open_gripper 步 unknown 全停，"
            f"先修 get_gripper_state 通路再上机")

    # 2.5 末端位姿（与 P0 的标定口径交叉校验）
    ee = io.get_ee_pose(chain="right_arm")
    if ee.get("success") and len(ee.get("pose") or []) == 7:
        p = ee["pose"]
        want = load_point(TEACH, "P0_measured").get("pose") or []
        gap = (math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(p[:3], want[:3])))
               if len(want) == 7 else None)
        add("PASS", "读末端位姿",
            f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}"
            + (f"；与 P0 标定口径差 {gap * 1000:.0f} mm（FK 残差 ~17 mm，几十 mm 内属正常）"
               if gap is not None else ""),
            {"pose": p, "p0_pose": want})
    else:
        add("WARN", "读末端位姿", f"读不到：{ee.get('message')}（不影响判通过，但少一路交叉校验）")

    # 2.6 里程计
    od = io.get_odom()
    add("PASS" if od.get("success") else "WARN", "读里程计",
        f"position={od.get('odom', {}).get('position')}" if od.get("success")
        else f"读不到：{od.get('message')}（preflight 记 failed，非致命）")

    # 2.7 is_holding（真机大概率没有；实验 2 不用它，靠宽度）
    try:
        h = io.is_holding()
        add("PASS", "is_holding 接口", f"存在，返回 {h}")
    except Exception as exc:  # noqa: BLE001
        add("WARN", "is_holding 接口",
            f"不可用（{type(exc).__name__}: {str(exc)[:60]}）—— 实验 2 不依赖它，靠回读宽度")


def check_camera():
    """可选：拉一帧验证相机通路。只读传感器，不改机器人状态（HPU 上落一张图）。"""
    from scenes.scene_medicine_check.pipeline.io import build_io

    io = build_io("wrapper")
    try:
        shot = io.capture_image(tag="prep_exp2", camera="head_left")
    except Exception as exc:  # noqa: BLE001
        shot = {"success": False, "message": f"抛异常：{exc}"}
    if shot.get("success"):
        add("PASS", "相机通路",
            f"head_left 出图：{shot.get('path')}（frame={shot.get('frame_id')}）")
    else:
        add("WARN", "相机通路",
            f"拍不到：{shot.get('message')} —— 视觉段会记 verified=False（advisory 不拦路），"
            f"但整段视觉就是空的")


# ---------------------------------------------------------------- C 上机方案
def print_plan(cfg):
    teach_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..",
                     "scenes/scene_medicine_check", TEACH))
    print("\n" + "=" * 78, flush=True)
    print("实验 2 上机方案（以下命令需要你字面授权后才会执行）", flush=True)
    print("=" * 78, flush=True)
    print("0) 现场确认：工作区净空、急停在手边；确认队友已释放遥操作占用。", flush=True)
    print("   ⚠️ 手臂随时可能被拖走（08:19 在位 → 08:40 偏 74°）。每步都要重读。", flush=True)
    print("", flush=True)
    print("1) 只读：本脚本（零指令）", flush=True)
    print("   MED_IO=wrapper python3 scripts/prep_exp2_readonly.py", flush=True)
    print("", flush=True)
    print("2) ★若不在 P0：先回正，再爬一次实验 1（两步都必须是真机授权）", flush=True)
    print("   MED_IO=wrapper python3 scripts/home_to_carry.py --yes", flush=True)
    print("   MED_IO=wrapper python3 scripts/run_exp.py --config "
          "scenes/scene_medicine_check/pipeline/configs/exp1_real_carry_p2_p1_p0.json", flush=True)
    print("   判据：回正退出码 0 且双臂偏差 ≤3°；实验 1 总状态 ok 且停在 P0。", flush=True)
    print("", flush=True)
    print("3) ★实验 2 本体：P0 → 视觉 → 下探 42cm → 合爪 → 回撤 P0 → P1 → P2 → 抱持位", flush=True)
    print("   MED_IO=wrapper python3 scripts/run_exp.py --config "
          "scenes/scene_medicine_check/pipeline/configs/exp2_visual_grasp_return.json", flush=True)
    print("   预期：19 步全 ok；grasp_close 回读 60–90 mm；hold_at_p0 / hold_at_carry 同区间；", flush=True)
    print("         最后停在抱持位不动（不自动张爪、不自动投放）。", flush=True)
    print("", flush=True)
    print("⚠️ 真机模式要求交互 TTY（sys.stdin.isatty()）。SSH 必须加 -tt 并输入 START。", flush=True)
    print("⚠️ 停下来先看 logs/ 里的 trace，不要凭屏幕滚动日志下结论。", flush=True)
    print("⚠️ 下探深度 42 cm、抽屉外壳顶高、药盒高度**都还没实测** —— 首次跑务必手放急停。", flush=True)
    print(f"⚠️ 示教文件：{teach_path}", flush=True)
    if os.path.exists(teach_path):
        print(f"   md5 = {md5(teach_path)}", flush=True)
        add("PASS", "上机方案", "已打印；示教文件 md5 一并给出，便于和 HPU 端核对",
            {"teach_md5": md5(teach_path)})
    else:
        add("FAIL", "上机方案", f"示教文件不存在：{teach_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description="实验 2 上机前只读体检")
    ap.add_argument("--skip-hw", action="store_true", help="只查文件与配置，完全不碰 SDK")
    ap.add_argument("--camera", action="store_true", help="额外拉一帧验证相机通路（只读传感器）")
    ap.add_argument("--json", default="", help="结果另存 JSON 路径")
    ap.add_argument("--quiet-plan", action="store_true", help="不打印上机方案")
    args = ap.parse_args()

    print("=" * 78, flush=True)
    print("实验 2 上机前只读体检（本脚本不发任何运动/夹爪指令）", flush=True)
    print(f"cwd = {os.getcwd()}", flush=True)
    print("=" * 78, flush=True)

    print("\n---- A 代码与数据 ----", flush=True)
    cfg = check_files()
    check_static_margins()

    if args.skip_hw:
        print("\n---- B 真机只读：已跳过（--skip-hw）----", flush=True)
        add("WARN", "真机只读", "按 --skip-hw 跳过，未验证 SDK 通路")
    else:
        print("\n---- B 真机只读（只读接口，不动臂）----", flush=True)
        try:
            check_hw()
        except Exception as exc:  # noqa: BLE001
            add("FAIL", "真机只读", f"整体失败：{type(exc).__name__}: {exc}")
        if args.camera:
            try:
                check_camera()
            except Exception as exc:  # noqa: BLE001
                add("WARN", "相机通路", f"整体失败：{type(exc).__name__}: {exc}")
        else:
            add("WARN", "相机通路", "未验证（加 --camera 才会拉一帧）")
        if os.name == "nt":
            add("WARN", "运行位置",
                "当前在 Windows 本地跑，缺 aarch64 的 galbot_sdk 是预期的。"
                "真机只读体检必须在 HPU 上跑："
                "ssh galbot@192.168.166.7 → source scripts/env_hpu.sh → 再执行本脚本")

    if cfg and not args.quiet_plan:
        print_plan(cfg)

    n_fail = sum(1 for r in rows if r["level"] == "FAIL")
    n_warn = sum(1 for r in rows if r["level"] == "WARN")
    n_pass = sum(1 for r in rows if r["level"] == "PASS")
    print("\n" + "=" * 78, flush=True)
    print(f"合计 {len(rows)} 项：PASS {n_pass} / WARN {n_warn} / FAIL {n_fail}", flush=True)
    print("结论：" + ("**不许上机**，先修 FAIL 项" if n_fail
                     else "可以进入授权流程（WARN 项要现场确认）"), flush=True)
    print("=" * 78, flush=True)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"rows": rows, "pass": n_pass, "warn": n_warn, "fail": n_fail},
                      fh, ensure_ascii=False, indent=2)
        print(f"已写 {args.json}", flush=True)

    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
