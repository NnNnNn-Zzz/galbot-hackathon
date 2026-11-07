"""实验 1/2/3 的通过判据 —— 单一事实来源。

设计原则
--------
1. **判据与执行分离**：这里不下发任何动作，只回答「这次跑完算不算过」。
2. **失败要给出下一步**：每个不通过都带 `next_action`，现场照做即可，
   不用临场推理。调参顺序固定为：先重摆盒 -> 再调 z -> 最后动 x/y（每次 <=1cm）。
3. **区分「链路」与「抓到」**：实验 2/3 的第一目标是链路跑通，
   没抓住但流程完整走完，也算链路通过（`link_ok=True, grabbed=False`）。

判据表（与 docs/real_validation_plan.md 对齐）
---------------------------------------------
实验 1（空桌）：preflight + grasp 两步都 ok；grasp 的 picked 为 False/None 属预期。
                任何 unknown 都算不通过（说明结构或接口有问题）。
实验 2（单盒）：preflight + grasp 都 ok；picked 不作硬要求。
实验 3（全流程）：7 步全 ok；且必须亲眼看到 must_run 归位被执行。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

# 实验编号 -> 说明（打印给现场看）
EXPERIMENT_TITLES: Dict[int, str] = {
    0: "实验 0 冒烟：接口连通性（无盒，5 分钟）",
    1: "实验 1 空桌盲抓：验证抓取轨迹安全（桌面不放盒）",
    2: "实验 2 单盒盲抓：验证能对准并夹住真实药盒",
    3: "实验 3 全流程低速：7 步完整链路 + 归位兜底",
}

# 实验 1/2 只跑这两步
GRASP_ONLY_STEPS: List[str] = ["preflight", "grasp"]


def _trace_by_step(result: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """把 trace 整理成 {step_id: entry}，方便按步查。"""
    out: Dict[str, Dict[str, Any]] = {}
    for entry in result.get("trace") or []:
        step = entry.get("step")
        if step:
            out[step] = entry
    return out


def _status_of(trace: Dict[str, Dict[str, Any]], step: str) -> Optional[str]:
    entry = trace.get(step)
    return None if entry is None else entry.get("status")


def judge_grasp_only(result: Dict[str, Any], experiment: int,
                     expect_pick: Optional[bool] = None) -> Dict[str, Any]:
    """实验 1 / 实验 2 的判据：只跑 preflight + grasp。

    参数
    ----
    result : run_pipeline 的返回值（键是 status，不是 success）
    experiment : 1 或 2，只影响文案和 picked 的宽松度
    expect_pick : 实验 2 想要求「必须抓到」时传 True；默认 None 表示不作硬要求

    返回
    ----
    {
      "passed": bool,          # 这次算不算过
      "link_ok": bool,         # 链路（动作序列）是否走完
      "grabbed": bool|None,    # 是否确证抓到（None = 无反馈通道，说不清）
      "reasons": [...],        # 不通过的具体原因
      "next_action": str,      # 现场下一步做什么
    }
    """
    trace = _trace_by_step(result)
    status = result.get("status")
    reasons: List[str] = []

    s0 = _status_of(trace, "preflight")
    s1 = _status_of(trace, "grasp")

    # --- preflight：非致命步，失败也能继续，但必须如实记下 ---
    if s0 is None:
        reasons.append("preflight 没有出现在 trace 里 —— 配置或注册可能有问题")
    elif s0 == "unknown":
        reasons.append("preflight 返回 unknown —— 连里程计都读不到，先查 SDK 连接")

    # --- grasp：核心判据 ---
    link_ok = s1 == "ok"
    if s1 is None:
        reasons.append("grasp 没有出现在 trace 里 —— 是否被前序步骤跳过了？")
    elif s1 == "skipped":
        reasons.append("grasp 被跳过（前序步骤失败）—— 看 preflight 的 message")
    elif s1 == "unknown":
        reasons.append("grasp 返回 unknown —— 机器人不知道自己在哪，"
                       "按团队规则立即全停、不要重试、不要归位")
    elif s1 == "failed":
        reasons.append(f"grasp 失败：{trace['grasp'].get('message')}")

    # --- 整体 unknown 一票否决 ---
    if status == "unknown":
        reasons.append("整场被判 unknown —— 这是最严重的结果，先停下排查再谈调参")

    # --- picked 的解释 ---
    picked: Optional[bool] = result.get("picked")
    if picked is None and "picked" not in result:
        # run_pipeline 不回 picked，从 ctx 里没传出来时按未知处理
        picked = None

    if experiment == 1:
        # 空桌：抓到反而是错的（说明桌上还有东西，或假阳性）
        if picked is True:
            reasons.append("空桌实验却报告 picked=True —— 桌上是不是还有东西没拿走？")
        passed = not reasons
    else:
        if expect_pick is True and picked is not True:
            reasons.append(f"要求必须抓到，但 picked={picked}")
            if picked is None:
                reasons.append("注意：picked=None 是本后端读不到夹爪反馈的正常表现，"
                               "不能当作抓到了；请靠目视确认")
        passed = not reasons

    return {
        "passed": passed,
        "link_ok": link_ok,
        "grabbed": picked,
        "reasons": reasons,
        "next_action": _next_action(experiment, reasons, picked),
    }


def judge_full_pipeline(result: Dict[str, Any]) -> Dict[str, Any]:
    """实验 3 的判据：7 步全流程。

    额外要求：如果中途 failed，必须观察到 must_run 的 home 步骤**真的被执行**，
    而不是被 skipped —— 这是「失败也要归位」这条安全约定的现场验证点。
    """
    trace = _trace_by_step(result)
    status = result.get("status")
    reasons: List[str] = []

    # 注意：这里用的是配置里的 step id，不是 stage 名。
    # turn_bin 的 stage 是 "turn"，id 却是 "turn_bin" —— 别混。
    expected = ["preflight", "grasp", "announce", "shift_left", "turn_bin",
                "drop", "home"]
    missing = [s for s in expected if s not in trace]
    if missing:
        reasons.append(f"trace 里缺少步骤：{missing} —— 配置被截断或注册失败")

    # 逐步状态
    bad: List[str] = []
    for step in expected:
        st = _status_of(trace, step)
        if st in ("failed", "unknown"):
            entry = trace[step]
            bad.append(f"{step}={st}（{entry.get('message')}）")
    if bad:
        reasons.append("步骤未通过：" + "；".join(bad))

    if status == "unknown":
        reasons.append("整场 unknown —— 已按规则全停，不要再发动作，人工接管")

    # failed 情况下必须看到 home 被执行（must_run 语义）
    home_status = _status_of(trace, "home")
    if status == "failed" and home_status == "skipped":
        reasons.append("严重：failed 之后 home 被跳过 —— must_run 语义失效，"
                       "机器人可能停在半空带盒状态，需人工检查")

    passed = not reasons
    return {
        "passed": passed,
        "link_ok": all(_status_of(trace, s) == "ok" for s in expected if s != "home"),
        "grabbed": result.get("picked"),
        "home_executed": home_status is not None and home_status != "skipped",
        "reasons": reasons,
        "next_action": _next_action(3, reasons, result.get("picked")),
    }


def _next_action(experiment: int, reasons: List[str], picked: Optional[bool]) -> str:
    """把「为什么不通过」翻译成「现场下一步做什么」。"""
    if not reasons:
        if experiment == 1:
            return "通过。下一步：进实验 2，按胶带把盒子摆到 x=0.40 / y=-0.10。"
        if experiment == 2:
            if picked is True:
                return "通过且确证抓到。下一步：进实验 3，先示教桶位（teach_drop_pose.py）。"
            return ("链路通过。若目视没抓住：先确认盒子是否在胶带交点上，"
                    "再按「重摆盒 -> 调 z -> 动 x/y」的顺序微调。")
        return "全流程通过。可以进 VLM 版（识别日期）了。"

    joined = " ".join(reasons)

    if "unknown" in joined:
        return ("停止操作。unknown 表示机器人无法确定自身状态，按团队规则："
                "不重试、不归位、不发任何动作，人工把臂摆回安全位后再排查。")

    if "碰撞" in joined or "急停" in joined:
        return "立即拍急停，不要再重试同一位姿。抬高 GRASP_POSE 的 z 0.01 后重新示教。"

    # 空桌却抓到东西：这不是调参问题，是环境问题，别去动坐标
    if experiment == 1 and "空桌实验却报告 picked=True" in joined:
        return ("先别调任何坐标。清空桌面（含盒子、纸片、边缘的卷尺），"
                "再跑一次同样的实验 1。若清空后仍报 picked=True，"
                "说明是本后端 is_holding() 的假阳性，记录后按 picked 不可信处理。")

    if experiment in (1, 2):
        return ("按固定顺序排查：① 先把盒子重新摆到胶带交点（xy 最容易错）；"
                "② 还不行为把 GRASP_POSE z 每次 +0.01（最高别超过 0.80）；"
                "③ 最后才动 x/y，每次 <=1cm，改完必须重跑实验 1 确认轨迹。")

    return ("先看 failed_step 是哪一步：抓取段失败 -> 退回实验 2 重标；"
            "投放段失败 -> 桶位 DROP_POSE 未标定，先跑 teach_drop_pose.py；"
            "移动段失败 -> 查底盘侧移是否被支持。")
