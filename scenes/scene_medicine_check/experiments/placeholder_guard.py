"""投放位姿占位值检测 —— 实验 3 的安全闸。

背景
----
tuning.py 里 PRE_DROP_POSE / DROP_POSE 目前是照旧流程抄来的占位值，
注释标着 TODO(标定)。左移 0.5 m + 右转 30 度之后，桶的真实位置未知。
若不做检查直接跑实验 3，手臂会朝一个没人验证过的坐标伸手，
最坏情况是撞到桶壁或桌面边缘。

这个模块只做一件事：判断「当前 DROP 值是否仍然是未经现场验证的占位」，
是的话就让实验 3 脚本拒绝启动。**不试图猜真实值** —— 猜出来的值比占位值更危险，
因为它看起来像是标定过的。
"""

from __future__ import annotations

# 占位值的原样（与 tuning.py 里 TODO(标定) 那两行逐位对应）。
# 现场示教后 tuning.py 会被改成实数，这里就对不上了，闸门自动放行。
PLACEHOLDER_PRE_DROP = [0.45, -0.35, 0.60, 0.0, 0.0, 0.0, 1.0]
PLACEHOLDER_DROP = [0.45, -0.35, 0.50, 0.0, 0.0, 0.0, 1.0]

_TOL = 1e-6


def _same(a, b) -> bool:
    if a is None or b is None or len(a) != len(b):
        return False
    return all(abs(float(x) - float(y)) < _TOL for x, y in zip(a, b))


def is_placeholder(pose, which: str = "drop") -> bool:
    """这组值是否还是未经标定的占位值。"""
    ref = PLACEHOLDER_DROP if which == "drop" else PLACEHOLDER_PRE_DROP
    return _same(pose, ref)


def check_drop_poses(pre_drop, drop) -> dict:
    """实验 3 启动前调用。

    返回 {"ok": bool, "blockers": [...], "warnings": [...]}
    ok=False 时调用方应当**拒绝执行**，并打印 blockers。
    """
    blockers = []
    warnings = []

    if is_placeholder(drop, "drop"):
        blockers.append(
            "DROP_POSE 仍是占位值 [0.45, -0.35, 0.50]（tuning.py 里标着 TODO(标定)）。"
            "左移+右转后桶的真实位置未知，直接跑会朝未验证坐标伸手。")
    if is_placeholder(pre_drop, "pre_drop"):
        blockers.append(
            "PRE_DROP_POSE 仍是占位值 [0.45, -0.35, 0.60]（同上）。")

    # 位姿维度与单位自检（不阻断，只提醒）
    for name, p in (("PRE_DROP_POSE", pre_drop), ("DROP_POSE", drop)):
        if p is None:
            continue
        if len(p) != 7:
            blockers.append(f"{name} 维度是 {len(p)}，应为 7（x,y,z,qx,qy,qz,qw）。")
        elif abs(float(p[6])) < 1e-6 and all(abs(float(v)) < 1e-6 for v in p[3:6]):
            warnings.append(f"{name} 四元数全 0，不是合法姿态，朝向会不确定。")
        if float(p[1]) >= 0:
            warnings.append(f"{name} 的 y={p[1]} >= 0，跑到了 base_link 左侧；"
                            "右臂作业区应在 y<0。")

    # 高度合理性：DROP 必须比 PRE_DROP 低（先在高处预备，再下降投放）
    if pre_drop is not None and drop is not None and len(pre_drop) >= 3 and len(drop) >= 3:
        if float(drop[2]) >= float(pre_drop[2]):
            warnings.append(
                f"DROP_POSE.z={drop[2]} 不低于 PRE_DROP_POSE.z={pre_drop[2]}，"
                "下降投放的动作方向反了，确认是否填反。")

    return {"ok": not blockers, "blockers": blockers, "warnings": warnings}
