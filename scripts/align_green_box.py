#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绿块视觉伺服：把右腕画面里的绿色标记对齐到「标准位置」。

标准位置来自文氓给的参考图（1280x720，与右腕相机同分辨率）::

    中心 (u0, v0) = (681.54, 383.25)     归一化 (0.5325, 0.5323)
    长边倾角       = +1.10°（相对图像竖直方向，顺时针为正）
    长边/短边      = 143.1 x 129.4 px（竖长，比例 1.106）

因为矩形只有 180° 对称、没有 90° 对称，所以把「倾角」定义成**相对竖直方向**、
范围 (-45°, +45°]，避免 minAreaRect 的 w/h 互换导致角度突然跳 90°。

控制约束（用户的硬要求）
------------------------
1. **只动手臂关节** —— 只下发 EndEffector 位姿，不碰底盘、不碰腰。
2. **夹爪指向不变** —— 接近轴（local +X）方向不动，只允许绕该轴自转，
   所以「指向」在语义上没变，变的只是两指开合方向。
3. **末端高度不变** —— z 恒定，只在 base 的 XY 平面内平移。
4. 矩形大小可以不同（距离可以变），只对齐**中心**和**倾角**。

控制律
------
用内参 K + 外参（base<-camera）+ 绿块中心深度，解析出图像对末端 XY 的
2x2 雅可比，再解 ``J @ [dx,dy] = -[du,dv]``。每步限幅后闭环迭代，
所以内参/外参有偏差也能收敛（相当于带自校正的比例控制）。

自转符号自适应：第一次旋转后比较倾角误差是变小还是变大，变大就翻转符号。

用法
----
    source ~/galbot_hackathon/scripts/env_hpu.sh

    python3 scripts/align_green_box.py --dry-run     # 只看差距和计划，不动
    python3 scripts/align_green_box.py               # 闭环对齐
    python3 scripts/align_green_box.py --no-rotation # 只平移，不碰手腕
    python3 scripts/align_green_box.py --ref-image /path/to/ref.jpg  # 换标准图

安全
----
* 单步平移默认 ≤ 3 cm、自转 ≤ 5°，最多 8 次迭代。
* 每步都回读末端位姿，确认 z 真的没变；z 漂移 > 3 mm 立即停。
* 误差连续变大 → 判定符号/增益错，立即停，不硬凑。
* 绿块跑出画面 → 停。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

import cv2
import numpy as np

try:  # 允许在没有 SDK 的机器上单独测试检测逻辑
    from sdk_wrapper.dev.algorithms import geometry as geo
except ImportError:  # pragma: no cover
    geo = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# 标准位置档案（每次标定一个腰高，键是腰高 cm）
# 参考图分辨率都是 1280x720，与右腕相机一致。
# ---------------------------------------------------------------------------
PROFILES = {
    "1327": {
        "waist_m": 1.3270,
        "center": (681.54, 383.25),
        "tilt_deg": 1.10,
        "long_px": 143.11,
        "short_px": 129.42,
        "note": "2026-11-08 首次标定",
    },
    "1234": {
        "waist_m": 1.2337,
        "center": (686.34, 481.38),
        "tilt_deg": 0.78,
        "long_px": 205.91,
        "short_px": 180.70,
        "note": "2026-11-08 腰高 1.234 m 标定",
    },
}

#: 默认档案（最新一次标定）
DEFAULT_PROFILE = "1234"

#: 腰高与档案允许的偏差；超过就提示用错了标准
PROFILE_WAIST_TOL_M = 0.02

# 兼容旧引用（参考图尺寸）
REF_W, REF_H = 1280, 720

# 绿块 HSV 阈值（深绿底 + 白字）
HSV_LO = (35, 60, 40)
HSV_HI = (85, 255, 205)

# 合理面积占比，超出就认为检测错了
AREA_MIN_RATIO = 0.0015
AREA_MAX_RATIO = 0.12

# 长宽比合理范围（绿块实测 ~1.10）
ASPECT_MIN, ASPECT_MAX = 0.55, 2.2

DEFAULT_CHAIN = "right_arm"
DEFAULT_CAMERA = "right_arm"

#: ★ SDK 实测（2026-11-08，right_arm，纯平移）：>=6 mm 精确执行，
#: <=5 mm 返回 SUCCESS 但**机器人一动不动**。所以每步要么不走，要么至少走这么多。
MIN_STEP_M = 0.006


def say(msg: str = "") -> None:
    print(msg, flush=True)


def _enum_name(value) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    return str(value).rsplit(".", 1)[-1]


# ---------------------------------------------------------------------------
# 检测
# ---------------------------------------------------------------------------


def detect_green_box(bgr: np.ndarray) -> Optional[dict]:
    """检测画面里最大的绿色矩形标记。返回中心/倾角/尺寸/掩膜。"""
    if bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_LO), np.array(HSV_HI))

    # 填掉白色「666」造成的洞，再去掉碎点
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    ratio = area / float(w * h)
    if ratio < AREA_MIN_RATIO or ratio > AREA_MAX_RATIO:
        return None

    rect = cv2.minAreaRect(cv2.convexHull(c))
    (cx, cy), (rw, rh), _ang = rect
    if rw < 5 or rh < 5:
        return None

    aspect = max(rw, rh) / max(1e-6, min(rw, rh))
    if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
        return None

    box = cv2.boxPoints(rect).astype(np.float64)
    edges = []
    for i in range(4):
        a, b = box[i], box[(i + 1) % 4]
        edges.append(float(np.linalg.norm(b - a)))
    long_len = max(edges)
    short_len = min(edges)

    # 长边方向角（图像坐标，v 轴向下）
    best = None
    for i in range(4):
        a, b = box[i], box[(i + 1) % 4]
        L = float(np.linalg.norm(b - a))
        if best is None or L > best[0]:
            best = (L, b - a)
    d = best[1]
    theta_long = math.degrees(math.atan2(float(d[1]), float(d[0])))

    # 统一成「相对竖直方向的倾角」，范围 (-45, 45]
    tilt = theta_long - 90.0
    while tilt <= -45.0:
        tilt += 90.0
    while tilt > 45.0:
        tilt -= 90.0

    return {
        "u": float(cx), "v": float(cy),
        "tilt_deg": float(tilt),
        "long_px": long_len, "short_px": short_len,
        "area_ratio": ratio,
        "box": box.astype(np.int32),
        "mask": mask,
        "frame_size": (w, h),
    }


def angle_delta_deg(a: float, b: float) -> float:
    """两个倾角之差，归一到 (-90, 90]。"""
    d = float(a) - float(b)
    while d > 90.0:
        d -= 180.0
    while d <= -90.0:
        d += 180.0
    return d


# ---------------------------------------------------------------------------
# 相机与位姿
# ---------------------------------------------------------------------------


def grab(camera: str, path: str = "/tmp/align_box_frame.jpg") -> dict:
    from sdk_wrapper.dev import toolhub

    res = toolhub.capture_frame(camera, path)
    if not res.get("success"):
        raise RuntimeError("抓帧失败：%s" % res.get("message"))
    img = cv2.imread(res["image_path"], cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("抓到的图读不出来：%s" % res.get("image_path"))
    res["bgr"] = img
    return res


def depth_at(frame: dict, u: float, v: float) -> Optional[float]:
    """取绿块中心的深度（米）。取 5x5 中位数，避开空洞。"""
    d = frame.get("depth_m")
    if d is None:
        return None
    arr = np.asarray(d, dtype=np.float64)
    if arr.ndim != 2:
        return None
    h, w = arr.shape[:2]
    # 深度图分辨率可能与彩色不同，按比例映射
    fw, fh = frame.get("width"), frame.get("height")
    if fw and fh and (w != fw or h != fh):
        u = float(u) * w / float(fw)
        v = float(v) * h / float(fh)
    iu, iv = int(round(u)), int(round(v))
    r = 2
    patch = arr[max(0, iv - r):iv + r + 1, max(0, iu - r):iu + r + 1]
    patch = patch[np.isfinite(patch) & (patch > 1e-4)]
    if patch.size == 0:
        return None
    return float(np.median(patch))


def intrinsic_of(frame: dict) -> Tuple[float, float, float, float]:
    intr = frame.get("intrinsic") or {}
    K = intr.get("K")
    if not K or len(K) < 6:
        raise RuntimeError("取不到相机内参 K")
    return float(K[0]), float(K[4]), float(K[2]), float(K[5])


def pose_of(ad, chain: str) -> List[float]:
    res = ad.get_end_effector_pose(chain)
    if not res.succeeded:
        raise RuntimeError("读末端位姿失败：%s" % (res.message or res.code))
    return [float(v) for v in res.data["pose"]]


def read_waist(ad) -> float:
    """读当前腰高（base_link <- head_base_link 的 Z）。"""
    robot = ad._ensure_interfaces()[0]
    pose, _ts = robot.get_transform(
        "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500)
    if pose is None or len(pose) < 3:
        raise RuntimeError("get_transform 返回空")
    return float(pose[2])


def rot_x(deg: float) -> np.ndarray:
    t = math.radians(float(deg))
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]], dtype=np.float64)


# ---------------------------------------------------------------------------
# 雅可比：图像像素 <- 末端 base XY 平移
# ---------------------------------------------------------------------------


def pixel_jacobian(frame: dict, u: float, v: float) -> Optional[np.ndarray]:
    """解析出 ∂(u,v)/∂(base_x, base_y)，2x2。失败返回 None。"""
    ext = frame.get("extrinsic")
    if not ext or len(ext) < 7:
        return None
    fx, fy, cx, cy = intrinsic_of(frame)
    depth = depth_at(frame, u, v)
    if depth is None or not math.isfinite(depth) or depth <= 0.05:
        return None

    R_cb = geo.quaternion_to_matrix(*[float(x) for x in ext[3:7]])  # base <- cam
    t_cb = np.array([float(x) for x in ext[:3]])

    # 目标点在相机系，再转 base
    P_cam = np.array([(u - cx) / fx * depth,
                      (v - cy) / fy * depth,
                      depth], dtype=np.float64)
    P_base = R_cb @ P_cam + t_cb

    # 重新用 base 系坐标推投影，避免外参与 depth 口径不一致
    Pc = R_cb.T @ (P_base - t_cb)
    Z = float(Pc[2])
    if Z <= 1e-3:
        return None
    X, Y = float(Pc[0]), float(Pc[1])

    dudP_cam = np.array([fx / Z, 0.0, -fx * X / (Z * Z)])
    dvdP_cam = np.array([0.0, fy / Z, -fy * Y / (Z * Z)])
    dPcam_dPbase = R_cb.T

    J = np.vstack([dudP_cam, dvdP_cam]) @ dPcam_dPbase   # 2x3
    return J[:, :2].copy()


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="把右腕画面里的绿块对齐到标准位置",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--chain", default=DEFAULT_CHAIN, help="手臂链")
    ap.add_argument("--camera", default=DEFAULT_CAMERA, help="相机 id")
    ap.add_argument("--profile", default="auto",
                    help="标准档案：auto（按腰高自动匹配）/ %s"
                         % " / ".join(sorted(PROFILES)))
    ap.add_argument("--ref-image", default=None,
                    help="从这张参考图重新提取标准中心/倾角（优先级最高）")
    ap.add_argument("--tol-px", type=float, default=10.0,
                    help="中心容差（像素）；受 SDK 最小步长 6mm≈8px 限制，别设太小")
    ap.add_argument("--tol-deg", type=float, default=1.5, help="倾角容差（度）")
    ap.add_argument("--max-iter", type=int, default=8, help="最多迭代次数")
    ap.add_argument("--max-step-m", type=float, default=0.03,
                    help="单步平移上限（米），不得小于 SDK 最小步长 0.006")
    ap.add_argument("--max-roll-deg", type=float, default=5.0, help="单步自转上限（度）")
    ap.add_argument("--no-rotation", action="store_true",
                    help="只平移，完全不碰手腕姿态")
    ap.add_argument("--z-tol-m", type=float, default=0.003,
                    help="末端高度允许漂移（米）")
    ap.add_argument("--dry-run", action="store_true", help="只测差距，不下发运动")
    ap.add_argument("--force", action="store_true",
                    help="腰高与档案不符时仍强行继续（危险）")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--save-dir", default="/tmp/align_box",
                    help="过程图保存目录")
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if args.max_step_m < MIN_STEP_M:
        say("--max-step-m %.1f mm 小于 SDK 最小有效步长，已抬到 %.0f mm"
            % (args.max_step_m * 1000.0, MIN_STEP_M * 1000.0))
        args.max_step_m = MIN_STEP_M

    os.makedirs(args.save_dir, exist_ok=True)

    from sdk_wrapper.dev.adapters.galbot_adapter import GalbotAdapter
    ad = GalbotAdapter(enable_real_control=True, real_authorized=True)

    # 读腰高：既用来挑标准档案，也用来防呆
    waist = None
    try:
        waist = read_waist(ad)
    except Exception as exc:  # noqa: BLE001
        say("! 读腰高失败（%s: %s），将使用默认档案" % (type(exc).__name__, exc))

    # ---- 选标准：参考图 > 指定档案 > 按腰高自动匹配 ----
    if args.ref_image:
        ref_img = cv2.imdecode(np.fromfile(args.ref_image, dtype=np.uint8),
                               cv2.IMREAD_COLOR)
        if ref_img is None:
            say("参考图读不出来：%s" % args.ref_image)
            return 2
        rdet = detect_green_box(ref_img)
        if rdet is None:
            say("参考图里没检出绿色矩形")
            return 2
        rh, rw = ref_img.shape[:2]
        ref_u, ref_v = rdet["u"], rdet["v"]
        ref_tilt = rdet["tilt_deg"]
        args._ref_long = rdet["long_px"]
        args._ref_size = (rw, rh)
        tag = "image:%s (%dx%d)" % (os.path.basename(args.ref_image), rw, rh)
    else:
        key = args.profile
        if key == "auto":
            if waist is not None:
                key = min(PROFILES,
                          key=lambda k: abs(PROFILES[k]["waist_m"] - waist))
            else:
                key = DEFAULT_PROFILE
        if key not in PROFILES:
            say("未知档案 %r；可用：auto / %s" % (key, " / ".join(sorted(PROFILES))))
            return 2
        prof = PROFILES[key]
        if waist is not None and \
                abs(prof["waist_m"] - waist) > PROFILE_WAIST_TOL_M and \
                not args.force:
            # 腰高对不上就别跑了：标准位置是绑死在腰高上的
            say("✗ 拒绝运行：当前腰高 %.4f m，最近的档案 %s 标定于 %.4f m，差 %.1f cm"
                % (waist, key, prof["waist_m"],
                   abs(prof["waist_m"] - waist) * 100.0))
            say("  标准位置与腰高是一一对应的，腰不对就不可能对得准。")
            say("  处置：① 先用腰部遥控把腰高调到位再跑；")
            say("        ② 或用 --ref-image 传该腰高下标定的标准图；")
            say("        ③ 确知风险时加 --force 强行继续。")
            return 2
        ref_u, ref_v = prof["center"]
        ref_tilt = prof["tilt_deg"]
        args._ref_long = prof.get("long_px")
        args._ref_size = (REF_W, REF_H)
        tag = "profile:%s（%s）" % (key, prof["note"])

    say("绿块视觉伺服")
    say("  标准来源 %s" % tag)
    say("  当前腰高 %.4f m" % waist if waist is not None else "  当前腰高 读取失败")
    say("  目标中心 (%.2f, %.2f)  倾角 %+.2f°（相对竖直）"
        % (ref_u, ref_v, ref_tilt))

    def to_runtime(ref_pt, frame_w, frame_h):
        """参考图坐标 -> 当前相机分辨率坐标。"""
        rw, rh = args._ref_size
        return ref_pt[0] * frame_w / float(rw), ref_pt[1] * frame_h / float(rh)

    frame = grab(args.camera, os.path.join(args.save_dir, "iter00.jpg"))
    fw, fh = frame["width"], frame["height"]
    tu, tv = to_runtime((ref_u, ref_v), fw, fh)
    say("相机 %dx%d，标准中心换算到本分辨率 = (%.2f, %.2f)" % (fw, fh, tu, tv))

    det = detect_green_box(frame["bgr"])
    if det is None:
        cv2.imwrite(os.path.join(args.save_dir, "fail_iter00.jpg"), frame["bgr"])
        say("✗ 当前画面没检出绿色矩形；图已存 %s/fail_iter00.jpg" % args.save_dir)
        return 3

    # ---- 误检护栏：绿块尺寸要和标准对得上，否则很可能检到了别的东西 ----
    ref_long = getattr(args, "_ref_long", None)
    suspicious = []
    if ref_long:
        ratio = det["long_px"] / float(ref_long)
        if ratio < 0.45 or ratio > 2.2:
            suspicious.append("长边 %.1f px 是标准 %.1f px 的 %.0f%%"
                              % (det["long_px"], ref_long, ratio * 100.0))
    if det["area_ratio"] < 0.004:
        suspicious.append("面积占比只有 %.2f%%" % (det["area_ratio"] * 100.0))
    if suspicious:
        cv2.imwrite(os.path.join(args.save_dir, "suspect_iter00.jpg"), frame["bgr"])
        say("")
        say("⚠ 检出的绿块不像目标：%s" % "；".join(suspicious))
        say("  很可能是画面里别的绿色物体（屏幕上绿字、指示灯之类）。")
        say("  图已存 %s/suspect_iter00.jpg，先看一眼。" % args.save_dir)
        if not args.yes:
            try:
                ans = input("仍然继续？(输入 y 继续)：").strip().lower()
            except (EOFError, KeyboardInterrupt):
                ans = ""
            if ans != "y":
                say("已停止，未下发任何运动。")
                return 3
        else:
            say("  --yes 已给，继续——但请确认你真的看到了这条警告")

    du = det["u"] - tu
    dv = det["v"] - tv
    dth = angle_delta_deg(det["tilt_deg"], ref_tilt)
    pos = pose_of(ad, args.chain)
    z0 = pos[2]

    say("")
    say("当前中心 (%.2f, %.2f)  倾角 %+.2f°  长边 %.1f px  面积占比 %.3f%%"
        % (det["u"], det["v"], det["tilt_deg"], det["long_px"], det["area_ratio"] * 100))
    say("需要移动 Δu = %+.2f px, Δv = %+.2f px, Δ倾角 = %+.2f°" % (du, dv, dth))
    say("末端当前位置 [%.4f, %.4f, %.4f]（z 全程锁定）" % (pos[0], pos[1], pos[2]))

    if abs(du) <= args.tol_px and abs(dv) <= args.tol_px and \
            (args.no_rotation or abs(dth) <= args.tol_deg):
        say("✓ 已在容差内，不需要动作。")
        return 0

    J = pixel_jacobian(frame, det["u"], det["v"])
    depth = depth_at(frame, det["u"], det["v"])
    if J is None:
        say("! 拿不到解析雅可比（外参/深度缺失），将用保守增益 + 闭环迭代。")
    else:
        say("雅可比（px/m）: [[%.1f, %.1f], [%.1f, %.1f]]  深度 %.3f m"
            % (J[0, 0], J[0, 1], J[1, 0], J[1, 1], depth if depth else -1))

    if args.dry_run:
        if J is not None:
            try:
                dxy = np.linalg.solve(J, np.array([du, dv]))
                say("预估末端增量 dx=%+.4f m, dy=%+.4f m（未下发）" % (dxy[0], dxy[1]))
            except np.linalg.LinAlgError:
                say("雅可比奇异，无法预估增量")
        say("--dry-run：未下发任何运动。")
        return 0

    say("")
    say("约束：只动手臂关节、夹爪指向不变（只允许绕接近轴自转）、末端高度不变。")
    if not args.yes:
        try:
            answer = input("确认开始闭环对齐？(输入 y 继续)：").strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = ""
        if answer != "y":
            say("已取消，未下发任何运动。")
            return 0

    roll_sign = 1.0
    center_err_start = math.hypot(du, dv)
    prev_err = center_err_start + abs(dth) * 10.0
    worsening = 0
    nomove = 0
    history = []
    ok = False
    try:
        pos_start = pose_of(ad, args.chain)
    except Exception:  # noqa: BLE001
        pos_start = list(pos)

    for it in range(1, args.max_iter + 1):
        du = det["u"] - tu
        dv = det["v"] - tv
        dth = angle_delta_deg(det["tilt_deg"], ref_tilt)

        err = math.hypot(du, dv) + (0.0 if args.no_rotation else abs(dth) * 10.0)
        history.append((it, det["u"], det["v"], det["tilt_deg"], err))

        center_ok = abs(du) <= args.tol_px and abs(dv) <= args.tol_px
        rot_ok = args.no_rotation or abs(dth) <= args.tol_deg
        if center_ok and rot_ok:
            say("")
            say("✓ 第 %d 次检查达标：Δu=%+.2f Δv=%+.2f Δθ=%+.2f"
                % (it, du, dv, dth))
            ok = True
            break

        # 算增量。推导：相机随末端平移 δ 时 Δ(u,v) = -J·δ，
        # 要消除误差（Δ = -[du,dv]）就得 J·δ = [du,dv] → δ = J⁻¹·[du,dv]
        if J is not None:
            try:
                dxy = np.linalg.solve(J, np.array([du, dv]))
            except np.linalg.LinAlgError:
                dxy = np.array([du, dv]) * 0.0008
        else:
            scale = 0.0008 if depth is None else (depth / 655.0)
            dxy = np.array([du, dv]) * scale

        n = float(np.linalg.norm(dxy))
        rot_done = args.no_rotation or abs(dth) <= args.tol_deg
        if n < MIN_STEP_M * 0.95 and rot_done:
            # 需要的位移比 SDK 能执行的最小步长还小 —— 再走只会过冲震荡
            say("  误差已在物理极限内（需要 %.1f mm < 最小步长 %.0f mm），停止迭代"
                % (n * 1000.0, MIN_STEP_M * 1000.0))
            break
        if n > 1e-9:
            # 小于 SDK 最小有效步长就放大（否则会被静静吞掉）
            if n < MIN_STEP_M:
                dxy = dxy * (MIN_STEP_M / n)
            elif n > args.max_step_m:
                dxy = dxy * (args.max_step_m / n)
        # 实测（2026-11-08）：droll=+5° 使图像倾角变化约 -6.2°，即 dθ/droll ≈ -1.24。
        # 要消除 dth 就得 droll ≈ dth/1.24 ≈ +dth*0.8。
        droll = 0.0 if args.no_rotation else roll_sign * dth * 0.8
        droll = max(-args.max_roll_deg, min(args.max_roll_deg, droll))

        say("")
        say("[第 %d 次] Δu=%+7.2f px  Δv=%+7.2f px  Δθ=%+6.2f°   →  "
            "dx=%+.4f m  dy=%+.4f m  droll=%+.2f°"
            % (it, du, dv, dth, dxy[0], dxy[1], droll))

        pos = pose_of(ad, args.chain)
        R_cur = geo.quaternion_to_matrix(*pos[3:7])
        R_new = R_cur @ rot_x(droll)
        target = [pos[0] + float(dxy[0]),
                  pos[1] + float(dxy[1]),
                  z0,                       # ★ z 锁定，绝不改
                  *geo.matrix_to_quaternion(R_new)]

        # 下发前先只读 IK 预检；过不了就退化成纯平移
        chk = ad.check_end_effector_reachable(args.chain, target)
        if not chk.succeeded:
            say("  ! 目标位姿过不了 IK/碰撞预检（%s），降级为纯平移"
                % ((chk.data or {}).get("motion_status") or chk.code))
            target = [pos[0] + float(dxy[0]), pos[1] + float(dxy[1]), z0,
                      pos[3], pos[4], pos[5], pos[6]]
            chk = ad.check_end_effector_reachable(args.chain, target)
            if not chk.succeeded:
                say("  ✗ 纯平移也过不了预检，停止")
                break

        # straight_line=False：平移+旋转同时进行时直线规划会 FAULT
        res = ad.move_ee_to(args.chain, target, straight_line=False,
                            enable_collision_check=True, timeout_s=20.0)
        if not res.succeeded:
            say("  ✗ 运动失败：%s（%s）"
                % (res.message or res.code,
                   (res.data or {}).get("motion_status") if isinstance(res.data, dict) else "?"))
            break

        # 回读校验：z 必须没变，而且必须真的动了
        pos2 = pose_of(ad, args.chain)
        dz = pos2[2] - z0
        if abs(dz) > args.z_tol_m:
            say("  ✗ 末端高度漂了 %.1f mm（> %.0f mm），立即停止"
                % (dz * 1000.0, args.z_tol_m * 1000.0))
            break
        moved = math.hypot(pos2[0] - pos[0], pos2[1] - pos[1])
        say("  ✓ 已下发，实测位移 %.1f mm，z 漂移 %+.1f mm"
            % (moved * 1000.0, dz * 1000.0))
        if moved < MIN_STEP_M * 0.4:
            nomove += 1
            say("  ! 几乎没位移（SDK 对小位移静默忽略），累计 %d 次" % nomove)
            if nomove >= 2:
                say("  ✗ 连续两次没有实际位移，停止（确认没有别的进程在抢 SDK）")
                break
        else:
            nomove = 0

        frame = grab(args.camera, os.path.join(args.save_dir, "iter%02d.jpg" % it))
        newdet = detect_green_box(frame["bgr"])
        if newdet is None:
            cv2.imwrite(os.path.join(args.save_dir, "fail_iter%02d.jpg" % it),
                        frame["bgr"])
            say("  ✗ 绿块丢了（跑出画面或被遮挡），停止。图存 %s/fail_iter%02d.jpg"
                % (args.save_dir, it))
            break

        # 自转方向自适应：第一次旋转后看误差有没有变小
        if (not args.no_rotation) and it == 1 and abs(droll) > 0.2:
            new_dth = angle_delta_deg(newdet["tilt_deg"], ref_tilt)
            if abs(new_dth) > abs(dth):
                roll_sign *= -1.0
                say("  ! 自转方向反了（|Δθ| %.2f° → %.2f°），已翻转符号"
                    % (abs(dth), abs(new_dth)))
            else:
                say("  ✓ 自转方向正确（|Δθ| %.2f° → %.2f°）"
                    % (abs(dth), abs(new_dth)))

        det = newdet
        new_center = math.hypot(det["u"] - tu, det["v"] - tv)
        new_err = (new_center
                   + (0.0 if args.no_rotation else
                      abs(angle_delta_deg(det["tilt_deg"], ref_tilt)) * 10.0))

        # 第一步只用「中心误差」判失败：旋转符号反了会暂时拉高总分，
        # 但上面已经翻转符号了，不该因此回退。
        if it == 1 and new_center > center_err_start * 1.3:
            say("  ✗ 第一步后中心误差变大（%.1f → %.1f px），平移标定有问题；回退原位并停止"
                % (center_err_start, new_center))
            try:
                ad.move_ee_to(args.chain, pos_start, straight_line=False,
                              enable_collision_check=True, timeout_s=20.0)
                say("  已回退到起始位姿。")
            except Exception as exc:  # noqa: BLE001
                say("  回退失败：%s；请现场确认机器人位置" % exc)
            break

        if new_err > prev_err * 1.2:
            worsening += 1
            if worsening >= 2:
                say("  ✗ 误差连续两次变大（%.1f → %.1f），符号或增益有问题，停止"
                    % (prev_err, new_err))
                break
        else:
            worsening = 0
        prev_err = new_err
        J = pixel_jacobian(frame, det["u"], det["v"])

    # ---- 汇总 ----
    say("")
    say("=" * 68)
    final_frame = grab(args.camera, os.path.join(args.save_dir, "final.jpg"))
    fdet = detect_green_box(final_frame["bgr"])
    if fdet is not None:
        fdu = fdet["u"] - tu
        fdv = fdet["v"] - tv
        fdth = angle_delta_deg(fdet["tilt_deg"], ref_tilt)
        say("最终中心 (%.2f, %.2f)  倾角 %+.2f°" % (fdet["u"], fdet["v"], fdet["tilt_deg"]))
        say("残差     Δu=%+.2f px  Δv=%+.2f px  Δθ=%+.2f°" % (fdu, fdv, fdth))
        c_ok = abs(fdu) <= args.tol_px and abs(fdv) <= args.tol_px
        r_ok = args.no_rotation or abs(fdth) <= args.tol_deg
        ok = bool(c_ok and r_ok)
        say("中心 %s   倾角 %s" % ("✓" if c_ok else "✗", "✓" if r_ok else "✗"))
    else:
        say("✗ 最终帧没检出绿块")
        ok = False

    try:
        posf = pose_of(ad, args.chain)
        say("末端最终位置 [%.4f, %.4f, %.4f]，相对初始 z 变化 %+.1f mm"
            % (posf[0], posf[1], posf[2], (posf[2] - z0) * 1000.0))
    except Exception:  # noqa: BLE001
        pass

    if history:
        say("迭代轨迹：")
        for it, uu, vv, th, ee in history:
            say("  #%d  (%.1f, %.1f)  %+.2f°  err=%.1f" % (it, uu, vv, th, ee))

    say("过程图在 %s/" % args.save_dir)
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
