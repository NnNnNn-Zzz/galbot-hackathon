#!/usr/bin/env python3
"""脚手架抓取：VLM 只做「看」，几何与运动全由代码闭环。

针对任务：夹起桌上的矿泉水瓶（可换成任意站立在桌面的细高物体）。

与 vlm_pilot（纯 VLM 直控）的分工区别
────────────────────────────────────────────────────────────────
vlm_pilot: 模型看图 → 自己出动作（空间推理靠猜，真机实测会夹空/重复）
scaffold : 模型只做两次视觉判断，其余全是解析几何：
    ① 检测——头部图里瓶子的外接框（归一化，代码乘分辨率）
    ② 终验——夹起后确认瓶子真的被夹离桌面
    几何/运动/收敛判断/闭爪时机全部代码算：
    - 桌面高度 z_table：右腕深度相机在「框下方像素」取深度（桌面不透明，
      深度可靠；打在瓶身上的深度不可信，不取）→ 反投影成 base 系高度
    - 瓶脚 (x,y)：头部图框底边中点像素射线 ∩ 平面 z=z_table
    - 目标点 = (x, y, z_table + grasp_h)，与末端位姿求差，每步 ≤3cm 逼近
    - 自动选臂：瓶脚 y>0.03 用左臂 / y<-0.03 用右臂（手臂不跨中线）

流程
    1. 张爪（8cm 开度，瓶径 ~6cm）
    2. 测 z_table（3 帧中值，兜底 0.709）
    3. 头部图 VLM 检测 → 瓶脚 → 选臂 → 目标点
    4. 闭环逼近（每轮重检测；瓶子被手臂挡住 = 已贴近，用上一目标死推）
    5. 水平差 <1.5cm → 对齐高度 → 闭爪 → 抬 0.10m
    6. VLM 终验（头部图：瓶子被夹起来了吗）

跑法（机器人上）
    cd ~/galbot_hackathon_dev && source ~/galbot_hackathon/scripts/env_hpu.sh
    set -a; . ./.env.local; set +a
    python3 agent/scaffold_grasp.py --dry      # 只检测+算目标，不动
    python3 agent/scaffold_grasp.py            # 实跑
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from sdk_wrapper import wrapper
from sdk_wrapper.dev import toolhub

logger = logging.getLogger("scaffold_grasp")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
                    datefmt="%H:%M:%S")

# ---- 可调参数（全部宏定义，按需覆盖） ------------------------------------- #
GRASP_H_M = 0.08            # 抓取点高度：桌面以上 8cm（瓶腰，重心下方）
GRIP_OPEN_CM = 8.0          # 张爪开度（瓶径 ~6cm 留余量）
ARRIVE_XY_M = 0.015         # 水平收敛阈值
ARRIVE_Z_M = 0.015          # 高度收敛阈值
MAX_STEP_M = 0.03           # 单步最大位移
MAX_ITERS = 12              # 逼近轮数上限
TABLE_FALLBACK_M = 0.709    # 桌面高度兜底值（实测 0.706~0.710 稳定）
LIFT_M = 0.10               # 闭爪后抬起高度
MIDLINE_MARGIN_M = 0.03     # 选臂中线死区
SENSORS = ["LEFT_ARM_CAMERA", "RIGHT_ARM_CAMERA",
           "LEFT_ARM_DEPTH_CAMERA", "RIGHT_ARM_DEPTH_CAMERA",
           "HEAD_LEFT_CAMERA"]

DETECT_PROMPT = (
    "这是一台人形机器人头部相机拍的照片。请找出画面里的矿泉水瓶（塑料瓶装水，"
    "瓶身有标签；可能被手臂部分遮挡，只要能看到瓶身就算找到），"
    "返回它的紧致外接矩形，**坐标用 0~1 的比例表示**（左上角是 0,0，右下角是 1,1）。"
    "框要框住整个瓶身，从瓶盖或瓶肩到瓶底。\n"
    "只输出 JSON：\n"
    '{"found": true, "bbox_norm": [x1, y1, x2, y2]}\n'
    "没找到就返回 {\"found\": false}。"
)

VERIFY_PROMPT = (
    "这是机器人头部相机刚拍的照片。机器人的夹爪刚才闭合并抬起。"
    "请判断：夹爪现在是否夹着矿泉水瓶，且瓶子已离开桌面？"
    '只输出 JSON：{"held": true/false, "detail": "一句话中文依据"}'
)


# ---- VLM 调用（OpenAI vision 兼容，零依赖） ------------------------------- #
def _vlm_raw(prompt: str, image_paths: list[str]) -> dict:
    api_key = os.environ.get("LLM_API_KEY", "")
    if not api_key:
        raise RuntimeError("未设 LLM_API_KEY")
    base_url = os.environ.get("LLM_BASE_URL", "https://api.deepseek.com/v1").rstrip("/")
    model = os.environ.get("LLM_VLM_MODEL") or os.environ.get("LLM_MODEL") or "deepseek-flash"
    content: list[dict] = [{"type": "text", "text": prompt}]
    for p in image_paths:
        with open(p, "rb") as fh:
            b64 = base64.b64encode(fh.read()).decode()
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})
    body = json.dumps({"model": model,
                       "messages": [{"role": "user", "content": content}],
                       "response_format": {"type": "json_object"},
                       "temperature": 0}).encode()
    req = urllib.request.Request(f"{base_url}/chat/completions", data=body,
                                 headers={"Content-Type": "application/json",
                                          "Authorization": f"Bearer {api_key}"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        text = json.loads(resp.read().decode())["choices"][0]["message"]["content"] or ""
    s, e = text.find("{"), text.rfind("}")
    if s < 0:
        return {"found": False}
    try:
        return json.loads(text[s:e + 1])
    except json.JSONDecodeError:
        return {"found": False}


def detect_bottle(image_path: str) -> dict:
    """VLM 检测瓶子。归一化框由代码乘分辨率，不让模型报像素。"""
    import cv2
    img = cv2.imread(image_path)
    if img is None:
        return {"success": False, "message": "读不到图片"}
    h, w = img.shape[:2]
    d = _vlm_raw(DETECT_PROMPT, [image_path])
    if not d.get("found") or not d.get("bbox_norm"):
        return {"success": False, "message": "VLM 没找到瓶子"}
    n = [float(v) for v in d["bbox_norm"]]
    bbox = [max(0.0, min(w - 1.0, n[0] * w)), max(0.0, min(h - 1.0, n[1] * h)),
            max(0.0, min(w - 1.0, n[2] * w)), max(0.0, min(h - 1.0, n[3] * h))]
    return {"success": True, "bbox": bbox, "width": w, "height": h}


# ---- 几何工具（抄自 _plane_grasp / _head_grasp，含踩坑注释） -------------- #
def quat_to_R(q):
    qx, qy, qz, qw = [float(v) for v in q]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]]


def ray_plane_hit(u, v, K, ext, z_plane):
    """像素射线 ∩ z=z_plane。⚠️ R @ d_cam 必须行·列，用转置会飞到几米外。"""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    d_cam = [(u - cx) / fx, (v - cy) / fy, 1.0]
    R = quat_to_R(ext[3:7])
    d_base = [sum(R[r][c] * d_cam[c] for c in range(3)) for r in range(3)]
    cam = [float(ext[0]), float(ext[1]), float(ext[2])]
    if abs(d_base[2]) < 1e-6:
        return None
    t = (z_plane - cam[2]) / d_base[2]
    if t <= 0:
        return None
    return [cam[0] + t * d_base[0], cam[1] + t * d_base[1], z_plane]


def median_depth(depth, u, v, k=3):
    h, w = depth.shape
    patch = depth[max(0, v - k):min(h, v + k + 1), max(0, u - k):min(w, u + k + 1)]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)]
    return float(np.median(valid)) if valid.size else None


def estimate_table_z(wrist_arm: str = "right_arm", samples: int = 3) -> tuple[float, str]:
    """右腕深度相机测桌面高度：深度只取『瓶子框下方』像素（必落在桌面，可靠）。"""
    cam_id = f"{wrist_arm.replace('_arm', '')}_arm"
    cap = toolhub.capture_frame(cam_id, save_path="/tmp/sg_wrist.jpg")
    K = (cap.get("intrinsic") or {}).get("K")
    depth = cap.get("depth_m")
    if not cap.get("success") or not K or not cap.get("extrinsic") or depth is None:
        return TABLE_FALLBACK_M, "腕部取帧缺参，用兜底值"
    ext = cap["extrinsic"]
    R = quat_to_R(ext[3:7])
    z_list = []
    for _ in range(samples):
        box = detect_bottle("/tmp/sg_wrist.jpg")
        if not box.get("success"):
            break
        x1, y1, x2, y2 = [float(v) for v in box["bbox"]]
        h = depth.shape[0]
        u = int(round((x1 + x2) / 2))
        z, v_probe = None, None
        for dv in (4, 10, 16, 24, 32):
            v_try = int(min(h - 1, y2 + dv))
            z = median_depth(depth, u, v_try)
            if z:
                v_probe = v_try
                break
        if not z:
            continue
        p_cam = [(u - K[2]) / K[0] * z, (v_probe - K[5]) / K[4] * z, z]
        z_list.append(float(sum(R[2][j] * p_cam[j] for j in range(3)) + float(ext[2])))
    if not z_list:
        return TABLE_FALLBACK_M, "深度取不到，用兜底值"
    return float(np.median(z_list)), f"样本 {[round(z, 3) for z in z_list]}"


# ---- 执行工具 -------------------------------------------------------------- #
def move_step(delta: list[float], chain: str) -> dict:
    """单步移动：限幅 → delta；FAULT 时改绝对位姿重试（实测更稳）。"""
    mag = max(abs(c) for c in delta)
    if mag > MAX_STEP_M:
        delta = [c * MAX_STEP_M / mag for c in delta]
    r = wrapper.move_arm_delta(delta[0], delta[1], delta[2], chain=chain)
    if r.get("success"):
        return r
    cur = wrapper.get_ee_pose(chain)
    if cur.get("success"):
        cp = [float(v) for v in cur["pose"]]
        tgt = [cp[0] + delta[0], cp[1] + delta[1], cp[2] + delta[2]] + cp[3:]
        r2 = wrapper.move_arm_to(tgt, chain=chain)
        if r2.get("success"):
            logger.info("  (delta 失败，绝对位姿重试成功)")
            return r2
    return r


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="只检测+算目标，不动机器人")
    ap.add_argument("--grasp-h", type=float, default=GRASP_H_M)
    ap.add_argument("--table-z", type=float, default=None, help="手动指定桌面高度(米)")
    ap.add_argument("--arm", default="auto", choices=["auto", "left_arm", "right_arm"])
    args = ap.parse_args()

    run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "sdk_wrapper", "dev", "runs", "scaffold",
                           time.strftime("%Y%m%d_%H%M%S"))
    run_dir = os.path.abspath(run_dir)
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "decisions.jsonl")
    logger.info("运行目录: %s", run_dir)

    def record(step: int, phase: str, data: dict):
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps({"step": step, "phase": phase, **data},
                               ensure_ascii=False) + "\n")

    toolhub.configure(sensors=SENSORS, force=True)
    logger.info("=== 脚手架抓取：VLM 只看，代码闭环 ===")

    # ── 1. 测桌面高度 ─────────────────────────────────────────────────
    if args.table_z is not None:
        z_table = args.table_z
        logger.info("桌面高度 z_table=%.3f m（手动指定）", z_table)
    else:
        z_table, src = estimate_table_z()
        logger.info("桌面高度 z_table≈%.3f m（%s）", z_table, src)
    record(0, "table", {"z_table": z_table})

    # ── 2. 头部图检测瓶子 → 瓶脚 → 选臂 → 目标点 ─────────────────────
    head = toolhub.capture_frame("head_left", save_path=os.path.join(run_dir, "head_000.jpg"))
    if not head.get("success"):
        logger.error("头部取图失败: %s", head.get("message"))
        wrapper.shutdown()
        return 1
    box = detect_bottle(os.path.join(run_dir, "head_000.jpg"))
    if not box.get("success"):
        logger.error("检测失败: %s", box.get("message"))
        wrapper.shutdown()
        return 1
    x1, y1, x2, y2 = [float(v) for v in box["bbox"]]
    K = [float(v) for v in head["intrinsic"]["K"]]
    ext = [float(v) for v in head["extrinsic"]]
    foot = ray_plane_hit((x1 + x2) / 2, y2, K, ext, z_table)
    if foot is None:
        logger.error("射线与桌面无交点（像素 %.0f,%.0f）", (x1 + x2) / 2, y2)
        wrapper.shutdown()
        return 1
    logger.info("瓶脚=(%.3f, %.3f) 框=[%d,%d,%d,%d]", foot[0], foot[1],
                x1, y1, x2, y2)
    record(0, "detect", {"bbox": [x1, y1, x2, y2], "foot": foot})

    # 自动选臂：瓶脚在左半区用左臂，右半区用右臂（不跨中线）
    if args.arm == "auto":
        if foot[1] > MIDLINE_MARGIN_M:
            chain = "left_arm"
        elif foot[1] < -MIDLINE_MARGIN_M:
            chain = "right_arm"
        else:
            chain = "right_arm"
            logger.warning("瓶脚在中线死区(±%.2fm)，默认右臂，注意观察", MIDLINE_MARGIN_M)
    else:
        chain = args.arm
    ee_name = "left_gripper" if chain == "left_arm" else "right_gripper"
    if (chain == "right_arm" and foot[1] > MIDLINE_MARGIN_M) or \
       (chain == "left_arm" and foot[1] < -MIDLINE_MARGIN_M):
        logger.error("目标在 %s（y=%.2f）但指定 %s 会跨中线，中止。换 --arm 或用 auto",
                     "左半区" if foot[1] > 0 else "右半区", foot[1], chain)
        wrapper.shutdown()
        return 1
    logger.info("选臂: %s / %s", chain, ee_name)
    record(0, "arm", {"chain": chain, "ee": ee_name})

    target = [foot[0], foot[1], z_table + args.grasp_h]
    logger.info("目标点=(%.3f, %.3f, %.3f)（瓶腰，桌面+%.2fm）",
                target[0], target[1], target[2], args.grasp_h)

    if args.dry:
        # 在头部图上画目标十字，方便人工核对几何算得对不对
        import cv2
        img = cv2.imread(os.path.join(run_dir, "head_000.jpg"))
        px = ((x1 + x2) / 2, y2)
        cv2.drawMarker(img, (int(px[0]), int(px[1])), (0, 0, 255), cv2.MARKER_CROSS, 60, 4)
        cv2.putText(img, "FOOT", (int(px[0]) + 12, int(px[1]) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(run_dir, "head_000_annot.jpg"), img)
        logger.info("[--dry] 标注图: %s", os.path.join(run_dir, "head_000_annot.jpg"))
        wrapper.shutdown()
        return 0

    # ── 3. 张爪 + 闭环逼近 ────────────────────────────────────────────
    r = wrapper.set_gripper(ee_name, width_cm=GRIP_OPEN_CM)
    if not r.get("success"):
        r = wrapper.open_gripper(ee_name)
    logger.info("张爪: %s", "成功" if r.get("success") else r.get("message"))

    ee = None
    last_target = target
    for i in range(1, MAX_ITERS + 1):
        head = toolhub.capture_frame("head_left",
                                     save_path=os.path.join(run_dir, f"head_{i:03d}.jpg"))
        box = detect_bottle(os.path.join(run_dir, f"head_{i:03d}.jpg")) \
            if head.get("success") else {"success": False}
        if box.get("success") and head.get("success"):
            b = [float(v) for v in box["bbox"]]
            K = [float(v) for v in head["intrinsic"]["K"]]
            ext = [float(v) for v in head["extrinsic"]]
            foot = ray_plane_hit((b[0] + b[2]) / 2, b[3], K, ext, z_table)
            if foot is not None:
                last_target = [foot[0], foot[1], z_table + args.grasp_h]
        else:
            logger.info("  第%d轮：头部看不到瓶子（多半被手臂挡住=已贴近），用上一目标",
                        i)

        pose = wrapper.get_ee_pose(chain)
        if not pose.get("success"):
            logger.error("读末端位姿失败: %s", pose.get("message"))
            break
        ee = [float(v) for v in pose["pose"][:3]]
        d = [last_target[k] - ee[k] for k in range(3)]
        xy = (d[0] ** 2 + d[1] ** 2) ** 0.5
        logger.info("  第%d轮：瓶脚=(%.3f,%.3f) 末端=(%.3f,%.3f,%.3f) "
                    "水平差=%.1fcm 高度差=%+.1fcm",
                    i, last_target[0], last_target[1], ee[0], ee[1], ee[2],
                    xy * 100, d[2] * 100)
        record(i, "approach", {"foot": last_target, "ee": ee,
                               "dxy_m": round(xy, 4), "dz_m": round(d[2], 4)})

        if xy <= ARRIVE_XY_M and abs(d[2]) <= ARRIVE_Z_M:
            logger.info("  → 三轴都已对准，准备闭爪")
            break
        if xy <= ARRIVE_XY_M:
            logger.info("  → 水平已对准，只补高度")
            d = [0.0, 0.0, d[2]]
        elif abs(d[2]) > ARRIVE_Z_M * 2:
            # 高度差大时先粗调 z 再走 xy，避免斜着扫倒瓶子
            d = [d[0] * 0.5, d[1] * 0.5, d[2]]
        r = move_step(d, chain)
        logger.info("      走 (%+.1f,%+.1f,%+.1f)cm → %s",
                    d[0] * 100, d[1] * 100, d[2] * 100,
                    "成功" if r.get("success") else r.get("message"))
        if not r.get("success"):
            logger.error("  连绝对位姿重试也失败，停止逼近")
            break
        time.sleep(1.0)

    if ee is None:
        wrapper.shutdown()
        return 1

    # ── 4. 闭爪 → 抬起 ────────────────────────────────────────────────
    pose = wrapper.get_ee_pose(chain)
    ee = [float(v) for v in pose["pose"][:3]]
    d = [last_target[k] - ee[k] for k in range(3)]
    if (d[0] ** 2 + d[1] ** 2) ** 0.5 > 2.5 * ARRIVE_XY_M:
        logger.warning("闭爪前水平差仍 >%.1fcm，闭爪可能夹空，继续尝试", ARRIVE_XY_M * 250)
    logger.info("闭爪…")
    r = wrapper.close_gripper(ee_name)
    logger.info("闭爪: %s", "成功" if r.get("success") else r.get("message"))
    record(99, "close", {"result": bool(r.get("success"))})

    time.sleep(1.0)
    logger.info("抬起 %.2fm…", LIFT_M)
    r = move_step([0.0, 0.0, LIFT_M], chain)
    logger.info("抬起: %s", "成功" if r.get("success") else r.get("message"))
    record(100, "lift", {"result": bool(r.get("success"))})
    time.sleep(1.0)

    # ── 5. VLM 终验：瓶子真的被夹起来了吗 ────────────────────────────
    head = toolhub.capture_frame("head_left", save_path=os.path.join(run_dir, "head_final.jpg"))
    verdict = {"held": None, "detail": "终验取图失败"}
    if head.get("success"):
        v = _vlm_raw(VERIFY_PROMPT, [os.path.join(run_dir, "head_final.jpg")])
        verdict = {"held": bool(v.get("held")), "detail": v.get("detail", "")}
    logger.info("终验: held=%s（%s）", verdict["held"], verdict["detail"])
    record(101, "verify", verdict)

    logger.info("完成。记录: %s", log_path)
    wrapper.shutdown()
    return 0 if verdict.get("held") else 2


if __name__ == "__main__":
    raise SystemExit(main())
