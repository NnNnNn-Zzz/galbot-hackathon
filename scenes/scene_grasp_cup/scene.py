# -*- coding: utf-8 -*-
"""scene_grasp_cup —— 识别桌上的杯子并夹起来（VLM 语义识别 + 右臂 + 右腕相机）。

链路（眼在手上，右腕相机）：
    右臂到观测位姿 → capture 右腕相机 → VLM 找杯子像素(u,v) → 反投影出 base_link 3D
    → 张爪 → 移到杯子上方(预抓取) → 下降到抓取位 → 闭爪 → 抬起 → 归位

真机标定点（mock 验逻辑，真机调这些）：
    * OBSERVE_POSE  —— 右臂观测位姿，要让右腕相机能俯视到桌面杯子
    * GRASP_ORIENTATION —— 抓取时末端朝向（四元数），决定夹爪怎么对准杯子
    * 深度-彩色对齐  —— cameras.json 标 unverified，取深度前确认同分辨率/同光心
    * VLM 像素精度  —— deepseek-flash/qwen-vl-max 给的 (u,v) 有误差，反投影会放大

场景契约：run(params) -> {"success": bool, "feedback": str, "suggested_adjustment": ...}
finally 必归位（纪律：场景退出必须归位）。
"""

from __future__ import annotations

import base64
import json
import logging
import os
import urllib.request

from sdk_wrapper import baseline, wrapper

logger = logging.getLogger("scene.grasp_cup")

# ---- 可标定参数（换桌子/换杯子只改这里）----------------------------------- #
#: 右臂观测位姿 [x,y,z,qx,qy,qz,qw]（base_link，米）。TODO(标定): 让右腕相机俯视到桌面。
OBSERVE_POSE = [0.35, -0.25, 0.75, 0.0, 0.0, 0.0, 1.0]
#: 抓取时末端朝向（四元数）。TODO(标定): 夹爪竖直朝下对准杯子的朝向，真机示教确认。
GRASP_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
#: 预抓取在杯子上方的高度（米）
PRE_GRASP_LIFT_M = 0.12
#: 抓取时相对杯子中心的下沉量（米，夹爪伸到杯子中部）
GRASP_SINK_M = 0.02
#: 抓起后抬升高度（米）
LIFT_M = 0.15
#: 默认抓取链与夹爪
CHAIN = "right_arm"
END_EFFECTOR = "right_gripper"
#: 单次运动超时
MOVE_TIMEOUT_S = 15.0


# ---- VLM 识别：发图问杯子像素坐标 ---------------------------------------- #
def _vlm_locate(image_path: str, query: str) -> dict:
    """发图给多模态大模型，返回目标中心像素坐标 {"u":..,"v":..}。

    用 OpenAI vision 兼容格式（image_url + base64），deepseek-flash / qwen-vl-max 通用。
    """
    api_key = os.environ.get("LLM_API_KEY", "")
    if not api_key:
        return {"success": False, "message": "未设 LLM_API_KEY，无法调 VLM"}
    base_url = os.environ.get("LLM_BASE_URL", "https://api.deepseek.com/v1")
    model = os.environ.get("LLM_VLM_MODEL",
                           os.environ.get("LLM_MODEL", "deepseek-flash"))

    with open(image_path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")

    prompt = (
        f"这张机器人腕部相机画面里有一个【{query}】。请找出它，并给出它中心点的像素坐标。"
        "图像坐标原点在左上角，u 是列（向右增大），v 是行（向下增大）。"
        "严格只输出 JSON：{\"u\": <整数列像素>, \"v\": <整数行像素>}。不要输出别的。"
    )
    body = json.dumps({
        "model": model,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url",
             "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
        ]}],
        "response_format": {"type": "json_object"},
        "temperature": 0.0,
    }).encode("utf-8")

    req = urllib.request.Request(
        f"{base_url}/chat/completions", data=body,
        headers={"Content-Type": "application/json",
                 "Authorization": f"Bearer {api_key}"})
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        content = data["choices"][0]["message"]["content"]
        obj = json.loads(content)
        u, v = int(obj["u"]), int(obj["v"])
        return {"success": True, "u": u, "v": v, "model": model}
    except Exception as e:  # noqa: BLE001
        return {"success": False, "message": f"VLM 识别失败：{e}"}


# ---- 反投影：像素 → base_link 3D ----------------------------------------- #
def _deproject_to_base(u: float, v: float, cap: dict) -> dict:
    """把杯子像素 (u,v) 用深度+内参+外参反投影成 base_link 3D 坐标。"""
    from sdk_wrapper.dev.algorithms import geometry as geo

    intr = cap["intrinsic"]
    K = intr.get("K") or intr.get("k")
    if not K or len(K) < 6:
        return {"success": False, "message": "相机内参 K 缺失"}
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])

    depth = cap["depth_m"]
    dh, dw = depth.shape[0], depth.shape[1]
    w, h = cap["width"], cap["height"]
    # 深度与彩色不同分辨率时，把彩色像素按比例映射到深度图
    su = u * (dw / w) if w else u
    sv = v * (dh / h) if h else v
    ui, vi = int(round(su)), int(round(sv))
    if not (0 <= vi < dh and 0 <= ui < dw):
        return {"success": False,
                "message": f"像素 ({ui},{vi}) 超出深度图 {dw}x{dh}"}
    z = float(depth[vi, ui])
    if z <= 0.05 or z != z:  # 无效深度/NaN
        return {"success": False,
                "message": f"像素 ({ui},{vi}) 深度无效 z={z:.3f}m（超量程/镜面/黑体）"}

    import types
    # geometry.deproject 需要带 .fx/.fy/.cx/.cy 属性的对象
    intr_obj = types.SimpleNamespace(fx=fx, fy=fy, cx=cx, cy=cy)

    pt_cam = geo.deproject(float(u), float(v), z, intr_obj)
    pt_base = geo.transform_point(pt_cam, cap["extrinsic"])
    return {"success": True, "base_xyz": [float(x) for x in pt_base],
            "depth_m": z, "pixel": [u, v]}


# ---- 场景主流程 ----------------------------------------------------------- #
def run(params: dict) -> dict:
    p = params or {}
    query = str(p.get("query", "杯子"))
    chain = str(p.get("chain", CHAIN))
    ee = str(p.get("end_effector", END_EFFECTOR))
    observe_pose = p.get("observe_pose", OBSERVE_POSE)

    try:
        # 1) 右臂到观测位姿（让右腕相机能看到桌面）
        r = wrapper.move_arm_to(observe_pose, chain, timeout_s=MOVE_TIMEOUT_S)
        if not r.get("success"):
            return {"success": False,
                    "feedback": f"移动到观测位姿失败：{r.get('message')}",
                    "suggested_adjustment": "检查手臂是否被占用/碰撞"}

        # 2) 定位杯子（base_link 3D）
        if wrapper._mock_mode:
            # mock：不碰真机相机/VLM，直接用固定坐标验证编排逻辑
            cup = [0.45, -0.25, 0.55]
            logger.info("[MOCK] 杯子固定坐标 %s", cup)
        else:
            cap = wrapper.capture_right_wrist()
            if not cap.get("success"):
                return {"success": False, "feedback": "右腕相机取图失败",
                        "suggested_adjustment": "检查相机是否装配、depth 对齐"}
            loc = _vlm_locate(cap["image_path"], query)
            if not loc.get("success"):
                return {"success": False, "feedback": f"VLM 没找到{query}：{loc.get('message')}",
                        "suggested_adjustment": "换个角度拍 / 换更强的 VLM / 确认杯子在视野内"}
            dep = _deproject_to_base(loc["u"], loc["v"], cap)
            if not dep.get("success"):
                return {"success": False, "feedback": f"反投影失败：{dep.get('message')}",
                        "suggested_adjustment": "深度无效，调整观测距离或角度"}
            cup = dep["base_xyz"]
            logger.info("杯子 base_link 坐标 %s（深度 %.3fm，像素 %s）",
                        [round(x, 3) for x in cup], dep["depth_m"], dep["pixel"])

        x, y, z = cup[0], cup[1], cup[2]
        qx, qy, qz, qw = GRASP_ORIENTATION

        # 3) 张爪
        r = wrapper.open_gripper(ee)
        if not r.get("success"):
            return {"success": False, "feedback": f"张爪失败：{r.get('message')}",
                    "suggested_adjustment": "检查夹爪"}

        # 4) 预抓取：杯子上方
        pre = [x, y, z + PRE_GRASP_LIFT_M, qx, qy, qz, qw]
        r = wrapper.move_arm_to(pre, chain, timeout_s=MOVE_TIMEOUT_S)
        if not r.get("success"):
            return {"success": False, "feedback": f"移动到预抓取位失败：{r.get('message')}",
                    "suggested_adjustment": "预抓取位可能不可达/碰撞"}

        # 5) 下降到抓取位
        grasp = [x, y, z + GRASP_SINK_M, qx, qy, qz, qw]
        r = wrapper.move_arm_to(grasp, chain, timeout_s=MOVE_TIMEOUT_S)
        if not r.get("success"):
            return {"success": False, "feedback": f"下降抓取失败：{r.get('message')}",
                    "suggested_adjustment": "下降量过大或碰到杯子"}

        # 6) 闭爪
        r = wrapper.close_gripper(ee)
        if not r.get("success"):
            return {"success": False, "feedback": f"闭爪失败：{r.get('message')}",
                    "suggested_adjustment": "检查夹爪"}

        # 7) 抬起
        lift = [x, y, z + LIFT_M, qx, qy, qz, qw]
        r = wrapper.move_arm_to(lift, chain, timeout_s=MOVE_TIMEOUT_S)
        if not r.get("success"):
            return {"success": False, "feedback": f"抬起失败：{r.get('message')}",
                    "suggested_adjustment": "可能夹着杯子卡住了"}

        return {"success": True,
                "feedback": f"已夹起{query}（base_link 位置 {[round(v,3) for v in cup]}）",
                "suggested_adjustment": None}

    except Exception as e:  # noqa: BLE001
        logger.exception("scene_grasp_cup 异常")
        return {"success": False, "feedback": f"场景异常：{e}",
                "suggested_adjustment": "查看场景日志"}
    finally:
        # 纪律：场景退出必归位（异常路径也归位）
        try:
            baseline.reset_to_baseline()
        except Exception:  # noqa: BLE001
            logger.warning("归位失败", exc_info=True)
