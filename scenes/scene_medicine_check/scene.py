"""场景：药品管理 —— 过期药品识别与清理。

负责人: 卓杭、韩桐欣

两种环境（params["mode"]）：
  drawer（默认）: 桌面固定一个小抽屉，内含 999 与维生素各一件。
                  流程 = 对位 → 拉开抽屉 → 逐件取出观察 → 判定 → 放回/丢弃 → 推回抽屉
  table（基线版）: 药品直接摆在桌面固定位置，跳过抽屉动作，
                  只做「抓取 → 读日期 → 判定 → 播报 → 丢弃/放回」。
                  比赛现场抽屉做不出来时用这个兜底。

设计约束（严格遵守 docs/DEVELOPMENT_RULES.md）：
  1. 只通过 sdk_wrapper 操作硬件，绝不 import galbot_sdk。
  2. YOLO 检测与手写日期识别不在本进程做，走独立感知服务（HTTP），
     对应规范第一节「算法隔离为独立感知服务进程，只传结构化结果」。
  3. finally 中必须归位。
  4. 封装层暂缺的接口一律软降级（_call），不阻塞主流程、不拖垮主进程。
     缺失清单见 README.md，需负责人 A 补齐后本场景才能真正上真机。
"""

import json
import logging
import os
import re
import time
import urllib.error
import urllib.request
from datetime import date, timedelta

from sdk_wrapper import baseline, wrapper

logger = logging.getLogger("scene.medicine")

# 感知服务地址：由负责人 A 或环境变量配置，形如 http://127.0.0.1:8010
PERCEPTION_URL = os.environ.get("MEDICINE_PERCEPTION_URL", "").rstrip("/")

# 药品名归一化：感知服务返回的类名 -> 播报用的中文名
MEDICINE_ALIASES = {
    "999": "999 感冒灵",
    "999_ganmao": "999 感冒灵",
    "vitamin": "维生素",
    "vitc": "维生素",
}


# ---------------------------------------------------------------------------
# 封装层软调用：接口缺失时降级而不是崩掉
# ---------------------------------------------------------------------------

def _call(func_name: str, *args, **kwargs):
    """调用 sdk_wrapper 的接口。返回统一 dict 结构。

    封装层若尚未实现该函数（或抛 NotImplementedError），记一条警告并返回
    success=False，让调用方决定继续还是跳过，绝不向上抛异常。
    """
    fn = getattr(wrapper, func_name, None)
    if fn is None:
        logger.warning("[依赖缺失] sdk_wrapper.%s 未实现，本步骤跳过（需负责人 A 补齐）",
                       func_name)
        return {"success": False, "message": f"{func_name} 未实现"}
    try:
        ret = fn(*args, **kwargs)
        if isinstance(ret, dict):
            return ret
        return {"success": True, "message": str(ret)}
    except NotImplementedError as exc:
        logger.warning("[依赖缺失] sdk_wrapper.%s 抛 NotImplementedError: %s",
                       func_name, exc)
        return {"success": False, "message": str(exc)}
    except Exception as exc:  # noqa: BLE001 - 单步失败不应终止整场巡检
        logger.error("调用 sdk_wrapper.%s 失败: %s", func_name, exc)
        return {"success": False, "message": str(exc)}


def _announce(text: str, wait_s: float = 0.0) -> None:
    """语音播报。封装层未提供播报接口时退化为日志输出。

    当前 SDK 只有 PCM 播放通道（write_audio_stream_output），没有 TTS，
    也没有封装层的 speak()。负责人 A 补齐 wrapper.speak(text) 后此处自动生效。
    注意：播报是「喊话」，不阻塞运动；wait_s 仅为等人类回应的固定停顿。
    """
    logger.info("[播报] %s", text)
    _call("speak", text)
    if wait_s > 0:
        # 固定停顿等人回应。当前未接入 ASR，无法真正解析「好/不行」，
        # 后续若需要双向确认，改由感知服务提供语音接口。
        time.sleep(wait_s)


# ---------------------------------------------------------------------------
# 日期解析与过期判定
# ---------------------------------------------------------------------------

# 手写日期常见写法，按优先级匹配。识别不了的直接判为「无法判定」，不瞎猜。
_DATE_PATTERNS = (
    (re.compile(r"(20\d{2})\s*[-/.年]\s*(\d{1,2})\s*[-/.月]\s*(\d{1,2})"), "ymd"),
    (re.compile(r"(20\d{2})\s*[-/.年]\s*(\d{1,2})"), "ym"),
    (re.compile(r"(\d{1,2})\s*[-/.月]\s*(\d{1,2})"), "md"),
)


def parse_expiry(text: str, today: date | None = None) -> date | None:
    """把手写/印刷的到期日期字符串解析成 date。无法解析返回 None。

    规则（写死在这里，避免口头约定扯皮）：
      - 年月日齐全 → 直接用
      - 只有年月    → 视为该月最后一天有效（药品有效期通常到月底）
      - 只有月日    → 默认当年；若算出来比今天晚超过 30 天，视为去年
    """
    if not text:
        return None
    today = today or date.today()
    raw = str(text).strip()
    for pattern, kind in _DATE_PATTERNS:
        m = pattern.search(raw)
        if not m:
            continue
        try:
            if kind == "ymd":
                return date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
            if kind == "ym":
                year, month = int(m.group(1)), int(m.group(2))
                first_next = date(year + (month == 12), month % 12 + 1, 1)
                return first_next - timedelta(days=1)
            month, day = int(m.group(1)), int(m.group(2))
            guess = date(today.year, month, day)
            if guess > today + timedelta(days=30):
                guess = date(today.year - 1, month, day)
            return guess
        except ValueError:
            logger.warning("日期字段非法，跳过: %s", raw)
            return None
    return None


def is_expired(expiry: date | None, today: date | None = None) -> bool | None:
    """过期判定。到期当天仍算有效；无法解析返回 None（交由人工复核）。"""
    if expiry is None:
        return None
    return expiry < (today or date.today())


# ---------------------------------------------------------------------------
# 感知服务（YOLO 定位 + 手写日期识别，独立进程，只传结构化结果）
# ---------------------------------------------------------------------------

def _capture(tag: str) -> str | None:
    """拍一张图存临时文件，返回路径。封装层未实现相机接口时返回 None。"""
    ret = _call("capture_image", tag)
    path = ret.get("path") if isinstance(ret, dict) else None
    if path and os.path.exists(path):
        return path
    logger.warning("取图失败（%s），本件药品跳过视觉判定", tag)
    return None


def _detect(image_path: str) -> list:
    """调感知服务，返回药品列表。

    服务约定（POST {PERCEPTION_URL}/detect，body 为 JSON）：
      请求: {"image": "<本地图片路径>", "task": "medicine"}
      响应: {"items": [{"name": "999", "expiry_text": "2025-08-31",
                        "bbox": [x, y, w, h], "confidence": 0.93}, ...]}
    YOLO 与手写 OCR 全部在服务端完成，本进程只拿结果，符合规范第一节。
    """
    if not PERCEPTION_URL:
        logger.warning("未配置 MEDICINE_PERCEPTION_URL，返回空结果")
        return []
    body = json.dumps({"image": image_path, "task": "medicine"}).encode("utf-8")
    req = urllib.request.Request(f"{PERCEPTION_URL}/detect", data=body,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        return payload.get("items", [])
    except (urllib.error.URLError, TimeoutError, ValueError) as exc:
        logger.error("感知服务调用失败: %s", exc)
        return []


def _pretty_name(name: str) -> str:
    return MEDICINE_ALIASES.get(str(name).strip().lower(), str(name))


# ---------------------------------------------------------------------------
# 动作序列
# ---------------------------------------------------------------------------

def _open_drawer() -> bool:
    """对位把手 → 右夹爪夹住 → 向后平动拉开。

    对应场景描述：右手夹爪正对把手（腕部相机辅助），向后拉。
    依赖封装层的 wrist_camera / grasp / pull 接口。
    """
    for step, func, args in (
        ("接近抽屉", "move_forward", (30,)),
        ("腕部相机确认把手", "capture_image", ("wrist",)),
        ("夹住把手", "grasp", ()),
        ("向后拉开", "move_backward", (25,)),
    ):
        ret = _call(func, *args)
        if not ret.get("success"):
            logger.error("开抽屉失败于「%s」：%s", step, ret.get("message"))
            return False
    return True


def _close_drawer() -> None:
    """把抽屉推回去。失败也只记日志，不影响已完成的判定结果。"""
    if not _call("release").get("success"):
        logger.warning("释放夹爪失败，抽屉可能未推回")
    _call("move_forward", 25)


def _pick(item: dict) -> bool:
    """抓起单件药品并对准腕部相机。"""
    bbox = item.get("bbox")
    ret = _call("grasp", bbox)
    return bool(ret.get("success"))


def _drop_to_trash() -> None:
    """换手把过期药品丢进左侧垃圾桶。"""
    _call("release")


def _put_back(item: dict) -> None:
    """未过期：放回原位。"""
    _call("place", item.get("bbox"))


def _inspect(item: dict, wait_s: float) -> dict:
    """对单件药品做「拍 → 认 → 判 → 播报 → 处置」，返回本件结果。"""
    name = _pretty_name(item.get("name", "未知药品"))
    expiry_text = item.get("expiry_text", "")
    expiry = parse_expiry(expiry_text)
    verdict = is_expired(expiry)

    if verdict is None:
        logger.warning("%s 的日期「%s」无法解析，视为需人工复核，放回原位", name, expiry_text)
        _put_back(item)
        return {"name": name, "expiry": expiry_text, "action": "manual_check"}

    if not verdict:
        logger.info("%s 未过期（%s），放回原位", name, expiry.isoformat())
        _put_back(item)
        return {"name": name, "expiry": expiry.isoformat(), "action": "kept"}

    days = (date.today() - expiry).days
    _announce(f"识别到{name}已过期 {days} 天，帮你清理掉吧～", wait_s=wait_s)
    _drop_to_trash()
    return {"name": name, "expiry": expiry.isoformat(), "action": "discarded", "days_overdue": days}


# ---------------------------------------------------------------------------
# 场景入口
# ---------------------------------------------------------------------------

def run(params: dict) -> dict:
    """场景入口，签名固定（见 docs/DEVELOPMENT_RULES.md 4.1）。

    params:
        mode              "drawer"（默认）| "table"（基线版，桌面摆药）
        confirm_wait_s    播报后等待人类回应的秒数，默认 5.0
        max_items         本次最多检查几件，默认 4（防止无限循环）
        perception_url    覆盖默认感知服务地址
    """
    global PERCEPTION_URL
    mode = str(params.get("mode", "drawer")).lower()
    wait_s = float(params.get("confirm_wait_s", 5.0))
    max_items = int(params.get("max_items", 4))
    if params.get("perception_url"):
        PERCEPTION_URL = str(params["perception_url"]).rstrip("/")

    stats = {"checked": 0, "discarded": 0, "kept": 0, "manual": 0, "failed": 0}
    details = []
    result = None

    try:
        # --- 阶段 1：进入可作业位置 ---
        if mode == "drawer":
            if not _open_drawer():
                result = {
                    "success": False,
                    "feedback": "抽屉未能拉开（对位或夹取失败）",
                    "suggested_adjustment": "改用 table 模式把药品摆到桌面，或调整把手高度后重试",
                }
                return result
        else:
            logger.info("table 模式：跳过抽屉动作，直接在桌面作业")

        # --- 阶段 2：取一件 → 拍照 → 识别 → 判定 → 处置 ---
        done = 0
        while done < max_items:
            frame = _capture("wrist")
            if frame is None:
                stats["failed"] += 1
                break
            items = _detect(frame)
            if not items:
                logger.info("视野内未检出药品，结束巡检")
                break
            item = items[0]                      # 一次只处理视野内置信度最高的一件
            if not _pick(item):
                stats["failed"] += 1
                logger.warning("抓取「%s」失败，跳过", item.get("name"))
                done += 1
                continue

            info = _inspect(item, wait_s)
            stats["checked"] += 1
            details.append(info)
            if info["action"] == "discarded":
                stats["discarded"] += 1
            elif info["action"] == "kept":
                stats["kept"] += 1
            else:
                stats["manual"] += 1
            done += 1

        # --- 阶段 3：收尾 ---
        if mode == "drawer":
            _close_drawer()

        if stats["checked"] == 0:
            result = {
                "success": False,
                "feedback": "本次未成功检查任何药品（取图或抓取失败）",
                "suggested_adjustment": "检查相机接口与夹爪对位；确认药品在视野范围内",
            }
        else:
            result = {
                "success": True,
                "feedback": (f"药检完成：共检查 {stats[checked]} 件，"
                             f"清理过期 {stats[discarded]} 件，"
                             f"放回 {stats[kept]} 件，"
                             f"待人工复核 {stats[manual]} 件"),
                "suggested_adjustment": None,
                "details": details,
            }
        return result

    except Exception as exc:  # noqa: BLE001 - 场景异常不能拖垮主进程
        logger.exception("药品管理场景异常")
        result = {
            "success": False,
            "feedback": f"药品管理场景异常: {exc}",
            "suggested_adjustment": "检查感知服务与封装层接口状态，归位后重试",
        }
        return result

    finally:
        # 硬性规范：无论成功、失败还是异常，都必须归位
        baseline.reset_to_baseline()
