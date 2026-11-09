"""Offline front-end logic; no SDK calls, detector, or OCR implementation."""
from dataclasses import dataclass
from datetime import date
from calendar import monthrange
import re
from typing import Callable

NAMES = {"ganmaoling": "感冒灵", "siji": "四季感冒片",
         "vitamin_c": "维C泡腾片", "fish_oil": "鱼油"}


@dataclass(frozen=True)
class Expiry:
    value: str
    precision: str
    valid_through: date


def parse_expiry_info(text: str) -> Expiry:
    """Demo policy: valid through stated day, or last day of stated month."""
    if not isinstance(text, str):
        raise TypeError("日期必须为字符串")
    text = re.sub(r"\s+", "", text)
    text = re.sub(r"^(?:到期日期|EXP)[:：]?", "", text, flags=re.IGNORECASE)
    month = re.fullmatch(r"(\d{4})-(\d{1,2})", text)
    if month is not None:
        year, number = map(int, month.groups())
        date(year, number, 1)  # Reject invalid year/month before calendar arithmetic.
        return Expiry(f"{year:04d}-{number:02d}", "month",
                      date(year, number, monthrange(year, number)[1]))
    match = re.fullmatch(r"(\d{4})-(\d{1,2})-(\d{1,2})", text)
    if match is None:
        match = re.fullmatch(r"(\d{4})年(\d{1,2})月(\d{1,2})日", text)
    if match is None:
        raise ValueError("到期日期须为 YYYY-MM、YYYY-MM-DD 或 YYYY年MM月DD日")
    value = date(*map(int, match.groups()))
    return Expiry(value.isoformat(), "day", value)


def parse_expiry(text: str) -> date:
    """Compatibility helper: returns comparison boundary, not observed precision."""
    return parse_expiry_info(text).valid_through


@dataclass(frozen=True)
class Observation:
    instance_id: str
    class_id: str
    image_x: float
    expiry_text: str
    label_verified: bool
    # Upstream must explicitly validate OCR and instance-to-label association.


def select_target(items: list[Observation], today: date) -> dict:
    """Leftmost expired instance; expiry day remains valid. No grasp authorization."""
    import math
    ids = [item.instance_id for item in items]
    if any(not i for i in ids) or len(set(ids)) != len(ids):
        raise ValueError("实例编号必须非空且唯一")
    expired, unresolved = [], []
    for item in items:
        if (item.class_id not in NAMES or not item.label_verified
                or not math.isfinite(item.image_x)):
            unresolved.append(item.instance_id)
            continue
        try:
            expiry = parse_expiry_info(item.expiry_text)
        except (ValueError, TypeError):
            unresolved.append(item.instance_id)
            continue
        if today > expiry.valid_through:
            expired.append((item, expiry))
    if not items:
        return {"decision": "needs_observation", "target": None, "unresolved": []}
    if not expired:
        return {"decision": "needs_review" if unresolved else "no_expired",
                "target": None, "unresolved": unresolved}
    item, expiry = min(expired, key=lambda pair: (pair[0].image_x, pair[0].instance_id))
    return {"decision": "target_selected", "unresolved": unresolved,
            "target": {"instance_id": item.instance_id, "name": NAMES[item.class_id],
                       "expiry_date": expiry.valid_through.isoformat(),
                       "expiry_label_value": expiry.value,
                       "expiry_precision": expiry.precision,
                       "expiry_raw_text": item.expiry_text,
                       "comparison_policy": "valid_through_stated_day_or_month_end",
                       "speech_text": f"识别到{NAMES[item.class_id]}过期，帮你清理掉吧～"}}


DRAWER_STEPS = (
    "approach_drawer", "locate_handle", "orient_right_gripper_down",
    "approach_handle_from_above", "grasp_handle_and_verify",
    "pull_along_rail_and_verify", "release_handle", "retreat_up",
    "move_to_observation_pose",
)


def prepare_drawer(execute: Callable[[str], dict]) -> dict:
    """Adapter must return existing six-field envelopes with completion semantics.

    Every step requires observed completion. Running/unknown stops orchestration;
    caller must resolve state, never blindly restart this sequence.
    All calibrated geometry and safety checks belong to the adapter.
    """
    trace = []
    for step in DRAWER_STEPS:
        try:
            result = execute(step)
        except Exception as exc:
            return {"ready": False, "stopped_at": step, "trace": trace,
                    "reason": "adapter_exception_state_uncertain",
                    "exception_type": type(exc).__name__}
        trace.append({"step": step, "result": result})
        if not isinstance(result, dict) or result.get("status") != "succeeded":
            return {"ready": False, "stopped_at": step, "trace": trace}
    return {"ready": True, "trace": trace}
