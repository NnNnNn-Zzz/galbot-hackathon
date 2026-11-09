"""Integration boundaries. Coordinates are base_link metres, angles radians."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol


@dataclass
class Context:
    run_id: str
    setup_id: str
    target: dict | None = None
    picked: bool | None = None
    flags: dict = field(default_factory=dict)
    trace: list[dict] = field(default_factory=list)
    stopped_reason: str | None = None


@dataclass(frozen=True)
class Result:
    status: str                 # ok / failed / unknown
    message: str = ""
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Frame:
    frame_id: str
    camera_id: str
    capture_timestamp_ns: int  # capture clock mapped to IO.now_ns domain
    image_size_wh: tuple[int, int]
    image_ref: str
    calibration_id: str
    setup_id: str


@dataclass(frozen=True)
class Action:
    name: str
    params: dict = field(default_factory=dict)
    required: tuple[str, ...] = ()
    request_id: str = ""


@dataclass(frozen=True)
class Receipt:
    status: str                # command return alone is insufficient
    action_id: str
    timestamp_ns: int
    evidence: dict = field(default_factory=dict)
    message: str = ""


class DemoIO(Protocol):
    """Adapter owns the approved shared Agent, controller and independent stop.

    execute must enforce deadline, current named joint branch, head lock,
    unchanged base pose except for the two yaw actions this recipe authorizes,
    tool config, whole-path IK and swept gripper/payload clearance, plus
    continuously monitored telemetry/force. It must not return ok until fresh
    physical feedback confirms arrival: for base yaw that means odometry, for
    the drawer push that means measured drawer travel. No queue may survive
    stop().
    """
    backend: str

    def now_ns(self) -> int: ...
    def preflight(self, recipe: dict) -> dict: ...
    def capture(self, viewpoint: str, timeout_s: float) -> Frame: ...
    def segment(self, frame: Frame, timeout_s: float) -> dict: ...
    def execute(self, action: Action, timeout_s: float) -> Receipt: ...
    def stop(self, reason: str) -> dict: ...


class Stage(Protocol):
    def run(self, ctx: Context, io: DemoIO) -> Result: ...


class GateError(ValueError):
    """Known invalid evidence or configuration; never authorizes recovery."""


def require(condition: Any, message: str) -> None:
    if not condition:
        raise GateError(message)
