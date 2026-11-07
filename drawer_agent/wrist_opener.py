"""Planning-only seam for the teammate's wrist-vision drawer opener."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class WristOpenRequest:
    setup_id: str
    frame: Any                  # drawer_demo.Frame, the fresh wrist RGB capture
    target_opening_m: float
    start_tcp: tuple[float, ...]
    handle_release_tcp: tuple[float, ...]
    handle_clear_tcp: tuple[float, ...]
    deadline_ns: int            # monotonic clock of the shared Agent


class WristOpenPlanner(Protocol):
    def plan(self, request: WristOpenRequest) -> list[dict]:
        """Return base_link route steps, with no motion or controller calls.

        Route order: approach -> grip -> negative-X pull steps -> release ->
        HANDLE_CLEAR. Raise when handle detection or pose is uncertain.
        """
        ...
