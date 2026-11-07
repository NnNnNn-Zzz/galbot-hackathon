"""Read-only G1 foundation-stereo depth tool.

The SDK stores the depth image in DetectionResult.instance_mask. On the G1
head stereo model, the raw unit was verified as millimetres on 2026-11-08.
This tool preserves the original array and reports ``raw_to_m=0.001``.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from sdk_wrapper.dev.contracts import ErrorCode, RunMode, ToolResult, failed, ok, rejected

__all__ = ["StereoDepthTool"]

_DEFAULT_STORE = Path(__file__).resolve().parents[1] / "runs" / "stereo_depth"
_SDK_LOCK = threading.Lock()
_CAPTURE_LOCK = threading.Lock()
_SDK_PERCEPTION: Any = None
_SDK_READY = False


class StereoDepthTool:
    """Allowlisted tool for one SDK stereo inference; no robot motion or shutdown."""

    TOOL_NAMES = ("capture_stereo_depth",)

    def __init__(
        self,
        *,
        output_dir: Path | None = None,
        perception: Any = None,
        module: Any = None,
        warmup_s: float = 10.0,
    ) -> None:
        # Injection is for offline verification; callers cannot set paths via the tool.
        if (perception is None) != (module is None):
            raise ValueError("perception and module must be supplied together")
        self.mode = RunMode.REAL if perception is None else RunMode.MOCK
        self._perception = perception
        self._module = module
        self._ready = False
        self._output_dir = Path(output_dir) if output_dir is not None else _DEFAULT_STORE
        self._warmup_s = warmup_s
        self._lock = threading.Lock()

    def _sdk(self) -> tuple[Any, Any]:
        global _SDK_PERCEPTION, _SDK_READY
        if self._perception is not None:
            if not self._ready:
                if not self._perception.init({self._module}):
                    raise RuntimeError("GalbotPerception.init failed")
                if self._warmup_s:
                    time.sleep(self._warmup_s)
                self._ready = True
            return self._perception, self._module

        with _SDK_LOCK:
            from sdk_wrapper.wrapper import _sdk_bootstrap

            _sdk_bootstrap()
            from galbot_sdk.g1 import GalbotPerception, PerceptionModule

            module = PerceptionModule.FOUNDATION_STEREO
            if _SDK_PERCEPTION is None:
                _SDK_PERCEPTION = GalbotPerception()
            if not _SDK_READY:
                if not _SDK_PERCEPTION.init({module}):
                    raise RuntimeError("GalbotPerception.init(FOUNDATION_STEREO) failed")
                time.sleep(self._warmup_s)
                _SDK_READY = True
            return _SDK_PERCEPTION, module

    def capture_stereo_depth(self, timeout_s: float = 8.0) -> ToolResult:
        """Capture one map; return a six-field ToolResult with file references."""
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s)
            or not 0 < timeout_s <= 30
        ):
            return rejected(ErrorCode.INVALID_ARGUMENT, "timeout_s must be in (0, 30] seconds", self.mode)

        try:
            # Serialize run_once/wait/get_latest_result so concurrent calls cannot mix frames.
            with (_CAPTURE_LOCK if self.mode is RunMode.REAL else self._lock):
                perception, module = self._sdk()
                if not perception.run_once(module):
                    return failed(ErrorCode.SDK_ERROR, "FOUNDATION_STEREO run_once failed", self.mode)
                if not perception.wait_for_new_result(module, timeout_s=float(timeout_s)):
                    return failed(ErrorCode.TIMEOUT, "FOUNDATION_STEREO produced no new result", self.mode)
                received, result = perception.get_latest_result(module)
                if not received:
                    return failed(ErrorCode.NOT_READY, "FOUNDATION_STEREO result unavailable", self.mode)
                depth = np.asarray(result.instance_mask)
                if depth.ndim != 2 or depth.size == 0 or depth.dtype.kind not in "uif":
                    return failed(ErrorCode.SDK_ERROR, "FOUNDATION_STEREO depth map has invalid shape or dtype", self.mode)
                valid = np.isfinite(depth) & (depth > 0)
                if not np.any(valid):
                    return failed(ErrorCode.NOT_READY, "FOUNDATION_STEREO depth map has no valid pixels", self.mode)
                depth = depth.copy()  # SDK may reuse its result buffer.
                sensor_name = str(getattr(result, "sensor_name", ""))
                timestamp_ns = int(getattr(result, "timestamp_ns", 0))

            capture_id = uuid.uuid4().hex
            folder = self._output_dir / capture_id
            folder.mkdir(parents=True, exist_ok=False)
            raw_path = folder / "depth_raw.npy"
            preview_path = folder / "depth_preview.png"
            np.save(raw_path, depth, allow_pickle=False)

            values = depth[valid].astype(np.float64)
            p01, p50, p99 = np.percentile(values, [1, 50, 99])
            scaled = np.clip((depth.astype(np.float64) - p01) / max(p99 - p01, 1.0), 0, 1)
            scaled[~valid] = 0
            preview = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            preview[~valid] = 0
            if not cv2.imwrite(str(preview_path), preview):
                raise OSError(f"Unable to save {preview_path}")

            return ok("双目深度抓取成功；原始值单位为毫米", self.mode, {
                "capture_id": capture_id,
                "depth_ref": str(raw_path),
                "preview_ref": str(preview_path),
                "sensor_name": sensor_name,
                "timestamp_ns": timestamp_ns,
                "width": int(depth.shape[1]),
                "height": int(depth.shape[0]),
                "dtype": str(depth.dtype),
                "unit": "mm",
                "raw_to_m": 0.001,
                "valid_ratio": float(np.mean(valid)),
                "raw_min": float(np.min(values)),
                "raw_median": float(p50),
                "raw_max": float(np.max(values)),
            })
        except Exception as exc:  # SDK bindings can raise platform-specific errors.
            return failed(ErrorCode.SDK_ERROR, f"双目深度抓取失败：{type(exc).__name__}: {exc}", self.mode)
