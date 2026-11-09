"""Collision-safe decomposition of Cartesian end-effector pose changes.

The motion service can select a distant IK branch when a single Cartesian
request combines significant translation and rotation. This module keeps the
transition on the current branch by splitting it into bounded phases:

1. translate while preserving the measured orientation;
2. rotate in place after translation is sufficiently close.

Callers regenerate every waypoint from the latest measured pose, so planner
drift never accumulates across steps.
"""

from __future__ import annotations

import math
import typing

import numpy as np

__all__ = [
    "DEFAULT_MAX_POSITION_STEP_M",
    "DEFAULT_MAX_ROTATION_STEP_DEG",
    "PoseTransitionStep",
    "PoseTransitionController",
    "quaternion_slerp",
    "rotation_error_deg",
    "next_pose_transition_step",
]


DEFAULT_MAX_POSITION_STEP_M = 0.02
DEFAULT_MAX_ROTATION_STEP_DEG = 7.5


def _pose_array(pose: typing.Sequence[float]) -> np.ndarray:
    arr = np.asarray(pose, dtype=np.float64).reshape(-1)
    if arr.size != 7 or not np.all(np.isfinite(arr)):
        raise ValueError("pose must contain 7 finite values")
    quat = arr[3:7]
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError("pose quaternion has zero norm")
    arr = arr.copy()
    arr[3:7] = quat / norm
    return arr


def rotation_error_deg(
    first: typing.Sequence[float],
    second: typing.Sequence[float],
) -> float:
    """Return the shortest relative orientation error in degrees."""
    q0 = _pose_array(first)[3:7]
    q1 = _pose_array(second)[3:7]
    dot = max(-1.0, min(1.0, abs(float(q0 @ q1))))
    return math.degrees(2.0 * math.acos(dot))


def quaternion_slerp(
    first: typing.Sequence[float],
    second: typing.Sequence[float],
    fraction: float,
) -> list[float]:
    """Interpolate scalar-last quaternions over the shortest rotation."""
    q0 = np.asarray(first, dtype=np.float64).reshape(4)
    q1 = np.asarray(second, dtype=np.float64).reshape(4)
    q0 /= max(float(np.linalg.norm(q0)), 1e-12)
    q1 /= max(float(np.linalg.norm(q1)), 1e-12)
    t = max(0.0, min(1.0, float(fraction)))
    dot = float(q0 @ q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        out /= max(float(np.linalg.norm(out)), 1e-12)
        return [float(v) for v in out]
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    out = (
        math.sin((1.0 - t) * theta) / sin_theta * q0
        + math.sin(t * theta) / sin_theta * q1
    )
    return [float(v) for v in out]


class PoseTransitionStep(typing.NamedTuple):
    kind: str
    target_pose: list[float]
    remaining_position_m: float
    remaining_rotation_deg: float
    step_translation_m: float
    step_rotation_deg: float

    def to_dict(self) -> dict[str, typing.Any]:
        return {
            "kind": self.kind,
            "target_pose": self.target_pose,
            "remaining_position_m": round(self.remaining_position_m, 7),
            "remaining_rotation_deg": round(
                self.remaining_rotation_deg, 6
            ),
            "step_translation_m": round(self.step_translation_m, 7),
            "step_rotation_deg": round(self.step_rotation_deg, 6),
        }


class PoseTransitionController:
    """Stateful branch-stable pose transition controller.

    The translation phase keeps one immutable orientation reference. The
    rotation phase keeps one immutable position reference. This prevents tiny
    per-step errors from accumulating into a distant IK branch.
    """

    def __init__(
        self,
        target_pose: typing.Sequence[float],
        *,
        position_tolerance_m: float = 0.01,
        position_handoff_tolerance_m: float = 0.002,
        rotation_tolerance_deg: float = 3.0,
        max_position_step_m: float = DEFAULT_MAX_POSITION_STEP_M,
        max_rotation_step_deg: float = DEFAULT_MAX_ROTATION_STEP_DEG,
    ) -> None:
        self.target = _pose_array(target_pose)
        self.position_tolerance_m = max(0.0, float(position_tolerance_m))
        self.position_handoff_tolerance_m = min(
            self.position_tolerance_m,
            max(0.0, float(position_handoff_tolerance_m)),
        )
        self.rotation_tolerance_deg = max(
            0.0,
            float(rotation_tolerance_deg),
        )
        self.max_position_step_m = max(1e-6, float(max_position_step_m))
        self.max_rotation_step_deg = max(
            1e-6,
            float(max_rotation_step_deg),
        )
        self.phase = "translate"
        self.translation_reference_quaternion: list[float] | None = None
        self.rotation_reference_position: list[float] | None = None

    def next(
        self,
        current_pose: typing.Sequence[float],
    ) -> PoseTransitionStep:
        current = _pose_array(current_pose)
        position_error = float(np.linalg.norm(self.target[:3] - current[:3]))
        angular_error = rotation_error_deg(current, self.target)

        if self.phase == "translate":
            if self.translation_reference_quaternion is None:
                self.translation_reference_quaternion = [
                    float(v) for v in current[3:7]
                ]
            if position_error <= self.position_handoff_tolerance_m:
                self.phase = "rotate"
                self.rotation_reference_position = [
                    float(v) for v in current[:3]
                ]
            else:
                fraction = min(
                    1.0,
                    self.max_position_step_m / position_error,
                )
                position = current[:3] + (
                    self.target[:3] - current[:3]
                ) * fraction
                target_step = [
                    *position.tolist(),
                    *self.translation_reference_quaternion,
                ]
                return PoseTransitionStep(
                    kind="translate",
                    target_pose=[float(v) for v in target_step],
                    remaining_position_m=position_error,
                    remaining_rotation_deg=angular_error,
                    step_translation_m=position_error * fraction,
                    step_rotation_deg=0.0,
                )

        if self.rotation_reference_position is None:
            self.rotation_reference_position = [
                float(v) for v in current[:3]
            ]
        if angular_error > self.rotation_tolerance_deg:
            fraction = min(
                1.0,
                self.max_rotation_step_deg / max(angular_error, 1e-12),
            )
            quaternion = quaternion_slerp(
                current[3:7],
                self.target[3:7],
                fraction,
            )
            target_step = [
                *self.rotation_reference_position,
                *quaternion,
            ]
            return PoseTransitionStep(
                kind="rotate",
                target_pose=[float(v) for v in target_step],
                remaining_position_m=float(
                    np.linalg.norm(
                        self.target[:3]
                        - np.asarray(self.rotation_reference_position)
                    )
                ),
                remaining_rotation_deg=angular_error,
                step_translation_m=0.0,
                step_rotation_deg=angular_error * fraction,
            )

        return PoseTransitionStep(
            kind="complete",
            target_pose=[float(v) for v in self.target],
            remaining_position_m=position_error,
            remaining_rotation_deg=angular_error,
            step_translation_m=0.0,
            step_rotation_deg=0.0,
        )


def next_pose_transition_step(
    current_pose: typing.Sequence[float],
    target_pose: typing.Sequence[float],
    *,
    position_tolerance_m: float = 0.01,
    position_handoff_tolerance_m: float = 0.002,
    rotation_tolerance_deg: float = 3.0,
    max_position_step_m: float = DEFAULT_MAX_POSITION_STEP_M,
    max_rotation_step_deg: float = DEFAULT_MAX_ROTATION_STEP_DEG,
) -> PoseTransitionStep:
    """Return the next translation-only, rotation-only, or completion step."""
    current = _pose_array(current_pose)
    target = _pose_array(target_pose)
    position_error = float(np.linalg.norm(target[:3] - current[:3]))
    angular_error = rotation_error_deg(current, target)
    handoff_tolerance = min(
        max(0.0, float(position_tolerance_m)),
        max(0.0, float(position_handoff_tolerance_m)),
    )

    if position_error > handoff_tolerance:
        max_step = max(1e-6, float(max_position_step_m))
        fraction = min(1.0, max_step / position_error)
        position = current[:3] + (target[:3] - current[:3]) * fraction
        target_step = [*position.tolist(), *current[3:7].tolist()]
        return PoseTransitionStep(
            kind="translate",
            target_pose=[float(v) for v in target_step],
            remaining_position_m=position_error,
            remaining_rotation_deg=angular_error,
            step_translation_m=position_error * fraction,
            step_rotation_deg=0.0,
        )

    if angular_error > max(0.0, float(rotation_tolerance_deg)):
        max_step_deg = max(1e-6, float(max_rotation_step_deg))
        fraction = min(1.0, max_step_deg / max(angular_error, 1e-12))
        quaternion = quaternion_slerp(
            current[3:7],
            target[3:7],
            fraction,
        )
        target_step = [*current[:3].tolist(), *quaternion]
        return PoseTransitionStep(
            kind="rotate",
            target_pose=[float(v) for v in target_step],
            remaining_position_m=position_error,
            remaining_rotation_deg=angular_error,
            step_translation_m=0.0,
            step_rotation_deg=angular_error * fraction,
        )

    return PoseTransitionStep(
        kind="complete",
        target_pose=[float(v) for v in target],
        remaining_position_m=position_error,
        remaining_rotation_deg=angular_error,
        step_translation_m=0.0,
        step_rotation_deg=0.0,
    )
