"""Compose a safe SLAM observation pose from a head-camera 3-D target."""

from __future__ import annotations

import math
import typing

import numpy as np

from sdk_wrapper.dev.algorithms import geometry as geo

__all__ = [
    "plan_map_observation_pose",
    "plan_wrist_camera_pose",
    "end_effector_pose_for_camera",
    "plan_end_effector_grasp_pose",
    "base_point_to_map",
    "compose_map_pose_with_base_pose",
]


def base_point_to_map(
    point_base_m: typing.Sequence[float],
    map_from_base: typing.Sequence[float],
) -> list[float]:
    return [
        float(v)
        for v in geo.transform_point(point_base_m, map_from_base)
    ]


def compose_map_pose_with_base_pose(
    pose_base: typing.Sequence[float],
    map_from_base: typing.Sequence[float],
) -> list[float]:
    """Compose ``T_map_base @ T_base_goal`` into [x,y,z,qx,qy,qz,qw]."""
    base_mat = geo.extrinsic_to_matrix(pose_base)
    map_mat = geo.extrinsic_to_matrix(map_from_base)
    goal_map = map_mat @ base_mat
    qx, qy, qz, qw = geo.matrix_to_quaternion(goal_map[:3, :3])
    return [
        float(goal_map[0, 3]),
        float(goal_map[1, 3]),
        float(goal_map[2, 3]),
        qx,
        qy,
        qz,
        qw,
    ]


def plan_map_observation_pose(
    target_center_base_m: typing.Sequence[float],
    map_from_base: typing.Sequence[float],
    *,
    observation_distance_m: float = 0.8,
    min_observation_distance_m: float = 0.55,
    max_observation_distance_m: float = 1.50,
    min_approach_vector_m: float = 0.20,
) -> dict[str, typing.Any]:
    """Place the chassis on the robot->target line, looking at the target.

    The returned pose is a SLAM map-frame goal. It is not the object coordinate
    and must still pass ``check_path_reachability``.
    """
    target = [float(v) for v in target_center_base_m]
    if len(target) != 3 or not all(math.isfinite(v) for v in target):
        raise ValueError("target_center_base_m must contain 3 finite values")
    distance = float(observation_distance_m)
    reasons: list[str] = []
    if distance < float(min_observation_distance_m):
        raise ValueError("observation_distance_m is below the safety minimum")
    if distance > float(max_observation_distance_m):
        reasons.append("observation_distance_above_maximum")

    robot_xy = np.asarray([0.0, 0.0], dtype=np.float64)
    target_xy = np.asarray(target[:2], dtype=np.float64)
    target_vector = target_xy - robot_xy
    target_range = float(np.linalg.norm(target_vector))
    if target_range < float(min_approach_vector_m):
        reasons.append("target_too_close_to_compose_approach_direction")
        unit = np.asarray([1.0, 0.0])
    else:
        unit = target_vector / target_range

    observation_xy = target_xy - unit * distance
    yaw = math.atan2(
        float(target_xy[1] - observation_xy[1]),
        float(target_xy[0] - observation_xy[0]),
    )
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    pose_base = [
        float(observation_xy[0]),
        float(observation_xy[1]),
        0.0,
        0.0,
        0.0,
        qz,
        qw,
    ]
    pose_map = compose_map_pose_with_base_pose(pose_base, map_from_base)
    target_map = base_point_to_map(target, map_from_base)
    return {
        "safe_to_submit": not reasons,
        "reasons": reasons,
        "target_center_map_m": [round(v, 6) for v in target_map],
        "observation_pose_map": [round(v, 7) for v in pose_map],
        "observation_pose_base": [round(v, 7) for v in pose_base],
        "observation_distance_m": round(distance, 6),
        "facing_yaw_rad": round(yaw, 7),
        "grasp_coordinate": None,
    }


def _normalize(vec: np.ndarray) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        raise ValueError("zero-length vector")
    return arr / norm


def plan_wrist_camera_pose(
    target_center_base_m: typing.Sequence[float],
    surface_normal_base: typing.Sequence[float],
    long_axis_base: typing.Sequence[float],
    *,
    camera_standoff_m: float = 0.30,
    min_camera_standoff_m: float = 0.20,
    max_camera_standoff_m: float = 0.55,
) -> dict[str, typing.Any]:
    """Plan the wrist optical frame above the target, looking normal to it."""
    center = np.asarray(target_center_base_m, dtype=np.float64).reshape(3)
    normal = _normalize(surface_normal_base)
    if normal[2] < 0:
        normal = -normal
    z_axis = -normal  # OpenCV camera optical +z points at the surface.
    long_axis = np.asarray(long_axis_base, dtype=np.float64).reshape(3)
    x_axis = long_axis - float(long_axis @ z_axis) * z_axis
    if np.linalg.norm(x_axis) < 1e-9:
        raise ValueError("long axis is parallel to camera approach axis")
    x_axis = _normalize(x_axis)
    y_axis = _normalize(np.cross(z_axis, x_axis))
    rotation = np.column_stack([x_axis, y_axis, z_axis])
    standoff = float(camera_standoff_m)
    reasons: list[str] = []
    if standoff < float(min_camera_standoff_m):
        reasons.append("camera_standoff_below_minimum")
    if standoff > float(max_camera_standoff_m):
        reasons.append("camera_standoff_above_maximum")
    position = center + normal * standoff
    qx, qy, qz, qw = geo.matrix_to_quaternion(rotation)
    pose = [*position.tolist(), qx, qy, qz, qw]
    return {
        "safe_to_plan": not reasons,
        "reasons": reasons,
        "camera_pose_base": [round(float(v), 7) for v in pose],
        "camera_position_base": [round(float(v), 7) for v in position],
        "camera_forward_axis_base": [round(float(v), 8) for v in z_axis],
        "camera_right_axis_base": [round(float(v), 8) for v in x_axis],
        "camera_down_axis_base": [round(float(v), 8) for v in y_axis],
        "standoff_m": round(standoff, 6),
    }


def end_effector_pose_for_camera(
    desired_camera_pose_base: typing.Sequence[float],
    current_end_effector_pose_base: typing.Sequence[float],
    current_camera_pose_base: typing.Sequence[float],
) -> dict[str, typing.Any]:
    """Rigidly transfer a desired camera pose to the flange pose."""
    desired_cam = geo.extrinsic_to_matrix(desired_camera_pose_base)
    current_ee = geo.extrinsic_to_matrix(current_end_effector_pose_base)
    current_cam = geo.extrinsic_to_matrix(current_camera_pose_base)
    ee_from_cam = np.linalg.inv(current_ee) @ current_cam
    desired_ee = desired_cam @ np.linalg.inv(ee_from_cam)
    qx, qy, qz, qw = geo.matrix_to_quaternion(desired_ee[:3, :3])
    pose = [
        float(desired_ee[0, 3]),
        float(desired_ee[1, 3]),
        float(desired_ee[2, 3]),
        qx,
        qy,
        qz,
        qw,
    ]
    return {
        "end_effector_pose_base": [round(v, 7) for v in pose],
        "ee_from_camera": [round(float(v), 8) for v in ee_from_cam.reshape(-1)],
    }


def _rotation_error_deg_candidates(
    first: np.ndarray,
    second: np.ndarray,
) -> float:
    delta = first.T @ second
    cosine = max(-1.0, min(1.0, (float(np.trace(delta)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def plan_end_effector_grasp_pose(
    surface_center_base_m: typing.Sequence[float],
    surface_normal_base: typing.Sequence[float],
    long_edge_axis_base: typing.Sequence[float],
    *,
    standoff_m: float,
    current_end_effector_pose_base: typing.Sequence[float] | None = None,
    min_standoff_m: float = 0.02,
    max_standoff_m: float = 0.30,
) -> dict[str, typing.Any]:
    """Plan a top-down pre-grasp pose from a validated charger-top surface.

    The ``EndEffector`` convention used by this project is:

    * local +x: gripper long axis;
    * local -y: jaw closing axis (the physical axis is sign-symmetric);
    * local +z: approach direction, pointing from the gripper toward the top.

    The target remains above the surface by ``standoff_m``.  The gripper long
    axis has a 180-degree symmetry for this grasp, so when the current
    end-effector pose is known we choose the equivalent yaw that needs less
    rotation.
    """
    center = np.asarray(surface_center_base_m, dtype=np.float64).reshape(3)
    normal = _normalize(surface_normal_base)
    if normal[2] < 0:
        normal = -normal

    long_axis = np.asarray(long_edge_axis_base, dtype=np.float64).reshape(3)
    long_axis = long_axis - float(long_axis @ normal) * normal
    if np.linalg.norm(long_axis) < 1e-9:
        raise ValueError("long edge axis is parallel to the surface normal")
    long_axis = _normalize(long_axis)
    closing_axis = _normalize(np.cross(normal, long_axis))

    try:
        standoff = float(standoff_m)
    except (TypeError, ValueError) as exc:
        raise ValueError("standoff_m must be numeric") from exc
    reasons: list[str] = []
    if not math.isfinite(standoff):
        reasons.append("standoff_not_finite")
    elif standoff < float(min_standoff_m):
        reasons.append("standoff_below_minimum")
    elif standoff > float(max_standoff_m):
        reasons.append("standoff_above_maximum")

    approach_axis = -normal
    # [long, -closing, -normal] is a right-handed frame whose local +z looks
    # toward the surface and local y crosses the charger's short dimension.
    candidates = [
        np.column_stack([long_axis, -closing_axis, approach_axis]),
        np.column_stack([-long_axis, closing_axis, approach_axis]),
    ]
    selected_index = 0
    current_rotation_error_deg: float | None = None
    if current_end_effector_pose_base is not None:
        current_pose = np.asarray(
            current_end_effector_pose_base, dtype=np.float64
        ).reshape(-1)
        if current_pose.size == 7 and all(math.isfinite(v) for v in current_pose):
            current_rotation = geo.quaternion_to_matrix(*current_pose[3:7])
            errors = [
                _rotation_error_deg_candidates(candidate, current_rotation)
                for candidate in candidates
            ]
            selected_index = int(np.argmin(errors))
            current_rotation_error_deg = round(errors[selected_index], 6)

    rotation = candidates[selected_index]
    position = center + normal * (standoff if math.isfinite(standoff) else 0.0)
    qx, qy, qz, qw = geo.matrix_to_quaternion(rotation)
    pose = [*position.tolist(), qx, qy, qz, qw]
    selected_long = rotation[:, 0]
    selected_closing = -rotation[:, 1]
    return {
        "safe_to_plan": not reasons,
        "reasons": reasons,
        "frame": "base_link",
        "target_kind": "wrist_charger_pregrasp",
        "surface_center_m": [round(float(v), 7) for v in center],
        "surface_normal_base": [round(float(v), 8) for v in normal],
        "approach_axis_base": [round(float(v), 8) for v in approach_axis],
        "gripper_long_axis_base": [round(float(v), 8) for v in selected_long],
        "jaw_closing_axis_base": [round(float(v), 8) for v in selected_closing],
        "pre_grasp_position_m": [round(float(v), 7) for v in position],
        "pre_grasp_pose": [round(float(v), 7) for v in pose],
        "standoff_m": round(standoff, 7) if math.isfinite(standoff) else None,
        "selected_long_axis_sign": 1.0 if selected_index == 0 else -1.0,
        "rotation_from_current_deg": current_rotation_error_deg,
        "end_effector_axis_convention": {
            "x": "gripper_long_axis",
            "y": "negative_jaw_closing_axis",
            "z": "approach_axis_toward_surface",
        },
    }
