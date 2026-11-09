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
    "plan_diagonal_observation_pose",
    "plan_grasp_descent_pose",
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

    The G1 ``EndEffector`` convention verified against the wrist camera and
    attached gripper TCP is:

    * local +x: approach direction, pointing from the flange toward the TCP;
    * local y: jaw closing axis;
    * local z: gripper long axis.

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
    # The attached gripper TCP lies on local +x. The wrist optical +z also
    # aligns with local +x, so approach is local +x toward -surface_normal.
    # Flip both jaw-closing and long axes together for the equivalent 180-deg
    # rotation around the approach axis.
    candidates = [
        np.column_stack([approach_axis, closing_axis, long_axis]),
        np.column_stack([approach_axis, -closing_axis, -long_axis]),
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
    selected_long = rotation[:, 2]
    selected_closing = rotation[:, 1]
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
            "x": "approach_axis_toward_surface",
            "y": "jaw_closing_axis",
            "z": "gripper_long_axis",
        },
    }


def plan_diagonal_observation_pose(
    surface_center_base_m: typing.Sequence[float],
    surface_normal_base: typing.Sequence[float],
    long_edge_axis_base: typing.Sequence[float],
    *,
    observation_distance_m: float = 0.05,
    tilt_from_normal_deg: float = 35.0,
    current_end_effector_pose_base: typing.Sequence[float] | None = None,
    min_observation_distance_m: float = 0.03,
    max_observation_distance_m: float = 0.08,
    tool_approach_offset_m: float = 0.0,
) -> dict[str, typing.Any]:
    """Plan a close diagonal observation pose whose approach ray hits center."""
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

    distance = float(observation_distance_m)
    tilt_deg = float(tilt_from_normal_deg)
    tool_offset = float(tool_approach_offset_m)
    reasons: list[str] = []
    if not math.isfinite(distance):
        reasons.append("observation_distance_not_finite")
        distance = min(max(0.0, float(min_observation_distance_m)), 0.05)
    elif distance < float(min_observation_distance_m):
        reasons.append("observation_distance_below_minimum")
    elif distance > float(max_observation_distance_m):
        reasons.append("observation_distance_above_maximum")
    if not math.isfinite(tilt_deg) or not 0.0 < tilt_deg < 85.0:
        reasons.append("observation_tilt_invalid")
        tilt_deg = 35.0
    if not math.isfinite(tool_offset) or not 0.0 <= tool_offset <= 0.25:
        reasons.append("tool_approach_offset_invalid")
        tool_offset = 0.0

    lateral_sign = 1.0
    if current_end_effector_pose_base is not None:
        current_pose = np.asarray(
            current_end_effector_pose_base, dtype=np.float64
        ).reshape(-1)
        if current_pose.size == 7 and all(math.isfinite(v) for v in current_pose):
            radial = current_pose[:3] - center
            projection = float(radial @ closing_axis)
            if abs(projection) > 1e-6:
                lateral_sign = 1.0 if projection >= 0.0 else -1.0

    radial_direction = float(np.cos(math.radians(tilt_deg)))
    lateral_amount = float(np.sin(math.radians(tilt_deg)))
    offset_direction = _normalize(
        normal * radial_direction
        + closing_axis * lateral_sign * lateral_amount
    )
    approach_axis = -offset_direction
    tcp_position = center + offset_direction * distance
    position = tcp_position - approach_axis * tool_offset
    aligned_long = long_axis - float(long_axis @ approach_axis) * approach_axis
    if np.linalg.norm(aligned_long) < 1e-9:
        raise ValueError("long edge axis is parallel to observation approach")
    aligned_long = _normalize(aligned_long)
    aligned_closing = _normalize(np.cross(aligned_long, approach_axis))

    candidates = [
        np.column_stack([approach_axis, aligned_closing, aligned_long]),
        np.column_stack([approach_axis, -aligned_closing, -aligned_long]),
    ]
    selected_index = 0
    rotation_error_deg: float | None = None
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
            rotation_error_deg = round(errors[selected_index], 6)

    rotation = candidates[selected_index]
    qx, qy, qz, qw = geo.matrix_to_quaternion(rotation)
    pose = [*position.tolist(), qx, qy, qz, qw]
    return {
        "safe_to_plan": not reasons,
        "reasons": reasons,
        "frame": "base_link",
        "target_kind": "wrist_charger_diagonal_observation",
        "surface_center_m": [round(float(v), 7) for v in center],
        "surface_normal_base": [round(float(v), 8) for v in normal],
        "observation_position_m": [round(float(v), 7) for v in position],
        "tcp_observation_position_m": [
            round(float(v), 7) for v in tcp_position
        ],
        "observation_pose": [round(float(v), 7) for v in pose],
        "observation_distance_m": round(distance, 7),
        "tool_approach_offset_m": round(tool_offset, 7),
        "tilt_from_normal_deg": round(tilt_deg, 6),
        "offset_direction_base": [round(float(v), 8) for v in offset_direction],
        "approach_axis_base": [round(float(v), 8) for v in approach_axis],
        "gripper_long_axis_base": [
            round(float(v), 8) for v in rotation[:, 2]
        ],
        "jaw_closing_axis_base": [
            round(float(v), 8) for v in rotation[:, 1]
        ],
        "selected_long_axis_sign": 1.0 if selected_index == 0 else -1.0,
        "rotation_from_current_deg": rotation_error_deg,
        "camera_visibility": (
            "wrist optical +z follows local +x and points at surface center"
        ),
    }


def plan_grasp_descent_pose(
    surface_center_base_m: typing.Sequence[float],
    long_edge_axis_base: typing.Sequence[float],
    current_end_effector_pose_base: typing.Sequence[float],
    *,
    clearance_m: float = 0.01,
    min_clearance_m: float = 0.002,
    max_clearance_m: float = 0.05,
    max_descent_m: float = 0.12,
    tool_approach_offset_m: float = 0.0,
) -> dict[str, typing.Any]:
    """Plan a short descent along the current diagonal approach axis."""
    center = np.asarray(surface_center_base_m, dtype=np.float64).reshape(3)
    current_pose = np.asarray(
        current_end_effector_pose_base, dtype=np.float64
    ).reshape(-1)
    if current_pose.size != 7 or not all(math.isfinite(v) for v in current_pose):
        raise ValueError("current pose must contain 7 finite values")
    current_rotation = geo.quaternion_to_matrix(*current_pose[3:7])
    approach_axis = _normalize(current_rotation[:, 0])

    long_axis = np.asarray(long_edge_axis_base, dtype=np.float64).reshape(3)
    long_axis = long_axis - float(long_axis @ approach_axis) * approach_axis
    if np.linalg.norm(long_axis) < 1e-9:
        raise ValueError("long edge axis is parallel to approach axis")
    long_axis = _normalize(long_axis)
    if float(long_axis @ current_rotation[:, 2]) < 0.0:
        long_axis = -long_axis
    closing_axis = _normalize(np.cross(long_axis, approach_axis))
    rotation = np.column_stack([approach_axis, closing_axis, long_axis])

    clearance = float(clearance_m)
    tool_offset = float(tool_approach_offset_m)
    tcp_target = center - approach_axis * clearance
    target = tcp_target - approach_axis * tool_offset
    delta = target - current_pose[:3]
    descent_m = float(np.linalg.norm(delta))
    surface_ahead_m = float((center - current_pose[:3]) @ approach_axis)
    reasons: list[str] = []
    if not math.isfinite(clearance):
        reasons.append("descent_clearance_not_finite")
    elif clearance < float(min_clearance_m):
        reasons.append("descent_clearance_below_minimum")
    elif clearance > float(max_clearance_m):
        reasons.append("descent_clearance_above_maximum")
    if surface_ahead_m <= 0.0:
        reasons.append("surface_not_ahead_along_approach_axis")
    if descent_m > float(max_descent_m):
        reasons.append("descent_distance_above_maximum")
    if not math.isfinite(tool_offset) or not 0.0 <= tool_offset <= 0.25:
        reasons.append("tool_approach_offset_invalid")

    qx, qy, qz, qw = geo.matrix_to_quaternion(rotation)
    pose = [*target.tolist(), qx, qy, qz, qw]
    return {
        "safe_to_plan": not reasons,
        "reasons": reasons,
        "frame": "base_link",
        "target_kind": "wrist_charger_descent",
        "surface_center_m": [round(float(v), 7) for v in center],
        "descent_pose": [round(float(v), 7) for v in pose],
        "descent_position_m": [round(float(v), 7) for v in target],
        "tcp_descent_position_m": [round(float(v), 7) for v in tcp_target],
        "descent_vector_base": [round(float(v), 7) for v in delta],
        "descent_distance_m": round(descent_m, 7),
        "surface_ahead_m": round(surface_ahead_m, 7),
        "clearance_m": round(clearance, 7),
        "tool_approach_offset_m": round(tool_offset, 7),
        "approach_axis_base": [round(float(v), 8) for v in approach_axis],
        "gripper_long_axis_base": [round(float(v), 8) for v in long_axis],
        "jaw_closing_axis_base": [round(float(v), 8) for v in closing_axis],
    }
