"""Pure geometry and gates for re-gripping the open drawer handle, then closing it.

No hardware access here. The right arm travels back to the recorded *release*
pose (the TCP at which the gripper let go of the handle after the 30 cm pull),
closes on the handle again, and then pushes along the recorded pull path in
reverse. Every tolerance is explicit so the caller can refuse to move instead
of guessing.

Why the recorded path and not a straight line: the pull record shows the TCP
sagging 17 mm in Z and drifting 7 mm in Y over 30 cm of travel. Reversing that
observed curve keeps the push roughly along the real rail instead of loading
the handle sideways.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

from medicine_frontend.dataset import DEFAULT_ROOT


WORKSPACE = Path(__file__).resolve().parents[1]
RECIPE_PATH = (WORKSPACE / "galbot-hackathon/scenes/scene_medicine_check/data"
               / "recorded_drawer_regrip_close_20261107.json")
SCHEMA = "drawer_regrip_close_recipe/1"
RIGHT_ARM_JOINTS = tuple(f"right_arm_joint{i}" for i in range(1, 8))

APPROACH_CONFIRMATIONS = ("handle_clear", "path_clear", "exclusive_control",
                          "emergency_stop_ready", "operator_present")
GRIP_CONFIRMATIONS = ("handle_between_fingers_confirmed", "operator_hands_clear_confirmed",
                      "emergency_stop_ready")
PUSH_CONFIRMATIONS = ("drawer_supported", "path_clear", "operator_present",
                      "emergency_stop_ready")

# "Open enough to straddle the handle", not "exactly at the recorded 50 mm":
# a gripper left at its full stroke reports ~85 mm and is a valid start state.
OPEN_GRIPPER_RANGE_M = (0.040, 0.120)
GRIPPED_RANGE_M = (0.020, 0.030)


# ---------------------------------------------------------------- quaternions

def _finite(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _qnorm(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(v * v for v in q))
    if not 0.5 <= norm <= 1.5 or not all(_finite(v) for v in q):
        raise ValueError("Invalid quaternion")
    return [v / norm for v in q]


def _qmul(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def _qconj(q: list[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def angle_deg(left: list[float], right: list[float]) -> float:
    """Shortest angle between two orientations, in degrees."""
    dot = abs(sum(float(a) * float(b) for a, b in zip(_qnorm(left), _qnorm(right))))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))


def _matrix(q: list[float]) -> list[list[float]]:
    x, y, z, w = _qnorm(q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rotation_error_base(current: list[float], target: list[float]) -> list[float]:
    """Rotation vector (base_link axes, radians) turning `current` into `target`."""
    delta = _qnorm(_qmul(_qconj(_qnorm(current)), _qnorm(target)))
    if delta[3] < 0:
        delta = [-delta[0], -delta[1], -delta[2], -delta[3]]
    vec = delta[:3]
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0]
    axis = [v / norm for v in vec]
    angle = 2 * math.atan2(norm, max(-1.0, min(1.0, delta[3])))
    matrix = _matrix(current)  # tool-frame rotation vector -> base_link
    return [angle * sum(matrix[row][col] * axis[col] for col in range(3))
            for row in range(3)]


def yaw_deg(base: dict[str, Any]) -> float:
    qx, qy, qz, qw = (float(base[key]) for key in ("qx", "qy", "qz", "qw"))
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                   1 - 2 * (qy * qy + qz * qz)))


# -------------------------------------------------------------------- recipe

def make_recipe(source_state_path: Path, closed_tcp_base_link: list[float], *,
                standoff_m: float = 0.09, checkpoints: list[list[float]] | None = None,
                reference_wrist_image: Path | None = None,
                wrist_image_rotation_quarter_turns: int = 1) -> dict[str, Any]:
    """Build the recipe from a recorded pull `final_state.json`."""
    raw = source_state_path.read_bytes()
    state = json.loads(raw)
    if state.get("errors"):
        raise ValueError("Source release state has read errors")
    release = state["reads"]["right_end_effector"]["data"]["pose"]
    base = state["reads"]["navigation"]["data"]["pose"]
    joints = {entry["name"]: entry["position"]
              for entry in state["reads"]["robot"]["data"]["joint_state"]}
    sensors = [item["event"] for item in state["events"]
               if item.get("event", {}).get("type") == "robot_sensors"]
    if not sensors:
        raise ValueError("Source release state has no gripper feedback")
    grip = sensors[-1]["data"]["grippers"]["right"]
    previous = json.loads((WORKSPACE / "galbot-hackathon/scenes/scene_medicine_check/data/"
                           "recorded_drawer_pull_20261107.json").read_text(encoding="utf-8"))
    path = [[*list(point[:3]), *list(closed_tcp_base_link[3:])]
            for point in (checkpoints or [])] or None
    if path is None:
        points = list(reversed(previous["observed_tcp_checkpoints_base_link"]))
        path = [[*list(point[:3]), *list(closed_tcp_base_link[3:])] for point in points]
    path[0] = list(release)
    if math.dist(path[-1][:3], closed_tcp_base_link[:3]) > 0.005:
        path.append(list(closed_tcp_base_link))  # keep the last measured waypoint
    else:
        path[-1] = list(closed_tcp_base_link)
    if any(path[i + 1][0] <= path[i][0] for i in range(len(path) - 1)):
        raise ValueError("Recorded push path X must increase monotonically")
    recipe = {
        "schema": SCHEMA,
        "source_session": source_state_path.parent.name,
        "source_state_path": str(source_state_path.relative_to(DEFAULT_ROOT)).replace("\\", "/"),
        "source_state_sha256": hashlib.sha256(raw).hexdigest(),
        "release_tcp_base_link": list(release),
        "release_base_map": dict(base),
        "release_chassis_joints_rad": {name: value for name, value in joints.items()
                                       if name.startswith("chassis_joint")},
        "release_right_arm_joints_rad": {name: joints[name] for name in RIGHT_ARM_JOINTS},
        "release_right_gripper": dict(grip),
        "closed_tcp_base_link": list(closed_tcp_base_link),
        "push_path_tcp_base_link": path,
        "approach_standoff_m": standoff_m,
        "grip_command": {"side": "right", "width": 0.02, "speed": 0.01, "effort": 10.0},
        "open_command": {"side": "right", "width": 0.05, "speed": 0.02, "effort": 5.0},
        "reference_wrist_image": None if reference_wrist_image is None else
        str(reference_wrist_image.relative_to(DEFAULT_ROOT)).replace("\\", "/"),
        "wrist_image_rotation_quarter_turns": wrist_image_rotation_quarter_turns % 4,
        "wrist_camera_note": "mounted 90 deg rotated: the horizontal handle bar appears "
                             "vertical in raw wrist frames",
        "frame_id": "base_link",
        "control_semantics": "short pose-goal offsets, verified after every command",
        "verified_on_robot": False,
        "limitations": [
            "Release pose is the recorded TCP before the gripper opened; if the drawer "
            "moved since then, the handle is no longer at this pose.",
            "Base repeatability after driving away and back is unverified; base_link "
            "poses are only valid within the recorded base_map tolerance.",
            "No independent drawer travel measurement exists; TCP advance plus intact "
            "grip is the proxy, confirmed visually by the operator at the end.",
        ],
    }
    return recipe


def load_recipe(path: Path = RECIPE_PATH, dataset_root: Path = DEFAULT_ROOT) -> dict[str, Any]:
    recipe = json.loads(Path(path).read_text(encoding="utf-8"))
    if recipe.get("schema") != SCHEMA:
        raise ValueError("Unknown drawer re-grip recipe schema")
    source = Path(dataset_root) / recipe["source_state_path"]
    if (not source.is_file()
            or hashlib.sha256(source.read_bytes()).hexdigest() != recipe["source_state_sha256"]):
        raise ValueError("Release-pose source state is missing or changed")
    release, closed = recipe["release_tcp_base_link"], recipe["closed_tcp_base_link"]
    for pose in (release, closed):
        if (len(pose) != 7 or not all(_finite(v) for v in pose)
                or not 0.95 <= math.sqrt(sum(v * v for v in pose[3:])) <= 1.05):
            raise ValueError("Recipe pose is not a valid 7-element pose")
    path_points = recipe["push_path_tcp_base_link"]
    if (len(path_points) < 3 or any(len(point) != 7 for point in path_points)
            or any(path_points[i + 1][0] <= path_points[i][0]
                   for i in range(len(path_points) - 1))):
        raise ValueError("Recipe push path is invalid or not monotonic in X")
    if math.dist(path_points[0][:3], release[:3]) > 1e-4:
        raise ValueError("Push path does not start at the release pose")
    if math.dist(path_points[-1][:3], closed[:3]) > 1e-4:
        raise ValueError("Push path does not end at the closed pose")
    if closed[0] - release[0] < 0.05:
        raise ValueError("Re-grip and close is pointless: travel under 50 mm")
    if not 0.0 < float(recipe["approach_standoff_m"]) <= 0.15:
        raise ValueError("Unsupported approach standoff")
    if recipe.get("wrist_image_rotation_quarter_turns", 0) % 4 not in (0, 1, 2, 3):
        raise ValueError("Unsupported wrist image rotation")
    return recipe


# ------------------------------------------------------------------ geometry

def release_pose(recipe: dict[str, Any]) -> list[float]:
    return list(recipe["release_tcp_base_link"])


def closed_pose(recipe: dict[str, Any]) -> list[float]:
    return list(recipe["closed_tcp_base_link"])


def travel_to_closed(recipe: dict[str, Any]) -> float:
    return closed_pose(recipe)[0] - release_pose(recipe)[0]


def recorded_travel_mm(recipe: dict[str, Any]) -> float:
    return round(travel_to_closed(recipe) * 1000.0, 1)


def stroke_issues(measured_mm: float | None, recipe: dict[str, Any], *,
                  tolerance_mm: float = 15.0) -> list[str]:
    """Reconcile the recorded stroke with the drawer the operator measured.

    The push path ends wherever the pull started, so an over-long stroke pushes
    the front panel into the cabinet instead of stopping at it. The real drawer
    stroke has only ever been estimated (25 cm in the handoff notes) while this
    recipe encodes 30.1 cm of TCP travel, so the measurement decides who is
    right - before anything moves, not after.
    """
    if measured_mm is None:
        return []
    if not float(measured_mm) > 0:
        return ["measured drawer travel must be a positive millimetre value"]
    recorded = recorded_travel_mm(recipe)
    gap = abs(float(measured_mm) - recorded)
    if gap > tolerance_mm:
        return [f"measured drawer travel {float(measured_mm):.0f} mm disagrees with the "
                f"recorded {recorded:.0f} mm stroke (limit +/-{tolerance_mm:.0f} mm); the "
                f"recorded release pose is probably not this drawer's at-rest handle"]
    return []


def push_segments(recipe: dict[str, Any]) -> list[dict[str, Any]]:
    """Segments from the release pose to the closed pose, one command each."""
    points = recipe["push_path_tcp_base_link"]
    closed_x = closed_pose(recipe)[0]
    segments = []
    for index in range(1, len(points)):
        start, end = list(points[index - 1]), list(points[index])
        remaining = closed_x - start[0]
        segments.append({
            "index": index,
            "from_pose": start,
            "to_pose": end,
            "dx": end[0] - start[0],
            "dy": end[1] - start[1],
            "dz": end[2] - start[2],
            "travelled_before_m": start[0] - release_pose(recipe)[0],
            "remaining_to_closed_m": remaining,
            "near_closed": remaining <= 0.035,
        })
    return segments


def standoff_pose(recipe: dict[str, Any]) -> list[float]:
    pose = release_pose(recipe)
    return [pose[0] - float(recipe["approach_standoff_m"]), pose[1], pose[2], *pose[3:]]


def approach_command(current: list[float], recipe: dict[str, Any], *,
                     allow_rotation: bool = False, max_step_m: float = 0.060,
                     engage_step_m: float = 0.020, max_rotation_rad: float = 0.10,
                     clear_lift_m: float = 0.040,
                     max_manual_jog_deg: float = 45.0) -> dict[str, Any]:
    """One bounded offset that walks the TCP to the release pose from the robot side.

    Zones by TCP X, because the drawer front panel and the handle plane are what
    the fingers must not cross while descending or sliding sideways:

    ``clear``   TCP is at or beyond the handle plane: lift 40 mm and back off to
                the standoff before anything else.
    ``align``   TCP is behind the standoff: align Y/Z at standoff X, where the
                finger tips are still clear of the handle.
    ``engage``  TCP is between standoff and handle: advance at 20 mm per command.

    Rotation is only sent when the operator explicitly allows it: the console
    rotation semantics are unverified, so the caller must re-check the observed
    orientation afterwards.
    """
    if len(current) != 7 or not all(_finite(v) for v in current):
        raise ValueError("Current TCP is not a valid pose")
    target = release_pose(recipe)
    standoff = standoff_pose(recipe)
    orientation_gap = angle_deg(current[3:], target[3:])
    if orientation_gap > max_manual_jog_deg:
        raise ValueError("orientation_gap_over_45deg:jog the arm manually first")
    if orientation_gap > 6.0 and not allow_rotation:
        raise ValueError("orientation_gap_requires_--allow-rotation")

    if current[0] > target[0] + 0.005:
        mode = "clear"
        goal = [standoff[0], target[1], target[2] + clear_lift_m, *standoff[3:]]
        limit = max_step_m
    elif current[0] < standoff[0] + 0.005:
        mode = "align"
        goal = standoff
        limit = max_step_m
    else:
        mode = "engage"
        goal = target
        limit = engage_step_m
    if mode == "align" and math.dist(current[:3], standoff[:3]) < 0.003:
        mode, goal, limit = "engage", target, engage_step_m  # aligned; stop stalling

    error = [goal[axis] - current[axis] for axis in range(3)]
    length = math.sqrt(sum(v * v for v in error))
    if length <= 1e-6:
        position = list(current[:3])
        clamped = False
    else:
        scale = min(1.0, limit / length)
        position = [current[axis] + error[axis] * scale for axis in range(3)]
        clamped = scale < 1.0

    rotvec = rotation_error_base(current[3:], target[3:])
    rotation = [0.0, 0.0, 0.0]
    rotation_clamped = False
    if allow_rotation:
        magnitude = math.sqrt(sum(v * v for v in rotvec))
        if magnitude > 1e-6:
            scale = min(1.0, max_rotation_rad / magnitude)
            rotation = [v * scale for v in rotvec]
            rotation_clamped = scale < 1.0

    command = {"arm": "right_arm",
               "dx": position[0] - current[0], "dy": position[1] - current[1],
               "dz": position[2] - current[2],
               "roll": rotation[0], "pitch": rotation[1], "yaw": rotation[2],
               "confirmed": True}
    if max(abs(command[key]) for key in ("dx", "dy", "dz")) > max_step_m + 1e-9:
        raise ValueError("Calculated approach offset exceeds the step limit")
    if max(abs(v) for v in rotation) > max_rotation_rad + 1e-9:
        raise ValueError("Calculated rotation offset exceeds the step limit")
    return {"mode": mode, "goal_pose": goal, "target_position": position,
            "position_step_m": math.dist(position, current[:3]),
            "clamped": clamped, "orientation_error_deg": orientation_gap,
            "rotation_rad": rotation, "rotation_clamped": rotation_clamped,
            "distance_to_release_m": math.dist(current[:3], target[:3]),
            "command": command}


# --------------------------------------------------------------------- gates

def base_return_issues(state: dict[str, Any], recipe: dict[str, Any], *,
                       position_tolerance_m: float = 0.030,
                       yaw_tolerance_deg: float = 2.0,
                       chassis_tolerance_rad: float = 0.005) -> list[str]:
    """The base drove away and must be back where the pull was recorded."""
    issues = []
    record = recipe["release_base_map"]
    base = state["base_map"]
    shift = math.hypot(base["x"] - record["x"], base["y"] - record["y"])
    if shift > position_tolerance_m:
        issues.append(
            f"base is {shift * 1000:.0f} mm from the recorded release position "
            f"(limit {position_tolerance_m * 1000:.0f} mm)")
    drift = abs(yaw_deg(base) - yaw_deg(record))
    drift = min(drift, 360 - drift)
    if drift > yaw_tolerance_deg:
        issues.append(f"base yaw differs by {drift:.1f} deg (limit {yaw_tolerance_deg:.1f})")
    recorded = recipe["release_chassis_joints_rad"]
    current = state["chassis_joints_rad"]
    if set(current) != set(recorded):
        issues.append("chassis joint set differs from the recorded release pose")
    else:
        worst = max(abs(current[name] - value) for name, value in recorded.items())
        if worst > chassis_tolerance_rad:
            issues.append(f"chassis joints moved {worst:.4f} rad (limit "
                          f"{chassis_tolerance_rad:.3f})")
    return issues


# ------------------------------------------------- re-localising onto a moved base

REBASE_SHIFT_LIMIT_M = 0.25
REBASE_YAW_LIMIT_DEG = 10.0
REBASE_REACH_X_RANGE_M = (0.35, 0.85)
# Lateral reach is much wider than the forward band; the frontal-approach
# requirement is already carried by the yaw limit, not by this number.
REBASE_LATERAL_LIMIT_M = 0.45


def base_delta(record: dict[str, Any], current: dict[str, Any]) -> dict[str, float]:
    dyaw = (yaw_deg(current) - yaw_deg(record) + 180.0) % 360.0 - 180.0
    return {"dx_m": current["x"] - record["x"], "dy_m": current["y"] - record["y"],
            "shift_m": math.hypot(current["x"] - record["x"], current["y"] - record["y"]),
            "dyaw_deg": dyaw}


def rebase_pose(pose: list[float], record: dict[str, Any],
                current: dict[str, Any]) -> list[float]:
    """Carry a base_link pose from the recorded base pose to the current one.

    `p_map = R(yaw0) * p + t0`, so `p1 = R(yaw1)^-1 * (p_map - t1)`. The chassis
    only yaws about the map z axis, so z, roll and pitch are untouched. This is
    exact geometry, not a tolerance relaxation: the requirement moves from
    "drive back to the same millimetre" to "localisation is trustworthy and the
    drawer has not moved in the map".
    """
    yaw0 = math.radians(yaw_deg(record))
    yaw1 = math.radians(yaw_deg(current))
    turn = yaw0 - yaw1
    cos_t, sin_t = math.cos(turn), math.sin(turn)
    dx = float(record["x"]) - float(current["x"])
    dy = float(record["y"]) - float(current["y"])
    cos_r, sin_r = math.cos(-yaw1), math.sin(-yaw1)
    shift = [cos_r * dx - sin_r * dy, sin_r * dx + cos_r * dy]
    x, y, z = (float(v) for v in pose[:3])
    position = [cos_t * x - sin_t * y + shift[0], sin_t * x + cos_t * y + shift[1], z]
    spin = [0.0, 0.0, math.sin(turn / 2), math.cos(turn / 2)]
    return position + _qnorm(_qmul(spin, _qnorm([float(v) for v in pose[3:]])))


def rebase_recipe(recipe: dict[str, Any], current_base_map: dict[str, Any]) -> dict[str, Any]:
    import copy

    rebased = copy.deepcopy(recipe)
    record = recipe["release_base_map"]
    for key in ("release_tcp_base_link", "closed_tcp_base_link"):
        rebased[key] = rebase_pose(recipe[key], record, current_base_map)
    rebased["push_path_tcp_base_link"] = [rebase_pose(point, record, current_base_map)
                                         for point in recipe["push_path_tcp_base_link"]]
    rebased["rebase"] = {"from_base_map": dict(record), "to_base_map": dict(current_base_map),
                         "delta": base_delta(record, current_base_map),
                         "assumption": "drawer unchanged in map; localisation trustworthy"}
    rebased["release_base_map"] = dict(current_base_map)
    rebased["verified_on_robot"] = False
    return rebased


def resolve_recipe_for_base(recipe: dict[str, Any], current_base_map: dict[str, Any], *,
                            strict_shift_m: float = 0.030, strict_yaw_deg: float = 2.0,
                            shift_limit_m: float = REBASE_SHIFT_LIMIT_M,
                            yaw_limit_deg: float = REBASE_YAW_LIMIT_DEG
                            ) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    """Pick the recipe to run at the current base pose, plus the blocking issues.

    Inside the strict tolerance the original recipe is used untouched. Outside it,
    the whole recipe is re-expressed in the current base frame and gated on arm
    reach and approach direction instead of return-to-pose accuracy.
    """
    delta = base_delta(recipe["release_base_map"], current_base_map)
    info: dict[str, Any] = {"delta": delta, "rebased": False}
    if delta["shift_m"] <= strict_shift_m and abs(delta["dyaw_deg"]) <= strict_yaw_deg:
        return recipe, info, []
    issues = []
    if delta["shift_m"] > shift_limit_m:
        issues.append(f"base is {delta['shift_m'] * 1000:.0f} mm from the recorded pose; "
                      f"re-localising only supports {shift_limit_m * 1000:.0f} mm")
    if abs(delta["dyaw_deg"]) > yaw_limit_deg:
        issues.append(f"base yaw differs by {abs(delta['dyaw_deg']):.1f} deg; "
                      f"re-localising only supports {yaw_limit_deg:.1f} deg")
    active = rebase_recipe(recipe, current_base_map)
    release = release_pose(active)
    if not REBASE_REACH_X_RANGE_M[0] <= release[0] <= REBASE_REACH_X_RANGE_M[1]:
        issues.append(f"re-localised release TCP X {release[0]:.3f} m is outside "
                      f"{REBASE_REACH_X_RANGE_M}")
    if abs(release[1]) > REBASE_LATERAL_LIMIT_M:
        issues.append(f"re-localised release TCP Y {release[1]:.3f} m is too far to the side "
                      f"(limit {REBASE_LATERAL_LIMIT_M} m)")
    if standoff_pose(active)[0] < 0.15:
        issues.append("re-localised approach standoff would put the wrist inside the body")
    if max(point[0] for point in active["push_path_tcp_base_link"]) > REBASE_REACH_X_RANGE_M[1]:
        issues.append("re-localised push path exceeds arm reach")
    info.update({"rebased": True, "release_tcp_base_link": release})
    if issues:
        return recipe, info, issues
    return active, info, []


def prepare_issues(state: dict[str, Any], controller: dict[str, Any],
                   recipe: dict[str, Any], confirmations: dict[str, bool], *,
                   check_base: bool = True) -> list[str]:
    """Read-only start gate: control authority, open gripper, arm roughly in place.

    `check_base=False` is used by the tool after it has already resolved the base
    offset through `resolve_recipe_for_base`.
    """
    issues = []
    if not (controller.get("group") == "right_arm"
            and str(controller.get("active", "")).lower() == "right_arm_pvt_ctrl"
            and controller.get("dry_run") is False):
        issues.append("right_arm_pvt_ctrl is not active (teleop or unknown controller)")
    grip = state["right_gripper"]
    if not OPEN_GRIPPER_RANGE_M[0] <= grip["width"] <= OPEN_GRIPPER_RANGE_M[1]:
        issues.append(f"right gripper width {grip['width']:.3f} m is not open enough "
                      f"(need >= {OPEN_GRIPPER_RANGE_M[0]:.3f}, plausible < "
                      f"{OPEN_GRIPPER_RANGE_M[1]:.3f})")
    if grip["is_moving"]:
        issues.append("right gripper is moving")
    if check_base:
        issues.extend(base_return_issues(state, recipe))
    tcp = state["tcp"]
    if not 0.10 <= tcp[0] <= 0.85:
        issues.append(f"TCP X {tcp[0]:.3f} m is outside the reachable drawer band")
    if angle_deg(tcp[3:], release_pose(recipe)[3:]) > 45.0:
        issues.append("TCP orientation is over 45 deg from the recorded release pose")
    if abs(tcp[1] - release_pose(recipe)[1]) > 0.25:
        issues.append("TCP Y is far from the drawer; jog the arm closer first")
    for name in APPROACH_CONFIRMATIONS:
        if confirmations.get(name) is not True:
            issues.append(f"operator confirmation missing: {name}")
    return issues


def grip_issues(before: dict[str, Any], after: dict[str, Any],
                recipe: dict[str, Any]) -> list[str]:
    """A verified re-grip needs the same width/effort signature as the pull."""
    issues = []
    target = release_pose(recipe)
    if math.dist(before["tcp"][:3], target[:3]) > 0.006:
        issues.append("TCP is not at the recorded release pose before gripping")
    if angle_deg(before["tcp"][3:], target[3:]) > 3.0:
        issues.append("TCP orientation is not at the recorded release pose")
    grip = after["right_gripper"]
    if not GRIPPED_RANGE_M[0] <= grip["width"] <= GRIPPED_RANGE_M[1]:
        issues.append(f"gripper closed to {grip['width']:.3f} m, expected a "
                      f"{GRIPPED_RANGE_M[0]:.3f}..{GRIPPED_RANGE_M[1]:.3f} m handle")
    if grip["effort"] < 4.0:
        issues.append(f"gripper effort {grip['effort']:.1f} N is too low: nothing between "
                      f"the fingers")
    if grip["is_moving"]:
        issues.append("gripper is still moving after the grip command")
    if math.dist(after["tcp"][:3], before["tcp"][:3]) > 0.005:
        issues.append("TCP moved while closing the gripper")
    return issues


def push_issues(before: dict[str, Any], after: dict[str, Any], segment: dict[str, Any],
                baseline_pose: list[float], *, grip_width_baseline_m: float,
                max_axis_drift_m: float = 0.006, path_tolerance_m: float = 0.014,
                max_orientation_error_deg: float = 4.0,
                min_progress_fraction: float = 0.75,
                max_grip_width_change_m: float = 0.004) -> list[str]:
    """Per-segment checks: did the arm advance, and is the handle still held.

    Two separate lateral checks, because the start gate already allows the TCP to
    sit up to 8 mm off the recorded segment start. Judging the result against the
    recorded path alone would then fail a run that behaved perfectly:

    * ``drift``  - where the arm ended versus where this very command sent it.
    * ``path``   - where it ended versus the recorded push path (start error plus
      drift, so the limit is the sum of both tolerances).

    `baseline_pose` is the TCP recorded at the verified re-grip: the handle must
    not be twisted away from the orientation it was grasped with.
    """
    issues = []
    goal = segment["to_pose"]
    actual_dx = after["tcp"][0] - before["tcp"][0]
    if actual_dx <= 0:
        issues.append(f"right arm did not advance (+{actual_dx * 1000:.1f} mm); the drawer "
                      f"did not move")
    elif actual_dx < segment["dx"] * min_progress_fraction:
        issues.append(f"right arm advanced {actual_dx * 1000:.1f} mm of "
                      f"{segment['dx'] * 1000:.1f} mm: blocked or slipping")
    for axis, name, requested in ((1, "Y", segment["dy"]), (2, "Z", segment["dz"])):
        drift = after["tcp"][axis] - (before["tcp"][axis] + requested)
        if abs(drift) > max_axis_drift_m:
            issues.append(f"TCP {name} drifted {drift * 1000:+.1f} mm from the commanded "
                          f"offset (limit {max_axis_drift_m * 1000:.0f} mm)")
        error = after["tcp"][axis] - goal[axis]
        if abs(error) > path_tolerance_m:
            issues.append(f"TCP {name} is {error * 1000:+.1f} mm off the recorded push path "
                          f"(limit {path_tolerance_m * 1000:.0f} mm)")
    if angle_deg(after["tcp"][3:], baseline_pose[3:]) > max_orientation_error_deg:
        issues.append("TCP orientation drifted from the orientation used to grasp the handle")
    grip = after["right_gripper"]
    if not GRIPPED_RANGE_M[0] <= grip["width"] <= GRIPPED_RANGE_M[1]:
        issues.append(f"gripper width {grip['width']:.3f} m left the gripped range: "
                      f"handle lost")
    if abs(grip["width"] - grip_width_baseline_m) > max_grip_width_change_m:
        issues.append("gripper width changed during the push: handle slipping")
    if grip["effort"] < 4.0:
        issues.append(f"gripper effort {grip['effort']:.1f} N is too low during the push")
    return issues


def closed_issues(state: dict[str, Any], recipe: dict[str, Any], *,
                  tolerance_m: float = 0.008) -> list[str]:
    issues = []
    error = state["tcp"][0] - closed_pose(recipe)[0]
    if abs(error) > tolerance_m:
        issues.append(f"TCP is {error * 1000:+.1f} mm from the closed reference "
                      f"(limit +/-{tolerance_m * 1000:.0f} mm)")
    grip = state["right_gripper"]
    if not GRIPPED_RANGE_M[0] <= grip["width"] <= GRIPPED_RANGE_M[1]:
        issues.append("gripper no longer holds a handle-sized object")
    return issues


# --------------------------------------------------------- camera comparison

def red_region(bgr, roi: tuple[float, float, float, float] | None = None) -> dict[str, Any]:
    """Largest red blob in a wrist image, in pixels only.

    The handle bar itself is horizontal; it only looks vertical because the wrist
    camera is mounted 90 degrees rotated (`handle_view` undoes that first).
    `drawer_perception.segment_handle` rejects this view either way - verified on
    the recorded 13:17 frame in both the raw and the rotated orientation, where it
    returns `handle_candidate_count: 0` because the fingers occlude the ring and
    the remaining blob is solid - so the re-grip check uses a plain red-region
    proxy instead of the ring detector.
    """
    import cv2
    import numpy as np

    if bgr is None or getattr(bgr, "ndim", 0) != 3:
        raise ValueError("Expected a BGR image")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = ((((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 160))
             & (hsv[:, :, 1] > 85) & (hsv[:, :, 2] > 35)).astype("uint8") * 255)
    height, width = mask.shape
    x0, y0, x1, y1 = (0, 0, width, height) if roi is None else (
        int(roi[0] * width), int(roi[1] * height), int(roi[2] * width), int(roi[3] * height))
    allowed = np.zeros_like(mask)
    allowed[max(0, y0):min(height, y1), max(0, x0):min(width, x1)] = 255
    mask &= allowed
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 200:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        candidates.append({"bbox": [x, y, w, h], "area": float(area),
                           "centroid": [x + w / 2, y + h / 2]})
    if not candidates:
        raise ValueError("no_red_region")
    return max(candidates, key=lambda item: item["area"])


def handle_view(reference_bgr, live_bgr, *, max_offset_px: float = 90.0,
                min_area_ratio: float = 0.45, max_area_ratio: float = 2.2,
                quarter_turns: int = 1,
                roi: tuple[float, float, float, float] = (0.10, 0.05, 0.90, 0.95)
                ) -> dict[str, Any]:
    """Compare the red handle in two wrist views taken from the same pose.

    Pixel space only: no extrinsics are assumed. A large centroid shift means
    the drawer is not where the recorded release pose expects it, which is the
    one thing base repeatability cannot guarantee.

    `quarter_turns` rotates both frames into the canonical view first. The wrist
    camera is mounted 90 degrees rotated, so the horizontal handle bar shows up
    vertical in the raw image; comparing in the canonical frame keeps the
    reported centroid offsets interpretable and keeps the aspect check meaningful.
    """
    import cv2
    import math as _math

    rotate = {1: cv2.ROTATE_90_CLOCKWISE, 2: cv2.ROTATE_180,
              3: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(quarter_turns % 4)
    result: dict[str, Any] = {"geometry_assumption": "same_pose_same_view",
                              "quarter_turns": quarter_turns % 4, "roi": roi}
    try:
        reference = red_region(reference_bgr if rotate is None else cv2.rotate(reference_bgr, rotate), roi)
        live = red_region(live_bgr if rotate is None else cv2.rotate(live_bgr, rotate), roi)
    except Exception as error:
        result["status"] = "handle_not_detected"
        result["detail"] = type(error).__name__ + ": " + str(error)
        return result
    offset = _math.dist(live["centroid"], reference["centroid"])
    ratio = live["area"] / reference["area"] if reference["area"] > 0 else float("inf")
    result.update({"reference_bbox": reference["bbox"], "live_bbox": live["bbox"],
                   "reference_centroid_px": [round(v, 1) for v in reference["centroid"]],
                   "live_centroid_px": [round(v, 1) for v in live["centroid"]],
                   "centroid_offset_canonical_px": [round(live["centroid"][0]
                                                          - reference["centroid"][0], 1),
                                                    round(live["centroid"][1]
                                                          - reference["centroid"][1], 1)],
                   "centroid_offset_px": round(offset, 1),
                   "canonical_aspect": round(live["bbox"][2] / max(live["bbox"][3], 1), 2),
                   "area_ratio": round(ratio, 3)})
    failures = []
    if offset > max_offset_px:
        failures.append(f"handle shifted {offset:.0f} px (limit {max_offset_px:.0f})")
    if not min_area_ratio <= ratio <= max_area_ratio:
        failures.append(f"visible red area ratio {ratio:.2f} outside "
                        f"{min_area_ratio}..{max_area_ratio}")
    if quarter_turns and result["canonical_aspect"] < 1.3:
        failures.append(f"red region aspect {result['canonical_aspect']:.2f} is not the "
                        f"horizontal handle bar")
    result["status"] = "ok" if not failures else "mismatch"
    if failures:
        result["failures"] = failures
    return result


def handle_gate(check: dict[str, Any], tcp: list[float], recipe: dict[str, Any], *,
                advisory: bool = False, engage_distance_m: float = 0.030,
                far_distance_m: float = 0.120) -> dict[str, Any]:
    """Decide whether a wrist view is comparable to the reference at this pose.

    `handle_view` is a same-pose, same-view comparison. After the medicine run
    the arm is stowed somewhere unrelated, so the view differs for reasons that
    say nothing about the drawer. Comparing anyway would block `prepare` every
    single time - the check was written as a gate but would have acted as a wall.
    Trust it only inside the engage band, and call it advisory in between.
    """
    distance = math.dist(tcp[:3], standoff_pose(recipe)[:3])
    status = str(check.get("status"))
    verdict: dict[str, Any] = {"status": status, "distance_to_standoff_m": round(distance, 4)}
    if distance > far_distance_m:
        verdict.update(gate="skipped", reason="arm_far_from_release_pose")
    elif status in ("no_reference_image", "cv_unavailable", "not_attempted"):
        verdict.update(gate="skipped", reason="no_comparable_frame")
    elif distance > engage_distance_m:
        verdict.update(gate="advisory", reason="still_far_from_the_handle")
    elif status == "mismatch" and not advisory:
        verdict.update(gate="blocking", reason="handle_not_where_the_record_expects")
        verdict["failures"] = check.get("failures", [])
    else:
        verdict["gate"] = "ok" if status == "ok" else "advisory"
    return verdict


# ---------------------------------------------------------------------- plan
def plan(recipe: dict[str, Any] | None = None) -> dict[str, Any]:
    recipe = load_recipe() if recipe is None else recipe
    segments = push_segments(recipe)
    return {
        "dry_run": True,
        "motion_commands_sent": [],
        "flow": ["prepare (read-only)", "approach xN", "grip", "push xN",
                 "release", "retract"],
        "release_tcp_base_link": release_pose(recipe),
        "closed_tcp_base_link": closed_pose(recipe),
        "travel_to_close_m": round(travel_to_closed(recipe), 4),
        "recorded_stroke_mm": recorded_travel_mm(recipe),
        "stroke_must_be_confirmed_by_tape_measure": True,
        "approach_standoff_m": recipe["approach_standoff_m"],
        "manual_alignment_required_before_prepare": True,
        "target_alignment_tcp_base_link": [round(v, 4) for v in standoff_pose(recipe)[:3]],
        "manual_alignment_tolerance_m": 0.040,
        "push_segments_mm": [round(segment["dx"] * 1000, 1) for segment in segments],
        "push_segment_dy_dz_mm": [[round(segment["dy"] * 1000, 1),
                                   round(segment["dz"] * 1000, 1)] for segment in segments],
        "grip_command": recipe["grip_command"],
        "requires_active_controller": "right_arm_pvt_ctrl",
        "base_pose_handling": {
            "exact_match_within": {"shift_m": 0.030, "yaw_deg": 2.0},
            "re_localised_recipe_within": {"shift_m": REBASE_SHIFT_LIMIT_M,
                                           "yaw_deg": REBASE_YAW_LIMIT_DEG},
            "re_localised_reach_x_m": list(REBASE_REACH_X_RANGE_M),
            "re_localised_lateral_limit_m": REBASE_LATERAL_LIMIT_M,
            "requires_drawer_unmoved_confirmation": True,
            "requires_stationary_base": "drift <= 5 mm / 0.3 deg between two reads",
            "assumption": "drawer unchanged in map; localisation trustworthy "
                          "(base_link poses are carried exactly into the current base frame)",
        },
        "requires_operator_handle_check_before_grip": True,
        "one_command_per_invocation": True,
        "no_automatic_retry": True,
        "verified_on_robot": recipe.get("verified_on_robot", False),
        "limitations": recipe.get("limitations", []),
    }
