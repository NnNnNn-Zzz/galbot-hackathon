"""Supervised drawer-pull replay with immutable command attempts and full states.

Default CLI invocation only prints the plan. Live phases require --execute, a
new session, current console access, an active right-arm PVT controller, and a
fresh continuous recorder. The base is never commanded. This script does not
switch controllers or move the arm into its initial alignment pose.

The old demonstration supplies relative X checkpoints and a start-pose guard,
not a continuous path. Recorded Y/Z drift is not used as a motion target.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
from typing import Any
from urllib import request

from medicine_frontend.dataset import Dataset, DEFAULT_ROOT
from medicine_frontend.tools.capture_final_drawer import CONSOLE, sdk_snapshot
from medicine_frontend.tools.record_drawer_stream import login


WORKSPACE = Path(__file__).resolve().parents[2]
RECIPE_PATH = (WORKSPACE / "galbot-hackathon/scenes/scene_medicine_check/data/"
               "recorded_drawer_pull_20261107.json")
TARGETS_M = (0.02, 0.07, 0.12, 0.17, 0.22, 0.25, 0.275, 0.30)
SESSION_PATTERN = re.compile(r"drawer_replay_[0-9]{8}_[0-9]{6}(?:_[A-Za-z0-9_-]+)?\Z")
RIGHT_ARM_JOINTS = tuple(f"right_arm_joint{i}" for i in range(1, 8))
REQUIRED_CONFIRMATIONS = ("alignment", "support", "travel", "clear_path",
                          "exclusive_control", "emergency_stop_ready")


def _finite(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _angle_deg(left: list[float], right: list[float]) -> float:
    dot = abs(sum(float(a) * float(b) for a, b in zip(left, right)))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))


def _accepted(response: Any) -> bool:
    return (isinstance(response, dict) and response.get("accepted") is True
            and response.get("dry_run") is False
            and str(response.get("sdk_result", "")).endswith("SUCCESS"))


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_new(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    with path.open("xb") as output:
        output.write(raw)
        output.flush()
        os.fsync(output.fileno())


def _update(path: Path, value: dict[str, Any]) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    temporary = path.with_name(path.name + f".{time.time_ns()}.tmp")
    _write_new(temporary, value)
    temporary.replace(path)


def _load_recipe(path: Path = RECIPE_PATH) -> dict[str, Any]:
    recipe = _read_json(path)
    if recipe.get("schema") != "drawer_replay_recipe/1" or tuple(
            recipe.get("pull_targets_cumulative_x_m", ())) != TARGETS_M:
        raise ValueError("Recorded drawer pull recipe changed")
    source = WORKSPACE / "medicine_frontend/data/medicine_dataset" / recipe[
        "source_demonstration"]
    if (not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()
            != recipe["source_demonstration_sha256"]):
        raise ValueError("Source demonstration is missing or changed")
    return recipe


def _extract(snapshot: dict[str, Any], *, fresh: bool = True) -> dict[str, Any]:
    if snapshot.get("schema") != "sdk_console_snapshot/1" or snapshot.get("errors"):
        raise ValueError("SDK state snapshot is incomplete")
    finished = snapshot.get("finished_ns")
    if (not isinstance(finished, int) or isinstance(finished, bool)
            or (fresh and not 0 <= time.time_ns() - finished <= 5_000_000_000)):
        raise ValueError("SDK snapshot is stale")
    reads = snapshot["reads"]
    nav = reads["navigation"]["data"]
    robot = reads["robot"]["data"]
    end_effector = reads["right_end_effector"]["data"]
    if (nav.get("connected") is not True or nav.get("localized") is not True
            or nav["pose"].get("frame_id") != "map" or robot.get("dry_run") is not False
            or end_effector.get("available") is not True
            or end_effector.get("dry_run") is not False
            or end_effector.get("frame_id") != "base_link"):
        raise ValueError("Localization, robot or right TCP is unavailable")
    tcp = end_effector["pose"]
    base = nav["pose"]
    if (len(tcp) != 7 or not all(_finite(v) for v in tcp)
            or not 0.95 <= math.sqrt(sum(v * v for v in tcp[3:])) <= 1.05
            or not all(_finite(base.get(key)) for key in ("x", "y", "qz", "qw"))):
        raise ValueError("Invalid TCP or map pose")
    joints = {entry["name"]: entry["position"] for entry in robot["joint_state"]
              if isinstance(entry, dict) and "name" in entry and "position" in entry}
    arm = {name: joints[name] for name in RIGHT_ARM_JOINTS}
    chassis = {name: value for name, value in joints.items() if name.startswith("chassis_joint")}
    if (not chassis or not all(_finite(v) for v in arm.values())
            or not all(_finite(v) for v in chassis.values())):
        raise ValueError("Right-arm or chassis joint feedback missing")
    sensors = [item["event"] for item in snapshot["events"]
               if item.get("event", {}).get("type") == "robot_sensors"]
    if not sensors:
        raise ValueError("Right gripper feedback missing")
    sensor = sensors[-1]
    grip = sensor["data"]["grippers"]["right"]
    # `finished_ns` is stamped before the last sensor event lands, so the event
    # is commonly ~0.1-0.3 s NEWER than the snapshot. Only a symmetric window
    # works here; a one-sided `0 <= finished - ts` rejects healthy samples.
    if (not all(_finite(grip.get(key)) for key in ("width", "effort"))
            or not isinstance(grip.get("is_moving"), bool)
            or (fresh and abs(finished - sensor["timestamp_ns"]) > 3_000_000_000)):
        raise ValueError("Right gripper feedback is invalid or stale")
    return {"tcp": list(tcp), "base_map": dict(base), "right_arm_joints_rad": arm,
            "chassis_joints_rad": chassis, "right_gripper": dict(grip),
            "sample_finished_ns": finished}


def _base_shift(state: dict[str, Any], baseline: dict[str, Any]) -> float:
    return math.hypot(state["base_map"]["x"] - baseline["base_map"]["x"],
                      state["base_map"]["y"] - baseline["base_map"]["y"])


def _chassis_shift(state: dict[str, Any], baseline: dict[str, Any]) -> float:
    current = state["chassis_joints_rad"]
    original = baseline["chassis_joints_rad"]
    if set(current) != set(original):
        return math.inf
    return max(abs(current[name] - value) for name, value in original.items())


def _grip_ok(state: dict[str, Any]) -> bool:
    grip = state["right_gripper"]
    return (0.020 <= grip["width"] <= 0.030 and grip["effort"] >= 4.0
            and not grip["is_moving"])


def _controller_active(response: dict[str, Any]) -> bool:
    return (response.get("group") == "right_arm"
            and str(response.get("active", "")).lower() == "right_arm_pvt_ctrl"
            and response.get("dry_run") is False)


def _stream_active(folder: Path) -> bool:
    for name in ("events.ndjson", "tcp.ndjson"):
        path = folder / "continuous" / name
        if (not path.is_file() or path.stat().st_size == 0
                or not 0 <= time.time() - path.stat().st_mtime <= 4.0):
            return False
    return True


def _observe(io: Any, db: Dataset, folder: Path, name: str) -> dict[str, Any]:
    value = io.snapshot()
    path = folder / f"{name}.json"
    _write_new(path, value)
    db.ingest_state(path, folder.name)
    return _extract(value)


def _stop(io: Any, journal: dict[str, Any]) -> bool:
    try:
        result = io.stop_arm()
        journal["stop_result"] = result
        return _accepted(result)
    except Exception as error:
        journal["stop_error"] = type(error).__name__ + ": " + str(error)
        return False


def _assert_session(session: str) -> None:
    if not SESSION_PATTERN.fullmatch(session):
        raise ValueError("Use a new drawer_replay_YYYYMMDD_HHMMSS session name")


class ConsoleIO:
    """Existing SDK console only; credentials stay in process memory."""

    def __init__(self, code: str):
        self.code = code
        self.cookie = login(code)

    def _call(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        req = request.Request(CONSOLE + path, data=body,
                              headers={"Content-Type": "application/json",
                                       "Cookie": self.cookie})
        with request.urlopen(req, timeout=10) as response:
            return json.load(response)

    def snapshot(self) -> dict[str, Any]:
        return sdk_snapshot(self.code)

    def controller(self) -> dict[str, Any]:
        return self._call("/api/commands/robot/controller_snapshot", {"group": "right_arm"})

    def grip(self, command: dict[str, Any]) -> dict[str, Any]:
        return self._call("/api/commands/robot/set_gripper_command", command)

    def offset(self, command: dict[str, Any]) -> dict[str, Any]:
        return self._call("/api/commands/motion/move_end_effector_offset", command)

    def tcp(self) -> dict[str, Any]:
        return self._call("/api/commands/motion/end_effector_pose", {"arm": "right_arm"})

    def stop_arm(self) -> dict[str, Any]:
        return self._call("/api/commands/motion/stop_controller", {"group": "right_arm"})


def _real_force_guard(folder: Path, io: ConsoleIO, step: int):
    from medicine_frontend.tools.close_drawer_second_probe import ForceGuard
    guard = ForceGuard(folder, io.cookie)
    guard.path = folder / f"step_{step:02d}_wrist_force.ndjson"
    return guard


def prepare(session: str, io: Any, *, confirmations: dict[str, bool],
            dataset_root: Path = DEFAULT_ROOT, recipe_path: Path = RECIPE_PATH) -> dict[str, Any]:
    _assert_session(session)
    recipe = _load_recipe(recipe_path)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        if folder.exists() and any(
                child.name not in ("continuous", "camera_stream") or not child.is_dir()
                for child in folder.iterdir()):
            raise FileExistsError("Replay attempt already exists; preserve the earlier attempt")
        folder.mkdir(parents=True, exist_ok=True)
        controller = io.controller()
        state = _observe(io, db, folder, "prepare_state")
        tcp_reference = recipe["pre_grasp_tcp_base_link"]
        base_reference = {"base_map": recipe["alignment_base_map"]}
        issues = []
        if not _controller_active(controller):
            issues.append("right_arm_pvt_ctrl is not active")
        if math.dist(state["tcp"][:3], tcp_reference[:3]) > 0.015:
            issues.append("TCP is more than 15 mm from recorded pre-grasp")
        if _angle_deg(state["tcp"][3:], tcp_reference[3:]) > 3.0:
            issues.append("TCP orientation differs by more than 3 degrees")
        if _base_shift(state, base_reference) > 0.04:
            issues.append("base map position differs by more than 40 mm")
        if any(abs(state["right_arm_joints_rad"][name] - value) > 0.12
               for name, value in recipe["alignment_right_arm_joints_rad"].items()):
            issues.append("right-arm joints differ from recorded alignment")
        if not 0.040 <= state["right_gripper"]["width"] <= 0.060:
            issues.append("right gripper is not in the recorded open range")
        for name in REQUIRED_CONFIRMATIONS:
            if confirmations.get(name) is not True:
                issues.append(f"operator confirmation missing: {name}")
        result = {"schema": "drawer_replay_phase/1", "recorded_ns": time.time_ns(),
                  "session": session, "phase": "prepare", "status": "ready" if not issues else "blocked",
                  "recipe": str(recipe_path), "confirmations": confirmations,
                  "controller": controller, "state_file": db.relative(folder / "prepare_state.json"),
                  "state": state, "issues": issues, "motion_commands_sent": []}
        path = folder / "prepare.json"
        _write_new(path, result)
        db.ingest_teaching_event(path, session)
        return result


def grip(session: str, io: Any, *, dataset_root: Path = DEFAULT_ROOT,
         recipe_path: Path = RECIPE_PATH, require_stream: bool = True) -> dict[str, Any]:
    _assert_session(session)
    recipe = _load_recipe(recipe_path)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        prepared = _read_json(folder / "prepare.json")
        journal_path = folder / "grip.json"
        if journal_path.exists():
            raise FileExistsError("Gripper attempt already recorded; refusing a duplicate command")
        if prepared["status"] != "ready" or time.time_ns() - prepared["recorded_ns"] > 600_000_000_000:
            raise ValueError("Fresh, ready alignment preparation is required")
        if require_stream and not _stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        controller = io.controller()
        before = _observe(io, db, folder, "grip_before")
        baseline = prepared["state"]
        if (not _controller_active(controller)
                or math.dist(before["tcp"][:3], baseline["tcp"][:3]) > 0.005
                or _angle_deg(before["tcp"][3:], baseline["tcp"][3:]) > 2.0
                or _base_shift(before, baseline) > 0.02
                or _chassis_shift(before, baseline) > 0.005
                or not 0.040 <= before["right_gripper"]["width"] <= 0.060):
            raise ValueError("Controller, arm, chassis, TCP or open gripper changed; no command sent")
        command = dict(recipe["grip_command"])
        journal = {"schema": "drawer_replay_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "grip", "status": "attempt_started",
                   "command": command, "submission": "started",
                   "before_state_file": db.relative(folder / "grip_before.json"),
                   "before": before, "before_controller": controller,
                   "base_commands_sent": [], "arm_commands_sent": []}
        _write_new(journal_path, journal)
        try:
            response = io.grip(command)
            journal["command_result"] = response
            journal["submission"] = "responded"
            _update(journal_path, journal)
            if not _accepted(response):
                raise RuntimeError("Gripper command was not accepted by SDK")
            after = _observe(io, db, folder, "grip_after")
            journal["after_state_file"] = db.relative(folder / "grip_after.json")
            journal["after"] = after
            journal["after_controller"] = io.controller()
            if (not _controller_active(journal["after_controller"]) or not _grip_ok(after)
                    or math.dist(after["tcp"][:3], before["tcp"][:3]) > 0.005
                    or _angle_deg(after["tcp"][3:], before["tcp"][3:]) > 2.0
                    or _base_shift(after, baseline) > 0.03
                    or _chassis_shift(after, baseline) > 0.005):
                raise RuntimeError("Grasp, TCP, base or right-arm controller did not verify")
            journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
            _stop(io, journal)
        finally:
            journal["finished_ns"] = time.time_ns()
            _update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def _pull_baseline(folder: Path, step: int) -> tuple[dict[str, Any], dict[str, Any]]:
    gripped = _read_json(folder / "grip.json")
    if gripped["status"] != "completed":
        raise ValueError("Verified right-gripper closure is required")
    start = gripped["after"]
    if step <= 6:
        stage = start
    else:
        rebased = _read_json(folder / "rebase_25cm.json")
        if rebased["status"] != "ready":
            raise ValueError("25 cm extension was not confirmed")
        stage = rebased["state"]
    return start, stage


def rebase_25cm(session: str, io: Any, *, operator_extension_confirmed: bool,
                dataset_root: Path = DEFAULT_ROOT, require_stream: bool = True) -> dict[str, Any]:
    _assert_session(session)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        path = folder / "rebase_25cm.json"
        if path.exists():
            raise FileExistsError("25 cm rebase already recorded")
        if operator_extension_confirmed is not True:
            raise ValueError("On-site confirmation for the extra 5 cm is required")
        if require_stream and not _stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        prior = _read_json(folder / "step_06.json")
        if prior["status"] not in ("paused_at_25cm", "paused_axis_drift"):
            raise ValueError("Sixth step did not reach a safe 25 cm pause")
        start, _ = _pull_baseline(folder, 6)
        controller = io.controller()
        state = _observe(io, db, folder, "rebase_25cm_state")
        if (not _controller_active(controller) or not _grip_ok(state)
                or abs(state["tcp"][0] - (start["tcp"][0] - 0.25)) > 0.008
                or _base_shift(state, start) > 0.03
                or _chassis_shift(state, start) > 0.005
                or abs(state["tcp"][1] - start["tcp"][1]) > 0.012
                or abs(state["tcp"][2] - start["tcp"][2]) > 0.012
                or _angle_deg(state["tcp"][3:], start["tcp"][3:]) > 4.0):
            raise ValueError("25 cm pose, grasp or controller outside rebase guard")
        result = {"schema": "drawer_replay_phase/1", "recorded_ns": time.time_ns(),
                  "session": session, "phase": "rebase_25cm", "status": "ready",
                  "operator_extension_confirmed": True,
                  "state_file": db.relative(folder / "rebase_25cm_state.json"),
                  "state": state, "controller": controller,
                  "rebase_only_y_z_and_orientation": True,
                  "base_commands_sent": [], "arm_commands_sent": []}
        _write_new(path, result)
        db.ingest_teaching_event(path, session)
        return result


def _pose_from_reply(reply: dict[str, Any]) -> list[float]:
    pose = reply.get("pose")
    if (reply.get("available") is not True or reply.get("dry_run") is not False
            or reply.get("frame_id") != "base_link" or not isinstance(pose, list)
            or len(pose) != 7 or not all(_finite(v) for v in pose)):
        raise ValueError("Fresh right TCP poll unavailable")
    return pose


def step(session: str, index: int, io: Any, *, dataset_root: Path = DEFAULT_ROOT,
         require_stream: bool = True, guard_factory=None, poll_timeout_s: float = 12.0,
         poll_interval_s: float = 0.25) -> dict[str, Any]:
    _assert_session(session)
    if index not in range(1, 9):
        raise ValueError("Step must be 1 through 8")
    guard_factory = guard_factory or _real_force_guard
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / f"step_{index:02d}.json"
        if journal_path.exists():
            raise FileExistsError("This step has an attempt record; refusing duplicate motion")
        if require_stream and not _stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        start, stage = _pull_baseline(folder, index)
        if index > 1:
            previous = _read_json(folder / f"step_{index - 1:02d}.json")
            allowed = ("paused_at_25cm", "paused_axis_drift") if index == 7 else ("completed",)
            if previous["status"] not in allowed:
                raise ValueError("Previous step was not completed or safely paused")
        target_x = start["tcp"][0] - TARGETS_M[index - 1]
        previous_x = start["tcp"][0] - (TARGETS_M[index - 2] if index > 1 else 0.0)
        controller = io.controller()
        before = _observe(io, db, folder, f"step_{index:02d}_before")
        if (not _controller_active(controller) or not _grip_ok(before)
                or abs(before["tcp"][0] - previous_x) > 0.008
                or abs(before["tcp"][1] - stage["tcp"][1]) > 0.005
                or abs(before["tcp"][2] - stage["tcp"][2]) > 0.005
                or _angle_deg(before["tcp"][3:], stage["tcp"][3:]) > 2.0
                or _base_shift(before, start) > 0.03
                or _chassis_shift(before, start) > 0.005):
            raise ValueError("Controller, TCP, base or grasp precondition failed; no command sent")
        command = {"arm": "right_arm", "dx": target_x - before["tcp"][0],
                   "dy": stage["tcp"][1] - before["tcp"][1],
                   "dz": stage["tcp"][2] - before["tcp"][2],
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "confirmed": True}
        if (not -0.1 <= command["dx"] < 0 or abs(command["dy"]) > 0.02
                or abs(command["dz"]) > 0.02):
            raise ValueError("Calculated offset outside guarded range")
        journal = {"schema": "drawer_replay_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "pull_step", "step": index,
                   "status": "attempt_started", "submission": "not_started",
                   "target_cumulative_x_m": TARGETS_M[index - 1],
                   "target_tcp_x_base_link_m": target_x,
                   "stage_reference_tcp": stage["tcp"],
                   "before_state_file": db.relative(folder / f"step_{index:02d}_before.json"),
                   "before": before, "before_controller": controller,
                   "command": command, "base_commands_sent": [],
                   "wrist_force_policy": {"sensor_frame": True, "baseline_samples": 20,
                                          "sustained_delta_N": 5.0,
                                          "immediate_delta_N": 8.0,
                                          "sample_interval_s": 0.05}}
        _write_new(journal_path, journal)
        guard = None
        poll_path = folder / f"step_{index:02d}_tcp_polls.ndjson"
        try:
            guard = guard_factory(folder, io, index)
            guard.start()  # no arm command if wrist-force monitoring is unavailable
            journal["force_baseline_sensor_frame_N"] = guard.baseline
            journal["submission"] = "started"
            journal["command_started_ns"] = time.time_ns()
            _update(journal_path, journal)
            guard.motion_started.set()
            response = io.offset(command)
            journal["command_result"] = response
            journal["submission"] = "responded"
            _update(journal_path, journal)
            if not _accepted(response):
                raise RuntimeError("SDK did not accept right-arm offset")
            deadline = time.monotonic() + poll_timeout_s
            with poll_path.open("x", encoding="utf-8") as output:
                while True:
                    guard.check()
                    if require_stream and not _stream_active(folder):
                        raise RuntimeError("Continuous recorder stopped during the arm step")
                    requested_ns = time.time_ns()
                    reply = io.tcp()
                    actual = _pose_from_reply(reply)
                    active = io.controller()
                    sample = {"requested_ns": requested_ns, "received_ns": time.time_ns(),
                              "pose_base_link": actual, "controller": active.get("active")}
                    output.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
                    output.flush()
                    if not _controller_active(active):
                        raise RuntimeError("Right-arm PVT controller lost authority")
                    if abs(actual[0] - target_x) <= 0.005:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Right TCP did not reach X checkpoint")
                    time.sleep(poll_interval_s)
            guard.check()
            after = _observe(io, db, folder, f"step_{index:02d}_after")
            journal["after_state_file"] = db.relative(folder / f"step_{index:02d}_after.json")
            journal["after"] = after
            journal["after_controller"] = io.controller()
            if (not _controller_active(journal["after_controller"]) or not _grip_ok(after)
                    or abs(after["tcp"][0] - target_x) > 0.008
                    or _base_shift(after, start) > 0.03
                    or _chassis_shift(after, start) > 0.005):
                raise RuntimeError("After-step X, grasp, base or controller guard failed")
            yz_error = [after["tcp"][axis] - stage["tcp"][axis] for axis in (1, 2)]
            angle_error = _angle_deg(after["tcp"][3:], stage["tcp"][3:])
            journal["observed_y_z_error_m"] = yz_error
            journal["observed_orientation_error_deg"] = angle_error
            axes_ok = max(abs(v) for v in yz_error) <= 0.005 and angle_error <= 2.0
            if index == 6:
                journal["status"] = "paused_at_25cm" if axes_ok else "paused_axis_drift"
                if not _stop(io, journal):
                    raise RuntimeError("Right-arm stop at 25 cm was not confirmed")
                guard.motion_started.clear()
            elif not axes_ok:
                journal["status"] = "failed_axes"
                raise RuntimeError("Y/Z or orientation exceeded endpoint tolerance")
            elif index == 8:
                journal["status"] = "completed"
                if not _stop(io, journal):
                    raise RuntimeError("Final right-arm stop was not confirmed")
                guard.motion_started.clear()
            else:
                journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
            if journal["submission"] != "not_started" and "stop_result" not in journal:
                _stop(io, journal)
        finally:
            if guard is not None:
                guard.motion_started.clear()
                try:
                    guard.close()
                except Exception as error:
                    journal["force_close_error"] = type(error).__name__ + ": " + str(error)
                    journal["status"] = "failed"
                journal["force_samples"] = guard.samples
                journal["max_raw_force_delta_N"] = guard.max_delta_n
                journal["force_trip_reason"] = guard.reason
                if guard.path.is_file():
                    journal["force_file"] = db.relative(guard.path)
                    journal["force_sha256"] = hashlib.sha256(guard.path.read_bytes()).hexdigest()
                if guard.trip.is_set() and journal["status"] in ("completed", "paused_at_25cm", "paused_axis_drift"):
                    journal["status"] = "failed"
                    journal["error"] = guard.reason or "Wrist-force guard tripped"
                if guard.stop_result is not None:
                    journal.setdefault("stop_result", guard.stop_result)
                if guard.stop_error:
                    journal["stop_error"] = guard.stop_error
            if "stop_result" in journal and journal["submission"] != "not_started":
                try:
                    post_stop = _observe(io, db, folder, f"step_{index:02d}_post_stop")
                    journal["post_stop_state_file"] = db.relative(
                        folder / f"step_{index:02d}_post_stop.json")
                    journal["post_stop"] = post_stop
                    if (journal["status"] in ("completed", "paused_at_25cm",
                                              "paused_axis_drift")
                            and (abs(post_stop["tcp"][0] - target_x) > 0.008
                                 or not _grip_ok(post_stop)
                                 or _base_shift(post_stop, start) > 0.03
                                 or _chassis_shift(post_stop, start) > 0.005)):
                        journal["status"] = "failed"
                        journal["error"] = "Post-stop TCP, grip or base guard failed"
                except Exception as error:
                    journal["post_stop_error"] = type(error).__name__ + ": " + str(error)
                    journal["status"] = "failed"
            if poll_path.is_file():
                journal["tcp_poll_file"] = db.relative(poll_path)
                journal["tcp_poll_sha256"] = hashlib.sha256(poll_path.read_bytes()).hexdigest()
            journal["finished_ns"] = time.time_ns()
            _update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def plan() -> dict[str, Any]:
    recipe = _load_recipe()
    return {"dry_run": True, "motion_commands_sent": [],
            "phases": ["prepare", "grip", *[f"step {i}" for i in range(1, 7)],
                       "rebase_25cm", "step 7", "step 8"],
            "targets_cumulative_x_m": list(TARGETS_M),
            "grip_command": recipe["grip_command"],
            "requires_active_right_arm_controller": "right_arm_pvt_ctrl",
            "requires_continuous_recording": True,
            "requires_on_site_alignment_support_travel_exclusive_control": True}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("plan", "prepare", "grip", "step", "rebase"),
                        nargs="?", default="plan")
    parser.add_argument("--session")
    parser.add_argument("--step", type=int)
    parser.add_argument("--execute", action="store_true", help="Enable one guarded live phase")
    for name in REQUIRED_CONFIRMATIONS:
        parser.add_argument("--" + name.replace("_", "-") + "-confirmed", action="store_true")
    parser.add_argument("--extension-confirmed", action="store_true")
    args = parser.parse_args(argv)
    if args.phase == "plan" or not args.execute:
        print(json.dumps(plan(), ensure_ascii=False, indent=2))
        return 0
    if not args.session:
        parser.error("--session is required with --execute")
    code = os.environ.get("GALBOT_CONSOLE_CODE")
    if not code:
        parser.error("Current GALBOT_CONSOLE_CODE must be set in the process environment")
    io = ConsoleIO(code)
    if args.phase == "prepare":
        confirmations = {name: getattr(args, name + "_confirmed")
                         for name in REQUIRED_CONFIRMATIONS}
        result = prepare(args.session, io, confirmations=confirmations)
    elif args.phase == "grip":
        result = grip(args.session, io)
    elif args.phase == "rebase":
        result = rebase_25cm(args.session, io,
                             operator_extension_confirmed=args.extension_confirmed)
    else:
        if args.step is None:
            parser.error("--step 1..8 is required for the step phase")
        result = step(args.session, args.step, io)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") in ("ready", "completed", "paused_at_25cm",
                                        "paused_axis_drift") else 1


if __name__ == "__main__":
    raise SystemExit(main())
