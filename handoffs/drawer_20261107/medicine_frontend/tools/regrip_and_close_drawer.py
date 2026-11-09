"""Re-grip the open drawer handle after the medicine throw, then push it closed.

Flow implemented here (one command per invocation, no automatic retry):

    prepare  read-only gate: control authority, open gripper, base back home
    approach move the right TCP to the recorded release pose, in bounded steps
    grip     close the gripper on the handle and verify width/effort
    push     advance along the recorded pull path, reversed, one segment at a time
    release  open the gripper only after the drawer is closed
    retract  back the arm 50 mm away from the closed drawer
    report   aggregate the session journal

Every phase writes its journal *before* submitting a command, refuses to reuse
a session file, and stops the arm on any guard failure. The base is never
commanded: this tool only verifies that somebody else drove it back.

Run without --execute to print the plan; nothing is sent in that mode.
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

from medicine_frontend import drawer_regrip_close as rclose
from medicine_frontend.dataset import Dataset, DEFAULT_ROOT
from medicine_frontend.tools import replay_drawer_pull as replay


SESSION_PATTERN = re.compile(r"drawer_regrip_[0-9]{8}_[0-9]{6}(?:_[A-Za-z0-9_-]+)?\Z")
PREPARE_VALIDITY_S = 600
FORCE_SUSTAINED_N = 5.0
FORCE_IMMEDIATE_N = 8.0
ENGAGE_SUSTAINED_N = 3.0
ENGAGE_IMMEDIATE_N = 6.0
PUSH_SUSTAINED_N = 6.0
PUSH_IMMEDIATE_N = 10.0
MAX_STEP_DY_DZ_M = 0.012


def _session_ok(session: str) -> None:
    if not SESSION_PATTERN.fullmatch(session):
        raise ValueError("Use a new drawer_regrip_YYYYMMDD_HHMMSS session")


def _guard(folder: Path, io: Any, name: str, sustained_n: float, immediate_n: float):
    from medicine_frontend.tools.close_drawer_second_probe import ForceGuard
    return ForceGuard(folder, io.cookie, path=folder / f"{name}_wrist_force.ndjson",
                      sustained_n=sustained_n, immediate_n=immediate_n)


def _real_guard_factory(sustained_n: float, immediate_n: float):
    def factory(folder: Path, io: Any, name: str):
        return _guard(folder, io, name, sustained_n, immediate_n)
    return factory


def _prepare_path(folder: Path) -> Path:
    return folder / "prepare.json"


def _fresh_prepare(folder: Path) -> dict[str, Any]:
    prepared = replay._read_json(_prepare_path(folder))
    if prepared["status"] != "ready":
        raise ValueError("Preparation is not ready; fix the reported issues first")
    if time.time_ns() - prepared["recorded_ns"] > PREPARE_VALIDITY_S * 1_000_000_000:
        raise ValueError("Preparation is older than ten minutes; re-run prepare")
    return prepared


def _prior_state(folder: Path, kind: str, index: int, prepared: dict[str, Any]) -> dict[str, Any]:
    """State that the current command must start from, or refuse."""
    if index == 1:
        return prepared["state"]
    previous = replay._read_json(folder / f"{kind}_{index - 1:02d}.json")
    if previous["status"] not in ("completed", "closed_pending_confirmation"):
        raise ValueError(f"Previous {kind} step did not complete")
    return previous["after"]


# ------------------------------------------------------------------- camera

def _camera(code: str | None):
    if not code:
        return None
    try:
        from medicine_frontend.tools.collect import Cam
        return Cam(code, None)
    except Exception:
        return None


def _capture(cam, folder: Path, name: str, sources=("wrist_right", "head_right")) -> dict[str, Any]:
    """Save one RGB frame per source as evidence; never blocks a phase by itself."""
    result: dict[str, Any] = {"schema": "drawer_regrip_frames/1", "frames": [], "errors": []}
    if cam is None:
        result["errors"].append({"frame": "all", "error": "camera_unavailable"})
        return result
    for source in sources:
        try:
            status, headers, body = cam.get(f"/frame/{source}")
            if status != 200:
                # The bridge answers 503 with a JSON body while a camera warms up;
                # saving that as .jpg would poison the evidence folder.
                result["errors"].append({"frame": source, "error": f"http_{status}",
                                         "detail": body[:160].decode("utf-8", "replace")})
                continue
            path = folder / f"{name}_{source}.jpg"
            path.write_bytes(body)
            result["frames"].append({
                "source": source, "path": path.name, "bytes": len(body),
                "sha256": hashlib.sha256(body).hexdigest(),
                "captured_ns": headers.get("X-Frame-Captured-Ns"),
                "age_ms": None if not headers.get("X-Frame-Captured-Ns") else
                round((time.time_ns() - int(headers["X-Frame-Captured-Ns"])) / 1e6, 1)})
        except Exception as error:
            result["errors"].append({"frame": source, "error": type(error).__name__ + ": "
                                     + str(error)})
    return result


def _handle_check(recipe: dict[str, Any], live_path: Path) -> dict[str, Any]:
    reference_name = recipe.get("reference_wrist_image")
    if not reference_name:
        return {"status": "no_reference_image"}
    reference_path = DEFAULT_ROOT / reference_name
    if not reference_path.is_file():
        return {"status": "no_reference_image"}
    try:
        import cv2
        reference = cv2.imread(str(reference_path))
        live = cv2.imread(str(live_path))
    except Exception as error:
        return {"status": "cv_unavailable", "detail": str(error)}
    return rclose.handle_view(
        reference, live,
        quarter_turns=int(recipe.get("wrist_image_rotation_quarter_turns", 0)))


# ------------------------------------------------------------------ phases

def _active_recipe(folder: Path, fallback: dict[str, Any]) -> dict[str, Any]:
    """prepare may have written a recipe re-expressed in the current base frame."""
    path = folder / "rebased_recipe.json"
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return fallback


def prepare(session: str, io: Any, *, recipe: dict[str, Any], confirmations: dict[str, bool],
            dataset_root: Path = DEFAULT_ROOT, cam=None,
            measured_travel_mm: float | None = None,
            base_settle_s: float = 1.0) -> dict[str, Any]:
    _session_ok(session)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        if folder.exists() and any(
                item.name not in ("continuous", "camera_stream") or not item.is_dir()
                for item in folder.iterdir()):
            raise FileExistsError("Re-grip attempt already exists; use a new session")
        folder.mkdir(parents=True, exist_ok=True)
        controller = io.controller()
        first = replay._observe(io, db, folder, "prepare_state")
        settle: dict[str, Any] = {"shift_m": 0.0, "dyaw_deg": 0.0}
        state = first
        if base_settle_s > 0:
            time.sleep(base_settle_s)
            state = replay._observe(io, db, folder, "prepare_state_settled")
            settle = rclose.base_delta(first["base_map"], state["base_map"])
        active, rebase_info, issues = rclose.resolve_recipe_for_base(recipe, state["base_map"])
        issues = list(issues)
        rebase_info["settle_drift"] = settle
        if settle["shift_m"] > 0.005 or abs(settle["dyaw_deg"]) > 0.3:
            issues.append(f"base moved {settle['shift_m'] * 1000:.1f} mm / "
                          f"{abs(settle['dyaw_deg']):.2f} deg while measuring; "
                          f"re-localisation needs a stationary base")
        issues.extend(rclose.prepare_issues(state, controller, active, confirmations,
                                           check_base=False))
        if rebase_info["rebased"] and confirmations.get("drawer_unmoved") is not True:
            issues.append("operator confirmation missing: drawer_unmoved "
                          "(re-localised recipe assumes the drawer did not move in the map)")
        issues.extend(rclose.stroke_issues(measured_travel_mm, active))
        recipe = active
        frames = _capture(cam, folder, "prepare")
        check: dict[str, Any] = {"status": "not_attempted"}
        gate: dict[str, Any] = {"gate": "skipped", "reason": "no_comparable_frame"}
        wrist = [item for item in frames["frames"] if item["source"] == "wrist_right"]
        if wrist:
            check = _handle_check(recipe, folder / wrist[0]["path"])
            gate = rclose.handle_gate(check, state["tcp"], recipe)
            if gate["gate"] == "blocking":
                issues.append("wrist view does not match the recorded open-drawer view: "
                              + "; ".join(check.get("failures", [])))
        result = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                  "session": session, "phase": "prepare",
                  "status": "ready" if not issues else "blocked",
                  "recipe": {"release_tcp_base_link": rclose.release_pose(recipe),
                             "closed_tcp_base_link": rclose.closed_pose(recipe),
                             "source_session": recipe["source_session"]},
                  "state_file": db.relative(folder / "prepare_state.json"),
                  "state": state, "controller": controller, "confirmations": confirmations,
                  "rebase": rebase_info,
                  "measured_travel_mm": measured_travel_mm,
                  "recorded_stroke_mm": rclose.recorded_travel_mm(recipe),
                  "frames": frames, "handle_view": check, "handle_gate": gate,
                  "issues": issues, "motion_commands_sent": []}
        if not issues and rebase_info["rebased"]:
            replay._write_new(folder / "rebased_recipe.json", recipe)
            result["active_recipe_file"] = "rebased_recipe.json"
        path = _prepare_path(folder)
        replay._write_new(path, result)
        db.ingest_teaching_event(path, session)
        return result


def approach(session: str, index: int, io: Any, *, recipe: dict[str, Any],
             dataset_root: Path = DEFAULT_ROOT, allow_rotation: bool = False,
             guard_factory=None, require_stream: bool = False, cam=None,
             handle_check_advisory: bool = False,
             poll_timeout_s: float = 20.0, poll_interval_s: float = 0.25) -> dict[str, Any]:
    _session_ok(session)
    if not 1 <= index <= 40:
        raise ValueError("Approach step must be 1..40")
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / f"approach_{index:02d}.json"
        if journal_path.exists():
            raise FileExistsError("Approach step already recorded; no retry")
        if require_stream and not replay._stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        prepared = _fresh_prepare(folder)
        recipe = _active_recipe(folder, recipe)
        prior = _prior_state(folder, "approach", index, prepared)

        controller = io.controller()
        before = replay._observe(io, db, folder, f"approach_{index:02d}_before")
        if (not replay._controller_active(controller)
                or not rclose.OPEN_GRIPPER_RANGE_M[0] <= before["right_gripper"]["width"]
                <= rclose.OPEN_GRIPPER_RANGE_M[1]
                or before["right_gripper"]["is_moving"]
                or replay._base_shift(before, prepared["state"]) > 0.02
                or replay._chassis_shift(before, prepared["state"]) > 0.005
                or math.dist(before["tcp"][:3], prior["tcp"][:3]) > 0.008
                or rclose.angle_deg(before["tcp"][3:], prior["tcp"][3:]) > 2.0):
            raise ValueError("Controller, base, gripper or TCP changed; no command sent")

        command = rclose.approach_command(before["tcp"], recipe, allow_rotation=allow_rotation)
        target = command["target_position"]
        if command["mode"] != "clear":
            now = math.dist(before["tcp"][:3], rclose.release_pose(recipe)[:3])
            after_step = math.dist(target, rclose.release_pose(recipe)[:3])
            if after_step >= now - 1e-4:
                raise ValueError("Approach step would not get closer to the release pose")
        sustained, immediate = ((ENGAGE_SUSTAINED_N, ENGAGE_IMMEDIATE_N)
                                if command["mode"] == "engage" else (FORCE_SUSTAINED_N,
                                                                     FORCE_IMMEDIATE_N))
        journal = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "approach", "step": index,
                   "status": "attempt_started", "submission": "not_started",
                   "mode": command["mode"],
                   "release_tcp_base_link": rclose.release_pose(recipe),
                   "standoff_tcp_base_link": rclose.standoff_pose(recipe),
                   "target_tcp_position_base_link_m": target,
                   "distance_before_m": command["distance_to_release_m"],
                   "orientation_error_deg": command["orientation_error_deg"],
                   "rotation_rad": command["rotation_rad"],
                   "before_state_file": db.relative(folder / f"approach_{index:02d}_before.json"),
                   "before": before, "controller_before": controller, "command": command["command"],
                   "wrist_force_policy": {"sensor_frame": True, "sustained_delta_N": sustained,
                                          "immediate_delta_N": immediate},
                   "base_commands_sent": []}
        replay._write_new(journal_path, journal)
        guard, poll_path = None, folder / f"approach_{index:02d}_tcp_polls.ndjson"
        try:
            guard = (guard_factory or _real_guard_factory(sustained, immediate))(folder, io,
                                                                               f"approach_{index:02d}")
            guard.start()
            journal["force_baseline_sensor_frame_N"] = guard.baseline
            journal["submission"] = "started"
            journal["command_started_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            guard.motion_started.set()
            response = io.offset(command["command"])
            journal["command_result"] = response
            journal["submission"] = "responded"
            replay._update(journal_path, journal)
            if not replay._accepted(response):
                raise RuntimeError("SDK did not accept the approach offset")
            deadline = time.monotonic() + poll_timeout_s
            with poll_path.open("x", encoding="utf-8") as output:
                while True:
                    guard.check()
                    requested_ns = time.time_ns()
                    actual = replay._pose_from_reply(io.tcp())
                    active = io.controller()
                    output.write(json.dumps({"requested_ns": requested_ns,
                                             "received_ns": time.time_ns(),
                                             "pose_base_link": actual,
                                             "controller": active.get("active")},
                                            ensure_ascii=False, allow_nan=False) + "\n")
                    output.flush()
                    if not replay._controller_active(active):
                        raise RuntimeError("Right-arm PVT controller lost authority")
                    reached = all(abs(actual[axis] - target[axis]) <= 0.004 for axis in range(3))
                    if reached:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("TCP did not reach the approach target")
                    time.sleep(poll_interval_s)
            guard.check()
            if not replay._stop(io, journal):
                raise RuntimeError("Right-arm stop after approach was not confirmed")
            guard.motion_started.clear()
            after = replay._observe(io, db, folder, f"approach_{index:02d}_after")
            journal["after_state_file"] = db.relative(folder / f"approach_{index:02d}_after.json")
            journal["after"] = after
            journal["controller_after"] = io.controller()
            journal["distance_to_release_m"] = math.dist(
                after["tcp"][:3], rclose.release_pose(recipe)[:3])
            journal["observed_position_error_m"] = [
                after["tcp"][axis] - target[axis] for axis in range(3)]
            problems = []
            if not replay._controller_active(journal["controller_after"]):
                problems.append("right-arm controller lost after the step")
            if any(abs(after["tcp"][axis] - target[axis]) > 0.006 for axis in range(3)):
                problems.append("TCP did not stop at the commanded approach target")
            if not rclose.OPEN_GRIPPER_RANGE_M[0] <= after["right_gripper"]["width"] \
                    <= rclose.OPEN_GRIPPER_RANGE_M[1]:
                problems.append("gripper did not stay open during the approach")
            if replay._base_shift(after, prepared["state"]) > 0.02:
                problems.append("base moved during the approach")
            commanded_rotation = math.sqrt(sum(v * v for v in command["rotation_rad"]))
            if commanded_rotation > 0.02:
                gained = (rclose.angle_deg(before["tcp"][3:], rclose.release_pose(recipe)[3:])
                          - rclose.angle_deg(after["tcp"][3:], rclose.release_pose(recipe)[3:]))
                journal["observed_rotation_gain_deg"] = gained
                if gained < math.degrees(commanded_rotation) * 0.4:
                    problems.append("commanded rotation was not applied by the arm")
            journal["handle_view"] = None
            journal["handle_gate"] = {"gate": "skipped", "reason": "no_camera_frame"}
            wrist = _capture(cam, folder, f"approach_{index:02d}")
            journal["frames"] = wrist
            if wrist["frames"]:
                live = [item for item in wrist["frames"] if item["source"] == "wrist_right"]
                if live:
                    journal["handle_view"] = _handle_check(recipe, folder / live[0]["path"])
                    journal["handle_gate"] = rclose.handle_gate(
                        journal["handle_view"], after["tcp"], recipe,
                        advisory=handle_check_advisory)
                    if journal["handle_gate"]["gate"] == "blocking":
                        problems.append("wrist view no longer matches the recorded open-drawer "
                                        "view: " + "; ".join(
                                            journal["handle_view"].get("failures", [])))
            if problems:
                raise RuntimeError("; ".join(problems))
            journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
            if journal["submission"] != "not_started" and "stop_result" not in journal:
                replay._stop(io, journal)
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
                if guard.trip.is_set():
                    journal["status"] = "failed"
                    journal["error"] = guard.reason or "Wrist-force guard tripped"
                if guard.stop_result is not None:
                    journal.setdefault("stop_result", guard.stop_result)
                if guard.stop_error:
                    journal["stop_error"] = guard.stop_error
            if poll_path.is_file():
                journal["tcp_poll_file"] = db.relative(poll_path)
                journal["tcp_poll_sha256"] = hashlib.sha256(poll_path.read_bytes()).hexdigest()
            journal["finished_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def grip(session: str, io: Any, *, recipe: dict[str, Any], confirmations: dict[str, bool],
         dataset_root: Path = DEFAULT_ROOT, cam=None) -> dict[str, Any]:
    _session_ok(session)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / "grip.json"
        if journal_path.exists():
            raise FileExistsError("Gripper attempt already recorded; no retry")
        prepared = _fresh_prepare(folder)
        recipe = _active_recipe(folder, recipe)
        approvals = [name for name in rclose.GRIP_CONFIRMATIONS
                     if confirmations.get(name) is not True]
        if approvals:
            raise ValueError("Missing on-site confirmations: " + ", ".join(approvals))
        attempts = sorted(folder.glob("approach_[0-9][0-9].json"))
        if not attempts:
            raise ValueError("No approach step was executed yet")
        if replay._read_json(attempts[-1])["status"] != "completed":
            raise ValueError("The last approach step did not complete")

        controller = io.controller()
        before = replay._observe(io, db, folder, "grip_before")
        target = rclose.release_pose(recipe)
        problems = []
        if not replay._controller_active(controller):
            problems.append("right_arm_pvt_ctrl is not active")
        if math.dist(before["tcp"][:3], target[:3]) > 0.006:
            problems.append(f"TCP is {1000 * math.dist(before['tcp'][:3], target[:3]):.1f} mm "
                            f"from the recorded release pose")
        if rclose.angle_deg(before["tcp"][3:], target[3:]) > 3.0:
            problems.append("TCP orientation is over 3 deg from the release pose")
        if not rclose.OPEN_GRIPPER_RANGE_M[0] <= before["right_gripper"]["width"] \
                <= rclose.OPEN_GRIPPER_RANGE_M[1]:
            problems.append("right gripper is not open before gripping")
        if replay._base_shift(before, prepared["state"]) > 0.02:
            problems.append("base moved since prepare")
        if replay._chassis_shift(before, prepared["state"]) > 0.005:
            problems.append("chassis joints moved since prepare")
        frames = _capture(cam, folder, "grip_before")
        handle: dict[str, Any] = {"status": "not_attempted"}
        wrist = [item for item in frames["frames"] if item["source"] == "wrist_right"]
        if wrist:
            handle = _handle_check(recipe, folder / wrist[0]["path"])
            if handle.get("status") == "mismatch":
                problems.append("wrist view mismatch before gripping: "
                                + "; ".join(handle.get("failures", [])))
        if problems:
            raise ValueError("; ".join(problems))

        journal = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "grip", "status": "attempt_started",
                   "submission": "started", "command": recipe["grip_command"],
                   "before_state_file": db.relative(folder / "grip_before.json"),
                   "before": before, "controller_before": controller,
                   "confirmations": confirmations, "frames": frames,
                   "handle_view": handle, "arm_commands_sent": [],
                   "base_commands_sent": []}
        replay._write_new(journal_path, journal)
        try:
            response = io.grip(recipe["grip_command"])
            journal["command_result"] = response
            journal["submission"] = "responded"
            replay._update(journal_path, journal)
            if not replay._accepted(response):
                raise RuntimeError("Gripper command was not accepted by the SDK")
            after = replay._observe(io, db, folder, "grip_after")
            journal["after_state_file"] = db.relative(folder / "grip_after.json")
            journal["after"] = after
            journal["controller_after"] = io.controller()
            issues = rclose.grip_issues(before, after, recipe)
            if not replay._controller_active(journal["controller_after"]):
                issues.append("right-arm controller lost during the grip")
            if issues:
                raise RuntimeError("; ".join(issues))
            journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
        finally:
            frames = _capture(cam, folder, "grip_after")
            journal["after_frames"] = frames
            journal["finished_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def push(session: str, index: int, io: Any, *, recipe: dict[str, Any],
         dataset_root: Path = DEFAULT_ROOT, confirmations: dict[str, bool] | None = None,
         guard_factory=None, require_stream: bool = False, cam=None,
         operator_drawer_travel_mm: float | None = None,
         closed_confirmed: bool = False, poll_timeout_s: float = 20.0,
         poll_interval_s: float = 0.25) -> dict[str, Any]:
    _session_ok(session)
    confirmations = confirmations or {}
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / f"push_{index:02d}.json"
        if journal_path.exists():
            raise FileExistsError("Push step already recorded; no retry")
        if require_stream and not replay._stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        prepared = _fresh_prepare(folder)
        recipe = _active_recipe(folder, recipe)
        # Segments must come from the recipe this session actually runs, otherwise
        # a re-localised session would be checked against the original path.
        segments = rclose.push_segments(recipe)
        if not 1 <= index <= len(segments):
            raise ValueError(f"Push step must be 1..{len(segments)}")
        segment = segments[index - 1]
        grip_journal = folder / "grip.json"
        if not grip_journal.is_file():
            raise ValueError("A verified re-grip is required before pushing")
        gripped = replay._read_json(grip_journal)
        if gripped["status"] != "completed":
            raise ValueError("A verified re-grip is required before pushing")
        missing = [name for name in rclose.PUSH_CONFIRMATIONS
                   if confirmations.get(name) is not True]
        if missing:
            raise ValueError("Missing on-site confirmations: " + ", ".join(missing))
        if index > 1:
            # Refuse to stack a segment on a failed one; the geometry check below
            # cannot see why the previous step stopped.
            _prior_state(folder, "push", index, prepared)
        grip_baseline = gripped["after"]["tcp"]

        if abs(segment["dy"]) > MAX_STEP_DY_DZ_M or abs(segment["dz"]) > MAX_STEP_DY_DZ_M:
            raise ValueError("Recorded push path asks for an unsafe lateral step")

        controller = io.controller()
        before = replay._observe(io, db, folder, f"push_{index:02d}_before")
        problems = []
        if not replay._controller_active(controller):
            problems.append("right_arm_pvt_ctrl is not active")
        if not rclose.GRIPPED_RANGE_M[0] <= before["right_gripper"]["width"] \
                <= rclose.GRIPPED_RANGE_M[1] or before["right_gripper"]["effort"] < 4.0:
            problems.append("gripper is not holding a handle-sized object")
        if math.dist(before["tcp"][:3], segment["from_pose"][:3]) > 0.008:
            problems.append("TCP is not at the start of this recorded segment")
        if rclose.angle_deg(before["tcp"][3:], grip_baseline[3:]) > 4.0:
            problems.append("TCP orientation differs from the grasped orientation")
        if replay._base_shift(before, prepared["state"]) > 0.02:
            problems.append("base moved since prepare")
        if replay._chassis_shift(before, prepared["state"]) > 0.005:
            problems.append("chassis joints moved since prepare")
        if problems:
            raise ValueError("; ".join(problems))

        command = {"arm": "right_arm", "dx": segment["dx"], "dy": segment["dy"],
                   "dz": segment["dz"], "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                   "confirmed": True}
        journal = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "push", "step": index,
                   "status": "attempt_started", "submission": "not_started",
                   "segment_from_pose": segment["from_pose"],
                   "segment_to_pose": segment["to_pose"],
                   "travelled_before_m": segment["travelled_before_m"],
                   "remaining_to_closed_m": segment["remaining_to_closed_m"],
                   "near_closed": segment["near_closed"],
                   "before_state_file": db.relative(folder / f"push_{index:02d}_before.json"),
                   "before": before, "controller_before": controller,
                   "gripper_width_baseline_m": before["right_gripper"]["width"],
                   "command": command,
                   "operator_drawer_travel_mm": operator_drawer_travel_mm,
                   "wrist_force_policy": {"sensor_frame": True,
                                          "sustained_delta_N": PUSH_SUSTAINED_N,
                                          "immediate_delta_N": PUSH_IMMEDIATE_N},
                   "base_commands_sent": []}
        replay._write_new(journal_path, journal)
        guard, poll_path = None, folder / f"push_{index:02d}_tcp_polls.ndjson"
        try:
            guard = (guard_factory or _real_guard_factory(PUSH_SUSTAINED_N,
                                                          PUSH_IMMEDIATE_N))(
                folder, io, f"push_{index:02d}")
            guard.start()
            journal["force_baseline_sensor_frame_N"] = guard.baseline
            journal["submission"] = "started"
            journal["command_started_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            guard.motion_started.set()
            response = io.offset(command)
            journal["command_result"] = response
            journal["submission"] = "responded"
            replay._update(journal_path, journal)
            if not replay._accepted(response):
                raise RuntimeError("SDK did not accept the push offset")
            deadline = time.monotonic() + poll_timeout_s
            with poll_path.open("x", encoding="utf-8") as output:
                while True:
                    guard.check()
                    requested_ns = time.time_ns()
                    actual = replay._pose_from_reply(io.tcp())
                    active = io.controller()
                    output.write(json.dumps({"requested_ns": requested_ns,
                                             "received_ns": time.time_ns(),
                                             "pose_base_link": actual,
                                             "controller": active.get("active")},
                                            ensure_ascii=False, allow_nan=False) + "\n")
                    output.flush()
                    if not replay._controller_active(active):
                        raise RuntimeError("Right-arm PVT controller lost authority")
                    if abs(actual[0] - segment["to_pose"][0]) <= 0.004:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("TCP did not reach the push segment target")
                    time.sleep(poll_interval_s)
            guard.check()
            if not replay._stop(io, journal):
                raise RuntimeError("Right-arm stop after the push segment was not confirmed")
            guard.motion_started.clear()
            after = replay._observe(io, db, folder, f"push_{index:02d}_after")
            journal["after_state_file"] = db.relative(folder / f"push_{index:02d}_after.json")
            journal["after"] = after
            journal["controller_after"] = io.controller()
            journal["observed_dx_m"] = after["tcp"][0] - before["tcp"][0]
            issues = rclose.push_issues(
                before, after, segment, grip_baseline,
                grip_width_baseline_m=journal["gripper_width_baseline_m"])
            if not replay._controller_active(journal["controller_after"]):
                issues.append("right-arm controller lost after the segment")
            journal["issues"] = issues
            if issues:
                raise RuntimeError("; ".join(issues))
            last = index == len(segments)
            journal["closed"] = rclose.closed_issues(after, recipe)
            if last:
                if journal["closed"]:
                    journal["status"] = "closed_pending_confirmation"
                    journal["error"] = ("TCP is not at the closed reference; confirm the drawer "
                                        "state before releasing")
                elif not closed_confirmed and not operator_drawer_travel_mm:
                    journal["status"] = "closed_pending_confirmation"
                    journal["note"] = ("Final segment done; confirm the drawer is closed and "
                                       "re-run report with --closed-confirmed")
                else:
                    journal["status"] = "completed"
            else:
                journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
            if journal["submission"] != "not_started" and "stop_result" not in journal:
                replay._stop(io, journal)
            try:
                stopped = replay._observe(io, db, folder, f"push_{index:02d}_stopped")
                journal["stopped_state"] = stopped
            except Exception as stop_error:
                journal["stopped_state_error"] = repr(stop_error)
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
                if guard.trip.is_set():
                    journal["status"] = "failed"
                    journal["error"] = guard.reason or "Wrist-force guard tripped"
                if guard.stop_result is not None:
                    journal.setdefault("stop_result", guard.stop_result)
                if guard.stop_error:
                    journal["stop_error"] = guard.stop_error
            if poll_path.is_file():
                journal["tcp_poll_file"] = db.relative(poll_path)
                journal["tcp_poll_sha256"] = hashlib.sha256(poll_path.read_bytes()).hexdigest()
            frames = _capture(cam, folder, f"push_{index:02d}")
            journal["frames"] = frames
            journal["finished_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def release(session: str, io: Any, *, recipe: dict[str, Any],
            dataset_root: Path = DEFAULT_ROOT, confirmations: dict[str, bool] | None = None,
            cam=None) -> dict[str, Any]:
    _session_ok(session)
    confirmations = confirmations or {}
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / "release.json"
        if journal_path.exists():
            raise FileExistsError("Release already recorded; no retry")
        prepared = _fresh_prepare(folder)
        recipe = _active_recipe(folder, recipe)
        segments = rclose.push_segments(recipe)
        final_push = folder / f"push_{len(segments):02d}.json"
        if not final_push.is_file():
            raise ValueError("The final push segment has not been attempted")
        last = replay._read_json(final_push)
        if last["status"] not in ("completed", "closed_pending_confirmation"):
            raise ValueError("The final push segment did not reach the closed position")
        if confirmations.get("closed_confirmed") is not True:
            raise ValueError("Operator must confirm the drawer is closed before releasing")
        controller = io.controller()
        before = replay._observe(io, db, folder, "release_before")
        if (not replay._controller_active(controller)
                or not rclose.GRIPPED_RANGE_M[0] <= before["right_gripper"]["width"]
                <= rclose.GRIPPED_RANGE_M[1]
                or replay._base_shift(before, prepared["state"]) > 0.02):
            raise ValueError("Controller, grip or base changed; no gripper command sent")
        journal = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "release", "status": "attempt_started",
                   "submission": "started", "command": recipe["open_command"],
                   "confirmations": confirmations,
                   "before_state_file": db.relative(folder / "release_before.json"),
                   "before": before, "controller_before": controller,
                   "frames": _capture(cam, folder, "release_before"),
                   "arm_commands_sent": [], "base_commands_sent": []}
        replay._write_new(journal_path, journal)
        try:
            response = io.grip(recipe["open_command"])
            journal["command_result"] = response
            journal["submission"] = "responded"
            replay._update(journal_path, journal)
            if not replay._accepted(response):
                raise RuntimeError("Gripper open command was not accepted")
            after = replay._observe(io, db, folder, "release_after")
            journal["after_state_file"] = db.relative(folder / "release_after.json")
            journal["after"] = after
            problems = []
            if not rclose.OPEN_GRIPPER_RANGE_M[0] <= after["right_gripper"]["width"] \
                    <= rclose.OPEN_GRIPPER_RANGE_M[1]:
                problems.append("gripper did not open to the 40..60 mm range")
            if math.dist(after["tcp"][:3], before["tcp"][:3]) > 0.005:
                problems.append("TCP moved while releasing the handle")
            if problems:
                raise RuntimeError("; ".join(problems))
            journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
        finally:
            journal["finished_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def retract(session: str, io: Any, *, recipe: dict[str, Any],
            dataset_root: Path = DEFAULT_ROOT, guard_factory=None, cam=None,
            distance_m: float = 0.05) -> dict[str, Any]:
    _session_ok(session)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / "retract.json"
        if journal_path.exists():
            raise FileExistsError("Retract already recorded; no retry")
        prepared = _fresh_prepare(folder)
        recipe = _active_recipe(folder, recipe)
        release_journal = folder / "release.json"
        if not release_journal.is_file():
            raise ValueError("The handle has not been released yet")
        released = replay._read_json(release_journal)
        if released["status"] != "completed":
            raise ValueError("The handle has not been released yet")
        controller = io.controller()
        before = replay._observe(io, db, folder, "retract_before")
        if (not replay._controller_active(controller)
                or not rclose.OPEN_GRIPPER_RANGE_M[0] <= before["right_gripper"]["width"]
                <= rclose.OPEN_GRIPPER_RANGE_M[1]
                or replay._base_shift(before, prepared["state"]) > 0.02):
            raise ValueError("Controller, open gripper or base changed; no command sent")
        command = {"arm": "right_arm", "dx": -abs(distance_m), "dy": 0.0, "dz": 0.0,
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "confirmed": True}
        journal = {"schema": "drawer_regrip_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "retract", "status": "attempt_started",
                   "submission": "not_started", "command": command,
                   "before_state_file": db.relative(folder / "retract_before.json"),
                   "before": before, "controller_before": controller,
                   "base_commands_sent": []}
        replay._write_new(journal_path, journal)
        guard, poll_path = None, folder / "retract_tcp_polls.ndjson"
        try:
            guard = (guard_factory or _real_guard_factory(FORCE_SUSTAINED_N,
                                                          FORCE_IMMEDIATE_N))(
                folder, io, "retract")
            guard.start()
            journal["force_baseline_sensor_frame_N"] = guard.baseline
            journal["submission"] = "started"
            replay._update(journal_path, journal)
            guard.motion_started.set()
            response = io.offset(command)
            journal["command_result"] = response
            journal["submission"] = "responded"
            replay._update(journal_path, journal)
            if not replay._accepted(response):
                raise RuntimeError("SDK did not accept the retract offset")
            target_x = before["tcp"][0] - abs(distance_m)
            deadline = time.monotonic() + 15.0
            with poll_path.open("x", encoding="utf-8") as output:
                while True:
                    guard.check()
                    actual = replay._pose_from_reply(io.tcp())
                    output.write(json.dumps({"received_ns": time.time_ns(),
                                             "pose_base_link": actual},
                                            ensure_ascii=False, allow_nan=False) + "\n")
                    output.flush()
                    if abs(actual[0] - target_x) <= 0.005:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("TCP did not retract")
                    time.sleep(0.25)
            guard.check()
            if not replay._stop(io, journal):
                raise RuntimeError("Right-arm stop after retract was not confirmed")
            guard.motion_started.clear()
            after = replay._observe(io, db, folder, "retract_after")
            journal["after_state_file"] = db.relative(folder / "retract_after.json")
            journal["after"] = after
            if after["tcp"][0] > before["tcp"][0] - abs(distance_m) * 0.8:
                raise RuntimeError("Retract did not clear the drawer")
            journal["status"] = "completed"
        except Exception as error:
            journal["status"] = "failed"
            journal["error"] = type(error).__name__ + ": " + str(error)
            if journal["submission"] != "not_started" and "stop_result" not in journal:
                replay._stop(io, journal)
        finally:
            if guard is not None:
                guard.motion_started.clear()
                try:
                    guard.close()
                except Exception as error:
                    journal["force_close_error"] = type(error).__name__ + ": " + str(error)
                journal["force_samples"] = guard.samples
                journal["max_raw_force_delta_N"] = guard.max_delta_n
                journal["force_trip_reason"] = guard.reason
                if guard.trip.is_set():
                    journal["status"] = "failed"
                    journal["error"] = guard.reason or "Wrist-force guard tripped"
                if guard.stop_result is not None:
                    journal.setdefault("stop_result", guard.stop_result)
            frames = _capture(cam, folder, "retract_after")
            journal["frames"] = frames
            journal["finished_ns"] = time.time_ns()
            replay._update(journal_path, journal)
            db.ingest_teaching_event(journal_path, session)
        return journal


def report(session: str, *, recipe: dict[str, Any], dataset_root: Path = DEFAULT_ROOT,
           closed_confirmed: bool = False) -> dict[str, Any]:
    _session_ok(session)
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        phases = {}
        for path in sorted(folder.glob("*.json")):
            if path.name in ("prepare.json", "grip.json", "release.json",
                             "retract.json") or path.stem.split("_")[0] in ("approach", "push"):
                value = replay._read_json(path)
                if isinstance(value, dict) and value.get("schema") == "drawer_regrip_phase/1":
                    phases[path.stem] = value.get("status")
        segments = rclose.push_segments(recipe)
        pushes = [phases.get(f"push_{index:02d}") for index in range(1, len(segments) + 1)]
        completed = sum(1 for status in pushes if status == "completed")
        pending = sum(1 for status in pushes if status == "closed_pending_confirmation")
        travel = None
        for index in range(len(segments), 0, -1):
            path = folder / f"push_{index:02d}.json"
            if path.is_file():
                travel = replay._read_json(path).get("operator_drawer_travel_mm")
                break
        closed_ok = bool(closed_confirmed) and phases.get("release") == "completed" \
            and phases.get("retract") == "completed"
        result = {"schema": "drawer_regrip_report/1", "recorded_ns": time.time_ns(),
                  "session": session, "phases": phases,
                  "push_segments_expected": len(segments),
                  "push_segments_completed": completed,
                  "push_segments_pending_confirmation": pending,
                  "push_segments_missing": sum(1 for status in pushes if status is None),
                  "push_segments_failed": sum(1 for status in pushes
                                              if status not in (None, "completed",
                                                                "closed_pending_confirmation")),
                  "recorded_stroke_mm": rclose.recorded_travel_mm(recipe),
                  "operator_drawer_travel_mm": travel,
                  "drawer_closed_verified": closed_ok,
                  "verified_on_robot": recipe.get("verified_on_robot", False),
                  "limitations": recipe.get("limitations", []),
                  "motion_commands_sent": []}
        path = folder / "report.json"
        if path.exists():
            path.replace(folder / f"report.{time.time_ns()}.previous.json")
        replay._write_new(path, result)
        db.ingest_teaching_event(path, session)
        return result


def plan(recipe: dict[str, Any]) -> dict[str, Any]:
    return rclose.plan(recipe)


# --------------------------------------------------------------------- CLI

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("phase", nargs="?", default="plan",
                        choices=("plan", "recipe", "prepare", "approach", "grip", "push",
                                 "release", "retract", "report"))
    parser.add_argument("--session")
    parser.add_argument("--step", type=int)
    parser.add_argument("--execute", action="store_true", help="Enable one guarded live phase")
    parser.add_argument("--recipe", default=str(rclose.RECIPE_PATH))
    parser.add_argument("--allow-rotation", action="store_true",
                        help="Send clamped orientation offsets during approach")
    parser.add_argument("--no-camera", action="store_true", help="Skip evidence frame capture")
    parser.add_argument("--handle-check-advisory", action="store_true",
                        help="Record wrist-view mismatches without blocking the approach step")
    parser.add_argument("--require-stream", action="store_true",
                        help="Require the continuous recorder to be writing")
    parser.add_argument("--from-session", help="Pull session used to rebuild the recipe")
    parser.add_argument("--drawer-travel-mm", type=float,
                        help="Operator tape measurement of drawer travel for this push step")
    parser.add_argument("--measured-travel-mm", type=float,
                        help="Operator tape measurement of how far the drawer actually pulled "
                             "out; prepare blocks if it contradicts the recorded stroke")
    for name in ("handle_clear", "path_clear", "exclusive_control", "emergency_stop_ready",
                 "operator_present", "handle_between_fingers_confirmed",
                 "operator_hands_clear_confirmed", "drawer_supported", "closed_confirmed",
                 "drawer_unmoved"):
        parser.add_argument("--" + name.replace("_", "-") + "-confirmed", action="store_true")
    args = parser.parse_args(argv)
    confirmations = {name: getattr(args, name + "_confirmed") for name in (
        "handle_clear", "path_clear", "exclusive_control", "emergency_stop_ready",
        "operator_present", "handle_between_fingers_confirmed",
        "operator_hands_clear_confirmed", "drawer_supported", "closed_confirmed",
        "drawer_unmoved")}

    if args.phase == "recipe":
        if not args.from_session:
            parser.error("recipe requires --from-session drawer_pull_...")
        previous = json.loads((rclose.WORKSPACE / "galbot-hackathon/scenes/scene_medicine_check/"
                               "data/recorded_drawer_pull_20261107.json").read_text(encoding="utf-8"))
        source = DEFAULT_ROOT / "teaching" / args.from_session / "final_state.json"
        built = rclose.make_recipe(source, previous["pre_grasp_tcp_base_link"])
        Path(args.recipe).write_text(json.dumps(built, ensure_ascii=False, indent=2) + "\n",
                                     encoding="utf-8")
        print(json.dumps({"written": args.recipe, "release": built["release_tcp_base_link"],
                          "closed": built["closed_tcp_base_link"]}, ensure_ascii=False, indent=2))
        return 0

    recipe = rclose.load_recipe(Path(args.recipe))
    if args.phase == "plan" or not args.execute:
        print(json.dumps(plan(recipe), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    if not args.session:
        parser.error("--session is required with --execute")
    if args.phase == "report":  # offline: reads the session journals only
        result = report(args.session, recipe=recipe,
                        closed_confirmed=confirmations["closed_confirmed"])
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if result.get("drawer_closed_verified") else 1
    code = os.environ.get("GALBOT_CONSOLE_CODE")
    if not code:
        parser.error("Current GALBOT_CONSOLE_CODE must be set in the process environment")
    try:
        io = replay.ConsoleIO(code)
    except Exception as error:
        print(json.dumps({
            "status": "console_unreachable",
            "detail": type(error).__name__ + ": " + str(error),
            "hint": "8080 SDK 控制台不可达或访问码已失效；本轮没有发送任何命令。"
                    "先恢复控制台连通、取本次新访问码，再重跑这一步。",
        }, ensure_ascii=False, indent=2))
        return 2
    cam = None if args.no_camera else _camera(os.environ.get("GALBOT_CAM_CODE"))

    if args.phase == "prepare":
        result = prepare(args.session, io, recipe=recipe, confirmations=confirmations, cam=cam,
                         measured_travel_mm=args.measured_travel_mm)
    elif args.phase == "approach":
        if args.step is None:
            parser.error("--step is required for approach")
        result = approach(args.session, args.step, io, recipe=recipe,
                          allow_rotation=args.allow_rotation, require_stream=args.require_stream,
                          cam=cam, handle_check_advisory=args.handle_check_advisory)
    elif args.phase == "grip":
        result = grip(args.session, io, recipe=recipe, confirmations=confirmations, cam=cam)
    elif args.phase == "push":
        if args.step is None:
            parser.error("--step is required for push")
        result = push(args.session, args.step, io, recipe=recipe, confirmations=confirmations,
                      require_stream=args.require_stream, cam=cam,
                      operator_drawer_travel_mm=args.drawer_travel_mm,
                      closed_confirmed=confirmations["closed_confirmed"])
    elif args.phase == "release":
        result = release(args.session, io, recipe=recipe, confirmations=confirmations, cam=cam)
    else:
        result = retract(args.session, io, recipe=recipe, cam=cam)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") in ("ready", "completed", "closed_pending_confirmation") else 1


if __name__ == "__main__":
    raise SystemExit(main())
