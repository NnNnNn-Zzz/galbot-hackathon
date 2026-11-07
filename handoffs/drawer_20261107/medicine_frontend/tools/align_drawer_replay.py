"""One guarded right-TCP alignment step toward the saved drawer pre-grasp pose.

Default CLI use is local dry run. Live execution requires --execute and sends
at most one positive X offset (<=20 mm). Optional Y/Z corrections are each
<=5 mm. No base, gripper or controller switch command is sent. A separate
drawer_alignment_* session preserves every attempted step without retries.
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

from medicine_frontend.dataset import Dataset, DEFAULT_ROOT
from medicine_frontend.tools import replay_drawer_pull as replay


SESSION_PATTERN = re.compile(r"drawer_alignment_[0-9]{8}_[0-9]{6}(?:_[A-Za-z0-9_-]+)?\Z")
CONFIRMATIONS = ("handle_clear", "path_clear", "exclusive_control",
                 "emergency_stop_ready", "operator_present")


def _session_ok(session: str) -> None:
    if not SESSION_PATTERN.fullmatch(session):
        raise ValueError("Use a new drawer_alignment_YYYYMMDD_HHMMSS session")


def _open_gripper(state: dict[str, Any]) -> bool:
    grip = state["right_gripper"]
    return 0.040 <= grip["width"] <= 0.060 and not grip["is_moving"]


def _reference() -> tuple[list[float], dict[str, Any]]:
    recipe = replay._load_recipe()
    source = (replay.WORKSPACE / "medicine_frontend/data/medicine_dataset" /
              recipe["source_demonstration"])
    demonstration = replay._read_json(source)
    final = dict(demonstration["final"])
    final_state = replay._read_json(source.parent / "final_state.json")
    final["chassis_joints_rad"] = replay._extract(
        final_state, fresh=False)["chassis_joints_rad"]
    return recipe["pre_grasp_tcp_base_link"], final


def _starting_issues(state: dict[str, Any], target: list[float],
                     final: dict[str, Any]) -> list[str]:
    issues = []
    x_gap = target[0] - state["tcp"][0]
    if not 0.005 < x_gap <= 0.35:
        issues.append("TCP X must be 5..350 mm behind saved pre-grasp")
    if any(abs(state["tcp"][axis] - target[axis]) > 0.030 for axis in (1, 2)):
        issues.append("TCP Y or Z differs by over 30 mm")
    if replay._angle_deg(state["tcp"][3:], target[3:]) > 5.0:
        issues.append("TCP orientation differs by over 5 degrees")
    if replay._base_shift(state, {"base_map": final["base_map"]}) > 0.08:
        issues.append("base map differs by over 80 mm from saved final pose")
    if replay._chassis_shift(state, {"chassis_joints_rad":
                                     final["chassis_joints_rad"]}) > 0.005:
        issues.append("chassis joint positions differ from saved pulled-out pose")
    return issues


def prepare(session: str, io: Any, *, confirmations: dict[str, bool],
            dataset_root: Path = DEFAULT_ROOT) -> dict[str, Any]:
    """Read-only current-state capture and operator-confirmed start guard."""
    _session_ok(session)
    target, final = _reference()
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        if folder.exists() and any(
                item.name not in ("continuous", "camera_stream") or not item.is_dir()
                for item in folder.iterdir()):
            raise FileExistsError("Alignment attempt already exists; use a new session")
        folder.mkdir(parents=True, exist_ok=True)
        controller = io.controller()
        state = replay._observe(io, db, folder, "prepare_state")
        issues = _starting_issues(state, target, final)
        if not replay._controller_active(controller):
            issues.append("right_arm_pvt_ctrl is not active")
        if not _open_gripper(state):
            issues.append("right gripper is not open 40..60 mm and stationary")
        for name in CONFIRMATIONS:
            if confirmations.get(name) is not True:
                issues.append(f"operator confirmation missing: {name}")
        result = {"schema": "drawer_alignment_phase/1", "recorded_ns": time.time_ns(),
                  "session": session, "phase": "prepare",
                  "status": "ready" if not issues else "blocked",
                  "target_tcp_base_link": target,
                  "reference_final_tcp_base_link": final["tcp_base_link"],
                  "state_file": db.relative(folder / "prepare_state.json"), "state": state,
                  "controller": controller, "confirmations": confirmations,
                  "issues": issues, "motion_commands_sent": []}
        path = folder / "prepare.json"
        replay._write_new(path, result)
        db.ingest_teaching_event(path, session)
        return result


def _prior(folder: Path, index: int) -> dict[str, Any]:
    if index == 1:
        prepared = replay._read_json(folder / "prepare.json")
        if prepared["status"] != "ready":
            raise ValueError("Alignment preparation is not ready")
        if time.time_ns() - prepared["recorded_ns"] > 600_000_000_000:
            raise ValueError("Alignment preparation is older than ten minutes")
        return prepared["state"]
    previous = replay._read_json(folder / f"step_{index - 1:02d}.json")
    if previous["status"] != "completed":
        raise ValueError("Previous alignment step did not complete")
    return previous["after"]


def step(session: str, index: int, dx: float, dy: float, dz: float, io: Any, *,
         dataset_root: Path = DEFAULT_ROOT, require_stream: bool = True,
         guard_factory=None, poll_timeout_s: float = 10.0,
         poll_interval_s: float = 0.25) -> dict[str, Any]:
    """Send one explicit positive-X offset, then stop and save observed state."""
    _session_ok(session)
    if not 1 <= index <= 99:
        raise ValueError("Step must be 1..99")
    if (not all(replay._finite(v) for v in (dx, dy, dz))
            or not 0 < dx <= 0.020 or abs(dy) > 0.005 or abs(dz) > 0.005):
        raise ValueError("Require 0 < dx <=20 mm and |dy|,|dz| <=5 mm")
    guard_factory = guard_factory or replay._real_force_guard
    target, _ = _reference()
    with Dataset(dataset_root) as db:
        folder = db.root / "teaching" / session
        journal_path = folder / f"step_{index:02d}.json"
        if journal_path.exists():
            raise FileExistsError("Alignment attempt already recorded; no retry")
        if require_stream and not replay._stream_active(folder):
            raise ValueError("Continuous event and TCP recorder is not active")
        prepared = replay._read_json(folder / "prepare.json")
        if prepared["status"] != "ready":
            raise ValueError("Alignment preparation is not ready")
        baseline = prepared["state"]
        prior = _prior(folder, index)
        if prior["tcp"][0] + dx > target[0] + 0.005:
            raise ValueError("Step would pass the saved pre-grasp X point")
        if (abs(prior["tcp"][1] + dy - target[1]) >
                abs(prior["tcp"][1] - target[1]) + 1e-6
                or abs(prior["tcp"][2] + dz - target[2]) >
                abs(prior["tcp"][2] - target[2]) + 1e-6):
            raise ValueError("Y/Z corrections must move toward the saved pre-grasp")
        controller = io.controller()
        before = replay._observe(io, db, folder, f"step_{index:02d}_before")
        if (not replay._controller_active(controller) or not _open_gripper(before)
                or replay._base_shift(before, baseline) > 0.03
                or replay._chassis_shift(before, baseline) > 0.005
                or math.dist(before["tcp"][:3], prior["tcp"][:3]) > 0.005
                or replay._angle_deg(before["tcp"][3:], prior["tcp"][3:]) > 2.0
                or any(abs(before["right_arm_joints_rad"][name] - value) > 0.02
                       for name, value in prior["right_arm_joints_rad"].items())):
            raise ValueError("Controller, open gripper, chassis, TCP or joints changed")
        requested = [before["tcp"][0] + dx, before["tcp"][1] + dy,
                     before["tcp"][2] + dz]
        if requested[0] > target[0] + 0.005:
            raise ValueError("Step would pass the saved pre-grasp X point")
        if (abs(requested[1] - target[1]) > abs(before["tcp"][1] - target[1]) + 1e-6
                or abs(requested[2] - target[2]) > abs(before["tcp"][2] - target[2]) + 1e-6):
            raise ValueError("Y/Z corrections must move toward the saved pre-grasp")
        command = {"arm": "right_arm", "dx": dx, "dy": dy, "dz": dz,
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "confirmed": True}
        journal = {"schema": "drawer_alignment_phase/1", "recorded_ns": time.time_ns(),
                   "session": session, "phase": "step", "step": index,
                   "status": "attempt_started", "submission": "not_started",
                   "saved_pre_grasp_tcp_base_link": target,
                   "target_tcp_position_base_link_m": requested,
                   "before_state_file": db.relative(folder / f"step_{index:02d}_before.json"),
                   "before": before, "controller_before": controller,
                   "command": command, "base_commands_sent": [],
                   "gripper_commands_sent": [],
                   "wrist_force_policy": {"sensor_frame": True, "baseline_samples": 20,
                                          "sustained_delta_N": 5.0,
                                          "immediate_delta_N": 8.0,
                                          "sample_interval_s": 0.05}}
        replay._write_new(journal_path, journal)
        guard = None
        poll_path = folder / f"step_{index:02d}_tcp_polls.ndjson"
        try:
            guard = guard_factory(folder, io, index)
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
                raise RuntimeError("SDK did not accept alignment offset")
            deadline = time.monotonic() + poll_timeout_s
            with poll_path.open("x", encoding="utf-8") as output:
                while True:
                    guard.check()
                    if require_stream and not replay._stream_active(folder):
                        raise RuntimeError("Continuous recorder stopped during alignment")
                    requested_ns = time.time_ns()
                    actual = replay._pose_from_reply(io.tcp())
                    active = io.controller()
                    output.write(json.dumps({
                        "requested_ns": requested_ns, "received_ns": time.time_ns(),
                        "pose_base_link": actual, "controller": active.get("active")},
                        ensure_ascii=False, allow_nan=False) + "\n")
                    output.flush()
                    if not replay._controller_active(active):
                        raise RuntimeError("Right-arm PVT controller lost authority")
                    if abs(actual[0] - requested[0]) <= 0.004:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("TCP did not reach alignment X target")
                    time.sleep(poll_interval_s)
            guard.check()
            if not replay._stop(io, journal):
                raise RuntimeError("Right-arm stop after alignment was not confirmed")
            guard.motion_started.clear()
            after = replay._observe(io, db, folder, f"step_{index:02d}_after")
            journal["after_state_file"] = db.relative(folder / f"step_{index:02d}_after.json")
            journal["after"] = after
            journal["controller_after"] = io.controller()
            if (not replay._controller_active(journal["controller_after"])
                    or not _open_gripper(after)
                    or any(abs(after["tcp"][axis] - requested[axis]) > 0.005
                           for axis in range(3))
                    or after["tcp"][0] > target[0] + 0.005
                    or replay._angle_deg(after["tcp"][3:], target[3:]) > 5.0
                    or replay._base_shift(after, baseline) > 0.03
                    or replay._chassis_shift(after, baseline) > 0.005):
                raise RuntimeError("After-step TCP, base, gripper or controller guard failed")
            journal["distance_to_pre_grasp_m"] = math.dist(after["tcp"][:3], target[:3])
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


def plan() -> dict[str, Any]:
    target, final = _reference()
    return {"dry_run": True, "motion_commands_sent": [],
            "saved_pre_grasp_tcp_base_link": target,
            "saved_pulled_out_tcp_base_link": final["tcp_base_link"],
            "one_command_per_invocation": True,
            "allowed_dx_m": "0 < dx <= 0.020",
            "allowed_dy_dz_m": "each |offset| <= 0.005, toward saved reference",
            "requires_active_controller": "right_arm_pvt_ctrl",
            "requires_open_gripper_m": [0.040, 0.060],
            "requires_continuous_recording": True,
            "no_automatic_loop_or_base_motion": True}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", nargs="?", choices=("plan", "prepare", "step"), default="plan")
    parser.add_argument("--session")
    parser.add_argument("--step", type=int)
    parser.add_argument("--dx", type=float)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--execute", action="store_true")
    for name in CONFIRMATIONS:
        parser.add_argument("--" + name.replace("_", "-") + "-confirmed", action="store_true")
    args = parser.parse_args(argv)
    if args.phase == "plan" or not args.execute:
        print(json.dumps(plan(), ensure_ascii=False, indent=2))
        return 0
    if not args.session:
        parser.error("--session is required with --execute")
    code = os.environ.get("GALBOT_CONSOLE_CODE")
    if not code:
        parser.error("Current GALBOT_CONSOLE_CODE must be set in the process environment")
    io = replay.ConsoleIO(code)
    if args.phase == "prepare":
        confirmations = {name: getattr(args, name + "_confirmed") for name in CONFIRMATIONS}
        result = prepare(args.session, io, confirmations=confirmations)
    else:
        if args.step is None or args.dx is None:
            parser.error("step requires explicit --step and --dx")
        result = step(args.session, args.step, args.dx, args.dy, args.dz, io)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") in ("ready", "completed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
