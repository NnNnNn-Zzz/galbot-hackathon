"""Build the immutable checkpoint record and portable recipe for the drawer pull.

This reads local data only. It cannot recover continuous joint trajectories or
drawer displacement between the previously captured checkpoints.
"""

import hashlib
import json
import math
from pathlib import Path

from medicine_frontend.dataset import Dataset


ALIGNMENT = "drawer_final_20261107_121006"
GRIP = "drawer_grip_verify_20261107_121348"
PULL = "drawer_pull_20261107_122158"
STEP_FILES = [*(f"arm_pull_step_{i:02d}.json" for i in range(1, 7)),
              *(f"arm_extend_step_{i:02d}.json" for i in range(1, 3))]
EXPECTED_DISTANCES = [0.02, 0.07, 0.12, 0.17, 0.22, 0.25, 0.275, 0.30]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_once(path, data):
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_bytes() != raw:
            raise ValueError(f"Existing record differs; preserve immutable source: {path}")
    else:
        path.write_bytes(raw)


def tcp(state):
    return state["reads"]["right_end_effector"]["data"]["pose"]


def base(state):
    return state["reads"]["navigation"]["data"]["pose"]


def joints(state):
    values = {item["name"]: item["position"]
              for item in state["reads"]["robot"]["data"]["joint_state"]}
    return {name: values[name] for name in (f"right_arm_joint{i}" for i in range(1, 8))}


def gripper(state):
    observations = [item["event"]["data"]["grippers"]["right"]
                    for item in state["events"]
                    if item["event"]["type"] == "robot_sensors"]
    if not observations:
        raise ValueError("Missing right gripper observations")
    return observations[-1]


def orientation_delta_deg(a, b):
    dot = abs(sum(x * y for x, y in zip(a[3:], b[3:])))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))


def summary(state):
    return {"tcp_base_link": tcp(state), "base_map": base(state),
            "right_arm_joints_rad": joints(state), "right_gripper": gripper(state),
            "sample_finished_ns": state["finished_ns"], "read_errors": state["errors"]}


def main():
    with Dataset() as db:
        root = db.root
        align_folder = root / "teaching" / ALIGNMENT
        grip_folder = root / "teaching" / GRIP
        pull_folder = root / "teaching" / PULL
        alignment = read_json(align_folder / "sdk_snapshot.json")
        grip_record = read_json(grip_folder / "verification.json")
        grip_before = read_json(grip_folder / "pre_command_state.json")
        grip_after = read_json(grip_folder / "post_command_state.json")
        pull_before = read_json(pull_folder / "pre_state.json")
        pull_after = read_json(pull_folder / "final_state.json")
        extension_before = read_json(pull_folder / "extension_pre_state.json")

        if any(state["errors"] for state in (alignment, grip_before, grip_after,
                                             pull_before, extension_before, pull_after)):
            raise ValueError("A required state snapshot contains read errors")
        if grip_record["command_result"].get("sdk_result") != "ControlStatus.SUCCESS":
            raise ValueError("Grip command was not accepted")
        events = []
        for index, filename in enumerate(STEP_FILES):
            event_path = pull_folder / filename
            event = read_json(event_path)
            if not math.isclose(event.get("overall_target_cumulative_m",
                                          event["target_cumulative_m"]),
                                EXPECTED_DISTANCES[index], abs_tol=1e-8):
                raise ValueError(f"Unexpected segment target in {filename}")
            before_path = pull_folder / filename.replace(".json", "_before.json")
            after_path = pull_folder / filename.replace(".json", "_after.json")
            before = read_json(before_path)
            after = read_json(after_path)
            if event.get("command_result", {}).get("sdk_result") != "MotionStatus.SUCCESS":
                raise ValueError(f"Unsuccessful SDK motion result in {filename}")
            if before["errors"] or after["errors"]:
                raise ValueError(f"Incomplete state in {filename}")
            events.append({
                "order": index + 1,
                "phase": "initial_25cm" if index < 6 else "extra_5cm_current_height",
                "command_file": db.relative(event_path),
                "command_sha256": digest(event_path),
                "command_started_ns": event["started_ns"],
                "command_finished_ns": event["command_finished_ns"],
                "command_payload": event["command"],
                "sdk_result": event["command_result"]["sdk_result"],
                "target_cumulative_x_m": EXPECTED_DISTANCES[index],
                "target_tcp_base_link": event["target_tcp"],
                "before_state_file": db.relative(before_path),
                "before_state_sha256": digest(before_path),
                "before": summary(before),
                "after_state_file": db.relative(after_path),
                "after_state_sha256": digest(after_path),
                "after": summary(after),
                "target_and_other_axes_within_tolerance": event["target_reached"],
                "stop_result": event.get("stop_result"),
                "error": event.get("error"),
            })

        frames = []
        for item in db.records():
            if item["session"] not in (ALIGNMENT, GRIP, PULL):
                continue
            frame = {"session": item["session"], "source": item["source"],
                     "timestamp_ns": item["timestamp_ns"],
                     "image_file": item["image_path"], "image_sha256": item["image_sha256"],
                     "quality_flags": item["quality_flags"],
                     "depth_file": item["depth_path"]}
            if item["depth_path"]:
                frame["depth_sha256"] = digest(root / item["depth_path"])
            frames.append(frame)
        frames.sort(key=lambda item: (item["timestamp_ns"] or 0, item["source"]))
        for frame in frames:
            if digest(root / frame["image_file"]) != frame["image_sha256"]:
                raise ValueError(f"Camera image changed: {frame['image_file']}")
        state_files = [
            {"session": row["reference_session"], "file": row["file_path"],
             "sha256": row["sha256"], "recorded_ns": row["recorded_ns"]}
            for row in db.db.execute(
                "SELECT reference_session,file_path,sha256,recorded_ns FROM state_snapshots "
                "WHERE reference_session IN (?,?,?) ORDER BY recorded_ns",
                (ALIGNMENT, GRIP, PULL))
        ]
        for snapshot in state_files:
            if digest(root / snapshot["file"]) != snapshot["sha256"]:
                raise ValueError(f"State snapshot changed: {snapshot['file']}")

        position_delta = [tcp(pull_after)[i] - tcp(pull_before)[i] for i in range(3)]
        extra_delta = [tcp(pull_after)[i] - tcp(extension_before)[i] for i in range(3)]
        manifest = {
            "schema": "drawer_demonstration/1", "recorded_ns": pull_after["finished_ns"],
            "source_sessions": {"alignment": ALIGNMENT, "grip": GRIP, "pull": PULL},
            "robot": "Galbot G1", "sdk_version_at_site": "1.10",
            "coordinate_frame": "base_link for TCP; map for base navigation observation",
            "operator_reports": [
                "At alignment, operator confirmed the gripper was correctly placed around the handle.",
                "For the pull, operator confirmed a person would steady the drawer and the rear path was clear.",
                "After 25 cm, operator confirmed at least 5 cm more travel and approved using then-current TCP height.",
            ],
            "alignment_reference": summary(alignment),
            "grip": {"command_file": db.relative(grip_folder / "verification.json"),
                     "command_sha256": digest(grip_folder / "verification.json"),
                     "command_payload": grip_record["command"],
                     "command_started_ns": grip_record["command_started_ns"],
                     "command_finished_ns": grip_record["command_finished_ns"],
                     "sdk_result": grip_record["command_result"]["sdk_result"],
                     "before": summary(grip_before), "after": summary(grip_after)},
            "pull_start": summary(pull_before),
            "extra_5cm_start": summary(extension_before),
            "pull_steps": events,
            "pull_summary_file": db.relative(pull_folder / "pull_record.json"),
            "pull_summary_sha256": digest(pull_folder / "pull_record.json"),
            "final": {**summary(pull_after),
                      "observed_tcp_delta_from_pull_start_m": position_delta,
                      "observed_extra_5cm_tcp_delta_m": extra_delta,
                      "orientation_delta_from_pull_start_deg": orientation_delta_deg(
                          tcp(pull_before), tcp(pull_after)),
                      "base_commands_sent": [],
                      "final_right_arm_stop_result": events[-1]["stop_result"]},
            "camera_frames": frames,
            "state_snapshot_files": state_files,
            "sampling_scope": {
                "continuous_joint_trajectory_recorded": False,
                "continuous_tcp_trajectory_recorded": False,
                "calibrated_drawer_slide_displacement_recorded": False,
                "direct_force_torque_recorded": False,
                "camera_and_sdk_clock_synchronized": False,
                "human_support_force_recorded": False,
                "checkpoint_states_and_rgbd_recorded": True,
            },
            "replay_assessment": {
                "command_sequence_reconstructable": True,
                "exact_physical_trajectory_reconstructable": False,
                "all_other_tcp_axes_held_constant": False,
                "safe_for_unattended_automatic_replay": False,
                "reason": "At 25 and 30 cm the TCP exceeded the 5 mm Y/Z guard; "
                          "the pose-goal API does not enforce a straight Cartesian path.",
            },
        }
        manifest_path = pull_folder / "demonstration.json"
        write_once(manifest_path, manifest)

        recipe = {
            "schema": "drawer_replay_recipe/1",
            "source_demonstration": db.relative(manifest_path),
            "source_demonstration_sha256": digest(manifest_path),
            "alignment_tcp_base_link": tcp(pull_before),
            "pre_grasp_tcp_base_link": tcp(grip_before),
            "alignment_base_map": base(pull_before),
            "alignment_right_arm_joints_rad": joints(pull_before),
            "pre_grasp_right_gripper_width_m": gripper(grip_before)["width"],
            "grip_command": {key: grip_record["command"][key]
                             for key in ("side", "width", "speed", "effort")},
            "pull_targets_cumulative_x_m": EXPECTED_DISTANCES,
            "issued_command_payloads": [event["command_payload"] for event in events],
            "observed_tcp_checkpoints_base_link": [event["after"]["tcp_base_link"]
                                                     for event in events],
            "recorded_stop_after_step": [index + 1 for index, event in enumerate(events)
                                         if event["stop_result"] is not None],
            "rebase_y_z_orientation_at_m": 0.25,
            "frame_id": "base_link",
            "control_semantics": "short pose-goal offsets with observed feedback after each step",
            "observed_final_tcp_delta_m": position_delta,
            "requires_operator_alignment_and_drawer_support": True,
            "requires_operator_approval_for_extension_after_25cm": True,
            "verified_for_unattended_real_replay": False,
        }
        repo_root = Path(__file__).resolve().parents[2]
        recipe_path = (repo_root / "galbot-hackathon" / "scenes" /
                       "scene_medicine_check" / "data" / "recorded_drawer_pull_20261107.json")
        write_once(recipe_path, recipe)

        indexed = [db.ingest_teaching_event(grip_folder / "verification.json", PULL)]
        indexed.extend(db.ingest_teaching_event(pull_folder / name, PULL)
                       for name in STEP_FILES)
        indexed.append(db.ingest_teaching_event(manifest_path, PULL))
        print(json.dumps({"demonstration": str(manifest_path), "recipe": str(recipe_path),
                          "teaching_events": len(indexed), "camera_frames": len(frames),
                          "observed_tcp_delta_m": position_delta}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
