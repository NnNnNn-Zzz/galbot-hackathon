"""Run one guarded Cartesian right-arm pull step on the existing SDK console.

The base is never commanded. Each target keeps the original base_link Y, Z and
quaternion, and the step is recorded before transmission to prevent retries.
The live console's offset command is a pose goal, so intermediate path geometry
is not guaranteed; use short steps and verify after each one.
"""

import argparse
import http.cookiejar
import json
import math
import os
import time
from urllib import request

from medicine_frontend.dataset import Dataset
from medicine_frontend.tools.capture_final_drawer import CONSOLE, sdk_snapshot


SESSION = "drawer_pull_20261107_122158"
TARGETS_M = (0.02, 0.07, 0.12, 0.17, 0.22, 0.25)
EXTENSION_TARGETS_M = (0.025, 0.05)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def gripper(state):
    events = [item["event"]["data"]["grippers"]["right"]
              for item in state["events"] if item["event"]["type"] == "robot_sensors"]
    if not events:
        raise RuntimeError("Right gripper sensor feedback missing")
    return events[-1]


def nav(state):
    return state["reads"]["navigation"]["data"]["pose"]


def pose(state):
    return state["reads"]["right_end_effector"]["data"]["pose"]


def angle_between(q1, q2):
    dot = abs(sum(float(a) * float(b) for a, b in zip(q1, q2)))
    return 2 * math.acos(max(-1.0, min(1.0, dot)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=int)
    parser.add_argument("--extension", action="store_true",
                        help="Use the user-approved current-height baseline for the extra 5 cm")
    args = parser.parse_args()
    targets = EXTENSION_TARGETS_M if args.extension else TARGETS_M
    if args.step not in range(1, len(targets) + 1):
        parser.error("step is outside the selected sequence")
    prefix = "arm_extend_step" if args.extension else "arm_pull_step"
    code = os.environ["GALBOT_CONSOLE_CODE"]
    with Dataset() as db:
        folder = db.root / "teaching" / SESSION
        initial_name = "extension_pre_state.json" if args.extension else "pre_state.json"
        initial = json.loads((folder / initial_name).read_text(encoding="utf-8"))
        if initial["errors"]:
            raise RuntimeError("Initial state has read errors")
        initial_pose = pose(initial)
        initial_nav = nav(initial)
        planned_x = initial_pose[0] - targets[args.step - 1]
        expected_x = initial_pose[0] - (targets[args.step - 2] if args.step > 1 else 0)
        record_path = folder / f"{prefix}_{args.step:02d}.json"
        if record_path.exists():
            raise RuntimeError("Step record exists; refusing duplicate command")
        if args.step > 1:
            previous_path = folder / f"{prefix}_{args.step - 1:02d}.json"
            if not previous_path.exists() or not json.loads(previous_path.read_text(
                    encoding="utf-8")).get("target_reached"):
                raise RuntimeError("Previous step did not finish successfully")

        before = sdk_snapshot(code)
        if before["errors"]:
            raise RuntimeError("Current state read failed: " + repr(before["errors"]))
        before_path = folder / f"{prefix}_{args.step:02d}_before.json"
        write_json(before_path, before)
        db.ingest_state(before_path, SESSION)
        current = pose(before)
        g = gripper(before)
        base_drift = math.hypot(nav(before)["x"] - initial_nav["x"],
                                nav(before)["y"] - initial_nav["y"])
        if (abs(current[0] - expected_x) > 0.008
                or abs(current[1] - initial_pose[1]) > 0.005
                or abs(current[2] - initial_pose[2]) > 0.005
                or angle_between(current[3:], initial_pose[3:]) > math.radians(2)
                or base_drift > 0.03
                or not 0.020 <= g["width"] <= 0.030
                or g["effort"] < 4 or g["is_moving"]):
            raise RuntimeError("Pose, base or grasp precondition changed; no motion command sent")

        command = {"arm": "right_arm", "dx": planned_x - current[0],
                   "dy": initial_pose[1] - current[1],
                   "dz": initial_pose[2] - current[2],
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "confirmed": True}
        if not -0.1 <= command["dx"] < 0:
            raise RuntimeError("Unsafe offset outside console limit")
        record = {"schema": "drawer_arm_pull_step/1", "session": SESSION,
                  "sequence": "current_height_extension" if args.extension else "original_pull",
                  "step": args.step, "target_cumulative_m": targets[args.step - 1],
                  "overall_target_cumulative_m": 0.25 + targets[args.step - 1]
                  if args.extension else targets[args.step - 1],
                  "initial_tcp": initial_pose, "target_tcp": [planned_x, *initial_pose[1:]],
                  "initial_base_pose": initial_nav, "before_tcp": current,
                  "before_gripper": g, "before_base_drift_m": base_drift,
                  "command": command, "submission": "started", "started_ns": time.time_ns(),
                  "base_commands_sent": []}
        write_json(record_path, record)

        jar = http.cookiejar.CookieJar()
        opener = request.build_opener(request.HTTPCookieProcessor(jar))
        def call(path, data=None):
            payload = None if data is None else json.dumps(data).encode()
            req = request.Request(CONSOLE + path, data=payload,
                                  headers={"Content-Type": "application/json"})
            with opener.open(req, timeout=20) as response:
                return json.load(response)

        def stop_arm():
            try:
                record["stop_result"] = call("/api/commands/motion/stop_controller",
                                             {"group": "right_arm"})
            except Exception as error:
                record["stop_error"] = type(error).__name__ + ": " + str(error)

        try:
            login = call("/api/auth/login", {"code": code})
            if login.get("authenticated") is not True:
                raise RuntimeError("Console login failed")
            result = call("/api/commands/motion/move_end_effector_offset", command)
            record["command_result"] = result
            record["submission"] = "responded"
            if (result.get("dry_run") or result.get("accepted") is not True
                    or not str(result.get("sdk_result", "")).endswith("SUCCESS")):
                raise RuntimeError("SDK did not accept the Cartesian offset")

            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                reading = call("/api/commands/motion/end_effector_pose", {"arm": "right_arm"})
                actual = reading["pose"]
                record["last_polled_tcp"] = actual
                if (abs(actual[0] - planned_x) <= 0.005
                        and abs(actual[1] - initial_pose[1]) <= 0.005
                        and abs(actual[2] - initial_pose[2]) <= 0.005
                        and angle_between(actual[3:], initial_pose[3:]) <= math.radians(2)):
                    break
                time.sleep(0.35)
            else:
                raise RuntimeError("End effector did not reach the Cartesian step target")
            record["target_reached"] = True
        except Exception as error:
            record["error"] = type(error).__name__ + ": " + str(error)
            record["target_reached"] = False
            stop_arm()
        finally:
            record["command_finished_ns"] = time.time_ns()
            write_json(record_path, record)

        after = sdk_snapshot(code)
        after_path = folder / f"{prefix}_{args.step:02d}_after.json"
        write_json(after_path, after)
        db.ingest_state(after_path, SESSION)
        record["after_state_path"] = db.relative(after_path)
        record["after_errors"] = after["errors"]
        if not after["errors"]:
            record["after_tcp"] = pose(after)
            record["after_gripper"] = gripper(after)
            record["after_base_shift_m"] = math.hypot(
                nav(after)["x"] - initial_nav["x"], nav(after)["y"] - initial_nav["y"])
            if (abs(record["after_tcp"][0] - planned_x) > 0.008
                    or abs(record["after_tcp"][1] - initial_pose[1]) > 0.005
                    or abs(record["after_tcp"][2] - initial_pose[2]) > 0.005
                    or angle_between(record["after_tcp"][3:], initial_pose[3:]) > math.radians(2)
                    or record["after_base_shift_m"] > 0.03
                    or not 0.020 <= record["after_gripper"]["width"] <= 0.030
                    or record["after_gripper"]["effort"] < 4):
                record["target_reached"] = False
                record["error"] = "Post-step TCP, base or gripper feedback outside guard"
                stop_arm()
        else:
            record["target_reached"] = False
            record["error"] = "Post-step state unavailable"
            stop_arm()
        write_json(record_path, record)
        print(json.dumps(record, ensure_ascii=False, indent=2), flush=True)
        if not record["target_reached"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
