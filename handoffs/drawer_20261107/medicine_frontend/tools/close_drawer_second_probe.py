"""One-shot 15 mm drawer-close probe after the first 5 mm request did not move.

The arm offset API has no hard force cap. This tool uses a 15 mm pose target,
monitors raw right-wrist force at 20 Hz, and stops on a sustained 5 N change,
any 8 N change, telemetry loss, pose error, or timeout. It does not move the
base or change the gripper. The previous failed command is never resubmitted.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from urllib import request

from medicine_frontend.dataset import Dataset
from medicine_frontend.tools.capture_final_drawer import CONSOLE, sdk_snapshot
from medicine_frontend.tools.probe_drawer_force import ssh_prefix
from medicine_frontend.tools.pull_drawer_arm_step import angle_between, gripper, nav, pose
from medicine_frontend.tools.record_drawer_stream import login


SESSION = "drawer_close_20261107_130612_probe2"
PULL = "drawer_pull_20261107_122158"
PREVIOUS = "drawer_close_20261107_125552"
FORCE_DELTA_LIMIT_N = 5.0
FORCE_IMMEDIATE_LIMIT_N = 8.0
PROGRESS_M = 0.015

REMOTE_FORCE_SOURCE = '''\
import json
import time
from galbot_sdk.g1 import GalbotRobot, GalbotOneFoxtrotSensor

robot = GalbotRobot()
if not robot.init():
    raise RuntimeError("Read-only force client initialization failed")
try:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        data = robot.get_force_sensor_data(
            GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE, False)
        print(json.dumps({"received_ns": time.time_ns(), "data": data}), flush=True)
        time.sleep(0.05)
finally:
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
'''


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def call(cookie, path, payload):
    req = request.Request(CONSOLE + path, data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json", "Cookie": cookie})
    with request.urlopen(req, timeout=5) as response:
        return json.load(response)


class ForceGuard:
    def __init__(self, folder, cookie, *, path=None, sustained_n=None, immediate_n=None,
                 frames=None, ready_timeout_s=None):
        self.path = folder / "close_step_02_force.ndjson" if path is None else path
        # SSH + SDK init on a loaded robot can take 8 s; 12 s leaves no margin there.
        self.ready_timeout_s = 12.0 if ready_timeout_s is None else float(ready_timeout_s)
        self.sustained_n = FORCE_DELTA_LIMIT_N if sustained_n is None else float(sustained_n)
        self.immediate_n = (FORCE_IMMEDIATE_LIMIT_N if immediate_n is None
                            else float(immediate_n))
        self.frames = 2 if frames is None else int(frames)
        self.cookie = cookie
        self.ready = threading.Event()
        self.trip = threading.Event()
        self.motion_started = threading.Event()
        self.stop_lock = threading.Lock()
        self.last_arrival = 0.0
        self.samples = 0
        self.baseline_vectors = []
        self.baseline = None
        self.max_delta_n = 0.0
        self.high_delta_samples = 0
        self.reason = None
        self.stop_result = None
        self.stop_error = None
        self.process = None
        self.thread = None

    def stop_arm(self):
        with self.stop_lock:
            if self.stop_result is not None or self.stop_error is not None:
                return
            try:
                self.stop_result = call(self.cookie,
                                        "/api/commands/motion/stop_controller",
                                        {"group": "right_arm"})
                if not str(self.stop_result.get("sdk_result", "")).endswith("SUCCESS"):
                    raise RuntimeError("Right arm stop was not confirmed")
            except Exception as error:
                self.stop_error = type(error).__name__ + ": " + str(error)
                try:
                    self.stop_result = call(self.cookie,
                                            "/api/commands/robot/emergency_stop", {})
                except Exception as emergency_error:
                    self.stop_error += "; emergency stop: " + repr(emergency_error)

    def mark_trip(self, reason):
        if not self.trip.is_set():
            self.reason = reason
            self.trip.set()
            if self.motion_started.is_set():
                self.stop_arm()

    def start(self):
        remote = ("cd /home/galbot/galbot-sdk-web-console && "
                  "PYTHONPATH=/data/galbot/lib PYTHONDONTWRITEBYTECODE=1 "
                  f"timeout {int(self.ready_timeout_s) + 10}s .venv/bin/python -u -")
        self.process = subprocess.Popen(
            [*ssh_prefix(), remote], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        self.process.stdin.write(REMOTE_FORCE_SOURCE)
        self.process.stdin.close()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()
        if not self.ready.wait(self.ready_timeout_s):
            self.mark_trip("Right wrist force baseline did not arrive")
            raise RuntimeError(self.reason)
        if self.trip.is_set() or time.monotonic() - self.last_arrival > 0.3:
            raise RuntimeError(self.reason or "Right wrist force stream stale")

    def _read(self):
        with self.path.open("x", encoding="utf-8") as output:
            for line in self.process.stdout:
                try:
                    item = json.loads(line)
                    data = item["data"]
                    force = data["force"]
                    vector = [float(force[axis]) for axis in ("x", "y", "z")]
                    if (not all(math.isfinite(x) for x in vector)
                            or not isinstance(data.get("timestamp_ns"), int)
                            or not 0 <= item["received_ns"] - data["timestamp_ns"]
                            <= 100_000_000):
                        raise ValueError("Invalid force sample")
                except (ValueError, KeyError, TypeError):
                    continue
                output.write(json.dumps(item, ensure_ascii=False) + "\n")
                output.flush()
                self.samples += 1
                self.last_arrival = time.monotonic()
                if self.baseline is None:
                    self.baseline_vectors.append(vector)
                    if len(self.baseline_vectors) >= 20:
                        self.baseline = [sum(row[i] for row in self.baseline_vectors) /
                                         len(self.baseline_vectors) for i in range(3)]
                        noise = max(math.dist(row, self.baseline)
                                    for row in self.baseline_vectors)
                        if noise > 1.5:
                            self.mark_trip("Unstable wrist force baseline")
                        self.ready.set()
                else:
                    delta = math.dist(vector, self.baseline)
                    self.max_delta_n = max(self.max_delta_n, delta)
                    if delta > self.immediate_n:
                        self.mark_trip(f"Raw wrist force changed by {delta:.2f} N")
                    elif delta > self.sustained_n:
                        self.high_delta_samples += 1
                        if self.high_delta_samples >= self.frames:
                            self.mark_trip(f"Raw wrist force stayed above "
                                           f"{self.sustained_n:.1f} N change: {delta:.2f} N")
                    else:
                        self.high_delta_samples = 0
            if not self.trip.is_set() and self.motion_started.is_set():
                self.mark_trip("Right wrist force stream ended during motion")

    def check(self):
        if self.trip.is_set():
            raise RuntimeError(self.reason)
        if time.monotonic() - self.last_arrival > 0.3:
            self.mark_trip("Right wrist force stream stale during motion")
            raise RuntimeError(self.reason)

    def close(self):
        if self.process is None:
            return
        try:
            self.process.wait(timeout=self.ready_timeout_s + 11)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=5)
        if self.thread is not None:
            self.thread.join(timeout=3)


def main():
    code = os.environ["GALBOT_CONSOLE_CODE"]
    with Dataset() as db:
        folder = db.root / "teaching" / SESSION
        previous_attempt = json.loads((db.root / "teaching" / PREVIOUS /
                                       "close_step_01.json").read_text(encoding="utf-8"))
        if (previous_attempt.get("target_reached") is not False
                or math.dist(previous_attempt["before_tcp"][:3],
                             previous_attempt["after_tcp"][:3]) > 0.002):
            raise RuntimeError("Previous probe did not end at its original pose")
        start = json.loads((folder / "initial_state.json").read_text(encoding="utf-8"))
        pull_final = json.loads((db.root / "teaching" / PULL / "final_state.json")
                                .read_text(encoding="utf-8"))
        prior = json.loads((db.root / "teaching" / PULL /
                            "arm_extend_step_01_after.json").read_text(encoding="utf-8"))
        journal_path = folder / "close_step_02.json"
        if journal_path.exists():
            raise RuntimeError("Second close-step journal exists; refusing duplicate motion")
        before = sdk_snapshot(code)
        before_path = folder / "close_step_02_before.json"
        write_json(before_path, before)
        db.ingest_state(before_path, SESSION)
        if before["errors"]:
            raise RuntimeError("Fresh SDK state incomplete; no motion sent")
        current = pose(before)
        g = gripper(before)
        base = nav(before)
        if (math.dist(current[:3], pose(start)[:3]) > 0.01
                or math.hypot(base["x"] - nav(start)["x"],
                              base["y"] - nav(start)["y"]) > 0.03
                or angle_between(current[3:], pose(start)[3:]) > math.radians(2)
                or not 0.020 <= g["width"] <= 0.030 or g["effort"] < 4
                or g["is_moving"]):
            raise RuntimeError("Current drawer grip/base/TCP differs from close start; no motion sent")

        recorded_final = pose(pull_final)
        recorded_prior = pose(prior)
        fraction = PROGRESS_M / (recorded_prior[0] - recorded_final[0])
        target = [current[0] + PROGRESS_M,
                  current[1] + fraction * (recorded_prior[1] - recorded_final[1]),
                  current[2] + fraction * (recorded_prior[2] - recorded_final[2])]
        command = {"arm": "right_arm", "dx": target[0] - current[0],
                   "dy": target[1] - current[1], "dz": target[2] - current[2],
                   "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "confirmed": True}
        if not 0 < command["dx"] <= 0.0151 or max(abs(command[k]) for k in ("dy", "dz")) > 0.004:
            raise RuntimeError("Unexpected second-step offset; no motion sent")

        cookie = login(code)
        guard = ForceGuard(folder, cookie)
        record = {"schema": "drawer_arm_close_step/1", "session": SESSION,
                  "recorded_ns": time.time_ns(),
                  "step": 2, "target_cumulative_m": PROGRESS_M,
                  "previous_failed_attempt_session": PREVIOUS,
                  "before_state_path": db.relative(before_path),
                  "before_tcp": current, "before_gripper": g, "before_base_map": base,
                  "target_tcp_position_base_link_m": target,
                  "command": command,
                  "raw_force_sustained_delta_limit_N": FORCE_DELTA_LIMIT_N,
                  "raw_force_immediate_delta_limit_N": FORCE_IMMEDIATE_LIMIT_N,
                  "sustained_sample_count": 2,
                  "force_is_calibrated": False, "base_commands_sent": [],
                  "gripper_commands_sent": [], "submission": "not_started"}
        try:
            guard.start()
            record["force_baseline_sensor_frame_N"] = guard.baseline
            record["started_ns"] = time.time_ns()
            record["submission"] = "started"
            write_json(journal_path, record)
            guard.motion_started.set()
            result = call(cookie, "/api/commands/motion/move_end_effector_offset", command)
            record["command_result"] = result
            record["submission"] = "responded"
            write_json(journal_path, record)
            if result.get("dry_run") or result.get("accepted") is not True \
                    or not str(result.get("sdk_result", "")).endswith("SUCCESS"):
                raise RuntimeError("Right-arm offset not accepted")

            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                guard.check()
                value = call(cookie, "/api/commands/motion/end_effector_pose",
                             {"arm": "right_arm"})
                actual = value["pose"]
                record["last_polled_tcp"] = actual
                if (abs(actual[0] - target[0]) <= 0.0015
                        and abs(actual[1] - target[1]) <= 0.002
                        and abs(actual[2] - target[2]) <= 0.002):
                    time.sleep(0.5)
                    guard.check()
                    record["target_reached"] = True
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("15 mm Cartesian target not reached within 6 seconds")
        except Exception as error:
            record["error"] = type(error).__name__ + ": " + str(error)
            record["target_reached"] = False
            if record["submission"] != "not_started":
                guard.stop_arm()
        finally:
            guard.close()
            guard.motion_started.clear()
            if guard.trip.is_set():
                record["target_reached"] = False
                record.setdefault("error", guard.reason)
            record["finished_ns"] = time.time_ns()
            record["force_samples"] = guard.samples
            record["max_raw_force_delta_N"] = guard.max_delta_n
            record["force_trip_reason"] = guard.reason
            record["stop_result"] = guard.stop_result
            record["stop_error"] = guard.stop_error
            if guard.path.exists():
                record["force_stream_file"] = db.relative(guard.path)
                record["force_stream_sha256"] = hashlib.sha256(
                    guard.path.read_bytes()).hexdigest()
            write_json(journal_path, record)

        after = sdk_snapshot(code)
        after_path = folder / "close_step_02_after.json"
        write_json(after_path, after)
        db.ingest_state(after_path, SESSION)
        record["after_state_path"] = db.relative(after_path)
        record["after_errors"] = after["errors"]
        if not after["errors"]:
            record["after_tcp"] = pose(after)
            record["after_gripper"] = gripper(after)
            record["after_base_map"] = nav(after)
            if (abs(record["after_tcp"][0] - target[0]) > 0.003
                    or abs(record["after_tcp"][1] - target[1]) > 0.003
                    or abs(record["after_tcp"][2] - target[2]) > 0.003
                    or angle_between(record["after_tcp"][3:], current[3:])
                    > math.radians(2)
                    or math.hypot(nav(after)["x"] - base["x"],
                                  nav(after)["y"] - base["y"]) > 0.03
                    or not 0.020 <= record["after_gripper"]["width"] <= 0.030
                    or record["after_gripper"]["effort"] < 4
                    or record["after_gripper"]["is_moving"]):
                record["target_reached"] = False
                record["postcheck_error"] = "TCP, base, or gripper changed outside guard"
                guard.stop_arm()
        else:
            record["target_reached"] = False
            record["postcheck_error"] = "Fresh post-step state unavailable"
            guard.stop_arm()
        record["stop_result"] = guard.stop_result
        record["stop_error"] = guard.stop_error
        write_json(journal_path, record)
        db.ingest_teaching_event(journal_path, SESSION)
        print(json.dumps({"step": 2, "target_reached": record["target_reached"],
                          "after_tcp": record.get("after_tcp"),
                          "max_raw_force_delta_N": guard.max_delta_n,
                          "force_trip_reason": guard.reason,
                          "error": record.get("error"),
                          "postcheck_error": record.get("postcheck_error"),
                          "journal": str(journal_path)}, ensure_ascii=False, indent=2))
        return 0 if record["target_reached"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
