"""Read only the right wrist force sensor for a stationary drawer baseline.

The SDK code runs over existing SSH through stdin, with no robot-side file or
configuration write and no motion command. Local samples are immutable.
"""

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess

from medicine_frontend.dataset import Dataset


REMOTE_SOURCE = '''\
import json
import sys
import time
from galbot_sdk.g1 import GalbotRobot, GalbotOneFoxtrotSensor

robot = GalbotRobot()
if not robot.init():
    raise RuntimeError("Read-only force client initialization failed")
try:
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        value = robot.get_force_sensor_data(
            GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE, False)
        print(json.dumps({"received_ns": time.time_ns(), "data": value}), flush=True)
        time.sleep(0.05)
finally:
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
'''


def ssh_prefix():
    """Use an operator-configured SSH alias/key; never read passwords from files."""
    import os
    target = os.environ.get("GALBOT_SSH_TARGET")
    if not target or target.startswith("-"):
        raise RuntimeError("Set GALBOT_SSH_TARGET to an authorized SSH alias/user@host")
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", target]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    args = parser.parse_args(argv)
    if not args.session.startswith("drawer_close_"):
        parser.error("session must identify a drawer_close_* capture")

    remote = ("cd /home/galbot/galbot-sdk-web-console && "
              "PYTHONPATH=/data/galbot/lib PYTHONDONTWRITEBYTECODE=1 "
              "timeout 15s .venv/bin/python -u -")
    result = subprocess.run([*ssh_prefix(), remote], input=REMOTE_SOURCE,
                            capture_output=True, text=True, timeout=25)
    samples = []
    for line in result.stdout.splitlines():
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        data = entry.get("data")
        force = data.get("force") if isinstance(data, dict) else None
        if isinstance(force, dict) and all(isinstance(force.get(k), (int, float))
                                           and math.isfinite(force[k]) for k in ("x", "y", "z")):
            samples.append(entry)

    with Dataset() as db:
        folder = db.root / "teaching" / args.session
        if not (folder / "initial_record.json").is_file():
            raise ValueError("Capture the drawer-close initial state first")
        stream_path = folder / "force_baseline.ndjson"
        with stream_path.open("x", encoding="utf-8") as output:
            for entry in samples:
                output.write(json.dumps(entry, ensure_ascii=False) + "\n")
        means = {axis: statistics.mean(s["data"]["force"][axis] for s in samples)
                 for axis in ("x", "y", "z")} if samples else {}
        deviations = [math.dist([s["data"]["force"][axis] for axis in ("x", "y", "z")],
                                [means[axis] for axis in ("x", "y", "z")]) for s in samples]
        timestamps = [s["data"].get("timestamp_ns") for s in samples]
        valid = (result.returncode == 0 and len(samples) >= 25
                 and all(isinstance(t, int) for t in timestamps)
                 and all(b > a for a, b in zip(timestamps, timestamps[1:])))
        manifest = {
            "schema": "drawer_force_baseline/1",
            "recorded_ns": samples[0]["received_ns"] if samples else
            int(datetime.now(timezone.utc).timestamp() * 1e9),
            "session": args.session, "sensor": "RIGHT_WRIST_FORCE",
            "calibrated": False, "robot_motion_commands_sent": [],
            "sample_count": len(samples), "valid_for_reactive_monitor": valid,
            "mean_force_sensor_frame_N": means,
            "max_deviation_from_mean_N": max(deviations, default=None),
            "sample_file": db.relative(stream_path),
            "sample_sha256": hashlib.sha256(stream_path.read_bytes()).hexdigest(),
            "read_error": result.stderr[-500:] if result.returncode else None,
        }
        manifest_path = folder / "force_baseline.json"
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
        db.ingest_teaching_event(manifest_path, args.session)
        print(json.dumps({"session": args.session, "sample_count": len(samples),
                          "valid_for_reactive_monitor": valid,
                          "mean_force_sensor_frame_N": means,
                          "max_deviation_from_mean_N": max(deviations, default=None),
                          "manifest": str(manifest_path)}, ensure_ascii=False, indent=2))
        return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
