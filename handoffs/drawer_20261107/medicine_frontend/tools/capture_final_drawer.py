"""One read-only final drawer teaching capture from existing HTTP services.

The SDK console motion getter is read-only but may initialize GalbotMotion.
Run only with the operator's authorization for that initialization.
"""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import base64
import hashlib
import http.cookiejar
import json
import os
from pathlib import Path
import socket
import struct
import time
from urllib import request

from medicine_frontend.dataset import Dataset
from medicine_frontend.tools import collect

CONSOLE = "http://192.168.166.7:8080"


def socket_events(cookie, seconds=7):
    """Receive existing event stream with standard library; no control messages."""
    key = base64.b64encode(os.urandom(16)).decode()
    connection = socket.create_connection(("192.168.166.7", 8080), timeout=8)
    connection.settimeout(8)
    handshake = ("GET /api/events HTTP/1.1\r\nHost: 192.168.166.7:8080\r\n"
                 "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                 f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                 f"Cookie: {cookie}\r\n\r\n").encode()
    connection.sendall(handshake)
    stream = connection.makefile("rb")
    try:
        header = stream.readline()
        if b" 101 " not in header:
            raise RuntimeError("WebSocket upgrade rejected: " + header.decode(errors="replace").strip())
        while stream.readline().strip():
            pass
        result = []
        deadline = time.monotonic() + seconds
        needed = {"navigation_state", "joint_state", "robot_sensors"}
        while time.monotonic() < deadline:
            two = stream.read(2)
            if len(two) != 2:
                break
            kind = two[0] & 15
            size = two[1] & 127
            if size == 126:
                size = struct.unpack("!H", stream.read(2))[0]
            elif size == 127:
                size = struct.unpack("!Q", stream.read(8))[0]
            if size > 4 * 1024 * 1024:
                raise ValueError("Unexpectedly large telemetry frame")
            mask = stream.read(4) if two[1] & 128 else None
            payload = stream.read(size)
            if len(payload) != size:
                raise RuntimeError("Truncated telemetry frame")
            if mask:
                payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
            if kind == 8:
                break
            if kind != 1:
                continue
            event = json.loads(payload)
            if event.get("type") in needed:
                result.append({"received_ns": time.time_ns(), "event": event})
                if needed.issubset({item["event"]["type"] for item in result}) and len(result) >= 9:
                    break
        return result
    finally:
        stream.close()
        connection.close()


def sdk_snapshot(code):
    jar = http.cookiejar.CookieJar()
    opener = request.build_opener(request.HTTPCookieProcessor(jar))
    def read(path, data=None):
        payload = None if data is None else json.dumps(data).encode()
        req = request.Request(CONSOLE + path, data=payload,
                              headers={"Content-Type": "application/json"})
        with opener.open(req, timeout=15) as response:
            return json.load(response)
    if read("/api/auth/login", {"code": code}).get("authenticated") is not True:
        raise RuntimeError("Console login failed")
    result = {"schema": "sdk_console_snapshot/1", "started_ns": time.time_ns(),
              "source": CONSOLE, "motion_commands_sent": [], "reads": {}, "events": [], "errors": []}
    for key, path, payload in (
        ("navigation", "/api/robot/state", None),
        ("robot", "/api/commands/robot/snapshot", {"include_lidar": False}),
        ("right_end_effector", "/api/commands/motion/end_effector_pose", {"arm": "right_arm"}),
    ):
        started = time.time_ns()
        try:
            result["reads"][key] = {"requested_ns": started, "data": read(path, payload),
                                     "received_ns": time.time_ns()}
        except Exception as error:
            result["errors"].append({"field": key, "error": type(error).__name__ + ": " + str(error)})
    cookie = "; ".join(f"{item.name}={item.value}" for item in jar)
    try:
        result["events"] = socket_events(cookie)
    except Exception as error:
        result["errors"].append({"field": "event_stream", "error": type(error).__name__ + ": " + str(error)})
    result["finished_ns"] = time.time_ns()
    return result


def main():
    camera_code = os.environ["GALBOT_CAM_CODE"]
    console_code = os.environ["GALBOT_CONSOLE_CODE"]
    session = "drawer_final_" + datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    print("FINAL_SESSION=" + session, flush=True)
    def camera(kind):
        args = [kind, "--sources", kind + "_right", kind + "_left", "--drug", "none",
                "--arrangement", "drawer_final", "--session", session, "--count", "1",
                "--grasp-state", "operator_confirmed_alignment_gripper_open",
                "--note", "Final canonical drawer alignment capture; right gripper open. No robot motion commanded.",
                "--code", camera_code, "--yes"]
        return collect.main(args)
    with ThreadPoolExecutor(max_workers=3) as pool:
        pending = {name: pool.submit(fn) for name, fn in (
            ("wrist", lambda: camera("wrist")), ("head", lambda: camera("head")),
            ("sdk", lambda: sdk_snapshot(console_code)))}
        results = {name: future.result() for name, future in pending.items()}
    if results["wrist"] != 0 or results["head"] != 0:
        raise RuntimeError("One of the camera groups failed")
    with Dataset() as db:
        records = [row for row in db.records() if row["session"] == session]
        if {row["source"] for row in records} != collect.SOURCES:
            raise RuntimeError("Four camera sources were not saved")
        if sum(row["depth_path"] is not None for row in records) != 2:
            raise RuntimeError("Two wrist depth files were not saved")
        folder = db.root / "teaching" / session
        folder.mkdir(parents=True)
        state = results["sdk"]
        state_path = folder / "sdk_snapshot.json"
        with state_path.open("x", encoding="utf-8") as output:
            json.dump(state, output, ensure_ascii=False, indent=2)
        snapshot_id = db.ingest_state(state_path, session)
        manifest = {"schema": "drawer_teaching_record/2", "session": session,
                    "status": "final_canonical_capture", "operator_confirmed_alignment": True,
                    "operator_reported_gripper": "open", "grasp_verified": False,
                    "replay_ready": False, "motion_commands_sent": [],
                    "capture_ids": sorted({row["capture_id"] for row in records}),
                    "state_snapshot_id": snapshot_id,
                    "state_snapshot_path": db.relative(state_path),
                    "frames": [{"source": row["source"], "id": row["id"],
                                "image_path": row["image_path"], "depth_path": row["depth_path"],
                                "quality_flags": row["quality_flags"]} for row in records]}
        with (folder / "teaching_record.json").open("x", encoding="utf-8") as output:
            json.dump(manifest, output, ensure_ascii=False, indent=2)
        print(json.dumps({"session": session, "manifest": str((folder / "teaching_record.json").resolve()),
                          "snapshot": str(state_path.resolve()), "camera_frames": len(records),
                          "event_types": sorted({event["event"]["type"] for event in state["events"]}),
                          "reads": list(state["reads"]), "errors": state["errors"]},
                         ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
