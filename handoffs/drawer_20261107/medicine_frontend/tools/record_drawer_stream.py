"""Read-only continuous SDK telemetry for the next drawer demonstration.

Start before anyone moves the robot and stop after the action. The access code
is read from GALBOT_CONSOLE_CODE and is never written to the dataset. This
recorder sends no robot command; its only POST calls are login and the
end-effector-pose getter. Camera captures should run in parallel and retain
their own timestamps.
"""

import argparse
import base64
from datetime import datetime, timedelta, timezone
import hashlib
import http.cookiejar
import json
import os
from pathlib import Path
import select
import socket
import struct
import threading
import time
from urllib import request

from medicine_frontend.dataset import Dataset
from medicine_frontend.tools.capture_final_drawer import CONSOLE


HOST = "192.168.166.7"
PORT = 8080
EVENT_TYPES = {"navigation_state", "joint_state", "robot_sensors"}


def json_line(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n"


def read_exact(sock, buffer, count, deadline):
    while len(buffer) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Telemetry stream ended")
        ready, _, _ = select.select([sock], [], [], min(remaining, 2.0))
        if not ready:
            continue
        chunk = sock.recv(max(4096, count - len(buffer)))
        if not chunk:
            raise ConnectionError("Telemetry socket closed")
        buffer.extend(chunk)
    result = bytes(buffer[:count])
    del buffer[:count]
    return result


def connect_events(cookie, deadline):
    sock = socket.create_connection((HOST, PORT), timeout=8)
    sock.settimeout(8)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    handshake = (f"GET /api/events HTTP/1.1\r\nHost: {HOST}:{PORT}\r\n"
                 "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                 f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                 f"Cookie: {cookie}\r\n\r\n").encode("ascii")
    sock.sendall(handshake)
    buffer = bytearray()
    while b"\r\n\r\n" not in buffer:
        if time.monotonic() >= deadline:
            sock.close()
            raise TimeoutError("WebSocket handshake timed out")
        chunk = sock.recv(4096)
        if not chunk:
            sock.close()
            raise ConnectionError("WebSocket closed during handshake")
        buffer.extend(chunk)
        if len(buffer) > 16384:
            sock.close()
            raise RuntimeError("WebSocket handshake too large")
    split = buffer.index(b"\r\n\r\n") + 4
    header = bytes(buffer[:split])
    del buffer[:split]
    if b" 101 " not in header.split(b"\r\n", 1)[0]:
        sock.close()
        raise RuntimeError("WebSocket authentication or upgrade failed")
    return sock, buffer


def read_event_frame(sock, buffer, deadline):
    first, second = read_exact(sock, buffer, 2, deadline)
    opcode = first & 15
    size = second & 127
    if size == 126:
        size = struct.unpack("!H", read_exact(sock, buffer, 2, deadline))[0]
    elif size == 127:
        size = struct.unpack("!Q", read_exact(sock, buffer, 8, deadline))[0]
    if size > 4 * 1024 * 1024:
        raise ValueError("Telemetry frame unexpectedly large")
    mask = read_exact(sock, buffer, 4, deadline) if second & 128 else None
    payload = read_exact(sock, buffer, size, deadline)
    if mask:
        payload = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    return opcode, payload


def send_pong(sock, payload):
    mask = os.urandom(4)
    length = len(payload)
    if length > 125:
        return
    frame = bytes((0x8A, 0x80 | length)) + mask
    frame += bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    sock.sendall(frame)


def login(code):
    jar = http.cookiejar.CookieJar()
    opener = request.build_opener(request.HTTPCookieProcessor(jar))
    req = request.Request(CONSOLE + "/api/auth/login",
                          data=json.dumps({"code": code}).encode(),
                          headers={"Content-Type": "application/json"})
    with opener.open(req, timeout=10) as response:
        value = json.load(response)
    if value.get("authenticated") is not True:
        raise RuntimeError("SDK console login failed")
    return "; ".join(f"{item.name}={item.value}" for item in jar)


def poll_tcp(cookie, output, deadline, stop, rate_hz, counters, errors):
    interval = 1.0 / rate_hz
    next_poll = time.monotonic()
    while not stop.is_set() and time.monotonic() < deadline:
        wait = next_poll - time.monotonic()
        if wait > 0:
            stop.wait(min(wait, 0.2))
            continue
        next_poll += interval
        if next_poll < time.monotonic():
            next_poll = time.monotonic() + interval
        sent = time.time_ns()
        try:
            req = request.Request(
                CONSOLE + "/api/commands/motion/end_effector_pose",
                data=b'{"arm":"right_arm"}',
                headers={"Content-Type": "application/json", "Cookie": cookie})
            with request.urlopen(req, timeout=3) as response:
                payload = json.load(response)
            if not payload.get("available") or len(payload.get("pose", [])) != 7:
                raise ValueError("Right TCP pose unavailable")
            output.write(json_line({"requested_ns": sent, "received_ns": time.time_ns(),
                                    "frame_id": payload.get("frame_id"),
                                    "pose": payload["pose"]}))
            output.flush()
            counters["tcp"] += 1
        except Exception as error:
            errors.append({"source": "tcp", "at_ns": sent,
                           "error": type(error).__name__ + ": " + str(error)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--duration-s", type=float, default=600.0)
    parser.add_argument("--tcp-hz", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not args.session.startswith("drawer_") or not all(
            char.isalnum() or char in "_-" for char in args.session):
        parser.error("session must be a drawer_* name with letters, numbers, _ or -")
    if not 1.0 <= args.duration_s <= 900.0 or not 1.0 <= args.tcp_hz <= 10.0:
        parser.error("duration-s must be 1..900, tcp-hz must be 1..10")
    code = os.environ["GALBOT_CONSOLE_CODE"]
    cookie = login(code)
    with Dataset() as db:
        folder = db.root / "teaching" / args.session / "continuous"
        if folder.exists():
            raise FileExistsError("Continuous recording exists; use a new session")
        folder.mkdir(parents=True)
        event_path = folder / "events.ndjson"
        tcp_path = folder / "tcp.ndjson"
        counters = {"navigation_state": 0, "joint_state": 0,
                    "robot_sensors": 0, "tcp": 0}
        errors = []
        started_ns = time.time_ns()
        deadline = time.monotonic() + args.duration_s
        sock = None
        stop = threading.Event()
        worker = None
        interrupted = False
        try:
            with event_path.open("x", encoding="utf-8") as events_out, \
                    tcp_path.open("x", encoding="utf-8") as tcp_out:
                try:
                    sock, buffer = connect_events(cookie, deadline)
                    worker = threading.Thread(target=poll_tcp,
                                              args=(cookie, tcp_out, deadline, stop,
                                                    args.tcp_hz, counters, errors), daemon=True)
                    worker.start()
                    while time.monotonic() < deadline:
                        try:
                            opcode, payload = read_event_frame(sock, buffer, deadline)
                        except TimeoutError:
                            break
                        if opcode == 8:
                            errors.append({"source": "events", "error": "WebSocket closed"})
                            break
                        if opcode == 9:
                            send_pong(sock, payload)
                            continue
                        if opcode != 1:
                            continue
                        event = json.loads(payload)
                        kind = event.get("type")
                        if kind in EVENT_TYPES:
                            events_out.write(json_line({"received_ns": time.time_ns(),
                                                        "event": event}))
                            events_out.flush()
                            counters[kind] += 1
                finally:
                    stop.set()
                    if worker is not None:
                        worker.join()
                    events_out.flush()
        except KeyboardInterrupt:
            interrupted = True
        except Exception as error:
            errors.append({"source": "events", "error": type(error).__name__ + ": " + str(error)})
        finally:
            if sock is not None:
                sock.close()
        finished_ns = time.time_ns()
        manifest = {"schema": "drawer_telemetry_stream/1", "recorded_ns": started_ns,
                    "finished_ns": finished_ns, "session": args.session,
                    "source": CONSOLE, "motion_commands_sent": [],
                    "stopped_by_operator": interrupted,
                    "requested_duration_s": args.duration_s, "requested_tcp_hz": args.tcp_hz,
                    "counts": counters, "errors": errors,
                    "streams": [
                        {"file": db.relative(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                         "bytes": path.stat().st_size}
                        for path in (event_path, tcp_path)]}
        manifest_path = folder / "recording.json"
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
        db.ingest_teaching_event(manifest_path, args.session)
        print(json.dumps({"manifest": str(manifest_path), "counts": counters,
                          "errors": errors}, ensure_ascii=False, indent=2))
        return 0 if all(counters[k] > 0 for k in counters) and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
