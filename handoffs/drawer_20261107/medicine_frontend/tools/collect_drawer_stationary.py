"""Stationary multi-position drawer perception capture. Sends NO motion commands.

Step 1 of the improved drawer-pull flow: park at each candidate position, keep
the robot still, and record right-wrist RGB + depth, intrinsics, extrinsics
and joint state. The operator moves the robot between captures by whatever
means the site allows; this process never sends arm, chassis or gripper
commands. Run on the robot host:

    python -X utf8 -m medicine_frontend.tools.collect_drawer_stationary \
        --session drawer_stationary_YYYYMMDD_HHMMSS --captures 5

Output: data/drawer_stationary/<session>/capture_XX/{rgb.jpg,depth.bin,capture.json}
plus manifest.json with hashes. Offline analysis loads these files and calls
medicine_frontend.drawer_stationary.analyze_frame / summarize.

Known gaps, recorded per capture instead of guessed:
- base pose in map frame: no confirmed SDK 1.10 Python reader; if the site
  console provides it, note it externally per capture index.
- RGB/depth pixel alignment and depth scale: verified later by replaying these
  files, never assumed from nominal specs.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import time

ROOT = Path(__file__).resolve().parents[2] / "data" / "drawer_stationary"


def _default_sensors():
    from galbot_sdk.g1 import RgbOutputFormat, SensorType
    return (SensorType.RIGHT_ARM_CAMERA, SensorType.RIGHT_ARM_DEPTH_CAMERA,
            RgbOutputFormat.JPEG)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _best_effort(description, fn):
    try:
        return {"data": fn(), "error": None}
    except Exception as error:  # read-only attempts must never abort a capture
        return {"data": None,
                "error": f"{description}: {type(error).__name__}: {error}"[:300]}


def capture_once(robot, folder, *, sensors=None, settle_s=0.5):
    """Save one stationary capture. Raises on any camera failure; no retries."""
    rgb_sensor, depth_sensor, rgb_format = sensors or _default_sensors()
    folder = Path(folder)
    if folder.exists():
        raise FileExistsError(f"Capture folder already exists: {folder}")
    if not 0 <= settle_s <= 5:
        raise ValueError("settle_s out of range")
    time.sleep(settle_s)  # let the arm stop vibrating before sampling

    captured_ns = time.time_ns()
    captured_monotonic = time.monotonic()
    rgb = robot.get_rgb_data(rgb_sensor, rgb_format, True)
    raw_rgb = bytes((rgb or {}).get("data") or b"")
    if "JPEG" in str(rgb_format) and not raw_rgb.startswith(b"\xff\xd8"):
        raise RuntimeError("Right wrist RGB is not a JPEG frame")
    if not raw_rgb:
        raise RuntimeError("Empty right wrist RGB frame")
    depth = robot.get_depth_data(depth_sensor)
    if not isinstance(depth, dict) or not depth.get("data"):
        raise RuntimeError("No right wrist depth frame")
    raw_depth = bytes(depth["data"])
    scale = float(depth.get("depth_scale", 0))
    if not scale > 0:
        raise RuntimeError("Invalid depth scale from SDK")

    intrinsic_rgb = robot.get_camera_intrinsic(rgb_sensor)
    intrinsic_depth = robot.get_camera_intrinsic(depth_sensor)
    extrinsic_rgb, extrinsic_rgb_ts = robot.get_sensor_extrinsic(rgb_sensor, "base_link")
    extrinsic_depth, extrinsic_depth_ts = robot.get_sensor_extrinsic(depth_sensor,
                                                                     "base_link")
    joints = _best_effort(
        "joint_state",
        lambda: robot.get_synced_observation([rgb_sensor], with_joint_state=True))
    base_pose = {"data": None,
                 "error": "base_pose_api_unconfirmed:record the console map pose "
                          "externally for this capture index"}

    folder.mkdir(parents=True)
    rgb_path = folder / "rgb.jpg"
    depth_path = folder / "depth.bin"
    with rgb_path.open("xb") as stream:
        stream.write(raw_rgb)
    with depth_path.open("xb") as stream:
        stream.write(raw_depth)
    metadata = {
        "schema": "drawer_stationary_capture/1",
        "captured_ns": captured_ns,
        "captured_monotonic_s": captured_monotonic,
        "settle_s": settle_s,
        "motion_commands_sent": [],
        "sensors": {"rgb": str(rgb_sensor), "depth": str(depth_sensor),
                    "rgb_format": str(rgb_format)},
        "rgb": {**{k: v for k, v in rgb.items() if k != "data"},
                "file": rgb_path.name, "bytes": len(raw_rgb),
                "sha256": sha256_bytes(raw_rgb)},
        "depth": {**{k: v for k, v in depth.items() if k != "data"},
                  "file": depth_path.name, "bytes": len(raw_depth),
                  "sha256": sha256_bytes(raw_depth)},
        "intrinsics": {"rgb": intrinsic_rgb, "depth": intrinsic_depth},
        "extrinsics": {
            "reference_frame": "base_link",
            "pose_order": "[x,y,z,qx,qy,qz,qw]",
            "rgb": {"pose": list(extrinsic_rgb), "timestamp": extrinsic_rgb_ts},
            "depth": {"pose": list(extrinsic_depth), "timestamp": extrinsic_depth_ts},
        },
        "joint_state": joints,
        "base_pose_map": base_pose,
    }
    meta_path = folder / "capture.json"
    with meta_path.open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2, default=str)
        stream.write("\n")
    return metadata


def _bridge_frame(cam, source, *, attempts=4):
    """Newest JPEG for a source; mirrors tools/collect.capture_once retry logic."""
    best = None
    for _ in range(attempts):
        _, headers, body = cam.get(f"/frame/{source}")
        if body.startswith(b"\xff\xd8") and body.endswith(b"\xff\xd9"):
            age_ms = (time.time_ns() - int(headers["X-Frame-Captured-Ns"])) / 1e6
            if best is None or age_ms < best[0]:
                best = (age_ms, headers, body)
            if age_ms < 600:
                break
        time.sleep(0.25)
    if best is None:
        raise RuntimeError(f"no valid JPEG for {source}")
    return best


def capture_once_bridge(cam, folder, *, source="wrist_right", info=None, settle_s=0.5):
    """One stationary capture over the HPU camera bridge (read-only HTTP).

    The bridge exposes RGB, depth and colour intrinsics only: extrinsics and
    joint state stay unavailable and are recorded as such, never guessed.
    """
    from medicine_frontend.tools.collect import fetch_depth
    folder = Path(folder)
    if folder.exists():
        raise FileExistsError(f"Capture folder already exists: {folder}")
    if not 0 <= settle_s <= 5:
        raise ValueError("settle_s out of range")
    time.sleep(settle_s)
    captured_ns = time.time_ns()
    captured_monotonic = time.monotonic()
    age_ms, headers, raw_rgb = _bridge_frame(cam, source)
    rgb_captured = int(headers["X-Frame-Captured-Ns"])
    depth_raw, depth_meta = fetch_depth(cam, source, rgb_captured)
    if depth_raw is None:
        raise RuntimeError("depth unavailable:" + str(depth_meta.get("error")))
    intrinsics = (info or {}).get("intrinsics", {}).get(source)
    if not intrinsics or "K" not in intrinsics:
        raise RuntimeError(f"bridge has no intrinsics for {source}")
    folder.mkdir(parents=True)
    (folder / "rgb.jpg").write_bytes(raw_rgb)
    (folder / "depth.bin").write_bytes(depth_raw)
    metadata = {
        "schema": "drawer_stationary_capture/1",
        "transport": "camera_bridge",
        "captured_ns": captured_ns,
        "captured_monotonic_s": captured_monotonic,
        "settle_s": settle_s,
        "motion_commands_sent": [],
        "sensors": {"rgb": headers.get("X-Frame-Camera"),
                    "depth": depth_meta.get("camera_id"), "rgb_format": "JPEG"},
        "rgb": {"file": "rgb.jpg", "bytes": len(raw_rgb),
                "sha256": sha256_bytes(raw_rgb),
                "camera_id": headers.get("X-Frame-Camera"),
                "frame_timestamp_ns": int(headers["X-Frame-Timestamp-Ns"]),
                "host_captured_ns": rgb_captured,
                "age_ms": round(age_ms, 1), "stale": age_ms > 2500},
        "depth": {"file": "depth.bin", "bytes": len(depth_raw),
                  "sha256": sha256_bytes(depth_raw),
                  "format": "raw_uint16_le",
                  "depth_scale": depth_meta["depth_scale"],
                  "height": depth_meta["height"], "width": depth_meta["width"],
                  "camera_id": depth_meta.get("camera_id"),
                  "frame_timestamp_ns": depth_meta.get("frame_timestamp_ns"),
                  "host_captured_ns": depth_meta.get("host_captured_ns"),
                  "rgb_depth_skew_ms": depth_meta.get("rgb_depth_skew_ms")},
        "intrinsics": {"rgb": intrinsics, "depth": intrinsics,
                       "depth_intrinsics_source": "color_intrinsics_assumed_aligned"},
        "extrinsics": {"reference_frame": "base_link", "available": False,
                       "reason": "camera_bridge_exposes_no_extrinsics"},
        "joint_state": {"data": None,
                        "error": "camera_bridge_joint_state_unusable"},
        "base_pose_map": {"data": None,
                          "error": "record_console_map_pose_per_position"},
    }
    with (folder / "capture.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2, default=str)
        stream.write("\n")
    return metadata


def finish_manifest(session, session_dir, captures, started_ns, errors):
    session_dir = Path(session_dir)
    files = []
    for path in sorted(p for p in session_dir.rglob("*") if p.is_file()):
        if path.name == "manifest.json":
            continue
        if path.is_symlink():
            raise ValueError(f"Symlink in session: {path}")
        files.append({"file": path.relative_to(session_dir).as_posix(),
                      "bytes": path.stat().st_size,
                      "sha256": sha256_bytes(path.read_bytes())})
    manifest = {"schema": "drawer_stationary/1",
                "session": session,
                "started_ns": started_ns,
                "finished_ns": time.time_ns(),
                "motion_commands_sent": [],
                "captures": len(captures),
                "errors": errors,
                "files": files}
    with (session_dir / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return manifest


def _capture_loop(args, session_dir, capture_fn):
    captures, errors = [], []
    for index in range(1, args.captures + 1):
        if not args.no_prompt:
            input(f"[{index}/{args.captures}] Park at the next position, keep the "
                  "robot still, then press Enter. No motion commands are sent. ")
        try:
            meta = capture_fn(session_dir / f"capture_{index:02d}")
            captures.append(meta)
            print(f"SAVED capture_{index:02d} "
                  f"rgb={meta['rgb']['bytes']}B depth={meta['depth']['bytes']}B",
                  flush=True)
        except Exception as error:
            errors.append({"capture": index,
                           "error": f"{type(error).__name__}: {error}"[:500]})
            print(f"FAILED capture_{index:02d}: {error}", flush=True)
    return captures, errors


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    parser.add_argument("--session", default=f"drawer_stationary_{stamp}")
    parser.add_argument("--captures", type=int, default=5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--transport", choices=("sdk", "bridge"), default="sdk",
                        help="sdk: on-robot GalbotRobot; bridge: HTTP camera bridge")
    parser.add_argument("--cam-source", default="wrist_right")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Capture immediately without Enter between positions")
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    if not re.fullmatch(r"drawer_[A-Za-z0-9_-]+", args.session):
        parser.error("session must be drawer_* with ASCII letters, digits, _ or -")
    if not 2 <= args.captures <= 20 or not 0 <= args.settle_s <= 5:
        parser.error("captures must be 2..20 and settle-s 0..5")
    session_dir = args.root.resolve() / args.session
    if session_dir.exists():
        parser.error(f"Session folder already exists: {session_dir}")
    session_dir.mkdir(parents=True)

    started_ns = time.time_ns()
    if args.transport == "bridge":
        from medicine_frontend.tools.collect import Cam
        cam = Cam(os.environ.get("GALBOT_CAM_CODE"),
                  os.environ.get("GALBOT_CAM_TOKEN"))
        _, _, body = cam.get("/api/info", timeout=8)
        info = json.loads(body)
        captures, errors = _capture_loop(
            args, session_dir,
            lambda folder: capture_once_bridge(cam, folder, source=args.cam_source,
                                               info=info, settle_s=args.settle_s))
    else:
        from galbot_sdk.g1 import GalbotRobot
        robot = GalbotRobot()
        captures, errors = [], []
        try:
            sensors = _default_sensors()
            if not robot.init({sensors[0], sensors[1]}):
                raise RuntimeError("SDK camera initialization failed")
            time.sleep(2)
            captures, errors = _capture_loop(
                args, session_dir,
                lambda folder: capture_once(robot, folder, sensors=sensors,
                                            settle_s=args.settle_s))
        except KeyboardInterrupt:
            errors.append({"error": "operator_interrupt"})
        finally:
            robot.request_shutdown()
            robot.wait_for_shutdown()
            robot.destroy()
    manifest = finish_manifest(args.session, session_dir, captures, started_ns, errors)
    print(json.dumps({"manifest": str(session_dir / "manifest.json"),
                      "captures": manifest["captures"],
                      "errors": errors}, ensure_ascii=False), flush=True)
    return 0 if captures and not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
