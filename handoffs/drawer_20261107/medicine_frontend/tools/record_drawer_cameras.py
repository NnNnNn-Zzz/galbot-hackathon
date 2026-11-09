"""Timed, read-only four-camera capture for a drawer demonstration.

Run alongside record_drawer_stream.py. RGB and wrist depth retain the camera
bridge timestamps and are indexed by the existing local Dataset. The two
camera groups are captured concurrently; every tick records missing sources.
The camera access code is read from the process environment, never written to
the dataset or included in a command-line argument.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time

from medicine_frontend.dataset import Dataset
from medicine_frontend.tools import collect


GROUPS = {
    "wrist": ("wrist_right", "wrist_left"),
    "head": ("head_right", "head_left"),
}
ALL_SOURCES = tuple(source for pair in GROUPS.values() for source in pair)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root).as_posix()


def camera_args(root: Path, info_file: str, kind: str):
    return argparse.Namespace(
        drug="none", arrangement="drawer_replay", distance_cm=30.0,
        lighting="demo", grasp_state="observed_during_drawer_replay",
        note="Timed four-camera drawer demonstration; observation only.",
        sources=list(GROUPS[kind]), depth=True, joint_state=True,
        root=root, camera_info_file=info_file,
        label_id=None, expiry_truth=None,
    )


def capture_group(cam, args, seq: int, session: str, kind: str, folder: Path):
    """Capture one group; keep the other group's result if this one fails."""
    result = {"kind": kind, "requested_sources": list(GROUPS[kind]),
              "capture_returned": False, "error": None}
    try:
        result["capture_returned"] = bool(
            collect.capture_once(cam, args, seq, session, kind, folder))
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"[:500]
    matches = list(folder.glob(f"{seq:04d}_*.json"))
    if len(matches) == 1:
        result["metadata_file"] = matches[0]
        try:
            metadata = json.loads(matches[0].read_text(encoding="utf-8"))
            result["sources"] = metadata.get("sources", {})
        except (OSError, ValueError, TypeError) as error:
            result["sources"] = {}
            result["error"] = (result["error"] or "") + f"; metadata: {error}"
    else:
        result["metadata_file"] = None
        result["sources"] = {}
        if len(matches) > 1:
            result["error"] = (result["error"] or "") + "; multiple metadata files for tick"
    result["saved_sources"] = sorted(result["sources"])
    result["missing_sources"] = sorted(set(GROUPS[kind]) - set(result["sources"]))
    return result


def index_for_session(root: Path, session: str):
    with Dataset(root) as dataset:
        rows = dataset.db.execute(
            "SELECT frames.source,frames.image_path,frames.depth_path,"
            "frames.quality_json,captures.metadata_path "
            "FROM frames JOIN captures ON frames.capture_id=captures.id "
            "WHERE captures.session=?", (session,)).fetchall()
    by_metadata = {}
    for row in rows:
        by_metadata.setdefault(row["metadata_path"], {})[row["source"]] = {
            "image_path": row["image_path"], "depth_path": row["depth_path"],
            "quality_flags": json.loads(row["quality_json"]),
        }
    return by_metadata


def file_inventory(root: Path, folders: dict[str, Path], tick_log: Path):
    paths = [tick_log]
    for folder in folders.values():
        paths.extend(path for path in folder.rglob("*") if path.is_file())
    files = []
    for path in sorted(set(paths)):
        if path.is_symlink():
            raise ValueError(f"Symlink in camera session: {path}")
        files.append({"file": relative(root, path), "bytes": path.stat().st_size,
                      "sha256": sha256_file(path)})
    return files


def record(session: str, duration_s: float, interval_s: float,
           max_empty_ticks: int, root: Path):
    root = root.resolve()
    with Dataset(root) as dataset:
        if dataset.db.execute("SELECT 1 FROM captures WHERE session=? LIMIT 1",
                              (session,)).fetchone():
            raise FileExistsError(f"Session already indexed: {session}")
    teaching = root / "teaching" / session / "camera_stream"
    folders = {kind: root / "raw" / kind / session for kind in GROUPS}
    if teaching.exists() or any(folder.exists() for folder in folders.values()):
        raise FileExistsError(f"Session folder already exists: {session}")
    teaching.mkdir(parents=True)
    for folder in folders.values():
        folder.mkdir(parents=True)

    started_ns = time.time_ns()
    started_monotonic = time.monotonic()
    deadline = started_monotonic + duration_s
    tick_log = teaching / "ticks.ndjson"
    camera_info_errors = []
    stop_reason = "duration_completed"
    tick_count = 0
    scheduled_skips = 0
    empty_streak = 0
    cam = None
    try:
        code = os.environ.get("GALBOT_CAM_CODE")
        token = os.environ.get("GALBOT_CAM_TOKEN")
        if not code and not token:
            raise ValueError("Set GALBOT_CAM_CODE or GALBOT_CAM_TOKEN in the process environment")
        cam = collect.Cam(code, token)
        try:
            _, _, raw_info = cam.get("/api/info", timeout=5)
            info = json.loads(raw_info)
        except Exception as error:
            info = {}
            camera_info_errors.append(f"{type(error).__name__}: {error}"[:500])
        info_name = f"camera_info_{started_ns}.json"
        for folder in folders.values():
            info_path = folder / info_name
            info_path.write_text(json.dumps({
                "base_url": collect.BASE, "fetched_at_ns": time.time_ns(),
                "info": info, "errors": camera_info_errors,
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        args = {kind: camera_args(root, info_name, kind) for kind in GROUPS}
        with tick_log.open("x", encoding="utf-8") as log, \
                ThreadPoolExecutor(max_workers=2) as pool:
            tick_index = 0
            while True:
                scheduled = started_monotonic + tick_index * interval_s
                if scheduled >= deadline:
                    break
                remaining = scheduled - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                if time.monotonic() >= deadline:
                    break
                begun_ns = time.time_ns()
                futures = {kind: pool.submit(capture_group, cam, args[kind],
                                             tick_index + 1, session, kind, folder)
                           for kind, folder in folders.items()}
                groups = {kind: future.result() for kind, future in futures.items()}
                tick = {
                    "tick": tick_index + 1,
                    "scheduled_ns": started_ns + round(tick_index * interval_s * 1e9),
                    "started_ns": begun_ns,
                    "finished_ns": time.time_ns(),
                    "groups": {},
                    "missing_sources": [],
                }
                for kind, group in groups.items():
                    metadata_path = group.pop("metadata_file")
                    group.pop("sources")
                    if metadata_path is not None:
                        group["metadata_file"] = relative(root, metadata_path)
                        group["metadata_sha256"] = sha256_file(metadata_path)
                    else:
                        group["metadata_file"] = None
                    tick["groups"][kind] = group
                    tick["missing_sources"].extend(group["missing_sources"])
                tick["missing_sources"].sort()
                log.write(json.dumps(tick, ensure_ascii=False, allow_nan=False) + "\n")
                log.flush()
                os.fsync(log.fileno())
                tick_count += 1
                empty_streak = empty_streak + 1 if len(tick["missing_sources"]) == 4 else 0
                if empty_streak >= max_empty_ticks:
                    stop_reason = "all_cameras_failed"
                    break
                next_index = tick_index + 1
                elapsed_intervals = math.ceil(
                    (time.monotonic() - started_monotonic) / interval_s - 1e-9)
                if elapsed_intervals > next_index:
                    scheduled_skips += elapsed_intervals - next_index
                    next_index = elapsed_intervals
                tick_index = next_index
    except KeyboardInterrupt:
        stop_reason = "operator_interrupt"
    except (Exception, SystemExit) as error:
        stop_reason = "capture_error"
        camera_info_errors.append(f"{type(error).__name__}: {error}"[:500])

    finished_ns = time.time_ns()
    indexed = index_for_session(root, session)
    counts = {source: {"saved": 0, "indexed": 0, "missing_ticks": 0,
                       "stale_rgb": 0, "camera_error": 0, "clock_offset": 0,
                       "depth_saved": 0, "depth_missing": 0,
                       "rgb_depth_skew": 0, "depth_error_or_stale": 0,
                       "duplicate_timestamps": 0,
                       "rgb_age_ms_min": None, "rgb_age_ms_max": None,
                       "depth_abs_skew_ms_max": None}
              for source in ALL_SOURCES}
    last_timestamp = {}
    errors = list(camera_info_errors)
    with tick_log.open("a+", encoding="utf-8") as log:
        log.seek(0)
        for line in log:
            tick = json.loads(line)
            for source in tick["missing_sources"]:
                counts[source]["missing_ticks"] += 1
            for group in tick["groups"].values():
                if group["error"]:
                    errors.append({"tick": tick["tick"], "kind": group["kind"],
                                   "error": group["error"]})
                metadata_rel = group["metadata_file"]
                if metadata_rel is None:
                    continue
                try:
                    metadata = json.loads((root / metadata_rel).read_text(encoding="utf-8"))
                except (OSError, ValueError, TypeError) as error:
                    errors.append({"tick": tick["tick"], "error": f"metadata: {error}"})
                    continue
                db_rows = indexed.get(metadata_rel, {})
                for source, frame in metadata.get("sources", {}).items():
                    if source not in counts:
                        errors.append({"tick": tick["tick"], "error": f"Unknown source: {source}"})
                        continue
                    stat = counts[source]
                    stat["saved"] += 1
                    if source in db_rows:
                        stat["indexed"] += 1
                        flags = db_rows[source]["quality_flags"]
                        stat["stale_rgb"] += "stale_rgb" in flags
                        stat["camera_error"] += "camera_error" in flags
                        stat["clock_offset"] += "clock_offset_detected" in flags
                        stat["rgb_depth_skew"] += "rgb_depth_skew" in flags
                        stat["depth_error_or_stale"] += "depth_error_or_stale" in flags
                    else:
                        errors.append({"tick": tick["tick"], "source": source,
                                       "error": "Frame was saved but not indexed"})
                        stat["stale_rgb"] += bool(frame.get("stale"))
                        stat["camera_error"] += bool(frame.get("camera_error"))
                        stat["clock_offset"] += frame.get("age_ms", 0) < 0
                    age = frame.get("age_ms")
                    if isinstance(age, (int, float)) and math.isfinite(age):
                        stat["rgb_age_ms_min"] = (age if stat["rgb_age_ms_min"] is None
                                                   else min(stat["rgb_age_ms_min"], age))
                        stat["rgb_age_ms_max"] = (age if stat["rgb_age_ms_max"] is None
                                                   else max(stat["rgb_age_ms_max"], age))
                    if source.startswith("wrist"):
                        depth = frame.get("depth") or {}
                        stat["depth_saved"] += bool(depth.get("file"))
                        stat["depth_missing"] += not bool(depth.get("file"))
                        skew = depth.get("rgb_depth_skew_ms")
                        if isinstance(skew, (int, float)) and math.isfinite(skew):
                            skew = abs(skew)
                            stat["depth_abs_skew_ms_max"] = (
                                skew if stat["depth_abs_skew_ms_max"] is None
                                else max(stat["depth_abs_skew_ms_max"], skew))
                            if source not in db_rows:
                                stat["rgb_depth_skew"] += skew > 1500
                        if source not in db_rows:
                            stat["depth_error_or_stale"] += bool(depth.get("error")) or (
                                isinstance(depth.get("age_ms"), (int, float))
                                and depth["age_ms"] > 2500)
                    stamp = frame.get("frame_timestamp_ns")
                    if isinstance(stamp, int):
                        if source in last_timestamp and stamp <= last_timestamp[source]:
                            stat["duplicate_timestamps"] += 1
                        last_timestamp[source] = stamp

    files = file_inventory(root, folders, tick_log)
    manifest = {
        "schema": "drawer_camera_stream/1",
        "session": session,
        "recorded_ns": started_ns,
        "finished_ns": finished_ns,
        "source": collect.BASE,
        "robot_motion_commands_sent": [],
        "requested_duration_s": duration_s,
        "requested_interval_s": interval_s,
        "ticks_attempted": tick_count,
        "ticks_skipped_due_to_slow_capture": scheduled_skips,
        "achieved_tick_rate_hz": round(tick_count / max((finished_ns - started_ns) / 1e9, 0.001), 3),
        "stop_reason": stop_reason,
        "camera_info_errors": camera_info_errors,
        "counts_by_source": counts,
        "errors": errors,
        "tick_stream": relative(root, tick_log),
        "catalog": relative(root, root / "catalog.sqlite3"),
        "files": files,
    }
    manifest_path = teaching / "recording.json"
    with manifest_path.open("x", encoding="utf-8") as output:
        json.dump(manifest, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    return manifest_path, manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", default="drawer_replay_" + datetime.now(
        timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--duration-s", type=float, default=600.0)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--max-empty-ticks", type=int, default=5)
    parser.add_argument("--root", type=Path, default=collect.DATA_ROOT)
    args = parser.parse_args(argv)
    if not re.fullmatch(r"drawer_[A-Za-z0-9_-]+", args.session):
        parser.error("session must be drawer_* with ASCII letters, digits, _ or -")
    if not 0.1 <= args.duration_s <= 900.0 or not 0.25 <= args.interval_s <= 10.0:
        parser.error("duration-s must be 0.1..900, interval-s must be 0.25..10")
    if not 1 <= args.max_empty_ticks <= 20:
        parser.error("max-empty-ticks must be 1..20")
    path, manifest = record(args.session, args.duration_s, args.interval_s,
                            args.max_empty_ticks, args.root)
    print(json.dumps({"manifest": str(path),
                      "stop_reason": manifest["stop_reason"],
                      "ticks_attempted": manifest["ticks_attempted"],
                      "counts_by_source": manifest["counts_by_source"],
                      "error_count": len(manifest["errors"])}, ensure_ascii=False), flush=True)
    return 0 if (manifest["ticks_attempted"] > 0
                 and manifest["stop_reason"] in ("duration_completed", "operator_interrupt")
                 and all(manifest["counts_by_source"][source]["indexed"] > 0
                         for source in ALL_SOURCES)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
