"""Local capture catalogue. Original files stay on disk; no robot dependencies."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3
from datetime import datetime, timezone
from uuid import NAMESPACE_URL, uuid5

from .core import NAMES, parse_expiry_info

DEFAULT_ROOT = Path(__file__).resolve().parent / "data" / "medicine_dataset"
SOURCES = {"wrist_right", "wrist_left", "head_right", "head_left"}


def json_text(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


def numeric_joints(value):
    states = value.get("joint_state") if isinstance(value, dict) else None
    return (isinstance(states, list) and bool(states) and
            all(isinstance(s, dict) and isinstance(s.get("position"), (int, float))
                and not isinstance(s["position"], bool) and math.isfinite(s["position"])
                for s in states))


class Dataset:
    def __init__(self, root=DEFAULT_ROOT):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / "catalog.sqlite3"
        self.db = sqlite3.connect(self.path, timeout=15)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA foreign_keys=ON")
        version = self.db.execute("PRAGMA user_version").fetchone()[0]
        if version not in (0, 1, 2, 3):
            self.db.close()
            raise ValueError(f"Unsupported catalogue version: {version}")
        self.db.executescript("""
          CREATE TABLE IF NOT EXISTS captures (
            id TEXT PRIMARY KEY, metadata_path TEXT NOT NULL UNIQUE,
            session TEXT NOT NULL, kind TEXT NOT NULL, drug TEXT NOT NULL,
            captured_at TEXT, metadata_json TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS frames (
            id TEXT PRIMARY KEY, capture_id TEXT NOT NULL REFERENCES captures(id),
            source TEXT NOT NULL, image_path TEXT NOT NULL, depth_path TEXT,
            image_sha256 TEXT NOT NULL, timestamp_ns INTEGER,
            quality_json TEXT NOT NULL, metadata_json TEXT NOT NULL,
            UNIQUE(capture_id, source));
          CREATE TABLE IF NOT EXISTS annotations (
            frame_id TEXT NOT NULL REFERENCES frames(id), instance_id TEXT NOT NULL,
            annotation_json TEXT NOT NULL, updated_at TEXT NOT NULL,
            PRIMARY KEY(frame_id, instance_id));
          CREATE TABLE IF NOT EXISTS measurements (
            drug TEXT PRIMARY KEY, measurements_json TEXT NOT NULL, updated_at TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS state_snapshots (
            id TEXT PRIMARY KEY, file_path TEXT NOT NULL UNIQUE, sha256 TEXT NOT NULL,
            reference_session TEXT NOT NULL, schema_name TEXT NOT NULL,
            recorded_ns INTEGER NOT NULL, payload_json TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS teaching_events (
            id TEXT PRIMARY KEY, file_path TEXT NOT NULL UNIQUE, sha256 TEXT NOT NULL,
            reference_session TEXT NOT NULL, schema_name TEXT NOT NULL,
            recorded_ns INTEGER NOT NULL, payload_json TEXT NOT NULL);
          CREATE INDEX IF NOT EXISTS teaching_events_session_time
            ON teaching_events(reference_session, recorded_ns);
          PRAGMA user_version=3;
        """)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.db.close()

    def relative(self, path):
        return Path(path).resolve().relative_to(self.root).as_posix()

    def asset(self, parent, name):
        path = (parent / name).resolve()
        self.relative(path)  # Reject path escapes, including symlinks.
        return path

    def ingest(self, metadata_path):
        path = Path(metadata_path).resolve()
        relative = self.relative(path)
        meta = json.loads(path.read_text(encoding="utf-8"))
        if meta.get("schema") != "medicine_capture/1" or not meta.get("sources"):
            raise ValueError("Expected medicine_capture/1 with at least one source")
        capture_id = str(uuid5(NAMESPACE_URL, relative))
        frames = []
        camera_path = self.asset(path.parent, meta.get("camera_info_file", "camera_info.json"))
        info = json.loads(camera_path.read_text(encoding="utf-8")) if camera_path.is_file() else {}
        intrinsics = info.get("info", {}).get("intrinsics", {})
        for source, data in meta["sources"].items():
            if source not in SOURCES:
                raise ValueError(f"Unknown camera: {source}")
            rgb = self.asset(path.parent, data.get("file", f"{path.stem}_{source}.jpg"))
            raw = rgb.read_bytes()
            if not raw.startswith(b"\xff\xd8") or not raw.endswith(b"\xff\xd9"):
                raise ValueError(f"Incomplete JPEG: {rgb.name}")
            digest = hashlib.sha256(raw).hexdigest()
            if data.get("jpeg_sha256") and data["jpeg_sha256"] != digest:
                raise ValueError(f"Image checksum mismatch: {rgb.name}")
            flags = []
            if data.get("camera_error"):
                flags.append("camera_error")
            if data.get("stale") or data.get("age_ms", 0) > 2500:
                flags.append("stale_rgb")
            if data.get("age_ms", 0) < 0:
                flags.append("clock_offset_detected")
            if set(meta.get("requested_sources", meta["sources"])) - set(meta["sources"]):
                flags.append("incomplete_capture")
            intrinsic = intrinsics.get(source, {})
            if not isinstance(intrinsic.get("K"), list) or len(intrinsic["K"]) != 9:
                flags.append("missing_intrinsics")
            depth_path = None
            if source.startswith("wrist"):
                depth = data.get("depth", {})
                if depth.get("file"):
                    dp = self.asset(path.parent, depth["file"])
                    if dp.stat().st_size != depth["width"] * depth["height"] * 2:
                        raise ValueError(f"Depth byte count mismatch: {dp.name}")
                    if depth.get("sha256") and hashlib.sha256(dp.read_bytes()).hexdigest() != depth["sha256"]:
                        raise ValueError(f"Depth checksum mismatch: {dp.name}")
                    depth_path = self.relative(dp)
                    if not isinstance(depth.get("depth_scale"), (float, int)) or depth["depth_scale"] <= 0:
                        flags.append("invalid_depth_scale")
                    skew = depth.get("rgb_depth_skew_ms")
                    if skew is None or not math.isfinite(skew) or abs(skew) > 1500:
                        flags.append("rgb_depth_skew")
                    if depth.get("error") or depth.get("age_ms", 0) > 2500:
                        flags.append("depth_error_or_stale")
                    if depth.get("age_ms", 0) < 0 and "clock_offset_detected" not in flags:
                        flags.append("clock_offset_detected")
                else:
                    flags.append("missing_depth")
                if not numeric_joints(meta.get("joint_state")):
                    flags.append("joint_state_not_numeric")
                # Saving both files does not prove spatial registration/extrinsics.
                flags.append("geometry_calibration_unverified")
            frames.append((f"{capture_id}:{source}", capture_id, source,
                           self.relative(rgb), depth_path, digest,
                           data.get("frame_timestamp_ns"), json_text(flags), json_text(data)))
        # Existing captures are immutable; replay import never destroys annotations.
        existing = self.db.execute("SELECT metadata_json FROM captures WHERE id=?", (capture_id,)).fetchone()
        if existing:
            if json.loads(existing[0]) != meta:
                raise ValueError("Previously indexed metadata changed; preserve original and create a new capture")
            indexed_hashes = dict(self.db.execute("SELECT id,image_sha256 FROM frames WHERE capture_id=?", (capture_id,)))
            if any(indexed_hashes.get(frame[0]) != frame[5] for frame in frames):
                raise ValueError("Previously indexed image changed; preserve the original capture")
            # Recompute derived quality warnings without changing source metadata or labels.
            with self.db:
                self.db.executemany("UPDATE frames SET quality_json=? WHERE id=?",
                                    [(frame[7], frame[0]) for frame in frames])
            return capture_id
        with self.db:
            self.db.execute("INSERT INTO captures VALUES (?,?,?,?,?,?,?)", (
                capture_id, relative, str(meta.get("capture_session", path.parent.name)),
                meta.get("kind", "unknown"), meta.get("drug", "none"),
                meta.get("captured_local"), json_text(meta)))
            self.db.executemany("INSERT INTO frames VALUES (?,?,?,?,?,?,?,?,?)", frames)
        return capture_id

    def import_existing(self):
        added, errors = [], []
        for path in sorted((self.root / "raw").rglob("*.json")):
            if path.name.startswith("camera_info") or path.name == "session_index.json":
                continue
            try:
                meta = json.loads(path.read_text(encoding="utf-8"))
                if meta.get("schema") == "medicine_capture/1":
                    added.append(self.ingest(path))
            except (ValueError, KeyError, TypeError, OSError) as exc:
                errors.append({"file": self.relative(path), "error": str(exc)})
        return {"indexed": len(added), "errors": errors}

    def ingest_state(self, path, reference_session):
        """Index separately timed SDK observations, never relabel them as camera-time state."""
        path = Path(path).resolve()
        relative = self.relative(path)
        raw = path.read_bytes()
        value = json.loads(raw.decode("utf-8"))
        if value.get("schema") not in ("sdk_console_snapshot/1", "sdk_end_effector_snapshot/1"):
            raise ValueError("Unsupported state snapshot schema")
        if value.get("motion_commands_sent") != []:
            raise ValueError("Expected a read-only state snapshot")
        recorded = value.get("finished_ns", value.get("received_ns"))
        if not isinstance(recorded, int) or isinstance(recorded, bool) or recorded <= 0:
            raise ValueError("A recorded timestamp is required")
        digest = hashlib.sha256(raw).hexdigest()
        identity = str(uuid5(NAMESPACE_URL, relative))
        existing = self.db.execute("SELECT sha256,reference_session FROM state_snapshots WHERE id=?", (identity,)).fetchone()
        if existing:
            if tuple(existing) != (digest, reference_session):
                raise ValueError("Previously indexed state snapshot changed")
            return identity
        with self.db:
            self.db.execute("INSERT INTO state_snapshots VALUES (?,?,?,?,?,?,?)", (
                identity, relative, digest, reference_session, value["schema"], recorded, json_text(value)))
        return identity

    def ingest_teaching_event(self, path, reference_session):
        """Index an immutable command/result record or demonstration manifest."""
        path = Path(path).resolve()
        relative = self.relative(path)
        raw = path.read_bytes()
        value = json.loads(raw.decode("utf-8"))
        schema = value.get("schema")
        if schema not in ("drawer_grip_verification/1", "drawer_arm_pull_step/1",
                          "drawer_demonstration/1", "drawer_telemetry_stream/1",
                          "drawer_close_start/1", "drawer_force_baseline/1",
                          "drawer_arm_close_step/1", "drawer_close_stop/1",
                          "drawer_controller_switch/1", "drawer_regrip/1",
                          "drawer_replay_phase/1", "drawer_replay_observation/1",
                          "drawer_camera_stream/1", "drawer_alignment_phase/1",
                          "drawer_regrip_phase/1", "drawer_regrip_report/1",
                          "drawer_arm_step/1"):
            raise ValueError("Unsupported teaching event schema")
        recorded = (value.get("started_ns") or value.get("command_started_ns")
                    or value.get("recorded_ns"))
        if not isinstance(recorded, int) or isinstance(recorded, bool) or recorded <= 0:
            raise ValueError("Teaching event requires a positive recorded timestamp")
        digest = hashlib.sha256(raw).hexdigest()
        identity = str(uuid5(NAMESPACE_URL, relative))
        existing = self.db.execute(
            "SELECT sha256,reference_session FROM teaching_events WHERE id=?", (identity,)).fetchone()
        if existing:
            if tuple(existing) != (digest, reference_session):
                raise ValueError("Previously indexed teaching event changed")
            return identity
        with self.db:
            self.db.execute("INSERT INTO teaching_events VALUES (?,?,?,?,?,?,?)", (
                identity, relative, digest, reference_session, schema, recorded, json_text(value)))
        return identity

    def annotate(self, frame_id, annotation):
        """Human labels are training truth only, never runtime OCR evidence."""
        instance = annotation.get("instance_id")
        if not isinstance(instance, str) or not instance.strip():
            raise ValueError("instance_id required")
        if annotation.get("drug") not in NAMES:
            raise ValueError("drug must be one of the four fixed medicines")
        if annotation.get("geometry") not in ("box", "cylinder", "unknown"):
            raise ValueError("geometry must be box/cylinder/unknown")
        box = annotation.get("bbox_xywh")
        if (not isinstance(box, list) or len(box) != 4 or
                not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in box)
                or min(box[:2]) < 0 or min(box[2:]) <= 0):
            raise ValueError("bbox_xywh requires nonnegative origin and positive size")
        truth = annotation.get("expiry_truth")
        if truth is not None:
            expiry = parse_expiry_info(truth)
            annotation = dict(annotation, expiry_precision=expiry.precision)
        if not annotation.get("annotator"):
            raise ValueError("annotator required")
        with self.db:
            self.db.execute("INSERT INTO annotations VALUES (?,?,?,?) ON CONFLICT(frame_id,instance_id) "
                            "DO UPDATE SET annotation_json=excluded.annotation_json,updated_at=excluded.updated_at",
                            (frame_id, instance, json_text(annotation), datetime.now(timezone.utc).isoformat()))

    def measure(self, drug, dimensions):
        if drug not in NAMES:
            raise ValueError("Unknown medicine")
        keys = ("length_mm", "width_mm", "height_mm") if drug in ("ganmaoling", "siji") else (
            "height_mm", "cap_diameter_mm", "grip_diameter_mm")
        if not all(isinstance(dimensions.get(k), (int, float)) and not isinstance(dimensions[k], bool)
                   and math.isfinite(dimensions[k]) and dimensions[k] > 0 for k in keys):
            raise ValueError(f"Positive measurements required: {keys}")
        with self.db:
            self.db.execute("INSERT INTO measurements VALUES (?,?,?) ON CONFLICT(drug) DO UPDATE SET "
                            "measurements_json=excluded.measurements_json,updated_at=excluded.updated_at",
                            (drug, json_text(dimensions), datetime.now(timezone.utc).isoformat()))

    def records(self):
        for row in self.db.execute("SELECT frames.*,captures.session,captures.kind,captures.drug,captures.metadata_path, "
                                   "captures.metadata_json AS capture_metadata_json "
                                   "FROM frames JOIN captures ON captures.id=frames.capture_id ORDER BY frames.rowid"):
            item = dict(row)
            item["quality_flags"] = json.loads(item.pop("quality_json"))
            item["metadata"] = json.loads(item.pop("metadata_json"))
            item["capture_metadata"] = json.loads(item.pop("capture_metadata_json"))
            item["annotations"] = [json.loads(a[0]) for a in self.db.execute(
                "SELECT annotation_json FROM annotations WHERE frame_id=? ORDER BY instance_id", (item["id"],))]
            # Export grouping hints; a training splitter must also group shared label entities.
            item["split_group"] = item["session"]
            item["label_entity_id"] = item["capture_metadata"].get("label_truth", {}).get("entity_id")
            item["review_status"] = "labelled" if item["annotations"] else "unlabelled"
            yield item

    def status(self):
        return {"database": str(self.path), **{table: self.db.execute(f"SELECT count(*) FROM {table}").fetchone()[0]
                for table in ("captures", "frames", "annotations", "measurements", "state_snapshots",
                              "teaching_events")},
                "frames_by_source": dict(self.db.execute("SELECT source,count(*) FROM frames GROUP BY source")),
                "frames_with_warnings": self.db.execute("SELECT count(*) FROM frames WHERE quality_json!='[]'").fetchone()[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["init", "import", "status", "export", "annotate", "measure"])
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--input", type=Path, help="Annotation or measurement JSON")
    parser.add_argument("--frame-id")
    parser.add_argument("--drug")
    args = parser.parse_args()
    with Dataset(args.root) as dataset:
        if args.command == "import":
            result = dataset.import_existing()
        elif args.command == "export":
            if not args.output:
                parser.error("export requires --output")
            with args.output.open("x", encoding="utf-8") as output:
                for record in dataset.records():
                    output.write(json_text(record) + "\n")
            result = {"output": str(args.output)}
        elif args.command in ("annotate", "measure"):
            if not args.input:
                parser.error("--input required")
            value = json.loads(args.input.read_text(encoding="utf-8"))
            if args.command == "annotate":
                dataset.annotate(args.frame_id, value)
            else:
                dataset.measure(args.drug, value)
            result = dataset.status()
        else:
            result = dataset.status()
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1 if result.get("errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
