import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from medicine_frontend.dataset import Dataset


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.folder = self.root / "raw" / "wrist" / "test"
        self.folder.mkdir(parents=True)
        self.path = self.folder / "0001.json"
        self.rgb = b"\xff\xd8test\xff\xd9"
        (self.folder / "0001_wrist_right.jpg").write_bytes(self.rgb)
        (self.folder / "depth.bin").write_bytes(bytes(8))
        self.meta = {"schema": "medicine_capture/1", "capture_session": "test", "kind": "wrist", "drug": "siji",
                     "joint_state": {"joint_state": ["<JointState object>"]}, "sources": {"wrist_right": {
                         "jpeg_sha256": hashlib.sha256(self.rgb).hexdigest(),
                         "depth": {"file": "depth.bin", "width": 2, "height": 2, "depth_scale": 10000,
                                   "rgb_depth_skew_ms": 1694.7}}}}
        self.save()

    def tearDown(self):
        self.temp.cleanup()

    def save(self):
        self.path.write_text(json.dumps(self.meta), encoding="utf-8")

    def test_import_idempotent_and_preserves_annotations(self):
        with Dataset(self.root) as db:
            db.ingest(self.path)
            frame = next(db.records())
            db.annotate(frame["id"], {"instance_id": "one", "drug": "siji", "geometry": "box",
                                      "bbox_xywh": [1, 2, 3, 4], "expiry_truth": "EXP 2027-02", "annotator": "tester"})
            db.ingest(self.path)
            self.assertEqual(db.status()["captures"], 1)
            self.assertEqual(db.status()["annotations"], 1)
            frame = next(db.records())
            self.assertIn("rgb_depth_skew", frame["quality_flags"])
            self.assertIn("joint_state_not_numeric", frame["quality_flags"])
            self.assertEqual(frame["annotations"][0]["expiry_precision"], "month")

    def test_bad_checksum_does_not_create_partial_rows(self):
        self.meta["sources"]["wrist_right"]["jpeg_sha256"] = "invalid"
        self.save()
        with Dataset(self.root) as db:
            with self.assertRaises(ValueError):
                db.ingest(self.path)
            self.assertEqual(db.status()["captures"], 0)

    def test_escape_and_depth_corruption(self):
        with Dataset(self.root) as db:
            self.meta["sources"]["wrist_right"]["file"] = "../../../../../outside.jpg"
            self.save()
            with self.assertRaises(ValueError):
                db.ingest(self.path)
            del self.meta["sources"]["wrist_right"]["file"]
            self.save()
            (self.folder / "depth.bin").write_bytes(b"x")
            with self.assertRaises(ValueError):
                db.ingest(self.path)

    def test_reimport_changed_metadata_rejected(self):
        with Dataset(self.root) as db:
            db.ingest(self.path)
            self.meta["drug"] = "fish_oil"
            self.save()
            with self.assertRaises(ValueError):
                db.ingest(self.path)

    def test_measurements_require_real_values(self):
        with Dataset(self.root) as db:
            with self.assertRaises(ValueError):
                db.measure("siji", {"length_mm": 0, "width_mm": 2, "height_mm": 3})
            db.measure("siji", {"length_mm": 10, "width_mm": 20, "height_mm": 30})
            self.assertEqual(db.status()["measurements"], 1)

    def test_clock_offset_and_partial_capture_flagged(self):
        self.meta["requested_sources"] = ["wrist_right", "wrist_left"]
        self.meta["sources"]["wrist_right"]["age_ms"] = -350
        self.save()
        with Dataset(self.root) as db:
            db.ingest(self.path)
            flags = next(db.records())["quality_flags"]
            self.assertIn("clock_offset_detected", flags)
            self.assertIn("incomplete_capture", flags)

    def test_state_snapshot_preserves_time_and_is_immutable(self):
        path = self.root / "state.json"
        payload = {"schema": "sdk_console_snapshot/1", "motion_commands_sent": [],
                   "finished_ns": 123456, "reads": {"navigation": {"data": {"pose": None}}}}
        path.write_text(json.dumps(payload), encoding="utf-8")
        with Dataset(self.root) as db:
            identity = db.ingest_state(path, "camera_session")
            self.assertEqual(db.ingest_state(path, "camera_session"), identity)
            self.assertEqual(db.status()["state_snapshots"], 1)
            row = db.db.execute("SELECT recorded_ns,payload_json FROM state_snapshots").fetchone()
            self.assertEqual(row[0], 123456)
            self.assertIsNone(json.loads(row[1])["reads"]["navigation"]["data"]["pose"])
            payload["finished_ns"] = 123457
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(ValueError):
                db.ingest_state(path, "camera_session")

    def test_teaching_event_is_ordered_and_immutable(self):
        first = self.root / "step.json"
        second = self.root / "manifest.json"
        step = {"schema": "drawer_arm_pull_step/1", "started_ns": 100,
                "command": {"dx": -0.02}, "target_reached": True}
        manifest = {"schema": "drawer_demonstration/1", "recorded_ns": 200,
                    "source_sessions": ["a", "b"]}
        first.write_text(json.dumps(step), encoding="utf-8")
        second.write_text(json.dumps(manifest), encoding="utf-8")
        with Dataset(self.root) as db:
            identity = db.ingest_teaching_event(first, "demo")
            db.ingest_teaching_event(second, "demo")
            self.assertEqual(db.ingest_teaching_event(first, "demo"), identity)
            rows = db.db.execute("SELECT schema_name,recorded_ns FROM teaching_events "
                                 "WHERE reference_session=? ORDER BY recorded_ns", ("demo",)).fetchall()
            self.assertEqual([tuple(row) for row in rows], [
                ("drawer_arm_pull_step/1", 100), ("drawer_demonstration/1", 200)])
            step["command"]["dx"] = -0.03
            first.write_text(json.dumps(step), encoding="utf-8")
            with self.assertRaises(ValueError):
                db.ingest_teaching_event(first, "demo")

    def test_telemetry_manifest_is_indexed_and_immutable(self):
        path = self.root / "telemetry.json"
        payload = {"schema": "drawer_telemetry_stream/1", "recorded_ns": 300,
                   "counts": {"tcp": 5}, "motion_commands_sent": []}
        path.write_text(json.dumps(payload), encoding="utf-8")
        with Dataset(self.root) as db:
            identity = db.ingest_teaching_event(path, "drawer_next")
            self.assertEqual(db.ingest_teaching_event(path, "drawer_next"), identity)
            row = db.db.execute("SELECT schema_name,recorded_ns FROM teaching_events").fetchone()
            self.assertEqual(tuple(row), ("drawer_telemetry_stream/1", 300))
            payload["counts"]["tcp"] = 6
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(ValueError):
                db.ingest_teaching_event(path, "drawer_next")


if __name__ == "__main__":
    unittest.main()
