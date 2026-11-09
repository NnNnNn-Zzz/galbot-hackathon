import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from medicine_frontend.tools import collect
from medicine_frontend.dataset import Dataset


class CameraFixture:
    def __init__(self, *args):
        pass

    def get(self, path, timeout=10):
        if path == "/api/info":
            return 200, {}, json.dumps({"intrinsics": {s: {"K": [1] * 9} for s in collect.SOURCES}}).encode()
        if path == "/api/joint_state":
            return 200, {}, b'{"joint_state": ["unusable"]}'
        ts = time.time_ns()
        headers = {"X-Frame-Captured-Ns": str(ts), "X-Frame-Timestamp-Ns": str(ts)}
        if path.startswith("/depth/"):
            headers.update({"X-Depth-Width": "2", "X-Depth-Height": "2", "X-Depth-Scale": "1000"})
            return 200, headers, bytes(8)
        return 200, headers, b"\xff\xd8fixture\xff\xd9"


class CollectionTests(unittest.TestCase):
    def test_two_cameras_persist_and_register_with_annotations_separate(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(collect, "Cam", CameraFixture):
            code = collect.main(["wrist", "--sources", "wrist_right", "wrist_left", "--root", directory,
                                 "--code", "test", "--drug", "siji", "--session", "test", "--yes",
                                 "--label-id", "siji01", "--expiry-truth", "EXP 2027-02"])
            self.assertEqual(code, 0)
            with Dataset(directory) as db:
                self.assertEqual(db.status()["frames"], 2)
                self.assertEqual(db.status()["annotations"], 0)
                record = next(db.records())
                self.assertTrue((Path(directory) / record["depth_path"]).is_file())
                self.assertIn("joint_state_not_numeric", record["quality_flags"])
                self.assertEqual(db.import_existing()["errors"], [])
                self.assertEqual(db.status()["frames"], 2)

    def test_filename_escape_rejected_before_network(self):
        with patch.object(collect, "Cam", side_effect=AssertionError("no network")):
            with self.assertRaises(SystemExit):
                collect.main(["wrist", "--arrangement", "../../escape", "--code", "test"])


if __name__ == "__main__":
    unittest.main()
