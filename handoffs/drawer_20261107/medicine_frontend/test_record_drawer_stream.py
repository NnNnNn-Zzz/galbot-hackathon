"""Local protocol checks for the read-only drawer recorder."""

from io import BytesIO, StringIO
import json
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from medicine_frontend.tools import record_drawer_stream as recorder
from medicine_frontend.dataset import Dataset


class RecorderTests(unittest.TestCase):
    def test_websocket_extended_length_preserves_joint_event(self):
        event = {"type": "joint_state", "timestamp_ns": 123,
                 "data": [{"name": "right_arm_joint1", "position": 0.25}] * 20}
        payload = json.dumps(event).encode("utf-8")
        frame = bytes((0x81, 126)) + struct.pack("!H", len(payload)) + payload
        opcode, actual = recorder.read_event_frame(
            None, bytearray(frame), time.monotonic() + 1)
        self.assertEqual(opcode, 1)
        self.assertEqual(json.loads(actual), event)

    def test_tcp_poll_writes_timestamped_pose_without_command(self):
        output = StringIO()
        counts = {"tcp": 0}
        errors = []
        response = BytesIO(json.dumps({"available": True, "frame_id": "base_link",
                                       "pose": [0, 0, 0, 0, 0, 0, 1]}).encode())
        class Opened:
            def __enter__(self):
                response.seek(0)
                return response

            def __exit__(self, *_):
                pass

        with patch.object(recorder.request, "urlopen", return_value=Opened()) as urlopen:
            recorder.poll_tcp("session=test", output, time.monotonic() + 0.12,
                              threading.Event(), 10, counts, errors)
        self.assertGreaterEqual(counts["tcp"], 1)
        self.assertEqual(errors, [])
        sample = json.loads(output.getvalue().splitlines()[0])
        self.assertEqual(sample["frame_id"], "base_link")
        self.assertLessEqual(sample["requested_ns"], sample["received_ns"])
        self.assertTrue(all(call.args[0].full_url.endswith(
            "/api/commands/motion/end_effector_pose") for call in urlopen.call_args_list))

    def test_recording_manifest_indexes_both_streams(self):
        with tempfile.TemporaryDirectory() as temp:
            class Socket:
                def close(self):
                    pass

            kinds = iter(("navigation_state", "joint_state", "robot_sensors"))

            def next_frame(*_):
                time.sleep(0.01)
                try:
                    kind = next(kinds)
                except StopIteration:
                    raise TimeoutError("Recording finished")
                return 1, json.dumps({"type": kind, "timestamp_ns": 123}).encode()

            def fake_tcp(_, output, __, ___, ____, counts, _____):
                output.write(recorder.json_line({"requested_ns": 1, "received_ns": 2,
                                                 "frame_id": "base_link",
                                                 "pose": [0, 0, 0, 0, 0, 0, 1]}))
                counts["tcp"] += 1

            with patch.object(recorder, "Dataset", lambda: Dataset(temp)), \
                    patch.object(recorder, "login", return_value="session=test"), \
                    patch.object(recorder, "connect_events", return_value=(Socket(), bytearray())), \
                    patch.object(recorder, "read_event_frame", side_effect=next_frame), \
                    patch.object(recorder, "poll_tcp", side_effect=fake_tcp), \
                    patch.dict(os.environ, {"GALBOT_CONSOLE_CODE": "test"}):
                self.assertEqual(recorder.main(["--session", "drawer_test", "--duration-s", "1"]), 0)

            folder = Path(temp) / "teaching" / "drawer_test" / "continuous"
            manifest = json.loads((folder / "recording.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["counts"], {
                "navigation_state": 1, "joint_state": 1, "robot_sensors": 1, "tcp": 1})
            self.assertEqual(manifest["motion_commands_sent"], [])
            with Dataset(temp) as db:
                self.assertEqual(db.status()["teaching_events"], 1)
            for item in manifest["streams"]:
                self.assertGreater(item["bytes"], 0)
                self.assertEqual(len(item["sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
