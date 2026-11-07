"""Contract checks for the SDK stereo result boundary."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus
from sdk_wrapper.dev.robot_tools.stereo_depth import StereoDepthTool


class FakePerception:
    def __init__(self, depth, *, wait=True):
        self.depth = depth
        self.wait = wait
        self.init_calls = 0
        self.run_calls = 0

    def init(self, modules):
        self.init_calls += 1
        return True

    def run_once(self, module):
        self.run_calls += 1
        return True

    def wait_for_new_result(self, module, timeout_s):
        return self.wait

    def get_latest_result(self, module):
        return True, SimpleNamespace(
            instance_mask=self.depth,
            sensor_name="head_left_camera_color_optical_frame",
            timestamp_ns=123456789,
        )


class StereoDepthToolTest(unittest.TestCase):
    def test_preserves_raw_depth_and_contract(self):
        raw = np.array([[0, 390, 2921], [7669, 4000, 0]], dtype=np.uint16)
        fake = FakePerception(raw)
        with tempfile.TemporaryDirectory() as directory:
            tool = StereoDepthTool(output_dir=Path(directory), perception=fake, module="stereo", warmup_s=0)
            first = tool.capture_stereo_depth()
            second = tool.capture_stereo_depth()
            self.assertEqual(first.status, TaskStatus.SUCCEEDED)
            self.assertEqual(set(first.to_dict()), {"status", "mode", "message", "task_id", "data", "error_code"})
            self.assertEqual(first.data["unit"], "sdk_raw_unverified")
            np.testing.assert_array_equal(np.load(first.data["depth_ref"], allow_pickle=False), raw)
            self.assertTrue(Path(first.data["preview_ref"]).is_file())
            self.assertEqual(first.data["timestamp_ns"], 123456789)
            self.assertEqual(fake.init_calls, 1)
            self.assertEqual(fake.run_calls, 2)
            self.assertNotEqual(first.data["capture_id"], second.data["capture_id"])

    def test_timeout_and_invalid_map_are_explicit(self):
        fake = FakePerception(np.zeros((2, 2), dtype=np.uint16), wait=False)
        with tempfile.TemporaryDirectory() as directory:
            tool = StereoDepthTool(output_dir=Path(directory), perception=fake, module="stereo", warmup_s=0)
            self.assertEqual(tool.capture_stereo_depth(0).error_code, ErrorCode.INVALID_ARGUMENT)
            self.assertEqual(fake.run_calls, 0)
            self.assertEqual(tool.capture_stereo_depth().error_code, ErrorCode.TIMEOUT)
            fake.wait = True
            self.assertEqual(tool.capture_stereo_depth().error_code, ErrorCode.NOT_READY)


if __name__ == "__main__":
    unittest.main()
