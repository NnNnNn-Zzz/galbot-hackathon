import unittest

from sdk_wrapper.dev.adapters import MockAdapter
from sdk_wrapper.dev.contracts import ErrorCode, TaskStatus, ok
from sdk_wrapper.dev.robot_tools.calibration import CameraCalibrationTool
from sdk_wrapper.dev.robot_tools.perception import PerceptionTool


class CameraCalibrationToolTest(unittest.TestCase):
    def test_matching_official_rgbd_calibration_passes(self):
        tool = CameraCalibrationTool(PerceptionTool(MockAdapter()))

        result = tool.check_right_wrist_rgbd_calibration()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["calibration_match"])
        self.assertTrue(result.data["live_alignment_acceptance_required"])
        self.assertEqual(result.data["translation_delta_m"], 0.0)
        self.assertEqual(result.data["rotation_delta_deg"], 0.0)

    def test_mismatched_depth_extrinsic_is_explicit_failure(self):
        adapter = MockAdapter()
        original = adapter.get_sensor_extrinsic

        def mismatched(sensor, reference_frame="base_link"):
            transform, timestamp = original(sensor, reference_frame)
            if sensor == "RIGHT_ARM_DEPTH_CAMERA":
                transform = list(transform)
                transform[2] += 0.005
            return transform, timestamp

        adapter.get_sensor_extrinsic = mismatched
        result = CameraCalibrationTool(PerceptionTool(adapter)).check_right_wrist_rgbd_calibration()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertFalse(result.data["calibration_match"])
        self.assertFalse(result.data["checks"]["translation_match"])

    def test_motion_camera_frame_matches_official_extrinsic(self):
        tool = CameraCalibrationTool(PerceptionTool(MockAdapter()))

        result = tool.check_right_arm_eye_in_hand()

        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertTrue(result.data["matched"])
        self.assertEqual(result.data["translation_delta_m"], 0.0)
        self.assertEqual(result.data["rotation_delta_deg"], 0.0)

    def test_motion_camera_frame_mismatch_is_explicit_failure(self):
        adapter = MockAdapter()
        original = adapter.get_chain_frame_pose

        def shifted(chain="right_arm", frame_id="Camera", reference_frame="base_link"):
            result = original(chain, frame_id, reference_frame)
            if result.succeeded:
                data = dict(result.data)
                data["pose"] = list(data["pose"])
                data["pose"][0] += 0.01
                return ok(result.message, result.mode, data=data)
            return result

        adapter.get_chain_frame_pose = shifted
        result = CameraCalibrationTool(PerceptionTool(adapter)).check_right_arm_eye_in_hand()

        self.assertEqual(result.status, TaskStatus.FAILED)
        self.assertEqual(result.error_code, ErrorCode.NOT_READY)
        self.assertFalse(result.data["matched"])
        self.assertAlmostEqual(result.data["translation_delta_m"], 0.01, places=6)


if __name__ == "__main__":
    unittest.main()
