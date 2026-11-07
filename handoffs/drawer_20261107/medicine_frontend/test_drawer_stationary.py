"""Offline synthetic checks for stationary drawer perception analysis."""
import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from medicine_frontend import drawer_stationary as ds
from medicine_frontend.tools import analyze_drawer_stationary as analyze
from medicine_frontend.tools import collect_drawer_stationary as collect

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
IDENTITY = np.eye(4)


def plane_depth(tilt_deg=0.0, z0=0.5, shape=(480, 640)):
    """Depth (z-component) of rays hitting the plane n.p = n_z*z0."""
    height, width = shape
    u, v = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
    theta = math.radians(tilt_deg)
    n = np.array([math.sin(theta), 0.0, math.cos(theta)])
    rays = np.stack(((u - 320.0) / 600.0, (v - 240.0) / 600.0,
                     np.ones_like(u)), axis=-1)
    t = (n[2] * z0) / (rays @ n)
    return t * rays[..., 2]


def draw_handle(bgr, center=(320, 240), size=(80, 20)):
    w, h = size
    x, y = center[0] - w // 2, center[1] - h // 2
    cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 0, 220), 4)
    return bgr


def make_frame(tilt_deg=0.0, handles=1):
    bgr = np.full((480, 640, 3), (110, 110, 110), np.uint8)
    if handles >= 1:
        draw_handle(bgr)
    if handles >= 2:
        draw_handle(bgr, center=(480, 360))
    return bgr, plane_depth(tilt_deg)


def rotation_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)],
                     [0, math.sin(a), math.cos(a)]])


def make_extrinsic(rotation=None, translation=(0.0, 0.0, 0.0)):
    out = np.eye(4)
    if rotation is not None:
        out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


def frame_dict(name, position, normal, base_to_map=None, inlier=0.95):
    handle = np.eye(4)
    handle[:3, 3] = position
    entry = {"name": name, "handle_to_base": handle.tolist(),
             "normal_base": list(normal), "plane_inlier_ratio": inlier}
    if base_to_map is not None:
        entry["base_to_map"] = np.asarray(base_to_map, float).tolist()
    return entry


class Pose7Tests(unittest.TestCase):
    def test_identity_quaternion(self):
        np.testing.assert_allclose(ds.pose7_to_matrix([0, 0, 0, 0, 0, 0, 1]),
                                   np.eye(4), atol=1e-9)

    def test_half_turn_about_z(self):
        m = ds.pose7_to_matrix([0.1, 0.2, 0.3, 0, 0, 1, 0])
        np.testing.assert_allclose(m[:3, :3], np.diag([-1, -1, 1]), atol=1e-9)
        np.testing.assert_allclose(m[:3, 3], [0.1, 0.2, 0.3], atol=1e-9)

    def test_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            ds.pose7_to_matrix([0, 0, 0, 0, 0, 0, 2])  # non-unit quaternion
        with self.assertRaises(ValueError):
            ds.pose7_to_matrix([0, 0, 0, 0, 0, 1])


class PanelMaskTests(unittest.TestCase):
    def test_excludes_handle_and_clips_to_image(self):
        mask = ds.panel_mask((480, 640), [280, 230, 80, 20])
        self.assertFalse(mask[240, 320])        # handle centre excluded
        self.assertTrue(mask[240, 170])         # panel ROI inside
        edge = ds.panel_mask((480, 640), [0, 0, 80, 20])
        self.assertTrue(edge.any())

    def test_rejects_invalid_arguments(self):
        with self.assertRaises(ValueError):
            ds.panel_mask((480, 640), [280, 230, 80, 20], expand=1.0)
        with self.assertRaises(ValueError):
            ds.panel_mask((480, 640), [600, 470, 80, 20])


class AnalyzeFrameTests(unittest.TestCase):
    def test_recovers_geometry_fronto_parallel(self):
        bgr, depth = make_frame()
        result = ds.analyze_frame(bgr, depth, K, IDENTITY)
        position = np.asarray(result["handle_to_base"])[:3, 3]
        np.testing.assert_allclose(position, [0.0, 0.0, 0.5], atol=2e-3)
        np.testing.assert_allclose(result["normal_base"], [0, 0, -1], atol=1e-2)
        self.assertGreaterEqual(result["plane_inlier_ratio"], 0.9)
        self.assertGreater(result["handle_pixels"], 100)
        self.assertGreater(result["panel_pixels"], 1000)

    def test_geometry_follows_camera_extrinsic(self):
        bgr, depth = make_frame()
        rotation = rotation_x(-90)
        extrinsic = make_extrinsic(rotation, (0.3, -0.2, 0.8))
        result = ds.analyze_frame(bgr, depth, K, extrinsic)
        position = np.asarray(result["handle_to_base"])[:3, 3]
        np.testing.assert_allclose(position, rotation @ [0, 0, 0.5] + [0.3, -0.2, 0.8],
                                   atol=3e-3)
        np.testing.assert_allclose(result["normal_base"], rotation @ [0, 0, -1],
                                   atol=1e-2)

    def test_missing_or_ambiguous_handle_rejected(self):
        bgr, depth = make_frame(handles=0)
        with self.assertRaises(ValueError):
            ds.analyze_frame(bgr, depth, K, IDENTITY)
        bgr2, _ = make_frame(handles=2)
        with self.assertRaises(ValueError):
            ds.analyze_frame(bgr2, depth, K, IDENTITY)

    def test_oblique_panel_rejected(self):
        bgr, depth = make_frame(tilt_deg=89.0)
        with self.assertRaises(ValueError) as caught:
            ds.analyze_frame(bgr, depth, K, IDENTITY)
        self.assertIn("obliquely", str(caught.exception))

    def test_invalid_depth_rejected(self):
        bgr, _ = make_frame()
        with self.assertRaises(ValueError):
            ds.analyze_frame(bgr, np.zeros((480, 640)), K, IDENTITY)
        with self.assertRaises(ValueError):
            ds.analyze_frame(bgr, np.full((240, 320), 0.5), K, IDENTITY)


class SummarizeTests(unittest.TestCase):
    def test_consistent_positions_pass_in_map_frame(self):
        # Robot moved 10 cm in +X between captures; same physical drawer must
        # land on one map position once base_to_map is applied.
        f1 = frame_dict("p1", [0.70, -0.20, 0.80], [0, 0, -1], np.eye(4))
        shift = make_extrinsic(translation=(0.10, 0.0, 0.0))
        f2 = frame_dict("p2", [0.60, -0.20, 0.80], [0, 0, -1], shift)
        result = ds.summarize([f1, f2])
        self.assertTrue(result["passed"], result["failures"])
        self.assertEqual(result["comparison_frame"], "map")
        self.assertAlmostEqual(result["position_spread_m"], 0.0, places=6)

    def test_position_drift_detected(self):
        f1 = frame_dict("p1", [0.70, -0.20, 0.80], [0, 0, -1], np.eye(4))
        shift = make_extrinsic(translation=(0.10, 0.0, 0.0))
        f2 = frame_dict("p2", [0.63, -0.20, 0.80], [0, 0, -1], shift)
        result = ds.summarize([f1, f2])
        self.assertFalse(result["passed"])
        self.assertAlmostEqual(result["position_spread_m"], 0.03, places=6)
        self.assertTrue(any("position" in f for f in result["failures"]))

    def test_normal_drift_detected(self):
        a = math.radians(5)
        f1 = frame_dict("p1", [0.70, -0.20, 0.80], [0, 0, -1], np.eye(4))
        f2 = frame_dict("p2", [0.70, -0.20, 0.80], [math.sin(a), 0, -math.cos(a)],
                        np.eye(4))
        result = ds.summarize([f1, f2])
        self.assertFalse(result["passed"])
        self.assertAlmostEqual(result["normal_spread_deg"], 5.0, places=3)

    def test_missing_map_pose_fails_closed_unless_repeatability(self):
        f1 = frame_dict("p1", [0.70, -0.20, 0.80], [0, 0, -1])
        f2 = frame_dict("p2", [0.70, -0.20, 0.80], [0, 0, -1])
        blocked = ds.summarize([f1, f2])
        self.assertFalse(blocked["passed"])
        self.assertTrue(any("map_transform_unavailable" in f
                            for f in blocked["failures"]))
        repeat = ds.summarize([f1, f2], allow_base_frame=True)
        self.assertTrue(repeat["passed"], repeat["failures"])

    def test_rejects_invalid_input(self):
        with self.assertRaises(ValueError):
            ds.summarize([frame_dict("p1", [0, 0, 0], [0, 0, -1])])
        with self.assertRaises(ValueError):
            ds.summarize([frame_dict("a", [0, 0, 0], [0, 0, -1]),
                          frame_dict("b", [0, 0, 0], [0, 0, -1])],
                         position_tolerance_m=0.5)


class FakeRobot:
    def __init__(self, rgb_ok=True):
        self.rgb_ok = rgb_ok

    def get_rgb_data(self, sensor, fmt, flag):
        if not self.rgb_ok:
            return {"data": b""}
        ok, encoded = cv2.imencode(".jpg", np.zeros((4, 4, 3), np.uint8))
        assert ok
        return {"data": encoded.tobytes(), "timestamp_ns": 123}

    def get_depth_data(self, sensor):
        return {"format": "raw_uint16_le", "depth_scale": 1000.0,
                "height": 2, "width": 2, "timestamp_ns": 124,
                "data": np.full((2, 2), 500, np.uint16).tobytes()}

    def get_camera_intrinsic(self, sensor):
        return {"K": [600, 0, 320, 0, 600, 240, 0, 0, 1], "D": [0, 0, 0, 0, 0]}

    def get_sensor_extrinsic(self, sensor, frame):
        return ([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0], 125)

    def get_synced_observation(self, sensors, with_joint_state=True):
        return {"joints": {"right_arm": [0.0] * 7}}


class CollectTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_capture_once_saves_hashed_frames_without_motion(self):
        folder = self.root / "capture_01"
        meta = collect.capture_once(FakeRobot(), folder, sensors=("RGB", "D", "JPEG"),
                                    settle_s=0)
        self.assertEqual(meta["motion_commands_sent"], [])
        rgb = (folder / "rgb.jpg").read_bytes()
        depth = (folder / "depth.bin").read_bytes()
        self.assertEqual(meta["rgb"]["sha256"], collect.sha256_bytes(rgb))
        self.assertEqual(len(depth), 8)
        self.assertEqual(meta["depth"]["format"], "raw_uint16_le")
        self.assertEqual(meta["extrinsics"]["reference_frame"], "base_link")
        self.assertIsNone(meta["base_pose_map"]["data"])
        saved = json.loads((folder / "capture.json").read_text(encoding="utf-8"))
        self.assertEqual(saved["schema"], "drawer_stationary_capture/1")
        with self.assertRaises(FileExistsError):
            collect.capture_once(FakeRobot(), folder, sensors=("RGB", "D", "JPEG"),
                                 settle_s=0)

    def test_empty_rgb_fails_without_partial_folder(self):
        with self.assertRaises(RuntimeError):
            collect.capture_once(FakeRobot(rgb_ok=False), self.root / "capture_02",
                                 sensors=("RGB", "D", "JPEG"), settle_s=0)
        self.assertFalse((self.root / "capture_02").exists())

    def test_manifest_lists_files_and_no_motion(self):
        session = self.root / "drawer_stationary_test"
        meta = collect.capture_once(FakeRobot(), session / "capture_01",
                                    sensors=("RGB", "D", "JPEG"), settle_s=0)
        manifest = collect.finish_manifest("drawer_stationary_test", session,
                                           [meta], started_ns=1, errors=[])
        self.assertEqual(manifest["motion_commands_sent"], [])
        self.assertEqual(manifest["captures"], 1)
        names = {entry["file"] for entry in manifest["files"]}
        self.assertEqual(names, {"capture_01/rgb.jpg", "capture_01/depth.bin",
                                 "capture_01/capture.json"})


def write_capture(folder, *, tamper=False, extrinsics=True):
    """Synthetic capture folder in the collect_drawer_stationary format."""
    import hashlib
    folder = Path(folder)
    folder.mkdir(parents=True)
    bgr, depth = make_frame()
    ok, encoded = cv2.imencode(".jpg", bgr)
    assert ok
    rgb_raw = encoded.tobytes()
    depth_raw = np.round(depth * 1000.0).astype("<u2").tobytes()
    (folder / "rgb.jpg").write_bytes(rgb_raw)
    (folder / "depth.bin").write_bytes(depth_raw)
    if tamper:
        (folder / "rgb.jpg").write_bytes(b"tampered")
    meta = {
        "schema": "drawer_stationary_capture/1",
        "rgb": {"file": "rgb.jpg", "bytes": len(rgb_raw),
                "sha256": hashlib.sha256(rgb_raw).hexdigest()},
        "depth": {"file": "depth.bin", "bytes": len(depth_raw),
                  "sha256": hashlib.sha256(depth_raw).hexdigest(),
                  "format": "raw_uint16_le", "depth_scale": 1000.0,
                  "height": 480, "width": 640},
        "intrinsics": {"rgb": {"K": K.flatten().tolist()},
                       "depth": {"K": K.flatten().tolist()}},
        "extrinsics": {"reference_frame": "base_link",
                       "depth": {"pose": [0, 0, 0, 0, 0, 0, 1] if extrinsics else None,
                                 "timestamp": 1},
                       "rgb": {"pose": [0, 0, 0, 0, 0, 0, 1] if extrinsics else None,
                               "timestamp": 1}},
    }
    (folder / "capture.json").write_text(json.dumps(meta), encoding="utf-8")
    return folder


class AnalyzeSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.session = Path(self.temp.name) / "drawer_stationary_test"

    def tearDown(self):
        self.temp.cleanup()

    def test_two_identical_captures_pass_repeatability(self):
        write_capture(self.session / "capture_01")
        write_capture(self.session / "capture_02")
        report = analyze.analyze_session(self.session, allow_base_frame=True)
        self.assertEqual(report["errors"], [])
        self.assertTrue(report["summary"]["passed"], report["summary"]["failures"])
        self.assertAlmostEqual(report["summary"]["position_spread_m"], 0.0, places=6)
        normal = report["per_capture"][0]["normal_base"]
        np.testing.assert_allclose(normal, [0, 0, -1], atol=1e-2)
        self.assertTrue((self.session / "analysis.json").is_file())

    def test_missing_map_pose_fails_closed_by_default(self):
        write_capture(self.session / "capture_01")
        write_capture(self.session / "capture_02")
        report = analyze.analyze_session(self.session)
        self.assertFalse(report["summary"]["passed"])
        self.assertTrue(any("map_transform_unavailable" in f
                            for f in report["summary"]["failures"]))

    def test_tampered_file_reported_not_analyzed(self):
        write_capture(self.session / "capture_01")
        write_capture(self.session / "capture_02", tamper=True)
        report = analyze.analyze_session(self.session, allow_base_frame=True)
        self.assertEqual(report["captures_analyzed"], 1)
        self.assertEqual(len(report["errors"]), 1)
        self.assertIn("hash_mismatch", report["errors"][0]["error"])

    def test_missing_extrinsics_blocks_cross_position_summary(self):
        write_capture(self.session / "capture_01", extrinsics=False)
        write_capture(self.session / "capture_02", extrinsics=False)
        report = analyze.analyze_session(self.session, allow_base_frame=True)
        self.assertIsNone(report["summary"])
        self.assertIn("extrinsics_unavailable", report["blocked"])
        self.assertEqual(report["captures_analyzed"], 2)
        self.assertTrue(report["per_capture"][0]["camera_frame_only"])

    def test_empty_session_rejected(self):
        self.session.mkdir(parents=True)
        with self.assertRaises(ValueError):
            analyze.analyze_session(self.session)


class FakeCam:
    """Minimal camera-bridge stand-in: only /frame and /depth are used."""

    def __init__(self):
        self.calls = []

    def get(self, path, timeout=10.0):
        import time
        self.calls.append(path)
        now = time.time_ns()
        if path.startswith("/frame/"):
            ok, encoded = cv2.imencode(".jpg", np.zeros((4, 4, 3), np.uint8))
            assert ok
            return 200, {"X-Frame-Captured-Ns": str(now),
                         "X-Frame-Timestamp-Ns": str(now),
                         "X-Frame-Camera": "RIGHT_ARM_CAMERA"}, encoded.tobytes()
        if path.startswith("/depth/"):
            body = np.full((4, 4), 500, np.uint16).tobytes()
            return 200, {"X-Depth-Width": "4", "X-Depth-Height": "4",
                         "X-Depth-Scale": "10000.0", "X-Depth-Format": "16UC1",
                         "X-Depth-Camera": "RIGHT_ARM_DEPTH_CAMERA",
                         "X-Frame-Captured-Ns": str(now),
                         "X-Frame-Timestamp-Ns": str(now)}, body
        raise AssertionError(path)


class BridgeCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.info = {"intrinsics": {"wrist_right": {"K": K.flatten().tolist(),
                                                    "width": 640, "height": 360}}}

    def tearDown(self):
        self.temp.cleanup()

    def test_bridge_capture_records_missing_extrinsics_honestly(self):
        cam = FakeCam()
        folder = self.root / "capture_01"
        meta = collect.capture_once_bridge(cam, folder, source="wrist_right",
                                           info=self.info, settle_s=0)
        self.assertEqual(meta["transport"], "camera_bridge")
        self.assertEqual(meta["motion_commands_sent"], [])
        self.assertFalse(meta["extrinsics"]["available"])
        self.assertIsNone(meta["joint_state"]["data"])
        self.assertEqual(meta["depth"]["format"], "raw_uint16_le")
        self.assertEqual(meta["depth"]["depth_scale"], 10000.0)
        self.assertEqual(meta["depth"]["sha256"],
                         collect.sha256_bytes((folder / "depth.bin").read_bytes()))
        self.assertIn("no_extrinsics", meta["extrinsics"]["reason"])

    def test_bridge_capture_refuses_existing_folder(self):
        folder = self.root / "capture_01"
        folder.mkdir()
        with self.assertRaises(FileExistsError):
            collect.capture_once_bridge(FakeCam(), folder, source="wrist_right",
                                        info=self.info, settle_s=0)


if __name__ == "__main__":
    unittest.main()
