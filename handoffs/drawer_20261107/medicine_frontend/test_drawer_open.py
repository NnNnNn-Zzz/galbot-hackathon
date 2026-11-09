import time
import unittest
import numpy as np
from medicine_frontend.drawer_geometry import (calibrate, grasp_targets, locate,
                                               measured_opening, targets)
from medicine_frontend.drawer_open import run


class Backend:
    def __init__(self):
        self.opening = 0.0
        self.tcp = np.eye(4)
        self.gripped = False
        self.moves = []
        self.stops = 0
        self.slip = False
        self.rail = True
        self.reachable = True

    def observe(self):
        return dict(sample_monotonic_s=time.monotonic(), metric_geometry_verified=True,
                    cabinet_reference_verified=True, body_stationary=True,
                    exclusive_right_arm_control=True, path_clear=True,
                    continuous_guards_ready=True, opening_m=self.opening,
                    opening_uncertainty_m=0.001, available_opening_m=0.35,
                    drawer_id="test", handle_to_base=np.eye(4), body_reference="fixed",
                    target_reached=True, grip_verified=self.gripped,
                    tcp_to_base=self.tcp.copy(), force_within_limits=True,
                    rail_direction_verified=self.rail)

    def plan_right_arm(self, poses, **kwargs):
        self.poses = poses
        return dict(success=self.reachable, right_arm_only=True, full_path_verified=True,
                    continuous_ik=True, collision_checked=True, joint_limits_checked=True,
                    singularity_checked=True)

    def execute_right_arm(self, plan, *, stage, index):
        self.moves.append((stage, index))
        if stage == "pull":
            self.tcp = np.asarray(self.poses["pull"][index])
            if not self.slip:
                self.opening = float(self.tcp[2, 3])
        return dict(success=True, completion="completed")

    def grip_right(self, **kwargs):
        self.gripped = True
        return dict(success=True, completion="completed")

    def stop_right_arm(self):
        self.stops += 1
        return dict(success=True, completion="completed")


class DrawerTests(unittest.TestCase):
    calibration = dict(verified=True, drawer_id="test", tcp_in_handle=np.eye(4))

    def test_metric_plane_with_outliers(self):
        rng = np.random.default_rng(7)
        panel = rng.uniform(-0.2, 0.2, (200, 3))
        panel[:180, 2] = 0
        handle = np.column_stack((np.linspace(-.06, .06, 30), np.zeros(30), np.full(30, .03)))
        result = locate(panel, handle, np.eye(4), [0, 0, 1], unit="m")
        np.testing.assert_allclose(result["normal_base"], [0, 0, 1], atol=1e-8)
        with self.assertRaises(ValueError):
            locate(panel, handle, np.eye(4), [0, 0, 1], unit="sdk_raw_unverified")

    def test_relative_pose_rotates_with_drawer(self):
        h = np.eye(4)
        tcp = np.eye(4)
        tcp[2, 3] = .02
        relative = calibrate(h, tcp)
        h[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        h[:3, 3] = [1, 2, 3]
        path = targets(h, relative)
        np.testing.assert_allclose(np.array(path["pull"][-1])[:3, 3], [1.32, 2, 3])

    def test_midpoint_grasp_geometry(self):
        h = np.eye(4)
        h[:3, 3] = [0.7, -0.2, 0.8]
        path = grasp_targets(h, grasp_offset_m=-0.01)
        np.testing.assert_allclose(np.array(path["grasp"])[:3, 3],
                                   [0.7, -0.2, 0.79], atol=1e-9)
        np.testing.assert_allclose(np.array(path["pregrasp"])[:3, 3],
                                   [0.7, -0.2, 0.85], atol=1e-9)
        np.testing.assert_allclose(np.array(path["pull"][-1])[:3, 3],
                                   [0.7, -0.2, 1.09], atol=1e-9)
        np.testing.assert_allclose(np.array(path["grasp"])[:3, :3], np.eye(3),
                                   atol=1e-9)

    def test_midpoint_grasp_rotation_and_validation(self):
        h = np.eye(4)
        rot_z90 = np.eye(4)
        rot_z90[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        path = grasp_targets(h, gripper_in_handle=rot_z90)
        np.testing.assert_allclose(np.array(path["grasp"])[:3, :3],
                                   rot_z90[:3, :3], atol=1e-9)
        shifted = rot_z90.copy()
        shifted[0, 3] = 0.01
        with self.assertRaises(ValueError):
            grasp_targets(h, gripper_in_handle=shifted)  # rotation-only required
        with self.assertRaises(ValueError):
            grasp_targets(h, grasp_offset_m=-0.06)

    def test_midpoint_mode_success(self):
        backend = Backend()
        calibration = dict(verified=True, drawer_id="test", grasp_mode="midpoint",
                           grasp_offset_m=0.0)
        result = run(backend, calibration)
        self.assertTrue(result["success"], result)
        self.assertAlmostEqual(result["opening_m"], .30)

    def test_unknown_grasp_mode_no_motion(self):
        backend = Backend()
        calibration = dict(verified=True, drawer_id="test", grasp_mode="bogus")
        self.assertFalse(run(backend, calibration)["success"])
        self.assertEqual(backend.moves, [])

    def test_measured_success(self):
        backend = Backend()
        result = run(backend, self.calibration)
        self.assertTrue(result["success"], result)
        self.assertAlmostEqual(result["opening_m"], .30)
        self.assertEqual(backend.stops, 1)

    def test_cabinet_relative_measurement(self):
        cabinet = np.eye(4)
        cabinet[:3, 3] = [1, 2, 3]
        handle = cabinet.copy()
        handle[2, 3] += .30
        result = measured_opening(handle, cabinet, np.eye(4), [0, 0, 1])
        self.assertAlmostEqual(result["opening_m"], .30)
        self.assertAlmostEqual(result["lateral_m"], 0)

    def test_slip_cannot_pass_as_opening(self):
        backend = Backend()
        backend.slip = True
        self.assertFalse(run(backend, self.calibration)["success"])
        self.assertEqual(len(backend.moves), 3)
        self.assertEqual(backend.stops, 1)

    def test_probe_failure_stops_before_long_pull(self):
        backend = Backend()
        backend.rail = False
        self.assertFalse(run(backend, self.calibration)["success"])
        self.assertEqual(len(backend.moves), 3)

    def test_unreachable_no_motion(self):
        backend = Backend()
        backend.reachable = False
        self.assertFalse(run(backend, self.calibration)["success"])
        self.assertEqual(backend.moves, [])

    def test_stale_no_motion(self):
        backend = Backend()
        self.assertFalse(run(backend, self.calibration, clock=lambda: time.monotonic()+5)["success"])
        self.assertEqual(backend.moves, [])

    def test_missing_calibration_no_motion(self):
        backend = Backend()
        self.assertFalse(run(backend, {})["success"])
        self.assertEqual(backend.moves, [])


if __name__ == "__main__":
    unittest.main()
