import unittest

import numpy as np

from sdk_wrapper.dev.algorithms.observation_pose import (
    end_effector_pose_for_camera,
    plan_end_effector_grasp_pose,
    plan_map_observation_pose,
    plan_wrist_camera_pose,
)


class ObservationPoseTest(unittest.TestCase):
    def test_map_observation_stays_on_robot_target_line(self):
        result = plan_map_observation_pose(
            [1.0, 0.0, 0.4],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            observation_distance_m=0.8,
        )

        self.assertTrue(result["safe_to_submit"])
        np.testing.assert_allclose(
            result["observation_pose_map"][:3],
            [0.2, 0.0, 0.0],
            atol=1e-6,
        )
        self.assertAlmostEqual(result["facing_yaw_rad"], 0.0)

    def test_wrist_camera_is_above_and_faces_down_normal(self):
        result = plan_wrist_camera_pose(
            [1.0, 0.2, 0.3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            camera_standoff_m=0.3,
        )

        self.assertTrue(result["safe_to_plan"])
        np.testing.assert_allclose(
            result["camera_position_base"],
            [1.0, 0.2, 0.6],
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result["camera_forward_axis_base"],
            [0.0, 0.0, -1.0],
            atol=1e-6,
        )

    def test_desired_camera_pose_transfers_to_end_effector(self):
        result = end_effector_pose_for_camera(
            [1.0, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0],
        )

        np.testing.assert_allclose(
            result["end_effector_pose_base"][:3],
            [1.0, 0.0, 0.7],
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result["end_effector_pose_base"][3:],
            [1.0, 0.0, 0.0, 0.0],
            atol=1e-6,
        )

    def test_charger_grasp_pose_is_above_surface_and_points_down(self):
        result = plan_end_effector_grasp_pose(
            [0.5, -0.1, 0.7],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            standoff_m=0.12,
            current_end_effector_pose_base=[
                0.45, -0.2, 0.85, 0.0, 0.0, 0.0, 1.0
            ],
        )

        self.assertTrue(result["safe_to_plan"])
        np.testing.assert_allclose(
            result["pre_grasp_position_m"],
            [0.5, -0.1, 0.82],
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result["approach_axis_base"],
            [0.0, 0.0, -1.0],
            atol=1e-8,
        )
        rotation = np.column_stack(
            [
                result["approach_axis_base"],
                result["jaw_closing_axis_base"],
                result["gripper_long_axis_base"],
            ]
        )
        self.assertAlmostEqual(np.linalg.det(rotation), 1.0, places=7)

    def test_equivalent_long_axis_sign_follows_nearest_current_yaw(self):
        result = plan_end_effector_grasp_pose(
            [0.5, -0.1, 0.7],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            standoff_m=0.12,
            current_end_effector_pose_base=[
                0.45, -0.2, 0.85, -0.70710678, 0.0, 0.70710678, 0.0
            ],
        )

        self.assertAlmostEqual(result["selected_long_axis_sign"], -1.0)
        self.assertAlmostEqual(result["rotation_from_current_deg"], 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
