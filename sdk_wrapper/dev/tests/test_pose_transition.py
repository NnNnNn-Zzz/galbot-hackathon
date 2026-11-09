import unittest

from sdk_wrapper.dev.algorithms.pose_transition import (
    PoseTransitionController,
    next_pose_transition_step,
    quaternion_slerp,
    rotation_error_deg,
)


class PoseTransitionTest(unittest.TestCase):
    def test_translation_phase_preserves_orientation(self):
        current = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        target = [0.10, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        step = next_pose_transition_step(
            current,
            target,
            max_position_step_m=0.02,
        )

        self.assertEqual(step.kind, "translate")
        self.assertAlmostEqual(step.target_pose[0], 0.02)
        self.assertAlmostEqual(step.target_pose[1], 0.0)
        self.assertEqual(step.target_pose[3:], current[3:])
        self.assertAlmostEqual(step.step_translation_m, 0.02)
        self.assertAlmostEqual(step.step_rotation_deg, 0.0)

    def test_rotation_phase_only_starts_after_position_handoff(self):
        current = [
            0.001,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        target = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0]

        step = next_pose_transition_step(
            current,
            target,
            position_handoff_tolerance_m=0.002,
            max_rotation_step_deg=7.5,
        )

        self.assertEqual(step.kind, "rotate")
        self.assertEqual(step.target_pose[:3], current[:3])
        self.assertAlmostEqual(step.step_rotation_deg, 7.5, places=6)

    def test_slerp_and_error_cover_180_degrees(self):
        midpoint = quaternion_slerp(
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 0.0],
            0.5,
        )
        self.assertAlmostEqual(
            rotation_error_deg(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            ),
            180.0,
        )
        self.assertAlmostEqual(abs(midpoint[2]), 2 ** -0.5)
        self.assertAlmostEqual(abs(midpoint[3]), 2 ** -0.5)

    def test_tolerance_returns_complete(self):
        step = next_pose_transition_step(
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [0.001, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )

        self.assertEqual(step.kind, "complete")

    def test_controller_keeps_translation_orientation_reference_fixed(self):
        controller = PoseTransitionController(
            [0.10, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
            max_position_step_m=0.02,
            max_rotation_step_deg=7.5,
        )
        original_orientation = [0.0, 0.0, 0.0, 1.0]

        first = controller.next(
            [0.0, 0.0, 1.0, *original_orientation]
        )
        drifted = [0.019, 0.0, 1.0, 0.0, 0.0, 0.1, 0.994987]
        second = controller.next(drifted)

        self.assertEqual(first.kind, "translate")
        self.assertEqual(second.kind, "translate")
        self.assertEqual(
            first.target_pose[3:],
            original_orientation,
        )
        self.assertEqual(
            second.target_pose[3:],
            original_orientation,
        )

    def test_controller_locks_position_after_translation_phase(self):
        controller = PoseTransitionController(
            [0.10, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
            position_handoff_tolerance_m=0.002,
            max_position_step_m=0.02,
            max_rotation_step_deg=7.5,
        )

        rotation = controller.next(
            [0.099, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        next_rotation = controller.next(
            [0.1005, 0.001, 1.0, 0.0, 0.0, 0.05, 0.99875]
        )

        self.assertEqual(rotation.kind, "rotate")
        self.assertEqual(
            next_rotation.target_pose[:3],
            [0.099, 0.0, 1.0],
        )


if __name__ == "__main__":
    unittest.main()
