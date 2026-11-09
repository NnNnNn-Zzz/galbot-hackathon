import unittest
from unittest.mock import patch

from agent import main


def call(experiment, stage, **params):
    return main.execute_decision({
        "action": "call_scene", "target": "scene_drawer_experiment",
        "params": {"experiment": experiment, "stage": stage, **params},
    })


class DrawerSceneIntegrationTest(unittest.TestCase):
    def test_all_stages_through_native_agent_dispatch(self):
        stages = {1: ("vision", "retreat", "preview", "open"),
                  2: ("hover", "dry", "carry", "drop"),
                  3: ("hover", "full")}
        with patch.object(main.baseline, "check_baseline", side_effect=AssertionError("robot read")), \
                patch.object(main.baseline, "reset_to_baseline", side_effect=AssertionError("robot motion")):
            for experiment, phases in stages.items():
                for stage in phases:
                    with self.subTest(experiment=experiment, stage=stage):
                        result = call(experiment, stage)
                        self.assertTrue(result["success"], result)
                        details = result["details"]
                        if (experiment, stage) == (1, "retreat"):
                            self.assertEqual(details["data"]["reached"], ["P0", "P1", "P2", "INITIAL_CARRY"])
                            trace = details["trace"]
                            self.assertEqual(trace[0]["frame_id"], trace[1]["evidence"]["target_frame_id"])
                            samples = trace[1]["evidence"]["pose_samples"]
                            self.assertTrue(all(a["tcp"][2] > b["tcp"][2]
                                                for a, b in zip(samples, samples[1:])))
                        if (experiment, stage) == (3, "full"):
                            self.assertTrue(details["data"]["box_in_bin"])

    def test_invalid_vision_blocks_the_retreat_and_grasp(self):
        for experiment, stage, blocked in ((1, "retreat", "visual_return"), (3, "full", "descend")):
            result = call(experiment, stage, faults=["no_target"])
            self.assertFalse(result["success"])
            details = result["details"]
            self.assertNotIn(blocked, [entry["step"] for entry in details["trace"]])
            if experiment == 1:
                self.assertFalse(details["motion_dispatched"])

    def test_native_mock_scene_rejects_robot_backend(self):
        self.assertFalse(call(1, "retreat", backend="robot")["success"])
