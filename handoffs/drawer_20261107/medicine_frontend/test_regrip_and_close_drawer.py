"""Offline contract checks for the guarded re-grip and close tool; no robot."""

import copy
from contextlib import closing
import json
import math
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest

from medicine_frontend import drawer_regrip_close as rclose
from medicine_frontend.tools import regrip_and_close_drawer as tool


class FakeIO:
    def __init__(self, recipe, pose=None):
        self.pose = list(pose or [recipe["release_tcp_base_link"][0] - 0.15,
                                  *recipe["release_tcp_base_link"][1:]])
        self.base = dict(recipe["release_base_map"])
        self.arm = dict(recipe["release_right_arm_joints_rad"])
        self.chassis = dict(recipe["release_chassis_joints_rad"])
        self.gripper = {"width": 0.050, "effort": 2.9, "is_moving": False}
        self.active = "right_arm_pvt_ctrl"
        self.commands = []
        self.stops = 0
        self.move_on_offset = True
        self.motion_scale = 1.0
        self.base_drift_per_snapshot = 0.0

    def controller(self):
        return {"group": "right_arm", "active": self.active, "dry_run": False}

    def snapshot(self):
        now = time.time_ns()
        self.base["x"] += self.base_drift_per_snapshot
        joints = [{"name": k, "position": v} for k, v in (self.arm | self.chassis).items()]
        return {"schema": "sdk_console_snapshot/1", "started_ns": now - 100_000_000,
                "finished_ns": now, "motion_commands_sent": [], "errors": [],
                "reads": {
                    "navigation": {"data": {"connected": True, "localized": True,
                                            "pose": copy.deepcopy(self.base)}},
                    "robot": {"data": {"dry_run": False, "joint_state": joints}},
                    "right_end_effector": {"data": {"available": True, "dry_run": False,
                                                     "frame_id": "base_link",
                                                     "pose": list(self.pose)}}},
                "events": [{"event": {"type": "robot_sensors",
                                      "timestamp_ns": now - 5_000_000,
                                      "data": {"grippers": {"right": copy.deepcopy(
                                          self.gripper)}}}}]}

    def grip(self, command):
        self.commands.append(("grip", dict(command)))
        if command["width"] >= 0.040:
            self.gripper.update(width=0.050, effort=2.9)
        else:
            self.gripper.update(width=0.0245, effort=8.2)
        return {"accepted": True, "dry_run": False, "sdk_result": "ControlStatus.SUCCESS"}

    def offset(self, command):
        self.commands.append(("offset", dict(command)))
        if self.move_on_offset:
            for axis, key in enumerate(("dx", "dy", "dz")):
                self.pose[axis] += command[key] * self.motion_scale
        return {"accepted": True, "dry_run": False, "sdk_result": "MotionStatus.SUCCESS"}

    def tcp(self):
        return {"available": True, "dry_run": False, "frame_id": "base_link",
                "pose": list(self.pose)}

    def stop_arm(self):
        self.stops += 1
        return {"accepted": True, "dry_run": False, "sdk_result": "ControlStatus.SUCCESS"}


class FakeGuard:
    def __init__(self, folder, _io, name):
        self.path = folder / f"{name}_wrist_force.ndjson"
        self.motion_started = threading.Event()
        self.trip = threading.Event()
        self.baseline = [0.0, 0.0, 0.0]
        self.samples = 20
        self.max_delta_n = 0.0
        self.reason = None
        self.stop_result = None
        self.stop_error = None

    def start(self):
        self.path.write_text(json.dumps({"force": self.baseline}) + "\n", encoding="utf-8")

    def check(self):
        pass

    def close(self):
        pass


class FailedGuard(FakeGuard):
    def start(self):
        raise RuntimeError("No fresh right wrist force samples")


class RegripCloseTests(unittest.TestCase):
    SESSION = "drawer_regrip_20261107_170000"

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.recipe = rclose.load_recipe()
        self.io = FakeIO(self.recipe)
        self.approach_confirmations = {name: True for name in rclose.APPROACH_CONFIRMATIONS}
        self.grip_confirmations = {name: True for name in rclose.GRIP_CONFIRMATIONS}
        self.push_confirmations = {name: True for name in rclose.PUSH_CONFIRMATIONS}

    def tearDown(self):
        self.temp.cleanup()

    # helpers ------------------------------------------------------------
    def prepare(self, **kwargs):
        return tool.prepare(self.SESSION, self.io, recipe=self.recipe,
                            confirmations=kwargs.pop("confirmations",
                                                     self.approach_confirmations),
                            dataset_root=self.root, **kwargs)

    def approach(self, index, **kwargs):
        kwargs.setdefault("guard_factory", FakeGuard)
        kwargs.setdefault("poll_interval_s", 0)
        kwargs.setdefault("poll_timeout_s", 0.05)
        return tool.approach(self.SESSION, index, self.io, recipe=self.recipe,
                             dataset_root=self.root, **kwargs)

    def drive_to_release(self, limit=20):
        index = 1
        while index <= limit:
            result = self.approach(index)
            self.assertEqual(result["status"], "completed", result.get("error"))
            if result["distance_to_release_m"] <= 0.005:
                return index
            index += 1
        self.fail("approach did not converge")

    def grip(self):
        return tool.grip(self.SESSION, self.io, recipe=self.recipe,
                         confirmations=self.grip_confirmations, dataset_root=self.root)

    def push(self, index, **kwargs):
        kwargs.setdefault("guard_factory", FakeGuard)
        kwargs.setdefault("poll_interval_s", 0)
        kwargs.setdefault("poll_timeout_s", 0.05)
        return tool.push(self.SESSION, index, self.io, recipe=self.recipe,
                         dataset_root=self.root, confirmations=self.push_confirmations,
                         **kwargs)

    def push_all(self, **kwargs):
        statuses = []
        for index in range(1, len(rclose.push_segments(self.recipe)) + 1):
            statuses.append(self.push(index, **kwargs)["status"])
        return statuses

    # tests --------------------------------------------------------------
    def test_prepare_is_read_only_and_reports_state(self):
        result = self.prepare()
        self.assertEqual(result["status"], "ready", result["issues"])
        self.assertEqual(result["motion_commands_sent"], [])
        folder = self.root / "teaching" / self.SESSION
        self.assertTrue((folder / "prepare_state.json").is_file())

    def test_teleop_controller_blocks_before_any_command(self):
        self.io.active = "right_arm_pvt_ctrl_teleop"
        result = self.prepare()
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(any("not active" in issue for issue in result["issues"]))
        self.assertEqual(self.io.commands, [])

    def test_base_beyond_relocalising_envelope_blocks(self):
        self.io.base["x"] += 0.40
        result = self.prepare()
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(any("re-localising only supports" in issue for issue in result["issues"]))
        self.assertEqual(self.io.commands, [])

    def test_approximate_base_return_uses_relocalised_recipe(self):
        self.io.base["x"] += 0.10
        self.io.base["qz"], self.io.base["qw"] = self._yawed(self.io.base, 5.0)
        result = self.prepare(confirmations={**self.approach_confirmations,
                                             "drawer_unmoved": True})
        self.assertEqual(result["status"], "ready", result["issues"])
        self.assertTrue(result["rebase"]["rebased"])
        self.assertAlmostEqual(result["rebase"]["delta"]["shift_m"], 0.10, places=6)
        folder = self.root / "teaching" / self.SESSION
        self.assertTrue((folder / "rebased_recipe.json").is_file())
        active = tool._active_recipe(folder, self.recipe)
        self.assertNotEqual(rclose.release_pose(active), rclose.release_pose(self.recipe))
        self.drive_to_release()
        self.assertLess(abs(self.io.pose[0] - rclose.release_pose(active)[0]), 0.005)
        # FakeIO only translates; on the robot the wrist orientation is set by the
        # operator during manual alignment (or by approach --allow-rotation).
        self.io.pose[3:] = rclose.release_pose(active)[3:]
        self.assertEqual(self.grip()["status"], "completed")
        statuses = self.push_all()
        self.assertEqual(statuses[:-1], ["completed"] * (len(statuses) - 1))
        self.assertIn(statuses[-1], ("completed", "closed_pending_confirmation"))
        closed = rclose.closed_pose(active)
        self.assertLess(math.dist(self.io.pose[:3], closed[:3]), 0.008)
        self.assertTrue((folder / "push_08.json").is_file())

    def test_relocalising_requires_drawer_unmoved_confirmation(self):
        self.io.base["x"] += 0.10
        result = self.prepare()
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(any("drawer_unmoved" in issue for issue in result["issues"]))

    def test_moving_base_blocks_relocalising(self):
        self.io.base_drift_per_snapshot = 0.01
        result = self.prepare()
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(any("stationary base" in issue for issue in result["issues"]))

    @staticmethod
    def _yawed(base, degrees):
        import math as _math
        half = _math.radians(rclose.yaw_deg(base) + degrees) / 2
        return _math.sin(half), _math.cos(half)

    def test_approach_refuses_after_blocked_prepare(self):
        self.io.active = "right_arm_pvt_ctrl_teleop"
        self.prepare()
        with self.assertRaises(ValueError):
            self.approach(1)
        self.assertEqual(self.io.commands, [])

    def test_approach_converges_on_the_release_pose(self):
        self.prepare()
        used = self.drive_to_release()
        self.assertGreaterEqual(used, 3)
        offset_commands = [command for kind, command in self.io.commands if kind == "offset"]
        self.assertEqual(len(offset_commands), used)
        self.assertTrue(all(command["dx"] != 0 or command["dy"] != 0 or command["dz"] != 0
                            for command in offset_commands))
        self.assertLess(abs(self.io.pose[0] - rclose.release_pose(self.recipe)[0]), 0.005)
        self.assertGreaterEqual(self.io.stops, used)

    def test_approach_that_moves_nothing_fails_and_stops(self):
        self.prepare()
        self.io.move_on_offset = False
        result = self.approach(1)
        self.assertEqual(result["status"], "failed")
        self.assertIn("did not reach", result["error"])
        self.assertGreaterEqual(self.io.stops, 1)

    def test_duplicate_approach_step_is_never_resent(self):
        self.prepare()
        self.assertEqual(self.approach(1)["status"], "completed")
        before = len(self.io.commands)
        with self.assertRaises(FileExistsError):
            self.approach(1)
        self.assertEqual(len(self.io.commands), before)

    def test_grip_requires_an_on_site_handle_confirmation(self):
        self.prepare()
        self.drive_to_release()
        with self.assertRaises(ValueError):
            tool.grip(self.SESSION, self.io, recipe=self.recipe, confirmations={},
                      dataset_root=self.root)
        self.assertFalse(any(kind == "grip" for kind, _ in self.io.commands))

    def test_grip_requires_the_release_pose(self):
        self.prepare()
        with self.assertRaises(ValueError):
            self.grip()  # still 150 mm away
        self.assertFalse(any(kind == "grip" for kind, _ in self.io.commands))

    def test_grip_verifies_width_and_effort(self):
        self.prepare()
        self.drive_to_release()
        result = self.grip()
        self.assertEqual(result["status"], "completed", result.get("error"))
        self.assertAlmostEqual(result["after"]["right_gripper"]["width"], 0.0245)

    def test_grip_on_air_fails_the_verification(self):
        self.prepare()
        self.drive_to_release()
        original = self.io.grip

        def clamp_air(command):
            response = original(command)
            self.io.gripper.update(width=0.004, effort=0.4)
            return response

        self.io.grip = clamp_air
        result = self.grip()
        self.assertEqual(result["status"], "failed")
        self.assertIn("handle", result["error"])

    def test_push_requires_a_verified_grip(self):
        self.prepare()
        self.drive_to_release()
        before = sum(kind == "offset" for kind, _ in self.io.commands)
        with self.assertRaises(ValueError):
            self.push(1)
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), before)

    def test_full_flow_returns_the_drawer_to_the_closed_reference(self):
        self.prepare()
        self.drive_to_release()
        self.assertEqual(self.grip()["status"], "completed")
        statuses = self.push_all()
        self.assertEqual(statuses[-1], "closed_pending_confirmation")
        closed = rclose.closed_pose(self.recipe)
        self.assertLess(abs(self.io.pose[0] - closed[0]), 0.001)
        released = tool.release(self.SESSION, self.io, recipe=self.recipe,
                               confirmations={"closed_confirmed": True},
                               dataset_root=self.root)
        self.assertEqual(released["status"], "completed", released.get("error"))
        self.assertAlmostEqual(self.io.gripper["width"], 0.050)
        retracted = tool.retract(self.SESSION, self.io, recipe=self.recipe,
                                 dataset_root=self.root, guard_factory=FakeGuard)
        self.assertEqual(retracted["status"], "completed", retracted.get("error"))
        summary = tool.report(self.SESSION, recipe=self.recipe, dataset_root=self.root,
                              closed_confirmed=True)
        self.assertTrue(summary["drawer_closed_verified"])
        self.assertEqual(summary["push_segments_completed"]
                         + summary["push_segments_pending_confirmation"],
                         len(rclose.push_segments(self.recipe)))
        self.assertEqual(summary["push_segments_failed"], 0)
        with closing(sqlite3.connect(self.root / "catalog.sqlite3")) as db:
            self.assertEqual(db.execute("PRAGMA integrity_check").fetchone()[0], "ok")
            self.assertGreater(
                db.execute("SELECT COUNT(*) FROM teaching_events").fetchone()[0], 10)

    def test_push_that_cannot_reach_the_target_fails_and_stops(self):
        self.prepare()
        self.drive_to_release()
        self.grip()
        self.io.motion_scale = 0.4
        result = self.push(1)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any(text in result["error"]
                            for text in ("blocked or slipping", "did not advance",
                                         "did not reach")))
        self.assertGreaterEqual(self.io.stops, 1)
        self.assertIsNotNone(result.get("stopped_state"))

    def test_push_without_force_stream_fails_closed(self):
        self.prepare()
        self.drive_to_release()
        self.grip()
        offsets_before = sum(kind == "offset" for kind, _ in self.io.commands)
        result = self.push(1, guard_factory=FailedGuard)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["submission"], "not_started")
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), offsets_before)

    def test_release_blocks_until_the_operator_confirms_closure(self):
        self.prepare()
        self.drive_to_release()
        self.grip()
        self.push_all()
        with self.assertRaises(ValueError):
            tool.release(self.SESSION, self.io, recipe=self.recipe, confirmations={},
                         dataset_root=self.root)
        self.assertAlmostEqual(self.io.gripper["width"], 0.0245)

    def test_report_without_confirmation_is_not_a_verified_close(self):
        self.prepare()
        summary = tool.report(self.SESSION, recipe=self.recipe, dataset_root=self.root)
        self.assertFalse(summary["drawer_closed_verified"])
        self.assertFalse(summary["verified_on_robot"])


if __name__ == "__main__":
    unittest.main()
