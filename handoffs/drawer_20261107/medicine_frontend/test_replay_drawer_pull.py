"""Offline contract checks for guarded drawer replay; no robot connection."""

import copy
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest

from medicine_frontend.tools import replay_drawer_pull as replay


class FakeIO:
    def __init__(self):
        recipe = replay._load_recipe()
        self.pose = list(recipe["pre_grasp_tcp_base_link"])
        self.base = dict(recipe["alignment_base_map"])
        self.arm = dict(recipe["alignment_right_arm_joints_rad"])
        self.chassis = {"chassis_joint1": 0.0, "chassis_joint2": 0.0}
        self.gripper = {"width": 0.050, "effort": 2.9, "is_moving": False}
        self.active = "right_arm_pvt_ctrl"
        self.commands = []
        self.stops = 0
        self.move_on_offset = True
        self.drift_on_step = None

    def controller(self):
        return {"group": "right_arm", "active": self.active, "dry_run": False}

    def snapshot(self):
        now = time.time_ns()
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
                "events": [{"event": {"type": "robot_sensors", "timestamp_ns": now - 5_000_000,
                                      "data": {"grippers": {"right": copy.deepcopy(self.gripper)}}}}]}

    def grip(self, command):
        self.commands.append(("grip", dict(command)))
        self.gripper.update(width=0.0245, effort=8.2)
        return {"accepted": True, "dry_run": False, "sdk_result": "ControlStatus.SUCCESS"}

    def offset(self, command):
        self.commands.append(("offset", dict(command)))
        if self.move_on_offset:
            for axis, key in enumerate(("dx", "dy", "dz")):
                self.pose[axis] += command[key]
            if self.drift_on_step == sum(kind == "offset" for kind, _ in self.commands):
                self.pose[2] -= 0.008
        return {"accepted": True, "dry_run": False, "sdk_result": "MotionStatus.SUCCESS"}

    def tcp(self):
        return {"available": True, "dry_run": False, "frame_id": "base_link",
                "pose": list(self.pose)}

    def stop_arm(self):
        self.stops += 1
        return {"accepted": True, "dry_run": False, "sdk_result": "ControlStatus.SUCCESS"}


class FakeForceGuard:
    def __init__(self, folder, _io, step):
        self.path = folder / f"step_{step:02d}_wrist_force.ndjson"
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


class FailedForceGuard(FakeForceGuard):
    def start(self):
        raise RuntimeError("No fresh right wrist force samples")


class FreshnessTests(unittest.TestCase):
    """`_extract` must accept the live console's event ordering (18:10 finding)."""

    def setUp(self):
        self.snapshot = FakeIO().snapshot()

    def test_sensor_event_newer_than_snapshot_finish_is_healthy(self):
        self.snapshot["events"][0]["event"]["timestamp_ns"] = \
            self.snapshot["finished_ns"] + 250_000_000
        self.assertIn("right_gripper", replay._extract(self.snapshot))

    def test_old_sensor_event_still_blocks(self):
        self.snapshot["events"][0]["event"]["timestamp_ns"] = \
            self.snapshot["finished_ns"] - 4_000_000_000
        with self.assertRaises(ValueError):
            replay._extract(self.snapshot)

    def test_future_clock_jump_still_blocks(self):
        self.snapshot["events"][0]["event"]["timestamp_ns"] = \
            self.snapshot["finished_ns"] + 4_000_000_000
        with self.assertRaises(ValueError):
            replay._extract(self.snapshot)


class ReplayTests(unittest.TestCase):
    SESSION = "drawer_replay_20261107_140000"

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.io = FakeIO()
        self.confirmations = {name: True for name in replay.REQUIRED_CONFIRMATIONS}

    def tearDown(self):
        self.temp.cleanup()

    def prepared_and_gripped(self):
        preparation = replay.prepare(self.SESSION, self.io,
                                     confirmations=self.confirmations, dataset_root=self.root)
        self.assertEqual(preparation["status"], "ready")
        closure = replay.grip(self.SESSION, self.io, dataset_root=self.root,
                              require_stream=False)
        self.assertEqual(closure["status"], "completed")

    def pull_step(self, index):
        return replay.step(self.SESSION, index, self.io, dataset_root=self.root,
                           require_stream=False, guard_factory=FakeForceGuard,
                           poll_interval_s=0, poll_timeout_s=0.05)

    def test_plan_has_no_io(self):
        value = replay.plan()
        self.assertTrue(value["dry_run"])
        self.assertEqual(value["targets_cumulative_x_m"], list(replay.TARGETS_M))

    def test_prepare_can_join_recorders_already_running(self):
        teaching = self.root / "teaching" / self.SESSION
        (teaching / "continuous").mkdir(parents=True)
        (teaching / "camera_stream").mkdir()
        result = replay.prepare(self.SESSION, self.io,
                                confirmations=self.confirmations, dataset_root=self.root)
        self.assertEqual(result["status"], "ready")

    def test_prepare_rejects_teleop_controller(self):
        self.io.active = "right_arm_pvt_ctrl_teleop"
        result = replay.prepare(self.SESSION, self.io,
                                confirmations=self.confirmations, dataset_root=self.root)
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(self.io.commands, [])

    def test_full_eight_step_replay_and_db_index(self):
        self.prepared_and_gripped()
        for index in range(1, 9):
            if index == 7:
                value = replay.rebase_25cm(self.SESSION, self.io,
                    operator_extension_confirmed=True, dataset_root=self.root,
                    require_stream=False)
                self.assertEqual(value["status"], "ready")
            result = self.pull_step(index)
            self.assertEqual(result["status"],
                             "paused_at_25cm" if index == 6 else "completed")
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), 8)
        self.assertAlmostEqual(self.io.pose[0],
                               replay._load_recipe()["pre_grasp_tcp_base_link"][0] - 0.30)
        self.assertEqual(self.io.stops, 2)
        with closing(sqlite3.connect(self.root / "catalog.sqlite3")) as db:
            self.assertEqual(db.execute("SELECT COUNT(*) FROM teaching_events").fetchone()[0], 11)
            self.assertEqual(db.execute("PRAGMA integrity_check").fetchone()[0], "ok")

    def test_controller_teleop_blocks_before_grip(self):
        self.prepared_and_gripped()
        self.io.active = "right_arm_pvt_ctrl_teleop"
        with self.assertRaises(ValueError):
            self.pull_step(1)
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), 0)

    def test_accepted_without_tcp_motion_stops_and_refuses_retry(self):
        self.prepared_and_gripped()
        self.io.move_on_offset = False
        failed = self.pull_step(1)
        self.assertEqual(failed["status"], "failed")
        self.assertIn("did not reach", failed["error"])
        self.assertGreaterEqual(self.io.stops, 1)
        with self.assertRaises(FileExistsError):
            self.pull_step(1)
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), 1)

    def test_missing_force_monitor_prevents_command(self):
        self.prepared_and_gripped()
        failed = replay.step(self.SESSION, 1, self.io, dataset_root=self.root,
                             require_stream=False, guard_factory=FailedForceGuard)
        self.assertEqual(failed["status"], "failed")
        self.assertEqual(failed["submission"], "not_started")
        self.assertEqual(sum(kind == "offset" for kind, _ in self.io.commands), 0)

    def test_25cm_drift_requires_explicit_rebase(self):
        self.prepared_and_gripped()
        self.io.drift_on_step = 6
        for index in range(1, 7):
            result = self.pull_step(index)
        self.assertEqual(result["status"], "paused_axis_drift")
        with self.assertRaises(FileNotFoundError):
            self.pull_step(7)
        with self.assertRaises(ValueError):
            replay.rebase_25cm(self.SESSION, self.io,
                               operator_extension_confirmed=False,
                               dataset_root=self.root, require_stream=False)
        rebased = replay.rebase_25cm(self.SESSION, self.io,
                                    operator_extension_confirmed=True,
                                    dataset_root=self.root, require_stream=False)
        self.assertEqual(rebased["status"], "ready")
        self.assertEqual(self.pull_step(7)["status"], "completed")


if __name__ == "__main__":
    unittest.main()
