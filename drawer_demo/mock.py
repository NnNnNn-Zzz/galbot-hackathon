"""Synthetic fixtures only. Coordinates, radii and tolerances are invented.

Nothing here is a measurement. Mock numbers exist to exercise the state machine,
never to authorize a robot motion.
"""
from __future__ import annotations

import math
from copy import deepcopy

from .contracts import Frame, Receipt


def example_recipe(*, fixed=False):
    recipe = {
        "schema": "drawer_demo_recipe/2", "mode": "mock", "commissioned": True,
        "setup_id": "SYNTHETIC_ONLY", "model_id": "synthetic_top_mask",
        "arm": "right_arm", "reference_frame": "base_link", "tool_id": "mock_tool",
        "tcp_frame": "mock_tcp", "fixed_camera": True,
        "observation_source": "synthetic highest validated release-rise waypoint",
        "box_size_m": [0.085, 0.075, 0.075], "closing_axis_yaw_rad": 0.0,
        "box_center_to_tcp_xy_m": [0.0, 0.0],
        "tool_orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
        "above_tcp_z_m": 0.90, "grasp_tcp_z_m": 0.68, "short_lift_m": 0.02,
        "gripper_open_m": 0.095, "gripper_close_m": 0.065, "drawer_opening_m": 0.27,
        "calibration": {
            "id": "synthetic_cal", "verified": True, "camera_id": "synthetic_fixed_head",
            "evidence_ref": "synthetic_fixture_not_measurement",
            "pixel_convention": "original_distortion_compensated", "image_size_wh": [640, 480],
            "H_pixel_to_base_xy": [0.001, 0, 0.18, 0, -0.001, 0.24, 0, 0, 1],
            "opening_roi_xy_m": [0.36, -0.15, 0.64, 0.15],
            "nominal_center_xy_m": [0.50, 0.0], "box_top_z_m": 0.70,
        },
        "limits": {"min_confidence": 0.85, "min_rectangularity": 0.90,
                   "max_frame_age_s": 1.0, "max_receipt_age_s": 0.5,
                   "size_tolerance_m": 0.005, "max_axis_error_rad": 0.08,
                   "max_center_shift_m": 0.025, "max_recheck_delta_m": 0.003,
                   "max_recheck_axis_rad": 0.04, "perception_timeout_s": 3.0,
                   "action_timeout_s": 15.0, "max_linear_speed_m_s": 0.02,
                   "max_joint_speed_rad_s": 0.1, "gripper_speed_m_s": 0.01,
                   "gripper_effort_n": 5.0, "max_base_yaw_speed_rad_s": 0.15,
                   "stroke_tolerance_m": 0.02},
        "base_motion": {
            "allowed": True, "release_mode": "drop",
            # Negative is a right turn in the SDK convention.
            "drop_yaw_rad": -math.pi/6, "max_segment_rad": math.pi/12,
            "yaw_tolerance_rad": math.radians(2.0),
            "return_shift_tolerance_m": 0.030,
            "return_yaw_tolerance_rad": math.radians(2.0),
            "drop_radius_m": 0.62, "drop_offset_tolerance_m": 0.060,
            "rotate_segment_timeout_s": 12.0, "push_timeout_s": 45.0,
        },
        "waypoint_refs": {p: "SYNTHETIC/"+p for p in
                          ("HOME", "HANDLE_CLEAR", "OBSERVE_HIGH", "START", "DROP",
                           "HANDLE_RELEASE")},
        "route_refs": {p: "SYNTHETIC/"+p for p in
                       ("drawer_open_release", "rise_to_observe", "retreat_to_start",
                        "rotate_to_drop", "rotate_back", "regrip_handle",
                        "push_drawer_closed", "close_to_home")},
    }
    if fixed:
        recipe["flow"] = {"rotate_for_drop": False, "close_drawer": False}
        recipe["base_motion"]["allowed"] = False
        for route in ("carry_to_drop", "drop_to_home"):
            recipe["route_refs"][route] = "SYNTHETIC/" + route
    return recipe


def site_template():
    """Keep known box/opening dimensions; null all invented calibration/control values."""
    r = deepcopy(example_recipe(fixed=True))
    r.update(mode="shared_agent", commissioned=False, setup_id=None, model_id=None,
             source_kind="site", evidence_ref=None,
             tool_id=None, tcp_frame=None, observation_source=None,
             tool_orientation_xyzw=None, closing_axis_yaw_rad=None,
             box_center_to_tcp_xy_m=None,
             above_tcp_z_m=None, grasp_tcp_z_m=None, short_lift_m=None,
             gripper_open_m=None, gripper_close_m=None)
    r["calibration"] = {k: (False if k == "verified" else None) for k in r["calibration"]}
    r["calibration"]["pixel_convention"] = "original_distortion_compensated"
    r["drawer_open"] = {"mode": "taught", "camera_id": None, "calibration_id": None,
                        "use_view_route": False}
    r["base_motion"] = dict.fromkeys(r["base_motion"])
    r["base_motion"]["allowed"] = False
    r["base_motion"]["release_mode"] = "drop"
    r["route_refs"]["release_handle_clear"] = None
    r["route_refs"]["wrist_view"] = None
    r["limits"]["max_contact_force_n"] = None
    r["limits"]["joint_tolerance_rad"] = None
    r["limits"]["tcp_angle_tolerance_rad"] = None
    for group in ("limits", "waypoint_refs", "route_refs"):
        r[group] = dict.fromkeys(r[group])
    r["waypoint_refs"]["OBSERVE_HIGH"] = "P0"
    r["observation_source"] = "P0 / grasp_capture_20261107_194545"
    return r


class MockIO:
    backend = "mock"

    def __init__(self, recipe, fault=""):
        self.recipe, self.fault = recipe, fault
        self.clock, self.frame_count = 1_000_000_000, 0
        self.actions, self.stops = [], []
        self.held = self.lifted = self.at_start = self.at_drop = False
        self.released = self.in_bin = self.regripped = self.closed = False
        self.base_yaw_rad, self.base_shift_m = 0.0, 0.0

    def now_ns(self):
        self.clock += 1_000_000
        return self.clock

    def preflight(self, recipe):
        return dict.fromkeys(("shared_control_ready", "stop_channel_ready", "telemetry_fresh",
                              "legs_head_locked_and_base_parked", "tool_tcp_verified",
                              "camera_calibration_verified",
                              "routes_verified", "observe_high_is_verified_max_z", "bin_fixed_verified",
                              "chassis_pose_control_owned", "base_yaw_free_path_verified"), True)

    def capture(self, viewpoint, timeout_s):
        self.frame_count += 1
        cal = self.recipe["calibration"]
        return Frame(f"synthetic_{self.frame_count}", cal["camera_id"], self.now_ns(),
                     tuple(cal["image_size_wh"]), f"synthetic://{viewpoint}", cal["id"],
                     self.recipe["setup_id"])

    def segment(self, frame, timeout_s):
        poly = [[282.5, 197.5], [357.5, 197.5], [357.5, 282.5], [282.5, 282.5]]
        if self.fault == "shift" and self.frame_count == 2:
            poly = [[x+10, y] for x, y in poly]
        det = {"class_name": "medicine_box_top", "confidence": 0.98, "mask_polygon_xy_px": poly}
        result = {"schema": "drawer_box_seg/1", "frame_id": frame.frame_id,
                  "camera_id": frame.camera_id, "capture_timestamp_ns": frame.capture_timestamp_ns,
                  "image_size_wh": list(frame.image_size_wh), "model_id": self.recipe["model_id"],
                  "status": "ok", "detections": [det]}
        if self.fault == "stale":
            self.clock += 2_000_000_000
        if self.fault == "multiple":
            result["detections"].append(deepcopy(det))
        if self.fault == "no_target":
            result.update(status="no_target", detections=[])
        return result

    def execute(self, action, timeout_s):
        self.actions.append(action)
        name, params, base = action.name, action.params, self.recipe["base_motion"]
        if self.fault == "timeout" and name == "descend":
            raise TimeoutError("synthetic command acknowledgement lost")
        if self.fault == "rotate_stuck" and name == "rotate_back":
            raise TimeoutError("synthetic base did not acknowledge the return turn")

        if name == "close_gripper":
            self.held = self.fault != "empty_grasp"
        if name == "short_lift":
            self.lifted = self.held
        if name == "retreat_to_start":
            self.at_start = self.held and self.fault != "blocked_retreat"
        if name == "rotate_to_drop":
            want = float(params["yaw_rad"])
            # A short turn is the classic failure: the command returns ok but
            # odometry shows less yaw, so the drop aim is wrong.
            self.base_yaw_rad += want*(0.6 if self.fault == "yaw_short" else 1.0)
            self.at_drop = abs(self.base_yaw_rad-want) <= 1e-9
        if name == "move_to_drop":
            self.at_drop = self.at_start and self.held
        if name == "release_box":
            self.released, self.held = True, False
        if name == "verify_drop":
            self.in_bin = self.released and self.at_drop and self.fault != "bin_miss"
        if name == "rotate_back":
            self.base_yaw_rad = 0.0
            self.base_shift_m = 0.06 if self.fault == "base_drift" else 0.0
        if name == "regrip_handle":
            self.regripped = self.fault != "regrip_empty"
        if name == "push_drawer_closed":
            self.closed = self.regripped and self.fault != "close_blocked"
        if name == "release_handle":
            self.regripped = False

        evidence = dict.fromkeys(("scene_unchanged", "path_checked", "arrived", *action.required), True)
        evidence["source"] = "synthetic_mock"
        measured = {
            "holding_verified": self.held, "box_lift_verified": self.lifted,
            "payload_clear_of_drawer": self.at_start, "at_start": self.at_start,
            "above_bin": self.at_drop,
            "gripper_empty": not self.held, "box_in_bin": self.in_bin,
            "handle_grasped": self.regripped, "at_release_pose": self.regripped,
            "drawer_closed": self.closed, "grip_maintained": self.regripped,
            "yaw_measured_rad": self.base_yaw_rad, "base_shift_m": self.base_shift_m,
            "stroke_m": self.recipe["drawer_opening_m"] if self.closed else 0.0,
        }
        for fact, value in measured.items():
            if fact in action.required or fact in (
                    "yaw_measured_rad", "base_shift_m", "stroke_m"):
                evidence[fact] = value
        return Receipt("ok", action.request_id, self.now_ns(), evidence)

    def stop(self, reason):
        self.stops.append(reason)
        return {"confirmed": True, "source": "synthetic_mock"}
