"""SDK 1.10 right-arm transport; caller owns the shared initialized SDK objects.

Official reference: https://developer.galbot.com/docs/SDK/1.10.0/g1/zh/api_python_reference/
No controller switching, SDK lifecycle changes, or implicit robot connection.
Planner target construction is supplied by the installed SDK binding adapter
because the published Python reference does not document PoseState constructors.
"""
import threading
import time
import numpy as np


class RightArmSDK:
    def __init__(self, motion, robot, sdk, *, guard, target_factory):
        self.motion, self.robot, self.sdk = motion, robot, sdk
        self.guard, self.target_factory = guard, target_factory
        self._lock = threading.Lock()
        self._fault = False

    def preview(self, poses, *, start_state=None, straight=True):
        """Read-only trajectory generation. No returned flag implies scene validation."""
        if not poses:
            raise ValueError("Empty waypoint list")
        config = self.sdk.PlannerConfig()
        config.is_direct_execute = False
        config.is_blocking = True
        config.is_check_collision = True
        config.enable_env_collision_check = True
        config.is_relative_pose = False
        config.is_tool_pose = False
        config.reference_frame = "base_link"
        config.actuate_type = self.sdk.ActuateType.with_chain_only
        waypoints = []
        for pose in poses:
            target = self.target_factory(pose)
            if target.chain_name != "right_arm" or target.cart.assist_chains:
                raise ValueError("Planner target must use right_arm without assist chains")
            waypoints.append([target])
        method = self.motion.move_line if straight else self.motion.motion_plan
        status, trajectory = method(waypoints, config, start_state)
        if status != self.sdk.MotionStatus.SUCCESS:
            raise ValueError("SDK trajectory planning failed:" + str(status))
        # Other chains in returned output are rejected, even if constant.
        if set(trajectory) != {"right_arm"}:
            raise ValueError("Planner returned other chains")
        q = np.asarray(trajectory["right_arm"], float)
        if q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or not np.isfinite(q).all():
            raise ValueError("Invalid right-arm trajectory")
        return trajectory

    def move(self, pose_xyz_xyzw, *, straight=True, timeout_s=15):
        """Blocking SDK call on worker, monitored locally at 50 Hz.

        guard() must check fresh body/base/TCP/force/control observations and return
        exactly True. A failed/timeout operation latches this adapter until replaced
        by the session owner, preventing queued commands after uncertain stopping.
        """
        p = np.asarray(pose_xyz_xyzw, float)
        if (p.shape != (7,) or not np.isfinite(p).all()
                or not np.isclose(np.linalg.norm(p[3:]), 1, atol=1e-4)
                or not np.isfinite(timeout_s) or not 0 < timeout_s <= 30):
            raise ValueError("Invalid pose or timeout")
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("Right-arm transport busy")
        try:
            if self._fault or self.guard() is not True:
                raise RuntimeError("Right-arm guard not ready")
            params = self.sdk.Parameter()
            params.set_actuate("with_chain_only")
            params.set_move_line(straight)
            params.set_check_collision(True)
            params.set_enable_env_collision_check(True)
            params.set_reference_frame("base_link")
            params.set_timeout(timeout_s)
            # Match the EndEffector frame used by saved teaching, not a guessed tool tip.
            params.set_tool_pose(False)
            done = threading.Event()
            output = {}

            def worker():
                try:
                    output["status"] = self.motion.set_end_effector_pose(
                        p.tolist(), "right_arm", "base_link",
                        enable_collision_check=True, is_blocking=True,
                        timeout=timeout_s, params=params)
                except Exception as error:
                    output["error"] = str(error)
                finally:
                    done.set()

            threading.Thread(target=worker, daemon=True).start()
            deadline = time.monotonic()+timeout_s
            try:
                while not done.wait(.02):
                    if time.monotonic() >= deadline or self.guard() is not True:
                        raise RuntimeError("Motion timeout or live guard failure")
                if (output.get("status") != self.sdk.MotionStatus.SUCCESS
                        or self.guard() is not True):
                    raise RuntimeError("SDK completion/observation failed:" + str(output))
            except Exception:
                self._fault = True
                self.robot.stop_controller("right_arm")
                raise
            return {"success": True, "completion": "completed"}
        finally:
            self._lock.release()

    def stop(self):
        self._fault = True
        status = self.robot.stop_controller("right_arm")
        # Request acceptance is not physical standstill confirmation.
        return {"accepted": status == self.sdk.ControlStatus.SUCCESS,
                "sdk_status": str(status), "completion": "unverified"}
