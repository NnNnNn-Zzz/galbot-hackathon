#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读探测腰部状态：leg 组 5 个关节角 + head_base_link 在 base_link 下的位姿。

不发任何运动指令。
"""
from __future__ import annotations

import math
import sys


def quat_to_rpy_deg(q):
    qx, qy, qz, qw = (float(v) for v in q[:4])
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-9:
        return None
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def main() -> int:
    import galbot_sdk.g1 as gm

    motion = gm.GalbotMotion()
    if not motion.init():
        raise RuntimeError("GalbotMotion.init() 返回 False")
    robot = gm.GalbotRobot()
    if not robot.init(set()):
        raise RuntimeError("GalbotRobot.init() 返回 False")

    try:
        states = list(robot.get_joint_states(["leg"], []))
        print("")
        print("  %-14s %12s %12s" % ("joint", "rad", "deg"))
        print("  " + "-" * 40)
        for st in states:
            print("  %-14s %12.5f %12.3f" % (st.joint_name, st.position, math.degrees(st.position)))

        pose, ts = robot.get_transform("base_link", "head_base_link", timestamp_ns=0, timeout_ms=500)
        xyz = [float(v) for v in pose[:3]]
        rpy = quat_to_rpy_deg(pose[3:7])
        print("")
        print("  head_base_link（相对 base_link）")
        print("    xyz = (%.4f, %.4f, %.4f) m" % tuple(xyz))
        if rpy:
            print("    rpy = (%.3f, %.3f, %.3f)°" % tuple(rpy))
        print("    腰高 Z = %.4f m" % xyz[2])
        return 0
    finally:
        for name in ("request_shutdown", "wait_for_shutdown", "destroy"):
            f = getattr(robot, name, None)
            if callable(f):
                try:
                    f()
                except Exception:  # noqa: BLE001
                    pass


if __name__ == "__main__":
    sys.exit(main())
