"""fk_fix.py — 用交接包的真实 (关节角, TCP) 对反验 URDF FK，定位误差根因。

零副作用：只读 URDF + 只读交接包 JSON。
"""
from __future__ import annotations

import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "galbot_one_golf.urdf")
TEACH = os.path.join(os.path.dirname(HERE), "_handoff_zip", "drawer_20261107",
                     "medicine_frontend", "data", "medicine_dataset", "teaching")


# ---------------------------------------------------------------------------
def rpy_to_mat(r, p, y):
    """URDF 约定：固定轴 roll-pitch-yaw，等价 R = Rz(y) @ Ry(p) @ Rx(r)。"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_to_mat(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_z(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def load_tree(urdf_path=URDF):
    root = ET.parse(urdf_path).getroot()
    by_child, axes = {}, {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
        lim = j.find("limit")
        lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else 0.0
        hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else 0.0
        ax = j.find("axis")
        axis = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "0 0 1").split()])
        node = dict(name=j.get("name"), type=j.get("type"),
                    parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                    xyz=xyz, rpy=rpy, lo=lo, hi=hi, axis=axis)
        by_child[node["child"]] = node
    return by_child


def chain_to(by_child, leaf_link, root_link):
    """从 leaf 沿 parent 回溯到 root，返回 [root→leaf] 顺序的关节列表。"""
    chain, cur = [], leaf_link
    while cur != root_link:
        node = by_child.get(cur)
        if node is None:
            raise KeyError(f"回溯断链：{cur}（目标根 {root_link}）")
        chain.append(node)
        cur = node["parent"]
    chain.reverse()
    return chain


def rot_axis(axis, t):
    """Rodrigues：绕任意单位轴 axis 转 t。URDF 的 joint axis 不必是 z。"""
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * (K @ K)


def fk(chain, jmap):
    """jmap: {关节名: 角度}。返回 (各关节点, 末端 p, 末端 R)。"""
    R = np.eye(3)
    p = np.zeros(3)
    pts = {}
    for j in chain:
        p = p + R @ j["xyz"]
        R = R @ rpy_to_mat(*j["rpy"])
        pts[j["name"]] = p.copy()
        if j["type"] in ("revolute", "continuous"):
            R = R @ rot_axis(j["axis"], jmap.get(j["name"], 0.0))
    return pts, p, R


# ---------------------------------------------------------------------------
def load_samples():
    """返回 [(tag, {关节名:角}, TCP(3,), 姿态R或None)]"""
    out = []

    p = os.path.join(TEACH, "drawer_final_20261107_121006", "sdk_snapshot.json")
    d = json.load(open(p, encoding="utf-8"))
    js = {j["name"]: j["position"] for j in d["reads"]["robot"]["data"]["joint_state"]}
    ee = d["reads"]["right_end_effector"]["data"]["pose"]
    out.append(("P0(front_canonical)", js, np.array(ee[:3]), quat_to_mat(*ee[3:7])))

    p = os.path.join(TEACH, "drawer_pull_20261107_122158")
    if os.path.isdir(p):
        for f in ("pre_state.json", "post_state.json"):
            fp = os.path.join(p, f)
            if os.path.exists(fp):
                dd = json.load(open(fp, encoding="utf-8"))
                print(f"  [debug] {f} keys: {list(dd.keys())}")
    return out


def main():
    by_child = load_tree()
    chain = chain_to(by_child, "right_arm_end_effector_mount_link", "base_link")
    nrev = sum(1 for j in chain if j["type"] in ("revolute", "continuous"))
    print("=" * 78)
    print(f"完整链: base_link → 末端法兰，{len(chain)} 个关节（其中 {nrev} 个转动）")
    print("=" * 78)

    for tag, js, tcp, R_meas in load_samples():
        pts, p, R = fk(chain, js)
        err = np.linalg.norm(p - tcp)
        print(f"\n[{tag}]")
        print(f"  FK   = {np.round(p,4)}")
        print(f"  实测 = {np.round(tcp,4)}")
        print(f"  误差 = {err*1000:.1f} mm")
        if R_meas is not None:
            ang = math.degrees(math.acos(max(-1, min(1, (np.trace(R.T @ R_meas) - 1) / 2))))
            print(f"  姿态误差 = {ang:.2f}°")
        print("  关节点（base_link）:")
        for name in ("omni_chassis_leg_mount_joint", "leg_end_effector_mount_joint",
                     "torso_right_arm_mount_joint", "right_arm_joint1", "right_arm_joint3",
                     "right_arm_joint4", "right_arm_joint5", "right_arm_joint6",
                     "right_arm_end_effector_mount_joint"):
            if name in pts:
                print(f"    {name:34s} {np.round(pts[name],4)}")


if __name__ == "__main__":
    main()
