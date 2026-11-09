"""Offline analysis of a stationary capture session. No robot connection.

Reads data/drawer_stationary/<session>/capture_XX/ produced by
collect_drawer_stationary.py, verifies file hashes, runs handle + panel
geometry per capture and the cross-position stability summary.

Base poses (map frame) are not recorded by the capture tool; pass
--base-poses poses.json mapping capture folder names to SDK pose7 lists
(recorded from the console per position) to enable map-frame comparison.
Without it the summary fails closed unless --allow-base-frame is set
(single-position repeatability only).

    python -X utf8 -m medicine_frontend.tools.analyze_drawer_stationary \
        --session-dir data/drawer_stationary/drawer_stationary_20261107_A \
        --base-poses base_poses.json

Exit code 0 only when every capture analyzed and the summary passed.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from medicine_frontend.drawer_perception import decode_depth
from medicine_frontend.drawer_stationary import (analyze_frame, pose7_to_matrix,
                                                 summarize)


def _check_hash(folder, entry):
    raw = (folder / entry["file"]).read_bytes()
    if hashlib.sha256(raw).hexdigest() != entry["sha256"]:
        raise ValueError(f"hash_mismatch:{entry['file']}")
    return raw


def _intrinsic_matrix(info):
    k = np.asarray(info.get("K"), dtype=float)
    if k.shape != (9,) or not np.isfinite(k).all():
        raise ValueError("Intrinsics must carry finite K[9]")
    return k.reshape(3, 3)


def analyze_capture(folder, *, expand=4.0):
    """One capture folder -> geometry result; raises with the reason on failure."""
    folder = Path(folder)
    meta = json.loads((folder / "capture.json").read_text(encoding="utf-8"))
    rgb_raw = _check_hash(folder, meta["rgb"])
    bgr = cv2.imdecode(np.frombuffer(rgb_raw, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("undecodable_rgb")
    depth_info = meta["depth"]
    depth_raw = _check_hash(folder, depth_info)
    shape = None
    if depth_info.get("height") and depth_info.get("width"):
        shape = (int(depth_info["height"]), int(depth_info["width"]))
    depth = decode_depth(depth_raw, depth_scale=float(depth_info["depth_scale"]),
                         encoding=depth_info["format"], shape=shape)
    warnings = []
    if depth.shape != bgr.shape[:2]:
        raise ValueError(f"depth_rgb_resolution_mismatch:{depth.shape}!={bgr.shape[:2]}")
    intrinsics = meta["intrinsics"]
    k_depth = _intrinsic_matrix(intrinsics["depth"])
    k_rgb = _intrinsic_matrix(intrinsics["rgb"])
    if not np.allclose(k_depth, k_rgb, atol=1e-6):
        warnings.append("rgb_depth_intrinsics_differ:pixel_alignment_unverified")
    if intrinsics.get("depth_intrinsics_source") == "color_intrinsics_assumed_aligned":
        warnings.append("depth_intrinsics_assumed_from_color_sensor")
    skew = depth_info.get("rgb_depth_skew_ms")
    if isinstance(skew, (int, float)) and abs(skew) > 1500:
        warnings.append(f"rgb_depth_skew_ms:{skew:.0f}")
    pose = meta.get("extrinsics", {}).get("depth", {}).get("pose")
    camera_frame_only = not pose
    if camera_frame_only:
        camera_to_base = np.eye(4)
        warnings.append("extrinsics_unavailable:geometry_in_camera_frame")
    else:
        camera_to_base = pose7_to_matrix(pose)
    result = analyze_frame(bgr, depth, k_depth, camera_to_base, expand=expand)
    result["name"] = folder.name
    result["camera_frame_only"] = camera_frame_only
    result["warnings"] = warnings
    return result


def analyze_session(session_dir, *, base_poses=None, allow_base_frame=False,
                    position_tolerance_m=0.02, normal_tolerance_deg=3.0):
    session_dir = Path(session_dir)
    folders = sorted(p for p in session_dir.glob("capture_*") if p.is_dir())
    if not folders:
        raise ValueError(f"No capture folders under {session_dir}")
    base_poses = base_poses or {}
    results, errors = [], []
    for folder in folders:
        try:
            result = analyze_capture(folder)
            pose = base_poses.get(folder.name)
            if pose is not None:
                result["base_to_map"] = pose7_to_matrix(pose).tolist()
            results.append(result)
        except Exception as error:
            errors.append({"capture": folder.name,
                           "error": f"{type(error).__name__}: {error}"[:300]})
    summary = None
    blocked = None
    if len(results) >= 2:
        if any(r.get("camera_frame_only") for r in results):
            blocked = ("extrinsics_unavailable:cross_position_comparison_needs_"
                       "camera_to_base_link")
        else:
            summary = summarize(results, allow_base_frame=allow_base_frame,
                                position_tolerance_m=position_tolerance_m,
                                normal_tolerance_deg=normal_tolerance_deg)
    report = {"schema": "drawer_stationary_analysis/1",
              "session": session_dir.name,
              "captures_found": len(folders),
              "captures_analyzed": len(results),
              "blocked": blocked,
              "errors": errors,
              "summary": summary,
              "per_capture": results}
    with (session_dir / "analysis.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--base-poses", type=Path, default=None,
                        help="JSON: {capture_XX: [x,y,z,qx,qy,qz,qw]} from the console")
    parser.add_argument("--allow-base-frame", action="store_true",
                        help="Single-position repeatability only, no map comparison")
    parser.add_argument("--position-tolerance-m", type=float, default=0.02)
    parser.add_argument("--normal-tolerance-deg", type=float, default=3.0)
    args = parser.parse_args(argv)
    base_poses = None
    if args.base_poses is not None:
        base_poses = json.loads(args.base_poses.read_text(encoding="utf-8"))
    report = analyze_session(args.session_dir, base_poses=base_poses,
                             allow_base_frame=args.allow_base_frame,
                             position_tolerance_m=args.position_tolerance_m,
                             normal_tolerance_deg=args.normal_tolerance_deg)
    summary = report["summary"] or {}
    print(json.dumps({"analysis": str(args.session_dir / "analysis.json"),
                      "captures_analyzed": report["captures_analyzed"],
                      "blocked": report["blocked"],
                      "errors": report["errors"],
                      "passed": summary.get("passed"),
                      "position_spread_m": summary.get("position_spread_m"),
                      "normal_spread_deg": summary.get("normal_spread_deg"),
                      "failures": summary.get("failures")}, ensure_ascii=False),
          flush=True)
    ok = (not report["errors"] and report["captures_analyzed"] >= 2
          and summary.get("passed") is True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
