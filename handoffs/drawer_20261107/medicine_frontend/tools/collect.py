#!/usr/bin/env python3
"""药品数采客户端（腕部/头部原图采集）。

数据从 HPU 相机桥 (http://192.168.166.7:8092) 拉取，原图字节不经任何转码直接落盘，
配套保存原始 uint16 深度、时间戳、相机内参与关节状态。

先决条件:
  - HPU 相机桥已运行 (systemctl --user status galbot-camera-robot)
  - 访问码通过环境变量 GALBOT_CAM_CODE 或 --code 提供

用法示例（在 WSL 或任意 Python3 环境运行，仅用标准库）:
  python3 collect.py info                                   # 查看相机服务状态
  python3 collect.py wrist --drug ganmaoling --arrangement single --count 5
  python3 collect.py wrist --drug mix        --arrangement mixed  --count 10
  python3 collect.py head  --drug siji --count 5            # 头部持物（右手抓持后）
  python3 collect.py wrist --drug vitamin_c --arrangement single --count 1 --note "反光难例"
交互提示: Enter=拍摄, s=跳过这张, q=提前结束。
"""

from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import math
import os
import sys
import time
import re
from http.cookies import SimpleCookie
from datetime import datetime, timezone
from pathlib import Path
from urllib import request as urlreq

BASE = os.environ.get("GALBOT_CAM_BASE", "http://192.168.166.7:8092")
PROJECT = Path(__file__).resolve().parents[1]
if str(PROJECT.parent) not in sys.path:
    sys.path.insert(0, str(PROJECT.parent))
from medicine_frontend.dataset import Dataset, SOURCES
from medicine_frontend.core import parse_expiry_info
DATA_ROOT = Path(os.environ.get(
    "MEDICINE_DATA_ROOT",
    str(PROJECT / "data" / "medicine_dataset"),
))
DRUGS = {"ganmaoling": "感冒灵", "siji": "四季感冒片", "vitamin_c": "维C泡腾片", "fish_oil": "鱼油", "mix": "混合", "none": "无"}

SESSION_DIR = {
    "wrist": DATA_ROOT / "raw" / "wrist" / "session_001",
    "head": DATA_ROOT / "raw" / "head" / "session_001",
}
DEFAULT_SOURCES = {"wrist": ["wrist_right"], "head": ["head_right"]}


class Cam:
    def __init__(self, code: str | None, token: str | None):
        self.headers = {"Accept": "*/*"}
        if token:
            self.headers["X-Galbot-Camera-Token"] = token
        elif code:
            self._login(code)
        else:
            raise SystemExit("需要 --code / GALBOT_CAM_CODE / --token 之一")

    def _login(self, code: str) -> None:
        body = json.dumps({"code": code}).encode()
        req = urlreq.Request(f"{BASE}/api/session", data=body, method="POST",
                             headers={"Content-Type": "application/json"})
        resp = urlreq.urlopen(req, timeout=8)
        cookie = resp.headers.get("Set-Cookie", "")
        cookies = SimpleCookie()
        cookies.load(cookie)
        morsel = cookies.get("galbot_camera_session")
        session = morsel.value if morsel else ""
        if not session:
            raise SystemExit("登录失败：访问码错误或服务不可用")
        self.headers["Cookie"] = f"galbot_camera_session={session}"
        print("[auth] 登录成功")

    def get(self, path: str, timeout: float = 10.0):
        req = urlreq.Request(f"{BASE}{path}", headers=self.headers)
        resp = urlreq.urlopen(req, timeout=timeout)
        return resp.status, dict(resp.headers), resp.read()


def now_ns() -> int:
    return time.time_ns()


def iso_z(ts_ns: int) -> str:
    return datetime.fromtimestamp(ts_ns / 1e9, tz=timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def next_seq(folder: Path) -> int:
    highest = 0
    for path in folder.glob("*.json"):
        stem = path.stem.split("_")[0]
        if stem.isdigit():
            highest = max(highest, int(stem))
    return highest + 1


def fetch_depth(cam: Cam, source: str, rgb_captured_ns: int, max_age_ms: float = 1500.0):
    """Preserve latest depth and flag skew; never claim synchronized RGB-D."""
    try:
        _, headers, body = cam.get(f"/depth/{source}")
    except Exception as error:
        return None, {"error": str(error)}
    if headers.get("X-Frame-Error"):
        return None, {"error": headers["X-Frame-Error"]}
    width, height = int(headers["X-Depth-Width"]), int(headers["X-Depth-Height"])
    if len(body) != width * height * 2:
        return None, {"error": f"depth size mismatch {len(body)} vs {width}x{height}x2"}
    age_ms = (now_ns() - int(headers["X-Frame-Captured-Ns"])) / 1e6
    skew_ms = (int(headers["X-Frame-Captured-Ns"]) - rgb_captured_ns) / 1e6
    meta = {
        "width": width, "height": height,
        "depth_scale": float(headers["X-Depth-Scale"]),
        "pixel_format": headers.get("X-Depth-Format", "16UC1"),
        "camera_id": headers.get("X-Depth-Camera"),
        "frame_timestamp_ns": int(headers["X-Frame-Timestamp-Ns"]),
        "host_captured_ns": int(headers["X-Frame-Captured-Ns"]),
        "age_ms": round(age_ms, 1),
        "rgb_depth_skew_ms": round(skew_ms, 1),
        "units": "metres = raw / depth_scale",
    }
    if abs(skew_ms) > max_age_ms:
        meta["warning"] = f"depth skew {skew_ms:.0f}ms exceeds {max_age_ms:.0f}ms"
    return body, meta


def capture_once(cam: Cam, args, seq: int, session: str, kind: str, out_dir: Path) -> bool:
    stamp = iso_z(now_ns())
    base_name = f"{seq:04d}_{args.drug}_{args.arrangement}_{stamp}"
    shot_meta = {
        "schema": "medicine_capture/1",
        "capture_session": session,
        "seq": seq,
        "kind": kind,
        "drug": args.drug,
        "drug_cn": DRUGS.get(args.drug, args.drug),
        "arrangement": args.arrangement,
        "distance_cm": args.distance_cm,
        "lighting": args.lighting,
        "grasp_state": args.grasp_state,
        "operator_note": args.note or "",
        "captured_local": datetime.now().isoformat(timespec="milliseconds"),
        "camera_info_file": getattr(args, "camera_info_file", "camera_info.json"),
        "label_truth": {"entity_id": getattr(args, "label_id", None),
                        "expiry": getattr(args, "expiry_truth", None),
                        "source": "operator_metadata_not_ocr"},
        "sources": {},
        "requested_sources": list(args.sources),
    }
    saved_any = False
    source_metas: dict = {}
    joint = None
    if args.joint_state:
        try:
            _, _, body = cam.get("/api/joint_state", timeout=5)
            joint = json.loads(body)
        except Exception as error:
            joint = {"error": str(error)}
    if joint:
        shot_meta["joint_state"] = joint

    for source in args.sources:
        best = None
        for _ in range(4):  # 多拉几次，取最新的一帧，减小时间戳偏移
            try:
                _, headers, body = cam.get(f"/frame/{source}")
            except Exception as error:
                print(f"  [{source}] 获取失败: {error}")
                break
            if body.startswith(b"\xff\xd8") and body.endswith(b"\xff\xd9"):
                age = (now_ns() - int(headers["X-Frame-Captured-Ns"])) / 1e6
                if best is None or age < best[0]:
                    best = (age, headers, body)
                if age < 600:
                    break
            time.sleep(0.25)
        if best is None:
            print(f"  [{source}] 未取到有效 JPEG")
            continue
        age_ms, headers, body = best
        stale = age_ms > 2500
        if stale:
            print(f"  [{source}] 警告: 帧已滞后 {age_ms:.0f}ms，可能是相机卡顿，建议确认稳定后重拍")
        if age_ms < 0:
            print(f"  [{source}] 警告: 客户端与服务端时钟有偏差，不能以 age 判断帧新鲜度")
        err = headers.get("X-Frame-Error", "")
        rgb_captured = int(headers["X-Frame-Captured-Ns"])
        jpeg_path = out_dir / f"{base_name}_{source}.jpg"
        with jpeg_path.open("xb") as image_file:
            image_file.write(body)

        source_meta = {
            "file": jpeg_path.name,
            "camera_id": headers.get("X-Frame-Camera"),
            "frame_timestamp_ns": int(headers["X-Frame-Timestamp-Ns"]),
            "host_captured_ns": rgb_captured,
            "age_ms": round(age_ms, 1),
            "stale": stale,
            "jpeg_bytes": len(body),
            "jpeg_sha256": sha256(body),
            "camera_error": err or None,
        }
        if err:
            print(f"  [{source}] 相机侧告警: {err}")
        if kind == "wrist" and args.depth:
            depth_bytes, depth_meta = fetch_depth(cam, source, rgb_captured)
            if depth_bytes is not None:
                depth_path = out_dir / f"{base_name}_{source}_depth.bin"
                with depth_path.open("xb") as depth_file:
                    depth_file.write(depth_bytes)
                depth_meta["file"] = depth_path.name
                depth_meta["sha256"] = sha256(depth_bytes)
                source_meta["depth"] = depth_meta
                print(f"  [{source}] 深度已保存 ({depth_meta['width']}x{depth_meta['height']}, "
                      f"scale={depth_meta['depth_scale']}, skew={depth_meta['rgb_depth_skew_ms']}ms)")
            else:
                source_meta["depth"] = depth_meta
                print(f"  [{source}] 深度未保存: {depth_meta.get('error')}")
        source_metas[source] = source_meta
        print(f"  [{source}] {jpeg_path.name}  ({len(body)/1024:.0f} KB, age={age_ms:.0f}ms)")
        saved_any = True
    if saved_any:
        metadata_path = out_dir / f"{base_name}.json"
        with metadata_path.open("x", encoding="utf-8") as metadata_file:
            json.dump(shot_meta | {"sources": source_metas}, metadata_file, ensure_ascii=False, indent=2)
        with Dataset(getattr(args, "root", DATA_ROOT)) as dataset:
            capture_id = dataset.ingest(metadata_path)
            print(f"  数据库已登记: {capture_id}")
    return saved_any


def cmd_info(cam: Cam) -> None:
    for path in ("/health", "/api/info"):
        try:
            _, _, body = cam.get(path)
            print(f"--- {path}")
            print(json.dumps(json.loads(body), ensure_ascii=False, indent=2)[:2200])
        except Exception as error:
            print(f"{path} 失败: {error}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", nargs="?", default="info",
                        choices=["info", "wrist", "head"], help="info=查状态; wrist/head=采集")
    parser.add_argument("--drug", default="none", choices=list(DRUGS))
    parser.add_argument("--arrangement", default="single", help="摆放方式: single|mixed|custom...")
    parser.add_argument("--count", type=int, default=1, help="计划拍摄张数")
    parser.add_argument("--distance-cm", type=float, default=30.0, help="观察距离(cm)")
    parser.add_argument("--lighting", default="demo", help="光照: demo|white|dim|...")
    parser.add_argument("--grasp-state", default=None, help="头部图: held|empty|pre_grasp")
    parser.add_argument("--sources", nargs="*", default=None, help="相机源，默认按 kind 选择")
    parser.add_argument("--note", default="", help="自由备注")
    parser.add_argument("--code", default=os.environ.get("GALBOT_CAM_CODE"))
    parser.add_argument("--token", default=os.environ.get("GALBOT_CAM_TOKEN"))
    parser.add_argument("--no-depth", dest="depth", action="store_false", help="腕部不取深度")
    parser.add_argument("--no-joint", dest="joint_state", action="store_false", help="不取关节状态")
    parser.add_argument("--session", default="session_001", help="采集批次目录名")
    parser.add_argument("--root", type=Path, default=DATA_ROOT, help="数据根目录，默认跨 Windows/WSL 自动定位")
    parser.add_argument("--label-id", help="本次单件药品的实体标签编号；混合图用逐物体标注")
    parser.add_argument("--expiry-truth", help="人工记录的 EXP 真值，不作为模型识别结果")
    parser.add_argument("--yes", action="store_true", help="免交互自动拍满 count 张")
    args = parser.parse_args(argv)

    if args.count < 1 or not re.fullmatch(r"[\w-]+", args.session) or args.session in (".", ".."):
        parser.error("count 必须为正；session 只能包含字母、数字、下划线和横线")
    if not re.fullmatch(r"[\w-]+", args.arrangement):
        parser.error("arrangement 只能包含字母、数字、下划线和横线")
    if not math.isfinite(args.distance_cm) or args.distance_cm <= 0:
        parser.error("distance-cm 必须为正有限数")
    if args.expiry_truth:
        if args.drug in ("mix", "none") or not args.label_id:
            parser.error("单件日期真值需指定具体药品及 --label-id；混合图在数据库逐物体标注")
        try:
            parse_expiry_info(args.expiry_truth)
        except (ValueError, TypeError) as error:
            parser.error(str(error))
    args.root = args.root.resolve()
    if args.mode != "info":
        args.sources = args.sources or DEFAULT_SOURCES[args.mode]
        if (len(set(args.sources)) != len(args.sources) or
                any(s not in SOURCES or not s.startswith(args.mode) for s in args.sources)):
            parser.error("sources 必须是该模式对应的不同相机，例如 wrist_right/wrist_left")
    if not args.code and not args.token:
        args.code = getpass.getpass("相机访问码（不显示、不保存）：")

    cam = Cam(args.code, args.token)
    if args.mode == "info":
        cmd_info(cam)
        return 0

    kind = args.mode
    args.sources = args.sources or DEFAULT_SOURCES[kind]
    out_dir = args.root / "raw" / kind / args.session
    out_dir.mkdir(parents=True, exist_ok=True)

    info = {}
    try:
        _, _, body = cam.get("/api/info")
        info = json.loads(body)
    except Exception as error:
        print(f"[warn] /api/info 不可用: {error}")
    args.camera_info_file = f"camera_info_{time.time_ns()}.json"
    (out_dir / args.camera_info_file).write_text(
        json.dumps({"base_url": BASE, "fetched_at": datetime.now().isoformat(),
                    "info": info}, ensure_ascii=False, indent=2), encoding="utf-8")

    seq = next_seq(out_dir)
    print(f"输出目录: {out_dir}")
    print(f"起始序号: {seq:04d}  药品={args.drug}({DRUGS.get(args.drug, '?')}) 摆放={args.arrangement} "
          f"距离={args.distance_cm}cm 光照={args.lighting} 深度={'开' if kind == 'wrist' and args.depth else '关'}")
    print("每张: Enter=拍摄  s=跳过  q=结束\n")

    saved = 0
    index_path = out_dir / "session_index.jsonl"
    for i in range(args.count):
        prompt = f"[{i + 1}/{args.count}] 第 {seq:04d} 张 -> "
        if args.yes:
            action = ""
        else:
            try:
                action = input(prompt).strip().lower()
            except EOFError:
                break
        if action == "q":
            break
        if action == "s":
            print("  已跳过")
            continue
        if capture_once(cam, args, seq, args.session, kind, out_dir):
            with index_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps({"seq": seq, "drug": args.drug, "arrangement": args.arrangement,
                                     "distance_cm": args.distance_cm, "lighting": args.lighting,
                                     "kind": kind, "local_time": datetime.now().isoformat(timespec="seconds")},
                                    ensure_ascii=False) + "\n")
            saved += 1
            seq += 1
    print(f"\n完成: 本轮保存 {saved} 组采集记录（每组可能包含多个相机） -> {out_dir}")
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())
