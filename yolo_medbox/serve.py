"""Local YOLO service for the shared Agent. Images are read from a capture directory."""
from __future__ import annotations

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path

from .infer import IMG_SUFFIXES, process

SCHEMA = "drawer_box_seg/1"


class Segmenter:
    perception_backend = "yolo"

    def __init__(self, model, args):
        self.model, self.args = model, args
        self.root = Path(args.capture_dir).resolve(strict=True)
        if not self.root.is_dir():
            raise ValueError("capture-dir must be a directory")

    def identity(self):
        return {"model_id": self.args.model_id, "perception_backend": self.perception_backend,
                "weights_sha256": getattr(self.args, "weights_sha256", None)}

    def segment(self, request: dict) -> dict:
        if not isinstance(request, dict) or request.get("schema") != SCHEMA:
            raise ValueError("invalid request schema")
        if request.get("model_id") != self.args.model_id:
            raise ValueError("model_id mismatch")
        for key in ("frame_id", "camera_id", "image_ref"):
            if not isinstance(request.get(key), str) or not request[key]:
                raise ValueError(f"missing {key}")
        stamp, size = request.get("capture_timestamp_ns"), request.get("image_size_wh")
        if type(stamp) is not int or stamp <= 0:
            raise ValueError("invalid capture timestamp")
        if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
            raise ValueError("invalid capture size")
        path = Path(request["image_ref"])
        if not path.is_absolute():
            raise ValueError("capture path must be absolute")
        path = path.resolve()
        if not path.is_relative_to(self.root) or path.suffix.lower() not in IMG_SUFFIXES or not path.is_file():
            raise ValueError("capture file is outside the allowed image directory or missing")
        stat = path.stat()
        result = self.predict(path, request, stamp)
        after = path.stat()
        if result["image_size_wh"] != size or (after.st_mtime_ns, after.st_size) != (stat.st_mtime_ns, stat.st_size):
            raise ValueError("capture size mismatch or image changed during inference")
        return {**result, **self.identity()}

    def predict(self, path, request, stamp):
        return process(self.model, path, self.args, request["frame_id"], request["camera_id"],
                       capture_timestamp_ns=stamp)


def make_handler(segmenter: Segmenter):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self.reply(200, {"schema": SCHEMA, "status": "ok", **segmenter.identity()})
            else:
                self.reply(404, {"status": "error", "reason": "unknown endpoint"})

        def do_POST(self):
            if self.path != "/segment":
                self.reply(404, {"status": "error", "reason": "unknown endpoint"})
                return
            self.connection.settimeout(3)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise ValueError("invalid request length")
                payload = segmenter.segment(json.loads(self.rfile.read(length)))
            except (ValueError, OSError) as exc:
                self.reply(400, {"status": "error", "reason": str(exc)})
                return
            except Exception as exc:
                self.reply(500, {"status": "error", "reason": type(exc).__name__})
                return
            self.reply(200, payload)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--model-id", required=True)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--port", type=int, default=8010)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--max-det", type=int, default=5)
    parser.add_argument("--max-points", type=int, default=24)
    args = parser.parse_args()
    if (not args.model.is_file() or not args.model_id.strip()
            or not 2 <= args.max_det <= 100 or args.max_points not in (0, *range(4, 257))):
        parser.error("requires local weights, a model ID and valid mask limits")

    from ultralytics import YOLO
    digest = hashlib.sha256()
    with args.model.open("rb") as weights:
        for chunk in iter(lambda: weights.read(1024 * 1024), b""):
            digest.update(chunk)
    args.weights_sha256 = digest.hexdigest()
    model = YOLO(str(args.model))
    if model.task != "segment" or set(model.names.values()) != {"medicine_box_top"}:
        parser.error("expected a single-class medicine_box_top segmentation model")
    with HTTPServer(("127.0.0.1", args.port), make_handler(Segmenter(model, args))) as server:
        server.serve_forever()


if __name__ == "__main__":
    main()
