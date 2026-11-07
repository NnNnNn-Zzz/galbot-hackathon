"""Loopback-only temporary mask service for a full-top matte color cover."""
from __future__ import annotations

import argparse
from http.server import HTTPServer

from .serve import SCHEMA, Segmenter, make_handler


class ColorSegmenter(Segmenter):
    perception_backend = "bench_hsv"

    def predict(self, path, request, stamp):
        import cv2
        image = cv2.imread(str(path), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        if image is None:
            raise ValueError("Could not decode capture")
        return self.predict_image(image, request, stamp)

    def predict_image(self, image, request, stamp):
        import cv2
        import numpy as np

        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.args.hsv_low), np.array(self.args.hsv_high))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [c for c in contours if cv2.contourArea(c) >= self.args.min_area_px]
        detections = []
        if len(candidates) == 1:
            contour = candidates[0]
            poly = cv2.approxPolyDP(contour, 0.008*cv2.arcLength(contour, True), True).reshape(-1, 2)
            x, y, width, height = cv2.boundingRect(contour)
            if len(poly) >= 4 and x > 0 and y > 0 and x+width < w and y+height < h:
                detections.append({"class_name": "medicine_box_top", "confidence": 1.0,
                                   "bbox_xyxy_px": [x, y, x+width-1, y+height-1],
                                   "mask_polygon_xy_px": poly.tolist()})
        return {"schema": SCHEMA, "status": "ok" if detections else "no_target",
                "frame_id": request["frame_id"], "camera_id": request["camera_id"],
                "capture_timestamp_ns": stamp, "image_size_wh": [w, h],
                "model_id": self.args.model_id, "detections": detections,
                "diagnostics": {"source": "bench_hsv", "candidate_count": len(candidates)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--model-id", default="bench_top_mask_v1")
    parser.add_argument("--port", type=int, default=8010)
    parser.add_argument("--hsv-low", nargs=3, type=int, required=True)
    parser.add_argument("--hsv-high", nargs=3, type=int, required=True)
    parser.add_argument("--min-area-px", type=int, required=True)
    args = parser.parse_args()
    if (not args.model_id.startswith("bench_top_mask_") or args.min_area_px <= 0
            or any(not 0 <= a <= b <= limit for a,b,limit in zip(
                args.hsv_low, args.hsv_high, (179, 255, 255)))):
        parser.error("Invalid experimental model ID, area or HSV limits")
    segmenter = ColorSegmenter(None, args)
    with HTTPServer(("127.0.0.1", args.port), make_handler(segmenter)) as server:
        server.serve_forever()


if __name__ == "__main__":
    main()
