"""药盒顶面分割推理，输出 drawer_box_seg/1。

关键约束：输出的掩膜多边形必须是**原始图像像素坐标**。这里显式按
results.orig_shape 做一次坐标校验，不依赖框架内部缩放行为。

用法：
    python yolo_medbox/infer.py --model runs/drawer_box_seg/xxx/weights/best.pt --source <图片或目录>
    python yolo_medbox/infer.py --model ... --source ... --save-overlay --save-json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

IMG_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def simplify_polygon(pts: list[list[float]], max_points: int, eps_ratio: float = 0.004) -> list[list[float]]:
    """把逐像素阶梯的掩膜轮廓压成少量顶点。

    掩膜转多边形后默认是每像素级阶梯，48 个点里大量 1px 抖动，直接给机器人侧
    会让顶面平面拟合不稳。这里先做 Douglas-Peucker 简化，再按上限抽稀。
    """
    if len(pts) < 4:
        return pts
    import numpy as np
    arr = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    try:
        import cv2
        peri = cv2.arcLength(arr, True)
        eps = max(1.0, peri * eps_ratio)
        approx = cv2.approxPolyDP(arr, eps, True).reshape(-1, 2)
        out = [[float(x), float(y)] for x, y in approx]
        if len(out) >= 3:
            pts = out
    except Exception:  # noqa: BLE001
        pass
    if max_points and len(pts) > max_points:
        step = len(pts) / max_points
        pts = [pts[int(i * step)] for i in range(max_points)]
    return pts


def process(model, img_path: Path, args, frame_id: str | None, camera_id: str,
            *, capture_timestamp_ns: int = 0) -> dict:
    from PIL import Image
    t0 = time.perf_counter()

    with Image.open(img_path) as im:
        w, h = im.size

    result = model.predict(
        source=str(img_path),
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        device=args.device,
        verbose=False,
        retina_masks=True,
    )[0]
    if tuple(result.orig_shape) != (h, w):
        raise ValueError("YOLO original image size differs from capture size")

    detections = []
    if result.masks is not None and result.boxes is not None:
        polygons = result.masks.xy
        boxes = result.boxes
        for i in range(len(boxes)):
            poly = [[float(x), float(y)] for x, y in polygons[i]]
            if len(poly) < 3:
                continue
            if args.max_points >= 0:
                poly = simplify_polygon(poly, args.max_points)
            cls_id = int(boxes.cls[i].item())
            xyxy = [float(v) for v in boxes.xyxy[i].tolist()]
            detections.append({
                "class_name": model.names.get(cls_id, str(cls_id)),
                "confidence": round(float(boxes.conf[i].item()), 4),
                "bbox_xyxy_px": [round(v, 1) for v in xyxy],
                "mask_polygon_xy_px": [[round(x, 1), round(y, 1)] for x, y in poly],
                "mask_point_count": len(poly),
            })

    detections.sort(key=lambda d: d["confidence"], reverse=True)
    detections = detections[: args.max_det]
    elapsed_ms = (time.perf_counter() - t0) * 1000

    # 坐标越界自检：任何点落在原图外都说明缩放回映射出了问题
    oob = 0
    for d in detections:
        for x, y in d["mask_polygon_xy_px"]:
            if x < -1 or y < -1 or x > w + 1 or y > h + 1:
                oob += 1

    return {
        "schema": "drawer_box_seg/1",
        "frame_id": frame_id or img_path.stem,
        "camera_id": camera_id,
        "capture_timestamp_ns": capture_timestamp_ns,
        "image_size_wh": [w, h],
        "model_id": args.model_id,
        "status": "ok" if detections else "no_target",
        "detections": detections,
        "diagnostics": {
            "infer_ms": round(elapsed_ms, 1),
            "imgsz": args.imgsz,
            "conf_threshold": args.conf,
            "points_out_of_bounds": oob,
            "source_file": str(img_path),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--source", required=True, help="单张图片或目录")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--device", default="0")
    ap.add_argument("--max-det", type=int, default=5)
    ap.add_argument("--max-points", type=int, default=24,
                    help="每条掩膜顶点上限；先按轮廓简化，再按此值抽稀。0 表示只简化不抽稀")
    ap.add_argument("--camera-id", default="unknown")
    ap.add_argument("--model-id", default=None)
    ap.add_argument("--save-json", action="store_true")
    ap.add_argument("--save-overlay", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    model_path = Path(args.model).resolve()
    if not model_path.is_file():
        print(f"[FATAL] 权重不存在: {model_path}")
        return 2
    src = Path(args.source).resolve()
    if not src.exists():
        print(f"[FATAL] 输入不存在: {src}")
        return 2
    args.model_id = args.model_id or model_path.parent.parent.name or "box_top_seg_v1"

    images = [src] if src.is_file() else sorted(
        p for p in src.iterdir() if p.is_file() and p.suffix.lower() in IMG_SUFFIXES)
    if not images:
        print(f"[FATAL] {src} 下没有图像")
        return 2

    out_dir = Path(args.out) if args.out else (model_path.parent.parent / "infer_out")
    out_dir.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO
    model = YOLO(str(model_path))

    # CUDA 首次推理要初始化 context，不预热的话第一帧耗时会污染均值
    model.predict(source=str(images[0]), imgsz=args.imgsz, device=args.device,
                  verbose=False)

    records = []
    n_ok = 0
    for img in images:
        rec = process(model, img, args, None, args.camera_id)
        records.append(rec)
        if rec["status"] == "ok":
            n_ok += 1
        flag = "OK " if rec["status"] == "ok" else "-- "
        top = rec["detections"][0] if rec["detections"] else None
        conf = f"{top['confidence']:.3f}" if top else "  -  "
        print(f"{flag}{img.name:48s} {conf}  {rec['diagnostics']['infer_ms']:6.1f} ms  "
              f"det={len(rec['detections'])}")

        if args.save_json:
            (out_dir / f"{img.stem}.drawer_box_seg.json").write_text(
                json.dumps(rec, ensure_ascii=False, indent=2), encoding="utf-8")

        if args.save_overlay:
            from PIL import Image, ImageDraw
            with Image.open(img) as im:
                canvas = im.convert("RGB")
            draw = ImageDraw.Draw(canvas, "RGBA")
            lw = max(2, canvas.width // 320)
            for d in rec["detections"]:
                pts = [(x, y) for x, y in d["mask_polygon_xy_px"]]
                if len(pts) >= 3:
                    draw.polygon(pts, outline=(255, 40, 40, 255))
                    draw.line(pts + [pts[0]], fill=(255, 40, 40, 255), width=lw)
                x1, y1, x2, y2 = d["bbox_xyxy_px"]
                draw.rectangle([x1, y1, x2, y2], outline=(40, 200, 255, 255), width=max(1, lw // 2))
                draw.text((x1 + 4, max(0, y1 - 16)), f"{d['class_name']} {d['confidence']:.2f}",
                          fill=(255, 255, 0, 255))
            canvas.save(out_dir / f"{img.stem}.overlay.jpg", quality=88)

    summary = {
        "schema": "drawer_box_seg/1",
        "batch": {
            "model": str(model_path),
            "model_id": args.model_id,
            "source": str(src),
            "frames": len(images),
            "frames_with_target": n_ok,
            "frames_no_target": len(images) - n_ok,
            "imgsz": args.imgsz,
            "conf": args.conf,
            "out_of_bounds_total": sum(r["diagnostics"]["points_out_of_bounds"] for r in records),
            "mean_infer_ms": round(
                sum(r["diagnostics"]["infer_ms"] for r in records) / max(1, len(records)), 1),
        },
        "frames": records,
    }
    (out_dir / "batch_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\n{len(images)} 帧，检出 {n_ok}，无目标 {len(images) - n_ok}")
    print(f"平均推理 {summary['batch']['mean_infer_ms']} ms（含图像读写与掩膜还原）")
    if summary["batch"]["out_of_bounds_total"]:
        print(f"[WARN] 有 {summary['batch']['out_of_bounds_total']} 个点越出原图，检查坐标映射")
    print(f"输出目录: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
