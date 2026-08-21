#!/usr/bin/env python3
"""Standalone YOLOv8 ONNX smoke test -- no ROS anywhere in it.

This is the gate runner: the same file validates the runtime + model pair on
the host (x86 venv), in the riscv64 container, and in the guest, so a failure
isolates the environment rather than the code. --expect turns it into a
pass/fail check against known images:

    python3 detect.py --model yolov8n.onnx --source bus.jpg --expect person=3,bus=1

exits 0 only if at least 3 persons and 1 bus were found.
"""

import argparse
import json
import pathlib
import sys

import cv2

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yolov8_onnx import Yolov8Onnx


def parse_expect(spec):
    want = {}
    for part in spec.split(","):
        name, _, n = part.strip().partition("=")
        want[name] = int(n) if n else 1
    return want


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--source", required=True,
                    help="image file, or a directory of images")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--save-dir", help="write annotated copies here")
    ap.add_argument("--json", dest="json_out", help="write detections here")
    ap.add_argument("--expect",
                    help="minimum counts, e.g. person=3,bus=1 -- exit 1 if unmet")
    args = ap.parse_args()

    src = pathlib.Path(args.source)
    images = sorted(p for p in src.iterdir()
                    if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")) \
        if src.is_dir() else [src]
    if not images:
        sys.exit(f"no images under {src}")

    model = Yolov8Onnx(args.model, args.imgsz, args.conf, args.iou, args.threads)
    print(f"runtime: {model.runtime}  model: {args.model}  imgsz: {args.imgsz}")

    results = {}
    for path in images:
        img = cv2.imread(str(path))
        if img is None:
            print(f"{path.name}: unreadable, skipped")
            continue
        dets, dt = model(img)
        labels = " ".join(f"{d['cls_name']}:{d['conf']:.2f}" for d in dets)
        print(f"{path.name}: {len(dets)} det in {dt:.1f}s  {labels}")
        results[path.name] = dets
        if args.save_dir:
            for d in dets:
                x1, y1, x2, y2 = map(round, d["xyxy"])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"{d['cls_name']} {d['conf']:.2f}",
                            (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)
            out = pathlib.Path(args.save_dir)
            out.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out / path.name), img)

    if args.json_out:
        pathlib.Path(args.json_out).write_text(json.dumps(results, indent=1))

    if args.expect:
        want = parse_expect(args.expect)
        got = {}
        for dets in results.values():
            for d in dets:
                got[d["cls_name"]] = got.get(d["cls_name"], 0) + 1
        missing = {k: v for k, v in want.items() if got.get(k, 0) < v}
        if missing:
            sys.exit(f"EXPECT FAILED: wanted {want}, got {got}")
        print(f"EXPECT OK: {want} satisfied by {got}")


if __name__ == "__main__":
    main()
