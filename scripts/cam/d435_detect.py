#!/usr/bin/env python3
"""D435: object detection on the COLOR stream with per-object depth.

Runs YOLOv8 on the aligned color image and, for each detection, samples the
aligned depth frame at the box center (median of a small patch) to report the
distance in metres. Only the color window is shown — no depth visualization.

Usage:
    DISPLAY=:2 python3 scripts/d435_detect.py            # live window (q to quit)
    python3 scripts/d435_detect.py --headless --save out # one annotated frame, no GUI
    python3 scripts/d435_detect.py --conf 0.4 --weights /path/to/yolov8n.pt
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30
DEFAULT_WEIGHTS = "/home/oem/sy/ID_SLAM/yolov8n.pt"


def depth_at(depth_frame, cx, cy, half=4):
    """Median depth (m) over a (2*half+1)^2 patch around (cx,cy); 0 if unknown."""
    xs = range(max(0, cx - half), min(W, cx + half + 1))
    ys = range(max(0, cy - half), min(H, cy + half + 1))
    vals = [d for y in ys for x in xs
            if (d := depth_frame.get_distance(x, y)) > 0]
    return float(np.median(vals)) if vals else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--save", metavar="PREFIX")
    args = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    model = YOLO(args.weights)
    names = model.names

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipeline.start(cfg)
    dev = profile.get_device()
    print(f"D435 S/N {dev.get_info(rs.camera_info.serial_number)} | "
          f"weights {args.weights} | conf {args.conf}")

    align = rs.align(rs.stream.color)  # depth -> color frame
    try:
        for _ in range(15):
            pipeline.wait_for_frames()  # let AE settle

        while True:
            frames = align.process(pipeline.wait_for_frames())
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue
            img = np.asanyarray(color.get_data())

            res = model(img, conf=args.conf, verbose=False)[0]
            lines = []
            for box in res.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = depth_at(depth, cx, cy)
                label = names[int(box.cls[0])]
                conf = float(box.conf[0])
                dtxt = f"{dist:.2f}m" if dist > 0 else "--"
                cap = f"{label} {conf:.2f} {dtxt}"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
                ytxt = y1 - 6 if y1 > 16 else y1 + 16
                cv2.putText(img, cap, (x1, ytxt), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2, cv2.LINE_AA)
                lines.append(f"  {label:12s} conf {conf:.2f}  depth {dtxt}")

            if lines:
                print("detections:\n" + "\n".join(lines))

            if args.save:
                cv2.imwrite(f"{args.save}.png", img)
                print(f"saved {args.save}.png")
            if args.headless or args.save:
                break

            cv2.imshow("D435 detect (q to quit)", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print("stopped")


if __name__ == "__main__":
    sys.exit(main())
