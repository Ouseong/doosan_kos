#!/usr/bin/env python3
"""Color-camera-only object detection (NO depth).

Uses just the D435's RGB camera and HSV color segmentation to find colored
objects (e.g. an orange cube) and draw a box + label. The depth/IR streams,
laser projector and distance readout are intentionally not used.

Usage:
    DISPLAY=:2 python3 scripts/cam_detect.py                 # live, orange preset
    python3 scripts/cam_detect.py --color red --min-area 800
    python3 scripts/cam_detect.py --tune                     # interactive HSV sliders
    python3 scripts/cam_detect.py --headless --save out
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30

# HSV ranges (OpenCV H is 0-179). Orange wraps a bit toward red.
PRESETS = {
    "orange": [((5, 110, 110), (22, 255, 255))],
    "red":    [((0, 120, 100), (10, 255, 255)), ((170, 120, 100), (179, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 120, 60), (130, 255, 255))],
    "yellow": [((22, 110, 110), (35, 255, 255))],
}


def make_mask(hsv, ranges, cv2):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = m if mask is None else (mask | m)
    return mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="orange", choices=list(PRESETS))
    ap.add_argument("--min-area", type=int, default=600,
                    help="ignore blobs smaller than this many px")
    ap.add_argument("--no-detect", action="store_true",
                    help="just show the raw color camera, no color detection")
    ap.add_argument("--tune", action="store_true", help="interactive HSV sliders")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--save", metavar="PREFIX")
    args = ap.parse_args()

    import cv2
    ranges = PRESETS[args.color]

    if args.tune:
        cv2.namedWindow("tune")
        for name, val in (("Hlo", ranges[0][0][0]), ("Hhi", ranges[0][1][0]),
                          ("Slo", ranges[0][0][1]), ("Shi", 255),
                          ("Vlo", ranges[0][0][2]), ("Vhi", 255)):
            cv2.createTrackbar(name, "tune", val, 255, lambda x: None)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)  # color only
    profile = pipeline.start(cfg)
    dev = profile.get_device()
    print(f"D435 S/N {dev.get_info(rs.camera_info.serial_number)} | "
          f"COLOR-only | color={args.color} min_area={args.min_area}")
    kernel = np.ones((5, 5), np.uint8)

    try:
        for _ in range(15):
            pipeline.wait_for_frames()  # let auto-exposure settle

        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())

            if args.no_detect:
                if args.save:
                    cv2.imwrite(f"{args.save}.png", img)
                    print(f"saved {args.save}.png")
                if args.headless or args.save:
                    break
                cv2.imshow("Camera (raw, q to quit)", img)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                continue

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            if args.tune:
                g = lambda n: cv2.getTrackbarPos(n, "tune")
                cur = [((g("Hlo"), g("Slo"), g("Vlo")),
                        (g("Hhi"), g("Shi"), g("Vhi")))]
                mask = make_mask(hsv, cur, cv2)
            else:
                mask = make_mask(hsv, ranges, cv2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            lines = []
            for c in cnts:
                area = cv2.contourArea(c)
                if area < args.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 165, 255), 2)
                cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(img, f"{args.color}", (x, y - 6 if y > 16 else y + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)
                lines.append(f"  {args.color} area={int(area)} center=({cx},{cy})")

            if lines:
                print("detections:\n" + "\n".join(lines))

            if args.save:
                cv2.imwrite(f"{args.save}.png", img)
                print(f"saved {args.save}.png")
            if args.headless or args.save:
                break

            cv2.imshow("Cam detect (color only, q to quit)", img)
            if args.tune:
                cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                if args.tune:
                    print("final HSV:", cur)
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
